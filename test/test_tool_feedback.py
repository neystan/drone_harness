"""验证模型反馈精简不丢失有效性、运动事实和失败原因。"""

from copy import deepcopy
from dataclasses import replace
import json
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from drone_harness.runtime.agent_loop import _build_tool_message, agent_loop, append_observation
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at
from test_stage3_observe import fresh_snapshot, tool_call


@pytest.mark.parametrize('distance,valid,reason', [
    (18.6, True, 'OK'), (30, False, 'OUT_OF_RANGE'),
    (np.nan, False, 'INSUFFICIENT_VALID_DEPTH'),
])
def test_measurement_validity_and_unknown_distance_reach_same_model(tmp_path, distance, valid, reason):
    """测距调用成功不等于有效，未知值和原提示必须进入原请求。"""
    context=context_for(tmp_path)
    messages=[{'role':'user','content':'测量指定箱面'}]
    append_observation(context,messages,snapshot_at(distance))
    client=FakeClient([
        SimpleNamespace(content='',tool_calls=[tool_call('measure_distance',{'target':'指定箱面','x':.5,'y':.5})]),
        SimpleNamespace(content='继续判断',tool_calls=[]),
    ])
    agent_loop(client,'test',messages,context)
    visible=json.loads(client.requests[1][-1]['content'])
    assert visible['success'] and visible['valid'] is valid and visible['reason']==reason
    assert (visible['distance_m'] is not None) is valid
    assert len(client.requests)==2
    assert set(visible)=={'success','valid','distance_m','reason','message'}
    raw=json.loads((tmp_path/'logs/session_test/tool_calls.jsonl').read_text().splitlines()[0])['result']
    assert visible['message']==raw['message'] and raw['sample_count']==25
    assert 'pixel' in raw and raw['target']=='指定箱面'


@pytest.mark.parametrize('blocked',[False,True])
def test_forward_distinguishes_clamped_motion_and_zero_command(tmp_path,monkeypatch,blocked):
    """缩短请求仍报告下发与最终位置，深度无效明确未移动且不造位置。"""
    def capture(*,after_stamp_ns,view='front'):
        """返回正常通道或同步失败的当前深度。"""
        source=fresh_snapshot(after_stamp_ns)
        return replace(source,depth_error='DEPTH_UNSYNCED') if blocked else source

    controller=SimpleNamespace(wait_for_observation=capture,flight_state=lambda:'IN_AIR',
        vehicle_status=SimpleNamespace(mode='OFFBOARD',connected=True,armed=True))
    context=context_for(tmp_path,controller)
    context.profile=replace(context.profile,post_motion_wait_enabled=False)
    moved=Mock(return_value={'success':True,'message':'原始动作说明',
        'target_position_ned':[3,0,-2],'final_position_ned':[2.98,0,-2]})
    monkeypatch.setattr(flight,'move',moved)
    client=FakeClient([
        SimpleNamespace(content='',tool_calls=[tool_call('forward',{'distance_m':12})]),
        SimpleNamespace(content='继续观察',tool_calls=[]),
    ])
    agent_loop(client,'test',[{'role':'user','content':'沿通道前进'}],context)
    visible=json.loads(client.requests[1][-1]['content'])
    assert visible['success'] and visible['requested_distance_m']==12
    assert visible['motion_executed'] is (not blocked)
    assert not {'front_clearance_m','front_obstacle_m','forward_max_m','clamped','target_position_ned',
                'body_offset_frd','observation_note'} & visible.keys()
    if blocked:
        moved.assert_not_called()
        assert visible['commanded_distance_m']==0 and visible['depth_reason']=='DEPTH_UNSYNCED'
        assert '未移动' in visible['message'] and 'final_position_ned' not in visible
        assert 'observation_current' not in visible
    else:
        moved.assert_called_once()
        assert 0<visible['commanded_distance_m']<12
        assert visible['final_position_ned']==[2.98,0,-2]
        assert visible['observation_current'] is False and '原始动作说明' in visible['message']
    raw=json.loads((tmp_path/'logs/session_test/tool_calls.jsonl').read_text().splitlines()[0])['result']
    assert 'front_clearance_m' in raw and visible['message']==raw['message']


@pytest.mark.parametrize('tool_name,fields',[
    ('takeoff',{'target_height':3,'final_position_ned':[0,0,-3]}),
    ('up',{'body_offset_frd':[0,0,-2],'final_position_ned':[0,0,-5]}),
    ('down',{'body_offset_frd':[0,0,2],'final_position_ned':[0,0,-3]}),
    ('rotate',{'direction':'right','degrees':20,'target_yaw_rad':.35,'final_position_ned':[0,0,-3]}),
    ('land',{'final_position_ned':[0,0,0]}),
])
def test_flight_feedback_keeps_action_facts_without_mutating_result(tool_name,fields):
    """保留各动作的高度、位移或朝向依据，只在模型副本去掉内部目标点。"""
    result={'success':True,'message':'动作原文','observation_current':False,
        'observation_note':'重复观察说明','observation_id':'private',
        'target_position_ned':[10,20,-30],**fields}
    before=deepcopy(result)
    message=_build_tool_message('native',result,tool_name)
    assert message['tool_call_id']=='native'
    assert json.loads(message['content'])=={'success':True,'message':'动作原文',
        'observation_current':False,**fields}
    assert result==before


@pytest.mark.parametrize('success,error',[(False,'MOVE_TIMEOUT'),(True,'NAVIGATION_FINISHING')])
def test_failed_or_skipped_action_keeps_complete_visible_feedback(success,error):
    """实际超时和未执行的收尾拒绝都保留错误、状态、位置与限额。"""
    result={'success':success,'error':error,'message':'原始失败原因','safety_state':'HOLD_CONFIRMED',
        'ack_received':False,'final_position_ned':[1,2,-3],'target_position_ned':[1,2,-4],
        'max_distance_m':10,'motion_executed':False,'observation_id':'private'}
    before=deepcopy(result)
    visible=json.loads(_build_tool_message('native',result,'down')['content'])
    assert visible=={k:v for k,v in before.items() if k!='observation_id'}
    assert result==before


@pytest.mark.parametrize('tool_name',['get_state','update_navigation_plan','activate_skill'])
def test_unfiltered_tools_keep_existing_payload_and_hide_internal_metadata(tool_name):
    """状态、清单和搜索保留当前反馈，包括skill正文和开始搜索所需参数。"""
    result={'success':True,'message':'原文','prompt':'寻找红门','body':'skill正文',
        'current_subgoal_id':'g2','position_ned_m':{'north':1,'east':2,'down':-3},
        'observation_id':'private','compact_history':True}
    before=deepcopy(result)
    visible=json.loads(_build_tool_message('native',result,tool_name)['content'])
    assert visible=={k:v for k,v in before.items() if k not in {'observation_id','compact_history'}}
    assert result==before
