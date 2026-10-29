"""验证选点测距的同帧绑定、局部质量与模型反馈。"""

from dataclasses import replace
from types import SimpleNamespace
import json
from unittest.mock import Mock

import numpy as np
import pytest

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.agent_loop import agent_loop
from drone_harness.tools.registry import get_tool_definition
from drone_harness.vision.target_distance import measure_target_distance
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at
from test_observation_buffer import observation_config
from test_stage3_observe import fresh_snapshot, tool_call


@pytest.mark.parametrize('x,y,pixel', [(0, 0, (0, 0)), (1, 1, (15, 11)), (.5, .5, (8, 6))])
def test_ray_distance_uses_selected_surface_and_clips_patch(x, y, pixel):
    """边缘邻域正确裁剪，斜射线保持原始距离而非前方投影。"""
    result = measure_target_distance(snapshot_at(8), observation_config(), x, y)
    assert result['valid'] and result['distance_m'] == 8
    assert result['pixel'] == dict(zip(('x', 'y'), pixel))
    assert result['valid_count'] == result['sample_count']
    assert result['distance_reference'] == 'camera_to_visible_surface_ray'


@pytest.mark.parametrize('x,y', [(-.1, .5), (1.1, .5), (True, .5), ('0.5', .5), (None, 0), (np.nan, .5)])
def test_invalid_coordinates_are_rejected(x, y):
    """选点越界、布尔和非有限坐标不能被静默裁剪。"""
    result = measure_target_distance(snapshot_at(), observation_config(), x, y)
    assert not result['success'] and not result['valid'] and result['distance_m'] is None


@pytest.mark.parametrize('value', [np.nan, np.inf, 0, -1, 65504, 100000])
def test_invalid_or_saturated_depth_is_unknown(value):
    """缺测与饱和值不能当作目标超范围的证据。"""
    result = measure_target_distance(snapshot_at(value), observation_config(), .5, .5)
    assert result['success'] and not result['valid'] and result['distance_m'] is None
    assert result['reason'] == 'INSUFFICIENT_VALID_DEPTH'
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('limit,value', [(20, 20), (20, 25), (20, 100), (10, 12), (30, 30), (30, 35)])
def test_reliable_far_surface_reports_range_limit_without_exact_distance(limit, value):
    """一致远距单独提示当前上限，不暴露范围外精确距离。"""
    config = replace(observation_config(), depth_max_m=limit)
    result = measure_target_distance(snapshot_at(value), config, .5, .5)
    assert result['success'] and not result['valid'] and result['distance_m'] is None
    assert result['reason'] == 'OUT_OF_RANGE' and result['max_range_m'] == limit
    assert f'{limit} 米测距上限' in result['message']
    assert '若选点属于参照物' in result['message'] and '前进安全门' in result['message']
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('center', [np.nan, 0, 6, 65504, 100000, 40])
def test_far_neighbors_cannot_override_invalid_near_or_inconsistent_center(center):
    """周围远距不能掩盖中心缺测、近物、饱和或明显分歧。"""
    source = snapshot_at(25)
    source.depth[6, 8] = center
    result = measure_target_distance(source, observation_config(), .5, .5)
    assert not result['valid'] and result['distance_m'] is None
    assert result['reason'] != 'OUT_OF_RANGE'


@pytest.mark.parametrize('kind', ['near', 'mixed_far', 'missing'])
def test_unreliable_far_patch_keeps_distance_unknown(kind):
    """近远混合、远景分歧或有效样本不足时不能宣布超范围。"""
    source = snapshot_at(25)
    if kind == 'near':
        source.depth[4, 6] = 8
    elif kind == 'mixed_far':
        source.depth[4:6, 6:11] = 60
    else:
        source.depth[4:6, 6:11] = np.nan
    result = measure_target_distance(source, observation_config(), .5, .5)
    assert not result['valid'] and result['reason'] == 'INSUFFICIENT_VALID_DEPTH'


@pytest.mark.parametrize('missing_count,reason', [(5, 'OUT_OF_RANGE'), (6, 'INSUFFICIENT_VALID_DEPTH')])
def test_range_limit_needs_eighty_percent_samples(missing_count, reason):
    """超范围判断沿用八成样本门槛，中心仍必须可用。"""
    source = snapshot_at(25)
    patch = source.depth[4:9, 6:11]
    for index in range(missing_count):
        patch[index // 5, index % 5] = np.nan
    result = measure_target_distance(source, observation_config(), .5, .5)
    assert result['reason'] == reason and not result['valid']


def test_local_measurement_filters_holes_without_requiring_clear_corridor():
    """局部表面可测时，远处无效像素不影响目标测距。"""
    source = snapshot_at(8)
    source.depth[:] = np.nan
    source.depth[4:9, 6:11] = 8
    source.depth[4, 6] = np.nan
    result = measure_target_distance(source, observation_config(), .5, .5)
    assert result['valid'] and result['distance_m'] == 8 and result['valid_count'] == 24


@pytest.mark.parametrize('center_only', [False, True])
def test_foreground_background_mixture_is_not_reported_as_target(center_only):
    """邻域混入背景或中心落在另一个表面时拒绝给出中位数假确定值。"""
    source = snapshot_at(8)
    source.depth[6, 8] = 15
    if not center_only:
        source.depth[4:9, 9:11] = 15
    result = measure_target_distance(source, observation_config(), .5, .5)
    assert not result['valid'] and result['reason'] == 'MIXED_DEPTH'
    assert result['distance_m'] is None


@pytest.mark.parametrize('changes,reason', [
    ({'depth': None}, 'DEPTH_OR_INTRINSICS_MISSING'),
    ({'depth_frame_id': 'other'}, 'RGB_DEPTH_CAMERA_MISMATCH'),
    ({'depth_encoding': '16UC1'}, 'DEPTH_ENCODING_UNSUPPORTED'),
    ({'depth_stamp_ns': 1}, 'DEPTH_UNSYNCED'),
    ({'depth_error': 'DEPTH_MISSING'}, 'DEPTH_MISSING'),
])
def test_mismatched_or_missing_depth_cannot_be_measured(changes, reason):
    """同帧来源、格式与时间不匹配时不使用深度。"""
    result = measure_target_distance(replace(snapshot_at(), **changes), observation_config(), .5, .5)
    assert not result['valid'] and result['reason'] == reason


def test_measurement_never_captures_or_moves_and_requires_observation(tmp_path):
    """测距只读现有帧，未观察时让模型重新观察。"""
    context = context_for(tmp_path, Mock())
    handler = get_tool_definition('measure_distance').handler
    result = handler(context, {'target': '目标实体表面', 'x': .5, 'y': .5})
    assert result['success'] and not result['valid'] and result['reason'] == 'OBSERVATION_REQUIRED'
    context.observation = snapshot_at()
    result = handler(context, {'target': '目标实体表面', 'x': .5, 'y': .5})
    assert result['valid'] and result['observation_id'] == context.observation.observation_id
    assert context.controller.mock_calls == []


@pytest.mark.parametrize('distance,reason', [(4, 'OK'), (25, 'OK'), (29.9, 'OK'), (30, 'OUT_OF_RANGE'), (35, 'OUT_OF_RANGE')])
def test_measurement_reaches_next_vlm_and_motion_invalidates_source(tmp_path, monkeypatch, distance, reason):
    """距离与超范围提示均交给下一轮，动作后仍要求重新观察。"""
    wait = Mock(side_effect=lambda *, after_stamp_ns, view="front": fresh_snapshot(after_stamp_ns, distance))
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait))
    context.profile = replace(context.profile, post_motion_wait_enabled=False)
    original_dispatch = loop_module.dispatch_tool_call

    def dispatch(current, call):
        """模拟转向，观察和测距保留真实工具处理。"""
        return {'success': True, 'degrees': 90} if call.function.name == 'rotate' else original_dispatch(current, call)

    monkeypatch.setattr(loop_module, 'dispatch_tool_call', dispatch)
    calls = [('observe', {'prompt': '检查墙面'}), ('measure_distance', {'target': '目标实体表面', 'x': .5, 'y': .5}),
             ('rotate', {'direction': 'right', 'degrees': 90}), ('measure_distance', {'target': '目标实体表面', 'x': .5, 'y': .5})]
    client = FakeClient([SimpleNamespace(content='', tool_calls=[tool_call(name, args, str(index))])
                         for index, (name, args) in enumerate(calls)] +
                        [SimpleNamespace(content='需要重新观察', tool_calls=[])])
    agent_loop(client, 'test', [{'role': 'user', 'content': '靠近墙面'}], context)
    feedback = json.loads(client.requests[2][-1]['content'])
    assert feedback['reason'] == reason
    assert feedback['distance_m'] == (pytest.approx(distance) if reason == 'OK' else None)
    assert 'max_range_m' not in feedback and feedback['valid'] is (reason == 'OK')
    raw_events = [json.loads(line) for line in
                  (tmp_path / 'logs/session_test/tool_calls.jsonl').read_text().splitlines()]
    measured = next(event['result'] for event in raw_events if event['tool_name'] == 'measure_distance')
    assert measured['max_range_m'] == 30 and measured['reason'] == feedback['reason']
    if reason == 'OUT_OF_RANGE':
        assert '30 米测距上限' in feedback['message']
    assert json.loads(client.requests[4][-1]['content'])['reason'] == 'OBSERVATION_REQUIRED'
    wait.assert_called_once()
