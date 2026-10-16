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


@pytest.mark.parametrize('value', [np.nan, np.inf, 0, -1, 20, 65504])
def test_invalid_or_out_of_range_depth_is_unknown(value):
    """无效值与量程外深度不能返回为目标距离。"""
    result = measure_target_distance(snapshot_at(value), observation_config(), .5, .5)
    assert result['success'] and not result['valid'] and result['distance_m'] is None
    assert result['reason'] == 'INSUFFICIENT_VALID_DEPTH'
    json.dumps(result, allow_nan=False)


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
    result = handler(context, {'x': .5, 'y': .5})
    assert result['success'] and not result['valid'] and result['reason'] == 'OBSERVATION_REQUIRED'
    context.observation = snapshot_at()
    result = handler(context, {'x': .5, 'y': .5})
    assert result['valid'] and result['observation_id'] == context.observation.observation_id
    assert context.controller.mock_calls == []


def test_measurement_reaches_next_vlm_and_motion_invalidates_source(tmp_path, monkeypatch):
    """测量在下一轮可见，转向后再次测量必须先获取新图。"""
    wait = Mock(side_effect=lambda *, after_stamp_ns, view="front": fresh_snapshot(after_stamp_ns))
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait))
    context.profile = replace(context.profile, post_motion_wait_enabled=False)
    original_dispatch = loop_module.dispatch_tool_call

    def dispatch(current, call):
        """模拟转向，观察和测距保留真实工具处理。"""
        return {'success': True, 'degrees': 90} if call.function.name == 'rotate' else original_dispatch(current, call)

    monkeypatch.setattr(loop_module, 'dispatch_tool_call', dispatch)
    calls = [('observe', {'prompt': '检查墙面'}), ('measure_distance', {'x': .5, 'y': .5}),
             ('rotate', {'direction': 'right', 'degrees': 90}), ('measure_distance', {'x': .5, 'y': .5})]
    client = FakeClient([SimpleNamespace(content='', tool_calls=[tool_call(name, args, str(index))])
                         for index, (name, args) in enumerate(calls)] +
                        [SimpleNamespace(content='需要重新观察', tool_calls=[])])
    agent_loop(client, 'test', [{'role': 'user', 'content': '靠近墙面'}], context)
    assert json.loads(client.requests[2][-1]['content'])['distance_m'] == 4
    assert json.loads(client.requests[4][-1]['content'])['reason'] == 'OBSERVATION_REQUIRED'
    wait.assert_called_once()
