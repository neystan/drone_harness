"""验证前下视角选择、时间配对、图像身份与上下文替换。"""

import base64
from dataclasses import replace
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from drone_harness.runtime.agent_loop import agent_loop
from drone_harness.runtime.observation import ObservationBuffer, ImageSample, build_observation_message
from drone_harness.runtime.navigation import compact_navigation_history
from drone_harness.tools.registry import get_tool_definition
from drone_harness.vision.depth_rules import compute_depth_rules
from drone_harness.vision.target_distance import measure_target_distance
from test_agent_observation_loop import FakeClient, context_for
from test_observation_buffer import add_pair, observation_config
from test_stage3_observe import fresh_snapshot, tool_call


def sample_down(stamp):
    """构造与前视容易区分的下视样本。"""
    return ImageSample(np.full((12, 16, 3), 180, dtype=np.uint8), stamp,
                       time.monotonic_ns(), 'bgr8', 'PX4/CameraDown_optical')


def view_snapshot(after_stamp_ns, view):
    """构造一次请求之后采集的所选视角快照。"""
    source = fresh_snapshot(after_stamp_ns)
    down = sample_down(source.rgb_stamp_ns)
    if view == 'down':
        return replace(source, view=view, rgb=down.frame, rgb_frame_id=down.frame_id,
                       depth=None, intrinsics=None, depth_stamp_ns=None, depth_frame_id=None,
                       depth_encoding=None, depth_error='DOWN_RGB_ONLY')
    return replace(source, view=view, down_rgb=down if view == 'both' else None)


def image_parts(messages):
    """提取模型实际收到的图像块，保留对应消息。"""
    return [part for message in messages if isinstance(message.get('content'), list)
            for part in message['content'] if part['type'] == 'image_url']


@pytest.mark.parametrize('view', ['front', 'down'])
def test_single_view_does_not_wait_for_unrequested_camera(view, monkeypatch):
    """缺少另一台摄像头时，所选单视角立即可用。"""
    buffer = ObservationBuffer(observation_config())
    stamp = time.time_ns()
    if view == 'front':
        add_pair(buffer, stamp)
    else:
        buffer.add_rgb(sample_down(stamp).frame, stamp, 'CameraDown_optical', view='down')
    wait = Mock(side_effect=AssertionError('不得等待另一视角或下视深度'))
    monkeypatch.setattr(buffer._condition, 'wait', wait)
    result = buffer.wait_for_snapshot(view=view)
    assert result is not None and result.view == view
    wait.assert_not_called()
    if view == 'down':
        assert result.depth is None and result.depth_error == 'DOWN_RGB_ONLY'


@pytest.mark.parametrize('delta_ms,available', [(0, True), (40, True), (100, False)])
def test_dual_view_requires_fresh_time_matched_images(delta_ms, available):
    """双图检查源时间差，超出同步窗口不能静默返回半组。"""
    buffer = ObservationBuffer(observation_config())
    stamp = time.time_ns()
    add_pair(buffer, stamp)
    buffer.add_rgb(sample_down(stamp).frame, stamp - delta_ms * 1_000_000, 'CameraDown_optical', view='down')
    result = buffer.wait_for_snapshot(view='both', timeout_s=0)
    assert (result is not None) is available
    if result is not None:
        assert result.view == 'both' and result.down_rgb is not None
        assert result.down_rgb.stamp_ns == stamp - delta_ms * 1_000_000


def test_dual_view_uses_matching_buffered_pair_when_latest_front_does_not_match():
    """最新前视过晚时仍可选择缓冲内较新且匹配的完整图组。"""
    buffer = ObservationBuffer(observation_config())
    stamp = time.time_ns()
    add_pair(buffer, stamp - 60_000_000)
    add_pair(buffer, stamp)
    buffer.add_rgb(sample_down(stamp).frame, stamp - 70_000_000, view='down')
    result = buffer.latest_snapshot(view='both')
    assert result is not None and result.rgb_stamp_ns == stamp - 60_000_000


@pytest.mark.parametrize('view', ['front', 'down', 'both'])
def test_pre_request_frames_and_scene_reset_do_not_supply_current_observation(view):
    """旧图及场景重置前的各路缓存不能冒充新观察。"""
    buffer = ObservationBuffer(observation_config(), simulation_clock=True)
    buffer.add_clock(2_000_000_000)
    add_pair(buffer, 2_000_000_000)
    buffer.add_rgb(sample_down(2_000_000_000).frame, 2_000_000_000, view='down')
    assert buffer.latest_snapshot(after_stamp_ns=2_000_000_000, view=view) is None
    buffer.add_clock(1_000_000_000)
    assert buffer.latest_snapshot(view=view) is None
    buffer.add_clock(2_000_000_000)
    assert buffer.latest_snapshot(view=view) is None


@pytest.mark.parametrize('view,count', [('front', 1), ('down', 1), ('both', 2)])
def test_selected_images_are_labeled_and_pixel_sources_match(view, count):
    """不仅检查张数，也解码确认前下标签绑定正确的图片内容。"""
    snapshot = view_snapshot(time.time_ns(), view)
    rules = compute_depth_rules(snapshot, observation_config(), 19)
    message = build_observation_message(snapshot, rules, '核对目标')
    images = image_parts([message])
    assert len(images) == count
    texts = [part['text'] for part in message['content'] if part['type'] == 'text']
    assert ('【前方摄像头】' in texts[0]) is (view != 'down')
    assert ('【下方摄像头】' in ''.join(texts)) is (view != 'front')
    if view == 'down':
        assert '前进上限' not in ''.join(texts)
    levels = []
    for part in images:
        encoded = base64.b64decode(part['image_url']['url'].split(',', 1)[1])
        levels.append(int(cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)[5, 5, 0]))
    assert levels == ([0] if view == 'front' else [180] if view == 'down' else [0, 180])


@pytest.mark.parametrize('view', ['front', 'down', 'both'])
def test_selected_view_is_sent_to_same_vlm_in_one_observation(view, tmp_path):
    """每种模式只调用一次观察，双图进入同一次后续模型请求。"""
    wait = Mock(side_effect=lambda *, after_stamp_ns, view: view_snapshot(after_stamp_ns, view))
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait))
    client = FakeClient([
        SimpleNamespace(content='', tool_calls=[tool_call('observe', {'view': view, 'prompt': '核对目标'})]),
        SimpleNamespace(content='已看到所选视角', tool_calls=[]),
    ])
    agent_loop(client, 'same-vlm', [{'role': 'user', 'content': '检查位置'}], context)
    assert len(client.requests) == 2 and len(image_parts(client.requests[-1])) == (2 if view == 'both' else 1)
    wait.assert_called_once()
    assert wait.call_args.kwargs['view'] == view
    event = json.loads((tmp_path / 'logs/session_test/observations.jsonl').read_text().splitlines()[0])
    assert event['view'] == view
    assert (event['down_rgb_stamp_ns'] is not None) is (view != 'front')


def test_new_single_view_replaces_entire_previous_dual_group(tmp_path):
    """双图转单图后不残留旧的另一视角，计划压缩仍保留所选图组。"""
    context = context_for(tmp_path, SimpleNamespace(
        wait_for_observation=lambda *, after_stamp_ns, view: view_snapshot(after_stamp_ns, view)))
    client = FakeClient([SimpleNamespace(content='', tool_calls=[
        tool_call('observe', {'view': view, 'prompt': '检查'}, str(index))])
        for index, view in enumerate(['both', 'front', 'down'])] +
        [SimpleNamespace(content='结束', tool_calls=[])])
    messages = [{'role': 'user', 'content': '检查位置'}]
    agent_loop(client, 'same-vlm', messages, context)
    assert [len(image_parts(request)) for request in client.requests] == [0, 2, 1, 1]
    latest = next(item for item in reversed(messages) if image_parts([item]))
    assert '【下方摄像头】' in latest['content'][0]['text']
    compacted = messages + [{'role': 'assistant', 'content': 'plan'},
                                            {'role': 'tool', 'content': 'ok'}]
    compact_navigation_history(compacted, 0, observation_current=True)
    assert len(image_parts(compacted)) == 1
    measurement = get_tool_definition('measure_distance').handler(context, {'target': '目标实体表面', 'x': .5, 'y': .5})
    assert not measurement['valid'] and measurement['reason'] == 'FRONT_DEPTH_REQUIRED'


@pytest.mark.parametrize('view', [[], 'left', 123, None])
def test_invalid_view_does_not_capture(view, tmp_path):
    """非法视角参数返回明确错误，不取图。"""
    wait = Mock()
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait))
    result = get_tool_definition('observe').handler(context, {'view': view, 'prompt': '检查'})
    assert not result['success'] and result['error'] == 'INVALID_OBSERVE_VIEW'
    wait.assert_not_called()


def test_dual_view_measurement_uses_front_depth_only():
    """双图中测距只绑定前视，不能把下视 RGB 当作深度。"""
    source = view_snapshot(time.time_ns(), 'both')
    result = measure_target_distance(source, observation_config(), .5, .5)
    assert result['valid'] and result['distance_m'] == 4


def test_missing_requested_camera_feedback_allows_reselecting_view(tmp_path):
    """请求下视缺图后，错误交给 VLM，可重新选择可用前视。"""
    wait = Mock(side_effect=lambda *, after_stamp_ns, view: None if view == 'down' else view_snapshot(after_stamp_ns, view))
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait))
    client = FakeClient([
        SimpleNamespace(content='', tool_calls=[tool_call('observe', {'view': 'down', 'prompt': '看脚下'}, '1')]),
        SimpleNamespace(content='', tool_calls=[tool_call('observe', {'view': 'front', 'prompt': '看前方'}, '2')]),
        SimpleNamespace(content='前方图可用', tool_calls=[]),
    ])
    assert agent_loop(client, 'test', [{'role': 'user', 'content': '检查'}], context) == '前方图可用'
    assert not json.loads(client.requests[1][-1]['content'])['success']
    assert len(image_parts(client.requests[-1])) == 1


@pytest.mark.parametrize('view,count', [('front', 1), ('down', 1), ('both', 2)])
def test_plan_history_compaction_preserves_whole_selected_group(view, count):
    """计划压缩不能只留下双图中的一张，失效时整组标为历史。"""
    snapshot = view_snapshot(time.time_ns(), view)
    message = build_observation_message(snapshot, compute_depth_rules(snapshot, observation_config(), 19))
    messages = [message, {'role': 'assistant', 'content': '计划更新'}, {'role': 'tool', 'content': 'ok'}]
    compact_navigation_history(messages, 0, observation_current=False)
    assert len(image_parts(messages)) == count
    assert messages[0]['content'][0]['text'].startswith('历史观察')


def test_ros_down_callback_uses_separate_buffer_and_selected_wait():
    """ROS 下视回调与控制器等待都把视角传到缓存，前视深度不被改写。"""
    from test_mavros_controller import Px4Controller

    controller = object.__new__(Px4Controller)
    controller.observation_buffer = Mock()
    frame = np.full((4, 4, 3), 180, dtype=np.uint8)
    controller.bridge = SimpleNamespace(imgmsg_to_cv2=Mock(return_value=frame))
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0),
                                                    frame_id='PX4/CameraDown_optical'))
    controller.down_rgb_image_callback(message)
    controller.observation_buffer.add_rgb.assert_called_once()
    assert controller.observation_buffer.add_rgb.call_args.kwargs == {'view': 'down'}
    controller.observation_buffer.add_depth.assert_not_called()
    controller._attach_flight_state = Mock(side_effect=lambda value: value)
    controller.wait_for_observation(123, .2, view='down')
    controller.observation_buffer.wait_for_snapshot.assert_called_once_with(123, .2, view='down')


def test_forward_after_down_view_still_acquires_new_front_depth(tmp_path, monkeypatch):
    """刚看下视图也不能用下视 RGB 绕过前进前新深度安全门。"""
    from drone_harness.tools import flight

    views = []

    def wait_for_observation(*, after_stamp_ns, view='front'):
        """记录飞行动作实际获取的视角。"""
        views.append(view)
        return view_snapshot(after_stamp_ns, view)

    controller = SimpleNamespace(wait_for_observation=wait_for_observation,
                                 vehicle_status=SimpleNamespace(mode='OFFBOARD', connected=True, armed=True),
                                 flight_state=lambda: 'IN_AIR')
    context = context_for(tmp_path, controller)
    context.observation = view_snapshot(time.time_ns(), 'down')
    move = Mock(return_value={'success': True})
    monkeypatch.setattr(flight, 'move', move)
    result = flight.forward(context, 2)
    assert result['success'] and result['motion_executed'] and views == ['front']
    move.assert_called_once()


def test_malformed_observe_clears_previous_measurement_binding(tmp_path):
    """参数解析失败也清除旧图和深度，后续测距不能读取旧观察。"""
    from drone_harness.runtime.agent_loop import append_observation

    context = context_for(tmp_path)
    messages = [{'role': 'user', 'content': '检查位置'}]
    append_observation(context, messages, view_snapshot(time.time_ns(), 'front'))
    malformed = SimpleNamespace(id='bad', function=SimpleNamespace(name='observe', arguments='{'))
    client = FakeClient([
        SimpleNamespace(content='', tool_calls=[malformed]),
        SimpleNamespace(content='', tool_calls=[tool_call('measure_distance', {'target': '目标实体表面', 'x': .5, 'y': .5}, 'measure')]),
        SimpleNamespace(content='需要重新观察', tool_calls=[]),
    ])
    agent_loop(client, 'test', messages, context)
    assert not image_parts(client.requests[1])
    assert json.loads(client.requests[2][-1]['content'])['reason'] == 'OBSERVATION_REQUIRED'
    assert context.observation is None and context.depth_rules is None
