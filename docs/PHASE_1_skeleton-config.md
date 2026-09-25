# drone_agent Phase 1 项目骨架与配置 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立 `drone_agent` 的 Python 包骨架、profile 配置加载、环境变量密钥读取、CLI 入口，并提供可测试的 runtime stub，为后续迁移 `/download/takeoff.py` 打基础。

**Architecture:** Phase 1 不迁移 PX4 控制逻辑，也不接入真实 ROS2 runtime。它只建立 `config -> runtime -> cli` 的最小可运行链路：CLI 选择 profile，loader 读取 YAML 和环境变量，runtime 返回结构化启动信息。后续 Phase 2 才把 `Px4Controller` 接入 runtime。

**Tech Stack:** Python 3.10+、`dataclasses`、`argparse`、`PyYAML`、`pytest`、console script entry points。

---

## Scope

本计划只实现 `DRONE_AGENT_SPEC.md` 中的 Phase 1：

- 创建 Python 包骨架。
- 注册 `drone_agent`、`drone_agent_sim`、`drone_agent_real` 命令入口。
- 创建 `sim.yaml` 和 `real.yaml` profile。
- 实现 profile schema 和 loader。
- API key 通过环境变量读取，不写入源码。
- CLI 可以加载 profile，并调用 runtime stub。
- 单元测试覆盖配置加载、环境变量缺失、CLI profile 映射。

不在本计划实现：

- PX4 DDS controller。
- ROS2 executor。
- OpenAI client。
- tools registry。
- 真实飞行工具。
- VLM 图像分析。

## Target File Map

Create: `/download/drone_agent/pyproject.toml`

项目元数据、依赖和命令入口。Phase 1 只声明基础依赖：`PyYAML`、`pytest`。

Create: `/download/drone_agent/drone_agent/__init__.py`

包版本和公共包标识。

Create: `/download/drone_agent/drone_agent/__main__.py`

支持 `python -m drone_agent`，只调用 `cli.main()`。

Create: `/download/drone_agent/drone_agent/cli.py`

解析 `--profile`、`--task`，并提供 `drone_agent_sim` / `drone_agent_real` 快捷入口函数。

Create: `/download/drone_agent/drone_agent/config/__init__.py`

导出配置加载相关 API。

Create: `/download/drone_agent/drone_agent/config/schema.py`

定义 `RuntimeProfile`、`RosConfig`、`StorageConfig`、`ProviderConfig`、`VlmConfig`、`SafetyConfig`。

Create: `/download/drone_agent/drone_agent/config/loader.py`

读取 YAML profile，解析环境变量，校验字段，并返回 `RuntimeProfile`。

Create: `/download/drone_agent/drone_agent/config/profiles/sim.yaml`

仿真 profile。

Create: `/download/drone_agent/drone_agent/config/profiles/real.yaml`

真机 profile。

Create: `/download/drone_agent/drone_agent/core/__init__.py`

核心运行层包标识。

Create: `/download/drone_agent/drone_agent/core/runtime.py`

Phase 1 runtime stub。它加载 profile 并返回启动信息，不启动 ROS2。

Create: `/download/drone_agent/tests/unit/test_config_loader.py`

测试 profile 加载、环境变量解析、缺失环境变量报错。

Create: `/download/drone_agent/tests/unit/test_cli.py`

测试 CLI profile 选择和快捷入口映射。

## Task 1: Package Skeleton and Console Scripts

**Files:**
- Create: `/download/drone_agent/pyproject.toml`
- Create: `/download/drone_agent/drone_agent/__init__.py`
- Create: `/download/drone_agent/drone_agent/__main__.py`
- Create: `/download/drone_agent/drone_agent/config/__init__.py`
- Create: `/download/drone_agent/drone_agent/core/__init__.py`

- [ ] **Step 1: Create package directories**

Run:

```bash
mkdir -p /download/drone_agent/drone_agent/config/profiles
mkdir -p /download/drone_agent/drone_agent/core
mkdir -p /download/drone_agent/tests/unit
```

Expected: commands exit 0.

- [ ] **Step 2: Create `pyproject.toml`**

Write `/download/drone_agent/pyproject.toml`:

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "drone-agent"
version = "0.1.0"
description = "Natural-language UAV control agent using ROS2 px4_msgs DDS."
requires-python = ">=3.10"
dependencies = [
  "PyYAML>=6.0",
]

[project.optional-dependencies]
dev = [
  "pytest>=8.0",
]

[project.scripts]
drone_agent = "drone_agent.cli:main"
drone_agent_sim = "drone_agent.cli:main_sim"
drone_agent_real = "drone_agent.cli:main_real"

[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["."]
```

- [ ] **Step 3: Create package init files**

Write `/download/drone_agent/drone_agent/__init__.py`:

```python
"""drone_agent package."""

__version__ = "0.1.0"
```

Write `/download/drone_agent/drone_agent/config/__init__.py`:

```python
"""Configuration loading for drone_agent."""
```

Write `/download/drone_agent/drone_agent/core/__init__.py`:

```python
"""Core runtime package for drone_agent."""
```

- [ ] **Step 4: Create module entry point**

Write `/download/drone_agent/drone_agent/__main__.py`:

```python
from drone_agent.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 5: Verify package metadata is readable**

Run:

```bash
cd /download/drone_agent && python3 -m pip install -e .[dev]
```

Expected: command exits 0 and installs `drone-agent` in editable mode.

- [ ] **Step 6: Commit Task 1**

Run:

```bash
cd /download
git add drone_agent/pyproject.toml \
  drone_agent/drone_agent/__init__.py \
  drone_agent/drone_agent/__main__.py \
  drone_agent/drone_agent/config/__init__.py \
  drone_agent/drone_agent/core/__init__.py
git commit -m "feat: scaffold drone agent package"
```

Expected: commit succeeds and includes only the listed files.

## Task 2: Profile Schema

**Files:**
- Create: `/download/drone_agent/drone_agent/config/schema.py`
- Create: `/download/drone_agent/tests/unit/test_config_loader.py`

- [ ] **Step 1: Write failing schema construction test**

Write `/download/drone_agent/tests/unit/test_config_loader.py`:

```python
import pytest

from drone_agent.config.schema import (
    ProviderConfig,
    RosConfig,
    RuntimeProfile,
    SafetyConfig,
    StorageConfig,
    VlmConfig,
)


def test_runtime_profile_has_expected_nested_values():
    profile = RuntimeProfile(
        name="sim",
        mode="simulation",
        ros=RosConfig(
            node_name="drone_agent_sim",
            camera_scene_topic="/airsim_node/PX4/CameraDepth1/Scene",
        ),
        storage=StorageConfig(
            photo_save_dir="/home/hw/picture",
            analysis_save_dir="/home/hw/picture/analysis_frames",
            log_dir="/home/hw/drone_agent_logs/sim",
        ),
        llm=ProviderConfig(
            base_url="https://api.deepseek.com",
            model="deepseek-v4-flash",
            api_key_env="DRONE_AGENT_LLM_API_KEY",
            api_key="llm-key",
        ),
        vlm=VlmConfig(
            enabled=True,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
            model="qwen3-vl-flash",
            api_key_env="DRONE_AGENT_VLM_API_KEY",
            api_key="vlm-key",
        ),
        safety=SafetyConfig(
            require_confirmation_for_real_flight=False,
            max_takeoff_height_m=10.0,
            max_relative_move_m=20.0,
            max_vertical_move_m=10.0,
            max_rotation_deg=360.0,
            action_timeout_s=30.0,
            hover_on_timeout=True,
            stop_after_requires_confirmation=True,
        ),
    )

    assert profile.name == "sim"
    assert profile.ros.node_name == "drone_agent_sim"
    assert profile.vlm.enabled is True
    assert profile.safety.max_takeoff_height_m == 10.0


def test_safety_config_rejects_non_positive_timeout():
    with pytest.raises(ValueError, match="action_timeout_s must be positive"):
        SafetyConfig(
            require_confirmation_for_real_flight=False,
            max_takeoff_height_m=10.0,
            max_relative_move_m=20.0,
            max_vertical_move_m=10.0,
            max_rotation_deg=360.0,
            action_timeout_s=0.0,
            hover_on_timeout=True,
            stop_after_requires_confirmation=True,
        )
```

- [ ] **Step 2: Run test and verify it fails**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_config_loader.py -v
```

Expected: FAIL because `drone_agent.config.schema` does not exist.

- [ ] **Step 3: Implement schema dataclasses**

Write `/download/drone_agent/drone_agent/config/schema.py`:

```python
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RosConfig:
    node_name: str
    camera_scene_topic: str | None

    def __post_init__(self) -> None:
        if not self.node_name:
            raise ValueError("ros.node_name is required")


@dataclass(frozen=True)
class StorageConfig:
    photo_save_dir: str
    analysis_save_dir: str
    log_dir: str

    def __post_init__(self) -> None:
        if not self.photo_save_dir:
            raise ValueError("storage.photo_save_dir is required")
        if not self.analysis_save_dir:
            raise ValueError("storage.analysis_save_dir is required")
        if not self.log_dir:
            raise ValueError("storage.log_dir is required")


@dataclass(frozen=True)
class ProviderConfig:
    base_url: str
    model: str
    api_key_env: str
    api_key: str

    def __post_init__(self) -> None:
        if not self.base_url:
            raise ValueError("provider base_url is required")
        if not self.model:
            raise ValueError("provider model is required")
        if not self.api_key_env:
            raise ValueError("provider api_key_env is required")
        if not self.api_key:
            raise ValueError(f"environment variable {self.api_key_env} is required")


@dataclass(frozen=True)
class VlmConfig:
    enabled: bool
    base_url: str | None
    model: str | None
    api_key_env: str | None
    api_key: str | None

    def __post_init__(self) -> None:
        if not self.enabled:
            return
        if not self.base_url:
            raise ValueError("vlm.base_url is required when vlm.enabled=true")
        if not self.model:
            raise ValueError("vlm.model is required when vlm.enabled=true")
        if not self.api_key_env:
            raise ValueError("vlm.api_key_env is required when vlm.enabled=true")
        if not self.api_key:
            raise ValueError(f"environment variable {self.api_key_env} is required")


@dataclass(frozen=True)
class SafetyConfig:
    require_confirmation_for_real_flight: bool
    max_takeoff_height_m: float
    max_relative_move_m: float
    max_vertical_move_m: float
    max_rotation_deg: float
    action_timeout_s: float
    hover_on_timeout: bool
    stop_after_requires_confirmation: bool

    def __post_init__(self) -> None:
        if self.max_takeoff_height_m <= 0:
            raise ValueError("max_takeoff_height_m must be positive")
        if self.max_relative_move_m <= 0:
            raise ValueError("max_relative_move_m must be positive")
        if self.max_vertical_move_m <= 0:
            raise ValueError("max_vertical_move_m must be positive")
        if self.max_rotation_deg <= 0:
            raise ValueError("max_rotation_deg must be positive")
        if self.action_timeout_s <= 0:
            raise ValueError("action_timeout_s must be positive")


@dataclass(frozen=True)
class RuntimeProfile:
    name: str
    mode: str
    ros: RosConfig
    storage: StorageConfig
    llm: ProviderConfig
    vlm: VlmConfig
    safety: SafetyConfig

    def __post_init__(self) -> None:
        if self.name not in {"sim", "real"}:
            raise ValueError("profile name must be 'sim' or 'real'")
        if self.mode not in {"simulation", "real"}:
            raise ValueError("profile mode must be 'simulation' or 'real'")
```

- [ ] **Step 4: Run schema tests and verify they pass**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_config_loader.py -v
```

Expected: PASS for both tests.

- [ ] **Step 5: Commit Task 2**

Run:

```bash
cd /download
git add drone_agent/drone_agent/config/schema.py drone_agent/tests/unit/test_config_loader.py
git commit -m "feat: add runtime profile schema"
```

Expected: commit succeeds.

## Task 3: Profile YAML Files and Loader

**Files:**
- Create: `/download/drone_agent/drone_agent/config/loader.py`
- Create: `/download/drone_agent/drone_agent/config/profiles/sim.yaml`
- Create: `/download/drone_agent/drone_agent/config/profiles/real.yaml`
- Modify: `/download/drone_agent/drone_agent/config/__init__.py`
- Modify: `/download/drone_agent/tests/unit/test_config_loader.py`

- [ ] **Step 1: Extend tests for profile loading**

Append to `/download/drone_agent/tests/unit/test_config_loader.py`:

```python
from pathlib import Path

from drone_agent.config.loader import ConfigError, load_profile


def test_load_sim_profile_reads_yaml_and_environment(monkeypatch):
    monkeypatch.setenv("DRONE_AGENT_LLM_API_KEY", "llm-secret")
    monkeypatch.setenv("DRONE_AGENT_VLM_API_KEY", "vlm-secret")

    profile = load_profile("sim")

    assert profile.name == "sim"
    assert profile.mode == "simulation"
    assert profile.ros.node_name == "drone_agent_sim"
    assert profile.ros.camera_scene_topic == "/airsim_node/PX4/CameraDepth1/Scene"
    assert profile.llm.api_key == "llm-secret"
    assert profile.vlm.api_key == "vlm-secret"
    assert profile.safety.require_confirmation_for_real_flight is False


def test_load_real_profile_uses_stricter_confirmation(monkeypatch):
    monkeypatch.setenv("DRONE_AGENT_LLM_API_KEY", "llm-secret")
    monkeypatch.setenv("DRONE_AGENT_VLM_API_KEY", "vlm-secret")

    profile = load_profile("real")

    assert profile.name == "real"
    assert profile.mode == "real"
    assert profile.ros.node_name == "drone_agent_real"
    assert profile.safety.require_confirmation_for_real_flight is True
    assert profile.safety.max_takeoff_height_m <= 3.0


def test_load_profile_rejects_unknown_profile():
    with pytest.raises(ConfigError, match="unknown profile"):
        load_profile("lab")


def test_load_profile_requires_llm_api_key(monkeypatch):
    monkeypatch.delenv("DRONE_AGENT_LLM_API_KEY", raising=False)
    monkeypatch.setenv("DRONE_AGENT_VLM_API_KEY", "vlm-secret")

    with pytest.raises(ConfigError, match="DRONE_AGENT_LLM_API_KEY"):
        load_profile("sim")


def test_load_profile_can_use_explicit_profile_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("CUSTOM_LLM_KEY", "custom-llm")
    profile_dir = tmp_path / "profiles"
    profile_dir.mkdir()
    (profile_dir / "sim.yaml").write_text(
        '''
name: sim
mode: simulation
ros:
  node_name: custom_sim
  camera_scene_topic: /camera
storage:
  photo_save_dir: /tmp/photos
  analysis_save_dir: /tmp/analysis
  log_dir: /tmp/logs
llm:
  base_url: https://example.com/v1
  model: test-model
  api_key_env: CUSTOM_LLM_KEY
vlm:
  enabled: false
safety:
  require_confirmation_for_real_flight: false
  max_takeoff_height_m: 2
  max_relative_move_m: 3
  max_vertical_move_m: 1
  max_rotation_deg: 90
  action_timeout_s: 5
  hover_on_timeout: true
  stop_after_requires_confirmation: true
'''.strip(),
        encoding="utf-8",
    )

    profile = load_profile("sim", profile_dir=Path(profile_dir))

    assert profile.ros.node_name == "custom_sim"
    assert profile.vlm.enabled is False
    assert profile.vlm.api_key is None
```

- [ ] **Step 2: Run tests and verify loader failures**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_config_loader.py -v
```

Expected: FAIL because `drone_agent.config.loader` and profile YAML files do not exist.

- [ ] **Step 3: Create simulation profile**

Write `/download/drone_agent/drone_agent/config/profiles/sim.yaml`:

```yaml
name: sim
mode: simulation

ros:
  node_name: drone_agent_sim
  camera_scene_topic: /airsim_node/PX4/CameraDepth1/Scene

storage:
  photo_save_dir: /home/hw/picture
  analysis_save_dir: /home/hw/picture/analysis_frames
  log_dir: /home/hw/drone_agent_logs/sim

llm:
  base_url: https://api.deepseek.com
  model: deepseek-v4-flash
  api_key_env: DRONE_AGENT_LLM_API_KEY

vlm:
  enabled: true
  base_url: https://dashscope.aliyuncs.com/compatible-mode/v1
  model: qwen3-vl-flash
  api_key_env: DRONE_AGENT_VLM_API_KEY

safety:
  require_confirmation_for_real_flight: false
  max_takeoff_height_m: 10
  max_relative_move_m: 20
  max_vertical_move_m: 10
  max_rotation_deg: 360
  action_timeout_s: 30
  hover_on_timeout: true
  stop_after_requires_confirmation: true
```

- [ ] **Step 4: Create real-flight profile**

Write `/download/drone_agent/drone_agent/config/profiles/real.yaml`:

```yaml
name: real
mode: real

ros:
  node_name: drone_agent_real
  camera_scene_topic: null

storage:
  photo_save_dir: /home/hw/picture_real
  analysis_save_dir: /home/hw/picture_real/analysis_frames
  log_dir: /home/hw/drone_agent_logs/real

llm:
  base_url: https://api.deepseek.com
  model: deepseek-v4-flash
  api_key_env: DRONE_AGENT_LLM_API_KEY

vlm:
  enabled: true
  base_url: https://dashscope.aliyuncs.com/compatible-mode/v1
  model: qwen3-vl-flash
  api_key_env: DRONE_AGENT_VLM_API_KEY

safety:
  require_confirmation_for_real_flight: true
  max_takeoff_height_m: 3
  max_relative_move_m: 5
  max_vertical_move_m: 2
  max_rotation_deg: 180
  action_timeout_s: 20
  hover_on_timeout: true
  stop_after_requires_confirmation: true
```

- [ ] **Step 5: Implement profile loader**

Write `/download/drone_agent/drone_agent/config/loader.py`:

```python
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

from drone_agent.config.schema import (
    ProviderConfig,
    RosConfig,
    RuntimeProfile,
    SafetyConfig,
    StorageConfig,
    VlmConfig,
)


class ConfigError(RuntimeError):
    """Raised when runtime profile configuration is invalid."""


DEFAULT_PROFILE_DIR = Path(__file__).resolve().parent / "profiles"


def load_profile(profile_name: str, profile_dir: Path | None = None) -> RuntimeProfile:
    selected_dir = profile_dir or DEFAULT_PROFILE_DIR
    profile_path = selected_dir / f"{profile_name}.yaml"
    if profile_name not in {"sim", "real"}:
        raise ConfigError(f"unknown profile: {profile_name}")
    if not profile_path.exists():
        raise ConfigError(f"profile file not found: {profile_path}")

    try:
        raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"failed to parse profile {profile_path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ConfigError(f"profile must be a mapping: {profile_path}")

    try:
        return _build_profile(raw)
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(str(exc)) from exc


def _build_profile(raw: dict[str, Any]) -> RuntimeProfile:
    ros = raw["ros"]
    storage = raw["storage"]
    llm = raw["llm"]
    vlm = raw["vlm"]
    safety = raw["safety"]

    llm_key_env = str(llm["api_key_env"])
    llm_api_key = os.getenv(llm_key_env, "")

    vlm_enabled = bool(vlm.get("enabled", False))
    vlm_key_env = vlm.get("api_key_env")
    vlm_api_key = os.getenv(str(vlm_key_env), "") if vlm_enabled and vlm_key_env else None

    return RuntimeProfile(
        name=str(raw["name"]),
        mode=str(raw["mode"]),
        ros=RosConfig(
            node_name=str(ros["node_name"]),
            camera_scene_topic=ros.get("camera_scene_topic"),
        ),
        storage=StorageConfig(
            photo_save_dir=str(storage["photo_save_dir"]),
            analysis_save_dir=str(storage["analysis_save_dir"]),
            log_dir=str(storage["log_dir"]),
        ),
        llm=ProviderConfig(
            base_url=str(llm["base_url"]),
            model=str(llm["model"]),
            api_key_env=llm_key_env,
            api_key=llm_api_key,
        ),
        vlm=VlmConfig(
            enabled=vlm_enabled,
            base_url=vlm.get("base_url"),
            model=vlm.get("model"),
            api_key_env=str(vlm_key_env) if vlm_key_env else None,
            api_key=vlm_api_key,
        ),
        safety=SafetyConfig(
            require_confirmation_for_real_flight=bool(
                safety["require_confirmation_for_real_flight"]
            ),
            max_takeoff_height_m=float(safety["max_takeoff_height_m"]),
            max_relative_move_m=float(safety["max_relative_move_m"]),
            max_vertical_move_m=float(safety["max_vertical_move_m"]),
            max_rotation_deg=float(safety["max_rotation_deg"]),
            action_timeout_s=float(safety["action_timeout_s"]),
            hover_on_timeout=bool(safety["hover_on_timeout"]),
            stop_after_requires_confirmation=bool(
                safety["stop_after_requires_confirmation"]
            ),
        ),
    )
```

- [ ] **Step 6: Export loader API from config package**

Replace `/download/drone_agent/drone_agent/config/__init__.py`:

```python
"""Configuration loading for drone_agent."""

from drone_agent.config.loader import ConfigError, load_profile
from drone_agent.config.schema import RuntimeProfile

__all__ = ["ConfigError", "RuntimeProfile", "load_profile"]
```

- [ ] **Step 7: Run config tests and verify they pass**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_config_loader.py -v
```

Expected: PASS for all config loader tests.

- [ ] **Step 8: Commit Task 3**

Run:

```bash
cd /download
git add drone_agent/drone_agent/config/loader.py \
  drone_agent/drone_agent/config/__init__.py \
  drone_agent/drone_agent/config/profiles/sim.yaml \
  drone_agent/drone_agent/config/profiles/real.yaml \
  drone_agent/tests/unit/test_config_loader.py
git commit -m "feat: load runtime profiles"
```

Expected: commit succeeds.

## Task 4: Runtime Stub

**Files:**
- Create: `/download/drone_agent/drone_agent/core/runtime.py`
- Create: `/download/drone_agent/tests/unit/test_runtime.py`

- [ ] **Step 1: Write failing runtime test**

Write `/download/drone_agent/tests/unit/test_runtime.py`:

```python
from drone_agent.core.runtime import RuntimeStartResult, start_runtime


def test_start_runtime_returns_profile_summary(monkeypatch):
    monkeypatch.setenv("DRONE_AGENT_LLM_API_KEY", "llm-secret")
    monkeypatch.setenv("DRONE_AGENT_VLM_API_KEY", "vlm-secret")

    result = start_runtime(profile_name="sim", task="起飞到1米", interactive=False)

    assert isinstance(result, RuntimeStartResult)
    assert result.profile_name == "sim"
    assert result.mode == "simulation"
    assert result.node_name == "drone_agent_sim"
    assert result.task == "起飞到1米"
    assert result.interactive is False
    assert result.ros_started is False


def test_start_runtime_rejects_interactive_without_task_in_non_interactive_mode(monkeypatch):
    monkeypatch.setenv("DRONE_AGENT_LLM_API_KEY", "llm-secret")
    monkeypatch.setenv("DRONE_AGENT_VLM_API_KEY", "vlm-secret")

    result = start_runtime(profile_name="real", task=None, interactive=True)

    assert result.profile_name == "real"
    assert result.interactive is True
    assert result.task is None
```

- [ ] **Step 2: Run runtime test and verify it fails**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_runtime.py -v
```

Expected: FAIL because `drone_agent.core.runtime` does not exist.

- [ ] **Step 3: Implement runtime stub**

Write `/download/drone_agent/drone_agent/core/runtime.py`:

```python
from __future__ import annotations

from dataclasses import dataclass

from drone_agent.config.loader import load_profile


@dataclass(frozen=True)
class RuntimeStartResult:
    profile_name: str
    mode: str
    node_name: str
    task: str | None
    interactive: bool
    ros_started: bool


def start_runtime(
    profile_name: str,
    task: str | None = None,
    interactive: bool = True,
) -> RuntimeStartResult:
    profile = load_profile(profile_name)
    return RuntimeStartResult(
        profile_name=profile.name,
        mode=profile.mode,
        node_name=profile.ros.node_name,
        task=task,
        interactive=interactive,
        ros_started=False,
    )
```

- [ ] **Step 4: Run runtime tests and verify they pass**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_runtime.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit Task 4**

Run:

```bash
cd /download
git add drone_agent/drone_agent/core/runtime.py drone_agent/tests/unit/test_runtime.py
git commit -m "feat: add runtime startup stub"
```

Expected: commit succeeds.

## Task 5: CLI Entry Points

**Files:**
- Create: `/download/drone_agent/drone_agent/cli.py`
- Create: `/download/drone_agent/tests/unit/test_cli.py`

- [ ] **Step 1: Write failing CLI tests**

Write `/download/drone_agent/tests/unit/test_cli.py`:

```python
from types import SimpleNamespace

from drone_agent import cli


def test_main_uses_requested_profile(monkeypatch, capsys):
    calls = []

    def fake_start_runtime(profile_name, task=None, interactive=True):
        calls.append(
            {
                "profile_name": profile_name,
                "task": task,
                "interactive": interactive,
            }
        )
        return SimpleNamespace(
            profile_name=profile_name,
            mode="real",
            node_name="drone_agent_real",
            ros_started=False,
        )

    monkeypatch.setattr(cli, "start_runtime", fake_start_runtime)

    exit_code = cli.main(["--profile", "real", "--task", "查询状态"])

    assert exit_code == 0
    assert calls == [
        {
            "profile_name": "real",
            "task": "查询状态",
            "interactive": False,
        }
    ]
    assert "profile=real" in capsys.readouterr().out


def test_main_defaults_to_sim_profile(monkeypatch):
    calls = []

    def fake_start_runtime(profile_name, task=None, interactive=True):
        calls.append((profile_name, task, interactive))
        return SimpleNamespace(
            profile_name=profile_name,
            mode="simulation",
            node_name="drone_agent_sim",
            ros_started=False,
        )

    monkeypatch.setattr(cli, "start_runtime", fake_start_runtime)

    exit_code = cli.main([])

    assert exit_code == 0
    assert calls == [("sim", None, True)]


def test_main_sim_forces_sim_profile(monkeypatch):
    calls = []

    def fake_start_runtime(profile_name, task=None, interactive=True):
        calls.append((profile_name, task, interactive))
        return SimpleNamespace(
            profile_name=profile_name,
            mode="simulation",
            node_name="drone_agent_sim",
            ros_started=False,
        )

    monkeypatch.setattr(cli, "start_runtime", fake_start_runtime)

    exit_code = cli.main_sim(["--task", "起飞"])

    assert exit_code == 0
    assert calls == [("sim", "起飞", False)]


def test_main_real_forces_real_profile(monkeypatch):
    calls = []

    def fake_start_runtime(profile_name, task=None, interactive=True):
        calls.append((profile_name, task, interactive))
        return SimpleNamespace(
            profile_name=profile_name,
            mode="real",
            node_name="drone_agent_real",
            ros_started=False,
        )

    monkeypatch.setattr(cli, "start_runtime", fake_start_runtime)

    exit_code = cli.main_real(["--task", "查询电池"])

    assert exit_code == 0
    assert calls == [("real", "查询电池", False)]
```

- [ ] **Step 2: Run CLI tests and verify they fail**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_cli.py -v
```

Expected: FAIL because `drone_agent.cli` does not exist.

- [ ] **Step 3: Implement CLI module**

Write `/download/drone_agent/drone_agent/cli.py`:

```python
from __future__ import annotations

import argparse
from collections.abc import Sequence

from drone_agent.core.runtime import start_runtime


def build_parser(default_profile: str = "sim") -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="drone_agent",
        description="Natural-language UAV control agent.",
    )
    parser.add_argument(
        "--profile",
        choices=("sim", "real"),
        default=default_profile,
        help="Runtime profile to load.",
    )
    parser.add_argument(
        "--task",
        help="Run one natural-language task and exit.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return _run(profile_name=args.profile, task=args.task)


def main_sim(argv: Sequence[str] | None = None) -> int:
    parser = build_parser(default_profile="sim")
    parser.set_defaults(profile="sim")
    args = parser.parse_args(argv)
    return _run(profile_name="sim", task=args.task)


def main_real(argv: Sequence[str] | None = None) -> int:
    parser = build_parser(default_profile="real")
    parser.set_defaults(profile="real")
    args = parser.parse_args(argv)
    return _run(profile_name="real", task=args.task)


def _run(profile_name: str, task: str | None) -> int:
    result = start_runtime(
        profile_name=profile_name,
        task=task,
        interactive=task is None,
    )
    print(
        "drone_agent runtime prepared: "
        f"profile={result.profile_name} "
        f"mode={result.mode} "
        f"node={result.node_name} "
        f"ros_started={result.ros_started}"
    )
    return 0
```

- [ ] **Step 4: Run CLI tests and verify they pass**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_cli.py -v
```

Expected: PASS.

- [ ] **Step 5: Verify console entry command works**

Run:

```bash
cd /download/drone_agent && DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y drone_agent --profile sim --task "查询状态"
```

Expected output contains:

```text
drone_agent runtime prepared: profile=sim mode=simulation node=drone_agent_sim ros_started=False
```

Run:

```bash
cd /download/drone_agent && DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y drone_agent_real --task "查询状态"
```

Expected output contains:

```text
drone_agent runtime prepared: profile=real mode=real node=drone_agent_real ros_started=False
```

- [ ] **Step 6: Commit Task 5**

Run:

```bash
cd /download
git add drone_agent/drone_agent/cli.py drone_agent/tests/unit/test_cli.py
git commit -m "feat: add drone agent cli entrypoints"
```

Expected: commit succeeds.

## Task 6: Documentation and Phase 1 Verification

**Files:**
- Create: `/download/drone_agent/README.md`

- [ ] **Step 1: Create README quick start**

Write `/download/drone_agent/README.md`:

````markdown
# drone_agent

`drone_agent` 是一个自然语言无人机控制 Agent。当前项目主线基于 ROS2 `rclpy`、`px4_msgs` 和 PX4 uXRCE-DDS。Phase 1 只提供项目骨架、profile 配置和 CLI 入口；PX4 控制逻辑后续从 `/download/takeoff.py` 迁移。

## 安装

```bash
cd /download/drone_agent
python3 -m pip install -e .[dev]
```

## 环境变量

```bash
export DRONE_AGENT_LLM_API_KEY="your-llm-key"
export DRONE_AGENT_VLM_API_KEY="your-vlm-key"
```

API key 不能写入源码。

## 启动

```bash
drone_agent --profile sim
drone_agent --profile real
drone_agent_sim
drone_agent_real
```

单次任务：

```bash
drone_agent --profile sim --task "查询状态"
drone_agent_real --task "查询电池"
```

Phase 1 的 runtime 不启动 ROS2，只验证 profile 和 CLI 链路。真实 PX4 控制从 Phase 2 开始迁移。
````

- [ ] **Step 2: Run full Phase 1 tests**

Run:

```bash
cd /download/drone_agent && pytest -v
```

Expected: all tests pass.

- [ ] **Step 3: Run import check**

Run:

```bash
cd /download/drone_agent && python3 -m drone_agent --profile sim --task "查询状态"
```

Expected without environment variables: fails with message containing `DRONE_AGENT_LLM_API_KEY`.

Run:

```bash
cd /download/drone_agent && DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y python3 -m drone_agent --profile sim --task "查询状态"
```

Expected output contains:

```text
drone_agent runtime prepared: profile=sim mode=simulation node=drone_agent_sim ros_started=False
```

- [ ] **Step 4: Verify no API keys are committed**

Run:

```bash
cd /download/drone_agent && grep -R "sk-" -n drone_agent tests pyproject.toml README.md
```

Expected: no output and exit code 1.

- [ ] **Step 5: Commit Task 6**

Run:

```bash
cd /download
git add drone_agent/README.md
git commit -m "docs: add drone agent phase one usage"
```

Expected: commit succeeds.

## Final Verification

- [ ] **Step 1: Run all tests**

Run:

```bash
cd /download/drone_agent && pytest -v
```

Expected: all tests pass.

- [ ] **Step 2: Verify CLI commands**

Run:

```bash
cd /download/drone_agent && DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y drone_agent --profile sim --task "查询状态"
```

Expected output contains:

```text
profile=sim mode=simulation node=drone_agent_sim ros_started=False
```

Run:

```bash
cd /download/drone_agent && DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y drone_agent_real --task "查询状态"
```

Expected output contains:

```text
profile=real mode=real node=drone_agent_real ros_started=False
```

- [ ] **Step 3: Verify committed file set**

Run:

```bash
cd /download && git status --short drone_agent docs/superpowers/plans/2026-06-10-drone-agent-phase-1-skeleton-config.md
```

Expected: only files intentionally created by Phase 1 are changed or untracked before final commit.

- [ ] **Step 4: Commit implementation plan if not already committed**

Run:

```bash
cd /download
git add docs/superpowers/plans/2026-06-10-drone-agent-phase-1-skeleton-config.md
git commit -m "docs: add drone agent phase one implementation plan"
```

Expected: commit succeeds unless the plan was already committed earlier.

## Plan Self-Review

Spec coverage:

- Phase 1 package skeleton is covered by Task 1.
- Profile loading and environment-variable secret handling are covered by Tasks 2 and 3.
- CLI commands `drone_agent`、`drone_agent_sim`、`drone_agent_real` are covered by Tasks 1 and 5.
- Runtime lifecycle is represented as a stub in Task 4, with ROS2 intentionally deferred to Phase 2.
- Documentation is covered by Task 6.

Known intentional gaps:

- PX4 controller migration is deferred to Phase 2.
- Tools and agent loop migration are deferred to Phase 3.
- Safety runtime policy is deferred to Phase 4, except for profile safety values.
- VLM runtime calls are deferred; Phase 1 only validates VLM profile configuration.
