"""支持 `python -m drone_agent` 的模块入口。"""

def _main() -> int:
    """作为 `python -m drone_agent` 的进程入口，默认启动仿真模式。"""
    from drone_agent.cli import main_sim

    return main_sim()


if __name__ == "__main__":
    raise SystemExit(_main())
