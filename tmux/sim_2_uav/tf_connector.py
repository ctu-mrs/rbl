#!/usr/bin/env python3

import os
import sys
from pathlib import Path


def _maybe_reexec_with_system_python() -> None:
    system_python = "/usr/bin/python3"
    script_name = Path(sys.argv[0]).name if sys.argv else ""
    conda_prefix = os.environ.get("CONDA_PREFIX")
    using_conda_python = sys.executable.startswith(str(Path.home() / "miniconda3")) or sys.executable.startswith(
        str(Path.home() / "anaconda3")
    )

    if not Path(system_python).exists():
        return

    if not (conda_prefix or using_conda_python):
        return

    if script_name != Path(__file__).name:
        return

    if Path(sys.executable).resolve() == Path(system_python).resolve():
        return

    env = os.environ.copy()
    for key in (
        "CONDA_PREFIX",
        "CONDA_DEFAULT_ENV",
        "CONDA_PROMPT_MODIFIER",
        "CONDA_EXE",
        "_CE_CONDA",
        "_CE_M",
        "PYTHONHOME",
    ):
        env.pop(key, None)

    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "PATH"):
        value = env.get(key)
        if not value:
            continue

        filtered_entries = [entry for entry in value.split(":") if "conda" not in entry.lower()]
        if filtered_entries:
            env[key] = ":".join(filtered_entries)
        else:
            env.pop(key, None)

    os.execve(system_python, [system_python, *sys.argv], env)


_maybe_reexec_with_system_python()

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

try:
    import yaml
except ImportError as exc:
    raise SystemExit(f"PyYAML is required to run tf_connector.py: {exc}")


class TfConnector(Node):
    def __init__(self, config_path: Path) -> None:
        super().__init__("tf_connector")

        use_sim_time_default = os.environ.get("USE_SIM_TIME", "false").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", use_sim_time_default)

        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.connecting_frame = config["connecting_frame_id"]
        self.connections = config["connections"]

        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f"TF connector started: connecting_frame={self.connecting_frame}, "
            f"connections={len(self.connections)}, "
            f"use_sim_time={self.get_parameter('use_sim_time').value}"
        )

    def on_timer(self) -> None:
        now = self.get_clock().now().to_msg()

        for connection in self.connections:
            root_frame = connection["root_frame_id"]
            equal_frame = connection["equal_frame_id"]

            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.connecting_frame
            msg.child_frame_id = root_frame

            if root_frame == equal_frame:
                msg.transform.rotation.w = 1.0
                self.broadcaster.sendTransform(msg)
                continue

            try:
                transform = self.buffer.lookup_transform(
                    equal_frame,
                    root_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"Waiting for transform {equal_frame} <- {root_frame}: {exc}",
                    throttle_duration_sec=2.0,
                )
                continue

            msg.transform = transform.transform
            self.broadcaster.sendTransform(msg)


def main() -> int:
    user_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]

    if len(user_args) != 1:
        print("Usage: tf_connector.py <config.yaml>", file=sys.stderr)
        return 1

    config_path = Path(user_args[0]).resolve()
    if not config_path.exists():
        print(f"Config not found: {config_path}", file=sys.stderr)
        return 1

    rclpy.init()
    node = TfConnector(config_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
