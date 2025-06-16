# Adapted from LeKiwi

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.common.cameras.configs import CameraConfig
from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.common.robots.config import RobotConfig


def asimov_cameras_config() -> dict[str, CameraConfig]:
    return {
        "wrist_left": OpenCVCameraConfig(
            index_or_path=Path("/dev/video0"),
            fps=30,
            width=640,
            height=480,
        ),
        "wrist_right": OpenCVCameraConfig(
            index_or_path=Path("/dev/video2"),
            fps=30,
            width=640,
            height=480,
        ),
    }


@RobotConfig.register_subclass("network")
@dataclass
class NetworkConfig(RobotConfig):
    port = "/dev/ttyACM0"  # port to connect to the bus

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=asimov_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False


@dataclass
class NetworkHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 30

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("network_follower")
@dataclass
class NetworkClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "w",
            "backward": "s",
            "left": "a",
            "right": "d",
            "rotate_left": "z",
            "rotate_right": "x",
            # Speed control
            "speed_up": "r",
            "speed_down": "f",
            # quit teleop
            "quit": "q",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=asimov_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
