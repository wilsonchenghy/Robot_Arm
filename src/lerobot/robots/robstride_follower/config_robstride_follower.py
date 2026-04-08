from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@dataclass
class RobstrideArmFollowerConfigBase:
    """Configuration for a 6-DOF + gripper arm driven by Robstride MIT motors over CAN."""

    # CAN device: Linux `can0`, or e.g. `/dev/tty.usbmodem*` with `can_interface=slcan`
    port: str

    # CAN interface: "auto", "socketcan", or "slcan"
    can_interface: str = "auto"

    use_can_fd: bool = True
    can_bitrate: int = 1_000_000
    can_data_bitrate: int = 5_000_000

    disable_torque_on_disconnect: bool = True

    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # (send_can_id, recv_id_in_feedback_byte0, robstride_motor_type) per joint.
    # Motor types must match `MotorType` names in `lerobot.motors.robstride.tables` (e.g. O0, O2, O3).
    motor_config: dict[str, tuple[int, int, str]] = field(
        default_factory=lambda: {
            "shoulder_pan": (0x01, 0x11, "O2"),
            "shoulder_lift": (0x02, 0x12, "O2"),
            "elbow_flex": (0x03, 0x13, "O2"),
            "wrist_flex": (0x04, 0x14, "O2"),
            "wrist_roll": (0x05, 0x15, "O2"),
            "wrist_yaw": (0x06, 0x16, "O2"),
            "gripper": (0x07, 0x17, "O2"),
        }
    )

    # MIT position gains (one entry per motor in `motor_config` iteration order — use matching keys below).
    position_kp: list[float] = field(
        default_factory=lambda: [80.0, 80.0, 80.0, 60.0, 40.0, 40.0, 30.0]
    )
    position_kd: list[float] = field(
        default_factory=lambda: [2.0, 2.0, 2.0, 1.5, 1.0, 1.0, 0.8]
    )

    # Clip commanded goals (degrees). Tight defaults for first bring-up; widen after testing.
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (-5.0, 5.0),
            "shoulder_lift": (-5.0, 5.0),
            "elbow_flex": (-5.0, 5.0),
            "wrist_flex": (-5.0, 5.0),
            "wrist_roll": (-5.0, 5.0),
            "wrist_yaw": (-5.0, 5.0),
            "gripper": (-5.0, 0.0),
        }
    )


@RobotConfig.register_subclass("robstride_arm_follower")
@dataclass
class RobstrideArmFollowerConfig(RobotConfig, RobstrideArmFollowerConfigBase):
    pass
