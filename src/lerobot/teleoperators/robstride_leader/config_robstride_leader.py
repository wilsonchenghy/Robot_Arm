from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@dataclass
class RobstrideArmLeaderConfigBase:
    """Leader arm with Robstride motors (same `motor_config` convention as `RobstrideArmFollowerConfig`)."""

    port: str

    can_interface: str = "auto"
    use_can_fd: bool = True
    can_bitrate: int = 1_000_000
    can_data_bitrate: int = 5_000_000

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

    # When True (default), torque is off for gravity/compensation teaching; set False to hold MIT pose.
    manual_control: bool = True


@TeleoperatorConfig.register_subclass("robstride_arm_leader")
@dataclass
class RobstrideArmLeaderConfig(TeleoperatorConfig, RobstrideArmLeaderConfigBase):
    pass
