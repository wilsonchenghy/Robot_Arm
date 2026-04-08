import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_robstride_follower import RobstrideArmFollowerConfig

logger = logging.getLogger(__name__)


class RobstrideArmFollower(Robot):
    """
    6-DOF + gripper follower using Robstride motors on CAN (MIT mode), similar in role to SO100 follower.
    """

    config_class = RobstrideArmFollowerConfig
    name = "robstride_arm_follower"

    def __init__(self, config: RobstrideArmFollowerConfig):
        super().__init__(config)
        self.config = config

        motors: dict[str, Motor] = {}
        for motor_name, (send_id, recv_id, motor_type_str) in config.motor_config.items():
            motor = Motor(send_id, motor_type_str, MotorNormMode.DEGREES)
            motor.recv_id = recv_id
            motor.motor_type_str = motor_type_str
            motors[motor_name] = motor

        self.bus = RobstrideMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
            can_interface=self.config.can_interface,
            use_can_fd=self.config.use_can_fd,
            bitrate=self.config.can_bitrate,
            data_bitrate=self.config.can_data_bitrate if self.config.use_can_fd else None,
        )

        self._motor_order = list(self.bus.motors.keys())
        self.cameras = make_cameras_from_configs(config.cameras)

    def _kp_kd_for(self, motor_name: str) -> tuple[float, float]:
        idx = self._motor_order.index(motor_name)
        kp = self.config.position_kp[idx]
        kd = self.config.position_kd[idx]
        return kp, kd

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting Robstride arm on {self.config.port}...")
        self.bus.connect()

        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

        if self.is_calibrated:
            self.bus.set_zero_position()

        self.bus.enable_torque()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Using calibration file associated with the id {self.id}")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration for {self}")
        self.bus.disable_torque()

        input(
            "\nMove the arm to the desired zero pose (e.g. comfortable rest / match your leader), "
            "then press ENTER..."
        )
        self.bus.set_zero_position()
        logger.info("Zero position set on all motors.")

        logger.info("Recording joint ranges (move each joint through its workspace; ENTER to finish)...")
        arm_motors = [m for m in self._motor_order if m != "gripper"]
        range_mins, range_maxes = self.bus.record_ranges_of_motion(arm_motors)

        input("\nMove the gripper through open/close, then press ENTER to finish gripper range...")
        g_min, g_max = self.bus.record_ranges_of_motion(["gripper"])
        range_mins["gripper"] = g_min["gripper"]
        range_maxes["gripper"] = g_max["gripper"]

        self.calibration = {}
        for motor_name, motor in self.bus.motors.items():
            self.calibration[motor_name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=0,
                range_min=int(range_mins[motor_name]),
                range_max=int(range_maxes[motor_name]),
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
        for motor_name in self.bus.motors:
            kp, kd = self._kp_kd_for(motor_name)
            self.bus.write("Kp", motor_name, kp)
            self.bus.write("Kd", motor_name, kd)

    def setup_motors(self) -> None:
        raise NotImplementedError(
            "Robstride CAN IDs and motor types are configured in `motor_config`; use vendor tools if needed."
        )

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        obs_dict: dict[str, Any] = {}
        positions = self.bus.sync_read("Present_Position")
        for motor_name, val in positions.items():
            obs_dict[f"{motor_name}.pos"] = val

        for cam_key, cam in self.cameras.items():
            t0 = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            logger.debug(f"{self} read {cam_key}: {(time.perf_counter() - t0) * 1e3:.1f}ms")

        logger.debug(f"{self} read state: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        for motor_name, position in goal_pos.items():
            if motor_name in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[motor_name]
                clipped = max(min_limit, min(max_limit, position))
                if clipped != position:
                    logger.debug(f"Clipped {motor_name} from {position:.2f}° to {clipped:.2f}°")
                goal_pos[motor_name] = clipped

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        commands: dict[str | int, tuple[float, float, float, float, float]] = {}
        for motor_name, position_degrees in goal_pos.items():
            kp, kd = self._kp_kd_for(motor_name)
            commands[motor_name] = (kp, kd, float(position_degrees), 0.0, 0.0)

        self.bus._mit_control_batch(commands)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
