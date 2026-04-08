import logging
import time

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_robstride_leader import RobstrideArmLeaderConfig

logger = logging.getLogger(__name__)


class RobstrideArmLeader(Teleoperator):
    """
    Leader / teleop arm using Robstride CAN motors. Intended to pair with `RobstrideArmFollower` like SO100 leader/follower.
    """

    config_class = RobstrideArmLeaderConfig
    name = "robstride_arm_leader"

    def __init__(self, config: RobstrideArmLeaderConfig):
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

    @property
    def action_features(self) -> dict[str, type]:
        # Match SO100-style datasets: joint targets as *.pos only
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting Robstride leader on {self.config.port}...")
        self.bus.connect()

        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()

        if self.is_calibrated:
            self.bus.set_zero_position()

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
            "\nMove the leader to the desired zero pose (should match follower calibration concept), "
            "then press ENTER..."
        )
        self.bus.set_zero_position()
        logger.info("Zero position set.")

        logger.info("Recording joint ranges for leader...")
        arm_motors = [m for m in self.bus.motors if m != "gripper"]
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
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        if self.config.manual_control:
            self.bus.disable_torque()
        else:
            self.bus.configure_motors()

    def setup_motors(self) -> None:
        raise NotImplementedError(
            "Robstride CAN IDs and motor types are configured in `motor_config`; use vendor tools if needed."
        )

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        start = time.perf_counter()
        positions = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in positions.items()}
        logger.debug(f"{self} read action: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(disable_torque=self.config.manual_control)
        logger.info(f"{self} disconnected.")
