"""
GELLO teleoperation agent for YAM arms using a Dynamixel-based leader device.

Uses direct joint-to-joint mapping from the Galaxea R1 Lite Teleop (or any
isomorphic GELLO device with Dynamixel servos) to the YAM follower arms.
No inverse kinematics is needed -- the leader and follower share the same
kinematic structure so joint angles transfer directly.

The leader device connects over USB serial.  Both single-arm and bimanual
configurations are supported.

Usage
-----
Launch via YAML config::

    uv run robots_realtime/envs/launch.py --config_path configs/yam_gello_bimanual.yaml
"""

import logging
from typing import Any, Dict, Optional, Sequence

import numpy as np
from dm_env.specs import Array

from robots_realtime.agents.agent import Agent
from robots_realtime.dynamixel.dynamixel_reader import DynamixelReader
from robots_realtime.utils.portal_utils import remote

logger = logging.getLogger(__name__)

# YAM joint limits (radians) from robot_configs/yam/left.yaml
_YAM_JOINT_LIMITS = np.array(
    [
        [-2.09, 3.14],
        [0.00, 3.14],
        [0.05, 3.14],
        [-1.35, 1.35],
        [-1.50, 1.50],
        [-2.00, 2.00],
    ],
    dtype=np.float64,
)


class YamGelloAgent(Agent):
    """Teleoperate YAM arms via a Dynamixel GELLO leader device.

    On the first ``act()`` call the agent captures initial joint positions from
    both the leader device and the YAM follower (via *obs*).  Subsequent calls
    compute::

        yam_target = yam_initial + (leader_current - leader_initial)

    and clamp to YAM joint limits.

    Parameters
    ----------
    port : str
        USB serial port for the Dynamixel chain (e.g. ``"/dev/ttyUSB0"``).
    baudrate : int
        Serial baudrate.  Default 4 000 000 (R1 Lite Teleop default).
    bimanual : bool
        Whether to run in bimanual mode (two arms).
    left_motor_ids : Sequence[int]
        Dynamixel motor IDs for the left arm (base to tip).
    right_motor_ids : Sequence[int]
        Dynamixel motor IDs for the right arm (base to tip).
    joint_signs_left : Sequence[int]
        Per-joint sign correction for the left arm.
    joint_signs_right : Sequence[int]
        Per-joint sign correction for the right arm.
    default_gripper_value : float
        Gripper position sent every step (0.0 = open).
    """

    use_joint_state_as_action: bool = False

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 4_000_000,
        bimanual: bool = False,
        left_motor_ids: Sequence[int] = (1, 2, 3, 4, 5, 6),
        right_motor_ids: Sequence[int] = (7, 8, 9, 10, 11, 12),
        joint_signs_left: Sequence[int] = (1, 1, -1, -1, -1, 1),
        joint_signs_right: Sequence[int] = (1, 1, -1, -1, -1, 1),
        default_gripper_value: float = 0.0,
    ) -> None:
        self.bimanual = bimanual
        self._default_gripper = default_gripper_value
        self._joint_limits = _YAM_JOINT_LIMITS

        all_ids = list(left_motor_ids) + (list(right_motor_ids) if bimanual else [])
        all_signs = list(joint_signs_left) + (list(joint_signs_right) if bimanual else [])

        self._reader = DynamixelReader(
            port=port,
            motor_ids=all_ids,
            joint_signs=all_signs,
            baudrate=baudrate,
        )
        self._n_left = len(left_motor_ids)
        self._n_right = len(right_motor_ids) if bimanual else 0

        self._leader_initial: Optional[Dict[str, np.ndarray]] = None
        self._yam_initial: Optional[Dict[str, np.ndarray]] = None

        logger.info(
            "YamGelloAgent initialized (bimanual=%s, port=%s, left_ids=%s%s)",
            bimanual,
            port,
            list(left_motor_ids),
            f", right_ids={list(right_motor_ids)}" if bimanual else "",
        )

    def _capture_initial(self, obs: Dict[str, Any]) -> None:
        """Capture initial positions from both leader and follower on first act()."""
        leader_pos = self._reader.get_joint_positions()
        leader_left = leader_pos[: self._n_left]

        yam_left = np.asarray(obs["left"]["joint_pos"][:6], dtype=np.float64)

        self._leader_initial = {"left": leader_left.copy()}
        self._yam_initial = {"left": yam_left.copy()}

        if self.bimanual:
            leader_right = leader_pos[self._n_left : self._n_left + self._n_right]
            yam_right = np.asarray(obs["right"]["joint_pos"][:6], dtype=np.float64)
            self._leader_initial["right"] = leader_right.copy()
            self._yam_initial["right"] = yam_right.copy()

        logger.info("Initial positions captured. Leader left: %s", np.round(leader_left, 3))
        if self.bimanual:
            logger.info("Leader right: %s", np.round(self._leader_initial["right"], 3))

    def _compute_target(self, side: str, leader_current: np.ndarray) -> np.ndarray:
        """Compute clamped YAM target from leader delta."""
        assert self._leader_initial is not None and self._yam_initial is not None
        delta = leader_current - self._leader_initial[side]
        target = self._yam_initial[side] + delta
        return np.clip(target, self._joint_limits[:, 0], self._joint_limits[:, 1])

    def act(self, obs: Dict[str, Any]) -> Dict[str, Dict[str, np.ndarray]]:
        if self._leader_initial is None:
            self._capture_initial(obs)

        leader_pos = self._reader.get_joint_positions()
        leader_left = leader_pos[: self._n_left]
        left_target = self._compute_target("left", leader_left)

        action: Dict[str, Dict[str, np.ndarray]] = {
            "left": {
                "pos": np.concatenate([left_target, [self._default_gripper]]),
            }
        }

        if self.bimanual:
            leader_right = leader_pos[self._n_left : self._n_left + self._n_right]
            right_target = self._compute_target("right", leader_right)
            action["right"] = {
                "pos": np.concatenate([right_target, [self._default_gripper]]),
            }

        return action

    @remote(serialization_needed=True)
    def action_spec(self) -> Dict[str, Dict[str, Array]]:
        spec: Dict[str, Dict[str, Array]] = {
            "left": {"pos": Array(shape=(7,), dtype=np.float32)},
        }
        if self.bimanual:
            spec["right"] = {"pos": Array(shape=(7,), dtype=np.float32)}
        return spec

    def close(self) -> None:
        self._reader.close()
        logger.info("YamGelloAgent closed")
