"""
VR teleoperation agent for bimanual YAM arms using Pico VR controllers.
Replaces the Viser gizmo input with VR controller poses while reusing
the YamPyroki IK solver and Viser visualization.

Requires:
  - XRoboToolkit PC Service running on the PC
  - xrobotoolkit_sdk installed (bash scripts/install_xrobotoolkit_sdk.sh)
  - Pico headset connected and streaming
"""

import threading
import time
from copy import deepcopy
from typing import Any, Dict, Optional

import numpy as np
import viser
import viser.extras
import viser.transforms as vtf
from dm_env.specs import Array

from robots_realtime.agents.agent import Agent
from robots_realtime.robots.inverse_kinematics.yam_pyroki import YamPyroki
from robots_realtime.sensors.cameras.camera_utils import obs_get_rgb, resize_with_pad
from robots_realtime.utils.portal_utils import remote
from robots_realtime.utils.xr_client import XrClient

# Default VR-to-robot frame rotation.
# Maps VR headset frame (x=right, y=up, z=back) to robot base frame (x=forward, y=left, z=up).
# Adjust if your robot base orientation differs.
R_VR_TO_ROBOT_DEFAULT = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
], dtype=np.float64)

GRIP_ACTIVATION_THRESHOLD = 0.9
YAM_GRIPPER_OPEN = 0.0
YAM_GRIPPER_CLOSED = 2.4


class YamPyrokiVrAgent(Agent):
    def __init__(
        self,
        bimanual: bool = False,
        right_arm_extrinsic: Optional[Dict[str, Any]] = None,
        scale_factor: float = 1.5,
        R_vr_to_robot: Optional[np.ndarray] = None,
    ) -> None:
        self.bimanual = bimanual
        self.right_arm_extrinsic = right_arm_extrinsic
        self.scale_factor = scale_factor
        self.R_vr_to_robot = R_vr_to_robot if R_vr_to_robot is not None else R_VR_TO_ROBOT_DEFAULT
        self.R_vr_so3 = vtf.SO3.from_matrix(self.R_vr_to_robot)

        if bimanual:
            assert right_arm_extrinsic is not None, "right_arm_extrinsic must be provided for bimanual robot"

        self.viser_server = viser.ViserServer()
        self.ik = YamPyroki(viser_server=self.viser_server, bimanual=bimanual)

        # VR state tracking per arm
        self.sides = ["left", "right"] if bimanual else ["left"]
        self.ref_vr_pos: Dict[str, Optional[np.ndarray]] = {s: None for s in self.sides}
        self.ref_vr_rot: Dict[str, Optional[vtf.SO3]] = {s: None for s in self.sides}
        self.ref_ee_pos: Dict[str, Optional[np.ndarray]] = {s: None for s in self.sides}
        self.ref_ee_rot: Dict[str, Optional[vtf.SO3]] = {s: None for s in self.sides}
        self.gripper_value: Dict[str, float] = {s: YAM_GRIPPER_OPEN for s in self.sides}
        self.active: Dict[str, bool] = {s: False for s in self.sides}

        self.xr_client = XrClient()

        # Setup visualization before starting threads that depend on GUI handles
        self.obs = None
        self._setup_visualization()

        # Start IK solver thread
        self.ik_thread = threading.Thread(target=self.ik.run, daemon=True)
        self.ik_thread.start()

        # Start VR input processing thread
        self.vr_thread = threading.Thread(target=self._vr_processing_loop, daemon=True)
        self.vr_thread.start()

        # Start visualization thread
        self.real_vis_thread = threading.Thread(target=self._update_visualization, daemon=True)
        self.real_vis_thread.start()

    def _setup_visualization(self) -> None:
        """Setup semi-transparent real robot state overlay in Viser."""
        self.base_frame_left_real = self.viser_server.scene.add_frame("/base_left_real", show_axes=False)
        self.urdf_vis_left_real = viser.extras.ViserUrdf(
            self.viser_server,
            deepcopy(self.ik.urdf),
            root_node_name="/base_left_real",
            mesh_color_override=(0.8, 0.5, 0.5),
        )
        for mesh in self.urdf_vis_left_real._meshes:
            mesh.opacity = 0.25  # type: ignore

        if self.bimanual and self.right_arm_extrinsic is not None:
            self.ik.base_frame_right.position = np.array(self.right_arm_extrinsic["position"])
            self.ik.base_frame_right.wxyz = np.array(self.right_arm_extrinsic["rotation"])
            self.base_frame_right_real = self.viser_server.scene.add_frame(
                "/base_left_real/base_right_real", show_axes=False
            )
            self.base_frame_right_real.position = self.ik.base_frame_right.position
            self.urdf_vis_right_real = viser.extras.ViserUrdf(
                self.viser_server,
                deepcopy(self.ik.urdf),
                root_node_name="/base_left_real/base_right_real",
                mesh_color_override=(0.8, 0.5, 0.5),
            )
            for mesh in self.urdf_vis_right_real._meshes:
                mesh.opacity = 0.25  # type: ignore

        # Disable gizmo drag interaction — VR drives the targets, not mouse.
        # Gizmos remain visible as target position indicators.
        for handle in self.ik.transform_handles.values():
            if handle.control is not None:
                handle.control.visible = False

        # VR status display
        self.vr_status_handles = {}
        for side in self.sides:
            self.vr_status_handles[side] = self.viser_server.gui.add_text(
                f"VR {side.title()}", initial_value="inactive"
            )
        self.viser_cam_img_handles: Dict[str, Any] = {}

    def _transform_vr_pose(self, vr_pose: np.ndarray) -> tuple[np.ndarray, vtf.SO3]:
        """Transform a VR controller pose from VR frame to robot base frame.

        Args:
            vr_pose: [x, y, z, qx, qy, qz, qw] from SDK.

        Returns:
            (position_xyz, SO3_rotation) in robot frame.
        """
        pos_robot = self.R_vr_to_robot @ np.array(vr_pose[:3])

        quat_xyzw = vr_pose[3:7]
        rot_vr = vtf.SO3(np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]))
        rot_robot = self.R_vr_so3 @ rot_vr @ self.R_vr_so3.inverse()

        return pos_robot, rot_robot

    def _vr_processing_loop(self) -> None:
        """Read VR controller input and update IK targets at ~100Hz."""
        while True:
            for side in self.sides:
                grip_val = self.xr_client.get_grip(side)
                trigger_val = self.xr_client.get_trigger(side)
                self.gripper_value[side] = trigger_val * YAM_GRIPPER_CLOSED

                was_active = self.active[side]
                is_active = grip_val > GRIP_ACTIVATION_THRESHOLD
                self.active[side] = is_active

                if is_active:
                    vr_pose = self.xr_client.get_pose(f"{side}_controller")
                    pos_robot, rot_robot = self._transform_vr_pose(vr_pose)

                    if not was_active:
                        # Just activated — capture references
                        self.ref_vr_pos[side] = pos_robot.copy()
                        self.ref_vr_rot[side] = rot_robot
                        handle = self.ik.transform_handles[side]
                        if handle.control is not None:
                            self.ref_ee_pos[side] = np.array(handle.control.position)
                            self.ref_ee_rot[side] = vtf.SO3(np.array(handle.control.wxyz))
                        continue

                    ref_vr_pos = self.ref_vr_pos[side]
                    ref_vr_rot = self.ref_vr_rot[side]
                    ref_ee_pos = self.ref_ee_pos[side]
                    ref_ee_rot = self.ref_ee_rot[side]
                    if ref_vr_pos is None or ref_vr_rot is None or ref_ee_pos is None or ref_ee_rot is None:
                        continue

                    delta_pos = (pos_robot - ref_vr_pos) * self.scale_factor
                    delta_rot = rot_robot @ ref_vr_rot.inverse()

                    new_pos = ref_ee_pos + delta_pos
                    new_rot = delta_rot @ ref_ee_rot

                    handle = self.ik.transform_handles[side]
                    if handle.control is not None:
                        handle.control.position = tuple(new_pos)  # type: ignore
                        handle.control.wxyz = new_rot.wxyz  # type: ignore

                    self.vr_status_handles[side].value = f"active | grip={grip_val:.2f}"
                else:
                    if was_active:
                        # Just deactivated — clear references
                        self.ref_vr_pos[side] = None
                        self.ref_vr_rot[side] = None
                        self.ref_ee_pos[side] = None
                        self.ref_ee_rot[side] = None
                    self.vr_status_handles[side].value = "inactive"

            time.sleep(0.01)

    def _update_visualization(self) -> None:
        """Update real robot state visualization in Viser."""
        while self.obs is None:
            time.sleep(0.025)
        while True:
            if self.bimanual:
                self.urdf_vis_right_real.update_cfg(np.flip(self.obs["right"]["joint_pos"][:6]))
            self.urdf_vis_left_real.update_cfg(np.flip(self.obs["left"]["joint_pos"][:6]))

            rgb_images = obs_get_rgb(self.obs)
            if rgb_images:
                for key in rgb_images:
                    if key not in self.viser_cam_img_handles:
                        self.viser_cam_img_handles[key] = self.viser_server.gui.add_image(rgb_images[key], label=key)
                    self.viser_cam_img_handles[key].image = resize_with_pad(rgb_images[key], 224, 224)

            time.sleep(0.02)

    def act(self, obs: Dict[str, Any]) -> Any:
        self.obs = deepcopy(obs)

        action: Dict[str, Dict[str, np.ndarray]] = {
            "left": {
                "pos": np.concatenate([np.flip(self.ik.joints["left"]), [self.gripper_value["left"]]]),
            }
        }
        if self.bimanual:
            action["right"] = {
                "pos": np.concatenate([np.flip(self.ik.joints["right"]), [self.gripper_value["right"]]]),
            }

        return action

    @remote(serialization_needed=True)
    def action_spec(self) -> Dict[str, Dict[str, Array]]:
        """Define the action specification."""
        action_spec: Dict[str, Dict[str, Array]] = {
            "left": {"pos": Array(shape=(7,), dtype=np.float32)},
        }
        if self.bimanual:
            action_spec["right"] = {"pos": Array(shape=(7,), dtype=np.float32)}
        return action_spec
