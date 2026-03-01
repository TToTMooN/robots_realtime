import threading
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np
import viser
import viser.extras
from dm_env.specs import Array
from loguru import logger

from robots_realtime.agents.agent import Agent
from robots_realtime.sensors.cameras.camera_utils import obs_get_rgb, resize_with_pad
from robots_realtime.utils.portal_utils import remote


def _create_ik_solver(solver_name: str, ik_params: Optional[Dict[str, Any]] = None, **kwargs):
    extra = ik_params or {}
    if solver_name == "pyroki":
        from robots_realtime.robots.inverse_kinematics.yam_pyroki import YamPyroki

        return YamPyroki(**kwargs)
    elif solver_name == "pink":
        from robots_realtime.robots.inverse_kinematics.yam_pink import YamPink

        return YamPink(**{**kwargs, **extra})
    else:
        raise ValueError(f"Unknown IK solver: {solver_name!r}. Choose 'pyroki' or 'pink'.")


class YamViserAgent(Agent):
    def __init__(
        self,
        bimanual: bool = False,
        right_arm_extrinsic: Optional[Dict[str, Any]] = None,
        ik_solver: str = "pink",
        ik_params: Optional[Dict[str, Any]] = None,
    ):
        self.right_arm_extrinsic = right_arm_extrinsic
        self.bimanual = bimanual
        if bimanual:
            assert right_arm_extrinsic is not None, "right_arm_extrinsic must be provided for bimanual robot"
        self.viser_server = viser.ViserServer()
        self.ik = _create_ik_solver(ik_solver, ik_params=ik_params, viser_server=self.viser_server, bimanual=bimanual)
        self.ik_thread = threading.Thread(target=self.ik.run)
        self.ik_thread.start()
        self.obs = None
        self._recording = False
        self._writers: Dict[str, cv2.VideoWriter] = {}
        self._record_dir: Optional[Path] = None
        self._record_lock = threading.Lock()
        self.real_vis_thread = threading.Thread(target=self._update_visualization, daemon=True)
        self.real_vis_thread.start()
        self._setup_visualization()

    def _setup_visualization(self):
        self.base_frame_left_real = self.viser_server.scene.add_frame("/base_left_real", show_axes=False)
        self.urdf_vis_left_real = viser.extras.ViserUrdf(
            self.viser_server,
            deepcopy(self.ik.urdf),
            root_node_name="/base_left_real",
            mesh_color_override=(0.8, 0.5, 0.5),
        )
        for mesh in self.urdf_vis_left_real._meshes:
            mesh.opacity = 0.25  # type: ignore
        self.left_gripper_slider_handle = self.viser_server.gui.add_slider(
            "Left Gripper", min=0.0, max=2.4, step=0.01, initial_value=0.0
        )

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
            self.right_gripper_slider_handle = self.viser_server.gui.add_slider(
                "Right Gripper", min=0.0, max=2.4, step=0.01, initial_value=0.0
            )

        self.viser_cam_img_handles = {}

        self.record_button = self.viser_server.gui.add_button("Start Recording", color="green")

        @self.record_button.on_click
        def _(_event: viser.GuiEvent) -> None:
            self._toggle_recording()

    def _toggle_recording(self) -> None:
        with self._record_lock:
            if not self._recording:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                self._record_dir = Path("recordings") / f"recording_{ts}"
                self._record_dir.mkdir(parents=True, exist_ok=True)

                rgb_images = obs_get_rgb(self.obs) if self.obs is not None else {}
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                for cam_name, img in rgb_images.items():
                    h, w = img.shape[:2]
                    path = str(self._record_dir / f"{cam_name}.mp4")
                    self._writers[cam_name] = cv2.VideoWriter(path, fourcc, 30, (w, h))

                self._recording = True
                self.record_button.name = "Stop Recording"
                self.record_button.color = "red"
                logger.info(f"Recording started -> {self._record_dir}")
            else:
                for w in self._writers.values():
                    w.release()
                logger.info(f"Recording saved to {self._record_dir}")
                self._writers.clear()
                self._record_dir = None
                self._recording = False
                self.record_button.name = "Start Recording"
                self.record_button.color = "green"

    def _update_visualization(self):
        while self.obs is None:
            time.sleep(0.025)
        while True:
            if self.bimanual:
                self.urdf_vis_right_real.update_cfg(np.flip(self.obs["right"]["joint_pos"]))
            self.urdf_vis_left_real.update_cfg(np.flip(self.obs["left"]["joint_pos"]))

            rgb_images = obs_get_rgb(self.obs)
            if rgb_images:
                for key in rgb_images.keys():
                    if key not in self.viser_cam_img_handles.keys():
                        self.viser_cam_img_handles[key] = self.viser_server.gui.add_image(rgb_images[key], label=key)
                    self.viser_cam_img_handles[key].image = resize_with_pad(rgb_images[key], 224, 224)

                with self._record_lock:
                    if self._recording:
                        for cam_name, img in rgb_images.items():
                            writer = self._writers.get(cam_name)
                            if writer is None:
                                h, w = img.shape[:2]
                                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                                path = str(self._record_dir / f"{cam_name}.mp4")
                                writer = cv2.VideoWriter(path, fourcc, 30, (w, h))
                                self._writers[cam_name] = writer
                            writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

            time.sleep(0.02)

    def act(self, obs: Dict[str, Any]) -> Any:
        self.obs = deepcopy(obs)

        action = {
            "left": {
                "pos": np.concatenate([np.flip(self.ik.joints["left"]), [self.left_gripper_slider_handle.value]]),
            }
        }
        if self.bimanual:
            assert self.ik.joints.keys() == {"left", "right"}, (
                "bimanual mode must have both left and right joint ik solved"
            )
            action["right"] = {
                "pos": np.concatenate([np.flip(self.ik.joints["right"]), [self.right_gripper_slider_handle.value]]),
            }

        return action

    def close(self) -> None:
        with self._record_lock:
            if self._recording:
                for w in self._writers.values():
                    w.release()
                logger.info(f"Recording saved to {self._record_dir}")
                self._writers.clear()
                self._recording = False

    @remote(serialization_needed=True)
    def action_spec(self) -> Dict[str, Dict[str, Array]]:
        """Define the action specification."""
        action_spec = {
            "left": {"pos": Array(shape=(7,), dtype=np.float32)},
        }
        if self.bimanual:
            action_spec["right"] = {"pos": Array(shape=(7,), dtype=np.float32)}
        return action_spec
