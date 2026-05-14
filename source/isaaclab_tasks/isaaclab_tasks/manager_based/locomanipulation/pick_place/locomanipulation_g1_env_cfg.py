# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab_teleop import IsaacTeleopCfg, XrAnchorRotationMode, XrCfg

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from isaaclab_tasks.manager_based.locomanipulation.pick_place import mdp as locomanip_mdp
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.action_cfg import AgileBasedLowerBodyActionCfg
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.agile_locomotion_observation_cfg import (
    AgileTeacherPolicyObservationsCfg,
)
from isaaclab_tasks.manager_based.manipulation.pick_place import mdp as manip_mdp

from isaaclab_assets.robots.unitree import G1_29DOF_CFG

from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.pink_controller_cfg import (  # isort: skip
    G1_UPPER_BODY_IK_ACTION_CFG,
)


def _build_g1_locomanipulation_pipeline():
    """Build an IsaacTeleop retargeting pipeline for G1 locomanipulation teleoperation.

    Uses optical hand tracking (HandsSource) for wrist pose control via Se3AbsRetargeters,
    pinch-based finger closure detection via HandTrackingFingerRetargeter for hand joint
    control, and a fixed locomotion command (standing still) since hand tracking provides
    no thumbstick input.

    Returns:
        OutputCombiner with a single "action" output containing the flattened
        32D action tensor: [left_wrist(7), right_wrist(7), hand_joints(14), locomotion(4)].
    """
    import numpy as np

    from isaacteleop.retargeters import (
        LocomotionFixedRootCmdRetargeter,
        LocomotionFixedRootCmdRetargeterConfig,
        Se3AbsRetargeter,
        Se3RetargeterConfig,
        TensorReorderer,
    )
    from isaacteleop.retargeting_engine.deviceio_source_nodes import HandsSource
    from isaacteleop.retargeting_engine.interface import BaseRetargeter, OutputCombiner, ValueInput
    from isaacteleop.retargeting_engine.interface.retargeter_core_types import RetargeterIO, RetargeterIOType
    from isaacteleop.retargeting_engine.interface.tensor_group_type import OptionalType
    from isaacteleop.retargeting_engine.tensor_types import (
        HandInput,
        HandInputIndex,
        HandJointIndex,
        RobotHandJoints,
        TransformMatrix,
    )

    class HandTrackingFingerRetargeter(BaseRetargeter):
        """Maps optical hand tracking pinch/curl to G1 TriHand 7-DOF joint angles.

        Measures thumb-to-index-tip distance for pinch (trigger analog) and
        middle-tip-to-wrist distance for curl (squeeze analog), then applies
        the same joint mapping as TriHandMotionControllerRetargeter.
        """

        # Pinch distance thresholds (meters)
        PINCH_OPEN_M: float = 0.08
        PINCH_CLOSE_M: float = 0.02
        # Middle-tip to wrist distance thresholds (meters)
        CURL_OPEN_M: float = 0.20
        CURL_CLOSE_M: float = 0.10
        # Joint scale factors (mirror TriHandMotionControllerRetargeter)
        THUMB_PROXIMAL_SCALE: float = 0.4
        THUMB_DISTAL_SCALE: float = 0.7
        THUMB_ROTATION_SCALE: float = 0.5

        def __init__(self, hand_side: str, hand_joint_names: list, name: str) -> None:
            self._hand_side = hand_side.lower()
            self._hand_joint_names = hand_joint_names
            self._is_left = self._hand_side == "left"
            super().__init__(name=name)

        def input_spec(self) -> RetargeterIOType:
            return {f"hand_{self._hand_side}": OptionalType(HandInput())}

        def output_spec(self) -> RetargeterIOType:
            return {"hand_joints": RobotHandJoints(f"hand_joints_{self._hand_side}", self._hand_joint_names)}

        def _compute_fn(self, inputs: RetargeterIO, outputs: RetargeterIO, context) -> None:
            output_group = outputs["hand_joints"]
            hand_group = inputs[f"hand_{self._hand_side}"]

            if hand_group.is_none:
                for i in range(len(self._hand_joint_names)):
                    output_group[i] = 0.0
                return

            joint_positions = np.from_dlpack(hand_group[HandInputIndex.JOINT_POSITIONS])
            joint_valid = np.from_dlpack(hand_group[HandInputIndex.JOINT_VALID])

            # Pinch (thumb tip ↔ index tip distance) → trigger analog [0, 1]
            trigger = 0.0
            if joint_valid[HandJointIndex.THUMB_TIP] and joint_valid[HandJointIndex.INDEX_TIP]:
                dist = float(np.linalg.norm(
                    joint_positions[HandJointIndex.THUMB_TIP] - joint_positions[HandJointIndex.INDEX_TIP]
                ))
                trigger = 1.0 - float(np.clip(
                    (dist - self.PINCH_CLOSE_M) / (self.PINCH_OPEN_M - self.PINCH_CLOSE_M), 0.0, 1.0
                ))

            # Curl (middle tip ↔ wrist distance) → squeeze analog [0, 1]
            squeeze = 0.0
            if joint_valid[HandJointIndex.MIDDLE_TIP] and joint_valid[HandJointIndex.WRIST]:
                mid_dist = float(np.linalg.norm(
                    joint_positions[HandJointIndex.MIDDLE_TIP] - joint_positions[HandJointIndex.WRIST]
                ))
                squeeze = 1.0 - float(np.clip(
                    (mid_dist - self.CURL_CLOSE_M) / (self.CURL_OPEN_M - self.CURL_CLOSE_M), 0.0, 1.0
                ))

            # Same joint mapping as TriHandMotionControllerRetargeter
            hand_joints = np.zeros(7, dtype=np.float32)
            thumb_button = max(trigger, squeeze)
            thumb_rotation = self.THUMB_ROTATION_SCALE * trigger - self.THUMB_ROTATION_SCALE * squeeze
            if not self._is_left:
                thumb_rotation = -thumb_rotation
            hand_joints[0] = thumb_rotation
            hand_joints[1] = -thumb_button * self.THUMB_PROXIMAL_SCALE
            hand_joints[2] = -thumb_button * self.THUMB_DISTAL_SCALE
            hand_joints[3] = trigger
            hand_joints[4] = trigger
            hand_joints[5] = squeeze
            hand_joints[6] = squeeze

            if self._is_left:
                hand_joints = -hand_joints

            for i in range(min(len(self._hand_joint_names), 7)):
                output_group[i] = float(hand_joints[i])

    # Create input sources (hand trackers are auto-discovered from pipeline)
    hands = HandsSource(name="hands")

    # External input: world-to-anchor 4x4 transform matrix provided by IsaacTeleopDevice
    transform_input = ValueInput("world_T_anchor", TransformMatrix())

    # Apply the coordinate-frame transform to hand joint poses so that
    # downstream retargeters receive data in the simulation world frame.
    transformed_hands = hands.transformed(transform_input.output(ValueInput.VALUE))

    # -------------------------------------------------------------------------
    # SE3 Absolute Pose Retargeters (left and right wrists from hand tracking)
    # -------------------------------------------------------------------------
    left_se3_cfg = Se3RetargeterConfig(
        input_device=HandsSource.LEFT,
        zero_out_xy_rotation=False,
        use_wrist_rotation=True,
        use_wrist_position=True,
        target_offset_roll=45.0,
        target_offset_pitch=180.0,
        target_offset_yaw=-90.0,
    )
    left_se3 = Se3AbsRetargeter(left_se3_cfg, name="left_ee_pose")
    connected_left_se3 = left_se3.connect(
        {
            HandsSource.LEFT: transformed_hands.output(HandsSource.LEFT),
        }
    )

    right_se3_cfg = Se3RetargeterConfig(
        input_device=HandsSource.RIGHT,
        zero_out_xy_rotation=False,
        use_wrist_rotation=True,
        use_wrist_position=True,
        target_offset_roll=-135.0,
        target_offset_pitch=0.0,
        target_offset_yaw=90.0,
    )
    right_se3 = Se3AbsRetargeter(right_se3_cfg, name="right_ee_pose")
    connected_right_se3 = right_se3.connect(
        {
            HandsSource.RIGHT: transformed_hands.output(HandsSource.RIGHT),
        }
    )

    # -------------------------------------------------------------------------
    # Hand Tracking Finger Retargeters (pinch/curl → G1 TriHand 7-DOF angles)
    # -------------------------------------------------------------------------
    # Joint names matching G1 TriHand 7-DOF output order:
    #   [thumb_rotation, thumb_proximal, thumb_distal,
    #    index_proximal, index_distal, middle_proximal, middle_distal]
    hand_joint_names = [
        "thumb_rotation",
        "thumb_proximal",
        "thumb_distal",
        "index_proximal",
        "index_distal",
        "middle_proximal",
        "middle_distal",
    ]

    left_finger = HandTrackingFingerRetargeter(
        hand_side="left",
        hand_joint_names=hand_joint_names,
        name="finger_left",
    )
    connected_left_finger = left_finger.connect(
        {HandsSource.LEFT: transformed_hands.output(HandsSource.LEFT)}
    )

    right_finger = HandTrackingFingerRetargeter(
        hand_side="right",
        hand_joint_names=hand_joint_names,
        name="finger_right",
    )
    connected_right_finger = right_finger.connect(
        {HandsSource.RIGHT: transformed_hands.output(HandsSource.RIGHT)}
    )

    # -------------------------------------------------------------------------
    # Locomotion Root Command (fixed standing — no thumbstick with hand tracking)
    # -------------------------------------------------------------------------
    locomotion_cfg = LocomotionFixedRootCmdRetargeterConfig(hip_height=0.72)
    locomotion = LocomotionFixedRootCmdRetargeter(locomotion_cfg, name="locomotion")
    connected_locomotion = locomotion.connect({})

    # -------------------------------------------------------------------------
    # TensorReorderer: flatten into a 32D action tensor
    # -------------------------------------------------------------------------
    # Se3AbsRetargeter outputs 7D arrays: [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
    left_ee_elements = ["l_pos_x", "l_pos_y", "l_pos_z", "l_quat_x", "l_quat_y", "l_quat_z", "l_quat_w"]
    right_ee_elements = ["r_pos_x", "r_pos_y", "r_pos_z", "r_quat_x", "r_quat_y", "r_quat_z", "r_quat_w"]

    # Hand finger retargeter outputs 7 scalars per hand (positionally mapped):
    #   [thumb_rotation, thumb_proximal, thumb_distal,
    #    index_proximal, index_distal, middle_proximal, middle_distal]
    left_hand_elements = [
        "l_thumb_rotation",
        "l_thumb_proximal",
        "l_thumb_distal",
        "l_index_proximal",
        "l_index_distal",
        "l_middle_proximal",
        "l_middle_distal",
    ]
    right_hand_elements = [
        "r_thumb_rotation",
        "r_thumb_proximal",
        "r_thumb_distal",
        "r_index_proximal",
        "r_index_distal",
        "r_middle_proximal",
        "r_middle_distal",
    ]

    # Locomotion outputs 4D array: [vel_x, vel_y, rot_vel_z, hip_height]
    locomotion_elements = ["loco_vel_x", "loco_vel_y", "loco_rot_vel_z", "loco_hip_height"]

    # Output order must match the action space layout expected by the environment:
    #   [left_wrist(7), right_wrist(7), hand_joints(14), locomotion(4)]
    # Hand joints follow hand_joint_names order from G1_UPPER_BODY_IK_ACTION_CFG.
    # Locomotion (4D) is consumed by AgileBasedLowerBodyAction.
    output_order = (
        left_ee_elements
        + right_ee_elements
        + [
            # hand_joint_names indices 0-5  (proximal / 0-joints)
            "l_index_proximal",
            "l_middle_proximal",
            "l_thumb_rotation",
            "r_index_proximal",
            "r_middle_proximal",
            "r_thumb_rotation",
            # hand_joint_names indices 6-11 (distal / 1-joints)
            "l_index_distal",
            "l_middle_distal",
            "l_thumb_proximal",
            "r_index_distal",
            "r_middle_distal",
            "r_thumb_proximal",
            # hand_joint_names indices 12-13 (thumb tip / 2-joints)
            "l_thumb_distal",
            "r_thumb_distal",
        ]
        + locomotion_elements
    )

    reorderer = TensorReorderer(
        input_config={
            "left_ee_pose": left_ee_elements,
            "right_ee_pose": right_ee_elements,
            "left_hand_joints": left_hand_elements,
            "right_hand_joints": right_hand_elements,
            "locomotion": locomotion_elements,
        },
        output_order=output_order,
        name="action_reorderer",
        input_types={
            "left_ee_pose": "array",
            "right_ee_pose": "array",
            "left_hand_joints": "scalar",
            "right_hand_joints": "scalar",
            "locomotion": "array",
        },
    )
    connected_reorderer = reorderer.connect(
        {
            "left_ee_pose": connected_left_se3.output("ee_pose"),
            "right_ee_pose": connected_right_se3.output("ee_pose"),
            "left_hand_joints": connected_left_finger.output("hand_joints"),
            "right_hand_joints": connected_right_finger.output("hand_joints"),
            "locomotion": connected_locomotion.output("root_command"),
        }
    )

    return OutputCombiner({"action": connected_reorderer.output("output")})


##
# Scene definition
##
@configclass
class LocomanipulationG1SceneCfg(InteractiveSceneCfg):
    """Scene configuration for locomanipulation environment with G1 robot.

    This configuration sets up the G1 humanoid robot for locomanipulation tasks,
    allowing both locomotion and manipulation capabilities. The robot can move its
    base and use its arms for manipulation tasks.
    """

    # Table
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 0.55, -0.3], rot=[0.0, 0.0, 0.0, 1.0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/packing_table.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.35, 0.45, 0.6996], rot=[0, 0, 0, 1]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Mimic/pick_place_task/pick_place_assets/steering_wheel.usd",
            scale=(0.75, 0.75, 0.75),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        ),
    )

    # Humanoid robot w/ arms higher
    robot: ArticulationCfg = G1_29DOF_CFG

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(),
    )

    # Lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    upper_body_ik = G1_UPPER_BODY_IK_ACTION_CFG

    lower_body_joint_pos = AgileBasedLowerBodyActionCfg(
        asset_name="robot",
        joint_names=[
            ".*_hip_.*_joint",
            ".*_knee_joint",
            ".*_ankle_.*_joint",
        ],
        policy_output_scale=0.25,
        obs_group_name="lower_body_policy",  # need to be the same name as the on in ObservationCfg
        policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt",
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP.
    This class is required by the environment configuration but not used in this implementation
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group with state values."""

        actions = ObsTerm(func=manip_mdp.last_action)
        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_root_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})
        object_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("object")})
        object_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object")})
        robot_links_state = ObsTerm(func=manip_mdp.get_all_robot_link_state)

        left_eef_pos = ObsTerm(func=manip_mdp.get_eef_pos, params={"link_name": "left_wrist_yaw_link"})
        left_eef_quat = ObsTerm(func=manip_mdp.get_eef_quat, params={"link_name": "left_wrist_yaw_link"})
        right_eef_pos = ObsTerm(func=manip_mdp.get_eef_pos, params={"link_name": "right_wrist_yaw_link"})
        right_eef_quat = ObsTerm(func=manip_mdp.get_eef_quat, params={"link_name": "right_wrist_yaw_link"})

        hand_joint_state = ObsTerm(func=manip_mdp.get_robot_joint_state, params={"joint_names": [".*_hand.*"]})

        object = ObsTerm(
            func=manip_mdp.object_obs,
            params={"left_eef_link_name": "left_wrist_yaw_link", "right_eef_link_name": "right_wrist_yaw_link"},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    lower_body_policy: AgileTeacherPolicyObservationsCfg = AgileTeacherPolicyObservationsCfg()


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=locomanip_mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("object")}
    )

    object_too_far = DoneTerm(
        func=locomanip_mdp.object_too_far_from_robot,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "object_cfg": SceneEntityCfg("object"),
            "max_distance": 1.0,
        },
    )

    success = DoneTerm(
        func=manip_mdp.task_done_pick_place,
        params={
            "task_link_name": "right_wrist_yaw_link",
        },
    )


##
# MDP settings
##


@configclass
class LocomanipulationG1EnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 locomanipulation environment.

    This environment is designed for locomanipulation tasks where the G1 humanoid robot
    can perform both locomotion and manipulation simultaneously. The robot can move its
    base and use its arms for manipulation tasks, enabling complex mobile manipulation
    behaviors.
    """

    # Scene settings
    scene: LocomanipulationG1SceneCfg = LocomanipulationG1SceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    # MDP settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands = None
    terminations: TerminationsCfg = TerminationsCfg()

    # Unused managers
    rewards = None
    curriculum = None

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 1 / 200  # 200Hz
        self.sim.render_interval = 2

        # Set the URDF path for the IK controller. Path resolution (Nucleus → local) happens at runtime.
        self.actions.upper_body_ik.controller.urdf_path = f"{ISAACLAB_NUCLEUS_DIR}/Controllers/LocomanipulationAssets/unitree_g1_kinematics_asset/g1_29dof_with_hand_only_kinematics.urdf"  # noqa: E501

        self.xr = XrCfg(
            anchor_pos=(0.0, 0.0, -0.95),
            anchor_rot=(0.0, 0.0, 0.0, 1.0),
        )
        self.xr.anchor_prim_path = "/World/envs/env_0/Robot/pelvis"
        self.xr.fixed_anchor_height = True
        self.xr.anchor_rotation_mode = XrAnchorRotationMode.FOLLOW_PRIM_SMOOTHED

        self.isaac_teleop = IsaacTeleopCfg(
            pipeline_builder=_build_g1_locomanipulation_pipeline,
            sim_device=self.sim.device,
            xr_cfg=self.xr,
        )
