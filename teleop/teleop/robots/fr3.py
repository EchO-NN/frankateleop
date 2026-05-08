import time
import torch
from typing import Dict
from typing import Optional
import numpy as np
from teleop.robots.robot import Robot

MAX_OPEN = 0.08
GRIPPER_CLOSED_WIDTH = 0.005  # 不要给 0，否则 libfranka 会跑完整个 Unsuccessful 超时
GRIPPER_SWITCH_THRESHOLD = 0.5
GRIPPER_SPEED = 0.1
GRIPPER_FORCE = 40.0


class fr3Robot(Robot):
    """A class representing a UR robot."""

    def __init__(
            self, 
            robot_ip: str = "192.168.1.100", 
            franka_port: int=50051, 
            frankahand_port: int = 50053,
            joint_positions_desired: Optional[torch.Tensor] = None,
            ):
            
        from polymetis import GripperInterface, RobotInterface
        print(f"Connecting to robot at IP: {robot_ip}")

        self.robot = RobotInterface(
            ip_address=robot_ip,
            port=franka_port,
        )
        self.gripper = GripperInterface(
            ip_address=robot_ip,
            port=frankahand_port,
        )
        self._last_gripper_command_closed = None
        self.joint_positions_desired = None
        if joint_positions_desired is not None:
            if joint_positions_desired.shape != (7,):
                raise ValueError(f"Franka requires 7 joints params, current input is: {joint_positions_desired.shape}")
            # Only keep the requested startup pose for reference; do not move the arm
            # during server initialization.
            self.joint_positions_desired = joint_positions_desired
            
        self.robot.start_joint_impedance()
        self.gripper.goto(width=MAX_OPEN, speed=GRIPPER_SPEED, force=GRIPPER_FORCE)
        self._last_gripper_command_closed = False
        time.sleep(1)

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        return 8

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.robot.get_joint_positions()
        gripper_pos = self.gripper.get_state()
        # Match the teleop hand convention: 0=open, 1=closed.
        gripper_closed = np.clip(1 - gripper_pos.width / MAX_OPEN, 0.0, 1.0)
        pos = np.append(robot_joints, gripper_closed)
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        import torch

        self.robot.update_desired_joint_positions(torch.tensor(joint_state[:-1]))
        gripper_command_closed = joint_state[-1] >= GRIPPER_SWITCH_THRESHOLD
        if gripper_command_closed == self._last_gripper_command_closed:
            return

        width = GRIPPER_CLOSED_WIDTH if gripper_command_closed else MAX_OPEN
        print(
            f"[FR3_GRIPPER_CMD] t={time.time():.6f} "
            f"closed={gripper_command_closed} width={width:.3f}",
            flush=True,
        )
        if gripper_command_closed:
            # 闭合用 grasp，达到容差/抓到物体后会持续保持夹紧力
            self.gripper.grasp(
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                grasp_width=GRIPPER_CLOSED_WIDTH,
                epsilon_inner=MAX_OPEN,
                epsilon_outer=MAX_OPEN,
                blocking=False,
            )
        else:
            self.gripper.goto(
                width=width,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=False,
            )
        self._last_gripper_command_closed = gripper_command_closed

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot = fr3Robot()
    current_joints = robot.get_joint_state()
    # move a small delta 0.1 rad
    move_joints = current_joints + 0.05
    # make last joint (gripper) closed
    move_joints[-1] = 0.5
    time.sleep(1)
    m = 0.09
    robot.gripper.goto(1 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.05 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.1 * m, speed=255, force=255)
    time.sleep(1)


if __name__ == "__main__":
    main()
