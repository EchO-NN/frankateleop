import datetime
import glob
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

from teleop.agents.agent import BimanualAgent, DummyAgent
from teleop.agents.teleop_agent import TeleopAgent
from teleop.data_utils.format_obs import save_frame
from teleop.env import RobotEnv
from teleop.robots.robot import PrintRobot
from teleop.zmq_core.robot_node import ZMQClientRobot

# Reset-on-start: smaller rad per interpolation step => more steps => slower motion.
# Defaults were 0.01 rad and 100 steps; halving step size ≈ half reset speed.
_RESET_TRAVEL_STEP_RAD = 0.005
_RESET_TRAVEL_MAX_STEPS = 200


def _command_gripper_open_after_reset(env: RobotEnv) -> None:
    """Open gripper(s) after arm reset. fr3: commanded last dim 0=open, 1=closed."""
    obs = env.get_obs()
    q = np.asarray(obs["joint_positions"], dtype=np.float64).copy()
    n = int(q.shape[0])
    if n == 8:
        q[-1] = 0.0
    elif n == 16:
        q[7] = 0.0
        q[15] = 0.0
    elif n >= 8 and n % 8 == 0:
        for i in range(7, n, 8):
            q[i] = 0.0
    else:
        return
    env.step(q)


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    tele_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    teleop_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False
    reset_on_start: bool = True


def main(args):
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.tele_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    if args.bimanual:
        if args.agent == "teleop":
            # dynamixel control box port map (to distinguish left and right teleop)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0"
            left_agent = TeleopAgent(port=left)
            right_agent = TeleopAgent(port=right)
            agent = BimanualAgent(left_agent, right_agent)
        elif args.agent == "quest":
            from teleop.agents.quest_agent import SingleArmQuestAgent

            left_agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
            right_agent = SingleArmQuestAgent(
                robot_type=args.robot_type, which_hand="r"
            )
            agent = BimanualAgent(left_agent, right_agent)
            # raise NotImplementedError
        elif args.agent == "spacemouse":
            from teleop.agents.spacemouse_agent import SpacemouseAgent

            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            left_agent = SpacemouseAgent(
                robot_type=args.robot_type, device_path=left_path, verbose=args.verbose
            )
            right_agent = SpacemouseAgent(
                robot_type=args.robot_type,
                device_path=right_path,
                verbose=args.verbose,
                invert_button=True,
            )
            agent = BimanualAgent(left_agent, right_agent)
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([0, -90, -90, -90, 90, 0, 0])
        # Right arm home (7 DoF, rad) — snapshot from read_joint_positions / follower state.
        reset_joints_right = np.array(
            [
                -0.11559626,
                0.0642503,
                0.11347504,
                -1.46301174,
                -0.00683021,
                1.47942591,
                0.71888214,
            ],
            dtype=np.float64,
        )
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        if args.reset_on_start:
            max_delta = (np.abs(curr_joints - reset_joints)).max()
            steps = min(
                int(max_delta / _RESET_TRAVEL_STEP_RAD), _RESET_TRAVEL_MAX_STEPS
            )

            for jnt in np.linspace(curr_joints, reset_joints, steps):
                env.step(jnt)
            _command_gripper_open_after_reset(env)
    else:
        if args.agent == "teleop":
            teleop_port = args.teleop_port
            if teleop_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    teleop_port = usb_ports[0]
                    print(f"using port {teleop_port}")
                else:
                    raise ValueError(
                        "No teleop port found, please specify one or plug in teleop"
                    )
            if args.start_joints is None:
                # Default right-arm reset pose (rad); use --start-joints for left / other mounts.
                reset_joints = np.array(
                    [
                        -0.11559626,
                        0.0642503,
                        0.11347504,
                        -1.46301174,
                        -0.00683021,
                        1.47942591,
                        0.71888214,
                    ],
                    dtype=np.float64,
                )
            else:
                reset_joints = args.start_joints
                reset_joints = np.array(reset_joints)
            agent = TeleopAgent(port=teleop_port, start_joints=args.start_joints)
            curr_joints = env.get_obs()["joint_positions"]
            curr_joints = np.array(curr_joints)

            # Some envs append a gripper dimension to the 7-DoF arm joints.
            # Reuse the leader's current gripper command so we can still drive
            # the robot to the standard arm reset pose before teleop starts.
            if reset_joints.shape[0] + 1 == curr_joints.shape[0]:
                leader_joints = np.array(agent.act({"joint_positions": curr_joints}))
                reset_joints = np.concatenate([reset_joints, leader_joints[-1:]])

            if args.reset_on_start and reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(
                    int(max_delta / _RESET_TRAVEL_STEP_RAD), _RESET_TRAVEL_MAX_STEPS
                )

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
                _command_gripper_open_after_reset(env)
        elif args.agent == "quest":
            from teleop.agents.quest_agent import SingleArmQuestAgent

            agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
        elif args.agent == "spacemouse":
            from teleop.agents.spacemouse_agent import SpacemouseAgent

            agent = SpacemouseAgent(robot_type=args.robot_type, verbose=args.verbose)
        elif args.agent == "dummy" or args.agent == "none":
            agent = DummyAgent(num_dofs=robot_client.num_dofs())
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")

    # going to start position
    print("Going to start position")
    start_pos = agent.act(env.get_obs())
    obs = env.get_obs()
    joints = obs["joint_positions"]
    joints = np.array(joints)

    abs_deltas = np.abs(start_pos - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    max_joint_delta = 0.8
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        id_mask = abs_deltas > max_joint_delta
        print()

        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            start_pos[id_mask],
            joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    max_delta = 0.05
    for _ in range(25):
        obs = env.get_obs()
        command_joints = agent.act(obs)
        current_joints = obs["joint_positions"]
        delta = command_joints - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        env.step(current_joints + delta)

    obs = env.get_obs()
    joints = obs["joint_positions"]
    action = agent.act(obs)
    action_delta = action - joints
    max_action_delta = 0.5
    if (np.abs(action_delta) > max_action_delta).any():
        print("Action is too big")

        joint_index = np.where(np.abs(action_delta) > max_action_delta)[0]
        for j in joint_index:
            print(
                f"Joint [{j}], leader: {action[j]:.3f}, follower: {joints[j]:.3f}, diff: {action_delta[j]:.3f}"
            )
        exit()

    if args.use_save_interface:
        from teleop.data_utils.keyboard_interface import KBReset

        kb_interface = KBReset()

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))

    save_path = None
    start_time = time.time()
    while True:
        num = time.time() - start_time
        message = f"\rTime passed: {round(num, 2)}          "
        print_color(
            message,
            color="white",
            attrs=("bold",),
            end="",
            flush=True,
        )
        action = agent.act(obs)
        dt = datetime.datetime.now()
        if args.use_save_interface:
            state = kb_interface.update()
            if state == "start":
                dt_time = datetime.datetime.now()
                save_path = (
                    Path(args.data_dir).expanduser()
                    / args.agent
                    / dt_time.strftime("%m%d_%H%M%S")
                )
                save_path.mkdir(parents=True, exist_ok=True)
                print(f"Saving to {save_path}")
            elif state == "save":
                assert save_path is not None, "something went wrong"
                save_frame(save_path, dt, obs, action)
            elif state == "normal":
                save_path = None
            else:
                raise ValueError(f"Invalid state {state}")
        obs = env.step(action)


if __name__ == "__main__":
    main(tyro.cli(Args))
