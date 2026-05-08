"""Print current follower arm joint positions from the ZMQ robot node."""

from dataclasses import dataclass

import numpy as np
import tyro

from teleop.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    tele_port: int = 6001
    hostname: str = "127.0.0.1"


def main(args: Args) -> None:
    robot = ZMQClientRobot(port=args.tele_port, host=args.hostname)
    obs = robot.get_observations()
    q = np.asarray(obs["joint_positions"])
    print("joint_positions (rad):", q)
    print("joint_positions (deg):", np.rad2deg(q))


if __name__ == "__main__":
    main(tyro.cli(Args))
