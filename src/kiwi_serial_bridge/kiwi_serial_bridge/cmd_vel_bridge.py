#!/usr/bin/env python3
"""Console entry point for the Kiwi serial bridge."""

from typing import List, Optional

import rclpy

from .bridge_node import CmdVelSerialBridge


def main(args: Optional[List[str]] = None) -> None:
    """Run the single Kiwi serial bridge ROS process."""
    rclpy.init(args=args)
    node: Optional[CmdVelSerialBridge] = None
    try:
        node = CmdVelSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
