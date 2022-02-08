import rclpy
import rclpy.node
import re
import random
import time
from typing import List


def get_uuids_fast(node: rclpy.node.Node) -> List[str]:
    """Get the turtlebot uuids"""
    res = node.get_topic_names_and_types()

    uuids = []
    for topic, _ in res:
        match = re.compile(r"/(turtlebot|robomaster|minicar|crazyflie)_\d+").match(
            topic
        )
        if match:
            uuid = match.group().replace("/", "")
            if uuid not in uuids:
                uuids.append(uuid)
    return uuids


def get_uuids(node: rclpy.node.Node = None, max_retries: int = 3) -> List[str]:
    """Get the turtlebot uuids, but with retries"""
    use_tmp_init = not rclpy.ok()
    use_tmp_node = not bool(node)
    if use_tmp_init:
        rclpy.init()
    if use_tmp_node:
        node = rclpy.create_node(f"temp{random.randint(0,1000)}")
    all_uuids = set()
    for i in range(max_retries):
        all_uuids.update(get_uuids_fast(node))
        time.sleep(0.25)
    if use_tmp_node:
        node.destroy_node()
    if use_tmp_init:
        rclpy.shutdown()
    return sorted(list(all_uuids))
