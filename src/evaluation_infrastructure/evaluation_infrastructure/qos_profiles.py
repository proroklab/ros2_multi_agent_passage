import rclpy
from rclpy.qos import QoSProfile

mode_service_qos_profile = QoSProfile(
    history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=10,
    reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    liveliness=rclpy.qos.QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    deadline=rclpy.qos.Duration(),
    lifespan=rclpy.qos.Duration(),
    liveliness_lease_duration=rclpy.qos.Duration(),
)

initial_state_service_qos_profile = QoSProfile(
    history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=10,
    reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    liveliness=rclpy.qos.QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    deadline=rclpy.qos.Duration(),
    lifespan=rclpy.qos.Duration(),
    liveliness_lease_duration=rclpy.qos.Duration(),
)
