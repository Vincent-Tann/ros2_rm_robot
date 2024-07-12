import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        timer_period = 0.1  # 发布周期（秒）
        self.timer = self.create_timer(timer_period, self.publish_velocity_command)
    
    def publish_velocity_command(self):
        msg = Float64MultiArray()
        # 增大速度值
        msg.data = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
