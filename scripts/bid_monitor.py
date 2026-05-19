import rclpy
from rclpy.node import Node
from rmf_task_msgs.msg import BidNotice, BidResponse, DispatchCommand, DispatchAck

class BidMonitor(Node):
    def __init__(self):
        super().__init__('bid_monitor')
        self.sub_notice = self.create_subscription(BidNotice, '/rmf_task/bid_notice', self.notice_cb, 10)
        self.sub_response = self.create_subscription(BidResponse, '/rmf_task/bid_response', self.response_cb, 10)
        self.sub_dispatch = self.create_subscription(DispatchCommand, '/rmf_task/dispatch_request', self.dispatch_cb, 10)
        self.sub_ack = self.create_subscription(DispatchAck, '/rmf_task/dispatch_ack', self.ack_cb, 10)
        self.get_logger().info('Enhanced Monitor started...')

    def notice_cb(self, msg):
        self.get_logger().info(f'--- BID NOTICE: {msg.task_id}')

    def response_cb(self, msg):
        fleet = msg.proposal.fleet_name if msg.has_proposal else "N/A"
        robot = msg.proposal.expected_robot_name if msg.has_proposal else "N/A"
        finish = msg.proposal.finish_time.sec if msg.has_proposal else "N/A"
        self.get_logger().info(f'<<< BID RESPONSE: task={msg.task_id}, fleet={fleet}, robot={robot}, finish={finish}')

    def dispatch_cb(self, msg):
        self.get_logger().info(f'>>> DISPATCH COMMAND: task={msg.task_id}, fleet={msg.fleet_name}')

    def ack_cb(self, msg):
        self.get_logger().info(f'!!! DISPATCH ACK: success={not msg.errors}')

def main():
    rclpy.init()
    node = BidMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
