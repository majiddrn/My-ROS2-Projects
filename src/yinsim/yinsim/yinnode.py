import rclpy
from rclpy.node import Node
from yinyang_interfaces.srv import Message
from rclpy.action import ActionServer
from std_msgs.msg import String
from functools import partial
from yinyang_interfaces.action import MAction
from rclpy.parameter import Parameter

import time

class Yinnode(Node):

    def __init__(self):
        super().__init__('yinnode')
        self.conversation_topic = self.create_publisher(String, '/conversation', 50)
        self.client_yang = self.create_client(Message, 'yang_service')
        self.service = self.create_service(Message, 'yin_service', self.service_callback)
        self.declare_parameter('shout', 'false')
        self.declare_parameter('opacity', 100)

        self.logger = self.get_logger()

        self.action_server = ActionServer(self,
                                          MAction,
                                          'yin_action_server',
                                          self.action_server_callback)

        self.messages = ["I am Yin, some mistake me for an actual material entity but I am more of a concept",
        "Interesting Yang, so one could say, in a philosophical sense, we are two polar elements",
        "We, Yang, are therefore the balancing powers in the universe.",
        "Difficult and easy complete each other.",
        "Long and short show each other.",
        "Noise and sound harmonize each other.",
        "You shine your light."]

        self.m_i = 1

        checksum_m = 0

        for c in self.messages[0]:
            checksum_m += int(ord(c))

        req = Message.Request()
        req.name = 'Yin'
        req.msg = self.messages[0]
        req.length = len(self.messages[0])
        req.checksum = checksum_m

        # self.logger.info(f"m_i____1:{self.m_i}")


        while not self.client_yang.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # self.logger.info(f"m_i____2:{self.m_i}")
        
        future = self.client_yang.call_async(req)

        future.add_done_callback(partial(self.res_callback))

        # self.logger.info(f"m_i____3:{self.m_i}")

    def action_server_callback(self, goal_handle):
        feedback_msg = MAction.Feedback()
        opacity = self.get_parameter("opacity").get_parameter_value().integer_value

        feedback_msg.opacity = opacity
        goal_handle.publish_feedback(feedback_msg)

        while opacity > 0:
            opacity -= 1
            feedback_msg.opacity = opacity
            param = Parameter('opacity', Parameter.Type.INTEGER, 0)
            self.set_parameters([param])
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.01)

        goal_handle.succeed()

        result = MAction.Result()
        result.res = "farewell"
        
        return result

    def res_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
        else:
            # self.get_logger().info('Result got: %d' % (response.checksum_r,))
            self.logger.info(f"result: {future.result()}, ok: {rclpy.ok()}, done: {future.done()}, response: {response.checksum_r}")
    
    def service_callback(self, request, response):
        # conversation_val = request.name + " said: " + request.msg
        # conversation_val += ", " + str(request.length) + ", " + str(request.checksum)

        s = String()
        s.data = "Yang said: " + request.msg + ", " + str(request.length) + ", " + str(request.checksum)

        self.conversation_topic.publish(s)
        self.logger.info(f"data:{s.data}")

        checksum_y = 0

        for c in request.msg:
            checksum_y += int(ord(c))

        response.checksum_r = checksum_y

        if self.m_i <= 6:

            response_call = Message.Request()
            response_call.name = 'Yin'
            response_call.msg = self.messages[self.m_i]
            response_call.length = len(self.messages[self.m_i])
            response_call.checksum = checksum_y
            self.m_i += 1

            future = self.client_yang.call_async(response_call)
            future.add_done_callback(partial(self.res_callback))

            # self.logger.info(f"m_i:{self.m_i}")

        return response

        

def main():
    rclpy.init()
    yin = Yinnode()
    rclpy.spin(yin)
    yin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
