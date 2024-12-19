#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import PsController
import serial
import struct

class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")
        
        # subscriber for controller topic
        self.subscriber_ = self.create_subscription(PsController, "ps_controller", self.sub_callback, 10)
        self.read_in_ = bytearray() # Create array to hold data to be written
        self.goals_ = [0.33, 2.0, 0.0, 0.0, 0.0] # create array of goals to send
        write_byte = "w"
        self.write_buf_ = bytearray(struct.pack("fffff", self.goals_[0],self.goals_[1],self.goals_[2],self.goals_[3],self.goals_[4]))

        # Initialize serial comms using default arduino settings and 9600 baud
        self.ser_handle = serial.Serial( 
            port="/dev/ttyACM0",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=5.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
     

        self.get_logger().info("arm control node started")
        print("Byte count:", len(self.write_buf_))
        self.send_it()

    # subscriber callback
    def sub_callback(self, msg):
        pass

    def send_it(self):
        bytes_written_ = self.ser_handle.write(self.write_buf_)
        print("bytes written:", bytes_written_)
        bytes_read_ = self.ser_handle.read(20)
        print("bytes recieved: ", struct.unpack("fffff", bytes_read_))


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
