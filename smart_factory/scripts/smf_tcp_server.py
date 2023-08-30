#!/usr/bin/env python3

import rospy
import socket
import struct
from smart_factory.msg import AgvToSmfStatus, SmfToAgvStatus


class SmfTcpServer:
    def __init__(self):
        rospy.init_node("smf_tcp_server")
        self.ip_ = ""
        self.port_ = 65432
        self.boxMissing_ = False
        self.penMissing_ = False
        self.smfPrbtPickingBox_ = False
        self.smfPrbtPickingPen_ = False
        self.smfPrbtPlacingBox_ = False
        self.smfPrbtPlacingPen_ = False
        self.smfPrbtAtHome_ = False
        self.publisher_ = rospy.Publisher(
            "agv_to_smf_status", AgvToSmfStatus, queue_size=10
        )

        rospy.Subscriber(
            "smf_to_agv_status", SmfToAgvStatus, self.callback_SmfToAgvStatus
        )
        self.start_tcp_server()

    def callback_SmfToAgvStatus(self, msg):
        self.boxMissing_ = msg.boxMissing
        self.penMissing_ = msg.penMissing
        self.smfPrbtPickingBox_ = msg.smfPrbtPickingBox
        self.smfPrbtPickingPen_ = msg.smfPrbtPickingPen
        self.smfPrbtPlacingBox_ = msg.smfPrbtPlacingBox
        self.smfPrbtPlacingPen_ = msg.smfPrbtPlacingPen
        self.smfPrbtAtHome_ = msg.smfPrbtAtHome

    def start_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind((self.ip_, self.port_))
            sock.listen()
            conn, addr = sock.accept()
            if conn:
                rospy.loginfo("Connected by %s", addr)
            while not rospy.is_shutdown():
                tcp_send_msg = struct.pack(
                    "!???????",
                    self.boxMissing_,
                    self.penMissing_,
                    self.smfPrbtPickingBox_,
                    self.smfPrbtPickingPen_,
                    self.smfPrbtPlacingBox_,
                    self.smfPrbtPlacingPen_,
                    self.smfPrbtAtHome_,
                )
                try:
                    data = conn.recv(1024)
                    if data:
                        tcp_receive_msg = struct.unpack("!????", data)
                        agvToSmfMsg = AgvToSmfStatus()
                        agvToSmfMsg.agvAtSmf = tcp_receive_msg[0]
                        agvToSmfMsg.agvPrbtChangingBox = tcp_receive_msg[1]
                        agvToSmfMsg.agvPrbtChangingPen = tcp_receive_msg[2]
                        agvToSmfMsg.agvPrbtAtHome = tcp_receive_msg[3]
                        self.publisher_.publish(agvToSmfMsg)
                    conn.send(tcp_send_msg)
                    rospy.sleep(0.1)
                except socket.error as e:
                    rospy.logerr("Connection error: %s", e)
                    conn, addr = sock.accept()
                    if conn:
                        rospy.loginfo("Connected by %s", addr)
                    rospy.sleep(1)


if __name__ == "__main__":
    SmfTcpServer()
    rospy.spin()
