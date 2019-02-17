#! /usr/bin/env python

import socket
import sys
import rospy
from user_interface_forwarder.msg import Svg

def main():
    s = socket.socket()
    s.bind(("link.cs.washington.edu",7898))
    s.listen(10) # Accepts up to 10 connections.file_number = 1# Continually accept new files

    svgpub = rospy.Publisher('user_interface_forwarder/Svg', Svg, queue_size=10)
    rospy.init_node('ui_forwarder')

    while True:
        sc, address = s.accept()
        print('SVG Received')    # Open the next file
        l = sc.recv(1024)

        while (l):
            svgpub.publish(l)
            l = sc.recv(1024)    # Close and restart
        sc.close()
    s.close()

if __name__ == '__main__':
    main()
