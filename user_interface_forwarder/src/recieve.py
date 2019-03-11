#! /usr/bin/env python

import json
import socket
import sys
import rospy
from user_interface_forwarder.msg import Path
from geometry_msgs.msg import Point

def main():
    s = socket.socket()
    s.bind(("astro.cs.washington.edu",7898))
    s.listen(10) # Accepts up to 10 connections.file_number = 1# Continually accept new files

    svgpub = rospy.Publisher('user_interface_forwarder/Path', Path, queue_size=10)
    rospy.init_node('ui_forwarder')

    while True:
        sc, address = s.accept()
        print('SVG Received')    # Open the next file
        data = []
        l = sc.recv(1024)
        while (l):
            data.append(l)
            l = sc.recv(1024)    # Close and restart
        message = ''.join(data)
        #message = message.replace("(", "[")
        #message = message.replace(")", "]")
        print(message)
        paths = json.loads(message)
        pub_path = []
        
        for path in paths:
            for point in path:
                x, y = point
                pub_path.append(Point(x, y, 0))
                print(str(x) + ", " + str(y))
        pub_msg = Path(pub_path)
        svgpub.publish(pub_msg)
        
        sc.close()
    s.close()

if __name__ == '__main__':
    main()
