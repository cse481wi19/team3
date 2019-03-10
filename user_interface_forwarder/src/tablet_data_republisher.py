#!/usr/bin/env python 

from sensor_msgs.msg import PointCloud2
import perception
import rospy


def wait_for_time(): 
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

    
def main():                                                                             
    rospy.init_node('tablet_data_republisher')
    wait_for_time()                                                                     
    path = argv[1]
    camera = perception.MockCamera()
    cloud = camera.read_cloud(path)

    if cloud is None:
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1)       
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        cloud.header.stamp = rospy.Time.now()
        pub.publish(cloud)
        rate.sleep()                                          
    
    
if __name__ == '__main__':
    main()
