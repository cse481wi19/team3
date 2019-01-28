#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy                                                                                           
from sensor_msgs.msg import JointState
from robot_api import ArmJoints

JOINT_TOPIC = "/joint_states"

class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self):                                                                                
        self.arm_joints = ArmJoints.from_list([None] * 7)
        self.joint_names = self.arm_joints.names()
        self.torso_joint = None
        rospy.Subscriber(JOINT_TOPIC, JointState, self.callback)
                                                                                                       
    def callback(self, data):
        for name, position in zip(data.name, data.position):
            if name == self.joint_names[0]:
                self.arm_joints.set_shoulder_pan(position)
            elif name == self.joint_names[1]:
                self.arm_joints.set_shoulder_lift(position)
            elif name == self.joint_names[2]:
                self.arm_joints.set_upperarm_roll(position)
            elif name == self.joint_names[3]:
                self.arm_joints.set_elbow_flex(position)
            elif name == self.joint_names[4]:
                self.arm_joints.set_forearm_roll(position)
            elif name == self.joint_names[5]:
                self.arm_joints.set_wrist_flex(position)
            elif name == self.joint_names[6]:
                self.arm_joints.set_wrist_roll(position) 
            elif name == "torso_lift_joint":
                self.torso_joint = position
            

    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """                                                                                            
        if name == "torso_lift_joint":
            return self.torso_joint

        try:
            return self.arm_joints.values()[self.joint_names.index(name)]
        except ValueError:
            return None

                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """                                                                     
        ret = []
        for name in names:
            ret.append(self.get_joint(name))
        return ret
