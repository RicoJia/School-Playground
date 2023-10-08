#!/usr/bin/env python
"""  A test node to move the end effector to the camera and come back, using sample code from intera.
"""

import rospy

from geometry_msgs.msg import Pose, WrenchStamped

import intera_interface

from trajectory_generation import TrajGen

class Move2DesiredPose(object):

    def __init__(self):
        self._limb = intera_interface.Limb("right")
        self.initial_t = rospy.Time.now().to_sec()

        self.targetEePose = Pose()
        self.desiredEePose = Pose()
        self.initialPose = self._limb.endpoint_pose()

        self.maneuver_time = 4.0;

        self.targetEePose.position.x = 0.0 + self.initialPose['position'].x
        self.targetEePose.position.y = 0.0 + self.initialPose['position'].y
        self.targetEePose.position.z = -0.0 + self.initialPose['position'].z


        self.targetEePose.orientation.x = 1.0
        self.targetEePose.orientation.y = 0.0
        self.targetEePose.orientation.z = 0.0
        self.targetEePose.orientation.w = 0.0

   def advance(self):
        # joint commands are sent in this function
        self.currentPose = self._limb.endpoint_pose()
        self.desiredEePose.position.z = self.currentPose['position'].z # force control

        self.desiredEePose.orientation = self.targetEePose.orientation
        
        #Test
        print("Received pose: ", self.desiredEePose.position)
        joint_angles = self._limb.ik_request(self.desiredEePose)
        self._limb.set_joint_positions(joint_angles)


def main():
    """ The main() function. """
    rospy.init_node('move_to_desired_pose', anonymous=True)
    eePoseController = Move2DesiredPose()
    # eePoseController.gripper_close()
    # eePoseController.gripper_open()
    trajgen = TrajGen()

    while not rospy.is_shutdown():
        trajgen.update_trajectory_status()
        [eePoseController.desiredEePose.position.x, eePoseController.desiredEePose.position.y] = trajgen.get_xy()
        draw_status = trajgen.get_draw_status() #TODO: use this for force control
        #eePoseController.forceControl()
        eePoseController.advance()
        
        #testing 
        #currentPose = trajgen._limb.endpoint_pose()
       # [x,y,z] = [currentPose['position'].x, currentPose['position'].y, currentPose['position'].z]
       # print "x, y, z = ", x,"  ", y,"  ",z

        eePoseController.publishWrench()
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
