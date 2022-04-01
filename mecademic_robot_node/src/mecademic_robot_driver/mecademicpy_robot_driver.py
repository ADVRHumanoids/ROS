#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
import mecademicpy.robot as mdr
import mecademicpy.mx_robot_def as mdr_def
import logging


class MecademicRobot_Driver():
    """ROS Mecademic Robot Node Class to make a Node for the Mecademic Robot

    Attributes:
        subscriber: ROS subscriber to send command to the Mecademic Robot through a topic
        publisher: ROS publisher to place replies from the Mecademic Robot in a topic
        MecademicRobot : driver to control the MecademicRobot Robot
    """

    def __init__(self, robot):  # , feedback):
        """Constructor for the ROS MecademicRobot Driver
        """
        rospy.init_node("MecademicRobot_driver", anonymous=True)
        self.joint_subscriber = rospy.Subscriber("MecademicRobot_joint", JointState, self.joint_callback, queue_size=1)
        # self.pose_subscriber = rospy.Subscriber("MecademicRobot_pose", Pose, self.pose_callback)
        # self.command_subscriber = rospy.Subscriber("MecademicRobot_command", String, self.command_callback)
        self.gripper_subscriber = rospy.Subscriber("MecademicRobot_gripper", Bool, self.gripper_callback)
        self.reply_publisher = rospy.Publisher("MecademicRobot_reply", String, queue_size=1)
        self.joint_publisher = rospy.Publisher("MecademicRobot_joint_fb", JointState, queue_size=1)
        self.pose_publisher = rospy.Publisher("MecademicRobot_pose_fb", Pose, queue_size=1)
        self.status_publisher = rospy.Publisher("MecademicRobot_status", UInt8MultiArray, queue_size=1)

        robot.logger = logging.getLogger(f'rosout.{__name__}')

        self.robot = robot
        self.robot.SetRealTimeMonitoring('all')
        self.robot.SetMonitoringInterval(0.005)

        self.robot.SetBlending(100)
        self.robot.SetJointVel(100)
        self.robot.SetJointAcc(100)

        print(self.robot._monitoring_interval)

        # self.feedback = feedback

        self.socket_available = True

        # self.feedbackLoop()

    def __del__(self):
        """Deconstructor for the Mecademic Robot ROS driver
        Deactivates the robot and closes socket connection with the robot
        """
        self.robot.WaitIdle()
        self.robot.DeactivateRobot()
        self.robot.Disconnect()

    # def command_callback(self, command):
    #     """Forwards a ascii command to the Mecademic Robot

    #     :param command: ascii command to forward to the Robot
    #     """
    #     while not self.socket_available:  # wait for socket to be available
    #         pass
    #     self.socket_available = False  # block socket from being used in other processes
    #     if self.robot.is_in_error():
    #         self.robot.ResetError()
    #         self.robot.ResumeMotion()
    #     reply = self.robot.exchange_msg(command.data, decode=False)
    #     self.socket_available = True  # Release socket so other processes can use it
    #     if reply is not None:
    #         self.reply_publisher.publish(reply)

    def joint_callback(self, joints):
        """Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param joints: message received from topic containing position and velocity information
        """
        # while not self.socket_available:  # wait for the socket to be available
        #     pass
        # self.socket_available = False  # Block other processes from using the socket
        
        # if len(joints.velocity) > 0:
        #     self.robot.SetJointVel(joints.velocity[0])
        
        # Attempt to clear error if robot is in error.
        if self.robot.GetStatusRobot().error_status:
            # self.robot.ClearMotion()
            self.robot.ResetError()
            self.robot.ResumeMotion()

        try:
            self.robot.MoveJoints(joints.position[0], joints.position[1], joints.position[2],
                                        joints.position[3], joints.position[4], joints.position[5])
        except Exception as exception:
            rospy.logerr(str(exception))
            rospy.loginfo('Robot has encountered an error, attempting to clear...')
            raise
        
        #     reply = self.robot.MoveJoints(joints.position[0], joints.position[1], joints.position[2],
        #                                   joints.position[3])
        # self.socket_available = True  # Release the socket so other processes can use it
            # if reply is not None:
            #     self.reply_publisher.publish(reply)

    # def pose_callback(self, pose):
    #     """Callback when the MecademicRobot_emit topic receives a message
    #     Forwards message to driver that translate into real command
    #     to the Mecademic Robot

    #     :param pose: message received from topic containing position and orientation information
    #     """
    #     while (not self.socket_available):  # wait for socket to become available
    #         pass
    #     reply = None
    #     self.socket_available = False  # Block other processes from using the socket while in use
    #     if self.robot.is_in_error():
    #         self.robot.ResetError()
    #         self.robot.ResumeMotion()
    #     if pose.position.z is not None:
    #         reply = self.robot.MovePose(pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
    #                                     pose.orientation.y, pose.orientation.z)
    #     else:
    #         reply = self.robot.MovePose(pose.position.x, pose.position.y, pose.orientation.x, pose.orientation.y)
    #     self.socket_available = True  # Release socket so other processes can continue
    #     if reply is not None:
    #         self.reply_publisher.publish(reply)

    def gripper_callback(self, state):
        """Controls whether to open or close the gripper.
        True for open, False for close

        :param state: ROS Bool message
        """
        # Attempt to clear error if robot is in error.
        if self.robot.GetStatusRobot().error_status:
            # self.robot.ClearMotion()
            self.robot.ResetError()
            self.robot.ResumeMotion()
        if state.data:
            self.robot.GripperOpen()
        else:
            self.robot.GripperClose()

    def feedbackLoop(self, event=None):
        """Retrieves live position feedback and publishes the data 
        to its corresponding topic. (infinite loop)
        """
        # while not rospy.is_shutdown():
        try:
            # Robot Status Feedback
            if True:  # self.socket_available:
                # self.socket_available = False  # Block other operations from using the socket while in use
                robot_status = self.robot.GetStatusRobot()
                gripper_status = self.robot.GetStatusGripper()
                # self.socket_available = True  # Release the socket so other processes can happen
                status = UInt8MultiArray()
                status.data = [
                    robot_status.activation_state,
                    robot_status.homing_state,
                    robot_status.simulation_mode,
                    robot_status.error_status,
                    robot_status.pause_motion_status,
                    robot_status.end_of_block_status,
                    # robot_status["EOM"],  # Removed from current API version

                    # The values of the gripper_status object are all zero at the moment. Probably due to the FW version
                    gripper_status.present,
                    gripper_status.homing_state,
                    gripper_status.holding_part,
                    gripper_status.target_pos_reached,
                    gripper_status.error_status,
                    gripper_status.overload_error
                ]
                self.status_publisher.publish(status)

            # Position Feedback
            rt_data = robot.GetRobotRtData()
            
            # joints_ref = JointState() ... TODO
            joints_fb = JointState()
            joints_fb.position = rt_data.rt_joint_pos.data
            joints_fb.velocity = rt_data.rt_joint_vel.data
            joints_fb.effort = rt_data.rt_joint_torq.data
            
            pose_fb = Pose()
            pose_fb.position.x = rt_data.rt_cart_pos.data[0]
            pose_fb.position.y = rt_data.rt_cart_pos.data[1]
            pose_fb.position.z = rt_data.rt_cart_pos.data[2]
            pose_fb.orientation.x = rt_data.rt_cart_pos.data[3]
            pose_fb.orientation.y = rt_data.rt_cart_pos.data[4]
            pose_fb.orientation.z = rt_data.rt_cart_pos.data[5]
            
            self.joint_publisher.publish(joints_fb)
            self.pose_publisher.publish(pose_fb)
        
        except Exception as error:
            rospy.logerr(str(error))


    # def publish_rt_data(self):
    #     """Retrieves live position feedback and publishes the data 
    #     to its corresponding topic. (infinite loop)
    #     """
    #     while not rospy.is_shutdown():
    #         try:
    #             # Robot Status Feedback
    #             # if True:  # self.socket_available:
    #             #     # self.socket_available = False  # Block other operations from using the socket while in use
    #             #     robot_status = self.robot.GetStatusRobot()
    #             #     gripper_status = self.robot.GetStatusGripper()
    #             #     # self.socket_available = True  # Release the socket so other processes can happen
    #             #     status = UInt8MultiArray()
    #             #     status.data = [
    #             #         robot_status.activation_state,
    #             #         robot_status.homing_state,
    #             #         robot_status.simulation_mode,
    #             #         robot_status.error_status,
    #             #         robot_status.pause_motion_status,
    #             #         robot_status.end_of_block_status,
    #             #         # robot_status["EOM"],  # Removed from current API version

    #             #         # The values of the gripper_status object are all zero at the moment. Probably due to the FW version
    #             #         gripper_status.present,
    #             #         gripper_status.homing_state,
    #             #         gripper_status.holding_part,
    #             #         gripper_status.target_pos_reached,
    #             #         gripper_status.error_status,
    #             #         gripper_status.overload_error
    #             #     ]
    #             #     self.status_publisher.publish(status)

    #             # Position Feedback
    #             rt_data = robot.GetRobotRtData()
                
    #             # joints_ref = JointState() ... TODO
    #             joints_fb = JointState()
    #             joints_fb.position = rt_data.rt_joint_pos.data
    #             joints_fb.velocity = rt_data.rt_joint_vel.data
    #             joints_fb.effort = rt_data.rt_joint_torq.data
                
    #             pose_fb = Pose()
    #             pose_fb.position.x = rt_data.rt_cart_pos.data[0]
    #             pose_fb.position.y = rt_data.rt_cart_pos.data[1]
    #             pose_fb.position.z = rt_data.rt_cart_pos.data[2]
    #             pose_fb.orientation.x = rt_data.rt_cart_pos.data[3]
    #             pose_fb.orientation.y = rt_data.rt_cart_pos.data[4]
    #             pose_fb.orientation.z = rt_data.rt_cart_pos.data[5]
                
    #             self.joint_publisher.publish(joints_fb)
    #             self.pose_publisher.publish(pose_fb)
            
    #         except Exception as error:
    #             rospy.logerr(str(error))

def get_joint_limits(meca_robot):
    for i in range(1,7):
        get_event = meca_robot.SendCustomCommand(f'GetJointLimits({i})', [mdr_def.MX_ST_GET_JOINT_LIMITS])
        res_get = get_event.wait(timeout=10)
        print(res_get.data)

if __name__ == "__main__":
    robot = mdr.Robot()
    robot.Connect(address='192.168.0.100', disconnect_on_exception=False)

    # with mdr.Robot() as robot:
    #     # CHECK THAT IP ADDRESS IS CORRECT! #
    #     try:
    #         robot.Connect(address='192.168.0.100')
    #         print('Connected to robot')
    #     except mdr.CommunicationError as e:
    #         print(f'Robot failed to connect. Is the IP address correct? {e}')
    #         raise e

    # callbacks = mdr.RobotCallbacks()
    # callbacks.on_monitor_message = robot.publish_rt_data
    # robot.RegisterCallbacks(callbacks=callbacks, run_callbacks_in_separate_thread=True)

    robot.ActivateRobot()
    robot.Home()
    driver = MecademicRobot_Driver(robot)
    timer = rospy.Timer(rospy.Duration(0.005), driver.feedbackLoop)
    while not rospy.is_shutdown():
        # driver.feedbackLoop()
        # rate.sleep()
        rospy.spin()
