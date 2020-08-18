import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import threading
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import PoseStamped, Twist

# Create the action client for the grippers, this will send commands
# to the movo gripper topics
rightGripperActionClient = actionlib.SimpleActionClient(
  "/movo/right_gripper_controller/gripper_cmd", GripperCommandAction)
leftGripperActionClient = actionlib.SimpleActionClient(
  "/movo/left_gripper_controller/gripper_cmd", GripperCommandAction)

# Initialize the connection to the moveit robot
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
# Get the predefined group names for the movo, this determines what is available
# to control
#print robot.get_group_names()

# Create the interface to the move groups and give them more time to plan
rightArm = moveit_commander.MoveGroupCommander("right_arm")
rightArm.set_planning_time(10)
leftArm = moveit_commander.MoveGroupCommander("left_arm")
leftArm.set_planning_time(10)

# Use the RRTConnectkConfigDefault motion planner instead of the moveit default
rightArm.set_planner_id("RRTConnectkConfigDefault")
leftArm.set_planner_id("RRTConnectkConfigDefault")

class ein_movo:

  """
  Defines the methods for each incoming word and then calls all words.
  """
  def movo_commands_left(self ,command):
    data = command.data
    switch = {
      'closeGripper': [self.moveGripper,[leftGripperActionClient, 0.01]], 
      'openGripper': [self.moveGripper,[leftGripperActionClient, 0.165]],
      'moveToEEPose': [self.moveToEEPose,[leftArm]],
      'twistMove': [self.twistMove, []]
    }
    words = data.split()
    for i in range(len(words)):
      func, args = switch.get(words[i], ["Unsupported argument", []])
      try:
        func(words, i, args)
      except:
        pass

  def movo_commands_right(self ,command):
    data = command.data
    switch = {
      'closeGripper': [self.moveGripper,[rightGripperActionClient, 0.01]],
      'openGripper': [self.moveGripper,[rightGripperActionClient, 0.165]],
      'moveToEEPose': [self.moveToEEPose,[rightArm]],
      'twistMove': [self.twistMove, []]
    }
    words = data.split()
    for i in range(len(words)):
      func, args = switch.get(words[i], ["Unsupported argument", []])
      try:
        func(words, i, args)
      except:
        pass

  # Moves the gripper for the arm selected
  # Args are [correct gripper action client, position to move the gripper to]
  def moveGripper(self ,data, i, args):
    acquired = self.movingLock.acquire(False)
    if acquired:
      goal = GripperCommandGoal()
      goal.command.position = args[1]
      goal.command.max_effort = -1
      args[0].send_goal(goal)
      self.movingLock.release()
    else:
      pass
    return ("Move Gripper " + str(args[0]) + " " + str(args[1]))

  # Moves the selected end effector to the position specified.
  # Args is [the movement group for the arm being moved]
  def moveToEEPose(self ,data, i, incomingArgs):
    mover = threading.Thread(target=self.movementThread, args=(data, i, incomingArgs))
    mover.start()
    return "Move End Effector"

  # Starts a thread so that recurrent messages to move don't block and be submitted
  # multiple times
  def movementThread(self ,data, i, args):
    acquired = self.movingLock.acquire(not self.moving)
    if acquired:
      self.moving = True
      pose = PoseStamped()
      
      pose.pose.position.x = float(data[i - 7])
      pose.pose.position.y = float(data[i - 6])
      pose.pose.position.z = float(data[i - 5])
      pose.pose.orientation.x = float(data[i - 4])
      pose.pose.orientation.y = float(data[i - 3])
      pose.pose.orientation.z = float(data[i - 2])
      pose.pose.orientation.w = float(data[i - 1])

      pose.header.frame_id = "base_link"
      # Creates a pose target and then plans, finally submitting the plan for movement
      args[0].set_pose_target(pose)
      result = args[0].plan()
      args[0].go(wait=True)
      self.movingLock.release()
      self.moving = False
    else:
      pass

  # Moves the base of the movo
  def twistMove(self, data, i, incomingArgs):
    twister = threading.Thread(target=self.twistThread, args=(data, i, incomingArgs))
    twister.start()
    return "Twist Move"

  # Expecting this to come in the form 6 numbers and then twistMove
  def twistThread(self ,data, i, args):
    acquired = self.movingLock.acquire(not self.moving)
    if acquired:
      self.moving = True
      twist = Twist()
      
      twist.linear.x = float(data[i - 6])
      twist.linear.y = float(data[i - 5])
      twist.linear.z = float(data[i - 4])
      twist.angular.x = float(data[i - 3])
      twist.angular.y = float(data[i - 2])
      twist.angular.z = float(data[i - 1])

      self.twistPub.publish(twist)
      self.movingLock.release()
      self.moving = False
    else:
      pass

  def listener(self):
    rospy.init_node('movo_python')
    # Defines whether the robot is currently acting out a plan
    self.movingLock = threading.Lock()
    self.moving = False

    rospy.Subscriber("ein/right/forth_commands", String, self.movo_commands_right)
    rospy.Subscriber("ein/left/forth_commands", String, self.movo_commands_left)
    self.twistPub = rospy.Publisher("/movo/cmd_vel", Twist, queue_size=10)

    # Don't exit
    rospy.spin()

if __name__ == '__main__':
  ein = ein_movo()
  ein.listener()
