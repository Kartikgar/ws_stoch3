import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
import os
import time

def createSDF(mass):
  sdff = "<?xml version='1.0'?>\
  <sdf version=\"1.4\">\
    <model name=\"my_model\">\
      <pose>0 0 0.5 0 0 0</pose>\
      <static>false</static>\
      <link name=\"link\">\
        <inertial>\
          <mass>"+str(mass)+"</mass>\
          <inertia>\
            <ixx>0.2</ixx>\
            <ixy>0.0</ixy>\
            <ixz>0.0</ixz>\
            <iyy>0.2</iyy>\
            <iyz>0.0</iyz>\
            <izz>0.2</izz>\
          </inertia>\
        </inertial>\
        <collision name=\"collision\">\
          <geometry>\
            <box>\
              <size>0.2 0.2 0.2</size>\
            </box>\
          </geometry>\
        </collision>\
        <visual name=\"visual\">\
          <geometry>\
            <box>\
              <size>0.2 0.2 0.2</size>\
            </box>\
          </geometry>\
        </visual>\
      </link>\
    </model>\
  </sdf>"

  return sdff

def controlRobot():
  joy_msg = Joy()
  joy_pub = rospy.Publisher('/stoch3/joy', Joy, queue_size=1)
  joy_msg.buttons = (0, 0, 0, 0, 0, 0, 0, 0)
  joy_msg.axes = (0, 0, 0, 0, 0, 0)

  tim = 0
  ini_time = rospy.get_time()
  rate = rospy.Rate(10)
  ss_curr = 0;
  ss_prev = 1;
  ptr_str = "\n";
  while tim < 12:

    if ss_curr != ss_prev:
      print(ptr_str)

    ss_prev = ss_curr;
    if tim < 1:
      ptr_str = "Commanding Hold State"
      joy_msg.buttons = (0, 0, 0, 0, 0, 0, 0, 1)
      ss_curr = 1;
    elif tim < 3:  
      ptr_str = "Commanding Sit State"
      joy_msg.buttons = (0, 0, 1, 0, 0, 0, 0, 0)
      ss_curr = 0;
    elif tim < 10:
      ptr_str = "Commanding Stand State"
      joy_msg.buttons = (0, 0, 0, 1, 0, 0, 0, 0)
      ss_curr = 1;
    elif tim < 12:
      ptr_str = "Commanding Body Pose Controller State"
      joy_msg.buttons = (1, 0, 0, 0, 0, 0, 0, 0)
      ss_curr = 0;

    joy_pub.publish(joy_msg)

    tim = rospy.get_time() - ini_time;
    rate.sleep()

rospy.init_node('load_test_weight',log_level=rospy.INFO)

# Set zero configuration of the robot
print("Setting initial configuration of the robot")
pr1 = os.system('rosrun stoch3_gazebo set_model_configuration_node')
time.sleep(2)

# Spawn the test weight
initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0.3

mass = 10
file_name1 = 'log/expt2_bc_tp_load_'+str(mass)+'kg_1.csv';
sdf1 = createSDF(mass)
print("Spawning the " + str(mass) + " kg test weight")

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("test_weight", sdf1, "stoch3_test_weight", initial_pose, "world")

# Get the robot to body pose control
print("Commanding state machine")
controlRobot()

# Run the node that measures tracking performance
print("Measuring tracking performance")
os.system('rosrun stoch3_control measure_joint_tracking_performance '+file_name1);

