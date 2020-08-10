#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Int16MultiArray
from sensor_msgs.msg import CompressedImage

from gazebo_msgs.msg import LinkStates, ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState

from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
import numpy as np
import scipy.optimize
import cv2
import json

THETA_EXT = 0.27
THETA_RET = np.pi/4

L_FUDGE = 0.08
Q_FUDGE = 0.0325
Z_FUDGE = 0.04


Z_MAX_SIDE = -0.03
Z_MAX_DOWN = 0
Z_MIN = -0.045

CLOSE_ENOUGH = 0.02
DEFAULT_ROT = 0

S_MAX = 0.4

class Arm3Link:
    """
    credit: https://github.com/studywolf/blog/tree/master/InvKin
    """
    def __init__(self, L=None):
        # initial joint angles
        self.q = [0, 0, 0]
        # some default arm positions
        self.L = np.array([1, 1, 0.8]) if L is None else L
        self.max_y = 1
        self.min_y = 0

        self.end_angle_tol = 0.05
        self.end_angle = -np.pi/2
        self.max_angles = [1.6, np.pi/2, np.pi/2]
        self.min_angles = [0.27, -np.pi/2, -np.pi/2]

    def get_xy(self, q=None):
        if q is None:
            q = self.q

        x = self.L[0]*np.cos(q[0]) + \
            self.L[1]*np.cos(q[0]+q[1]) + \
            self.L[2]*np.cos(np.sum(q))

        y = self.L[0]*np.sin(q[0]) + \
            self.L[1]*np.sin(q[0]+q[1]) + \
            self.L[2]*np.sin(np.sum(q))

        return [x, y]

    def inv_kin(self, x, min_y, max_y, end_angle):

        def distance_to_default(q, x):
            x = (self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) +
                 self.L[2]*np.cos(np.sum(q))) - x
            return x**2

        def y_upper_constraint(q, *args):
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q)))
            return self.max_y - y

        def y_lower_constraint(q, *args):
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q)))
            return y - self.min_y

        def joint_limits_upper_constraint(q, *args):
            return self.max_angles - q

        def joint_limits_lower_constraint(q, *args):
            return q - self.min_angles

        def joint_limits_last_orientation(q, *args):
            return self.end_angle_tol - np.abs(np.sum(q)-self.end_angle)

        self.min_y = min_y
        self.max_y = max_y
        if end_angle is not None:
            self.end_angle = end_angle
        q = scipy.optimize.fmin_slsqp(
            func=distance_to_default,
            x0=self.q,
            # uncomment to add in min / max angles for the joints
            ieqcons=[joint_limits_last_orientation,
                     joint_limits_upper_constraint,
                     joint_limits_lower_constraint,
                     y_upper_constraint,
                     y_lower_constraint],
            args=(x,))  # iprint=0 suppresses output
        self.q = q
        return self.q

class GazeboLinkPose:
  link_name = ''
  link_pose = Pose()
  def __init__(self):
    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)

  def callback(self, data):
    try:
      self.data = data
    except ValueError:
      pass

  def get_link_position(self, link_name):
    ind = self.data.name.index(link_name)
    return self.data.pose[ind].position


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho*np.cos(phi)
    y = rho*np.sin(phi)
    return(x, y)

def get_other_angles(theta_shoulder):
  theta_wrist = theta_shoulder + np.pi/2
  theta_elbow = np.pi/2 - 2*theta_shoulder
  return theta_wrist, theta_elbow

class BraccioXYBBTargetInterface(object):
  """BraccioXYBBTargetInterface"""
  def __init__(self):
    super(BraccioXYBBTargetInterface, self).__init__()

    myargv = rospy.myargv(argv=sys.argv)
    self.class_name_path = myargv[1]
    self.sim=False
    self.classes = None
    if myargv[1]=='sim':
      self.sim = True
    else:
      with open(self.class_name_path, 'r') as f:
        self.classes = [line.strip() for line in f.readlines()]

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('braccio_xy_bb_target', anonymous=True)

    group_name = "braccio_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)

    self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")

    self.bounding_box = [0,0,0]
    self.subscriber = rospy.Subscriber("bounding_box",  Int16MultiArray, self.bb_callback, queue_size=1)
    self.homography = None

    self.input_image_compressed = "/pi3a/image/compressed"
    self.current_image = CompressedImage()

    self.subscriber = rospy.Subscriber(self.input_image_compressed,  CompressedImage, self.cal_im_callback, queue_size=1)
    self.mouseX = 100
    self.mouseY = 100

    self.selected_class = None
    self.link_locator = GazeboLinkPose()

    self.down = False
    self.kinematics = Arm3Link()

  def bb_callback(self, ros_data):
    self.bounding_box = ros_data.data

  def cal_im_callback(self, ros_data):
    self.current_image = ros_data

  def reset_target_position(self):
    state_msg = ModelState()
    state_msg.model_name = 'unit_box_0'
    print 'reset x='
    tst = raw_input()
    state_msg.pose.position.x = float(tst)
    print 'reset y='
    tst = raw_input()
    state_msg.pose.position.y = float(tst)
    print 'reset z='
    tst = raw_input()
    state_msg.pose.position.z = float(tst)
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

  def transform(self, x1, y1, r):
    if self.homography is not None:
      a = np.array([[x1, y1]], dtype='float32')
      res = cv2.perspectiveTransform(a[None, :, :], self.homography)[0][0]
      return float(res[0]), float(res[1]), DEFAULT_ROT
    else:
      raise ValueError('run or load calibration first!')

  def transform_bb(self):
    x, y, r = self.bounding_box_center()
    return self.transform(x,y,r)

  def select_target(self):
    if self.classes:
      print 'select target'
      for i in range(len(self.classes)):
        print '  enter '+str(i)+' for '+str(self.classes[i])
      tst = raw_input()
      self.selected_class = int(tst)

  def bounding_box_center(self):
    if self.sim:
      return self.get_link_position(['unit_box_0::link'])
    if len(self.bounding_box)==0:
      return 0, 0, 0
    ind = 0
    if self.selected_class:
      for i in range(len(self.bounding_box)/5):
        if self.bounding_box[i*5]==self.selected_class:
          ind = i*5
    x = self.bounding_box[ind*5 + 1]
    y = self.bounding_box[ind*5 + 2]
    return x, y, DEFAULT_ROT

  def draw_circle(self,event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
      self.mouseX, self.mouseY = x,y

  def wait_for_image_click(self, which=None):
    if self.sim:
      return self.get_link_position(which)
    self.mouseX, self.mouseY = None, None
    while(True):
      np_arr = np.fromstring(self.current_image.data, np.uint8)
      img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      if self.mouseX:
        cv2.circle(img,(self.mouseX, self.mouseY),10,(255,0,0),-1)
      cv2.imshow('image',img)
      k = cv2.waitKey(20) & 0xFF
      if self.mouseX or k == ord('p'):
          break
    return self.mouseX, self.mouseY

  def get_link_position(self, which):
    x = 0
    y = 0
    n = 0
    for w in which:
      res = self.link_locator.get_link_position(w)
      x += res.x
      y += res.y
      n += 1
    print x/n, y/n
    return x/n, y/n, DEFAULT_ROT

  def calibrate(self):
    if not self.sim:
      cv2.namedWindow('image')
      cv2.setMouseCallback('image',self.draw_circle)
      while len(self.current_image.data) == 0:
        time.sleep(10)

    src_pts = []
    dst_angs = []
    mouseX = None
    while not mouseX:
      print 'click on robot base.'
      mouseX, mouseY = self.wait_for_image_click(which=['kuka::base_link'])
    src_pts.append([mouseX,mouseY])

    self.gripper_middle()
    N = 8
    phi_min = np.pi/6
    phi_max = np.pi - np.pi/6
    for i in range(2,N):
      print 'click on location, press p to pass.'
      self.go_to_raise()
      if i % 2 == 0:
        rand_phi = phi_min + i*(phi_max-phi_min)/N
        theta_shoulder = THETA_RET
      else:
        theta_shoulder = THETA_EXT
      theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
      rand_targ = [rand_phi,theta_shoulder,theta_elbow, theta_wrist]
      self.go_to_joint(rand_targ)
      mouseX, mouseY = self.wait_for_image_click(which=['kuka::left_gripper_link','kuka::right_gripper_link'])
      if mouseX:
        src_pts.append([mouseX,mouseY])
        dst_angs.append(rand_targ)
    with open('calibration.json', 'w') as f:
      json.dump({'src_pts':src_pts,'dst_angs':dst_angs},f)
    self.load_calibrate()
    self.go_to_up()

  def load_calibrate(self):
    with open('calibration.json', 'r') as f:
      calib = json.load(f)
    src_pts = calib['src_pts']
    dst_angs = calib['dst_angs']

    s_ret_pts = src_pts[1::2]
    s_ext_pts = src_pts[2::2]
    arr = np.array(s_ret_pts)-np.array(s_ext_pts)
    self.L = np.sqrt((arr*arr).sum(axis=1)).mean()/(np.cos(THETA_EXT)-np.cos(THETA_RET))
    arr = np.array(s_ret_pts)-np.array(src_pts[0])
    l1 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_RET)
    arr = np.array(s_ext_pts)-np.array(src_pts[0])
    l2 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_EXT)
    self.l = (l1+l2)/2

    print 'l = ' + str(self.l)
    print 'L = ' + str(self.L)

    dst_pts = [[0,0]]
    for i in range(len(dst_angs)):
      phi = dst_angs[i][0]
      rho = self.L*np.cos(dst_angs[i][1]) + self.l
      x, y = pol2cart(rho, phi)
      dst_pts.append([x,y])

    print src_pts
    print dst_pts

    src_pts = np.array(src_pts)
    dst_pts = np.array(dst_pts)

    h, status = cv2.findHomography(src_pts, dst_pts)
    self.homography = h

    self.kinematics = Arm3Link(L=[self.L/2,self.L/2,self.l+L_FUDGE])
    print 'calibration done.'
    cv2.destroyAllWindows()

  def go_to_joint(self, joint_targets):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = joint_targets[0]
    joint_goal[1] = joint_targets[1]
    joint_goal[2] = joint_targets[2]
    joint_goal[3] = joint_targets[3]
    joint_goal[4] = 1.5708
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

  def gripper_close(self):
    self.go_gripper(1.2)

  def gripper_open(self):
    self.go_gripper(0.2)

  def gripper_middle(self):
    self.go_gripper(0.9)

  def go_gripper(self, val):
    joint_goal = self.gripper_group.get_current_joint_values()
    joint_goal[0] = val
    joint_goal[1] = val
    self.gripper_group.go(joint_goal, wait=True)
    self.gripper_group.stop()


  def go_to_raise(self):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.15
    joint_goal[2] = 0.13
    joint_goal[3] = 2.29
    self.go_to_joint(joint_goal)

  def go_to_pull(self, phi):
    self.go_to_raise()
    self.gripper_close()
    if phi:
      joint_goal = self.move_group.get_current_joint_values()
      joint_goal[0] = float(phi)
      self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.3
    joint_goal[2] = 1.8
    joint_goal[3] = 1.8
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.3
    joint_goal[2] = 1.8
    joint_goal[3] = 0.1
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.2
    joint_goal[2] = 0.4
    joint_goal[3] = 0.2
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.3
    joint_goal[2] = 1.8
    joint_goal[3] = 0.1
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.3
    joint_goal[2] = 1.8
    joint_goal[3] = 1.8
    self.go_to_joint(joint_goal)

  def go_to_push(self, phi):
    self.go_to_raise()
    self.gripper_close()
    if phi:
      joint_goal = self.move_group.get_current_joint_values()
      joint_goal[0] = float(phi)
      self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.7
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.6
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.3
    joint_goal[2] = 1.8
    joint_goal[3] = 0.1
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.1
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.7
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)

  def get_targets(self,x,y):
    s, phi = cart2pol(x,y)
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_SIDE, 0)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print 'NO SOLUTION FOUND'
      print 'real= '+str(xy[0]) + ' goal='+str(s)
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def get_down_targets(self,x,y):
    s, phi = cart2pol(x,y)
    print s, phi
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -np.pi/2)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print 'NO SOLUTION FOUND'
      print 'real= '+str(xy[0]) + ' goal='+str(s)
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def go_to_xy(self, x, y, r, how):
    if how=='down':
      s, joint_targets = self.get_down_targets(x,y)
      print joint_targets
      if np.isnan(joint_targets[1]) and s < S_MAX:
        print '++++++ Too far out, pulling backwards +++++'
        print 'theta = ' + str(joint_targets[1])
        self.go_to_pull(joint_targets[0])
        return
      if np.isnan(joint_targets[1]):
        print '++++++ Not in Domain ++++++'
        print 'theta = ' + str(joint_targets[1])
        print '+++++++++++++++++++++++++++'
        return
    elif how=='side':
      s, joint_targets = self.get_targets(x,y)
      print joint_targets
      if np.isnan(joint_targets[1]) and s < S_MAX:
        print '++++++ Too close, pushing backwards +++++'
        print 'theta = ' + str(joint_targets[1])
        self.go_to_push(joint_targets[0])
        return
      if np.isnan(joint_targets[1]):
        print '++++++ Not in Domain ++++++'
        print 'theta = ' + str(joint_targets[1])
        print '+++++++++++++++++++++++++++'
        return

    self.go_to_raise()
    self.gripper_open()
    joint_goal = self.move_group.get_current_joint_values()

    joint_goal[0] = float(joint_targets[0])
    self.go_to_joint(joint_goal)

    joint_goal[1] = float(joint_targets[1])
    joint_goal[2] = float(joint_targets[2])
    joint_goal[3] = float(joint_targets[3])
    self.go_to_joint(joint_goal)
    self.gripper_close()
    self.go_to_home()

  def go_to_manual_joint(self):
    joint_goal = self.move_group.get_current_joint_values()
    for i in range(len(joint_goal)):
      print 'joint' + str(i) + ' ' + str(joint_goal[i])
      tst = raw_input()
      if tst!='':
          joint_goal[i] = float(tst)
    self.go_to_joint(joint_goal)

  def go_to_manual(self, how):
    print 'pos x?'
    tst = raw_input()
    if tst!='':
        x = float(tst)
    print 'pos y?'
    tst = raw_input()
    if tst!='':
        y = float(tst)
    self.go_to_xy(x, y, DEFAULT_ROT, how)

  def go_to_manual_gripper(self):
    print 'grip position?'
    tst = raw_input()
    if tst!='':
        v = float(tst)
    self.go_gripper(v)

  def go_to_target(self, how):
    x,y,r = self.transform_bb()
    print x, y, r
    self.go_to_xy(x, y, r, how)

  def go_to_home(self):
    self.go_to_raise()
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 3.14
    self.go_to_joint(joint_goal)
    self.gripper_open()
    self.gripper_open()

  def go_to_bowl(self):
    self.go_to_raise()
    self.gripper_open()
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.7
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.6
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 0.3
    joint_goal[2] = 1.8
    joint_goal[3] = 0.1
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.1
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 2.7
    joint_goal[2] = 0.01
    joint_goal[3] = 0.01
    self.go_to_joint(joint_goal)

  def go_to_up(self):
    self.go_to_raise()
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 1.5708
    joint_goal[1] = 1.5708
    joint_goal[2] = 1.5708
    joint_goal[3] = 1.5708
    self.go_to_joint(joint_goal)

  def print_pose(self):
    print self.gripper_group.get_current_joint_values()
    x, y = self.bounding_box_center()
    print x, y

def main():
  bb_targetter = BraccioXYBBTargetInterface()

  bb_targetter.load_calibrate()
  while True:
      print "============ instructions: p=print, h=home, u=up, c=calibrate, l=load_calibration, t=target, s=select_target, r=reset_target, m=manual, q=quit"
      inp = raw_input()
      if inp=='q':
          break
      if inp=='p':
          bb_targetter.print_pose()
      if inp=='c':
          bb_targetter.calibrate()
      if inp=='l':
          bb_targetter.load_calibrate()
      if inp=='t':
          bb_targetter.go_to_target('side')
      if inp=='d':
          bb_targetter.go_to_target('down')
      if inp=='m':
          bb_targetter.go_to_manual('side')
      if inp=='n':
          bb_targetter.go_to_manual('down')
      if inp=='h':
          bb_targetter.go_to_home()
      if inp=='u':
          bb_targetter.go_to_up()
      if inp=='s':
          bb_targetter.select_target()
      if inp=='r':
          bb_targetter.reset_target_position()
      if inp=='g':
          bb_targetter.go_to_manual_gripper()
      if inp=='b':
          bb_targetter.go_to_bowl()


if __name__ == '__main__':
  main()
