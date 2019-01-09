#!/usr/bin/env python

import roslib; roslib.load_manifest('p3dx_motionplanner')
import omgtools as omg
import rospy
import copy
import tf
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool
from p3dx_motionplanner.msg import Trigger, P3DXTrajectory, Settings, P3DXPose, Obstacle, Room


class SimpleController(object):
    
    def __init__(self, sample_time, update_time):
        self._sample_time = sample_time
        self._update_time = update_time
        self.terminal_pose = None
        self._robot_est_pose = [0., 0., 0.]
        self._vel_traj = {'x': [], 'y': [],'w': []}
        self._vel_traj_applied = {'x': [], 'y': [],'w': []}
        # self._cmd_vel_topic = rospy.Publisher('robot0/p3dx/cmd_vel', Twist, queue_size=1)
        self._cmd_vel_topic = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.get_model_states)
        rospy.Subscriber('mp_goal', P3DXPose, self.get_goal)
        rospy.Subscriber('mp_result', P3DXTrajectory, self.get_mp_result)
        self._mp_trigger_topic = rospy.Publisher('mp_trigger', Trigger, queue_size=1)

    def get_goal(self, goal) :
        self.terminal_pose = goal.pose
        self._vel_traj_applied = {'x': [], 'y': [],'w': []}
        self.set_goal(self.terminal_pose)

    def get_mp_result(self, data):
        # print 'got result!'
        x_traj = data.x_traj 
        y_traj = data.y_traj 
        if len(y_traj) == 0 :
            y_traj = [0.0 for i in x_traj]
        w_traj = data.w_traj
        self._vel_traj_strg = {'x':x_traj, 'y':y_traj, 'w':w_traj}
        # self._vel_traj_strg = {'x':x_traj, 'y':w_traj, 'w':y_traj}
        self._new_trajectories = True

    def get_model_states(self, data):
        # index = data.name.index('p3dx0')
        index = data.name.index('youbot')
        self._robot_est_pose[0] = data.pose[index].position.x
        self._robot_est_pose[1] = data.pose[index].position.y
        qt = data.pose[index].orientation
        r, p, y = tf.transformations.euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
        self._robot_est_pose[2] = y

    def update(self):
        current_pose = self._robot_est_pose
        # for new problem
        if self._init:
            if not self._new_trajectories:
                return
            self._index = int(self._update_time/self._sample_time)
            self._init = False
        # for regular update of current problem
        if self._index >= int(self._update_time/self._sample_time):
            if self._new_trajectories:
                # load fresh trajectories
                self.load_trajectories()
                self._time += self._index*self._sample_time
                self._index = 0
                self.over_limit = False
                # trigger motion planner
                self.fire_motionplanner(self._time, current_pose)
            else:
                # print 'safe overtime!'
                if self.over_limit :
                    return
                if self._index >= len(self._vel_traj['x']):
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0
                    cmd_vel.linear.y = 0
                    cmd_vel.angular.z = 0
                    self._cmd_vel_topic.publish(cmd_vel)
                    self._vel_traj_applied['x'].append(cmd_vel.linear.x)
                    self._vel_traj_applied['y'].append(cmd_vel.linear.y)
                    self._vel_traj_applied['w'].append(cmd_vel.angular.z)
                    self._index += 1
                    self.over_limit = True
                    return
        # send velocity sample
        cmd_vel = Twist()
        x, y = self.transform_from_world_to_robot(self._vel_traj['x'][self._index], self._vel_traj['y'][self._index])
        cmd_vel.linear.x = x
        cmd_vel.linear.y = y
        cmd_vel.angular.z = self._vel_traj['w'][self._index]
        self._cmd_vel_topic.publish(cmd_vel)
        self._vel_traj_applied['x'].append(cmd_vel.linear.x)
        self._vel_traj_applied['y'].append(cmd_vel.linear.y)
        self._vel_traj_applied['w'].append(cmd_vel.angular.z)
        self._index += 1

    def transform_from_world_to_robot(self, x, y) :
        theta = self._robot_est_pose[2]
        new_x = math.cos(theta) *x + math.sin(theta) * y
        new_y = -math.sin(theta) *x + math.cos(theta) * y
        return (new_x, new_y)

    def load_trajectories(self):
        self._vel_traj = copy.deepcopy(self._vel_traj_strg)
        self._new_trajectories = False

    def proceed(self):
        # if motion has not started
        if len(self._vel_traj_applied['x']) == 0:
            return True
        # reached goal
        stop = True
        pos_nrm = np.linalg.norm(np.array(self._robot_est_pose) - np.array(self._goal))
        vel_nrm = np.linalg.norm([self._vel_traj_applied['x'][-1], self._vel_traj_applied['y'][-1], self._vel_traj_applied['w'][-1]])
        stop *= (pos_nrm < 0.2 and vel_nrm < 0.2)
        return not stop

    def set_goal(self, goal):
        self._goal = goal
        self._time = 0.
        current_pose = self._robot_est_pose[:]
        self._new_trajectories = False
        self.fire_motionplanner(self._time, current_pose)
        self._init = True

    def fire_motionplanner(self, time, current_pose):
        # print 'firing!'
        trigger = Trigger()
        trigger.goal = [P3DXPose(self._goal)]
        trigger.state = [P3DXPose(current_pose)]
        trigger.current_time = time
        self._mp_trigger_topic.publish(trigger)

    def start(self):
        rate = rospy.Rate(1./self._sample_time)
        while not rospy.is_shutdown() :
            if self.terminal_pose is None :
                rate.sleep()
            elif (self.proceed()):
                controller.update()
                rate.sleep()
            else :
                # stop robot when goal is reached
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 0.
                self._cmd_vel_topic.publish(cmd_vel)
                print 'target reached!'
                self.terminal_pose = None

if __name__ == '__main__':
    rospy.init_node('single_controller')
    sample_time = 0.01
    update_time = 0.5
    controller = SimpleController(sample_time, update_time)
    rospy.sleep(0.5)
    controller.start()
