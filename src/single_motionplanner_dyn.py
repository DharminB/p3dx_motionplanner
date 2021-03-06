#!/usr/bin/env python

import roslib; roslib.load_manifest('p3dx_motionplanner')
import omgtools as omg
import rospy
import tf
import numpy as np
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from costmap_converter.msg import ObstacleArrayMsg
from p3dx_motionplanner.msg import Trigger, FleetTrajectories, P3DXTrajectory, Settings, P3DXPose, Obstacle, Room


class SimpleMotionPlannerDyn(object):
    
    def __init__(self, sample_time, update_time):
        self._sample_time = sample_time
        self._update_time = update_time
        self.new_goal_set = False
        self.obs_state = None
        rospy.Subscriber('mp_trigger', Trigger, self.update)
        self._mp_result_topic = rospy.Publisher('mp_result', P3DXTrajectory, queue_size=1)
        self._mp_goal_topic = rospy.Publisher('mp_goal', P3DXPose, queue_size=1)
        self._mp_path_topic = rospy.Publisher('mp_path', Path, queue_size=1)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.get_goal)
        rospy.Subscriber('moving_obstacle_velocity', ObstacleArrayMsg, self.obs_vel_cb)

    def obs_vel_cb(self, msg) :
        self.obs_state = msg

    def get_goal(self, goal) :
        self.goal = [0.0, 0.0, 0.0]
        self.goal[0] = goal.pose.position.x
        self.goal[1] = goal.pose.position.y
        qt = goal.pose.orientation
        r, p, y = tf.transformations.euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
        # self.goal[2] = y
        rospy.loginfo(self.goal)
        self._mp_goal_topic.publish(P3DXPose(self.goal))
        self.new_goal_set = True

    def update(self, trigger_msg):
        # print 'started motion planning update!'
        current_time = trigger_msg.current_time
        # current_state = trigger_msg.state[0].pose
        # goal = trigger_msg.goal[0].pose
        current_state = trigger_msg.state[0].pose[:2]
        goal = trigger_msg.goal[0].pose[:2]
        if self.new_goal_set :
            self._vehicle.set_terminal_conditions(goal)
            self._vehicle.set_initial_conditions(current_state)
            self._deployer.reset()
            self.new_goal_set = False
        # rospy.loginfo("deployer is reset")
        for i, obst in enumerate(self.obs_state.obstacles):
            pos = [obst.polygon.points[0].x, obst.polygon.points[0].y]
            vel = [obst.velocities.twist.linear.x, obst.velocities.twist.linear.y]
            pos = np.round(pos, 1)
            vel = np.round(vel, 1)
            self._deployer.problem.environment.obstacles[i].set_state({'position': pos, 'velocity': vel})
        trajectories = self._deployer.update(current_time, current_state)
        # rospy.loginfo(trajectories)
        self.publish_path(trajectories['pose'], trajectories['time'][0])
        if np.shape(trajectories['input'])[0] == 3 :
                traj_msg = P3DXTrajectory(x_traj=trajectories['input'][0, :], y_traj=trajectories['input'][1, :], w_traj=trajectories['input'][2, :])
        else :
            traj_msg = P3DXTrajectory(x_traj=trajectories['input'][0, :], w_traj=trajectories['input'][1, :])
        # rospy.loginfo(traj_msg)
        rospy.loginfo(traj_msg.x_traj[:10])
        rospy.loginfo(traj_msg.y_traj[:10])
        rospy.loginfo(traj_msg.w_traj[:10])

        # rospy.loginfo(self._vel_traj_strg)
        self._mp_result_topic.publish(traj_msg)

    def publish_path(self, posesList, timeList) :
        path = Path()
        current_ros_time = rospy.Time.now()
        path.header.stamp = current_ros_time
        path.header.frame_id = 'world'
        poses = []
        for i in range(0, len(posesList[0]), 10) :
            pose = PoseStamped()
            time_offset = timeList[i] - timeList[0]
            pose.header.stamp.secs = current_ros_time.secs + math.floor(time_offset)
            pose.pose.position.x = posesList[0][i]
            pose.pose.position.y = posesList[1][i]
            quat = tf.transformations.quaternion_from_euler(0, 0, posesList[2][i])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            path.poses.append(pose)
        self._mp_path_topic.publish(path)


    def configure(self):
        print 'configure motionplanner'
        # robots
        # init_ctr = [-3.5, -1.]
        init_ctr = [-1.0, 0.0]
        terminal_ctr = [1.5, 1.]
        self.init_pose = [init_ctr[0], init_ctr[1], np.pi/2.]
        self.terminal_pose = [terminal_ctr[0], terminal_ctr[1], 0.0]
        # self._vehicle = omg.Dubins(shapes=omg.Circle(0.35), options={'degree': 2}, bounds={'vmax': 1.5, 'vmin':-0.5,'wmax': np.pi/3., 'wmin': -np.pi/3.})
        # self._vehicle = omg.HolonomicOrient(shapes=omg.Rectangle(0.5, 0.3))
        # self._vehicle.set_options({'safety_distance': 0.1})
        # self._vehicle.set_initial_conditions(self.init_pose)
        # self._vehicle.set_terminal_conditions(self.terminal_pose)
        self._vehicle = omg.Holonomic(shapes=omg.Rectangle(0.5, 0.3))
        self._vehicle.set_options({'safety_distance': 0.1})
        self._vehicle.set_initial_conditions(self.init_pose[:2])
        self._vehicle.set_terminal_conditions(self.terminal_pose[:2])
        # environment
        room = {'shape': omg.Rectangle(4.0, 3.0)}
        self._obstacles = []
        for k, obst in enumerate(self.obs_state.obstacles):
            shape = omg.Circle(obst.radius)
            x = obst.polygon.points[0].x
            y = obst.polygon.points[0].y
            self._obstacles.append(omg.Obstacle({'position': [x , y]}, shape=shape))
        # shape = omg.Beam(width=4.0, height=0.1, orientation=np.pi/2.0)
        # self._obstacles.append(omg.Obstacle({'position': [-2.0, -2.3]}, shape=shape))

        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)
        # create problem
        print 'creating problem'
        problem = omg.Point2point(self._vehicle, environment, freeT=False)
        problem.set_options({'hard_term_con': False, 'horizon_time': 10.})
        problem.init()
        self._deployer = omg.Deployer(problem, self._sample_time, self._update_time)
        self._deployer.reset()
        self.goal = [0.0, 0.0, 0.0]

# obst_traj = {1: {'t': 10., 'v': 0.3, 'w': 0.}}

if __name__ == '__main__':
    rospy.init_node('single_motionplanner_dyn')
    sample_time = 0.01
    update_time = 0.5
    motionplanner = SimpleMotionPlannerDyn(sample_time, update_time)
    rospy.sleep(0.5)
    motionplanner.configure()
    rospy.spin()
