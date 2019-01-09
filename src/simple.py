#!/usr/bin/env python

import roslib; roslib.load_manifest('p3dx_motionplanner')
import omgtools as omg
import rospy
import tf
import copy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool
from p3dx_motionplanner.msg import Trigger, FleetTrajectories, P3DXTrajectory, Settings, P3DXPose, Obstacle, Room


class SimpleController(object):
    _trigger = Trigger()

    def __init__(self, sample_time, update_time):
        self._sample_time = sample_time
        self._update_time = update_time
        self._mp_status = False
        self._robot_est_pose = [0., 0., 0.]
        self._vel_traj = {'v': [], 'w': []}
        self._vel_traj_applied = {'v': [], 'w': []}
        self._cmd_vel_topic = rospy.Publisher('robot0/p3dx/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.get_model_states)

    def get_model_states(self, data):
        index = data.name.index('p3dx0')
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
                # trigger motion planner
                self.fire_motionplanner(self._time, current_pose)
            else:
                print 'overtime!'
        # send velocity sample
        cmd_vel = Twist()
        cmd_vel.linear.x = self._vel_traj['v'][self._index]
        cmd_vel.angular.z = self._vel_traj['w'][self._index]
        self._cmd_vel_topic.publish(cmd_vel)
        self._vel_traj_applied['v'].append(cmd_vel.linear.x)
        self._vel_traj_applied['w'].append(cmd_vel.angular.z)
        self._index += 1

    def load_trajectories(self):
        self._vel_traj = copy.deepcopy(self._vel_traj_strg)
        self._new_trajectories = False

    def proceed(self):
        # if motion has not started
        if len(self._vel_traj_applied['v']) == 0:
            return True
        # reached goal
        stop = True
        pos_nrm = np.linalg.norm(np.array(self._robot_est_pose) - np.array(self._goal))
        vel_nrm = np.linalg.norm([self._vel_traj_applied['v'][-1], self._vel_traj_applied['w'][-1]])
        rospy.loginfo(str(pos_nrm) + " " +  str(vel_nrm))
        rospy.loginfo(str(self._robot_est_pose))
        stop *= (pos_nrm < 0.1 and vel_nrm < 0.1)
        return not stop

    def set_goal(self, goal):
        self._goal = goal
        self._time = 0.
        current_pose = self._robot_est_pose[:]
        self._new_trajectories = False
        self.fire_motionplanner(self._time, current_pose)
        self._init = True

    def fire_motionplanner(self, time, current_pose):
        print 'firing!'
        self.motionplanner_update(time, current_pose, self._goal)

    def motionplanner_update(self, current_time, current_state, goal):
        print 'started motion planning update!'
        if self.goal != goal :
            self._vehicle.set_terminal_conditions(goal)
            self._vehicle.set_initial_conditions(current_state)
            self._deployer.reset()
            rospy.loginfo("deployer is reset")
        trajectories = self._deployer.update(current_time, current_state)
        # rospy.loginfo(trajectories['time'])
        self._vel_traj_strg = {'v':trajectories['input'][0, :], 'w':trajectories['input'][1, :]}
        # rospy.loginfo(self._vel_traj_strg)
        self._new_trajectories = True

    def start(self):
        rate = rospy.Rate(1./self._sample_time)
        proceed = True
        print 'controller started!'
        self.set_goal(self.terminal_pose)
        while (proceed):
            controller.update()
            proceed = controller.proceed()
            rate.sleep()
        # stop robot when goal is reached
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.angular.z = 0.
        self._cmd_vel_topic.publish(cmd_vel)
        print 'target reached!'

    def init_gazebo(self, init_pose):
        rospy.set_param('gazebo/use_sim_time', True)
        try:
            ssm = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException, e:
            print 'Service call failed: %s' % (e)
        pose0, twist0 = Pose(), Twist()
        pose0.position.x = init_pose[0]
        pose0.position.y = init_pose[1]
        x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, init_pose[2])
        pose0.orientation.x = x
        pose0.orientation.y = y
        pose0.orientation.z = z
        pose0.orientation.w = w
        twist0.linear.x = 0.
        twist0.angular.z = 0.
        mod0 = ModelState('p3dx0', pose0, twist0, 'world')
        ssm(mod0)

    def configure(self):
        print 'configure controller'
        # robots
        init_ctr = [-3.5, -1.]
        terminal_ctr = [3.5, 1.]
        self.init_pose = [init_ctr[0], init_ctr[1], np.pi/2.]
        self.terminal_pose = [terminal_ctr[0], terminal_ctr[1], 0.0]
        # init gazebo
        self.init_gazebo(self.init_pose)
        self._vehicle = omg.Dubins(shapes=omg.Circle(0.35), options={'degree': 2}, bounds={'vmax': 0.5, 'wmax': np.pi/3., 'wmin': -np.pi/3.})
        self._vehicle.set_initial_conditions(self.init_pose)
        self._vehicle.set_terminal_conditions(self.terminal_pose)
        # environment
        room = {'shape': omg.Rectangle(10.0, 5.0)}
        self._obstacles = []
        shape = omg.Beam(width=4.0, height=0.1, orientation=np.pi/2.0)
        self._obstacles.append(omg.Obstacle({'position': [-2.0, -2.3]}, shape=shape))

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
    rospy.init_node('p3dx_motionplanner')
    sample_time = 0.01
    update_time = 0.5
    controller = SimpleController(sample_time, update_time)
    rospy.sleep(0.5)
    controller.configure()
    controller.start()
