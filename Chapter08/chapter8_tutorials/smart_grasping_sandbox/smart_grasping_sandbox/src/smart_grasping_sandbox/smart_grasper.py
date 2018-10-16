import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, DeleteModel, \
    SpawnModel
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from moveit_commander import MoveGroupCommander
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy
import time
from tf_conversions import posemath, toMsg
import PyKDL
from threading import Timer


class SmartGrasper(object):
    """
    This is the helper library to easily access the different functionalities of the simulated robot
    from python.
    """

    __last_joint_state = None
    __current_model_name = "cricket_ball"
    __path_to_models = "/root/.gazebo/models/"
    
    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics and services
        and resets the world to start in a good position.
        """
        rospy.init_node("smart_grasper")
        
        self.__joint_state_sub = rospy.Subscriber("/joint_states", JointState, 
                                                  self.__joint_state_cb, queue_size=1)

        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        rospy.wait_for_service("/gazebo/reset_world", 10.0)
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("/gazebo/pause_physics")
        self.__pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.__unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.__switch_ctrl = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        rospy.wait_for_service("/gazebo/set_model_configuration")
        self.__set_model = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        
        rospy.wait_for_service("/gazebo/delete_model")
        self.__delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.__spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        
        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.__get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.__pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

        self.arm_commander = MoveGroupCommander("arm")
        self.hand_commander = MoveGroupCommander("hand")
        
        self.__hand_traj_client = SimpleActionClient("/hand_controller/follow_joint_trajectory", 
                                                     FollowJointTrajectoryAction)
        self.__arm_traj_client = SimpleActionClient("/arm_controller/follow_joint_trajectory", 
                                                    FollowJointTrajectoryAction)
                                              
        if self.__hand_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
            rospy.logfatal("Failed to connect to /hand_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /hand_controller/follow_joint_trajectory in 4sec.")
                                              
        if self.__arm_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
            rospy.logfatal("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")

        
        self.reset_world()

    def reset_world(self):
        """
        Resets the object poses in the world and the robot joint angles.
        """
        self.__switch_ctrl.call(start_controllers=[], 
                                stop_controllers=["hand_controller", "arm_controller", "joint_state_controller"], 
                                strictness=SwitchControllerRequest.BEST_EFFORT)
        self.__pause_physics.call()
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'H1_F1J1', 'H1_F1J2', 'H1_F1J3', 
                       'H1_F2J1', 'H1_F2J2', 'H1_F2J3', 'H1_F3J1', 'H1_F3J2', 'H1_F3J3']
        joint_positions = [1.2, 0.3, -1.5, -0.5, -1.5, 0.0, 0.0, -0.3, 0.0, 0.0, -0.3, 0.0, 0.0, -0.3, 0.0]
        
        self.__set_model.call(model_name="smart_grasping_sandbox", 
                              urdf_param_name="robot_description",
                              joint_names=joint_names, 
                              joint_positions=joint_positions)
            
        timer = Timer(0.0, self.__start_ctrl)
        timer.start()
        
        time.sleep(0.1)
        self.__unpause_physics.call()

        self.__reset_world.call()

    def get_object_pose(self):
        """
        Gets the pose of the ball in the world frame.
        
        @return The pose of the ball.
        """
        return self.__get_pose_srv.call(self.__current_model_name, "world").pose

    def get_tip_pose(self):
        """
        Gets the current pose of the robot's tooltip in the world frame.
        @return the tip pose
        """
        return self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose

    def move_tip_absolute(self, target):
        """
        Moves the tooltip to the absolute target in the world frame

        @param target is a geometry_msgs.msg.Pose
        @return True on success
        """
        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([target])
        plan = self.arm_commander.plan()
        if not self.arm_commander.execute(plan):
            return False
        return True
        
    def move_tip(self, x=0., y=0., z=0., roll=0., pitch=0., yaw=0.):
        """
        Moves the tooltip in the world frame by the given x,y,z / roll,pitch,yaw. 

        @return True on success
        """
        transform = PyKDL.Frame(PyKDL.Rotation.RPY(pitch, roll, yaw),
                                PyKDL.Vector(-x, -y, -z))
        
        tip_pose = self.get_tip_pose()
        tip_pose_kdl = posemath.fromMsg(tip_pose)
        final_pose = toMsg(tip_pose_kdl * transform)
            
        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([final_pose])
        plan = self.arm_commander.plan()
        if not  self.arm_commander.execute(plan):
            return False
        return True

    def send_command(self, command, duration=0.2):
        """
        Send a dictionnary of joint targets to the arm and hand directly.
        
        @param command: a dictionnary of joint names associated with a target:
                        {"H1_F1J1": -1.0, "shoulder_pan_joint": 1.0}
        @param duration: the amount of time it will take to get there in seconds. Needs to be bigger than 0.0
        """
        hand_goal = None
        arm_goal = None
        
        for joint, target in command.items():
            if "H1" in joint:
                if not hand_goal:
                    hand_goal = FollowJointTrajectoryGoal()
                    
                    point = JointTrajectoryPoint()
                    point.time_from_start = rospy.Duration.from_sec(duration)
                    
                    hand_goal.trajectory.points.append(point)
                    
                hand_goal.trajectory.joint_names.append(joint)
                hand_goal.trajectory.points[0].positions.append(target)
            else:
                if not arm_goal:
                    arm_goal = FollowJointTrajectoryGoal()
                    
                    point = JointTrajectoryPoint()
                    point.time_from_start = rospy.Duration.from_sec(duration)
                    
                    arm_goal.trajectory.points.append(point)
                    
                arm_goal.trajectory.joint_names.append(joint)
                arm_goal.trajectory.points[0].positions.append(target)
        
        if arm_goal:
            self.__arm_traj_client.send_goal_and_wait(arm_goal)
        if hand_goal:
            self.__hand_traj_client.send_goal_and_wait(hand_goal)

    def get_current_joint_state(self):
        """
        Gets the current state of the robot. 
        
        @return joint positions, velocity and efforts as three dictionnaries
        """
        joints_position = {n: p for n, p in
                           zip(self.__last_joint_state.name,
                               self.__last_joint_state.position)}
        joints_velocity = {n: v for n, v in
                           zip(self.__last_joint_state.name,
                           self.__last_joint_state.velocity)}
        joints_effort = {n: v for n, v in
                         zip(self.__last_joint_state.name, 
                         self.__last_joint_state.effort)}
        return joints_position, joints_velocity, joints_effort

    def open_hand(self):
        """
        Opens the hand.
        
        @return True on success
        """
        self.hand_commander.set_named_target("open")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        return True

    def close_hand(self):
        """
        Closes the hand.
        
        @return True on success
        """
        self.hand_commander.set_named_target("close")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        return True

    def check_fingers_collisions(self, enable=True):
        """
        Disables or enables the collisions check between the fingers and the objects / table
        
        @param enable: set to True to enable / False to disable
        @return True on success
        """
        objects = ["cricket_ball__link", "drill__link", "cafe_table__link"]

        while self.__pub_planning_scene.get_num_connections() < 1:
            rospy.loginfo("waiting for someone to subscribe to the /planning_scene")
            rospy.sleep(0.1)

        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.__get_planning_scene(request)
        acm = response.scene.allowed_collision_matrix

        for object_name in objects:
            if object_name not in acm.entry_names:
                # add object to allowed collision matrix
                acm.entry_names += [object_name]
                for row in range(len(acm.entry_values)):
                    acm.entry_values[row].enabled += [False]

                new_row = deepcopy(acm.entry_values[0])
                acm.entry_values += {new_row}

        for index_entry_values, entry_values in enumerate(acm.entry_values):
            if "H1_F" in acm.entry_names[index_entry_values]:
                for index_value, _ in enumerate(entry_values.enabled):
                    if acm.entry_names[index_value] in objects:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
            elif acm.entry_names[index_entry_values] in objects:
                for index_value, _ in enumerate(entry_values.enabled):
                    if "H1_F" in acm.entry_names[index_value]:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
        
        planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        self.__pub_planning_scene.publish(planning_scene_diff)
        rospy.sleep(1.0)

        return True

    def pick(self):
        """
        Does its best to pick the ball.
        """
        rospy.loginfo("Moving to Pregrasp")
        self.open_hand()
        time.sleep(0.1)
        
        ball_pose = self.get_object_pose()
        ball_pose.position.z += 0.5
        
        #setting an absolute orientation (from the top)
        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        ball_pose.orientation.x = quaternion[0]
        ball_pose.orientation.y = quaternion[1]
        ball_pose.orientation.z = quaternion[2]
        ball_pose.orientation.w = quaternion[3]
        
        self.move_tip_absolute(ball_pose)
        time.sleep(0.1)
        
        rospy.loginfo("Grasping")
        self.move_tip(y=-0.164)
        time.sleep(0.1)
        self.check_fingers_collisions(False)
        time.sleep(0.1)
        self.close_hand()
        time.sleep(0.1)
        
        rospy.loginfo("Lifting")
        for _ in range(5):
            self.move_tip(y=0.01)
            time.sleep(0.1)
            
        self.check_fingers_collisions(True)
        
    def swap_object(self, new_model_name):
        """
        Replaces the current object with a new one.Replaces
        
        @new_model_name the name of the folder in which the object is (e.g. beer)
        """
        try:
            self.__delete_model(self.__current_model_name)
        except:
            rospy.logwarn("Failed to delete: " + self.__current_model_name)
        
        try:
            sdf = None
            initial_pose = Pose()
            initial_pose.position.x = 0.15
            initial_pose.position.z = 0.82
            
            with open(self.__path_to_models + new_model_name + "/model.sdf", "r") as model:
                sdf = model.read()
            res = self.__spawn_model(new_model_name, sdf, "", initial_pose, "world")
            rospy.logerr( "RES: " + str(res) )
            self.__current_model_name = new_model_name
        except:
            rospy.logwarn("Failed to delete: " + self.__current_model_name)
   
   

    def __compute_arm_target_for_ball(self):
        ball_pose = self.get_object_pose()

        # come at it from the top
        arm_target = ball_pose
        arm_target.position.z += 0.5

        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        arm_target.orientation.x = quaternion[0]
        arm_target.orientation.y = quaternion[1]
        arm_target.orientation.z = quaternion[2]
        arm_target.orientation.w = quaternion[3]

        return arm_target

    def __pre_grasp(self, arm_target):
        self.hand_commander.set_named_target("open")
        plan = self.hand_commander.plan()
        self.hand_commander.execute(plan, wait=True)

        for _ in range(10):
            self.arm_commander.set_start_state_to_current_state()
            self.arm_commander.set_pose_targets([arm_target])
            plan = self.arm_commander.plan()
            if self.arm_commander.execute(plan):
                return True

    def __grasp(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z -= 0.12
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.arm_commander.execute(plan):
            return False

        self.hand_commander.set_named_target("close")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        self.hand_commander.attach_object("cricket_ball__link")

    def __lift(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z += 0.1
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.arm_commander.execute(plan):
            return False

    def __start_ctrl(self):
        rospy.loginfo("STARTING CONTROLLERS")
        self.__switch_ctrl.call(start_controllers=["hand_controller", "arm_controller", "joint_state_controller"], 
                                stop_controllers=[], strictness=1)
                                
    def __joint_state_cb(self, msg):
        self.__last_joint_state = msg
