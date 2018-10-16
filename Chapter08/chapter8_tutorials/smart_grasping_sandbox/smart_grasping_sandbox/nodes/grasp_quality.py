from smart_grasping_sandbox.smart_grasper import SmartGrasper
from tf.transformations import quaternion_from_euler
from math import pi
import time
import rospy
from math import sqrt, pow
import random
from sys import argv

sgs = SmartGrasper()

MIN_LIFT_STEPS = 5
MAX_BALL_DISTANCE = 0.6

CLOSED_HAND = {}

CLOSED_HAND["H1_F1J1"] = 0.0
CLOSED_HAND["H1_F1J2"] = 0.25
CLOSED_HAND["H1_F1J3"] = 0.4
CLOSED_HAND["H1_F2J1"] = 0.0
CLOSED_HAND["H1_F2J2"] = 0.25
CLOSED_HAND["H1_F2J3"] = 0.4
CLOSED_HAND["H1_F3J1"] = 0.0
CLOSED_HAND["H1_F3J2"] = 0.25
CLOSED_HAND["H1_F3J3"] = 0.4

JOINT_NAMES = CLOSED_HAND.keys()

class GraspQuality(object):
    def __init__(self, sgs):
        self.sgs = sgs
        self.last_distance = None

    def check_stable(self, joint_names):
        current_min = 1000
        positions = []
        velocities = []
        efforts = []
        for k in range(30):
            sgs.move_tip(y=0.02)
            ball_distance = self.__compute_euclidean_distance()
            if k > MIN_LIFT_STEPS and ball_distance < current_min:
                current_min = ball_distance
            if ball_distance > MAX_BALL_DISTANCE:
                break
            joints_positions, joints_velocity, joints_effort = self.sgs.get_current_joint_state()

            new_pos = []
            new_vel = []
            new_eff = []
            for name in joint_names:
                new_pos.append(joints_positions[name])
                new_vel.append(joints_velocity[name])
                new_eff.append(joints_effort[name])
            positions.append(new_pos)
            velocities.append(new_vel)
            efforts.append(new_eff)

            time.sleep(0.01)
        robustness = (1/(current_min - 0.18))**2
        return robustness, positions, velocities, efforts

    def __compute_euclidean_distance(self):
        ball_pose = self.sgs.get_object_pose()
        hand_pose = self.sgs.get_tip_pose()
        dist = sqrt((hand_pose.position.x - ball_pose.position.x)**2 + \
                     (hand_pose.position.y - ball_pose.position.y)**2 + \
                     (hand_pose.position.z - ball_pose.position.z)**2)
        return dist

quality = GraspQuality(sgs)

def experiment(grasp_distance=-0.163):
    sgs.reset_world()
    time.sleep(0.1)
    sgs.reset_world()
    time.sleep(0.1)

    sgs.open_hand()
    time.sleep(0.1)
    sgs.open_hand()
    time.sleep(0.1)

    ball_pose = sgs.get_object_pose()
    ball_pose.position.z += 0.5

    #setting an absolute orientation (from the top)
    quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
    ball_pose.orientation.x = quaternion[0]
    ball_pose.orientation.y = quaternion[1]
    ball_pose.orientation.z = quaternion[2]
    ball_pose.orientation.w = quaternion[3]

    sgs.move_tip_absolute(ball_pose)

    sgs.move_tip(y=grasp_distance)

    # close the grasp
    sgs.check_fingers_collisions(False)

    random_closed_hand = {}
    for joint in CLOSED_HAND:
        random_closed_hand[joint] = CLOSED_HAND[joint] + random.gauss(0.0, 0.04)

    sgs.send_command(random_closed_hand, duration=1.0)

    # lift slowly and check the quality
    joint_names = random_closed_hand.keys()
    joint_targets = random_closed_hand.values()

    robustness, positions, velocities, efforts = quality.check_stable(joint_names)

    rospy.loginfo("Grasp quality = " + str(robustness))

    sgs.check_fingers_collisions(True)
    return joint_names, joint_targets, robustness, positions, velocities, efforts


with open("/results/headers.txt", "wb") as txt_file:
    headers = "experiment_number; robustness; "
    for name in JOINT_NAMES:
        headers += name+"_pos ; "+name+"_vel ; "+name+"_eff ; "
    headers += "measurement_number\n"

    txt_file.write(headers)

grasp_distances = [float(i) for i in argv[1:-1]]
number_of_tests_per_distance = int(argv[-1])
print "Running the grasp script with the distances: ", grasp_distances, " / number of tests: ", number_of_tests_per_distance

import uuid

for dist in grasp_distances:
    for _ in range(number_of_tests_per_distance):
        rospy.loginfo("---- grasping ["+str(uuid.uuid4().hex)+"/"+str(len(grasp_distances*number_of_tests_per_distance))+"] - dist="+str(dist))
        joint_names, joint_targets, robustness, positions, velocities, efforts = experiment(dist)

        with open("/results/"+str(uuid.uuid4())+".txt", "a") as txt_file:
            base_line = str(uuid.uuid4().hex)+" ; "+str(robustness)

            for measurement_number in range(len(positions)):
                pos = positions[measurement_number]
                vel = velocities[measurement_number]
                eff = efforts[measurement_number]

                measurement_line = base_line + " ; "
                for id_name, _ in enumerate(joint_names):
                    measurement_line += str(pos[id_name]) + " ; " + str(vel[id_name])+" ; "+str(eff[id_name])+ " ; "
                measurement_line += str(measurement_number) + "\n"

                txt_file.write(measurement_line)
                measurement_number += 1
