#!/usr/bin/env python3

import rospy
from ros_utils_py.utils import devprint, keep_alive
import math
from shadow_hand import ShadowHand
from gazebo_msgs.msg import ContactsState
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

def callback(data: ContactsState):
	rospy.loginfo(rospy.get_name() + " I heard " + str(data.states))


def main() -> None:

	# waiting period for robot hand to start up...
	waiting_time: int = 5  # s
	rospy.loginfo(f"waiting {waiting_time} for hand to start...")
	time.sleep(waiting_time)

	# hand finder - gets data from the found hand(s)
	hand_finder: HandFinder = HandFinder()

	# hand config object
	hand_parameters = hand_finder.get_hand_parameters()

	# serial numbers for the hands found
	hand_serial_numbers: list = list(hand_parameters.mapping.keys())

	# the 0'th hand serial number (['1234', '0']) '0' is here a dummy and '1234' is the serial for the simulated hand
	hand_serial: str = hand_serial_numbers[0]

	# get a hand commander, which can communicate with the hand specified by the inputs
	hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)

	# in the hand config object we get the prefix (i.e. chirality) (either 'rh' or 'lh') depending on the type of hand with the serial number
	hand_chirality = hand_parameters.mapping[hand_serial]

	# get all the joint names in the hand with the hand prefix found above (e.g. 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', ...)
	joints = hand_finder.get_hand_joints()[hand_chirality]

	# print
	rospy.logerr(f"these are my joints {joints}...")

	# joint configuration, from base to tip (does this make contact with the pen? yes)
	q: list = [0.0, 0.0, math.pi / 2.0]
 
	q_dict = dict(zip(joints, q))
 
	# try and set the finger q
	try:
		rospy.logerr(f"attempting to set joint value {q}...")
		hand_commander.move_to_joint_value_target(q_dict)
	except:
		rospy.logerr(f"failed to set joint value {q}...")

	# subscribe and print the found contacts
	sub = rospy.Subscriber("/contacts/rh_ff/distal", ContactsState, callback=callback)

	# keep node alive
	keep_alive(rospy.get_name())

if __name__ == '__main__':
	try:
		rospy.init_node("biotac_sim_demo", anonymous=True)
		main()

	except rospy.ROSInterruptException:
		pass
