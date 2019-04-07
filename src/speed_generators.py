import numpy as np
from helpers import normalize

def NORM(desired_pose_change):

	xyz_change, orient_change = desired_pose_change
	
	SPEED = 65
	x, y, z = normalize(xyz_change) * SPEED
	_, pitch, _ = normalize(orient_change) * SPEED * -0.1

	command_dict = {"left": x, 
					"up": y, 
					"backward": z,
					"rotate right": pitch}
	# NOTE: we are IGNORING dimen 4 for now
	# NOTE: we are IGNORING dimen 6 for now

	return command_dict
