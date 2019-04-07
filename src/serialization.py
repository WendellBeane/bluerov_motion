import json
import numpy as np

def serialize(pose_list):
    """ pose_list: poses (time, position, orientation) from Controller.py """
        
    # convert unserializable numpy arrays to lists
    serializable_poses = []
    for pose in pose_list:
        if pose != "NO MARKER":
            time, position, orientation = pose
            pose = (time, position.tolist(), orientation.tolist())
        serializable_poses.append(pose)

    with open("data.json", "w") as write_file:
        json.dump(serializable_poses, write_file)


def deserialize():

    with open("data.json") as read_file:
        data = json.load(read_file)

    # convert back into numpy arrays
    for index, pose in enumerate(data):
        if pose != "NO MARKER":
            time, position, orientation = pose
            data[index] = (time, 
                np.array(position), 
                np.array(orientation))
    
    return data
        
