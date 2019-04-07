from mavros_msgs.msg import OverrideRCIn


RC_NEUTRAL = 1500

# Mapping from direction words to tuples containing
#   a) the index of the appropriate channel in the RC override message
#   b) either -1 or 1, to indicate whether the channel value should 
#      be above or below the neutral value
CHANNEL_INFO = {
    "forward":  (4, +1),
    "backward": (4, -1),
    "right":    (5, +1),
    "left":     (5, -1), 
    "down":     (2, -1),
    "up":       (2, +1),
    "rotate left":  (3, -1),
    "rotate right": (3, +1)
}

def neutral_command():
    msg = OverrideRCIn()
    for i in range(8):
        msg.channels[i] = RC_NEUTRAL
    return msg