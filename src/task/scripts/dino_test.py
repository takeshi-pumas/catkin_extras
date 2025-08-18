from smach_utils_receptionist import *
import matplotlib.pyplot as plt


voice.talk('Scanning table')
#head.set_joint_values([0.0, -0.3])

favorite_drink = "kuat" 
res,position = get_favorite_drink_location(favorite_drink)

if res:
    voice.talk(f"I found a {favorite_drink} on the {position}, take it please.")
else:
    voice.talk(f'There is no {favorite_drink}, if you want to, take another one.')