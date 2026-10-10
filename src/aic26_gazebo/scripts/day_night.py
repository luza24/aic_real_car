#!/usr/bin/env python3

"""Switches the simulated scene between day and night at runtime.

The plugin reads /day_night every frame, so running this while the simulation
is up changes the sun and the headlights live. Launching the world already sets
the initial value through the day_night launch argument.

    rosrun aic26_gazebo day_night.py _mode:=night
"""

import rospy

def main():
    rospy.init_node('day_night')

    mode = rospy.get_param('~mode', '')
    if mode not in ('day', 'night'):
        rospy.logerr("_mode must be 'day' or 'night', got '%s'", mode)
        return

    rospy.set_param('/day_night', mode)
    rospy.loginfo("/day_night = %s", mode)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
