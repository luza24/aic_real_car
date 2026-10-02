#!/usr/bin/env python3

"""
Traffic light controller.

Cycle:  R = 30s, G = 25s, Y = 5s  (total 60s)

Light 2 runs 15s BEHIND light 1:
    local_t = global_t - offset

Publishes aic26_msgs/TrafficLightCommand on /traffic_light/command.
"""

import rospy
from aic26_msgs.msg import TrafficLightCommand

DEFAULT_DURATION = {
    'RED':    30.0,
    'GREEN':  25.0,
    'YELLOW':  5.0,
}

def load_duration():
    # read from the "~durations" dictionary, replace the missing params with default values
    raw = rospy.get_param('~durations', {})
    merged = dict(DEFAULT_DURATION)
    for k, v in raw.items():
        key = k.upper()
        if key in merged:
            merged[key] = float(v)
        else:
            rospy.logwarn("Unknown phase '%s' in ~durations, ignored", k)
    return merged

DURATION = dict(DEFAULT_DURATION)

PHASE_ORDER = ['RED', 'GREEN', 'YELLOW']

CYCLE = sum(DURATION.values())          # 60.0

# precompute the initial phase
PHASE_STARTS = {}
_acc = 0.0
for _p in PHASE_ORDER:
    PHASE_STARTS[_p] = _acc
    _acc += DURATION[_p]

# the textures of the three configurations
MATERIALS = {
    'RED': {
        'red_lens':    'traffic_light/red_on',
        'yellow_lens': 'traffic_light/yellow_off',
        'green_lens':  'traffic_light/green_off',
    },
    'GREEN': {
        'red_lens':    'traffic_light/red_off',
        'yellow_lens': 'traffic_light/yellow_off',
        'green_lens':  'traffic_light/green_on',
    },
    'YELLOW': {
        'red_lens':    'traffic_light/red_off',
        'yellow_lens': 'traffic_light/yellow_on',
        'green_lens':  'traffic_light/green_off',
    },
}


def phase_at(local_t):
    t = local_t % CYCLE
    for p in PHASE_ORDER:
        if t < PHASE_STARTS[p] + DURATION[p]:
            return p
    return PHASE_ORDER[-1]     # can't reach it


def publish_phase(pub, model_name, phase):
    for light_name, material_name in MATERIALS[phase].items():
        msg = TrafficLightCommand()
        msg.model_name    = model_name
        msg.light_name    = light_name
        msg.material_name = material_name
        pub.publish(msg)


def main():
    rospy.init_node('traffic_controller')

    global DURATION, CYCLE, PHASE_STARTS
    DURATION = load_duration()
    CYCLE = sum(DURATION.values())

    PHASE_STARTS = {}
    _acc = 0.0
    for _p in PHASE_ORDER:
        PHASE_STARTS[_p] = _acc
        _acc += DURATION[_p]

    cmd_topic = rospy.get_param('~command_topic', '/traffic_light/command')
    rate_hz   = rospy.get_param('~rate_hz', 20.0)
    phase_offset = rospy.get_param('~phase_offset', 15.0)

    lights = [
        {'model': 'traffic_light_1', 'offset': 0.0},
        {'model': 'traffic_light_2', 'offset': phase_offset},
    ]

    rospy.loginfo("phase_offset = %.1fs (light2 behind light1)", phase_offset)

    pub = rospy.Publisher(cmd_topic, TrafficLightCommand, queue_size=10)

    # wait for connection
    rospy.sleep(0.5)

    rospy.loginfo(
        "traffic_controller started: cycle=%.0fs (R=%.0f G=%.0f Y=%.0f)",
        CYCLE, DURATION['RED'], DURATION['GREEN'], DURATION['YELLOW'])
    for cfg in lights:
        rospy.loginfo("  %s: offset=+%.0fs", cfg['model'], cfg['offset'])

    # publish order only when the command chaged
    last_phase = {cfg['model']: None for cfg in lights}

    # t = 0 initialy
    start_time = rospy.Time.now()
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        global_t = (rospy.Time.now() - start_time).to_sec()

        for cfg in lights:
            local_t = global_t - cfg['offset']
            phase = phase_at(local_t)

            if phase != last_phase[cfg['model']]:
                publish_phase(pub, cfg['model'], phase)
                last_phase[cfg['model']] = phase
                rospy.loginfo(
                    "[%s] -> %-6s  (global=%6.1fs  local=%6.1fs)",
                    cfg['model'], phase, global_t, local_t)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass