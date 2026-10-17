#!/usr/bin/env python3
"""
Traffic light controller.

Default cycle: RED 30 s, GREEN 25 s, YELLOW 5 s (60 s total).
Light 2 runs 15 s behind light 1 (phase_offset).

    pub : /traffic_light/command (aic26_msgs/TrafficLightCommand)
    pub : /traffic_light/state   (aic26_msgs/TrafficLightState)
"""

import rospy
from aic26_msgs.msg import TrafficLightCommand, TrafficLightState

DEFAULT_DURATION = {
    'RED': 30.0,
    'GREEN': 25.0,
    'YELLOW': 5.0,
}


def load_duration():
    """Merge the ~durations parameter over the built-in defaults."""
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

CYCLE = sum(DURATION.values())

# Start time of every phase within the default cycle.
PHASE_STARTS = {}
_acc = 0.0
for _p in PHASE_ORDER:
    PHASE_STARTS[_p] = _acc
    _acc += DURATION[_p]

# Lens material of each phase.
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
    return PHASE_ORDER[-1]


def publish_phase(pub, model_name, phase):
    for light_name, material_name in MATERIALS[phase].items():
        msg = TrafficLightCommand()
        msg.model_name = model_name
        msg.light_name = light_name
        msg.material_name = material_name
        pub.publish(msg)


def publish_state(pub, model_name, phase):
    # Ground truth state, used by the mission controller as a fallback.
    msg = TrafficLightState()
    msg.model_name = model_name
    msg.state = phase
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
    state_topic = rospy.get_param('~state_topic', '/traffic_light/state')
    rate_hz = rospy.get_param('~rate_hz', 20.0)
    phase_offset = rospy.get_param('~phase_offset', 15.0)
    # Wait after the first subscriber connects before starting the clock:
    # gzclient loads the world asynchronously and can apply materials only
    # after its render scene exists. 0 disables the wait.
    startup_delay = rospy.get_param('~startup_delay', 5.0)
    # Re-publish the full state at this period (seconds) so a late subscriber
    # (e.g. a gzclient that finished loading after us) converges without
    # waiting for the next phase change. <= 0 disables the refresh.
    refresh_period = rospy.get_param('~refresh_period', 2.0)

    lights = [
        {'model': 'traffic_light_1', 'offset': 0.0},
        {'model': 'traffic_light_2', 'offset': phase_offset},
    ]

    rospy.loginfo("phase_offset = %.1fs (light2 behind light1)", phase_offset)

    pub = rospy.Publisher(cmd_topic, TrafficLightCommand,
                          queue_size=10, latch=True)
    state_pub = rospy.Publisher(state_topic, TrafficLightState,
                                queue_size=10, latch=True)

    # Wait until the visual plugin has subscribed.
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.05)

    # gzclient may still be building the render scene when the subscriber
    # appears; wait a moment so the first command is not lost by the visual
    # plugin.
    if startup_delay > 0 and not rospy.is_shutdown():
        rospy.sleep(startup_delay)

    rospy.loginfo(
        "traffic_controller started: cycle=%.0fs (R=%.0f G=%.0f Y=%.0f)",
        CYCLE, DURATION['RED'], DURATION['GREEN'], DURATION['YELLOW'])
    for cfg in lights:
        rospy.loginfo("  %s: offset=+%.0fs", cfg['model'], cfg['offset'])

    # Publish a light only when its phase changes (or a refresh is due).
    last_phase = {cfg['model']: None for cfg in lights}
    last_refresh = 0.0

    start_time = rospy.Time.now()
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        global_t = (rospy.Time.now() - start_time).to_sec()

        refresh_due = (refresh_period > 0.0 and
                       global_t - last_refresh >= refresh_period)

        for cfg in lights:
            local_t = global_t - cfg['offset']
            phase = phase_at(local_t)

            if phase != last_phase[cfg['model']] or refresh_due:
                publish_phase(pub, cfg['model'], phase)
                publish_state(state_pub, cfg['model'], phase)
                if phase != last_phase[cfg['model']]:
                    rospy.loginfo(
                        "[%s] -> %-6s  (global=%6.1fs  local=%6.1fs)",
                        cfg['model'], phase, global_t, local_t)
                last_phase[cfg['model']] = phase

        if refresh_due:
            last_refresh = global_t

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
