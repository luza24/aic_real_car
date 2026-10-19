"""
Dataset capture cycle.

Drives the robot along the inspection route
(config/<backend>_screenshot_cycle.yaml mirrors config/<backend>_tasks.yaml)
and saves labeled screenshots at every stop. The vision detectors are
replaced by a directly published result, so the whole cycle runs end to end
without any model and the images are stored under

    .datasets/screenshots/<label>/<label>_%04i.jpg

    pub : /perception/result   (std_msgs/String, json, faked vision output)
    pub : /perception/detect   (std_msgs/String, detection trigger)
    sub : /traffic_light/state (aic26_msgs/TrafficLightState, sim truth)
    sub : /camera/image_raw    (sensor_msgs/Image)

The navigation / rotation / traffic-light logic comes from mission_common,
shared with the other aic26_decision nodes.
"""

import json
import os
import subprocess
import sys

import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_common import RobotDriver      # noqa: E402

try:
    import rospkg
    HAVE_ROSPKG = True
except ImportError:
    rospkg = None
    HAVE_ROSPKG = False


class ScreenshotCycle(RobotDriver):
    def __init__(self):
        light_topic = rospy.get_param('~light_topic', '/traffic_light/state')
        super(ScreenshotCycle, self).__init__(light_topic=light_topic)

        self.waypoints = self.load_waypoints(
            rospy.get_param('~waypoints_file'))
        self.stops = self.load_stops(rospy.get_param('~route_file'))
        self.out_dir = rospy.get_param('~output_dir')

        self.shots = 0
        self.frame = None

        # per label shot bookkeeping so the cycle can loop until every stop has
        # reached its target count (see run())
        self.targets = {s.get("label", s["name"]): int(s.get("shots", 1))
                        for s in self.stops}
        self.counts = dict((label, 0) for label in self.targets)
        self.cycle = 0
        self.until_done = rospy.get_param('~until_done', True)

        # advance the sky box time by this many hours after every cycle so the
        # dataset covers different lighting (see advance_sky_time)
        self.sky_step = rospy.get_param('~sky_step', 1.1)
        self.day_night_script = self.find_day_night_script()
        self.resume()

        self.bridge = CvBridge()

        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.detect_pub = rospy.Publisher(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, queue_size=10)

        rospy.Subscriber(rospy.get_param('~image_topic', '/camera/image_raw'),
                         Image, self.on_image)

    # ------------------------------------------------------------------ io
    def load_waypoints(self, path):
        with open(path) as f:
            return {w["name"]: w for w in yaml.safe_load(f)["waypoints"]}

    def load_stops(self, path):
        with open(path) as f:
            return yaml.safe_load(f)["waypoints"]

    def on_image(self, msg):
        self.frame = msg

    # --------------------------------------------------------------- capture
    def save_shot(self, label):
        if self.frame is None:
            rospy.logwarn("no camera frame for %s", label)
            return
        folder = os.path.join(self.out_dir, label)
        if not os.path.isdir(folder):
            os.makedirs(folder)

        # resume from what is already on disk: the cycle is expected to be run
        # several times (or in a loop) until every label has enough images
        index = self.counts.get(label, 0)
        while os.path.exists(os.path.join(folder,
                                          "%s_%04i.jpg" % (label, index + 1))):
            index += 1
        index += 1

        name = "%s_%04i.jpg" % (label, index)
        path = os.path.join(folder, name)
        try:
            import cv2
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            cv2.imwrite(path, frame)
            self.shots += 1
            self.counts[label] = index
            rospy.loginfo("saved %s (%d/%d)", path, index,
                          self.targets.get(label, index))
        except Exception as err:
            rospy.logwarn("failed to save %s: %s", path, err)

    def fake_vision(self, stop):
        # replace the detector output with the planned result
        result = stop.get("result")
        if not result:
            return
        try:
            target = json.loads(result).get("target", "")
        except ValueError:
            target = ""

        if target:
            trigger = String()
            trigger.data = target
            self.detect_pub.publish(trigger)

        msg = String()
        msg.data = result
        self.result_pub.publish(msg)
        rospy.loginfo("fake result: %s", result)

    # ------------------------------------------------------------------ tour
    def visit(self, stop):
        name = stop["name"]
        if name not in self.waypoints:
            rospy.logwarn("unknown waypoint %s, skipping", name)
            return
        wp = self.waypoints[name]

        rospy.loginfo("stop %s at %s", stop.get("label", name), name)
        if not self.go_to(wp):
            rospy.logwarn("failed to reach %s", name)
            return

        turn = stop.get("turn")
        if turn in ("left", "right"):
            self.turn_in_place(wp, turn)
        if stop.get("light"):
            self.wait_green(stop["light"], stop.get("wait", 30.0))

        rospy.sleep(stop.get("dwell", 0.0))
        self.fake_vision(stop)

        label = stop.get("label", name)
        # In loop mode only top up the labels that are still short, so a long
        # keyboard interrupt does not leave one stop with a huge surplus.
        want = self.targets.get(label, 1)
        if self.until_done:
            todo = max(0, want - self.counts.get(label, 0))
            todo = min(todo, int(stop.get("shots_per_pass",
                                          stop.get("shots", 1))))
        else:
            todo = int(stop.get("shots", 1))

        for _ in range(todo):
            self.save_shot(label)
            rospy.sleep(rospy.get_param('~shot_interval', 0.5))

        # face the waypoint heading again so the next go_to starts from the
        # planned orientation instead of the inspection heading
        if turn in ("left", "right") and not self.turn_back(wp):
            self.stop_motion()
            rospy.logwarn("could not restore heading at %s, holding", name)

    def pending(self):
        # labels that still need images, sorted so the summary is stable
        return {label: self.targets[label] - self.counts.get(label, 0)
                for label in sorted(self.targets)
                if self.counts.get(label, 0) < self.targets[label]}

    # ------------------------------------------------------------ sky / state
    def find_day_night_script(self):
        """Locate aic26_gazebo/scripts/day_night.py, or None if not found."""
        explicit = rospy.get_param('~day_night_script', '')
        if explicit:
            return explicit if os.path.isfile(explicit) else None
        if HAVE_ROSPKG:
            try:
                candidate = os.path.join(
                    rospkg.RosPack().get_path('aic26_gazebo'),
                    'scripts', 'day_night.py')
                if os.path.isfile(candidate):
                    return candidate
            except rospkg.ResourceNotFound:
                pass
        return None

    def advance_sky_time(self):
        """Move the sky box time forward by ~sky_step hours via day_night.py.

        Calls the day_night script with _time:=<current + step>; the script
        wraps the value into [0, 24) and writes /sky_time, which the gazebo
        light_switch plugin turns into the actual sun / headlights.
        """
        if not self.sky_step:
            return
        if not self.day_night_script:
            rospy.logwarn('day_night.py not found, sky time not advanced')
            return

        nxt = (rospy.get_param('/sky_time', 10.0) +
               float(self.sky_step)) % 24.0
        try:
            subprocess.call(['python3', self.day_night_script,
                             '_time:=%s' % nxt])
            rospy.loginfo('sky time advanced to %.2fh', nxt)
        except OSError as err:
            rospy.logwarn('could not run day_night.py: %s', err)

    def resume(self):
        """Count existing images so a re-run tops up, not refills.

        Filenames are <label>_%04i.jpg, so the highest index tells us how many
        shots this label already has. Without this a fresh process would start
        at zero and write a whole extra batch on top of a finished label.
        """
        for label in self.targets:
            folder = os.path.join(self.out_dir, label)
            index = 0
            while os.path.exists(
                    os.path.join(folder, "%s_%04i.jpg" % (label, index + 1))):
                index += 1
            self.counts[label] = index
        if any(self.counts.values()):
            rospy.loginfo("resuming, found: %s",
                          ", ".join("%s=%d/%d" % (l, self.counts[l],
                                                  self.targets[l])
                                    for l in sorted(self.targets)
                                    if self.counts[l]))

    def progress(self):
        want = sum(self.targets.values())
        got = sum(min(self.counts.get(label, 0), self.targets[label])
                  for label in self.targets)
        return 100.0 * got / want if want else 100.0

    # ------------------------------------------------------------------ run
    def run(self):
        self.wait_ready()
        rospy.loginfo("capture cycle started, %d stops, targets: %s",
                      len(self.stops),
                      ", ".join("%s=%d" % kv
                                for kv in sorted(self.targets.items())))

        try:
            while not rospy.is_shutdown():
                self.cycle += 1
                rospy.loginfo("cycle %d starting (%.0f%% done)",
                              self.cycle, self.progress())
                for stop in self.stops:
                    if rospy.is_shutdown():
                        break
                    self.visit(stop)

                # move the sky box forward so the next pass sees new lighting
                self.advance_sky_time()

                if not self.until_done:
                    break

                missing = self.pending()
                if not missing:
                    rospy.loginfo(
                        "all capture targets reached after %d cycles",
                        self.cycle)
                    break
                rospy.loginfo("cycle %d done, still short: %s",
                              self.cycle,
                              ", ".join("%s(+%d)" % kv
                                        for kv in missing.items()))
        except KeyboardInterrupt:
            rospy.loginfo("capture interrupted by KeyboardInterrupt")

        missing = self.pending()
        if missing:
            rospy.logwarn("finished with %d shots, still short: %s",
                          self.shots,
                          ", ".join("%s(+%d)" % kv for kv in missing.items()))
        else:
            rospy.loginfo("capture cycle finished, %d shots, all targets met",
                          self.shots)


def main():
    rospy.init_node("capture_cycle")
    ScreenshotCycle().run()


if __name__ == "__main__":
    main()
