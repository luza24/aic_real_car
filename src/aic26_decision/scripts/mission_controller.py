"""
Inspection mission controller.

Drives the robot along the route in config/sim_tasks.yaml or
config/real_tasks.yaml (selected by the backend launch argument): navigate
to each waypoint, optionally turn in place to face the target, wait at
traffic lights until they are GREEN, trigger the vision detectors and
announce the results.

    pub : /speech                (std_msgs/String)
    pub : /perception/detect     (std_msgs/String, detection trigger)
    sub : /perception/traffic_light (aic26_msgs/TrafficLightState, vision)
    sub : /traffic_light/state   (aic26_msgs/TrafficLightState, sim truth)
    sub : /perception/result     (std_msgs/String, json detection result)

The motion / rotation / traffic-light logic comes from mission_common, shared
with the other aic26_decision nodes; this node only adds the task list, the
speech and the detection result bookkeeping.

Task types (config/<backend>_tasks.yaml):
    goto           navigate to the waypoint, then continue
    traffic_light  turn (optional), wait at the stop line until GREEN, pass
    inspect        navigate, turn, dwell, trigger detection, report
    report         announce a collected result (no motion)

With ``~gui`` set, a small Tkinter panel is started as well: it shows the
latest annotated detection frame (boxes drawn) and a text summary, and writes
the current results to ``aic26_perception/results`` (overwriting them). The
panel closes the node when its window is closed. The panel is sized in pixels
and scaled by AIC26_GUI_SCALE / ~gui_scale.

``~light_fallback`` can name a generic vision light (for example
``traffic_light``) that is used when no state exists for the requested
``traffic_light_1/2`` model. The hardware launch sets this because the real
robot has no simulation truth.
"""

import json
import os
import sys
import threading
import uuid

import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_common import (GREEN, RobotDriver, SpeechAnnouncer,  # noqa: E402
                            format_result_message, load_waypoints)

try:
    from aic26_msgs.msg import TrafficLightState
    HAVE_LIGHT_MSG = True
except ImportError:
    TrafficLightState = None
    HAVE_LIGHT_MSG = False

SCHEMA_VERSION = 2

# Panel sizes in pixels (the same helper as day_night.py in aic26_gazebo)
GUI_SCALE_ENV = 'AIC26_GUI_SCALE'
PANEL_W, PANEL_H = 640, 360           # image panel, 16:9 like the camera
TEXT_PX, TEXT_COLS, TEXT_ROWS = 13, 40, 18
BUTTON_PX, BUTTON_COLS = 13, 12
PLACEHOLDER_PX = 16


def gui_scale(default=1.0):
    """Size multiplier for the panel: AIC26_GUI_SCALE, else ~gui_scale."""
    raw = os.environ.get(GUI_SCALE_ENV)
    if raw is None:
        raw = rospy.get_param('~gui_scale', default)
    try:
        scale = float(raw)
    except (TypeError, ValueError):
        rospy.logwarn('%s=%r is not a number, using %.2f',
                      GUI_SCALE_ENV, raw, default)
        return default
    return min(max(scale, 0.25), 4.0)


class MissionController(RobotDriver):
    def __init__(self):
        super(MissionController, self).__init__(light_topic=None)

        self.vision_states = {}   # light model -> legacy latched vision state
        self.truth_states = {}    # light model -> state from sim truth
        self.vision_events = {}   # light model -> v2 vision event payload
        self.results = {}         # latest result, keyed by target (GUI/diag)
        self.result_seq = {}      # target -> completed results received
        self.light_fallback = rospy.get_param('~light_fallback', '')

        self.people_detect_timeout = float(
            rospy.get_param('~people_detect_timeout', 4.0))
        self.car_detect_timeout = float(
            rospy.get_param('~car_detect_timeout', 12.0))
        self.protocol_wait = float(rospy.get_param('~protocol_wait', 10.0))

        self.run_uuid = uuid.uuid4().hex[:8]
        self.request_seq = 0
        self.current_request = {}     # target -> request_id
        self.completed = {}           # (target, request_id) -> complete result
        self.result_cv = threading.Condition()
        self.protocols = {}           # target -> schema version
        self.car_ready = {}           # latest car ocr_ready payload

        self.speech_language = rospy.get_param('~speech_language', 'zh')
        self.speech_enabled = bool(rospy.get_param('~speech_enabled', True))
        self.announcer = SpeechAnnouncer(
            rospy.get_param('~speech_topic', '/speech'),
            self.speech_language, self.speech_enabled)
        self.detect_pub = rospy.Publisher(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, queue_size=10)

        rospy.Subscriber(
            rospy.get_param('~result_topic', '/perception/result'),
            String, self.on_result)

        protocol_topics = {
            'people': rospy.get_param(
                '~people_protocol_topic', '/perception/people/protocol'),
            'car': rospy.get_param(
                '~car_protocol_topic', '/perception/car/protocol'),
            'traffic_light': rospy.get_param(
                '~traffic_protocol_topic',
                '/perception/traffic_light/protocol'),
        }
        for target, topic in protocol_topics.items():
            rospy.Subscriber(topic, String, self.on_protocol,
                             callback_args=target, queue_size=1)
        rospy.Subscriber(
            rospy.get_param('~car_ready_topic', '/perception/car/ocr_ready'),
            String, self.on_car_ready, queue_size=1)
        rospy.Subscriber(
            rospy.get_param('~traffic_event_topic',
                            '/perception/traffic_light/event'),
            String, self.on_traffic_event, queue_size=10)

        if HAVE_LIGHT_MSG:
            rospy.Subscriber(
                rospy.get_param('~vision_topic', '/perception/traffic_light'),
                TrafficLightState, self.on_vision)
            rospy.Subscriber(
                rospy.get_param('~truth_topic', '/traffic_light/state'),
                TrafficLightState, self.on_truth)
        else:
            rospy.logwarn("aic26_msgs/TrafficLightState missing, "
                          "traffic lights are treated as green")

    # ------------------------------------------------------------ callbacks
    def on_vision(self, msg):
        if msg.state and msg.state != 'UNKNOWN':
            self.vision_states[msg.model_name] = msg.state

    def on_truth(self, msg):
        self.truth_states[msg.model_name] = msg.state

    def on_protocol(self, msg, target):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("not an object")
            schema = int(payload.get('schema', 0))
        except (TypeError, ValueError):
            rospy.logwarn("bad %s protocol: %s", target, msg.data)
            return
        if schema >= SCHEMA_VERSION:
            self.protocols[target] = schema
            rospy.loginfo("%s detector protocol v%d", target, schema)
        else:
            rospy.logwarn("%s detector protocol too old: %s", target, msg.data)

    def on_car_ready(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("not an object")
        except (TypeError, ValueError):
            rospy.logwarn("bad car ready: %s", msg.data)
            return
        self.car_ready = payload
        level = rospy.loginfo if payload.get('ready') else rospy.logwarn
        level("car OCR ready=%s status=%s reason=%s",
              payload.get('ready'), payload.get('status'),
              payload.get('reason', ''))

    def on_traffic_event(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        model = payload.get('model_name')
        if not model:
            return
        current = self.current_request.get('traffic_light')
        event_id = payload.get('request_id')
        if current is not None and event_id != current:
            return
        if current is None and event_id is not None:
            # A v2 event without an active request is stale.
            return
        self.vision_events[model] = payload

    def on_result(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("bad detection result: %s", msg.data)
            return
        if not isinstance(payload, dict):
            rospy.logwarn("bad detection result: %s", msg.data)
            return
        # keep each target's result apart: car and people both publish on
        # /perception/result and both carry a "total" key, so a flat merge
        # would let one overwrite the other
        target = payload.get("target")
        if not target:
            return
        request_id = payload.get('request_id')
        complete = payload.get('complete')
        with self.result_cv:
            # Legacy and incomplete messages are useful for the GUI/diagnosis
            # but must never satisfy a v2 task request.
            self.results[target] = payload
            if request_id is not None and complete is True:
                self.completed[(target, str(request_id))] = payload
                self.result_seq[target] = self.result_seq.get(target, 0) + 1
                self.result_cv.notify_all()

    def result_for(self, target):
        return self.results.get(target, {})

    def light_state(self, model):
        # Simulation truth is instant and authoritative; vision is only used
        # when truth is unavailable. A v2 event must belong to the current
        # traffic request, otherwise a latched state from the previous stop
        # could be mistaken for this one.
        state = self.truth_states.get(model)
        if state in (GREEN, 'RED', 'YELLOW'):
            return state

        names = [model]
        if self.light_fallback and self.light_fallback not in names:
            names.append(self.light_fallback)
        current = self.current_request.get('traffic_light')
        for name in names:
            event = self.vision_events.get(name)
            if not event:
                continue
            event_id = event.get('request_id')
            if current is not None and event_id != current:
                continue
            state = event.get('state')
            if state in (GREEN, 'RED', 'YELLOW'):
                return state

        # Legacy vision state is only meaningful without an active v2 request.
        if current is None:
            state = self.vision_states.get(model)
            if state in (GREEN, 'RED', 'YELLOW'):
                return state
            if self.light_fallback:
                state = self.vision_states.get(self.light_fallback)
                if state in (GREEN, 'RED', 'YELLOW'):
                    return state
        return 'UNKNOWN'

    # ---------------------------------------------------------------- output
    def speak(self, text):
        return self.announcer.speak(text)

    def build_message(self, task, result=None):
        # ``result`` is the payload received for this stop.  The shared
        # formatter stays silent when there is no usable result, so a
        # missing detection is never announced as zero people.
        target = task.get("target")
        if target in ("people", "car"):
            return format_result_message(
                target, result, self.speech_language)
        return task.get("message", "inspection finished")

    def _publish_detect(self, target, request_id=None):
        req = String()
        req.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': target,
            'request_id': request_id,
        })
        self.detect_pub.publish(req)

    def start_detect(self, target):
        """Start a v2 request and return its request id."""
        self.request_seq += 1
        request_id = '%s:%d' % (self.run_uuid, self.request_seq)
        self.current_request[target] = request_id
        self._publish_detect(target, request_id)
        rospy.loginfo("detect start: %s (%s)", target, request_id)
        return request_id

    def wait_result(self, target, request_id, timeout):
        """Wait for the matching complete result, or return None."""
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        with self.result_cv:
            while not rospy.is_shutdown():
                payload = self.completed.get((target, request_id))
                if payload is not None:
                    return payload
                remaining = (deadline - rospy.Time.now()).to_sec()
                if remaining <= 0.0:
                    rospy.logwarn("no complete %s result for %s within %.1fs",
                                  target, request_id, timeout)
                    return None
                self.result_cv.wait(min(0.1, remaining))
        return None

    def stop_detect(self, target, request_id=None):
        """Tell one detector to stop; a v2 stop only matches its request."""
        if request_id is None:
            request_id = self.current_request.get(target)
        self._publish_detect('stop', request_id)
        self.current_request.pop(target, None)

    def stop_all_detects(self):
        for target in list(self.current_request):
            self.stop_detect(target)

    def wait_until(self, deadline):
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            remaining = (deadline - rospy.Time.now()).to_sec()
            if remaining <= 0.0:
                break
            rospy.sleep(min(0.1, remaining))

    def wait_protocols(self, required):
        """Refuse to start a task loop against old detector code."""
        if not required:
            return True
        deadline = rospy.Time.now() + rospy.Duration(self.protocol_wait)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            missing = [t for t in required
                       if self.protocols.get(t, 0) < SCHEMA_VERSION]
            if not missing:
                return True
            rospy.sleep(0.1)
        missing = [t for t in required
                   if self.protocols.get(t, 0) < SCHEMA_VERSION]
        rospy.logerr("detector protocol v%d missing for: %s",
                     SCHEMA_VERSION, ', '.join(sorted(missing)))
        return False

    def report_car_ready(self):
        """Log the preload state but never let it block the route."""
        status = self.car_ready.get('status', 'unknown')
        if self.car_ready.get('ready'):
            rospy.loginfo("car OCR preload ready (status=%s)", status)
        else:
            rospy.logwarn("car OCR preload not ready (status=%s, reason=%s); "
                          "patrol continues and car stops may retry OCR",
                          status, self.car_ready.get('reason', ''))

    # ---------------------------------------------------------------- tasks
    def run_task(self, task, waypoints):
        ttype = task.get("type", "goto")
        name = task.get("name", ttype)

        if ttype == "report":
            self.speak(task.get("message", "report"))
            return

        wp = waypoints.get(task["waypoint"]) if "waypoint" in task else None
        if wp is None:
            rospy.logwarn("unknown waypoint for task %s, skipping", name)
            return
        if not self.go_to(wp):
            rospy.logwarn("failed to reach %s", name)
            return

        turn = task.get("turn")
        turned = False
        if turn in ("left", "right"):
            # rotate first: at a stop line the camera has to already face the
            # light / people before we start waiting or detecting
            turned = self.turn_in_place(wp, turn)

        turn_failed = turn in ("left", "right") and not turned

        if ttype == "traffic_light":
            # Share one trigger time between the detector request and the
            # green wait; simulation truth can satisfy it immediately.
            self.vision_states.pop(task["light"], None)
            self.vision_events.pop(task["light"], None)
            if self.light_fallback:
                self.vision_states.pop(self.light_fallback, None)
                self.vision_events.pop(self.light_fallback, None)
            trigger_start = rospy.Time.now()
            request_id = self.start_detect("traffic_light")
            self.wait_green(task["light"], task.get("wait", 30.0),
                            request_id=request_id,
                            trigger_start=trigger_start)
            self.stop_detect("traffic_light", request_id)

        if ttype == "inspect":
            target = task.get("target")
            if turn_failed:
                rospy.logwarn("turn failed at %s, skipping %s detection",
                              name, target or "inspection")
            else:
                # Trigger at the beginning of the dwell: camera start-up,
                # inference and OCR overlap with the stop time.
                dwell_deadline = (rospy.Time.now() +
                                  rospy.Duration(float(
                                      task.get("dwell", 0.0))))
                result = None
                request_id = None
                if target:
                    timeout = (self.people_detect_timeout
                               if target == "people"
                               else self.car_detect_timeout)
                    request_id = self.start_detect(target)
                    result = self.wait_result(target, request_id, timeout)
                    self.wait_until(dwell_deadline)
                    if task.get("announce", True):
                        self.speak(self.build_message(task, result))
                    self.stop_detect(target, request_id)
                else:
                    self.wait_until(dwell_deadline)

        # face the waypoint heading again so the next go_to starts from the
        # planned orientation instead of the inspection heading
        if turn in ("left", "right") and not self.turn_back(wp):
            self.stop_motion()
            rospy.logwarn("could not restore heading at %s, holding", name)

    def run(self):
        with open(rospy.get_param("~tasks_file")) as f:
            tasks = yaml.safe_load(f)["tasks"]
        waypoints = {w["name"]: w for w in load_waypoints(
            rospy.get_param("~waypoints_file"))}

        required = set()
        for task in tasks:
            ttype = task.get("type", "goto")
            target = task.get("target")
            if ttype == "traffic_light":
                required.add("traffic_light")
            elif ttype == "inspect" and target in ("people", "car"):
                required.add(target)

        self.wait_ready()
        if not self.wait_protocols(required):
            rospy.logerr("mission not started: detector protocol mismatch")
            return
        if "car" in required:
            # Preloading is an optimisation. It must never hold up the route.
            self.report_car_ready()
        rospy.loginfo("mission started, %d tasks (run %s)",
                      len(tasks), self.run_uuid)

        try:
            for task in tasks:
                if rospy.is_shutdown():
                    break
                if self.localisation_lost or self.motion_blocked:
                    rospy.logerr("localisation lost, aborting the route")
                    return
                rospy.loginfo("task: %s", task.get("name", task.get("type")))
                self.run_task(task, waypoints)
        except KeyboardInterrupt:
            self.stop_motion()
            rospy.loginfo("mission interrupted by KeyboardInterrupt")
        finally:
            self.stop_all_detects()
        rospy.loginfo("mission finished")


class ResultViewer(object):
    """Tkinter panel that shows the detection results and saves them.

    Subscribes to the json results on /perception/result and to the three
    detector debug images (/perception/{people,car,traffic_light}/image) and
    draws the latest annotated frame next to a text summary. Every update is
    written to ``aic26_perception/results`` (overwriting the previous file), so
    the folder always holds the current state.

    The GUI runs in its own thread; when tkinter or a display is missing the
    viewer logs and disables itself instead of taking the mission down.
    """

    IMAGE_TOPICS = {
        'people': '/perception/people/image',
        'car': '/perception/car/image',
        'traffic_light': '/perception/traffic_light/image',
    }

    def __init__(self, out_dir, result_topic='/perception/result'):
        self.out_dir = out_dir
        self.result = {}          # target -> latest json payload
        self.frames = {}          # target -> latest bgr frame (numpy)
        self.lock = threading.Lock()
        self.bridge = _make_bridge()
        self.image_warned = False   # log the first image decode failure only

        os.makedirs(self.out_dir, exist_ok=True)

        rospy.Subscriber(result_topic, String, self.on_result)
        for target, topic in self.IMAGE_TOPICS.items():
            rospy.Subscriber(topic, Image, self.on_image, callback_args=target)

        # the traffic light detector publishes a message, not a json result
        try:
            from aic26_msgs.msg import TrafficLightState
            rospy.Subscriber('/perception/traffic_light', TrafficLightState,
                             self.on_light)
        except ImportError:
            pass

    # ------------------------------------------------------------ callbacks
    def on_light(self, msg):
        payload = {'target': 'traffic_light', 'model_name': msg.model_name,
                   'state': msg.state}
        with self.lock:
            self.result['traffic_light'] = payload
        self.write_json('traffic_light', payload)
        self.write_summary()

    def on_result(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("not an object")
        except ValueError:
            rospy.logwarn("result viewer: bad json: %s", msg.data)
            return
        target = payload.get('target', 'traffic_light')
        complete = payload.get('complete', True)
        with self.lock:
            self.result[target] = payload
        # Progress updates are useful on screen but must not overwrite the
        # final JSON that the next run / reviewer reads.
        if complete:
            self.write_json(target, payload)
            self.write_summary()

    def on_image(self, msg, target):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:                       # noqa: BLE001
            # a persistent decode failure would silently leave the panel empty,
            # so report it once at warning level instead of hiding it at debug
            if not self.image_warned:
                self.image_warned = True
                rospy.logwarn("result viewer: image skipped: %s", err)
            return
        with self.lock:
            self.frames[target] = frame
        self.write_image(target, frame)

    # ------------------------------------------------------------- saving
    def write_json(self, target, payload):
        path = os.path.join(self.out_dir, '%s.json' % target)
        self._atomic_write(path, json.dumps(payload, ensure_ascii=False,
                                            indent=2))

    def write_image(self, target, frame):
        path = os.path.join(self.out_dir, '%s.jpg' % target)
        try:
            import cv2
            cv2.imwrite(path, frame)
        except Exception as err:                       # noqa: BLE001
            rospy.logdebug("result viewer: could not save %s: %s", path, err)

    def write_summary(self):
        with self.lock:
            summary = {'people': self.result.get('people', {}),
                       'car': self.result.get('car', {}),
                       'traffic_light': self.result.get('traffic_light', {})}
        path = os.path.join(self.out_dir, 'summary.json')
        self._atomic_write(path, json.dumps(summary, ensure_ascii=False,
                                            indent=2))

    @staticmethod
    def _atomic_write(path, text):
        """Write via a temp file + rename so readers never see a half file."""
        tmp = path + '.tmp'
        try:
            with open(tmp, 'w') as f:
                f.write(text)
            os.rename(tmp, path)
        except OSError as err:
            rospy.logwarn("result viewer: could not write %s: %s", path, err)

    # ------------------------------------------------------------- display
    def start(self):
        """Start the viewer thread; no-op when tkinter/display is missing."""
        try:
            import tkinter                                   # noqa: F401
        except ImportError:
            rospy.logwarn("result viewer: tkinter missing, GUI disabled")
            return None
        thread = threading.Thread(target=self._run_gui, name='result_viewer')
        thread.daemon = True
        thread.start()
        return thread

    def _run_gui(self):
        import tkinter as tk
        try:
            from PIL import Image as PilImage, ImageTk
        except ImportError:
            PilImage = ImageTk = None
            rospy.logwarn(
                "result viewer: Pillow missing, image panel disabled")

        try:
            root = tk.Tk()
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("result viewer: no display (%s), GUI disabled", err)
            return

        root.title('aic26 detection results')

        scale = gui_scale()

        def px(value):
            """Pixel size for this display (see gui_scale)."""
            return int(round(value * scale))

        # no window manager places the window on an X11 forwarded display
        root.geometry('+%d+%d' % (px(60), px(60)))

        # image panel sized in pixels: a Canvas honours width/height as pixels,
        # unlike a Label (which counts characters/lines and would blow up)
        panel_w, panel_h = px(PANEL_W), px(PANEL_H)
        canvas = tk.Canvas(root, width=panel_w, height=panel_h,
                           bg='#202020', highlightthickness=1,
                           highlightbackground='#555555')
        canvas.grid(row=0, column=0, rowspan=3, padx=6, pady=6, sticky='n')

        # negative font size = pixels in Tk, not points
        text = tk.Text(root, width=TEXT_COLS, height=TEXT_ROWS,
                       font=('TkFixedFont', -px(TEXT_PX)))
        text.grid(row=0, column=1, padx=6, pady=6, sticky='n')

        names = list(self.IMAGE_TOPICS)
        buttons = tk.Frame(root)
        buttons.grid(row=1, column=1, sticky='w', padx=6)
        shown = tk.StringVar(value=names[0])
        for i, name in enumerate(names):
            tk.Radiobutton(buttons, text=name, value=name, variable=shown,
                           indicatoron=False, width=BUTTON_COLS,
                           font=('TkDefaultFont', -px(BUTTON_PX))).grid(
                row=0, column=i)

        status = tk.StringVar(value='waiting for results ...')
        tk.Label(root, textvariable=status, anchor='w',
                 font=('TkDefaultFont', -px(BUTTON_PX))).grid(
            row=2, column=1, sticky='w', padx=6, pady=(0, 6))
        photo_holder = {'photo': None}

        def draw_frame(frame):
            img = PilImage.fromarray(np.ascontiguousarray(frame[:, :, ::-1]))
            img.thumbnail((panel_w, panel_h))
            photo = ImageTk.PhotoImage(img)
            photo_holder['photo'] = photo          # keep a reference alive
            canvas.delete('all')
            canvas.create_image(panel_w // 2, panel_h // 2, image=photo)

        def tick():
            target = shown.get()
            frame = self.frames.get(target)
            if frame is not None and PilImage is not None:
                try:
                    draw_frame(frame)
                except Exception as err:               # noqa: BLE001
                    rospy.logdebug("result viewer: frame draw failed: %s", err)
            else:
                photo_holder['photo'] = None
                canvas.delete('all')
                canvas.create_text(panel_w // 2, panel_h // 2, fill='#bbbbbb',
                                   text='no image yet (%s)' % target,
                                   font=('TkDefaultFont', -px(PLACEHOLDER_PX)))

            text.configure(state='normal')
            text.delete('1.0', 'end')
            text.insert('end', self.summary_text())
            text.configure(state='disabled')
            status.set('showing %s | out: %s' % (target, self.out_dir))
            root.after(300, tick)

        def on_close():
            root.destroy()
            if rospy.core.is_initialized():
                rospy.signal_shutdown('result viewer closed')

        root.protocol('WM_DELETE_WINDOW', on_close)
        root.after(300, tick)
        try:
            root.mainloop()
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("result viewer: GUI stopped (%s)", err)

    def summary_text(self):
        with self.lock:
            people = self.result.get('people', {})
            car = self.result.get('car', {})
            light = self.result.get('traffic_light', {})

        lines = ['== people ==']
        if people:
            lines.append('total    : %s' % people.get('total', 0))
            lines.append('community: %s' % people.get('community', 0))
            lines.append('intruder : %s' % people.get('intruder', 0))
            lines.append('intruders: %s' % people.get('intruders', []))
        else:
            lines.append('(no data)')

        lines.append('')
        lines.append('== car ==')
        if car:
            lines.append('total : %s' % car.get('total', 0))
            lines.append('plate : %s' % car.get('plate', ''))
            lines.append('conf  : %s' % car.get('plate_conf', 0.0))
        else:
            lines.append('(no data)')

        lines.append('')
        lines.append('== traffic light ==')
        if light:
            lines.append('model: %s' % light.get('model_name', ''))
            lines.append('state: %s' % light.get('state', ''))
        else:
            lines.append('(no data)')

        return '\n'.join(lines)


def _make_bridge():
    try:
        from cv_bridge import CvBridge
        return CvBridge()
    except ImportError:
        return None


def default_results_dir():
    """aic26_perception/results, or <home>/aic26_perception/results."""
    try:
        import rospkg
        return os.path.join(rospkg.RosPack().get_path('aic26_perception'),
                            'results')
    except Exception:                                  # noqa: BLE001
        return os.path.expanduser('~/aic26_perception/results')


def main():
    rospy.init_node("mission_controller")
    controller = MissionController()
    if rospy.get_param('~gui', False):
        ResultViewer(rospy.get_param('~results_dir', default_results_dir()),
                     rospy.get_param('~result_topic', '/perception/result')
                     ).start()
    controller.run()


if __name__ == "__main__":
    main()
