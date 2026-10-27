"""Sets the simulated sky box time and the matching light conditions.

The plugin reads /sky_time every frame and republishes it on the Gazebo sky
topic, so running this while the simulation is up moves the sky (sun/moon) and
switches the world sun plus the robot headlights live. /day_night is still
honoured as a day/night shorthand. Launching the world sets the initial value
through the day_night launch argument.

Command line (one shot, then exits):

    rosrun aic26_gazebo day_night.py _time:=22.0
    rosrun aic26_gazebo day_night.py _mode:=night

GUI (drag the slider to move the sky live, sunrise/sunset are editable):

    rosrun aic26_gazebo day_night.py _gui:=true

The panel is sized in pixels, so it keeps its size on every display; scale it
with AIC26_GUI_SCALE / _gui_scale (e.g. 1.5 for a HiDPI X11 client).

The time can also be advanced by other nodes through publish_time().
"""

import os

import rospy

# representative hours used when only a mode is given
MODE_TIME = {'day': 12.0, 'night': 0.0}

SUNRISE_PARAM = '/sky_sunrise'
SUNSET_PARAM = '/sky_sunset'
DEFAULT_SUNRISE = 6.0
DEFAULT_SUNSET = 20.0

# panel sizes in pixels (see gui_scale)
GUI_SCALE_ENV = 'AIC26_GUI_SCALE'
FONT_PX = 12
SLIDER_PX = 360


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


def wrap_hours(hours):
    """Bring a time of day in hours into the range [0, 24)."""
    return float(hours) % 24.0


def publish_time(hours):
    """Write the sky time; the light_switch plugin picks it up from here.

    This is the interface used by other nodes (e.g. screenshot_cycle).
    Returns the normalised time that was set.
    """
    hours = wrap_hours(hours)
    rospy.set_param('/sky_time', hours)
    return hours


def is_day(hours, sunrise=None, sunset=None):
    hours = wrap_hours(hours)
    if sunrise is None:
        sunrise = rospy.get_param(SUNRISE_PARAM, DEFAULT_SUNRISE)
    if sunset is None:
        sunset = rospy.get_param(SUNSET_PARAM, DEFAULT_SUNSET)
    return float(sunrise) <= hours < float(sunset)


def run_gui():
    """Tk control panel. Dragging the slider moves the sky live."""
    import tkinter as tk
    from tkinter import ttk
    import sys

    scale = gui_scale()

    def px(value):
        """Pixel size for this display (see gui_scale)."""
        return int(round(value * scale))

    root = tk.Tk()
    root.title('day / night')
    # no window manager places the window on an X11 forwarded display
    root.geometry('+%d+%d' % (px(40), px(40)))

    hours = tk.DoubleVar(value=rospy.get_param('/sky_time', 10.0))
    sunrise = tk.DoubleVar(
        value=rospy.get_param(SUNRISE_PARAM, DEFAULT_SUNRISE))
    sunset = tk.DoubleVar(
        value=rospy.get_param(SUNSET_PARAM, DEFAULT_SUNSET))
    readout = tk.StringVar()

    def refresh(*_):
        if rospy.is_shutdown():
            rospy.loginfo("day night gui exit")
            sys.exit(0)
        h = publish_time(hours.get())
        rospy.set_param(SUNRISE_PARAM, float(sunrise.get()))
        rospy.set_param(SUNSET_PARAM, float(sunset.get()))
        state = 'day' if is_day(h, sunrise.get(), sunset.get()) else 'night'
        readout.set('%02d:%02d  (%s)' % (int(h), int((h % 1) * 60), state))

    slider = ttk.Scale(root, from_=0.0, to=24.0, variable=hours,
                       orient='horizontal', command=refresh,
                       length=px(SLIDER_PX))
    slider.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

    # negative font size = pixels in Tk, not points
    big = ('TkDefaultFont', -px(FONT_PX))
    readout_label = ttk.Label(root, textvariable=readout)
    readout_label.configure(font=big)
    readout_label.grid(row=1, column=0, columnspan=2, pady=(0, 8))

    ttk.Label(root, text='sunrise', font=big).grid(
        row=2, column=0, sticky='e', padx=6)
    sun_box = ttk.Spinbox(root, from_=0.0, to=24.0, increment=0.5,
                          textvariable=sunrise, width=8, command=refresh)
    sun_box.configure(font=big)
    sun_box.grid(row=2, column=1, sticky='w', pady=2)

    ttk.Label(root, text='sunset', font=big).grid(
        row=3, column=0, sticky='e', padx=6)
    set_box = ttk.Spinbox(root, from_=0.0, to=24.0, increment=0.5,
                          textvariable=sunset, width=8, command=refresh)
    set_box.configure(font=big)
    set_box.grid(row=3, column=1, sticky='w', pady=2)

    def tick():
        # keep in sync if another node changes the time, then re-arm
        hours.set(rospy.get_param('/sky_time', hours.get()))
        refresh()
        root.after(500, tick)

    refresh()
    root.after(500, tick)
    root.mainloop()


def run_once():
    time = rospy.get_param('~time', None)
    if time is None:
        # The launch file writes /sky_time when a specific time of day was
        # requested; the GUI honours it too, so the one-shot path must not
        # overwrite it with the day/night shorthand.
        time = rospy.get_param('/sky_time', None)
    if time is None:
        mode = rospy.get_param('~mode', '')
        if mode not in MODE_TIME:
            rospy.logerr(
                "_mode must be 'day' or 'night' (or pass _time), got '%s'",
                mode)
            return
        time = MODE_TIME[mode]

    time = publish_time(time)
    rospy.loginfo("/sky_time = %.2f", time)


def main():
    rospy.init_node('day_night')

    if rospy.get_param('~gui', False):
        try:
            run_gui()
        except Exception as err:
            rospy.logerr('GUI unavailable (%s), falling back to one shot', err)
            run_once()
        return

    run_once()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

