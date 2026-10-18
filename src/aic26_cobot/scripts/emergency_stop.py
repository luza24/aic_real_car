"""Emergency stop for the STM32 chassis.

Sends zero-velocity frames directly to the serial port. This script does not
need a ROS master, so it also works when the chassis node or roslaunch has
already died.
"""

import argparse
import os
import termios
import time
import tty

ZERO_FRAME = b'\xaa\xbb\x0a\x12\x02\x00\x00\x00\x00\x00\x00\x00'

BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: getattr(termios, 'B230400', termios.B115200),
    460800: getattr(termios, 'B460800', termios.B115200),
    921600: getattr(termios, 'B921600', termios.B115200),
}


def send_zero_frames(port, baud, seconds, rate):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    try:
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        speed = BAUD_RATES.get(baud)
        if speed is None:
            raise ValueError('unsupported baud rate %d' % baud)
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        frames = max(1, int(seconds * rate))
        for _ in range(frames):
            os.write(fd, ZERO_FRAME)
            termios.tcdrain(fd)
            time.sleep(1.0 / rate)
        return frames
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/carserial')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--rate', type=float, default=50.0)
    args = parser.parse_args()

    try:
        count = send_zero_frames(args.port, args.baud,
                                 args.seconds, args.rate)
    except Exception as exc:                           # noqa: BLE001
        print('EMERGENCY STOP FAILED on %s: %s' % (args.port, exc))
        print('CUT MOTOR POWER NOW')
        return 1
    print('sent %d zero-velocity frames to %s' % (count, args.port))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
