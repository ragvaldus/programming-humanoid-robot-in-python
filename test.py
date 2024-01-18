# -*- encoding: UTF-8 -*-

import time
import almath
import argparse
from naoqi import ALProxy


def main(robotIP="nao4.local", PORT=9559):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    motionProxy.setStiffnesses("Head", 1.0)

    # Simple command for the HeadYaw joint at 10% max speed
    names      = ["HeadYaw", "HeadPitch"]
    angleLists = [[1.0, -1.0, 1.0, -1.0], [-1.0]]
    times      = [[1.0,  2.0, 3.0,  4.0], [ 5.0]]
    isAbsolute = True
    motionProxy.angleInterpolation(names, angleLists, times, isAbsolute)

    time.sleep(3.0)
    motionProxy.setStiffnesses("Head", 0.0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="nao4.local",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)

