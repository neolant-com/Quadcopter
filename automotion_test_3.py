import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/65/1M'

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.3  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True

                while keep_flying:
                    motion_commander.start_linear_motion(0.2, 0, 0)

                    if is_close(multiranger.front) and is_close(multiranger.right) and is_close(multiranger.left):
                        motion_commander.turn_right(180, 90)
                        motion_commander.start_forward(0.1)

                    if is_close(multiranger.front) and multiranger.right < multiranger.left:
                        motion_commander.turn_left(90, 90)

                    if is_close(multiranger.front) and multiranger.right > multiranger.left:
                        motion_commander.turn_right(90, 90)

                    if is_close(multiranger.front) and is_close(multiranger.right):
                        motion_commander.turn_left(90, 90)
                        motion_commander.start_forward(0.1)

                    if is_close(multiranger.front) and is_close(multiranger.left):
                        motion_commander.turn_right(90, 90)
                        motion_commander.start_forward(0.1)

                    if is_close(multiranger.back):
                        motion_commander.start_forward(0.2)

                    if is_close(multiranger.left):
                        motion_commander.turn_right(45)
                        motion_commander.start_forward(0.1)

                    if is_close(multiranger.right):
                        motion_commander.turn_left(45)
                        motion_commander.start_forward(0.1)

                    if is_close(multiranger.up):
                        motion_commander.land(0.1)
                        keep_flying = False

                    time.sleep(0.1)
