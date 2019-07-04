import logging
import sys
import time
import pointcloud_auto
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger


from PyQt4 import QtGui, QtCore

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


class MainWindow(QtGui.QMainWindow):

    def __init__(self):
        QtGui.QMainWindow.__init__(self)

        self.resize(700, 500)
        self.setWindowTitle('Multi-ranger point cloud')

        # self.setCentralWidget(self.canvas.native)

        # Connect callbacks from the Crazyflie API

        # self.scf = SyncCrazyflie(URI, cf=self.cf)
        # Connect to the Crazyflie
        # self.motion_commander.take_off(0.3, 0.1)
        # self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}

        # self.hoverTimer = QtCore.QTimer()
        # self.hoverTimer.timeout.connect(self.automotion)
        # self.hoverTimer.setInterval(100)
        # self.hoverTimer.start()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)

    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')
    scf = SyncCrazyflie(URI, cf=cf)
    cf.open_link(URI)
    time.sleep(5)
    # cf.connected.add_callback(MainWindow.connected)
    # cf.disconnected.add_callback(MainWindow.disconnected)
    mc = MotionCommander(scf, 0.3)
    mr = Multiranger(scf)
    mr.start()
    keep_flying = True
    mc.take_off(0.2)
    while keep_flying:
        appQt = QtGui.QApplication(sys.argv)
        win = MainWindow()
        win.show()

        print(f"Multiranger.up = {mr.up}")
        print(f"Multiranger.front = {mr.front}")
        mc.start_linear_motion(0.05, 0, 0)
        if is_close(mr.up):
            mc.land(0.05)
            keep_flying = False
            cf.close_link()
        if is_close(mr.front) and mr.right < mr.left:
            mc.turn_left(15, 90)
        if is_close(mr.front) and mr.right > mr.left:
            mc.turn_right(15, 90)
        time.sleep(0.1)
        appQt.exec()
    # with SyncCrazyflie(URI, cf=cf) as scf:
    #     with MotionCommander(scf) as motion_commander:
    #         with Multiranger(scf) as multiranger:
    #             keep_flying = True
    #
    #             while keep_flying:
    #                 motion_commander.start_linear_motion(0.05, 0, 0)
    #                 print(f"Multiranger.up = {multiranger.up}, {type(multiranger.up)}, {is_close(multiranger.up)}")
    #                 print(f"Multiranger.front = {multiranger.front}, {type(multiranger.front)}, {is_close(multiranger.front)}")
    #                 if is_close(multiranger.front) and is_close(multiranger.right) and is_close(multiranger.left):
    #                     motion_commander.turn_right(180, 90)
    #                     motion_commander.start_forward(0.05)
    #
    #                 if is_close(multiranger.front) and multiranger.right < multiranger.left:
    #                     motion_commander.turn_left(90, 90)
    #
    #                 if is_close(multiranger.front) and multiranger.right > multiranger.left:
    #                     motion_commander.turn_right(90, 90)
    #
    #                 if is_close(multiranger.front) and is_close(multiranger.right):
    #                     motion_commander.turn_left(90, 90)
    #                     motion_commander.start_forward(0.05)
    #
    #                 if is_close(multiranger.front) and is_close(multiranger.left):
    #                     motion_commander.turn_right(90, 90)
    #                     motion_commander.start_forward(0.05)
    #
    #                 if is_close(multiranger.back):
    #                     motion_commander.start_forward(0.2)
    #
    #                 if is_close(multiranger.left):
    #                     motion_commander.turn_right(45)
    #                     motion_commander.start_forward(0.05)
    #
    #                 if is_close(multiranger.right):
    #                     motion_commander.turn_left(45)
    #                     motion_commander.start_forward(0.05)
    #
    #                 if is_close(multiranger.up):
    #                     motion_commander.land(0.1)
    #                     keep_flying = False
    #
    #                 time.sleep(0.1)
    #
    # appQt = pointcloud_auto.QtGui.QApplication(sys.argv)
    # win = pointcloud_auto.MainWindow(URI)
    # win.show()
    # appQt.exec_()