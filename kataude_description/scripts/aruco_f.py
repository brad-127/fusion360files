import os
from tool.dym_setup import Control
from utils.save_data import RecordData
from extra.utils.Opencv.aruco_follow import ArUcoFollow

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

DymControl = Control()
samples = 256
cycles = 5
trails = 5
recordData = "norecord"  # RecordData(samples*cycles, 8, DymControl)#
seq_num = 2

aruco_follow = ArUcoFollow(marker_id=1)

try:
    while True:
        input_command = input(
            """Input Desired State of control
                i : initialize
                r : record
                LF: Leader follower Control
                e : END                   
                a : automate
                u : 6DOF LF
                ga : aruco marker code
               rs : Real sense

            Input : """
        )

        if input_command == "g":
            aruco_follow.move_servo_to_center(DymControl)
        elif input_command == "i":
            DymControl.initialize()
        elif input_command == "r":
            recordData.start_recording()
        elif input_command == "LF":
            DymControl.leader_follower_control()
        elif input_command == "e":
            break
        elif input_command == "a":
            DymControl.automate()
        elif input_command == "u":
            DymControl.six_dof_leader_follower()
        elif input_command == "ga":
            aruco_follow.show_real_time_detection(DymControl)
        elif input_command == "rs":
            aruco_follow.get_frames()

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    if os.name != "nt":
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)