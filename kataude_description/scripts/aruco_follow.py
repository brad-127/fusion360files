import cv2,os
import numpy as np
import pyrealsense2 as rs
from tool.dym_setup import Control
if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

DymControl = Control()
goal = DymControl.goal
d = 2
fs = 20
Wn = 2.0
import pyrealsense2 as rs

DymControl.LiveLPF_initialize(d=d, fs=fs, Wn=Wn)
class ArUcoFollow:
    def __init__(self, marker_id, servo_min=450, servo_max=3500, move_step=50,cam = "D435"):
        self.marker_id = marker_id
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.move_step = move_step
        self.device_info_list=[]
        self.get_realsense_device_info()
        print(self.device_info_list)

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        if len(self.device_info_list) > 0:
            for device_info in self.device_info_list:
                print(device_info)
                if cam in device_info['name']:
                    config.enable_device(device_info['serial_number'])
            # config.enable_device(self.device_info_list[0]['serial_number'])
        # config.enable_device('141322250166') 
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ArUco marker dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
    def get_realsense_device_info(self):
        """
        Retrieves information about connected RealSense devices.
        
        Returns:
            list: A list of dictionaries, each containing the information about a connected device.
        """
        # Create a context object. This object owns the handles to all connected realsense devices
        context = rs.context()

        # Get a list of all connected devices
        connected_devices = context.query_devices()

        if len(connected_devices) == 0:
            print("No RealSense devices connected.")
            return []

        device_info_list = []
        for device in connected_devices:
            device_info = {
                'name': device.get_info(rs.camera_info.name),
                'serial_number': device.get_info(rs.camera_info.serial_number),
                'firmware_version': device.get_info(rs.camera_info.firmware_version),
                'usb_type': device.get_info(rs.camera_info.usb_type_descriptor)
            }
            self.device_info_list.append(device_info)
        
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image

    def detect_marker(self):
        color_image, depth_image = self.get_frames()
        if color_image is None or depth_image is None:
            return None, None, None

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None and self.marker_id is not None and self.marker_id in ids:
            index = np.where(ids == self.marker_id)[0][0]
            marker_corners = corners[index][0]
            marker_center = np.mean(marker_corners, axis=0).astype(int)
            frame_center = np.array([color_image.shape[1] // 2, color_image.shape[0] // 2])
            offset = marker_center - frame_center

            # Ensure marker_center is within image bounds
            if 0 <= marker_center[1] < depth_image.shape[0] and 0 <= marker_center[0] < depth_image.shape[1]:
                depth = depth_image[marker_center[1], marker_center[0]]
            else:
                print("Marker center out of depth image bounds")
                return None, None, color_image
            # Draw marker and center line on the color image
            cv2.aruco.drawDetectedMarkers(color_image, corners)
            cv2.circle(color_image, tuple(marker_center), 5, (0, 255, 0), -1)
            cv2.line(color_image, (frame_center[0], 0), (frame_center[0], color_image.shape[0]), (0, 0, 255), 1)
            cv2.line(color_image, (0, frame_center[1]), (color_image.shape[1], frame_center[1]), (0, 0, 255), 1)

            return offset, depth, color_image
        return None, None, color_image

    def move_servo_to_center(self, DymControl):
        offset, _, color_image = self.detect_marker()
        
        if offset is not None:
            # for servo in DymControl.servo_com["F"]:
            servo = "xm430F"
            val = DymControl.getj(servo)
            angle = np.arctan2(offset[1], offset[0]) * 180 / np.pi
            print("offset",offset,f"Calculated angle for top/bottom movement: {angle:.2f} degrees", "value_read",val)
            self.move_step = round(min(max(abs(offset[1]) * 0.5, 1), 100))
            print(f"Dynamic move_step: {self.move_step}")
            if offset[1] < -2:
                # Comment out the actual move for now
                goal[servo] = DymControl.Filter([val[0]-self.move_step], servo)
                # print("goal",goal[servo],f"Moving up by {self.move_step} steps")

                DymControl.movej(goal, servo)  # Move up
                
            elif offset[1] > 2:
                # goal[servo] = [val[0]+self.move_step]
                goal[servo] = DymControl.Filter([val[0]+self.move_step], servo)
                # print("goal",goal[servo],f"Moving down by {self.move_step} steps")
                # Comment out the actual move for now
                DymControl.movej(goal, servo)  # Move down
                
        else:
            print("Marker not detected.")
        return color_image

    def show_real_time_detection(self, DymControl):
        while True:
            color_image = self.move_servo_to_center(DymControl)
            if color_image is not None:
                cv2.imshow('ArUco Marker Detection', color_image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.pipeline.stop()
        cv2.destroyAllWindows()

aruco_follow = ArUcoFollow(marker_id=1)
DymControl.goal["xm430F"] = DymControl.getj("xm430F")
DymControl.goal["xm430F"] = DymControl.Filter(DymControl.goal["xm430F"], "xm430F")
for servo in DymControl.servos:
    print("enable")
    DymControl.Enable(servo)
    print("enabled")
print(DymControl.goal)
# DymControl.movej(DymControl.goal,"xm430F")

print("2048")
if getch() == chr(0x1b):
    # for servo in DymControl.servos:
    #     DymControl.Disable(servo)
    quit()
aruco_follow.show_real_time_detection(DymControl)