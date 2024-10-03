import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os
import csv
import angle_trans

# 暗さを調節する変数
brightness_factor = 0.0  # 調整する場合は適切な値に変更

# カメラの設定
conf = rs.config()
# RGB
conf.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# distance
conf.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# stream start
pipe = rs.pipeline()
profile = pipe.start(conf)

num = 0

# 画像保存用ディレクトリの設定
output_directory = "img"
if not os.path.exists(output_directory):
    os.makedirs(output_directory)


#remove the old data 
for filename in os.listdir(output_directory):
    if filename.endswith(".png") or filename.endswith(".csv"):
        file_path = os.path.join(output_directory, filename)
        try:
            os.remove(file_path)
        except Exception as e:
            print(f"Error deleting {filename}: {e}")

# CSVファイルをオープンしてヘッダーを書き込む
csv_filename = os.path.join(output_directory, "data.csv")
with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Frame", "X", "Y", "Angle (degrees)"])

while True:
    # フレームの取得
    num += 1
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    
    # convert to image
    color_image = np.asanyarray(color_frame.get_data())

    ############ Define the region of interest (ROI) ############
    # (x, y) coordinates of the top-left and bottom-right corners of the ROI
    roi_top_left = (500, 0)
    roi_bottom_right = (1000, 500)

    # Draw a green rectangle to indicate the ROI
    cv2.rectangle(color_image, roi_top_left, roi_bottom_right, (0, 255, 0), 2)

    # Get the ROI from the color frame
    roi_color = color_image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
    
    ############ Looking for sky colour in HSV ############ 
    ##### convert BGR to HSV
    hsv_image = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
    
    # parameter setting for finding "sky colour" in HSV
    lower_sky_hsv = np.array([75, 70, 50])  # Hue, Saturation, Value lower bound for sky color
    upper_sky_hsv = np.array([95, 255, 255])  # Hue, Saturation, Value upper bound for sky color

    # mask
    sky_mask = cv2.inRange(hsv_image, lower_sky_hsv, upper_sky_hsv)

    ##### find rectangle
    contours, hierarchy = cv2.findContours(
        sky_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    largest_area = 0

    for i, contour in enumerate(contours):
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.intp(box)

        # record the largest rectangle
        if largest_area < ((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[2][1]) ** 2):
            largest_area = ((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[2][1]) ** 2)
            largest_box = box

    ############ Calculate the target box coordination and angle ############ 
    if largest_area != 0:
        # Calculate the center of the target in the ROI
        target_box_center_1 = (largest_box[0][0] + largest_box[2][0]) // 2
        target_box_center_2 = (largest_box[0][1] + largest_box[2][1]) // 2

        # Calculate the angle of the rotated rectangle

        # point 0 1 のx yの差を計算
        delta_x1 = largest_box[0][0] - largest_box[1][0]
        delta_y1 = largest_box[0][1] - largest_box[1][1]

        # point 1 2 のx yの差を計算
        delta_x2 = largest_box[1][0] - largest_box[2][0]
        delta_y2 = largest_box[1][1] - largest_box[2][1]

        if delta_x1**2+delta_y1**2 > delta_x2**2+delta_y2**2:
            delta_x = delta_x1
            delta_y = delta_y1
        else:
            delta_x = delta_x2
            delta_y = delta_y2

        # アークタンジェント関数を使用して角度を計算
        angle_rad = math.atan2(delta_y, delta_x)
        # 弧度法から度数法に変換
        angle = math.degrees(angle_rad)

        width = max(item[0] for item in largest_box)-min(item[0] for item in largest_box)
        height = max(item[1] for item in largest_box)-min(item[1] for item in largest_box)
        if width < height:
            print(f"\rangle:{angle} tate", end="")
        else:
            print(f"\rangle:{angle} yoko", end="")
            
        #angle_translation
        angle = angle_trans.angle_translation(angle)

        # 青空の領域を一時的に保存
        sky_region = roi_color.copy()
        sky_region[sky_mask == 0] = 0

        # 青空以外の領域を暗くする
        roi_color[sky_mask == 0] = (roi_color[sky_mask == 0] * brightness_factor).astype(np.uint8)

        # 青空の領域を元の画像に戻す
        roi_color[sky_mask != 0] = sky_region[sky_mask != 0]

        # Draw the center point on the ROI
        cv2.circle(roi_color, (target_box_center_1, target_box_center_2), 2, (0, 255, 0), 5)

        # Display the pixel coordinates and angle next to the point
        text = "Pixel X: {} Y: {} Angle: {:.2f} degrees".format(target_box_center_1, target_box_center_2, angle)
        cv2.putText(roi_color, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Draw the detected box on the ROI
        cv2.drawContours(roi_color, [largest_box], 0, (0, 0, 255), 2)

        # ファイル名の生成
        img_filename = os.path.join(output_directory, f"frame_{num:04d}.png")
        
        # 画像を保存
        cv2.imwrite(img_filename, roi_color)

        # CSVファイルにデータを追加
        with open(csv_filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([num, target_box_center_1, target_box_center_2, angle])
        
    # Display the image
    cv2.imshow("Image", roi_color)
    cv2.waitKey(200)