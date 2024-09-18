import numpy as np

def calculate_angles(start, end):
    # ベクトルを計算
    vector = np.array(end) - np.array(start)
    
    # ベクトルの長さを計算
    length = np.linalg.norm(vector)
    
    # 各軸に対する角度を計算
    angle_x = np.arccos(vector[0] / length)
    angle_y = np.arccos(vector[1] / length)
    angle_z = np.arccos(vector[2] / length)
    
    # ラジアンを度に変換
    angle_x_deg = np.degrees(angle_x)
    angle_y_deg = np.degrees(angle_y)
    angle_z_deg = np.degrees(angle_z)
    
    return angle_x_deg, angle_y_deg, angle_z_deg, length

if __name__ == '__main__':
    # 使用例
    start_point = [0, 0, 0]
    end_point = [0, 1, 1]

    angles = calculate_angles(start_point, end_point)
    print(f"各軸に対する角度 (度): X軸={angles[0]:.2f}, Y軸={angles[1]:.2f}, Z軸={angles[2]:.2f}, 長さ={angles[3]:.2f}")