import numpy as np
import math
def forward_kinematics(start, length, angles):
    
    angle_x, angle_y, angle_z = angles
    
    # 度をラジアンに変換
    # angle_x = np.radians(angle_x)
    # angle_y = np.radians(angle_y)
    # angle_z = np.radians(angle_z)
    
    # 各軸に対する方向ベクトルを計算
    dx = length * np.cos(angle_x)
    dy = length * np.cos(angle_y)
    dz = length * np.cos(angle_z)
    
    # 終点の座標を計算
    end_x = start[0] + dx
    end_y = start[1] + dy
    end_z = start[2] + dz
    
    return (end_x, end_y, end_z)
    # angles_ = np.radians(angles)
    # roll, pitch, yaw = angles_
    # rotation_matrix = np.array([
    #     [np.cos(yaw) * np.cos(pitch), 
    #      np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), 
    #      np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
         
    #     [np.sin(yaw) * np.cos(pitch), 
    #      np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll), 
    #      np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
         
    #     [-np.sin(pitch), 
    #      np.cos(pitch) * np.sin(roll), 
    #      np.cos(pitch) * np.cos(roll)]
    # ])
    
    # # 終点の座標の計算
    # endpoint = start + length * rotation_matrix[:, 0]
    
    # return endpoint

if __name__ == '__main__':
    print(forward_kinematics([0.0,0.0,0.0],math.sqrt(2),[math.pi/2,math.pi/4,math.pi/4]))