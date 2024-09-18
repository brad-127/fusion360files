#!/usr/bin/env python

import numpy as np
import xml.etree.ElementTree as ET
from scipy.optimize import minimize
import rospkg
import set45deg
#endeffecter buffer distance

end_effecter_distance = 0.01

# Xacroファイルのパスを取得
rospack = rospkg.RosPack()
package_path = rospack.get_path('kataude_description')
xacro_file = f"{package_path}/urdf/kataude.xacro"

# Xacroファイルの読み込み
tree = ET.parse(xacro_file)
root = tree.getroot()

# リンクの長さを取得するための辞書
link_lengths = {}
initial_joint_angles = {}
joint_names = {}
joint_origins = {}
right_arm = np.array(['ShoulderJoint1', 'ShoulderJoint2', 'ElbowJoint', 'xm540_1', 'xm540_2', 'xm540_3', 'EndEffecter'])
# Xacroからリンクと関節の情報を取得
for joint in root.findall('joint'):
    joint_name = joint.get('name')
    joint_names[joint_name] = joint

    parent_link = joint.find('parent').get('link')
    child_link = joint.find('child').get('link')

    # jointのoriginを取得
    origin = joint.find('origin')
    if origin is not None:
        joint_origins[joint_name] = list(map(float, origin.get('xyz').split()))

# 初期関節角度
initial_angles = np.zeros(len(joint_names))

# 目標位置
target_position = np.array([0.5, 0.5, 0.5])

# 関節名からインデックスへのマッピング
joint_index = {name: idx for idx, name in enumerate(joint_names)}

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

def forward_kinematics(start, length, angles):
    
    angle_x, angle_y, angle_z = angles
    
    # 度をラジアンに変換
    angle_x = np.radians(angle_x)
    angle_y = np.radians(angle_y)
    angle_z = np.radians(angle_z)

    # 各軸に対する方向ベクトルを計算
    dx = length * np.cos(angle_x)
    dy = length * np.cos(angle_y)
    dz = length * np.cos(angle_z)
    
    # 終点の座標を計算
    end_x = start[0] + dx
    end_y = start[1] + dy
    end_z = start[2] + dz
    
    
    return (end_x, end_y, end_z)

import math
import numpy as np

#各アームの長さLとジョイントの角度θを入力に手先座標[x, y, z]を出力する関数
def forwardKinematics3D(L, js):
    Ts = [] #各回転行列Tを収めるリスト

    #各ジョイントのsin, cosを計算
    c0 = math.cos(js[0])
    s0 = math.sin(js[0])
    c1 = math.cos(js[1])
    s1 = math.sin(js[1])
    c2 = math.cos(js[2])
    s2 = math.sin(js[2])
    c3 = math.cos(js[3])
    s3 = math.sin(js[3])
    c4 = math.cos(js[4])
    s4 = math.sin(js[4])
    c5 = math.cos(js[5])
    s5 = math.sin(js[5])

    #各回転行列Tを計算
    Ts.append(np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
    #スタンド先端までの回転行列T0
    Ts.append(np.matrix([[0, 0, 0, L[0][0]], [0, c0, -s0, L[0][1]], [0, s0, c0, L[0][2]], [0, 0, 0, 1]]))
    #アーム1先端までの回転行列T1
    Ts.append(np.matrix([[c1, 0, s1, L[1][0]], [0, 1, 0, L[1][1]], [-s1, 0, c1, L[1][2]], [0, 0, 0, 1]]))
    #アーム2先端までの回転行列T2
    Ts.append(np.matrix([[c2, 0, s2, L[2][0]], [0, 1, 0, L[2][1]], [-s2, 0, c2, L[2][2]], [0, 0, 0, 1]]))
    #アーム3先端までの回転行列T3
    Ts.append(np.matrix([[c3, -s3, 0, L[3][0]], [s3, c3, 0, L[3][1]], [0, 0, 1, L[3][2]], [0, 0, 0, 1]]))
    #アーム4先端までの回転行列T4
    Ts.append(np.matrix([[1, 0, 0, L[4][0]], [0, c4, -s4, L[4][1]], [0, s4, c4, L[4][2]], [0, 0, 0, 1]]))
    #アーム5先端までの回転行列T5
    Ts.append(np.matrix([[c5, 0, s5, L[5][0]], [0, 1, 0, L[5][1]], [-s5, 0, c5, L[5][2]], [0, 0, 0, 1]]))
    #アーム6先端までの回転行列T6
    # Ts.append(np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    #各回転行列をかけ合わせることでアーム6の手先座標を求める順運動学を解く
    TTs = Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]*Ts[5]*Ts[6]

    #手先座標[x, y, z]を出力
    return [TTs[0, 3], TTs[1, 3], TTs[2, 3]]


initial_joint_positions = {}
prev_position = np.array([0, 0, 0])
link_lengths_xyz = []

for idx, joint in enumerate(joint_names):
    if idx >= 7:  # リンクの長さをエンドエフェクタの手前まで算出
        break
    
    if joint in joint_origins:  # ジョイントがoriginに存在する場合
        origin = joint_origins[joint]
        current_position = np.array(origin)

        # リンクの長さを算出
        if initial_joint_positions:
            link_lengths[joint] = calculate_angles(prev_position,current_position)[3]
            initial_joint_angles[joint] = calculate_angles(prev_position,current_position)[:3]
            # print(f"リンク名: {idx}, 長さ: {link_length:.10f}")
            link_lengths_xyz.append(np.array(current_position) - np.array(prev_position))

        initial_joint_positions[joint] = current_position
        prev_position = current_position



# end effecter部分のリンクの計算
origin = joint_origins['EndEffecter1']
end_effector1_position = np.array(origin)
origin = joint_origins['Endeffecter2']
end_effector2_position = np.array(origin)
mid_point = (end_effector1_position + end_effector2_position) / 2

last_joint_position = prev_position
last_link_length = np.linalg.norm(mid_point - last_joint_position)
link_lengths['EndEffecter'] = calculate_angles(prev_position, mid_point)[3] + end_effecter_distance
initial_joint_angles['EndEffecter'] = calculate_angles(prev_position, mid_point)[:3]
initial_joint_positions['EndEffecter'] = mid_point

# print("AAA",initial_joint_angles[right_arm[1]])
# print("ABA",initial_joint_positions[right_arm[0]])
current_joint_angles = initial_joint_angles
current_joint_positions = initial_joint_positions
print("\n各リンクの長さ:")
print(link_lengths)
print("\n角度:")
print(current_joint_angles)
print("\n位置:")
print(current_joint_positions)

for i in range(len(right_arm)-1):
    # print(right_arm[i])
    if i == 0:
        pre_pos = current_joint_positions[right_arm[i]]
        print(right_arm[i],":",current_joint_positions[right_arm[i]])
    current_joint_positions[right_arm[i+1]] = forward_kinematics(pre_pos,link_lengths[right_arm[i+1]],current_joint_angles[right_arm[i+1]])
    pre_pos = current_joint_positions[right_arm[i+1]]
    print(right_arm[i+1],":",current_joint_positions[right_arm[i+1]])
    

Angles=[0,90,0,0,0,0]
    
print("順運動学")
position=forwardKinematics3D(link_lengths_xyz,Angles)
print(position)


def jacobian(L, js, delta=1e-6):
    J = np.zeros((3, len(js)))
    fk_init = forwardKinematics3D(L, js)
    
    for i in range(len(js)):
        js_delta = np.copy(js)
        js_delta[i] += delta
        fk_delta = forwardKinematics3D(L, js_delta)
        J[:, i] = (np.array(fk_delta) - np.array(fk_init)) / delta

    return J

def inverseKinematics3D(target, L, initial_guess, max_iterations=1000, tolerance=1e-6):
    js = np.copy(initial_guess)
    
    for _ in range(max_iterations):
        fk_current = forwardKinematics3D(L, js)
        error = np.array(target) - np.array(fk_current)
        
        if np.linalg.norm(error) < tolerance:
            break
        
        J = jacobian(L, js)
        d_js = np.linalg.pinv(J).dot(error)
        js += d_js

    return js

initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
target_position = [1.5, 0.5, 1.0]

solution_js = inverseKinematics3D(target_position, link_lengths_xyz, initial_guess)
print("解となるジョイント角度:", solution_js)