import pybullet as p
import pybullet_data
import xacro
import os
import time
import math
import random
import numpy as np
import cv2

# --- 1. 경로 설정 및 Xacro 변환 ---
current_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(current_dir, "leo_common-master", "leo_description")
xacro_file = os.path.join(base_path, "urdf", "leo.urdf.xacro")

try:
    with open(xacro_file, 'r') as f:
        content = f.read()
    content = content.replace("$(find leo_description)", base_path)
    doc = xacro.parse(content)
    xacro_process = xacro.process_doc(doc)
    robot_description = doc.toxml()
    robot_description = robot_description.replace("package://leo_description", base_path)
    
    urdf_file_path = os.path.join(current_dir, "temp_leo.urdf")
    with open(urdf_file_path, "w") as f:
        f.write(robot_description)
except Exception as e:
    print(f"Xacro 변환 오류: {e}")
    exit()

# --- 2. 시뮬레이션 환경 설정 ---
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# --- 3. 미로 및 Goal 생성 ---
def create_maze(width, height):
    maze = [[1] * width for _ in range(height)]
    def walk(x, y):
        maze[y][x] = 0
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx*2, y + dy*2
            if 0 <= nx < width and 0 <= ny < height and maze[ny][nx] == 1:
                maze[y + dy][x + dx] = 0
                walk(nx, ny)
    walk(1, 1)
    
    dead_ends = []
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if maze[y][x] == 0:
                walls = sum(1 for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)] if maze[y+dy][x+dx] == 1)
                if walls == 3 and not (x == 1 and y == 1):
                    dead_ends.append((x, y))
    goal_pos = random.choice(dead_ends) if dead_ends else (width-2, height-2)
    maze[goal_pos[1]][goal_pos[0]] = 2
    return maze, goal_pos

my_maze, goal_coords = create_maze(15, 15)

def build_maze_in_pybullet(maze):
    wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25], rgbaColor=[0.7, 0.7, 0.7, 1])
    goal_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.4, length=0.01, rgbaColor=[1, 0, 0, 1])
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 1: p.createMultiBody(0, wall_shape, wall_visual, [x, y, 0.25])
            elif cell == 2: p.createMultiBody(0, -1, goal_visual, [x, y, 0.01])

build_maze_in_pybullet(my_maze)

# --- 4. 로봇 로드 및 물리 설정 ---
leo_id = p.loadURDF(urdf_file_path, basePosition=[1, 1, 0.5])
left_wheels, right_wheels = [2, 3], [5, 6]
for j in (left_wheels + right_wheels):
    p.changeDynamics(leo_id, j, lateralFriction=1.2)

# 색상 입히기 (선택 사항)
# 색상 정의 (R, G, B, Alpha)
ORANGE = [1.0, 0.4, 0.0, 1.0]
DARK_GRAY = [0.2, 0.2, 0.2, 1.0]
BLACK = [0.1, 0.1, 0.1, 1.0]
YELLOW = [1.0, 1.0, 0.0, 1.0]
GRAY = [0.5, 0.5, 0.5, 1.0]
BLUE = [0.0, 0.0, 0.8, 1.0]

# 1. 몸체 색상 변경 (base_link는 index -1)
p.changeVisualShape(leo_id, -1, rgbaColor=GRAY)

# 2. 링크별 이름 기반 색상 할당
num_joints = p.getNumJoints(leo_id)
for i in range(num_joints):
    joint_info = p.getJointInfo(leo_id, i)
    link_name = joint_info[12].decode('utf-8').lower()
    
    if "wheel" in link_name:
        p.changeVisualShape(leo_id, i, rgbaColor=BLACK)
    elif "rocker" in link_name or "bogie" in link_name:
        p.changeVisualShape(leo_id, i, rgbaColor=DARK_GRAY)
    elif "camera" in link_name or "lidar" in link_name:
        p.changeVisualShape(leo_id, i, rgbaColor=YELLOW)
    else:
        p.changeVisualShape(leo_id, i, rgbaColor=BLUE)

# --- 5. 장애물 회피 포함 비전 자율주행 ---
p.setRealTimeSimulation(1)
start_time = time.time()

while True:
    # A. 카메라 이미지 획득
    cam_state = p.getLinkState(leo_id, 8) 
    cam_pos, cam_ori = cam_state[0], cam_state[1]
    rot_mat = p.getMatrixFromQuaternion(cam_ori)
    forward = [rot_mat[0], rot_mat[3], rot_mat[6]]
    up = [rot_mat[2], rot_mat[5], rot_mat[8]]
    target = [cam_pos[0] + forward[0], cam_pos[1] + forward[1], cam_pos[2] + forward[2]]
    
    view_mat = p.computeViewMatrix(cam_pos, target, up)
    proj_mat = p.computeProjectionMatrixFOV(65, 1.0, 0.1, 10.0)
    
    img_w, img_h = 80, 60
    _, _, rgb, _, _ = p.getCameraImage(img_w, img_h, view_mat, proj_mat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    
    frame = np.reshape(rgb, (img_h, img_w, 4))[:, :, :3].astype(np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # B. 장애물 감지 (이미지 하단부 분석)
    bottom_part = gray[int(img_h*0.7):, :]
    # 벽(회색)은 보통 중간 밝기값을 가짐
    wall_pixel_count = np.sum((bottom_part > 100) & (bottom_part < 200))
    is_obstacle_close = wall_pixel_count > (bottom_part.size * 0.6) # 하단 60% 이상이 벽이면 위험

    # C. 목표(Red) 감지
    lower_red = np.array([0, 150, 50])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    M = cv2.moments(mask)
    
    linear_vel = 4.0
    angular_vel = 0.0

    # D. 행동 결정 트리
    if is_obstacle_close:
        # 1순위: 장애물 회피 (강제 회전)
        linear_vel = -1.0 # 약간 후진하며
        angular_vel = 4.0  # 탈출할 때까지 회전
    elif M["m00"] > 5:
        # 2순위: 목표 추적
        cx = int(M["m10"] / M["m00"])
        err = cx - (img_w / 2)
        angular_vel = -err * 0.15
        if M["m00"] > (img_w * img_h * 0.2): 
            print(f"Goal 도달! 소요 시간: {time.time() - start_time:.2f}초")
            break
    else:
        # 3순위: 탐색 (제자리 회전)
        angular_vel = 2.0 

    # E. 모터 제어
    l_speed = linear_vel - angular_vel
    r_speed = linear_vel + angular_vel
    for j in left_wheels: p.setJointMotorControl2(leo_id, j, p.VELOCITY_CONTROL, targetVelocity=l_speed, force=30)
    for j in right_wheels: p.setJointMotorControl2(leo_id, j, p.VELOCITY_CONTROL, targetVelocity=r_speed, force=30)

    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=cam_pos)
    p.stepSimulation()
    time.sleep(1./60.)