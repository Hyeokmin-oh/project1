import pybullet as p
import pybullet_data
import xacro
import os
import time
import random
import math
import cv2
import numpy as np

# --- 1. 경로 설정 및 Xacro 변환 (Leo Rover 로직) ---
current_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(current_dir, "leo_common-master", "leo_description")
xacro_file = os.path.join(base_path, "urdf", "leo.urdf.xacro")

try:
    with open(xacro_file, 'r') as f:
        content = f.read()
    content = content.replace("$(find leo_description)", base_path)
    doc = xacro.parse(content)
    xacro.process_doc(doc)
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

# --- 3. 미로 생성 함수 ---
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
                wall_count = 0
                for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                    wall_count = 1
                if wall_count == 3:
                    if not (x==1 and y==1):
                        dead_ends.append((x, y))
    if dead_ends:
        goal_x, goal_y = random.choice(dead_ends)
    else :
        goal_x, goal_y = width - 2, height -2
    maze[goal_y][goal_x] = 2
    print(goal_x, goal_y)
    return maze, (goal_x, goal_y)

def build_maze_in_pybullet(maze, wall_height=0.5):
    wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, wall_height/2])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])
    goal_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.4, length=0.01, rgbaColor=[1, 0, 0, 0.5])
    
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 1:
                p.createMultiBody(0, wall_shape, wall_visual, [x, y, wall_height/2])
            elif cell == 2:
                p.createMultiBody(0, -1, goal_visual, [x, y, 0.01])

my_maze, goal_coords = create_maze(15, 15)
build_maze_in_pybullet(my_maze)

# --- 4. Leo Rover 로드 및 설정 ---
# 미로 시작점(1, 1)에 배치
leo_id = p.loadURDF(urdf_file_path, basePosition=[1, 1, 0.2])
wheel_joints = [2, 3, 5, 6] # Leo Rover 바퀴 인덱스
for j in wheel_joints:
    p.changeDynamics(leo_id, j, lateralFriction=0.8)

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
# --- 5. 메인 제어 루프 ---
print("\n조종법: 방향키 (전후진/회전)")
for i in range(p.getNumJoints(leo_id)):
    print(f"Index {i}: {p.getJointInfo(leo_id, i)[1].decode('utf-8')}")
frame_count = 0

while True:
    keys = p.getKeyboardEvents()
    linear_vel = 0
    angular_vel = 0

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        linear_vel = 15
    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        linear_vel = -15
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        angular_vel = 15
    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        angular_vel = -15

    # 차동 주행 계산
    left_speed = linear_vel - angular_vel
    right_speed = linear_vel + angular_vel

    for i, joint_id in enumerate(wheel_joints):
        target_v = left_speed if i < 2 else right_speed
        p.setJointMotorControl2(leo_id, joint_id, p.VELOCITY_CONTROL, targetVelocity=target_v, force=15.0)

    # 카메라 추적 (AMR Robot 로직)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(leo_id)
    yaw_deg = p.getEulerFromQuaternion(cubeOrn)[2] * 180 / math.pi
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=yaw_deg-90, cameraPitch=-50, cameraTargetPosition=cubePos)

    # 골인 체크
    dist_to_goal = math.sqrt((cubePos[0]-goal_coords[0])**2 + (cubePos[1]-goal_coords[1])**2)
    if dist_to_goal < 0.3:
        p.addUserDebugText("GOAL!", [goal_coords[0], goal_coords[1], 1.2], textColorRGB=[1,0,0], textSize=2, lifeTime=0.1)

    # B. opencv 카메라 시야
    frame_count += 1
    if frame_count % 10 == 0:
        cam_state = p.getLinkState(leo_id, 8) # 10번: 카메라 링크 가정
        cam_pos, cam_ori = cam_state[0], cam_state[1]
        
        rot_mat = p.getMatrixFromQuaternion(cam_ori)
        forward = [rot_mat[0], rot_mat[3], rot_mat[6]]
        up = [rot_mat[2], rot_mat[5], rot_mat[8]]
        target = [cam_pos[0] + forward[0], cam_pos[1] + forward[1], cam_pos[2] + forward[2]]
        
        view_mat = p.computeViewMatrix(cam_pos, target, up)
        proj_mat = p.computeProjectionMatrixFOV(fov=65, aspect=1.0, nearVal=0.1, farVal=100.0)
        
        # 해상도를 낮춰 연산 부하 감소
        p.getCameraImage(width=160, height=120, viewMatrix=view_mat, projectionMatrix=proj_mat)

   

    p.stepSimulation()
    time.sleep(1./240.)