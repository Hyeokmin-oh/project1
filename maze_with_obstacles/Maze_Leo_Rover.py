import pybullet as p
import pybullet_data
import xacro
import os
import time
import random
import math
import numpy as np
from collections import deque

# --- 1. 경로 설정 및 Xacro 변환 ---
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
# 녹화 시작 부분(startStateLogging) 삭제됨
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# --- 3. 미로 생성 및 3면이 막힌 Goal 탐색 ---
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

def solve_maze(maze, start, goal):
    queue = deque([([start], start)])
    visited = {start}
    while queue:
        path, (x, y) = queue.popleft()
        if (x, y) == goal: return path
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(maze[0]) and 0 <= ny < len(maze) and maze[ny][nx] != 1 and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((path + [(nx, ny)], (nx, ny)))
    return []

my_maze, goal_coords = create_maze(15, 15)
path_to_follow = solve_maze(my_maze, (1, 1), goal_coords)

def build_maze_in_pybullet(maze):
    wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25], rgbaColor=[0.7, 0.7, 0.7, 1])
    goal_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.4, length=0.01, rgbaColor=[1, 0, 0, 0.5])
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 1: p.createMultiBody(0, wall_shape, wall_visual, [x, y, 0.25])
            elif cell == 2: p.createMultiBody(0, -1, goal_visual, [x, y, 0.01])

build_maze_in_pybullet(my_maze)

# --- 4. Leo Rover 로드 및 자율주행 설정 ---
leo_id = p.loadURDF(urdf_file_path, basePosition=[1, 1, 0.5])
left_wheels, right_wheels = [2, 3], [5, 6]
for j in (left_wheels + right_wheels): p.changeDynamics(leo_id, j, lateralFriction=1.0)

# 색상 입히기
GRAY, BLACK, DARK_GRAY, YELLOW, BLUE = [0.5, 0.5, 0.5, 1.0], [0.1, 0.1, 0.1, 1.0], [0.2, 0.2, 0.2, 1.0], [1.0, 1.0, 0.0, 1.0], [0.0, 0.0, 0.8, 1.0]
p.changeVisualShape(leo_id, -1, rgbaColor=GRAY)
num_joints = p.getNumJoints(leo_id)
for i in range(num_joints):
    joint_info = p.getJointInfo(leo_id, i)
    link_name = joint_info[12].decode('utf-8').lower()
    if "wheel" in link_name: p.changeVisualShape(leo_id, i, rgbaColor=BLACK)
    elif "rocker" in link_name or "bogie" in link_name: p.changeVisualShape(leo_id, i, rgbaColor=DARK_GRAY)
    elif "camera" in link_name or "lidar" in link_name: p.changeVisualShape(leo_id, i, rgbaColor=YELLOW)
    else: p.changeVisualShape(leo_id, i, rgbaColor=BLUE)

# --- 5. 자율주행 메인 루프 ---
start_time = time.time()
path_idx = 0
frame_count = 0
p.setRealTimeSimulation(1)

while path_idx < len(path_to_follow):
    pos, orn = p.getBasePositionAndOrientation(leo_id)
    target_node = path_to_follow[path_idx]
    target_pos = [target_node[0], target_node[1]]

    dx, dy = target_pos[0] - pos[0], target_pos[1] - pos[1]
    dist = math.sqrt(dx**2 + dy**2)
    target_angle = math.atan2(dy, dx)
    
    _, _, yaw = p.getEulerFromQuaternion(orn)
    angle_diff = target_angle - yaw
    while angle_diff > math.pi: angle_diff -= 2*math.pi
    while angle_diff < -math.pi: angle_diff += 2*math.pi

    if dist < 0.3:
        path_idx += 1
        continue

    linear_vel, angular_vel = 0, 0
    if abs(angle_diff) > 0.2:
        angular_vel = 5.0 if angle_diff > 0 else -5.0
    else:
        linear_vel = 20.0
        angular_vel = angle_diff * 5.0

    l_speed, r_speed = linear_vel - angular_vel, linear_vel + angular_vel
    for j in left_wheels: p.setJointMotorControl2(leo_id, j, p.VELOCITY_CONTROL, targetVelocity=l_speed, force=30.0)
    for j in right_wheels: p.setJointMotorControl2(leo_id, j, p.VELOCITY_CONTROL, targetVelocity=r_speed, force=30.0)

    # [골인 체크 및 시간 출력 로직]
    dist_to_goal = math.sqrt((pos[0]-goal_coords[0])**2 + (pos[1]-goal_coords[1])**2)
    if not 'goal_reached' in locals():
        goal_reached = False
    
    if dist_to_goal < 0.4 and target_node == goal_coords:
        if not goal_reached:
            goal_reached = True
            end_time = time.time()
            elapsed_time = end_time - start_time
            minutes, seconds = int(elapsed_time // 60), int(elapsed_time % 60)
            time_str = f"Time : {minutes:02d}:{seconds:02d}"
            
            p.addUserDebugText("MISSION COMPLETE!", [pos[0], pos[1], 1.8], [0, 1, 0], 2, lifeTime=0)
            p.addUserDebugText(time_str, [pos[0], pos[1], 1.5], [1, 1, 1], 2, lifeTime=0)
            print("-" * 30 + f"\n미션 완료! 주행 시간: {time_str}\n" + "-" * 30)
            # 녹화 종료(stopStateLogging) 부분 삭제됨

    if goal_reached:
        l_speed, r_speed = 0, 0
        for j in left_wheels + right_wheels:
            p.setJointMotorControl2(leo_id, j, p.VELOCITY_CONTROL, targetVelocity=0, force=30.0)

    # 카메라 추적 및 시야 업데이트
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=yaw*180/math.pi-90, cameraPitch=-50, cameraTargetPosition=pos)
    
    frame_count += 1
    if frame_count % 30 == 0:
        cam_state = p.getLinkState(leo_id, 8)
        c_p, c_o = cam_state[0], cam_state[1]
        r_m = p.getMatrixFromQuaternion(c_o)
        fwd = [r_m[0], r_m[3], r_m[6]]
        v_m = p.computeViewMatrix(c_p, [c_p[0]+fwd[0], c_p[1]+fwd[1], c_p[2]+fwd[2]], [r_m[2], r_m[5], r_m[8]])
        p.getCameraImage(80, 60, viewMatrix=v_m, projectionMatrix=p.computeProjectionMatrixFOV(65, 1.0, 0.1, 100.0))

    time.sleep(1./1000.)