import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import xacro
import os
import time
import random
import math
import numpy as np
from collections import deque

class MazeLeoRover(Node):
    def __init__(self):
        super().__init__('maze_leo_rover')
        
        # 1. 시뮬레이션 환경 초기화
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        # 2. Xacro 변환 및 로봇 로드
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.load_leo_rover()

        # 3. 미로 생성 및 경로 탐색
        self.width, self.height = 15, 15
        self.my_maze, self.goal_coords = self.create_maze(self.width, self.height)
        self.path_to_follow = self.solve_maze(self.my_maze, (1, 1), self.goal_coords)
        self.build_maze_in_pybullet(self.my_maze)

        # 4. 주행 상태 변수 설정
        self.path_idx = 0
        self.goal_reached = False
        self.start_time = time.time()
        self.left_wheels, self.right_wheels = [2, 3], [5, 6]

        # 5. 제어 타이머 설정 (30Hz 주기로 control_loop 실행)
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        self.get_logger().info("ROS 2 Leo Rover 미로 찾기 노드가 시작되었습니다.")

    def load_leo_rover(self):
        base_path = os.path.join(self.current_dir, "leo_common-master", "leo_description")
        xacro_file = os.path.join(base_path, "urdf", "leo.urdf.xacro")
        
        try:
            with open(xacro_file, 'r') as f:
                content = f.read()
            content = content.replace("$(find leo_description)", base_path)
            doc = xacro.parse(content)
            xacro.process_doc(doc)
            robot_description = doc.toxml().replace("package://leo_description", base_path)
            
            urdf_file_path = os.path.join(self.current_dir, "temp_leo.urdf")
            with open(urdf_file_path, "w") as f:
                f.write(robot_description)
            
            self.leo_id = p.loadURDF(urdf_file_path, basePosition=[1, 1, 0.5])
            for j in (self.left_wheels_all := [2, 3, 5, 6]):
                p.changeDynamics(self.leo_id, j, lateralFriction=1.0)
            self.apply_visual_colors()
        except Exception as e:
            self.get_logger().error(f"로봇 로드 중 오류 발생: {e}")

    def apply_visual_colors(self):
        GRAY, BLACK, DARK_GRAY, YELLOW, BLUE = [0.5, 0.5, 0.5, 1.0], [0.1, 0.1, 0.1, 1.0], [0.2, 0.2, 0.2, 1.0], [1.0, 1.0, 0.0, 1.0], [0.0, 0.0, 0.8, 1.0]
        p.changeVisualShape(self.leo_id, -1, rgbaColor=GRAY)
        for i in range(p.getNumJoints(self.leo_id)):
            link_name = p.getJointInfo(self.leo_id, i)[12].decode('utf-8').lower()
            if "wheel" in link_name: p.changeVisualShape(self.leo_id, i, rgbaColor=BLACK)
            elif "rocker" in link_name or "bogie" in link_name: p.changeVisualShape(self.leo_id, i, rgbaColor=DARK_GRAY)
            elif "camera" in link_name or "lidar" in link_name: p.changeVisualShape(self.leo_id, i, rgbaColor=YELLOW)
            else: p.changeVisualShape(self.leo_id, i, rgbaColor=BLUE)

    def create_maze(self, width, height):
        maze = [[1] * width for _ in range(height)]
        def walk(x, y):
            maze[y][x] = 0
            dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            random.shuffle(dirs)
            for dx, dy in dirs:
                nx, ny = x + dx*2, y + dy*2
                if 0 <= nx < width and 0 <= ny < height and maze[ny][nx] == 1:
                    maze[y + dy][x + dx] = 0
                    walk(nx, ny)
        walk(1, 1)
        dead_ends = [(x, y) for y in range(1, height-1) for x in range(1, width-1) 
                     if maze[y][x] == 0 and sum(1 for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)] if maze[y+dy][x+dx] == 1) == 3 
                     and not (x==1 and y==1)]
        goal_pos = random.choice(dead_ends) if dead_ends else (width-2, height-2)
        maze[goal_pos[1]][goal_pos[0]] = 2
        return maze, goal_pos

    def solve_maze(self, maze, start, goal):
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

    def build_maze_in_pybullet(self, maze):
        wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25])
        wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25], rgbaColor=[0.7, 0.7, 0.7, 1])
        goal_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.4, length=0.01, rgbaColor=[1, 0, 0, 0.5])
        for y, row in enumerate(maze):
            for x, cell in enumerate(row):
                if cell == 1: p.createMultiBody(0, wall_shape, wall_visual, [x, y, 0.25])
                elif cell == 2: p.createMultiBody(0, -1, goal_visual, [x, y, 0.01])

    def control_loop(self):
        if self.goal_reached:
            for j in self.left_wheels + self.right_wheels:
                p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=0, force=30.0)
            return

        p.stepSimulation()
        pos, orn = p.getBasePositionAndOrientation(self.leo_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)

        if self.path_idx < len(self.path_to_follow):
            target_pos = self.path_to_follow[self.path_idx]
            dx, dy = target_pos[0] - pos[0], target_pos[1] - pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < 0.3:
                self.path_idx += 1
                return

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - yaw
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi

            linear_vel, angular_vel = (20.0, angle_diff * 5.0) if abs(angle_diff) <= 0.2 else (0.0, 5.0 if angle_diff > 0 else -5.0)
            
            l_speed, r_speed = linear_vel - angular_vel, linear_vel + angular_vel
            for j in self.left_wheels: p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=l_speed, force=30.0)
            for j in self.right_wheels: p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=r_speed, force=30.0)

            # 골인 체크
            if dist < 0.4 and target_pos == self.goal_coords:
                self.goal_reached = True
                elapsed = time.time() - self.start_time
                self.get_logger().info(f"미션 완료! 주행 시간: {int(elapsed//60):02d}:{int(elapsed%60):02d}")
                p.addUserDebugText("MISSION COMPLETE!", [pos[0], pos[1], 1.8], [0, 1, 0], 2)

        # 카메라 추적
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=yaw*180/math.pi-90, cameraPitch=-50, cameraTargetPosition=pos)

def main(args=None):
    rclpy.init(args=args)
    node = MazeLeoRover()
    try:
        rclpy.spin(node) # ROS 2 루프 실행
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()