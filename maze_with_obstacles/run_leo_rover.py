import pybullet as p
import pybullet_data
import xacro
import os
import re
import time

# 1. 파일 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(current_dir, "leo_common-master", "leo_description")
xacro_file = os.path.join(base_path, "urdf", "leo.urdf.xacro")

print(f"사용하려는 Xacro 파일: {xacro_file}")

# 2. Xacro 파일 수동 전처리 (ROS 종속성 제거)
try:
    with open(xacro_file, 'r') as f:
        content = f.read()
    
    # $(find leo_description) 구문을 실제 base_path로 강제 치환
    # 이 작업이 'ament_index_python' 오류를 방지합니다.
    content = content.replace("$(find leo_description)", base_path)
    
    # 치환된 내용을 바탕으로 xacro 프로세싱 수행
    doc = xacro.parse(content)
    xacro.process_doc(doc)
    robot_description = doc.toxml()
    
    # Pybullet 메쉬 경로 인식 문제 해결
    robot_description = robot_description.replace("package://leo_description", base_path)

except Exception as e:
    print(f"Xacro 변환 중 오류 발생: {e}")
    exit()

# 3. 임시 URDF 파일 저장
urdf_file_path = os.path.join(current_dir, "temp_leo.urdf")
with open(urdf_file_path, "w") as f:
    f.write(robot_description)

# 4. Pybullet 시뮬레이션 설정
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# 5. 로봇 로드
try:
    # 로봇 로드
    leo_id = p.loadURDF(urdf_file_path, basePosition=[0, 0, 0.2])

    # --- 로봇의 조인트 정보 로드 ---
    # 조인트 정보를 담을 리스트
    # wheel_joints = []

    # print("\n===== Leo Rover Joint Information =====")
    # num_joints = p.getNumJoints(leo_id)

    # for i in range(num_joints):
    #     info = p.getJointInfo(leo_id, i)
    #     joint_index = info[0]
        
    #     # 조인트 이름 처리 (bytes일 경우만 decode 실행)
    #     joint_name_raw = info[1]
    #     if isinstance(joint_name_raw, bytes):
    #         joint_name = joint_name_raw.decode('utf-8')
    #     else:
    #         joint_name = joint_name_raw
            
    #     joint_type = info[2] # 0: REVOLUTE, 1: PRISMATIC, 4: FIXED 등

    #     # 바퀴 조인트 찾기 (이름에 'wheel'이 포함된 조작 가능한 조인트)
    #     if "wheel" in joint_name and joint_type != 4:
    #         wheel_joints.append(joint_index)
            
    #     print(f"Index: {joint_index} | Name: {joint_name} | Type: {joint_type}")

    # print(f"\n확인된 바퀴 조인트 인덱스: {wheel_joints}")
    # print("=======================================\n")

    wheel_joints = [2, 3, 5, 6]

    for j in wheel_joints:
        p.changeDynamics(leo_id, j, lateralFriction=0.7)

    print("\n 조작 방법 : 방향키 (위/아래 : 전후진, 좌/우: 회전)")

    # --- 색상 수정 코드 시작 ---
    num_joints = p.getNumJoints(leo_id)
    GRAY = [0.5, 0.5, 0.5, 1.0]
    BLACK = [0.1, 0.1, 0.1, 1.0]
    BLUE = [0.0, 0.0, 0.8, 1.0]

    # 1. 몸체(Chassis) 색상 바꾸기
    # base_link (index -1)의 색상을 설정합니다.
    # [R, G, B, Alpha] (0~1 사이 값)
    p.changeVisualShape(leo_id, -1, rgbaColor=GRAY)

    # 2. 나머지 부품(바퀴, 로커 등) 색상 바꾸기
    for i in range(num_joints):
        joint_info = p.getJointInfo(leo_id, i)
        link_name = joint_info[12].decode('utf-8')
        
        # 바퀴(wheel)가 이름에 포함된 경우 검은색으로 변경
        if "wheel" in link_name:
            p.changeVisualShape(leo_id, i, rgbaColor=BLACK)
        
        # 로커(rocker) 부품은 파란색으로 변경
        elif "rocker" in link_name:
            p.changeVisualShape(leo_id, i, rgbaColor=BLUE)
            
        # 안테나나 카메라 등 기타 부품 색상
        else:
            p.changeVisualShape(leo_id, i, rgbaColor=GRAY)
    # --- 색상 수정 코드 끝 ---
    print(f"Leo Rover 로드 성공! ID: {leo_id}")
except Exception as e:
    print(f"URDF 로드 중 오류 발생: {e}")

while True:
    # 1. 키보드 이벤트 읽기
    keys = p.getKeyboardEvents()
    
    linear_vel = 0   # 전진/후진 속도
    angular_vel = 0  # 회전 속도

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        linear_vel = 10
    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        linear_vel = -10
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        angular_vel = 20
    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        angular_vel = -20

    # 2. 차동 구동(Differential Drive) 로직 적용
    # 왼쪽 바퀴(2, 3)와 오른쪽 바퀴(5, 6)의 속도를 다르게 주어 회전 구현
    left_speed = linear_vel - angular_vel
    right_speed = linear_vel + angular_vel

    # 3. 각 바퀴에 속도 명령 전달
    # 속도 제어 코드 부분 수정
    for i, joint_id in enumerate(wheel_joints):
        if i < 2: target_v = left_speed
        else: target_v = right_speed
        
        p.setJointMotorControl2(
            bodyUniqueId=leo_id, 
            jointIndex=joint_id, 
            controlMode=p.VELOCITY_CONTROL, 
            targetVelocity=target_v,
            force=10.0  # 모터의 최대 힘을 조절 (기본값보다 높여보세요)
        )

    # 4. 물리 연산 수행
    p.stepSimulation()
    time.sleep(1./240.) # 시뮬레이션 속도 안정화