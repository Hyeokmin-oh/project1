import pybullet as p
import pybullet_data
import time

# --- 1. 시뮬레이션 환경 설정 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# 로봇 로드
robot_id = p.loadURDF("AMR_Robot.urdf", [0, 0, 0.2])

# --- 2. 수동 조인트 매핑 (URDF 작성 순서 기준) ---
# 터미널에 출력되는 순서를 확인하고 숫자가 다르면 아래만 수정하세요.
WIDTH_JOINT    = 0  # width_joint
WHEEL_LF_BACK  = 1  # joint_wheel_lf_back
WHEEL_LF_FRONT = 2  # joint_wheel_lf_front
WHEEL_RT_BACK  = 3  # joint_wheel_rt_back
WHEEL_RT_FRONT = 4  # joint_wheel_rt_front

# 그룹화 (제어 편의성)
left_wheels = [WHEEL_LF_BACK, WHEEL_LF_FRONT]
right_wheels = [WHEEL_RT_BACK, WHEEL_RT_FRONT]

# --- 3. 제어 파라미터 ---
max_v = 15.0       # 이동 속도
steer_v = 8.0      # 회전 속도
current_width = 0.25 # 가변 몸체 위치

print("\n" + "="*30)
print("조종법:")
print("- 위/아래 방향키: 전진/후진")
print("- 좌/우 방향키: 좌회전/우회전")
print("- A / D 키: 몸체 폭 확장/축소")
print("="*30)

# 인덱스 검증을 위한 출력 (실행 시 터미널 확인용)
for i in range(p.getNumJoints(robot_id)):
    print(f"Index {i}: {p.getJointInfo(robot_id, i)[1].decode('utf-8')}")

while True:
    p.stepSimulation()
    keys = p.getKeyboardEvents()
    
    linear = 0
    angular = 0
    
    # --- 주행 제어 (위/아래: 전후진, 좌/우: 회전) ---
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        linear = max_v
    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        linear = -max_v
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        angular = steer_v
    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        angular = -steer_v

    # 차동 주행 속도 계산
    v_l = linear - angular
    v_r = linear + angular

    # 왼쪽 바퀴 제어
    for l_idx in left_wheels:
        p.setJointMotorControl2(robot_id, l_idx, p.VELOCITY_CONTROL, targetVelocity=v_l)
    
    # 오른쪽 바퀴 제어 (만약 거꾸로 돌면 앞에 -를 붙이세요)
    for r_idx in right_wheels:
        p.setJointMotorControl2(robot_id, r_idx, p.VELOCITY_CONTROL, targetVelocity=v_r)

    # --- 가변 몸체 제어 (A/D 키) ---
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        current_width = min(0.5, current_width + 0.005)
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        current_width = max(0.0, current_width - 0.005)

    p.setJointMotorControl2(robot_id, WIDTH_JOINT, p.POSITION_CONTROL, targetPosition=current_width)

    time.sleep(1./240.)