import pybullet as p
import pybullet_data
import time

# 1. 시뮬레이션 서버 연결 및 기본 설정
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 2. 바닥 및 환경 로드
p.loadURDF("plane.urdf")

# 3. 가변 로봇의 간이 모델 생성 (상자 두 개를 조인트로 연결)
# 실제 프로젝트에서는 URDF 파일을 로드하지만, 여기서는 코드로 직접 생성합니다.
base_id = p.loadURDF("r2d2.urdf", [0, 0, 0.5]) # 예시용 기본 로봇 모델

print("\n" + "="*50)
print("컨트롤 가이드:")
print("- 방향키: 로봇 이동")
print("- 'W' 키: 로봇 폭 확대 (Width Expansion)")
print("- 'S' 키: 로봇 폭 축소 (Width Contraction)")
print("="*50)

# 4. 실시간 루프
width = 0.5
try:
    while True:
        # 키보드 이벤트 처리
        keys = p.getKeyboardEvents()
        
        if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
            width += 0.01
            print(f"로봇 폭 확대 중: {width:.2f}m")
        if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            width -= 0.01
            print(f"로봇 폭 축소 중: {width:.2f}m")

        # 여기에 가변 조인트 제어 로직(p.setJointMotorControl2)이 들어갈 예정입니다.
        
        p.stepSimulation()
        time.sleep(1./240.)

except KeyboardInterrupt:
    p.disconnect()