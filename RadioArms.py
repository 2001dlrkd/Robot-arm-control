import time
import math
import pybullet as p
import pybullet_data as pd
import pygame

# === PyBullet 초기화 ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setTimeStep(1/240)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# === 조인트 정리 ===
num_joint = p.getNumJoints(robot)
movable = []
finger = []

for i in range(num_joint):
    info = p.getJointInfo(robot, i)
    name = info[1].decode()
    jtype = info[2]

    if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        if "finger" in name:
            finger.append(i)
        else:
            movable.append(i)

arm = movable  # 0~6: 7개 관절

# === 기본 위치로 이동 ===
home = [0, -0.5, 0, -2.0, 0, 1.5, 0.8]
joint_positions = home.copy()

p.setJointMotorControlArray(
    robot, arm,
    controlMode=p.POSITION_CONTROL,
    targetPositions=joint_positions,
    positionGains=[0.04] * len(arm)
)

# === Pygame 초기화 및 조이스틱 연결 ===
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 조이스틱을 찾을 수 없습니다.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"✅ 조이스틱 연결됨: {joystick.get_name()}")

# === 메인 루프 ===
while p.isConnected():
    pygame.event.pump()  # 이벤트 큐 처리

    # 🎮 왼쪽 스틱 X, Y
    axis_x = joystick.get_axis(0)  # -1.0 ~ +1.0
    axis_y = joystick.get_axis(1)

    # 첫 번째 관절 (회전) - 왼쪽 스틱 x축으로 조정
    joint_positions[0] += axis_x * 0.01  # 민감도 조절
    joint_positions[0] = max(-2.8, min(2.8, joint_positions[0]))  # 제한

    # 두 번째 관절 (위아래) - 왼쪽 스틱 y축으로 조정
    joint_positions[1] += axis_y * 0.01
    joint_positions[1] = max(-1.7, min(1.7, joint_positions[1]))

    # 관절 위치 적용
    p.setJointMotorControlArray(
        robot, arm,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_positions,
        positionGains=[0.05] * len(arm)
    )

    p.stepSimulation()
    time.sleep(1/240)
