import time
import numpy as np
import pybullet as p
import pybullet_data
import pandas as pd


PB_BallMass = 0.450  # масса шара
PB_BallRadius = 0.35  # радиус шара
# Инициализация PyBullet
physicsClient = p.connect(p.DIRECT, options=' --mp4=moviename.mp4')# запись видео
# Установка параметров камеры
cameraDistance = 30
cameraYaw = 0
cameraPitch = -15
cameraTargetPosition = [0, 0, 0]

restitution = 0.5
friction = 0.3

# Установка параметров камеры с помощью функции resetDebugVisualizerCamera
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

p.setGravity(0, 0, -9.81)

# Загрузка модели

class Ball():
    def __init__(self,ballPosition, ballOrientation):
        #ballPosition = [0, 0, 2]
        #ballOrientation = [0, 0, 0, 1]

        self.ballColShape = p.createCollisionShape(p.GEOM_SPHERE, radius=PB_BallRadius)

        self.ballVisualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=PB_BallRadius, rgbaColor=[0.25, 0.75, 0.25, 1])
        self.p_ballId = p.createMultiBody(PB_BallMass, self.ballColShape, self.ballVisualShapeId, ballPosition,
                                                    ballOrientation)  # (mass, collisionShape, visualShape, ballPosition, ballOrientation)
        p.changeVisualShape(self.p_ballId,-1,rgbaColor=[1,0.27,0,1])





#ball = Ball([0, 0, 2],[0, 0, 0, 1])
# Подготовка DataFrame для записи данных
df = pd.DataFrame(columns=['Throw_ID','time_step', 'pos_x', 'pos_y', 'pos_z'])

# Цикл симуляции и запись данных
for i in range(60):
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    # Создание формы коллизии для поверхности (например, плоскости)
    plane_shape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
    # Создание мульти-объекта (многотела) для представления поверхности
    plane_body = p.createMultiBody(baseCollisionShapeIndex=plane_shape, basePosition=[0, 0, 0])
    p.changeDynamics(plane_body, -1, restitution=restitution)
    p.changeDynamics(plane_body, -1, lateralFriction=friction)

    ball = Ball([0, 0, 5],[0, 0, 0, 5])
    p.changeDynamics(ball.p_ballId, -1, restitution=restitution)
    p.changeDynamics(ball.p_ballId, -1, lateralFriction=friction)
    p_xforce = np.random.uniform(500,1500,1)
    p_yforce = np.random.uniform(500,1500,1)
    p_zforce = np.random.uniform(500,1500,1)
    p.applyExternalForce(ball.p_ballId, -1, [p_xforce, p_yforce, p_zforce], [0, 0, 0], p.LINK_FRAME)
    pos, _ = p.getBasePositionAndOrientation(ball.p_ballId)
    j = 0
    while pos[2] > PB_BallRadius:
        p.stepSimulation()
        pos, _ = p.getBasePositionAndOrientation(ball.p_ballId)
        df = df._append({'Throw_ID': i,'time_step': j, 'pos_x': pos[0], 'pos_y': pos[1], 'pos_z': pos[2]}, ignore_index=True)
        #time.sleep(1. / 240.)
        j+=1

# Сохраняем датасет
df.to_csv('simulation_dataset.csv', index=False)

# Закрытие PyBullet
p.disconnect()