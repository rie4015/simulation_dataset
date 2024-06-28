import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
import pandas as pd
import time

air_friction_coefficient = 0.1

# Применение силы сопротивления воздуха к объекту
def apply_air_friction(object_id, linear_velocity):
    air_friction_force = [-air_friction_coefficient * v for v in linear_velocity]
    p.applyExternalForce(object_id, -1, forceObj=air_friction_force, posObj=[0,0,0], flags=p.LINK_FRAME)




PB_BallMass = 0.450  # масса шара
PB_BallRadius = 0.35  # радиус шара
# Инициализация PyBullet
physicsClient = p.connect(p.DIRECT)# запись видео
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
        self.p_ballId = p.createMultiBody(PB_BallMass, self.ballColShape, self.ballVisualShapeId,
                                          basePosition = ballPosition,
                                                    baseOrientation = ballOrientation)
        # (mass, collisionShape, visualShape, ballPosition, ballOrientation)
        p.changeVisualShape(self.p_ballId,-1,rgbaColor=[1,0.27,0,1])




p.setTimeStep(1/30)
#ball = Ball([0, 0, 2],[0, 0, 0, 1])
# Подготовка DataFrame для записи данных
df = pd.DataFrame(columns=['Throw_ID','time_step', 'pos_x', 'pos_y', 'pos_z'])
df.to_csv('simulation_dataset.csv', index=False)
steps = 5000
# Цикл симуляции и запись данных
start = time.time()
end = time.time() - start
for i in range(steps):
    print(i, '/', steps, end)
    start = time.time()
    #print(sys.getsizeof(df))
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    # Создание формы коллизии для поверхности (например, плоскости)
    plane_shape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
    # Создание мульти-объекта (многотела) для представления поверхности
    plane_body = p.createMultiBody(baseCollisionShapeIndex=plane_shape, basePosition=[0, 0, 0])
    p.changeDynamics(plane_body, -1, restitution=restitution)
    p.changeDynamics(plane_body, -1, lateralFriction=friction)
    ballpos = np.random.uniform(-10,10,4)
    ballpos[2]= PB_BallRadius + 0.01
    ballorin = ballpos
    ballorin[3] = 0

    ball = Ball(ballpos[0:3], ballorin)
    p.changeDynamics(ball.p_ballId, -1, restitution=restitution)
    p.changeDynamics(ball.p_ballId, -1, lateralFriction=friction)
    p_xforce = np.random.uniform(500,10000,1)
    p_yforce = np.random.uniform(500,10000,1)
    p_zforce = np.random.uniform(5000,15000,1)
    p_xforce[0], p_yforce[0],p_zforce[0] = 1000,1000,10000
    p.applyExternalForce(ball.p_ballId, -1, [p_xforce[0], p_yforce[0], p_zforce[0]], [0,0,0], p.LINK_FRAME)
    pos, _ = p.getBasePositionAndOrientation(ball.p_ballId)
    for j in range(64):
        p.stepSimulation()
        linearVelocity, _ = p.getBaseVelocity(ball.p_ballId)
        apply_air_friction(ball.p_ballId, linearVelocity)
        pos, _ = p.getBasePositionAndOrientation(ball.p_ballId)
        df = df._append({'Throw_ID': i,'time_step': j, 'pos_x': pos[0], 'pos_y': pos[1], 'pos_z': pos[2]}, ignore_index=True)
        #time.sleep(1. / 240.)
    end = time.time() - start
    df.to_csv('simulation_dataset.csv', index=False, mode='a', header=False)
    df.drop(df.index, inplace=True)


# Сохраняем датасет
#df.to_csv('simulation_dataset.csv', index=False, mode='a')

# Закрытие PyBullet
p.disconnect()
