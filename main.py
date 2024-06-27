import time

import pybullet as p
import pybullet_data
import pandas as pd


PB_BallMass = 0.450  # масса шара
PB_BallRadius = 0.2  # радиус шара
# Инициализация PyBullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Загрузка модели


ballPosition = [0, 0, 1]
ballOrientation = [0, 0, 0, 1]
ballColShape = p.createCollisionShape(p.GEOM_SPHERE, radius=PB_BallRadius)
ballVisualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=PB_BallRadius, rgbaColor=[0.25, 0.75, 0.25, 1])
p_ballId = p.createMultiBody(PB_BallMass, ballColShape, ballVisualShapeId, ballPosition,
                                            ballOrientation)  # (mass, collisionShape, visualShape, ballPosition, ballOrientation)
p.changeVisualShape(p_ballId,-1,rgbaColor=[1,0.27,0,1])



p_xforce = 0
p_yforce = 1000 * PB_BallMass
p_zforce = 1000 * PB_BallMass
p.applyExternalForce(p_ballId, -1, [p_xforce, p_yforce, p_zforce], [0, 0, 0], p.LINK_FRAME)


# Подготовка DataFrame для записи данных
df = pd.DataFrame(columns=['time_step', 'pos_x', 'pos_y', 'pos_z'])

# Цикл симуляции и запись данных
for i in range(400):
    p.stepSimulation()
    pos, _ = p.getBasePositionAndOrientation(p_ballId)
    df = df._append({'time_step': i, 'pos_x': pos[0], 'pos_y': pos[1], 'pos_z': pos[2]}, ignore_index=True)
    time.sleep(1. / 240.)

# Сохраняем датасет
df.to_csv('simulation_dataset.csv', index=False)

# Закрытие PyBullet
p.disconnect()