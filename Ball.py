import pybullet as pb
import time
import pybullet_data
from matplotlib import pyplot as plt, animation

pb.connect(pb.DIRECT)

# поверхность
floorColShape = pb.createCollisionShape(pb.GEOM_PLANE)
# для плоскости (GEOM_PLANE), visualShape - не отображается при рендеринге, будем использовать GEOM_BOX
floorVisualShapeId = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[100, 100, 0.0001], rgbaColor=[1, 1, .98, 1])
pb_floorId = pb.createMultiBody(0, floorColShape, floorVisualShapeId, [0, 0, 0], [0, 0, 0, 1])

# шар
PB_BallRadius = 0.2
PB_BallMass = 0.5
ballPosition = [0, 0, 5]
ballOrientation = [0, 0, 0, 1]
ballColShape = pb.createCollisionShape(pb.GEOM_SPHERE, radius=PB_BallRadius)
ballVisualShapeId = pb.createVisualShape(pb.GEOM_SPHERE, radius=PB_BallRadius, rgbaColor=[1, 0.27, 0, 1])
pb_ballId = pb.createMultiBody(PB_BallMass, ballColShape, ballVisualShapeId, ballPosition, ballOrientation)

# указатель цели
TARGET_Z = 8
targetPosition = [0, 0, TARGET_Z]
targetOrientation = [0, 0, 0, 1]
targetVisualShapeId = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[1, 0.025, 0.025], rgbaColor=[0, 0, 0, 1])
pb_targetId = pb.createMultiBody(0, -1, targetVisualShapeId, targetPosition, targetOrientation)


pb.setGravity(0,0,-10)
pb.setTimeStep(1./60)

pb_force = 10 * PB_BallMass
pb.applyExternalForce(pb_ballId, -1, [0,0,pb_force], [0,0,0], pb.LINK_FRAME)

# текущее положение и ориентация (берем только положение по оси Z curPos[2])
curPos, curOrient = pb.getBasePositionAndOrientation(pb_ballId)
# линейная и угловая скорость (берем только линейную скорость по оси Z lin_vel[2])
lin_vel, ang_vel= pb.getBaseVelocity(pb_ballId)

camTargetPos = [0, 0, 5]  # расположение цели (фокуса) камеры
camDistance = 10  # дистанция камеры от цели
yaw = 0  # угол рыскания относительно цели
pitch = 0  # наклон камеры относительно цели
roll = 0  # угол крена камеры относительно цели
upAxisIndex = 2  # ось вертикали камеры (z)

fov = 60  # угол зрения камеры
nearPlane = 0.01  # расстояние до ближней плоскости отсечения
farPlane = 20  # расстояние до дальной плоскости отсечения
pixelWidth = 320  # ширина изображения
pixelHeight = 200  # высота изображения
aspect = pixelWidth / pixelHeight;  # соотношение сторон изображения

# видовая матрица
viewMatrix = pb.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
# проекционная матрица
projectionMatrix = pb.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
# рендеринг изображения с камеры
img_arr = pb.getCameraImage(pixelWidth, pixelHeight, viewMatrix, projectionMatrix, shadow=0, lightDirection=[0, 1, 1],
                            renderer=pb.ER_TINY_RENDERER)
w = img_arr[0]  # width of the image, in pixels
h = img_arr[1]  # height of the image, in pixels
rgb = img_arr[2]  # color data RGB
dep = img_arr[3]  # depth data



print(w,h,rgb,dep)

pb.disconnect()