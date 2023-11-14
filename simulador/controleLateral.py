import class_car as cp
import numpy as np
import matplotlib.pyplot as plt
import csv

from controladorStanley import StanleyController
plt.ion()
plt.figure(1)

########################################
# cria comunicação com o carrinho
car = cp.CarCoppelia()
car.startMission()

# Parâmetros do controlador

posX = []
posY = []
time = []

# Velocidade de referência,
car.setU(5)

# Parâmetros do controlador lateral (Stanley)
k_crosstrack = 0.01
referencia_y = np.linspace(-3, 0, 60)
i = 0

velocidade_referencia = car.getVel()[0]
print(car.getVel()[0])

while car.t < 3.0:
    # lê sensores
    car.step()

    k = 10

# Posição atual do veículo (x, y, ângulo)
    current_pose = np.array([car.getPos()[0], car.getPos()[1]])

    controller = StanleyController(k_crosstrack)
    reference_pose = np.array([0, referencia_y[i]])

# Calcula o ângulo de direção
    steering_angle = controller.calculate_steering_angle(
        current_pose, reference_pose, car.v, car.getYaw())
    car.setSteer(-steering_angle)

    print(car.getPos()[1], referencia_y[i])

    i = i + 1

    # Salva dados
    time.append(car.t)
    pos_x = car.getPos()[0]
    pos_y = car.getPos()[1]
    posX.append(pos_x)
    posY.append(pos_y)

    plt.clf()
    plt.axhline(y=0, color='k', linestyle='--', label='Reta x = 0')
    plt.plot(time, posX, 'r', label='Posição atual')
    plt.xlabel('Tempo(s)')
    plt.ylabel('X')
    plt.legend()
    plt.show()
    plt.pause(0.01)

plt.savefig('testeLateral.png')

print('Terminou...')
