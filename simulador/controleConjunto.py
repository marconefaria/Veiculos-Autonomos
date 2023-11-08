import csv
import class_car as cp
import math
import numpy as np
import matplotlib.pyplot as plt

from controladorStanley import StanleyController
plt.ion()
plt.figure(1)

# Cria comunicação com o carrinho
car = cp.CarCoppelia()
car.startMission()

vel = []
time = []
posX = []
posY = []

# Velocidade de referência
car.setU(5)
velocidade_referencia = 0.5

# Configuração do gráfico
plt.ion()  # Modo interativo
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
ax1.set_xlabel('Tempo (s)')
ax1.set_ylabel('Velocidade (m/s)')
ax1.set_title('Gráfico de Velocidade')
ax1.set_ylim(-1, 1)  # Ajuste os limites conforme necessário
ax2.set_xlabel('Tempo (s)')
ax2.set_ylabel('X')
ax2.set_title('Gráfico de Posição')
ax2.set_ylim(-1, 1)  # Ajuste os limites conforme necessário

line1, = ax1.plot(time, vel, 'r', label='Velocidade Real')
ax1.axhline(y=velocidade_referencia, color='b', linestyle='--', label='Velocidade de Referência')
ax1.legend()

line2, = ax2.plot(time, posX, 'r', label='Posição atual')
ax2.axhline(y=0, color='k', linestyle='--', label='Reta x = 0')
ax2.legend()

# Parâmetros do controlador longitudinal (PID)
Kp_longitudinal = 1.0
Ki_longitudinal = 0.1
Kd_longitudinal = 0.01
integrated_error_longitudinal = 0.0
previous_error_longitudinal = 0.0

# Parâmetros do controlador lateral (Stanley)
k_lateral = 0.01
k_crosstrack = 10.0
max_steering_angle = 0.5  # Defina o ângulo máximo de esterçamento desejado

while car.t < 3.0:
    # Lê sensores
	car.step()

    # Medida da velocidade real (pode ser obtida na prática com sensores)
	velocidade_real = car.v
	print("Velocidade Real:", velocidade_real)

    # Erro de velocidade para o controlador longitudinal
	erro_longitudinal = velocidade_referencia - car.v

    # Ação de controle do PID (controlador longitudinal)
	integrated_error_longitudinal += erro_longitudinal
	derivative_error_longitudinal = erro_longitudinal - previous_error_longitudinal
	control_signal_longitudinal = (
    	Kp_longitudinal * erro_longitudinal +
    	Ki_longitudinal * integrated_error_longitudinal +
    	Kd_longitudinal * derivative_error_longitudinal
	)

    # Calcula o ângulo de direção com base no controlador lateral (Stanley)
	current_pose = np.array([car.getPos()[0], car.getPos()[1]])
	reference_pose = np.array([0, 0])
	steering_controller = StanleyController(k_lateral, k_crosstrack, max_steering_angle)
	steering_angle = steering_controller.calculate_steering_angle(current_pose, reference_pose)

    # Define a abertura do acelerador e o ângulo de direção com base nos controles
	car.setU(control_signal_longitudinal)
	car.setSteer(steering_angle)

    # Salva dados para plotagem
	vel.append(car.v)
	time.append(car.t)
	pos_x = car.getPos()[0]
	pos_y = car.getPos()[1]
	posX.append(pos_x)
	posY.append(pos_y)

    # Atualiza o erro anterior para o controlador longitudinal
	previous_error_longitudinal = erro_longitudinal

	line1.set_data(time, vel)
	line2.set_data(time, posX)

    # Ajuste os limites dos eixos conforme os dados aumentam
	ax1.relim()
	ax1.autoscale_view()
	ax2.relim()
	ax2.autoscale_view()

    # Redesenha os gráficos
	plt.draw()
	plt.pause(0.01)  # Tempo de espera para atualização (em segundos)

time = np.array(time)
vel = np.array(vel)

plt.savefig('testeFinal.png')

print('Terminou...')
