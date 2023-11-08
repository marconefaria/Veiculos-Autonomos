import csv
import class_car as cp
import numpy as np
import matplotlib.pyplot as plt
plt.ion()
plt.figure(1)

########################################
# cria comunicação com o carrinho
car = cp.CarCoppelia()
car.startMission()

vel = []
time = []

# Parâmetros do controlador PID
Kp = 1.0
Ki = 0.1
Kd = 0.01
integrated_error = 0.0
previous_error = 0.0

# Velocidade de referência,
car.setU(5)
velocidade_referencia = 0.5

while car.t < 3.0:
	# lê sensores
	car.step()
	
	# Medida da velocidade real (pode ser obtida na prática com sensores)
	velocidade_real = car.v
	print(velocidade_real)
	
	# Erro de velocidade
	erro = velocidade_referencia - car.v
	
	# Ação de controle do PID
	integrated_error += erro
	derivative_error = erro - previous_error
	control_signal = Kp * erro + Ki * integrated_error + Kd * derivative_error
	
	# Define a abertura do acelerador com base no controle
	car.setU(control_signal)
	
	# Salva dados para plotagem
	vel.append(car.v)
	time.append(car.t)
	
	# Atualiza o erro anterior
	previous_error = erro
	
	# Exibe gráfico em tempo real
	plt.clf()
	plt.plot(time, vel, 'r', label='Velocidade Real')
	plt.axhline(y=velocidade_referencia, color='b', linestyle='--', label='Velocidade de Referência')
	plt.xlabel('Tempo (s)')
	plt.ylabel('Velocidade (m/s)')
	plt.legend()
	plt.show()
	plt.pause(0.01)
	
time = np.array(time)
vel = np.array(vel)

with open('dadosT2.csv', 'w', newline='') as arquivo_csv:
    writer = csv.writer(arquivo_csv)
    writer.writerow(['Tempo', 'Velocidade'])  # Escreva cabeçalho
    for i in range(len(time)):
        writer.writerow([time[i], vel[i]])

plt.savefig('teste1.png')
	
print('Terminou...')
