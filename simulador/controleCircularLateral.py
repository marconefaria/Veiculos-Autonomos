import class_car as cp
import numpy as np
import matplotlib.pyplot as plt
import csv
from controladorStanley import StanleyController

plt.ion()
plt.figure(1)

# Cria comunicação com o carrinho
car = cp.CarCoppelia()
car.startMission()

# Parâmetros do controlador
posX = []
posY = []
time = []
refX = []
refY = []
banana = []
timeRef = []
time_ref = []

# Velocidade de referência
car.setU(5)

# Raio da trajetória circular (ajuste conforme necessário)
raio = 5.0

with open('referenciaCircular.csv', newline='') as csvfile:
	reader = csv.DictReader(csvfile)
    
	# Percorra as linhas do arquivo CSV
	for row in reader:
		# Acesse os valores das colunas pelo nome (por exemplo, "Tempo" e "Valor")
		x = float(row['PosicaoX'])
		y = float(row['PosicaoY'])
		tRef = float(row['Tempo'])
			
		# Armazene os valores nas listas
		refX.append(x)
		refY.append(y)
		timeRef.append(tRef)

i = 0

while car.t < 4.0:
    # Lê sensores
	car.step()

	time.append(car.t)
	posX.append(car.getPos()[0])
	posY.append(car.getPos()[1])

	car.step()
    
	k = 10.0
	k_crosstrack = 1.0
	max_steering_angle = np.radians(30.0)

	# Posição atual do veículo (x, y, ângulo)
	current_pose = np.array([car.getPos()[0], car.getPos()[1], car.getYaw()])

	# Calcula a trajetória circular como referência
	referenceX = refX[i]
	referenceY = refY[i]
	t_ref = timeRef[i]
	reference = np.array([referenceX, referenceY])

	# Calcula o ângulo de direção
	controller = StanleyController(k, k_crosstrack, max_steering_angle)
	steering_angle = controller.calculate_steering_angle(current_pose, reference)
	car.setSteer(steering_angle)

	# Salva dados
	time.append(car.t)
	pos_x = car.getPos()[0]
	pos_y = car.getPos()[1]
	posX.append(pos_x)
	posY.append(pos_y)
	banana.append(referenceX)
	time_ref.append(t_ref)
	print(referenceX)

	plt.clf()
	plt.plot(time, posX, 'r', label='Posição atual')
	plt.plot(time_ref, banana, 'k--', label='Trajetória Circular de Referência')
	#plt.plot(time, banana, 'k--', label='Trajetória Circular de Referência')
	plt.xlabel('Tempo(s)')
	plt.ylabel('X')
	plt.legend()
	plt.show()
	plt.pause(0.01)

	i += 1
        
plt.savefig('testeLateral.png')

print('Terminou...')
