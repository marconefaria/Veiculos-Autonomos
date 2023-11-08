import csv
import math
import sys
import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt
plt.ion()
plt.figure(1)

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

vel = []
time = []
posX = []
posY = []

while car.t < 3.0:
	
	# lê senores
	car.step()
	car.setU(0.0)

	if car.t < 2.0:
		car.setU(20.0)
	else:
		car.setU(0.0)

	vel.append(car.v)
	time.append(car.t)
	posX.append(car.getPos()[0])
	posY.append(car.getPos()[1])
	
	# lê exibe gráfico
	plt.clf()
	plt.plot(time, vel, 'r')
	plt.show()
	plt.pause(0.01)
	
time = np.array(time)
vel = np.array(vel)
idx = np.where((time > 0.1) & (time < 3.0))
time = time[idx]
vel = vel[idx]

with open('dadosControleLateral.csv', 'w', newline='') as arquivo_csv:
    writer = csv.writer(arquivo_csv)
    writer.writerow(['Tempo', 'Velocidade', 'PosicaoX', 'PosicaoY'])  # Escreva cabeçalho
    for i in range(len(time)):
        writer.writerow([time[i], vel[i], posX[i], posY[i]])

plt.savefig('teste2.png')
	
print('Terminou...')