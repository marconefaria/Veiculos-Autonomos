import math
import numpy as np

class StanleyController:
    def __init__(self, k, k_crosstrack):
        self.k = k  # Ganho do controlador
        self.k_crosstrack = k_crosstrack  # Ganho do erro lateral
        self.prev_error = 0  # Erro de direção anterior

    def calculate_steering_angle(self, current_pose, reference_pose, current_speed, current_yaw):
        # Extrai as coordenadas dos pontos
        a, b, c = calcular_coeficientes(reference_pose, current_pose)

        # Calcula o cross track error
        error = (a*current_pose[0] + b*current_pose[1] + c) / np.sqrt(a**2 + b**2)

        # Calcula o cross track steering
        cross_track_steering = np.arctan2(self.k_crosstrack * error, current_speed)

        # Calcula o heading error
        theta_c = math.atan2(reference_pose[1] - current_pose[1], reference_pose[0] - current_pose[0])
        heading_error = theta_c - current_yaw

        # Normaliza o resultado para o intervalo de -pi a pi
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi 

        # Calcula o steering angle
        steering_angle = math.radians(heading_error + cross_track_steering)

        return steering_angle
    
def calcular_coeficientes(ponto_referencia, ponto_atual):
    # Extrai as coordenadas dos pontos
    xr, yr = ponto_referencia
    xc, yc = ponto_atual

    # Calcula o ponto médio
    ponto_medio = ((xr + xc) / 2, (yr + yc) / 2)

    # Calcula a distância entre os dois pontos
    distancia = math.sqrt((xc - xr)**2 + (yc - yr)**2)

    # Calcula os coeficientes
    a = 1 / distancia
    b = -1 / distancia
    c = -a * ponto_medio[0] - b * ponto_medio[1]

    return a, b, c