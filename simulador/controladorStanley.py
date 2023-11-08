import math
import numpy as np

class StanleyController:
    def __init__(self, k, k_crosstrack, max_steering_angle):
        self.k = k  # Ganho do controlador
        self.k_crosstrack = k_crosstrack  # Ganho do erro lateral
        self.max_steering_angle = max_steering_angle  # Ângulo máximo de esterçamento

    def calculate_steering_angle(self, current_pose, reference_pose):
        # Calcula o erro lateral (cross-track error) separadamente para as coordenadas x e y
        erro_x = reference_pose[0] - current_pose[0]
        erro_y = reference_pose[1] - current_pose[1]

        # Calcula o módulo do erro lateral
        erro = math.sqrt(erro_x**2 + erro_y**2)

        # Calcula o ângulo de direção
        steering_angle = self.k * np.arctan2(1.0, self.k_crosstrack * erro)

        # Limita o ângulo de direção dentro dos limites
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        print(steering_angle)

        return steering_angle