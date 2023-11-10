import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt

from controladorStanley import StanleyController

vel = []
time = []
posX = []
posY = []

velocidade_referencia = 0.5

kp = 0.1  # Ganho proporcional
ki = 0.01  # Ganho integral
kd = 0.05  # Ganho derivativo

# Variáveis globais para o PID
previous_error = 0
integral = 0

# HSV values
WHITE_HSV_MIN = [0, 0, 150]
WHITE_HSV_MAX = [255, 30, 255]

# mapa de plot
mapa = cv2.cvtColor(cv2.imread('./coppelia/pista.png'), cv2.COLOR_RGB2BGR)


def gera_graficos():
    # Configuração do gráfico
    plt.ion()
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
    ax1.axhline(y=velocidade_referencia, color='b',
                linestyle='--', label='Velocidade de Referência')
    ax1.legend()

    line2, = ax2.plot(time, posX, 'r', label='Posição atual')
    ax2.axhline(y=0, color='k', linestyle='--', label='Reta x = 0')
    ax2.legend()

    return line1, line2, ax1, ax2


def inicializa_veiculo():
    # Cria comunicação com o carrinho
    car = cp.CarCoppelia()
    car.startMission()

    # Velocidade de referência
    car.setU(5)

    return car


def blob(image, min_hsv, max_hsv, color=(0, 255, 0)):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # Define the lower and upper bounds of the color you want to detect
    lower_color = np.array(min_hsv)  # hue_min, saturation_min, value_min
    upper_color = np.array(max_hsv)  # hue_max, saturation_max, value_max
    # Create a mask using the inRange() function to extract the color of interest
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Initialize variables to keep track of the largest blob
    largest_area = 0
    largest_contour = None
    # Loop through the contours and calculate areas
    for i, contour in enumerate(contours):
        # Calculate the area of the contour
        area = cv2.contourArea(contour)
        # If the current blob has a larger area, update the largest area and contour
        if area > largest_area:
            largest_area = area
            largest_contour = contour
    # Draw the largest contour on the original image
    cx = cy = -1
    if largest_contour is not None:
        # Calculate the centroid of the largest contour
        M = cv2.moments(largest_contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        # Green contour around the largest blob
        cv2.drawContours(image, [largest_contour], -1, color, 2)
    return image, largest_area, cx, cy


def processa_imagem(car):
    # pega imagem
    image = car.getImage()
    # blobs
    image, white_area, _, _ = blob(
        image, WHITE_HSV_MIN, WHITE_HSV_MAX, color=(255, 255, 255))

    # salva e mostra valores
    plt.subplot(121)
    plt.cla()
    plt.gca().imshow(image, origin='lower')
    plt.title('t = %.1f' % car.t)

    plt.subplot(122)
    plt.cla()
    x = car.p[0]
    y = car.p[1]
    # plota mapa
    plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.99)
    plt.plot(x, y, 'r')
    plt.axis('equal')
    plt.box(False)
    plt.ylabel('y [m]')
    plt.ylabel('x [m]')

    plt.show()
    plt.pause(0.01)


def roadDetection(frame_bgr):
    # Converte para tons de cinza
    frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    # Aplica uma técnica de limiarização para segmentar a linha
    _, thresh = cv2.threshold(frame_gray, 200, 255, cv2.THRESH_BINARY)

    # Encontra as bordas na imagem limiarizada
    edges = cv2.Canny(thresh, 50, 150)

    # Encontra as linhas retas usando a Transformada de Hough
    lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=50)

    # Desenha a linha reta na imagem original
    result = frame_bgr.copy()

    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 2)

    return result


def main():
    global vel, time, posX, posY

    # Inicializa o veículo
    car = inicializa_veiculo()
    line1, line2, ax1, ax2 = gera_graficos()

    # Parâmetros do controlador longitudinal (PID)
    Kp_longitudinal = 1.0
    Ki_longitudinal = 0.1
    Kd_longitudinal = 0.01
    integrated_error_longitudinal = 0.0
    previous_error_longitudinal = 0.0

    # Parâmetros do controlador lateral (Stanley)
    k_crosstrack = 10.0

    referencia_y = 0

    i = 0

    while car.t < 3.0:
        # Lê sensores
        car.step()

        # processa_imagem(car)
        """ image_with_lines = roadDetection(car.getImage())

        plt.clf()
        plt.gca().imshow(image_with_lines, origin='lower')
        plt.axis('off')
        plt.show()
        plt.pause(0.01) """

        # Atualiza a referência de posição
        referencia_y = referencia_y + 0.5*car.t

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
        reference_pose = np.array([0, referencia_y])
        steering_controller = StanleyController(k_crosstrack)
        steering_angle = steering_controller.calculate_steering_angle(
            current_pose, reference_pose, car.v, car.getYaw())

        # Define a abertura do acelerador e o ângulo de direção com base nos controles
        car.setU(control_signal_longitudinal)
        car.setSteer(steering_angle)

        print(steering_angle, car.getYaw())

        i = i + 1

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


if __name__ == "__main__":
    main()
