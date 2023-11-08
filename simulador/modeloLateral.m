% Parâmetros do controlador
k_crosstrack = 1.0; % Ganho do erro lateral
max_steering_angle = deg2rad(30.0); % Ângulo máximo de esterçamento

% Trajetória de referência (uma reta em x = 0)
reference_x = [0, 0];

% Simulação de tempo (por exemplo, de 0 a 10 segundos)
time = 0:0.1:10;

% Inicializa arrays para armazenar dados
current_pose = zeros(1, length(time));
steering_angles = zeros(1, length(time));

% Simulação do controlador Stanley
for i = 1:length(time)
    % Posição atual do veículo (assumindo que o veículo está na posição 0 inicialmente)
    current_pose(i) = 0;

    % Calcula o erro lateral (cross-track error)
    erro = reference_x(1) - current_pose(i);

    % Calcula o ângulo de direção
    steering_angle = atan2(k_crosstrack * erro, 1.0);

    % Limita o ângulo de direção dentro dos limites
    steering_angle = max(-max_steering_angle, min(steering_angle, max_steering_angle));

    % Armazena o ângulo de direção
    steering_angles(i) = steering_angle;
end

% Plotagem dos resultados
figure;
subplot(2, 1, 1);
plot(time, current_pose);
xlabel('Tempo (s)');
ylabel('Posição Atual');
title('Posição Atual do Veículo');

subplot(2, 1, 2);
plot(time, rad2deg(steering_angles));
xlabel('Tempo (s)');
ylabel('Ângulo de Direção (graus)');
title('Ângulo de Direção Calculado');

% Configuração da linha de referência
hold on;
yline(0, '--r', 'Reta x = 0');
legend('Ângulo de Direção', 'Reta x = 0');

% Fim da simulação
