% Lectura de los datos proporcionados por 
% el IMU en formato csv

clear,close,clc;

% Se leen los datos del archivo a partir de la fila 1
% y la columna 0. Esto se configura en el 2º y 3º
% parametro de la función 'csvread'
M = csvread('datos3.csv', 1, 0);
t = M(:,1) / 1000;
RollX = M(:,2);
RollY = M(:,3);
PitchX= M(:,4);
PitchY= M(:,5);

subplot(2,1,1);
plot(t,RollX,'r');
title('Variación posición eje X (ROLL)');
xlabel('Tiempo (s)');
ylabel('Cambio de posición');
hold on;
plot(t,PitchX,'g');
hold off;
legend('acelerómetro X', 'giroscopio X');

subplot(2,1,2);
plot(t,RollY,'r');
title('Variación posición eje Y (PITCH)');
xlabel('Tiempo (s)');
ylabel('Cambio de posición');
hold on;
plot(t,PitchY,'g');
legend('acelerómetro Y', 'giroscopio Y');

