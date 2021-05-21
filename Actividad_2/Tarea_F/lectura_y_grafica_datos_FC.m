% Lectura de los datos proporcionados por 
% el IMU en formato csv

clear,close,clc;

% Se leen los datos del archivo a partir de la fila 1
% y la columna 0. Esto se configura en el 2º y 3º
% parametro de la función 'csvread'
M = csvread('datos4.csv', 1, 0);
t = M(:,1) / 1000;
AX  = M(:,2);
AY  = M(:,3);
GX  = M(:,4);
GY  = M(:,5);
FCX = M(:,6);
FCY = M(:,7);

subplot(2,1,1);
plot(t,AX,'r');
title('Variación posición eje X (ROLL)');
xlabel('Tiempo (s)');
ylabel('Cambio de posición');
hold on;
plot(t,GX,'g');
hold on;
plot(t,FCX,'b');
hold off;
legend('acelerómetro X', 'giroscopio X','X Con Filtro');

subplot(2,1,2);
plot(t,AY,'r');
title('Variación posición eje Y (PITCH)');
xlabel('Tiempo (s)');
ylabel('Cambio de posición');
hold on;
plot(t,GY,'g');
hold on;
plot(t,FCY,'b');
legend('acelerómetro Y', 'giroscopio Y','Y Con Filtro');