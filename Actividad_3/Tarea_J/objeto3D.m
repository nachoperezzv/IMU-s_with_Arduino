% Creación de objeto 3D y rotación del mismo
close,clear,clc;

% Primero se leen los datos del archivo .csv
% Se leen los datos del archivo a partir de la fila 1
% y la columna 0. Esto se configura en el 2º y 3º
% parametro de la función 'csvread'
M     = csvread('datos4.csv', 1, 0);
t     = M(:,1);
Roll  = M(:,6);
Pitch = M(:,7);

i = 1;

% Definición de un objeto 3D con plot3
% x=[1 5]; y=[1 5]; z=[1 5];
% X=[x(1) x(1) x(2) x(2) x(1) x(1) x(1) x(1) x(2) x(2) x(1)];
% Y=[y(1) y(1) y(1) y(2) y(2) y(1) y(1) y(2) y(2) y(1) y(1)];
% Z=[z(1) z(1) z(1) z(1) z(1) z(1) z(2) z(2) z(2) z(2) z(2)];
% obj1 = plot3(X,Y,Z);
% hold on
% X1=[x(1) x(1) x(2) x(2) x(1) x(1) x(1) x(2) x(2) x(1)];
% Y1=[y(1) y(1) y(1) y(1) y(1) y(2) y(2) y(2) y(2) y(2)];
% Z1=[z(1) z(2) z(2) z(1) z(1) z(1) z(2) z(2) z(1) z(1)];
% obj2 = plot3(X1,Y1,Z1)
% xlabel('x-axis')
% ylabel('y-axis')
% zlabel('z-axis')

% Definicion de un objeto 3D con patch
obj1 = patch([0 0 1 1],[0 1 1 0],[1 1 1 1],'r');
obj2 = patch([0 1 1 0],[0 0 0 0],[0 0 1 1],'b');
obj3 = patch([0 0 0 0],[0 1 1 0],[0 0 1 1],'y');

view(-37.5, 30);
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
axis square;

rotate(obj1,[1 0 0],Roll(12,:)); rotate(obj1,[0 1 0],Pitch(12,:));
rotate(obj2,[1 0 0],Roll(12,:)); rotate(obj2,[0 1 0],Pitch(12,:));
rotate(obj3,[1 0 0],Roll(12,:)); rotate(obj3,[0 1 0],Pitch(12,:));


% Rotar el objeto el angulo recibido 
while i<200    
    rotate(obj1,[1 0 0],Roll(i,:)); rotate(obj1,[0 1 0],Pitch(i,:));
    rotate(obj2,[1 0 0],Roll(i,:)); rotate(obj2,[0 1 0],Pitch(i,:));
    rotate(obj3,[1 0 0],Roll(i,:)); rotate(obj3,[0 1 0],Pitch(i,:));
    i=i+1;
    pause(0.05);
end

close;
