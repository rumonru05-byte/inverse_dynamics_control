%% Justificación Analítica del Controlador PD (Feedback Linearization)
clear; clc; close all;

s = tf('s');

% Definimos 3 casos de ajuste para comparar:
Kp1 = 100; Kd1 = 5; 
G1 = Kp1 / (s^2 + Kd1*s + Kp1);

Kp2 = 100; Kd2 = 20; 
G2 = Kp2 / (s^2 + Kd2*s + Kp2);

Kp3 = 100; Kd3 = 40; 
G3 = Kp3 / (s^2 + Kd3*s + Kp3);

%% Graficar la comparativa con estilo Modo Oscuro
% Color gris oscuro exacto
colorFondo = [0.11 0.11 0.11]; 

% Creamos la figura forzando el color de fondo
figure('Name', 'Comparativa de Ajuste PD', 'Color', colorFondo);
hold on;
step(G1, 1.5);
step(G2, 1.5);
step(G3, 1.5);

% Configuración de los ejes para modo oscuro
ax = gca;
ax.Color = colorFondo;       
ax.XColor = 'w';             
ax.YColor = 'w';
ax.GridColor = 'w';          
ax.GridAlpha = 0.3;          
ax.MinorGridColor = 'w';
ax.MinorGridAlpha = 0.15;
grid on; grid minor;

% Textos en blanco
title('Respuesta Temporal del Error Articular (Sistema Linealizado)', 'Color', 'w');
xlabel('Tiempo [s]', 'Color', 'w');
ylabel('Posicion Articular [rad]', 'Color', 'w');

% --- 1. LÍNEAS MÁS GORDAS ---
grosorLinea = 2.5; % Cambia este número si las quieres aún más anchas
set(findall(gcf,'type','line'), 'LineWidth', grosorLinea);

% --- 2. LEYENDA VISIBLE Y ADAPTADA ---
lgd = legend(sprintf('Subamortiguado (Kp=%d, Kd=%d)', Kp1, Kd1), ...
       sprintf('Críticamente Amortiguado (Kp=%d, Kd=%d)', Kp2, Kd2), ...
       sprintf('Sobreamortiguado (Kp=%d, Kd=%d)', Kp3, Kd3), ...
       'Location', 'best'); % 'best' busca el hueco donde no tape las líneas
lgd.TextColor = 'w';
lgd.Color = colorFondo;
lgd.EdgeColor = 'w';
lgd.FontSize = 11;

%% Guardado en SVG respetando el fondo oscuro
ruta_destino = 'ajuste_pd_control.svg'; 
exportgraphics(gcf, ruta_destino, 'ContentType', 'vector', 'BackgroundColor', 'current');