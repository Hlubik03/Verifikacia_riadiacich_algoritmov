clc;
clear all;
close all;

%% System

% Zakladne parametre
m = 1000;  % hmotnosť vozidla v kg
b = 50;    % koeficient odporu (sila trenia/odporu)
pozadovane_rychlosti = 10:1:40;  % požadované rýchlosti vozidla 

% Časový interval simulácie
t = 0:0.1:20;  

% Prevodová funkcia systému
s = tf('s');  
P_cruise = 1/(m * s + b);  % Tempomat

% Hodnoty pre PI regulátor
Kp = 600;  
Ki = 20;
C = pid(Kp, Ki);  

% Uzavretá slučka s PI regulátorom a systémom tempomatu
uzavreta_slucka = feedback(C * P_cruise, 1);  

trvala_odchylka = zeros(size(pozadovane_rychlosti));  

%% Verifikacia
figure;
hold on;
box on;
grid on;

% Vektor 2 % hraníc
hranice_2_percent = 0.02 * pozadovane_rychlosti;

for i = 1:length(pozadovane_rychlosti)
    r = pozadovane_rychlosti(i);  
    [odozva, cas_odozvy] = step(r * uzavreta_slucka, t); 

    trvala_odchylka(i) = r - odozva(end);  
end

% Generovanie grafu odchýlok
p = plot(pozadovane_rychlosti, abs(trvala_odchylka), ...
    'DisplayName', 'Trvalá odchýlka');
set(p, 'LineWidth', 3);

% Pridanie čiary pre 2 % hranicu
p2 = plot(pozadovane_rychlosti, hranice_2_percent, '--r', ...
    'DisplayName', '2% hranica');
set(p2, 'LineWidth', 2);

% Nastavenie vlastností pre graf
set(gca, 'FontSize', 20, 'TickLabelInterpreter', 'latex');

% Nastavenie popiskov osí
xlabel_handle = xlabel('R\''{y}chlos\v{t} [m/s]', 'Interpreter', 'latex');
ylabel_handle = ylabel('$e(\infty)$ [m/s]', 'Interpreter', 'latex');

% Nastavenie veľkosti písma pre jednotlivé popisky
set(xlabel_handle, 'FontSize', 24);
set(ylabel_handle, 'FontSize', 24);

% Nastavenie názvu grafu
title_handle = title('Z\''{a}vislos\v{t} $e(\infty)$ od r\''{y}chlosti', 'Interpreter', 'latex');
set(title_handle, 'FontSize', 26);

% Nastavenie legendy
leg = legend('$e(\infty)$','Hranica po\v{z}iadavky');
set(leg, 'Location', 'NorthWest', 'Orientation', 'vertical', 'Interpreter', 'latex')


% Uloženie grafu
print('Sus_odchylka_linear', '-dpng');
