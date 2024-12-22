clc;
clear;
close all;

% Zakladne parametre
m = 1000;  % hmotnosť vozidla v kg
b = 50;    % koeficient odporu (sila trenia/odporu)
pozadovane_rychlosti = 10:5:40;  % požadované rýchlosti vozidla 

% Časový interval simulácie
t = 0:0.1:20;  

% Prevodová funkcia systému
s = tf('s');  
P_cruise = 1/(m * s + b);  

% Hodnoty pre PI regulátor
Kp = 600;  
Ki = 20;
C = pid(Kp, Ki);  

% Uzavretá slučka s PI regulátorom a systémom tempomatu
T = feedback(C * P_cruise, 1); 

% Generovanie grafu
figure;
hold on;
box on;
grid on;

% Kreslenie odozvy pre každú požadovanú rýchlosť
for r = pozadovane_rychlosti
    [y, t_out] = step(r * T, t);  % Simulácia odozvy systému
    p=plot(t_out, y,'DisplayName', ['R\''{y}chlos\v{t}: ', num2str(r), ' m/s']);
    set(p, 'LineWidth', 3)
end

% Nastavenie vlastností osí a popiskov s diakritikou
xlabel('\v{C}as [s]', 'Interpreter', 'latex', 'FontSize', 20);  % Pre os x
ylabel('R\''{y}chlos\v{t} [m/s]', 'Interpreter', 'latex', 'FontSize', 20);  % Pre os y

% Korektné zobrazenie nadpisu s LaTeX diakritikou
title('Odozvy syst\''{e}mu pre r\''{y}chlosti', ...
      'Interpreter', 'latex', 'FontSize', 30);

% Nastavenie legendy
legend('Interpreter', 'latex', 'FontSize', 10, 'Location', 'best', 'Orientation', 'vertical');

% Uloženie grafu
print('Odozvy_Rozne_Rychlosti', '-dpng');
