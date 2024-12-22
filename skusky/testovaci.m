clc, clear, close all

% Zakladne parametre
m = 1000;  % hmotnosť vozidla v kg
b = 50;    % koeficient odporu (sila trenia/odporu)
pozadovane_rychlosti = 10:1:40;  % požadované rýchlosti vozidla 

% Časový interval simulácie
t = 0:0.1:20;  

% Prevodová funkcia systému
s = tf('s');  
P_cruise = 1/(m * s + b);  

% Hodnoty pre PI regulátor
Kp = 600;  
Ki = 20;
C = pid(Kp, Ki);  

% Výpočet pre najväčšiu rýchlosť
r = max(pozadovane_rychlosti);  % Najväčšia požadovaná rýchlosť

% Uzavretá slučka s PI regulátorom a systémom tempomatu
T = feedback(C * P_cruise, 1); 

% Simulácia odozvy systému
[y, t_out] = step(r * T, t);  

% Výpočet chyby
e = r - y;  % Chyba medzi referenciou a skutočnou hodnotou

%Grafu
figure;
hold on;
box on;
grid on;


area_handle = area(t_out, abs(e), 'FaceAlpha', 0.5, 'DisplayName', 'Chyba');
set(area_handle, 'FaceColor', [1, 0.8, 0.8], 'LineStyle', 'none');

p1 = plot(t_out, y, 'DisplayName', 'Odozva systému');
set(p1, 'LineWidth', 3, 'Color', 'b');


p2 = plot(t_out, r * ones(size(t_out)), '--', 'DisplayName', 'Referenčná rýchlosť');
set(p2, 'LineWidth', 2, 'Color', 'k');

total_error = trapz(t_out, abs(e));  % Integrál chyby cez čas
disp(['Celková kumulatívna chyba: ', num2str(total_error)]);


xlabel_handle = xlabel('\v{C}as [s]', 'Interpreter', 'latex');  
ylabel_handle = ylabel('R\''{y}chlos\v{t} a chyba [m/s]', 'Interpreter', 'latex');  

% Korektné zobrazenie nadpisu s LaTeX diakritikou
title_handle = title(['Odozva syst\''{e}mu a vyzna\v{c}en\''{a} chyba'], ...
                     'Interpreter', 'latex', 'FontSize', 30);


% Nastavenie legendy
legend({'Chyba', 'Odozva syst\''{e}mu', 'Referen\v{c}n\''{a} r\''{y}chlos\v{t}'}, ...
       'Interpreter', 'latex', 'FontSize', 16, 'Location', 'NorthEast', 'Orientation', 'vertical');

% Korekcia vzhľadu textu na osi
set([xlabel_handle, ylabel_handle, title_handle], 'FontSize', 20);

% Uloženie grafu
print('Odozva_Chyba_Najvacsia_Rychlost', '-dpng');