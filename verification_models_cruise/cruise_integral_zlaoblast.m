
clc, clear, close all

% Zakladne parametre
m = 1000;  % hmotnosť vozidla v kg
b = 50;    % koeficient odporu (sila trenia/odporu)
u = 500;   % hodnota vstupného napätia (riadiaci signál)

% Časový interval simulácie
t = 0:0.1:20;  % časové kroky od 0 do 20 sekúnd s krokom 0.1 s

% Prevodová funkcia systému
s = tf('s');  % Laplaceova premenna
P_cruise = 1/(m*s + b);  % Prevodová funkcia pre tempomat, závisí na hmotnosti a odpore

% Hodnoty pre PI regulátor
Kp = 600;  
Ki = 20;

% Definícia PI regulátora
C = pid(Kp, Ki);  

% Priprava na simulaciu pre rôzne referenčné rýchlosti
velocities = 10:5:40;  % Rozsah rýchlostí od 10 do 40 m/s s krokom 5
integrals = zeros(size(velocities));  % Na uloženie integrálov riadiacej veličiny
errors = {};  % Na uloženie chýb pre každú rýchlosť
responses = {}; % Na uloženie odoziev pre každú rýchlosť
times = {}; % Na uloženie časov pre každú rýchlosť

figure;
hold on;

for i = 1:length(velocities)
    r = velocities(i);  % Referenčná rýchlosť
    
    % Uzavretá slučka s PI regulátorom a systémom tempomatu
    T = feedback(C*P_cruise, 1);  % Uzavretá regulačná slučka s odozvou systému
    
    % Simulácia odozvy systému
    [y, t_out] = step(r*T, t);  % Odozva systému na krokovú zmenu
    
    % Výpočet chyby a integrálu chyby
    e = r - y;  % Chyba medzi referenciou a skutočnou hodnotou
    integral_error = trapz(t_out, abs(e));  % Numerický integrál chyby pomocou metódy trapezoidov
    
    % Uloženie výsledkov
    integrals(i) = integral_error;
    errors{i} = abs(e);
    responses{i} = y;
    times{i} = t_out;
    
    % Zobrazenie odozvy systému pre každú rýchlosť
    plot(t_out, y, 'DisplayName', sprintf('Rýchlosť: %d m/s', r));
end

% Nájdeme index najväčšieho integrálu
[~, max_idx] = max(integrals);

% Vizualizácia plochy pre najväčší integrál
t_max = times{max_idx};
e_max = errors{max_idx};
r_max = velocities(max_idx);
y_max = responses{max_idx};

% Zvýraznenie plochy pod krivkou chyby
% Zvýraznenie plochy pod krivkou chyby so zachovaním tvaru
% Zvýraznenie plochy pod krivkou chyby so zachovaním tvaru (bez zrkadlenia)
% Zvýraznenie plochy pod krivkou chyby so zachovaním pôvodného tvaru
% Zvýraznenie plochy pod krivkou chyby bez zmeny tvaru
% Zvýraznenie plochy pod krivkou chyby bez zmeny tvaru
figure;
hold on;

% Posunutie chyby nahor o absolútnu hodnotu jej minimálnej hodnoty
offset = abs(min(e_max)); % Absolútna hodnota minimálnej chyby
e_shifted = e_max + offset; % Posunutie celého priebehu nahor

% Oblasť chyby presne podľa pôvodného tvaru
area(t_max, e_shifted, 'FaceColor', [0.8, 0.9, 1], 'EdgeColor', 'none', ...
     'DisplayName', 'Posunutá oblasť chyby');

% Pridanie referenčných čiar
plot(t_max, r_max * ones(size(t_max)), 'k--', 'LineWidth', 1, ...
     'DisplayName', 'Referenčná rýchlosť');
plot(t_max, y_max, 'b', 'LineWidth', 1.5, 'DisplayName', 'Odozva systému');

xlabel('Čas [s]');
ylabel('Rýchlosť/Chyba [m/s]');
title(sprintf('Chyba pre rýchlosť %d m/s (so zachovaným tvarom)', r_max));
legend('show');
grid on;











% Výsledky
disp('Integrály riadiacej veličiny pre jednotlivé rýchlosti:');
for i = 1:length(velocities)
    fprintf('Rýchlosť %d m/s: Integrál = %.4f', velocities(i), integrals(i));
end
