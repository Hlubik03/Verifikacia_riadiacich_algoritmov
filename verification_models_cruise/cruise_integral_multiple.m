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

% Premmene na verifikaciu
pozadovane_rychlosti = 10:1:40;  
integrali = zeros(size(pozadovane_rychlosti));  
errory = {}; 
odozva = {}; 
casy = {}; 

for i = 1:length(pozadovane_rychlosti)
    r = pozadovane_rychlosti(i); 
    
    % Uzavretá slučka s PI regulátorom a systémom tempomatu
    T = feedback(C*P_cruise, 1); 
    
    % Simulácia odozvy systému
    [y, t_out] = step(r*T, t);  
    
    % Výpočet chyby a integrálu chyby
    e = r - y;  % Chyba medzi referenciou a skutočnou hodnotou
    integral_error = trapz(t_out, abs(e));  % Numerický integrál chyby pomocou metódy trapezoidov
    
    
    integrali(i) = integral_error;
    errory{i} = abs(e);
    odozva{i} = y;
    casy{i} = t_out;
end


[~, max_idx] = max(integrali);


t_max = casy{max_idx};
e_max = errory{max_idx};
r_max = pozadovane_rychlosti(max_idx);
y_max = odozva{max_idx};

figure;
hold on;

offset = abs(min(e_max));
e_posunuta = e_max + offset; 
area(t_max, e_posunuta, 'FaceColor', [1, 0.8, 0.8], 'EdgeColor', 'none', ...
     'DisplayName', 'Oblasť chyby');

plot(t_max, r_max * ones(size(t_max)), 'k--', 'LineWidth', 1, ...
     'DisplayName', 'Referenčná rýchlosť');

% Odozva systému
plot(t_max, y_max, 'b', 'LineWidth', 1.5, 'DisplayName', 'Odozva systému');

xlabel('Čas [s]');
ylabel('Rýchlosť/Chyba [m/s]');
title(sprintf('Presný tvar chyby pre rýchlosť %d m/s', r_max));
legend('show');
grid on;

% Výsledky
disp('Integrály riadiacej veličiny pre jednotlivé rýchlosti:');
for i = 1:length(pozadovane_rychlosti)
    fprintf('Rýchlosť %d m/s: Integrál = %.4f\n', pozadovane_rychlosti(i), integrali(i));
end

% Graf závislosti integrálu chyby od rýchlosti
figure;
plot(pozadovane_rychlosti, integrali, '-o', 'LineWidth', 1.5, 'MarkerSize', 6);
xlabel('Požadovaná rýchlosť [m/s]');
ylabel('Integrál chyby');
title('Závislosť veľkosti oblasti chyby od požadovanej rýchlosti');
grid on;
