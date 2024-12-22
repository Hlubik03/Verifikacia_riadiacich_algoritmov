clc;
clear;
close all;

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

% Premenne na verifikaciu
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

% Výsledky
disp('Integrály riadiacej veličiny pre jednotlivé rýchlosti:');
for i = 1:length(pozadovane_rychlosti)
    fprintf('Rýchlosť %d m/s: Integrál = %.4f\n', pozadovane_rychlosti(i), integrali(i));
end

% Graf závislosti integrálu chyby od rýchlosti
figure;
hold on;
p1 = plot(pozadovane_rychlosti, integrali, '-', 'LineWidth', 1.5, 'MarkerSize', 6);
set(p1, 'LineWidth', 3);

% Pridanie referencie vo výške 100
yline(100, '--r', 'LineWidth', 2, ...
      'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'middle');


% Popisky osí
xlabel_handle = xlabel('Po\v{z}adovan\''{a} r\''{y}chlos\v{t} [m/s]', 'Interpreter', 'latex');
ylabel_handle = ylabel('Integr\''{a}l chyby', 'Interpreter', 'latex');

% Názov grafu
title_handle = title(['Z\''{a}vislos\v{t} oblasti chyby od po\v{z}adovanej r\''{y}chlosti'], ...
                     'Interpreter', 'latex');

% Nastavenie písma
set([xlabel_handle, ylabel_handle, title_handle], 'FontSize', 20);

% Nastavenie legendy
legend({'Odozva syst\''{e}mu', 'Hranica po\v{z}iadavky'}, ...
       'Interpreter', 'latex', 'FontSize', 16, 'Location', 'NorthWest', 'Orientation', 'vertical');

% Aktivovanie mriežky
grid on;

% Uloženie grafu
print('Integral_multiple_zavislost', '-dpng');