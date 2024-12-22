%% skript na hodnoty copy paste
clc, clear all, close all

%% Parametre systému
m1 = 2500;  % Hmotnosť karosérie vozidla (kg)
m2 = 320;   % Hmotnosť kolesa a zavesenia (kg)
k1 = 80000; % Tuhosť pružiny medzi karosériou a kolesom (N/m)
k2 = 500000; % Tuhosť pneumatiky (N/m)
b1 = 350;   % Tlmenie medzi karosériou a kolesom (Ns/m)
b2 = 15020; % Tlmenie pneumatiky (Ns/m)

% Prenosová funkcia pre pohyb karosérie (G1)
n1 = [(m1 + m2) b2 k2];  
d1 = [(m1 * m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2];
G1 = tf(n1, d1);

% Prenosová funkcia pre pohyb kolesa (G2)
n2 = [-(m1*b2) -(m1*k2) 0 0]; 
d2 = [(m1*m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2];
G2 = tf(n2, d2);

% Prenosová funkcia pre silu
sila = tf(n2, n1);

% Parametre PID regulátora
Kd = 208025;  
Kp = 832100; 
Ki = 624075; 
C = pid(Kp, Ki, Kd); 

% Uzavretý systém so spätnou väzbou
sys_cl = sila * feedback(G1, C);

% Čas simulácie
t = 0:0.05:5;  

%% Verifikacia
p_hodnoty = -1:0.01:1;  
xi_hodnoty = zeros(size(p_hodnoty)); 
max_xi = -inf;                        
najlepsie_p = 0;                      

for i = 1:length(p_hodnoty)
    p = p_hodnoty(i);
    
    
    [odozva, cas] = step(p * sys_cl, t);
    
   
    y0 = odozva(1);                       
    
    % Detekcia prvého maxima  a  minima 
    [maxima, indexy_max] = findpeaks(odozva);         
    [minima, indexy_min] = findpeaks(-odozva);        
    
    % Kontrola
    if isempty(indexy_max) || isempty(indexy_min)
        xi_hodnoty(i) = NaN;  
        continue;             
    end
    
    % Určenie poradia prvého maxima a minima
    if indexy_max(1) < indexy_min(1)
        y1 = odozva(indexy_max(1));      % Prvé maximum (hore)
        y2 = odozva(indexy_min(1));      % Prvé minimum (dole)
    else
        y1 = odozva(indexy_min(1));      % Prvé minimum (dole)
        y2 = odozva(indexy_max(1));      % Prvé maximum (hore)
    end
    
    % Výpočet M a xi
    M = (y1 - y2) / (y1 - y0);
    xi = abs((log(M) / sqrt(pi^2 + (log(M))^2)));
    xi_hodnoty(i) = xi; 
    
    
    if xi > max_xi
        max_xi = xi;
        najlepsie_p = p;
    end
end

%% Výsledky
disp(['Najväčší tlmiaci pomer (ξ) je: ', num2str(max_xi)]);
disp(['Tento ξ bol dosiahnutý pri vymole p = ', num2str(najlepsie_p)]);


disp('Všetky hodnoty ξ pre jednotlivé vymoly (p hodnoty):');
for i = 1:length(p_hodnoty)
    fprintf('p = %.2f, ξ = %.4f\n', p_hodnoty(i), xi_hodnoty(i));
end

[odozva_najlepsie, cas_najlepsie] = step(najlepsie_p * sys_cl, t);

figure;
p(1)=plot(cas_najlepsie, odozva_najlepsie);
hold on;


[maxima_najlepsie, indexy_max_najlepsie] = findpeaks(odozva_najlepsie);
[minima_najlepsie, indexy_min_najlepsie] = findpeaks(-odozva_najlepsie);

if ~isempty(indexy_max_najlepsie)
    p(3)=plot(cas_najlepsie(indexy_max_najlepsie(1)), odozva_najlepsie(indexy_max_najlepsie(1)), 'ro', 'MarkerSize', 8, 'DisplayName', 'y1 (Prvé maximum)');
end
if ~isempty(indexy_min_najlepsie)
    p(4)=plot(cas_najlepsie(indexy_min_najlepsie(1)), odozva_najlepsie(indexy_min_najlepsie(1)), 'go', 'MarkerSize', 8, 'DisplayName', 'y2 (Prvé minimum)');
end
p(2)=plot(cas_najlepsie(1), odozva_najlepsie(1), 'bo', 'MarkerSize', 8, 'DisplayName', 'y0 (Počiatočná hodnota)');

legend;
title(['Odozva na výmol pri p = ', num2str(najlepsie_p), ' s maximálnym ξ']);
xlabel('Čas (s)');
ylabel('Odozva systému');
grid on;

%% Generovanie grafov co chcem
%% inicializacia
%set(vlastnost) vypise svetko co mozeme nastavit
%% generovat data z simulinku

% Vytvorenie grafu
%p = plot(vymoly, max_preregulovanie);
%x = linspace(-0.1, 0.1, 100);
%y = 0.5 * ones(size(x));
%plot(x, y, 'r-', 'LineWidth', 2); 

% Nastavenie vlastností pre graf
set(p, 'LineWidth', 3)

% Nastavenie vlastností pre osi grafu
set(gca, 'FontSize', 20, 'TickLabelInterpreter', 'latex')

% Nastavenie popiskov osí
xlabel_handle = xlabel('\v{C}as [$s$]', 'Interpreter', 'latex');
ylabel_handle = ylabel('Pohyb karos\''{e}rie [m]', 'Interpreter', 'latex');


% Nastavenie veľkosti písma pre jednotlivé popisky
set(xlabel_handle, 'FontSize', 24)
set(ylabel_handle, 'FontSize', 24)

title_handle = title('Hodnoty potrebn\''{e} na vypo\v{c}\''{i}tanie tlmenia', 'Interpreter', 'latex');
set(title_handle, 'FontSize', 26)


% Nastavenie legendy
leg = legend('Odozva syst\''{e}mu', 'y1=Prv\''{e} maximum','y2=Prv\''{e} minimum','y0=Po\v{c}iato\v{c}n\''{a} hodnota');
set(leg, 'Location', 'NorthEast', 'Orientation', 'vertical', 'Interpreter', 'latex')

% Uloženie grafu
print('Kmity', '-dpng')

