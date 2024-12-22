%% skript na hodnoty copy paste
clc, clear all, close all
%% System
% Zakladne parametre systému
m1 = 2500;  % Hmotnosť karosérie vozidla (kg)
m2 = 320;   % Hmotnosť kolesa a zavesenia (kg)
k1 = 80000; % Tuhosť pružiny medzi karosériou a kolesom (N/m)
k2 = 500000; % Tuhosť pneumatiky (N/m)
b1 = 350;   % Tlmenie medzi karosériou a kolesom (Ns/m)
b2 = 15020; % Tlmenie pneumatiky (Ns/m)

% Prenosové funkcie systému odpruženia
n1 = [(m1 + m2) b2 k2]; 
d1 = [(m1 * m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2]; 
G1 = tf(n1, d1);  % pohyb karosérie

n2 = [-(m1*b2) -(m1*k2) 0 0]; 
G2 = tf(n2, d1);  % pohyb kolesa

% Prenosová funkcia pre silu
sila = tf(n2, n1);

% PID parametre
Kd = 208025;  
Kp = 832100; 
Ki = 624075;  

C = pid(Kp, Ki, Kd);  

% Odsimulovanie uzavretého systému so spätnou väzbou
sys_cl = sila * feedback(G1, C);

% Časová os simulácie
t = 0:0.05:5;

%% Verifikacia
% Vstupné poruchy (výmoly) od -0.1 do +0.1 s krokom 0.001
vymoly = -0.1:0.001:0.1;

% Nastavenie tolerancie pre reguláciu
tolerance = 0.0005;

% čas pre každý vymol
cas_regulacie_vsetky = zeros(size(vymoly));

% Maximálny čas regulácie
max_cas_regulacie = 0;
najhorsi_vymol = 0;

% Pre každý vymol čas regulácie
for i = 1:length(vymoly)
    % Definovanie vstupnej poruchy ako skok s aktuálnou hodnotou vymolu
    [y, t] = step(vymoly(i) * sys_cl, t);
    
    % Posledný čas, kedy je odozva mimo tolerančného intervalu
    index_regulacie = find(abs(y) > tolerance, 1, 'last');
    
    % Skontroluj
    if ~isempty(index_regulacie)
        cas_regulacie = t(index_regulacie);
        cas_regulacie_vsetky(i) = cas_regulacie;
        
        % Aktualizácia najdlhšieho času 
        if cas_regulacie > max_cas_regulacie
            max_cas_regulacie = cas_regulacie;
            najhorsi_vymol = vymoly(i);
        end
    else
        cas_regulacie_vsetky(i) = NaN; % Ak sa systém ustálil okamžite, nastavíme NaN
    end
end



%% Generovanie grafov co chcem
%% inicializacia
%set(vlastnost) vypise svetko co mozeme nastavit
%% generovat data z simulinku

figure
hold on
box on
grid on

% Vytvorenie grafu
[y_najhorsi, t] = step(najhorsi_vymol * sys_cl, t);
p = plot(t, y_najhorsi);
yline(tolerance, 'r--', 'Tolerancia +-0.0005');
yline(-tolerance, 'r--');

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

title_handle = title('Odozva syst\''{e}mu s najdlh\v{s}\''{i}m \v{c}asom regul\''{a}cie', 'Interpreter', 'latex');
set(title_handle, 'FontSize', 26)


% Nastavenie legendy
leg = legend('Odozva syst\''{e}mu', 'Tolerancia');
set(leg, 'Location', 'North', 'Orientation', 'Horizontal', 'Interpreter', 'latex')

% Uloženie grafu
print('Cas_regulaice_najhorsi', '-dpng')
