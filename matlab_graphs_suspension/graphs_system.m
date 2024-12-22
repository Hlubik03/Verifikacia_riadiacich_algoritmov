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
v=-0.1; %Porucha/vymol (m)

% Prenosové funkcie systému odpruženia
% Numerátor a menovateľ prenosovej funkcie G1, opisuje pohyb karosérie
n1 = [(m1 + m2) b2 k2];  
d1 = [(m1 * m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2]; 

G1 = tf(n1, d1);  % Prenosová funkcia G1 (pohyb karosérie)

% Numerátor a menovateľ prenosovej funkcie G2, opisuje pohyb kolesa
n2 = [-(m1*b2) -(m1*k2) 0 0]; 
d2 = [(m1*m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2];

G2 = tf(n2, d2);  % Prenosová funkcia G2 (pohyb kolesa)

% Prenosová funkcia pre silu
sila = tf(n2, n1);

% PID parametre 
Kd = 208025;  
Kp = 832100; 
Ki = 624075; 

C = pid(Kp, Ki, Kd); 

% Odsimulovanie systému so spätnou väzbou
sys_cl = sila * feedback(G1, C);  

% Čas simulacie
t = 0:0.05:5;  

% Vykreslenie odozvy na vstupnú zmenu (schodový vstup) vo výške 0.1m

%title('Odozva na -0.1m výmol');  % Graf odozvy na výmol


%% Generovanie grafov co chcem
%% inicializacia
%set(vlastnost) vypise svetko co mozeme nastavit
%% generovat data z simulinku

figure
hold on
box on
grid on

% Vytvorenie grafu

[y,w] = step(v * sys_cl, t);
p = plot(t, y);

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

title_handle = title('Odozva na -0.1m v\''{y}mol', 'Interpreter', 'latex');
set(title_handle, 'FontSize', 26)


% Nastavenie legendy
leg = legend('Odozva syst\''{e}mu', 'Referencia');
set(leg, 'Location', 'North', 'Orientation', 'Horizontal', 'Interpreter', 'latex')

% Uloženie grafu
print('System', '-dpng')

