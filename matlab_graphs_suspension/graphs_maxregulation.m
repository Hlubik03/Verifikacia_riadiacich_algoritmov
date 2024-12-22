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

%Maximálne preregulovanie pre každý výmol
max_preregulovanie = zeros(size(vymoly));
% Simulácia pre každý výmol a hodnotenie maximálneho preregulovania
for i = 1:length(vymoly)
    [y, ~] = step(vymoly(i) * sys_cl, t);  % Odozva systému na výmol
    preregulovanie = (max(y) - 0)*100;  % Hodnota preregulovania od požadovanej hodnoty (0)
    max_preregulovanie(i) = preregulovanie;  
end

%maximálne preregulovanie a prislúchajúcu hodnotu výmolu
[max_preregul, idx] = max(max_preregulovanie);
najhorsie_vymol = vymoly(idx);

%% Generovanie grafov co chcem
%% inicializacia
%set(vlastnost) vypise svetko co mozeme nastavit
%% generovat data z simulinku

figure
hold on
box on
grid on

% Vytvorenie grafu
p = plot(vymoly, max_preregulovanie);
x = linspace(-0.1, 0.1, 100);
y = 0.5 * ones(size(x));
plot(x, y, 'r-', 'LineWidth', 2); 

% Nastavenie vlastností pre graf
set(p, 'LineWidth', 3)

% Nastavenie vlastností pre osi grafu
set(gca, 'FontSize', 20, 'TickLabelInterpreter', 'latex')

% Nastavenie popiskov osí
xlabel_handle = xlabel('Porucha [$m$]', 'Interpreter', 'latex');
ylabel_handle = ylabel('Preregulovanie [\%]', 'Interpreter', 'latex');


% Nastavenie veľkosti písma pre jednotlivé popisky
set(xlabel_handle, 'FontSize', 24)
set(ylabel_handle, 'FontSize', 24)

title_handle = title('Maxim\''{a}lne preregulovanie', 'Interpreter', 'latex');
set(title_handle, 'FontSize', 26)


% Nastavenie legendy
leg = legend('Odozva syst\''{e}mu', 'referencia');
set(leg, 'Location', 'North', 'Orientation', 'Horizontal', 'Interpreter', 'latex')

% Uloženie grafu
print('Preregulovanie', '-dpng')

