clc; clear; close all;

% Zakladne parametre systému
m1 = 2500;  % Hmotnosť karosérie vozidla (kg)
m2 = 320;   % Hmotnosť kolesa a zavesenia (kg)
k1 = 80000; % Tuhosť pružiny medzi karosériou a kolesom (N/m)
k2 = 500000; % Tuhosť pneumatiky (N/m)
b1 = 350;   % Tlmenie medzi karosériou a kolesom (Ns/m)
b2 = 15020; % Tlmenie pneumatiky (Ns/m)
p=0.1; %Porucha/vymol (m)

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
step(p * sys_cl, t);
title('Odozva na -0.1m výmol');  % Graf odozvy na výmol
