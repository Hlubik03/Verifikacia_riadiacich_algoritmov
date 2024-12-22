clc, clear, close all

%Zakladne parametre
M1 = 2500;
M2 = 320;
K1 = 80000;
K2 = 500000;
b1 = 350;
b2 = 15020;

%Prenosove funkcie 
s = tf('s');
G1 = ((M1+M2)*s^2+b2*s+K2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1)); %kontrolovana sila
G2 = (-M1*b2*s^3-M1*K2*s^2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1)); %vymol na ceste

%step(G1) %V autobuse to necitia ale dlho sa ustaluje
step(0.1*G2)% 10 cm vymol sposobi dlhu oscilaciu a aj nepohodlie
