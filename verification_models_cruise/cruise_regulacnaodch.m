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
P_cruise = 1/(m * s + b);  %Tempomat

% Hodnoty pre PI regulátor
Kp = 600;  
Ki = 20;
C = pid(Kp, Ki);  

% Uzavretá slučka s PI regulátorom a systémom tempomatu
uzavreta_slucka = feedback(C * P_cruise, 1);  


trvala_odchylka = zeros(size(pozadovane_rychlosti));  

figure;
hold on;
title('Odozva systému pre rôzne požiadavky na rýchlosť');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');

for i = 1:length(pozadovane_rychlosti)
    r = pozadovane_rychlosti(i);  
    [odozva, cas_odozvy] = step(r * uzavreta_slucka, t); 

   
    trvala_odchylka(i) = r - odozva(end);  

    
    plot(cas_odozvy, odozva, 'DisplayName', ['Požiadavka = ' num2str(r) ' m/s']);
end

legend show;
grid on;


disp('Trvalá regulačná odchylka pre rôzne rýchlosti:');
for i = 1:length(pozadovane_rychlosti)
    fprintf('Požadovaná rýchlosť = %d m/s, Trvalá regulačná odchylka = %.4f m/s\n', pozadovane_rychlosti(i), trvala_odchylka(i));
end

%Najväčšia regulačna odchylka
[max_odchylka, max_index] = max(abs(trvala_odchylka));  
najvacsia_odchylka = pozadovane_rychlosti(max_index);   

% Odozva pre najväčšiu trvalú regulačnú odchylku
figure;
r = najvacsia_odchylka;
[odozva, cas_odozvy] = step(r * uzavreta_slucka, t);
plot(cas_odozvy, odozva, 'LineWidth', 2);
title(['Najväčšia trvala regulačna odchylka pri rychlosti: ' num2str(r) ' m/s']);
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;

% Graf zmeny regulačnej odchylky v závislosti od požadovanej rýchlosti
figure;
plot(pozadovane_rychlosti, abs(trvala_odchylka), '-o', 'LineWidth', 2);
title('Závislosť trvalej regulačnej odchylky od požadovanej rýchlosti');
xlabel('Požadovaná rýchlosť (m/s)');
ylabel('Trvalá regulačná odchylka (m/s)');
grid on;
