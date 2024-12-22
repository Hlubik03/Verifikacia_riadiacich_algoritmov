clc, clear, close all  

% Zakladne parametre
m = 1000;  % hmotnosť vozidla v kg
b = 50;    % koeficient odporu (sila trenia/odporu)
u = 500;   % hodnota vstupného napätia (riadiaci signál)
r = 10;    % požadovaná rýchlosť vozidla (referenčná hodnota)

% Časový interval simulácie
t = 0:0.1:20;  % časové kroky od 0 do 20 sekúnd s krokom 0.1 s

% Prevodová funkcia systému
s = tf('s');  % Laplaceova premenna
P_cruise = 1/(m*s + b);  % Prevodová funkcia pre tempomat, závisí na hmotnosti a odpore

% Hodnoty pre PI regulátor
Kp = 600;  
Ki=20

% Definícia PI regulátora
C = pid(Kp,Ki);  

% Uzavretá slučka s PI regulátorom a systémom tempomatu
T = feedback(C*P_cruise, 1);  % Uzavretá regulačná slučka s odozvou systému

% Odozva systému na krokovú zmenu požiadavky (rýchlosti) v čase t
step(r*T, t); 

