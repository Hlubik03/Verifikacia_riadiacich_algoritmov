clc;
clear all;
close all;

%% System

% Zakladne parametre
m = 1000;  % hmotnosť vozidla v kg
b = 50;    % koeficient odporu (sila trenia/odporu)
pozadovane_rychlosti = 10:1:40;  % požadované rýchlosti vozidla 

% Časový interval simulácie
t = 0:0.1:20;  

% Prevodová funkcia systému
s = tf('s');  
P_cruise = 1/(m * s + b);  % Tempomat

% Hodnoty pre PI regulátor
Kp = 600;  
Ki = 1;
C = pid(Kp, Ki);  

% Uzavretá slučka s PI regulátorom a systémom tempomatu
uzavreta_slucka = feedback(C * P_cruise, 1);  

% Tolerancia pre stabilitu
tolerancia = 0.01;  % povolená zmena hodnoty odozvy pre dosiahnutie stability

% Výpočet času regulácie pre jednotlivé rýchlosti
cas_regulacie = zeros(size(pozadovane_rychlosti));
najdlhsie_cas_regulacie = 0;
index_najhorsia_rychlost = 0;

%% Verifikacia

for i = 1:length(pozadovane_rychlosti)
    r = pozadovane_rychlosti(i);  
    [odozva, cas_odozvy] = step(r * uzavreta_slucka, t);  % Odozva systému 

    % Rozdiely medzi po sebe idúcimi hodnotami odozvy
    rozdiely_odozvy = abs(diff(odozva));
    
    % Prvý index, kde odozva vstúpi a zostane v tolerančnom pásme
    index = find(rozdiely_odozvy < tolerancia, 1, 'first');
    
    % Overenie, či odozva zostáva v rámci malých rozdielov až do konca simulácie
    if ~isempty(index) && all(rozdiely_odozvy(index:end) < tolerancia)
        cas_regulacie(i) = cas_odozvy(index); 
    end

    % Záznam najhoršej (najdlhšej) odozvy
    if cas_regulacie(i) > najdlhsie_cas_regulacie
        najdlhsie_cas_regulacie = cas_regulacie(i);
        index_najhorsia_rychlost = i;
    end
end

%% Generovanie grafov
figure;
hold on;
box on;
grid on;

% Vytvorenie grafu
bar(pozadovane_rychlosti, cas_regulacie);

% Pridanie červenej čiary na výšku 5 sekúnd
yline(7, '--r', 'LineWidth', 5, 'LabelHorizontalAlignment', 'left');

% Nastavenie vlastností pre osi grafu
set(gca, 'FontSize', 20, 'TickLabelInterpreter', 'latex');

% Nastavenie popiskov osí
xlabel_handle = xlabel('R\''{y}chlos\v{t} [$m/s$]', 'Interpreter', 'latex');
ylabel_handle = ylabel('\v{C}as regul\''{a}cie [s]', 'Interpreter', 'latex');

% Nastavenie legendy
leg = legend('Odozva syst\''{e}mu','Hranica po\v{z}iadavky');
set(leg, 'Location', 'NorthWest', 'Orientation', 'vertical', 'Interpreter', 'latex')



% Nastavenie veľkosti písma pre jednotlivé popisky
set(xlabel_handle, 'FontSize', 24);
set(ylabel_handle, 'FontSize', 24);

title_handle = title('\v{C}as regul\''{a}cie pre jednotliv\''{e} r\''{y}chlos\v{t}i', 'Interpreter', 'latex');
set(title_handle, 'FontSize', 26);

% Uloženie grafu
print('Sus_cas_bars', '-dpng');
