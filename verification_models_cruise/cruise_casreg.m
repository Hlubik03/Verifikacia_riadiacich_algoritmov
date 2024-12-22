close all;

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

% Graf odozvy pre každú požadovanú rýchlosť
figure;
hold on;
title('Odozva systému pre rôzne požadované rýchlosti');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');

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

    % Plot odozvy pre danú rýchlosť
    plot(cas_odozvy, odozva, 'DisplayName', sprintf('Požadovaná rýchlosť = %d m/s', r));
    
    % čas regulácie na grafe
    if ~isnan(cas_regulacie(i))
        plot(cas_regulacie(i), odozva(index), 'rx');  
    end
    
    % Záznam najhoršej (najdlhšej) odozvy
    if cas_regulacie(i) > najdlhsie_cas_regulacie
        najdlhsie_cas_regulacie = cas_regulacie(i);
        index_najhorsia_rychlost = i;
    end
end

hold off;

% Graf regulácie pre jednotlivé rýchlosti
figure;
bar(pozadovane_rychlosti, cas_regulacie);
title('Čas regulácie pre jednotlivé požadované rýchlosti');
xlabel('Rýchlosť (m/s)');
ylabel('Čas regulácie (s)');
grid on;

% Graf najhoršej odozvy 
figure;
r_najhorsie = pozadovane_rychlosti(index_najhorsia_rychlost);  
[odozva_najhorsie, cas_odozvy_najhorsie] = step(r_najhorsie * uzavreta_slucka, t);

plot(cas_odozvy_najhorsie, odozva_najhorsie);
title(sprintf('Odozva pre najhoršiu reguláciu: Požadovaná rýchlosť = %d m/s', r_najhorsie));
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;

% Označenie času regulácie
index_najhorsie = find(cas_odozvy_najhorsie >= najdlhsie_cas_regulacie, 1, 'first');
hold on;
plot(najdlhsie_cas_regulacie, odozva_najhorsie(index_najhorsie), 'rx', 'MarkerSize', 8);
hold off;
