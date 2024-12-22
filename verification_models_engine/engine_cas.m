clc; clear; close all;

% Parametre motora a PID regulátora
R = 1;         % Odpor [Ohm]
L = 0.5;       % Indukčnosť [H]
k_e = 0.01;    % Konštanta spätného napätia [V·s/rad]
k_t = 0.01;    % Konštanta momentu [N·m/A]
J = 0.01;      % Moment zotrvačnosti [kg·m^2]
b = 0.1;       % Viskózny odpor [N·m·s/rad]
Kp = 2;        % P-zisk
Ki = 1;        % I-zisk
Kd = 0.05;     % D-zisk
% Počiatočné podmienky
initial_conditions = [0; 0; 0; 0]; % [i, omega, integral_error, prev_error]

% Časový interval simulácie
tspan = [0 80]; 

% Referenčné hodnoty uhlovej rýchlosti
omega_refs = 5:1:30;  % Rôzne referenčné rýchlosti [rad/s]

% Tolerancia pre reguláciu
tolerance = 0.05; % 5 % odchýlka od referenčnej hodnoty

% Uloženie časov regulácie
regulation_times = zeros(size(omega_refs));
max_regulation_time = 0;
best_omega_ref = omega_refs(1);

% Iterácia cez referenčné hodnoty
for j = 1:length(omega_refs)
    omega_ref = omega_refs(j); % Aktuálna referenčná hodnota
    
    % Simulácia systému
    [t, y] = ode45(@(t, y) motor_pid_system(t, y, R, L, k_e, k_t, J, b, Kp, Ki, Kd, omega_ref), tspan, initial_conditions);

    % Extrakcia uhlovej rýchlosti
    omega = y(:, 2);

    % Nájsť čas regulácie (keď sa systém ustáli v tolerancii)
    error = abs(omega - omega_ref);
    idx_within_tolerance = find(error <= tolerance, 1); % Prvý čas v tolerancii

    if ~isempty(idx_within_tolerance)
        regulation_times(j) = t(idx_within_tolerance);
    else
        regulation_times(j) = NaN; % Nenájdený čas regulácie
    end
    
    % Ak je tento čas regulácie najdlhší, uložíme ho
    if regulation_times(j) > max_regulation_time
        max_regulation_time = regulation_times(j);
        best_omega_ref = omega_ref;
        best_t = t;
        best_omega = omega;
    end
end

% Zobrazenie výsledkov - Graf 1: Čas regulácie pre rôzne referenčné hodnoty
figure;
plot(omega_refs, regulation_times, 'bo-', 'MarkerFaceColor', 'b', 'LineWidth', 1.5);
xlabel('Referenčná uhlová rýchlosť \omega_{ref} [rad/s]');
ylabel('Čas regulácie [s]');
title('Čas regulácie pre rôzne referenčné hodnoty');
grid on;

% Zobrazenie priebehu pre najdlhší čas regulácie - Graf 2
figure;
plot(best_t, best_omega, 'b', 'LineWidth', 2);
hold on;
yline(best_omega_ref, 'r--', 'LineWidth', 2);
xlabel('Čas [s]');
ylabel('Uhlová rýchlosť [rad/s]');
title(['Priebeh pre najdlhší čas regulácie pri \omega_{ref} = ' num2str(best_omega_ref) ' rad/s']);
legend('Simulovaná rýchlosť', 'Referenčná rýchlosť');
grid on;

% Funkcia definujúca dynamiku systému
function dydt = motor_pid_system(t, y, R, L, k_e, k_t, J, b, Kp, Ki, Kd, omega_ref)
    % Stavové premenné
    i = y(1);              % Prúd motora
    omega = y(2);          % Uhlová rýchlosť
    integral_error = y(3); % Integrál regulačnej odchýlky
    prev_error = y(4);     % Predchádzajúca odchýlka

    % Regulačná odchýlka
    error = omega_ref - omega;

    % PID regulátor
    dt = max(t - 1e-6, 1e-6); % Ochrana proti deleniu nulou
    derivative_error = (error - prev_error) / dt;
    V = Kp * error + Ki * integral_error + Kd * derivative_error;

    % Elektrická časť motora
    di_dt = (1 / L) * (V - R * i - k_e * omega);

    % Mechanická časť motora
    domega_dt = (1 / J) * (k_t * i - b * omega);

    % Aktualizácia stavových premenných
    dintegral_error_dt = error; % Integrácia regulačnej odchýlky
    dprev_error_dt = error;     % Uloženie aktuálnej odchýlky

    % Návrat diferenciálnych rovníc
    dydt = [di_dt; domega_dt; dintegral_error_dt; dprev_error_dt];
end
