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

% Rôzne referenčné uhlové rýchlosti
omega_refs = 5:1:30; % [rad/s]

% Časový interval simulácie
tspan = [0 80]; 

% Príprava na výsledky
final_errors = zeros(size(omega_refs));

for idx = 1:length(omega_refs)
    omega_ref = omega_refs(idx);
    % Počiatočné podmienky
    initial_conditions = [0; 0; 0; 0]; % [i, omega, integral_error, prev_error]
    
    % Simulácia systému
    [t, y] = ode45(@(t, y) motor_pid_system(t, y, R, L, k_e, k_t, J, b, Kp, Ki, Kd, omega_ref), tspan, initial_conditions);
    
    % Výsledky
    omega = y(:, 2); % Uhlová rýchlosť
    
    % Regulačná odchýlka na konci simulácie
    final_error = omega_ref - omega(end);
    final_errors(idx) = final_error;
end

% Graf závislosti regulačnej odchýlky od referenčnej rýchlosti
figure;
plot(omega_refs, final_errors, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('Referenčná uhlová rýchlosť \omega_{ref} [rad/s]');
ylabel('Trvalá regulačná odchýlka [rad/s]');
title('Závislosť regulačnej odchýlky od referenčnej rýchlosti');
grid on;

% Zhrnutie výsledkov
fprintf('Výsledky regulačných odchýlok na konci simulácie:\n');
for idx = 1:length(omega_refs)
    fprintf('Pre omega_ref = %.2f rad/s: Trvalá odchýlka = %.4f rad/s\n', omega_refs(idx), final_errors(idx));
end

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
