clc; clear; close all;

%% System Parameters
m1 = 2500;  % Hmotnosť karosérie vozidla (kg)
m2 = 320;   % Hmotnosť kolesa a zavesenia (kg)
k1 = 80000; % Tuhosť pružiny medzi karosériou a kolesom (N/m)
k2 = 500000; % Tuhosť pneumatiky (N/m)
b1 = 350;   % Tlmenie medzi karosériou a kolesom (Ns/m)
b2 = 15020; % Tlmenie pneumatiky (Ns/m)
p = 1;    % Porucha/vymol (m)

% Transfer Function for Body Movement (G1)
n1 = [(m1 + m2) b2 k2];  
d1 = [(m1 * m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2];
G1 = tf(n1, d1);

% Transfer Function for Wheel Movement (G2)
n2 = [-(m1*b2) -(m1*k2) 0 0]; 
d2 = [(m1*m2) (m1*(b1+b2)) + (m2*b1) (m1*(k1+k2)) + (m2*k1) + (b1*b2) (b1*k2) + (b2*k1) k1*k2];
G2 = tf(n2, d2);

% Transfer Function for Force
sila = tf(n2, n1);

% PID Controller Parameters
Kd = 208025;  
Kp = 832100; 
Ki = 624075; 
C = pid(Kp, Ki, Kd); 

% Closed-Loop System with Feedback
sys_cl = sila * feedback(G1, C);

% Simulation Time
t = 0:0.05:5;  

% Simulate Step Response
[response, time] = step(p * sys_cl, t);

%% Finding y0, y1, and y2
y0 = response(1);                       % Initial response value

% Detect first peak (upward) for y1
[~, peakIndices] = findpeaks(response); % Find upward peaks
if ~isempty(peakIndices)
    y1 = response(peakIndices(1));      % First upward peak
else
    error('No upward peak found to determine y1.');
end

% Detect first downward peak for y2
[~, troughIndices] = findpeaks(-response); % Find downward peaks by inverting response
if ~isempty(troughIndices)
    y2 = response(troughIndices(1));    % First downward peak (minimum)
else
    error('No downward peak found to determine y2.');
end

%% Calculate M and Damping Ratio (xi)
M = (y1 - y2) / (y1 - y0);
xi = abs(log(M) / sqrt(pi^2 + (log(M))^2));

%% Display Results
disp(['Initial Value (y0): ', num2str(y0)]);
disp(['First Peak (y1): ', num2str(y1)]);
disp(['First Trough (y2): ', num2str(y2)]);
disp(['Overshoot Ratio (M): ', num2str(M)]);
disp(['Damping Ratio (ξ): ', num2str(xi)]);

%% Plot the Response with Markers for Verification
figure;
plot(time, response);
hold on;
plot(time(peakIndices(1)), y1, 'ro', 'MarkerSize', 8, 'DisplayName', 'y1 (First Peak)');
plot(time(troughIndices(1)), y2, 'go', 'MarkerSize', 8, 'DisplayName', 'y2 (First Trough)');
plot(time(1), y0, 'bo', 'MarkerSize', 8, 'DisplayName', 'y0 (Initial Value)');
legend;
title('Odozva na -0.1m výmol s označením y0, y1, y2');
xlabel('Čas (s)');
ylabel('Odozva systému');
grid on;
