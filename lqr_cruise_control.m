clc; clear;

% --- Simulation Parameters ---
Ts = 0.1;
N = 300;
minDist = 6;

profile = 2;

% --- Lead vehicle velocity profile ---
if profile == 1
    t = (0:N-1)' * Ts;
    v_lead = 10 + 2 * sin(2 * pi * 0.1 * t);
elseif profile == 2
    v_lead = zeros(N,1);
    v_lead(1:20) = linspace(0, 10, 20);  % accelerate
    v_lead(21:60) = 10;
    v_lead(61:80) = linspace(10, 0, 20); % decelerate
    v_lead(81:end) = 0;
end

x_lead = zeros(N,1);
for k = 2:N
    x_lead(k) = x_lead(k-1) + Ts * v_lead(k-1);
end

% --- Ego Vehicle Model ---
A = [1 Ts; 0 1];
B = [0; Ts];
C = eye(2);  % position and velocity

% --- LQR Design ---
Q = diag([1, 1]);  % penalize position error more
R = 1;            % penalize control effort

K = dlqr(A, B, Q, R);  % LQR gain

% Feedforward gain to track position reference
Nbar = rscale(A, B, K);  % helper function below

% --- Initial Conditions ---
x_ego = zeros(2,1);
x_lead0 = 6;
x_lead = x_lead + x_lead0;

ulog = zeros(N,1);
dist = zeros(N,1);
v_ego_log = zeros(N,1);

% --- Simulation Loop ---
for k = 1:N
    ref_pos = x_lead(k) - minDist;

    u = -K * x_ego + Nbar * ref_pos;

    u = min(max(u, -3), 2);

    x_ego = A * x_ego + B * u;

    ulog(k) = u;
    dist(k) = x_lead(k) - x_ego(1);
    v_ego_log(k) = x_ego(2);
end

% --- Plotting ---
t = (0:N-1)*Ts;

figure;
subplot(3,1,1)
plot(t, dist, 'b-', 'LineWidth', 1.5); hold on;
yline(minDist, 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Distance (m)');
title('Distance Between Vehicles'); grid on;

subplot(3,1,2)
plot(t, ulog, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
title('Control Input'); grid on;

subplot(3,1,3)
plot(t, v_lead, 'k-', t, v_ego_log, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('Lead Vehicle', 'Ego Vehicle');
title('Velocities'); grid on;

