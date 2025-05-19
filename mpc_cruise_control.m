clc; clear;

% --- Simulation Parameters ---
Ts = 0.1;
N = 300;
minDist = 6;

profile = 1;

% --- Lead vehicle velocity profile ---

if profile == 1
    t = (0:N-1)' * Ts;
    v_lead = 10 + 2 * sin(2 * pi * 0.1 * t);
elseif profile == 2
    v_lead = zeros(N,1);
    v_lead(1:20) = linspace(0, 10, 20);  % accelerate to 10 m/s
    v_lead(21:80) = 10;                  % maintain
    v_lead(81:100) = linspace(10, 0, 20); % decelerate
    v_lead(101:end) = 0;
end

x_lead = zeros(N,1);  % lead vehicle position
for k = 2:N
    x_lead(k) = x_lead(k-1) + Ts * v_lead(k-1);
end

% --- Ego vehicle model (double integrator) ---
A = [1 Ts; 0 1];
B = [0; Ts];
C = eye(2);  
D = 0;

plant = ss(A, B, C, D, Ts);
plant = minreal(plant);
mpcobj = mpc(plant, Ts, 10, 2);
mpcobj.Weights.OutputVariables = [.5 1];  % don't penalize velocity directly
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.MV.Min = -3;
mpcobj.MV.Max = 2;

% --- Initial Conditions ---
x_ego = zeros(2,1);         % [position; velocity]
x_lead0 = 6;               % Start lead vehicle 30 m ahead
x_lead = x_lead + x_lead0;  % Shift lead position forward

mpc_state = mpcstate(mpcobj);
ulog = zeros(N,1);
dist = zeros(N,1);
v_ego_log = zeros(N,1);

% --- Simulation Loop ---
for k = 1:N
    dist_k = x_lead(k) - x_ego(1);  % current distance to lead
    y = x_ego;                      % full state [position; velocity]

    % Reference: maintain safe following distance
    r = [x_lead(k) - minDist; v_lead(k)];

    % Apply MPC
    u = mpcmove(mpcobj, mpc_state, y, r);

    % Update ego state
    x_ego = A * x_ego + B * u;

    % Log
    ulog(k) = u;
    dist(k) = dist_k;
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
legend('Distance', 'Min Safe Distance');

subplot(3,1,2)
plot(t, ulog, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
title('Ego Vehicle Control Input'); grid on;

subplot(3,1,3)
plot(t, v_lead, 'k-', t, v_ego_log, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('Lead Vehicle', 'Ego Vehicle');
title('Vehicle Velocities'); grid on;
title('Vehicle Velocities'); grid on;

