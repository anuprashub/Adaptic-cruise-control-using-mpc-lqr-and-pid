% --- Parameters ---
Cr = 0.01; g = 9.81; Ts = 0.1;
sim_time = 40; N_sim = sim_time / Ts;
Np = 10; % prediction horizon

% --- System Dynamics ---
Ad = [1 Ts 0; 0 1 0; 0 0 1];
Bd = [0.5*Ts^2; Ts; Ts];
Cd = eye(3); Dd = zeros(3,1); % full state output

sys = ss(Ad, Bd, Cd, Dd, Ts);

% --- Define MPC Controller ---
mpc_obj = mpc(sys, Ts, Np, 3);  % prediction, control horizons

% Weights: prioritize d_rel, then v_rel, then v_ego
mpc_obj.Weights.ManipulatedVariables = 0.1;
mpc_obj.Weights.ManipulatedVariablesRate = 0.1;
mpc_obj.Weights.OutputVariables = [10 5 1];

% Input constraints
mpc_obj.MV.Min = -3;
mpc_obj.MV.Max = 2;

% Optional: State constraints (soft constraints via penalty)
% You can't directly add constraints on states like d_rel ∈ [6, 10],
% but you can model it as output and tune weights accordingly,
% or use custom constraints (below).

% --- Initial Conditions ---
% --- Initial Conditions ---
x0 = [5; 5; 20];  % [d_rel; v_rel; v_ego]
x = x0;
X_log = x0; U_log = []; t_log = 0;

a_lead = 0.5 * ones(1, N_sim);
v_lead = zeros(1, N_sim); v_lead(1) = x0(3) + x0(2);

% --- Initialize MPC internal state object ---
mpc_state = mpcstate(mpc_obj);

% --- MPC Simulation Loop ---
for k = 1:N_sim-1
    % Define measured disturbance (lead acceleration effect)
    d_step = [0; a_lead(k)*Ts - Cr*g*Ts; -Cr*g*Ts];
    
    % Compute optimal control input using mpcmove with state object
    u = mpcmove(mpc_obj, mpc_state, x, x, []);
    
    % Update system
    x = Ad*x + Bd*u + d_step;
    
    % Log data
    X_log(:,end+1) = x;
    U_log(end+1) = u;
    t_log(end+1) = k*Ts;
    v_lead(k+1) = v_lead(k) + a_lead(k)*Ts;
end


% --- Plotting ---
d_rel = X_log(1,:)
v_rel = X_log(2,:);
v_ego = X_log(3,:);

figure;
subplot(3,1,1);
plot(t_log, d_rel, 'b', 'LineWidth', 2); hold on;
yline(6, 'r--'); yline(10, 'r--');
ylabel('Distance to Lead (m)');
title('MPC Cruise Control (Toolbox)');
grid on;

subplot(3,1,2);
plot(t_log, v_ego, 'g', 'LineWidth', 2); hold on;
plot(t_log, v_lead(1:length(t_log)), 'm--', 'LineWidth', 2);
ylabel('Velocities (m/s)');
legend('Ego', 'Lead');
grid on;

subplot(3,1,3);
plot(t_log(1:end-1), U_log, 'k', 'LineWidth', 2);
ylabel('Control Input (m/s^2)');
xlabel('Time (s)');
grid on;

