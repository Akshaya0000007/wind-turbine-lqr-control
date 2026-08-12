% Wind Turbine Parameters
J = 10000;   % Inertia of the turbine (kg.m^2)
B = 100;     % Damping coefficient (N.m.s)
Kp = 0.01;   % Power coefficient
rho = 1.225; % Air density (kg/m^3)
A_blades = 50; % Swept area of the blades (m^2)

% Control and external parameters
desired_speed = 12;  % Desired rotor speed in rad/s
T_wind_nominal = 8;  % Nominal wind torque (N.m)

% State-space model (SISO system)
A_sys = -B/J;       % System matrix
B_sys = 1/J;        % Input matrix (pitch angle adjustment)
C_sys = 1;          % Output matrix (rotor speed)
D_sys = 0;          % Feedthrough matrix

% Create state-space system for wind turbine
sys = ss(A_sys, B_sys, C_sys, D_sys);

% Simulating wind disturbances (reduced disturbances)
wind_speed = 12 + 0.5 * randn(1, 100000);  % Mean wind speed of 12 m/s with smaller disturbances
T_wind = rho * A_blades * Kp * wind_speed.^3 / 2;  % Wind torque as a function of wind speed

% LQR (Linear Quadratic Regulator) Design (improved weights)
Q = 1000 * (C_sys' * C_sys);  % Further increase penalty on rotor speed error
R = 0.01;  % Further reduce penalty on control effort

% Design LQR controller with improved weights
K_lqr = lqr(A_sys, B_sys, Q, R);

% Simulation setup
T_sim = 1000;  % Simulation time in seconds
dt = 0.1;      % Time step
num_steps = T_sim / dt;

% Initialize variables
rotor_speed = 8;  % Start closer to desired speed
speed_history = zeros(1, num_steps);  % Store the rotor speed values

% Simulation loop
for i = 1:min(num_steps, length(T_wind))
    % Wind disturbance at this time step
    T_wind_current = T_wind(i);
    
    % Compute control action using the LQR controller with feedforward wind compensation
    u = 1.5 * (-K_lqr * (rotor_speed - desired_speed)) + T_wind_current/J;  % Amplified control action
    
    % Simulate the system (update rotor speed)
    dT = (T_wind_current - B * rotor_speed + u) * dt / J;  % Wind turbine dynamics
    rotor_speed = rotor_speed + dT;  % Update rotor speed
    
    % Store the current rotor speed for plotting
    speed_history(i) = rotor_speed;
end

% Plot results
figure;
plot(0:dt:(min(num_steps, length(T_wind))-1)*dt, speed_history(1:min(num_steps, length(T_wind))), 'LineWidth', 2);
hold on;
yline(desired_speed, '--r', 'LineWidth', 1.5);  % Desired rotor speed line
title('Rotor Speed Control with Further LQR Improvements');
xlabel('Time (seconds)');
ylabel('Rotor Speed (rad/s)');
legend('Rotor Speed', 'Desired Speed');
grid on;
