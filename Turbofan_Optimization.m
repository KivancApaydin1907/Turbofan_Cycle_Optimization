% TURBOFAN OPTIMIZATION
tic
clear; clc;

% Inputs
x0 = [400; 10; 3; 40; 1800]; % Initial guess for variables

% Constants
M0 = 0.8; % Mach number
P0 = 19330.41; % Pa, pressure at 12km altitude
T0 = 218.15; % K, temperature at 12km altitude
cp_c = 1005; % J/(kg*K), specific heat (cold section)
cp_h = 1150; % J/(kg*K), specific heat (hot section)
R = 287; % J/kg*K, gas constant
gamma_c = cp_c / (cp_c - R); % Specific heat ratio (cold)
gamma_h = cp_h / (cp_h - R); % Specific heat ratio (hot)
a0 = sqrt(gamma_c * R * T0); % Speed of sound
V0 = M0 * a0; % Intake velocity
hpr = 42000000; % J, heat of combustion

% Efficiencies and ratios
e_f = 0.90; e_c = 0.89; eta_b = 0.985; pi_b = 0.97;
eta_mech_hpt = 0.90; e_hpt = 0.90;
eta_mech_lpt = 0.90; e_lpt = 0.91;
pi_cn = 0.98; pi_bn = 0.98;
pi_d = 0.97;

% Mass flow rates
mc = @(x) x(1) / (x(2) + 1); % Core flow rate
mbp = @(x) (x(1) * x(2)) / (x(2) + 1); % Bypass flow rate

% Atmospheric data
Pt0 = P0 * (1 + ((gamma_c - 1) / 2) * M0^2)^(gamma_c / (gamma_c - 1));
Tt0 = T0 * (1 + ((gamma_c - 1) / 2) * M0^2);

% Diffuser
Tt2 = Tt0; % Adiabatic assumption
pi_recovery = 1; % No shock waves at that Mach number
pi_inlet = pi_d * pi_recovery; % Total reduction in pressure
Pt2 = Pt0 * pi_inlet; % Pa, diffuser outlet pressure

% Fan Blade
tau_f = @(x) x(3)^((gamma_c - 1) / (gamma_c * e_f));
Tt2_5 = @(x) Tt2 * tau_f(x);
Pt2_5 = @(x) Pt2 * x(3);
W_dot_fan = @(x) mbp(x) * cp_c * (Tt2_5(x) - Tt2);

% Compressor
tau_c = @(x) x(4)^((gamma_c - 1) / (gamma_c * e_c));
Tt3 = @(x) Tt2_5(x) * tau_c(x);
Pt3 = @(x) Pt2_5(x) * x(4);
W_dot_compressor = @(x) mc(x) * cp_c * (Tt3(x) - Tt2_5(x));

% Combustor
m_dot_f = @(x) (mc(x) * cp_h * x(5) - mc(x) * cp_c * Tt3(x)) / ...
    (hpr * eta_b - cp_h * x(5));
Pt4 = @(x) Pt3(x) * pi_b;

% High-Pressure Turbine
W_dot_hpt = @(x) W_dot_compressor(x) / eta_mech_hpt;
Tt4_5 = @(x) ((mc(x) + m_dot_f(x)) * cp_h * x(5) - W_dot_compressor(x)) / ...
    ((mc(x) + m_dot_f(x)) * cp_h);
tau_hpt = @(x) Tt4_5(x) / x(5);
Pt4_5 = @(x) Pt4(x) * tau_hpt(x)^((gamma_h) / ((gamma_h - 1) * e_hpt));

% Low-Pressure Turbine
W_dot_lpt = @(x) W_dot_fan(x) / eta_mech_lpt;
Tt5 = @(x) ((mc(x) + m_dot_f(x)) * cp_h * Tt4_5(x) - W_dot_fan(x)) / ...
    ((mc(x) + m_dot_f(x)) * cp_h);
tau_lpt = @(x) Tt5(x) / Tt4_5(x);
Pt5 = @(x) Pt4_5(x) * tau_lpt(x)^((gamma_h) / ((gamma_h - 1) * e_lpt));

% Nozzle
Pt8 = @(x) Pt5(x) * pi_cn;
Tt8 = @(x) Tt5(x);
M8 = 1; % Assumption
P8 = @(x) Pt8(x) / (1 + ((gamma_h - 1) / 2) * M8^2)^(gamma_h / (gamma_h - 1));
T8 = @(x) Tt8(x) / (1 + ((gamma_h - 1) / 2) * M8^2);
a8 = @(x) sqrt(gamma_h * R * T8(x));
V8 = @(x) M8 * a8(x);

% Bypass Nozzle
Pt18 = @(x) Pt2_5(x) * pi_bn;
Tt18 = @(x) Tt2_5(x);
M18 = 1; % Assumption
P18 = @(x) Pt18(x) / (1 + ((gamma_c - 1) / 2) * M18^2)^(gamma_c / (gamma_c - 1));
T18 = @(x) Tt18(x) / (1 + ((gamma_c - 1) / 2) * M18^2);
a18 = @(x) sqrt(gamma_c * R * T18(x));
V18 = @(x) M18 * a18(x);

% Momentum flux and areas
m8 = @(x) mc(x) + m_dot_f(x);
MFP18 = sqrt(gamma_c / R) * M18 * (1 + ((gamma_c - 1) / 2) * M18^2)^(-(gamma_c - 1) / (2 * (gamma_c - 1)));
A18 = @(x) (mbp(x) * sqrt(Tt18(x))) / (Pt18(x) * 1000 * MFP18);
MFP8 = sqrt(gamma_h / R) * M8 * (1 + ((gamma_h - 1) / 2) * M8^2)^(-(gamma_h - 1) / (2 * (gamma_h - 1)));
A8 = @(x) (m8(x) * sqrt(Tt8(x))) / (Pt8(x) * 1000 * MFP8);

% Thrust
Thrust = @(x) (m8(x) * V8(x)) + (mbp(x) * V18(x)) - (x(1) * V0) + ...
    (P8(x) - P0) * A8(x) + (P18(x) - P0) * A18(x);

% Specific Fuel Consumption
SFC = @(x) m_dot_f(x) / Thrust(x);

% Optimization Objective
ObjectiveFunction = @(x) SFC(x);

% Constraints
nonlcon = @(x) deal([], 100000 - Thrust(x)); % Ensure thrust is at least 100 kN

% Bounds
lb = [200, 1, 1.3, 30, 1600]; % Lower bounds
ub = [600, 14, 4, 60, 2100]; % Upper bounds

% Optimization Options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Run Optimization
[optimized_params, fval] = fmincon(ObjectiveFunction, x0, [], [], [], [], lb, ub, nonlcon, options);

% Evaluate thrust at optimized parameters
optimized_thrust = Thrust(optimized_params);

% Display Results
disp('Optimized Parameters:');
disp('m0 (Initial Mass Flow Rate):');
disp(optimized_params(1));
disp('BPR (Bypass Ratio):');
disp(optimized_params(2));
disp('pi_f (Fan Pressure Ratio):');
disp(optimized_params(3));
disp('pi_c (Compressor Pressure Ratio):');
disp(optimized_params(4));
disp('Tt4 (Turbine Inlet Temperature):');
disp(optimized_params(5));

disp('Minimum Specific Fuel Consumption (SFC):');
disp(fval);

disp('Corresponding Thrust:');
disp(optimized_thrust);

toc
