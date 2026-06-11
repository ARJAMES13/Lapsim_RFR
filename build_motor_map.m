%% build_emrax228_hvcc_torque_map.m
% Build an approximate EMRAX 228 High Voltage torque map using PMSM dq equations.
%
% The script generates a lookup table:
%
%   torque_map(Vdc, rpm) = maximum available motor torque [Nm]
%
% considering:
%   - DC bus voltage limit
%   - RMS current limit
%   - torque limit
%   - mechanical power limit
%   - motor speed limit
%
% Output:
%   emrax228_hvcc_torque_map.mat

clear;
clc;
close all;

%% User settings

map_mode = "peak";      % Options: "peak" or "continuous"

output_file = "emrax228_hvcc_torque_map.mat";

%% EMRAX 228 High Voltage parameters

motor.name = "EMRAX 228 HV";

motor.p = 10;                 % Pole pairs [-]
motor.R = 18e-3;              % Phase resistance [ohm]
motor.Ld = 177e-6;            % d-axis inductance [H]
motor.Lq = 183e-6;            % q-axis inductance [H]
motor.lambda = 0.0542;        % Permanent magnet flux linkage [Vs]

motor.I_peak_rms = 240;       % Peak phase current [Arms]
motor.I_cont_rms = 115;       % Continuous phase current [Arms]

motor.T_peak = 240;           % Peak torque [Nm]
motor.T_cont = 125;           % Continuous torque [Nm]

motor.P_peak = 100e3;         % Peak mechanical power [W]
motor.P_cont = 55e3;          % Approx. continuous mechanical power [W]

motor.rpm_peak_limit = 6500;  % Peak speed limit [rpm]
motor.rpm_cont_limit = 5500;  % Normal / continuous speed limit [rpm]

%% Select operating limits

switch lower(map_mode)
    case "peak"
        limits.I_rms = motor.I_peak_rms;
        limits.T = motor.T_peak;
        limits.P = motor.P_peak;
        limits.rpm = motor.rpm_peak_limit;

    case "continuous"
        limits.I_rms = motor.I_cont_rms;
        limits.T = motor.T_cont;
        limits.P = motor.P_cont;
        limits.rpm = motor.rpm_cont_limit;

    otherwise
        error('Invalid map_mode. Use "peak" or "continuous".');
end

%% Lookup table grid

voltage_points = 300:25:670;      % DC bus voltage [V]
rpm_points = 0:100:6500;          % Motor speed [rpm]

num_voltage_points = numel(voltage_points);
num_rpm_points = numel(rpm_points);

torque_map = zeros(num_voltage_points, num_rpm_points);

%% Current search grid

% dq currents are represented as phase peak values.
% id < 0 is used for field weakening.
% iq > 0 produces positive motoring torque.

current_grid_size = 250;

I_peak_limit = limits.I_rms * sqrt(2);

id_candidates = linspace(-I_peak_limit, 0, current_grid_size);
iq_candidates = linspace(0, I_peak_limit, current_grid_size);

[ID, IQ] = meshgrid(id_candidates, iq_candidates);

I_rms = sqrt(ID.^2 + IQ.^2) / sqrt(2);
valid_current = I_rms <= limits.I_rms;

%% Generate torque map

for v_idx = 1:num_voltage_points

    Vdc = voltage_points(v_idx);

    % Approximate maximum line-line RMS voltage available from the inverter.
    % This is a simplified SVPWM-based voltage utilization assumption.
    Vll_rms_available = Vdc / sqrt(2);

    for r_idx = 1:num_rpm_points

        rpm = rpm_points(r_idx);

        if rpm > limits.rpm
            torque_map(v_idx, r_idx) = 0;
            continue;
        end

        omega_mech = rpm * 2*pi/60;          % Mechanical angular speed [rad/s]
        omega_elec = motor.p * omega_mech;   % Electrical angular speed [rad/s]

        % PMSM torque equation
        torque = 1.5 * motor.p .* ...
            (motor.lambda .* IQ + (motor.Ld - motor.Lq) .* ID .* IQ);

        % PMSM steady-state dq voltage equations
        Vd = motor.R .* ID - omega_elec .* motor.Lq .* IQ;
        Vq = motor.R .* IQ + omega_elec .* (motor.Ld .* ID + motor.lambda);

        Vdq_peak = sqrt(Vd.^2 + Vq.^2);

        % Convert dq phase peak voltage to line-line RMS voltage.
        Vll_rms_required = sqrt(3/2) .* Vdq_peak;

        % Mechanical output power
        power_mech = torque .* omega_mech;

        valid_voltage = Vll_rms_required <= Vll_rms_available;
        valid_torque = torque >= 0 & torque <= limits.T;
        valid_power = power_mech <= limits.P;

        valid_operating_points = ...
            valid_current & valid_voltage & valid_torque & valid_power;

        if any(valid_operating_points, "all")
            torque_map(v_idx, r_idx) = max(torque(valid_operating_points));
        else
            torque_map(v_idx, r_idx) = 0;
        end

    end
end

%% Store metadata

map_info.motor_name = motor.name;
map_info.map_mode = map_mode;
map_info.current_grid_size = current_grid_size;
map_info.voltage_model = "Vll_rms_available = Vdc / sqrt(2)";
map_info.description = ...
    "Approximate EMRAX 228 HV torque lookup table generated using dq PMSM equations.";

%% Save lookup table

save(output_file, ...
    "voltage_points", ...
    "rpm_points", ...
    "torque_map", ...
    "motor", ...
    "limits", ...
    "map_info");

fprintf("Saved torque map to: %s\n", output_file);

%% Plot torque map

figure;
surf(rpm_points, voltage_points, torque_map);

xlabel("Motor speed [rpm]");
ylabel("DC bus voltage [V]");
zlabel("Maximum motor torque [Nm]");
title("EMRAX 228 HV: Maximum Torque Map");

shading interp;
grid on;
view(135, 30);