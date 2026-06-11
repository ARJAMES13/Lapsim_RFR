%% postprocess_lap_simulation_terminal_only.m
% Post-processing plots for the lap simulation solver with logged outputs.
%
% Run this script AFTER running:
%
%   lap_time_simulation_with_logging.m
%
% The solver must save:
%
%   lap_solver_log.mat
%
% containing:
%
%   solverLog
%
% Figures generated:
%   Figure 1  — Track map (XY with speed colour-mapped)
%   Figure 2  — Velocity, acceleration, current, voltage, SoC, terminal power, accumulator heat loss vs distance
%   Figure 3  — GGV envelope surface
%   Figure 4  — Terminal power heat map along the track
%   Figure 6  — Summary statistics panel
%   Figure 7  — Electrical logs vs time
%   Figure 8  — Terminal energy and accumulator heat loss energy vs distance

%% Housekeeping

close all;
clc;

%% Load solver log

if exist("solverLog", "var") ~= 1

    if exist("lap_solver_log.mat", "file") == 2

        loaded_data = load("lap_solver_log.mat");

        if isfield(loaded_data, "solverLog")
            solverLog = loaded_data.solverLog;
        else
            error("lap_solver_log.mat exists, but it does not contain solverLog.");
        end

    else

        error("solverLog not found. Run the logged solver first or place lap_solver_log.mat in the current folder.");

    end

end

%% Check required logged fields

required_log_fields = [
    "distance_m"
    "X_m"
    "Y_m"
    "final_speed_ms"
    "accel_ms2"
    "SoC"
    "voltage_V"
    "current_A"
    "terminal_power_W"
    "heat_loss_W"
];

for k = 1:length(required_log_fields)

    field_name = required_log_fields(k);

    if ~isfield(solverLog, field_name)
        error("Missing field in solverLog: %s", field_name);
    end

end

%% Pull logged values

distance_m = solverLog.distance_m(:);
X = solverLog.X_m(:);
Y = solverLog.Y_m(:);

velocity_ms = solverLog.final_speed_ms(:);
acceleration_ms2 = solverLog.accel_ms2(:);

SoC = solverLog.SoC(:);
voltage_V = solverLog.voltage_V(:);
current_A = solverLog.current_A(:);

terminal_power_W = solverLog.terminal_power_W(:);

% This is I^2R loss in the accumulator internal resistance model.
% It is named as accumulator heat loss for plotting and reporting.
accumulator_heat_loss_W = solverLog.heat_loss_W(:);

if isfield(solverLog, "time_s")
    time_s = solverLog.time_s(:);
else
    time_s = [];
end

%% Clean vector shapes and lengths

n = min([
    length(distance_m)
    length(X)
    length(Y)
    length(velocity_ms)
    length(acceleration_ms2)
    length(SoC)
    length(voltage_V)
    length(current_A)
    length(terminal_power_W)
    length(accumulator_heat_loss_W)
]);

distance_m = distance_m(1:n);
X = X(1:n);
Y = Y(1:n);

velocity_ms = velocity_ms(1:n);
acceleration_ms2 = acceleration_ms2(1:n);

SoC = SoC(1:n);
voltage_V = voltage_V(1:n);
current_A = current_A(1:n);

terminal_power_W = terminal_power_W(1:n);
accumulator_heat_loss_W = accumulator_heat_loss_W(1:n);

if isempty(time_s)
    dx = [0; diff(distance_m)];
    velocity_for_dt = velocity_ms;
    velocity_for_dt(velocity_for_dt < 0.1) = 0.1;
    dt = dx ./ velocity_for_dt;
    time_s = cumsum(dt);
else
    time_s = time_s(1:min(length(time_s), n));
    if length(time_s) < n
        time_s(end + 1:n) = time_s(end);
    end
end

distance_km = distance_m / 1000;
velocity_kmh = velocity_ms * 3.6;

terminal_power_kW = terminal_power_W / 1000;
accumulator_heat_loss_kW = accumulator_heat_loss_W / 1000;

%% Pull optional GGV variables from base workspace

has_GGV = false;

if exist("GGV", "var") == 1
    has_GGV = true;
elseif evalin("base", "exist('GGV', 'var')")
    GGV = evalin("base", "GGV");
    has_GGV = true;
end

%% Energy calculations from logged values

terminal_energy_J = cumtrapz(time_s, terminal_power_W);
accumulator_heat_loss_energy_J = cumtrapz(time_s, accumulator_heat_loss_W);

terminal_energy_kWh = terminal_energy_J / 3.6e6;
accumulator_heat_loss_energy_kWh = accumulator_heat_loss_energy_J / 3.6e6;

%% Figure 1 — Track map with speed colour map

figure("Name", "Track map with speed colour map");

scatter(X, Y, 18, velocity_kmh, "filled");
axis equal;
grid on;
box on;

xlabel("X position [m]");
ylabel("Y position [m]");
title("Track map coloured by vehicle speed");

cb = colorbar;
ylabel(cb, "Speed [km/h]");

%% Figure 2 — Logged values vs distance

figure("Name", "Logged lap variables vs distance");

tiledlayout(6, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(distance_m, velocity_kmh, "LineWidth", 1.3);
grid on;
ylabel("Speed [km/h]");
title("Logged solver values vs distance");

nexttile;
plot(distance_m, acceleration_ms2, "LineWidth", 1.3);
grid on;
ylabel("a_x [m/s^2]");

nexttile;
plot(distance_m, SoC * 100, "LineWidth", 1.3);
grid on;
ylabel("SoC [%]");

nexttile;
plot(distance_m, voltage_V, "LineWidth", 1.3);
grid on;
ylabel("Voltage [V]");

nexttile;
plot(distance_m, current_A, "LineWidth", 1.3);
grid on;
ylabel("Current [A]");

nexttile;
plot(distance_m, terminal_power_kW, "LineWidth", 1.3);
grid on;
ylabel("Terminal power [kW]");



%% Figure 3 — GGV envelope surface

if has_GGV

    figure("Name", "GGV envelope surface");

    ax_values = reshape(GGV(:, :, 1), [], 1);
    ay_values = reshape(GGV(:, :, 2), [], 1);
    speed_values = reshape(GGV(:, :, 3), [], 1);

    scatter3( ...
        ay_values, ...
        ax_values, ...
        speed_values * 3.6, ...
        12, ...
        speed_values * 3.6, ...
        "filled");

    grid on;
    box on;

    xlabel("Lateral acceleration a_y [m/s^2]");
    ylabel("Longitudinal acceleration a_x [m/s^2]");
    zlabel("Speed [km/h]");
    title("GGV envelope");

    cb = colorbar;
    ylabel(cb, "Speed [km/h]");

    view(135, 25);

else

    warning("GGV was not found. Figure 3 skipped.");

end

%% Figure 4 — Terminal power heat map along the track

figure("Name", "Terminal power heat map along track");

scatter(X, Y, 22, terminal_power_kW, "filled");
axis equal;
grid on;
box on;

xlabel("X position [m]");
ylabel("Y position [m]");
title("Logged terminal power heat map along track");

cb = colorbar;
ylabel(cb, "Terminal power [kW]");

%% Figure 6 — Summary statistics panel

track_length_m = distance_m(end) - distance_m(1);

peak_speed_kmh = max(velocity_kmh);
avg_speed_kmh = mean(velocity_kmh, "omitnan");

peak_accel_ms2 = max(acceleration_ms2);
peak_decel_ms2 = min(acceleration_ms2);

peak_voltage_V = max(voltage_V);
min_voltage_V = min(voltage_V);

peak_current_A = max(current_A);
avg_current_A = mean(current_A, "omitnan");

peak_terminal_power_kW = max(terminal_power_kW);
avg_terminal_power_kW = mean(terminal_power_kW, "omitnan");

peak_accumulator_heat_loss_kW = max(accumulator_heat_loss_kW);
avg_accumulator_heat_loss_kW = mean(accumulator_heat_loss_kW, "omitnan");

initial_SoC = SoC(1);
final_SoC = SoC(end);
SoC_used = initial_SoC - final_SoC;

total_time_s = time_s(end);

summary_text = {
    sprintf("Track length                    : %.1f m", track_length_m)
    sprintf("Estimated lap time              : %.2f s", total_time_s)
    sprintf("Peak speed                      : %.2f km/h", peak_speed_kmh)
    sprintf("Average speed                   : %.2f km/h", avg_speed_kmh)
    sprintf("Peak acceleration               : %.2f m/s^2", peak_accel_ms2)
    sprintf("Peak deceleration               : %.2f m/s^2", peak_decel_ms2)
    sprintf("Initial SoC                     : %.3f %%", initial_SoC * 100)
    sprintf("Final SoC                       : %.3f %%", final_SoC * 100)
    sprintf("SoC used                        : %.3f %%", SoC_used * 100)
    sprintf("Maximum voltage                 : %.2f V", peak_voltage_V)
    sprintf("Minimum voltage                 : %.2f V", min_voltage_V)
    sprintf("Peak current                    : %.2f A", peak_current_A)
    sprintf("Average current                 : %.2f A", avg_current_A)
    sprintf("Peak terminal power             : %.2f kW", peak_terminal_power_kW)
    sprintf("Average terminal power          : %.2f kW", avg_terminal_power_kW)
    sprintf("Peak accumulator heat loss      : %.2f kW", peak_accumulator_heat_loss_kW)
    sprintf("Average accumulator heat loss   : %.2f kW", avg_accumulator_heat_loss_kW)
    sprintf("Terminal energy                 : %.4f kWh", terminal_energy_kWh(end))
    sprintf("Accumulator heat loss energy    : %.4f kWh", accumulator_heat_loss_energy_kWh(end))
};

figure("Name", "Summary statistics");

axis off;

text( ...
    0.05, ...
    0.95, ...
    summary_text, ...
    "Units", "normalized", ...
    "VerticalAlignment", "top", ...
    "FontName", "Consolas", ...
    "FontSize", 12);

title("Logged lap simulation summary statistics");

%% Figure 7 — Electrical logs vs time

figure("Name", "Electrical logs vs time");

tiledlayout(5, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(time_s, SoC * 100, "LineWidth", 1.3);
grid on;
ylabel("SoC [%]");
title("Electrical logs vs time");

nexttile;
plot(time_s, voltage_V, "LineWidth", 1.3);
grid on;
ylabel("Voltage [V]");

nexttile;
plot(time_s, current_A, "LineWidth", 1.3);
grid on;
ylabel("Current [A]");

nexttile;
plot(time_s, terminal_power_kW, "LineWidth", 1.3);
grid on;
ylabel("Terminal power [kW]");

nexttile;
plot(time_s, accumulator_heat_loss_kW, "LineWidth", 1.3);
grid on;
xlabel("Time [s]");
ylabel("Acc. heat loss [kW]");

%% Figure 8 — Energy accumulation vs distance

figure("Name", "Energy accumulation vs distance");

plot(distance_m, terminal_energy_kWh, "LineWidth", 1.3);
hold on;
plot(distance_m, accumulator_heat_loss_energy_kWh, "LineWidth", 1.3);

grid on;
box on;

xlabel("Distance [m]");
ylabel("Energy [kWh]");
title("Cumulative logged energy vs distance");

legend( ...
    "Terminal energy", ...
    "Accumulator heat loss energy", ...
    "Location", "best");

%% Export processed arrays to base workspace

post = struct();

post.distance_m = distance_m;
post.distance_km = distance_km;
post.X = X;
post.Y = Y;

post.time_s = time_s;

post.velocity_ms = velocity_ms;
post.velocity_kmh = velocity_kmh;
post.acceleration_ms2 = acceleration_ms2;

post.SoC = SoC;
post.voltage_V = voltage_V;
post.current_A = current_A;

post.terminal_power_W = terminal_power_W;
post.accumulator_heat_loss_W = accumulator_heat_loss_W;

post.terminal_energy_J = terminal_energy_J;
post.accumulator_heat_loss_energy_J = accumulator_heat_loss_energy_J;

post.terminal_energy_kWh = terminal_energy_kWh;
post.accumulator_heat_loss_energy_kWh = accumulator_heat_loss_energy_kWh;

post.summary_text = summary_text;

assignin("base", "post", post);

disp("Logged post-processing complete. Results stored in variable: post");
