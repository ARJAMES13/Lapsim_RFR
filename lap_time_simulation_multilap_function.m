%% lap_time_simulation.m
% Vehicle + track + simple lap simulation script.
%
% Requirements:
%   - rpm_points, voltage_points, and torque_map must already exist in the
%     workspace, or must be loaded before running this script.
%   - fsg_track_left_right_data.xlsx must be present in the working folder.


clearvars -except rpm_points voltage_points torque_map;
clc;

%% Constants, lookup tables, and tuning parameters

% File names
track_file_name = "fsg_track_left_right_data.xlsx";
solver_log_file_name = "lap_solver_log.mat";

% Simulation configuration
simulation.num_laps = 1;      % Set to 1, 2, 3, ... for multi-lap simulation

% Unit conversions and physical constants
constants.air_density_kgpm3 = 1.225;
constants.gravity_mps2 = 9.81;
constants.seconds_per_minute = 60;
constants.mps_to_kmph = 3.6;
constants.joule_per_kWh = 3.6e6;

% Vehicle geometry and mass
vehicle.tyre_radius = 229 / 1000;      % [m]
vehicle.fdr = 4.1;                     % final drive ratio [-]
vehicle.mass = 270;                    % [kg]
vehicle.frontal_area = 1.2;            % [m^2]
vehicle.front_load_dist = 0.57;        % front static load distribution [-]
vehicle.front_Aero_dist = 0.5;         % front aero load distribution [-]
vehicle.driven_wheels = 2;             % number of driven wheels [-]

% Tyre model parameters
tyre.longitudinal_friction_coeff = 1.1;
tyre.longitudinal_stiffness = 0.0001;
tyre.lateral_friction_coeff = 1.1;
tyre.lateral_stiffness = 0.0001;

% Aero model parameters
aero.default.Cd = 1.2;
aero.default.Cl = 2.2;
aero.rolling_resistance_coeff = 0.05;
aero.eff = 0.95;

% DRS configuration
aero.drs.Cd = 0.1488;
aero.drs.Cl = 0.5963;

% DRS zones are defined using track mesh indices.
% Each row is: [start_index, end_index]
aero.drs_zones = [
    30,   85;
    500,  580;
    700,  750;
    1155, 1225
];

aero.enable_drs = false;

% Track processing parameters
track.num_data_cols = 3;
track.rotation = 0;
track.curve_split_angle_deg = 2;
track.mirror = 0;
track.backward = 0;
track.mesh_size_m = 0.1;
track.straight_radius_threshold_m = 1000;

% GGV generation parameters
ggv.ellipse_points = 45;
ggv.speed_step_mps = 1;

% Battery and powertrain limits
battery.max_pack_voltage_V = 537.6;
battery.power_limit_W = 80000;
battery.internal_resistance_ohm = 0.8;
battery.capacity_Ah = 13.5;
battery.initial_SoC = 1.0;

driveLimits.motor_torque_limit_Nm = 130;

% Pack OCV lookup table
voltageLUT.soc_points = [ ...
    0.00
    0.10
    0.20
    0.30
    0.40
    0.50
    0.60
    0.70
    0.80
    0.90
    1.00
];

voltageLUT.cell_voltage_points = [ ...
    2.50
    3.20
    3.45
    3.58
    3.68
    3.76
    3.84
    3.92
    4.02
    4.12
    4.20
];

voltageLUT.series_cells = 128;

% DC current polynomial model coefficients
% Inputs:
%   x = pack voltage [V]
%   y = motor speed [rpm]
%   z = motor torque [Nm]
currentModel.coeffs = [ ...
     30.8902266940
     -0.2676776326
      0.0045047646
      0.0518698998
      0.0007522720
     -0.0000407406
     -0.0009983754
      0.0000011995
      0.0007925304
      0.0044117065
     -0.0000006752
      0.0000000589
      0.0000017863
     -0.0000000005
     -0.0000010541
     -0.0000080998
     -0.0000000002
     -0.0000000132
     -0.0000001955
     -0.0000167599
];

% Braking model
braking.max_decel_mps2 = 10;

%% Motor map and derived vehicle constants

num_laps = max(1, round(simulation.num_laps));

tyre_radius = vehicle.tyre_radius;
fdr = vehicle.fdr;
m = vehicle.mass;

frontal_area = vehicle.frontal_area;
front_load_dist = vehicle.front_load_dist;
front_Aero_dist = vehicle.front_Aero_dist;
driven_wheels = vehicle.driven_wheels;

Cd = aero.default.Cd;
Cr = aero.rolling_resistance_coeff;
Cl = aero.default.Cl;
eff = aero.eff;

g = constants.gravity_mps2;

longitudinal_friction_coeff = tyre.longitudinal_friction_coeff;
longitudinal_stiffness = tyre.longitudinal_stiffness;
lateral_friction_coeff = tyre.lateral_friction_coeff;
lateral_stiffness = tyre.lateral_stiffness;

mesh_size = track.mesh_size_m;

max_battery_pack_voltage = battery.max_pack_voltage_V;
battery_power_limit = battery.power_limit_W;
battery_internal_resistance = battery.internal_resistance_ohm;

speed_points = rpm_points ./ ...
    (2 * pi * tyre_radius * constants.seconds_per_minute * fdr);

accel_table = torque_map .* (fdr / (tyre_radius * m));

motorMap.rpm_points = rpm_points;
motorMap.voltage_points = voltage_points;
motorMap.torque_map = torque_map;

longitudinal_friction_load = m / 4 * g;
lateral_friction_load = m * g / 4;

front_load = m * front_load_dist;
back_load = m - front_load;

weight = m * g;

ellipse_points = ggv.ellipse_points;
V_ms = min(speed_points):ggv.speed_step_mps:max(speed_points);

GGV = zeros(length(V_ms), 2 * ellipse_points, 3);

%% Generate GGV map

for i = 1:length(V_ms)

    % z-axis / normal loads
    aero_downforce = ...
        1 / 2 * Cl * V_ms(i)^2 * constants.air_density_kgpm3 * frontal_area;

    front_normal = front_load * g + front_Aero_dist * aero_downforce;
    rear_normal = weight + (1 - front_Aero_dist) * aero_downforce - front_normal;

    % x-axis / longitudinal acceleration
    ax_aero_drag = ...
        -1 / m * 1 / 2 * Cd * V_ms(i)^2 * ...
        constants.air_density_kgpm3 * frontal_area;

    ax_tractive_force = getmaxaccel( ...
        max_battery_pack_voltage, ...
        V_ms(i), ...
        motorMap, ...
        vehicle, ...
        battery, ...
        driveLimits, ...
        currentModel, ...
        constants);

    ax_rolling_resistance = -1 / m * (weight + aero_downforce) * Cr;

    ax_tyre_acc_max = ...
        1 / m * ...
        (longitudinal_friction_coeff + ...
        longitudinal_stiffness * ...
        (longitudinal_friction_load - (weight + aero_downforce) / 2)) * ...
        (weight + aero_downforce) * 2;

    ax_tyre_dec_max = ...
        -1 / m * ...
        (longitudinal_friction_coeff + ...
        longitudinal_stiffness * ...
        (longitudinal_friction_load - (weight + aero_downforce) / 4)) * ...
        (weight + aero_downforce);

    % y-axis / lateral acceleration
    ay_max = ...
        1 / m * ...
        (lateral_friction_coeff + ...
        lateral_stiffness * ...
        (lateral_friction_load - (weight + aero_downforce) / 2)) * ...
        (weight + aero_downforce);

    ay = ay_max * cosd(linspace(0, 360, ellipse_points));

    ax_tyre_acc = ax_tyre_acc_max * sqrt(1 - (ay / ay_max).^2);

    ax_acc = min(ax_tyre_acc, ax_tractive_force) + ax_aero_drag;
    ax_dec = ax_tyre_dec_max * sqrt(1 - (ay / ay_max).^2) + ax_aero_drag;

    GGV(i, :, 1) = [ax_acc, ax_dec];
    GGV(i, :, 2) = [ay, ay];
    GGV(i, :, 3) = V_ms(i) * ones(1, 2 * ellipse_points);

end

%% Track modelling

data = readtable(track_file_name);
data = data(:, 1:track.num_data_cols);

rotation = track.rotation;
kappa = track.curve_split_angle_deg;
mirror = track.mirror;
backward = track.backward;

direction_temporary = table2array(data(:, 1));  % Left / Right / Straight
arc_length = table2array(data(:, 2));           % [m]
radius = table2array(data(:, 3));               % [m]

for i = 1:height(data)
    if radius(i) >= track.straight_radius_threshold_m
        direction_temporary{i} = "Straight";
        radius(i) = 0;
    end
end

radius(radius == 0) = inf;

direction = zeros(length(arc_length), 1);
direction(string(direction_temporary) == "Straight") = 0;
direction(string(direction_temporary) == "Left") = 1;
direction(string(direction_temporary) == "Right") = -1;

if mirror == 1
    direction = -direction;
end

% Remove zero-length segments
radius(arc_length == 0) = [];
direction(arc_length == 0) = [];
arc_length(arc_length == 0) = [];

angle_seg = rad2deg(arc_length ./ radius);

radius_new = radius;
direction_new = direction;
arc_length_new = arc_length;

% Point injection for long curves
for i = 1:length(radius)

    if angle_seg(i) > kappa

        length_injection = min( ...
            arc_length(i) / 3, ...
            deg2rad(kappa) * radius(i));

        arc_length_new = [ ...
            arc_length(1:i - 1);
            length_injection;
            arc_length(i) - 2 * length_injection;
            length_injection;
            arc_length(i + 1:end)];

        direction_new = [ ...
            direction(1:i - 1);
            direction(i);
            direction(i);
            direction(i);
            direction(i + 1:end)];

        radius_new = [ ...
            radius(1:i - 1);
            radius(i);
            radius(i);
            radius(i);
            radius(i + 1:end)];

    end

end

radius = radius_new;
direction = direction_new;
arc_length = arc_length_new;

end_point = cumsum(arc_length);
centre_point = cumsum(arc_length) - arc_length / 2;

j = 1;

x = zeros(length(end_point) + sum(radius == inf), 1);
r = zeros(length(end_point) + sum(radius == inf), 1);

for i = 1:length(end_point)

    if radius(i) == inf
        x(j) = end_point(i) - arc_length(i);
        x(j + 1) = end_point(i);
        j = j + 2;
    else
        x(j) = centre_point(i);
        r(j) = direction(i) / radius(i);
        j = j + 1;
    end

end

xx = x;
Length_of_track = x(end);

if Length_of_track < floor(Length_of_track)
    x = [(0:mesh_size:floor(Length_of_track)); Length_of_track];
else
    x = 0:mesh_size:floor(Length_of_track);
end

dx = [0, diff(x)];
n = length(x);

[r, idx] = unique(r);
xx = xx(idx);

r = interp1(xx, r, x, "spline", "extrap");

%% Track coordinates

X = zeros(n, 1);
Y = zeros(n, 1);

angle_of_turn = rad2deg(dx .* r);
heading_angle = cumsum(angle_of_turn);

%% Tangent correction

dh = [ ...
    mod(heading_angle(end), sign(heading_angle(end)) * 360);
    heading_angle(end) - sign(heading_angle(end)) * 360];

[~, idx] = min(abs(dh));
dh = dh(idx);

heading_angle = heading_angle - x / Length_of_track * dh;

angle_of_turn = [heading_angle(1), diff(heading_angle)];
heading_angle = heading_angle - heading_angle(1);

for i = 2:n

    p = [X(i - 1); Y(i - 1); 0];

    xyz = rotz(heading_angle(i - 1)) * [dx(i - 1); 0; 0] + p;

    X(i) = xyz(1);
    Y(i) = xyz(2);

end

DX = x / Length_of_track * (X(1) - X(end));
DY = x / Length_of_track * (Y(1) - Y(end));

X = X + DX';
Y = Y + DY';

figure;
hold on;
grid on;
axis equal;
axis tight;
xlabel("x [m]");
ylabel("y [m]");
title("Track map");
plot(X, Y, ".");

%% Precompute aero coefficients along track

Cd_track = aero.default.Cd * ones(n, 1);
Cl_track = aero.default.Cl * ones(n, 1);

if aero.enable_drs

    for zone_idx = 1:size(aero.drs_zones, 1)

        start_idx = aero.drs_zones(zone_idx, 1);
        end_idx = aero.drs_zones(zone_idx, 2);

        % Clamp indices so the code does not break if track length changes
        start_idx = max(start_idx, 1);
        end_idx = min(end_idx, n);

        if start_idx <= end_idx
            Cd_track(start_idx:end_idx) = aero.drs.Cd;
            Cl_track(start_idx:end_idx) = aero.drs.Cl;
        end

    end

end

%% Pack single-lap solver context

ctx = struct();

ctx.x = x(:);
ctx.X = X(:);
ctx.Y = Y(:);
ctx.r = r(:);
ctx.n = n;
ctx.mesh_size = mesh_size;
ctx.Length_of_track = Length_of_track;

ctx.Cd_track = Cd_track(:);
ctx.Cl_track = Cl_track(:);

ctx.speed_points = speed_points(:);
ctx.motorMap = motorMap;
ctx.vehicle = vehicle;
ctx.battery = battery;
ctx.driveLimits = driveLimits;
ctx.currentModel = currentModel;
ctx.constants = constants;
ctx.voltageLUT = voltageLUT;
ctx.braking = braking;

ctx.frontal_area = frontal_area;
ctx.Cr = Cr;
ctx.longitudinal_friction_coeff = longitudinal_friction_coeff;
ctx.longitudinal_stiffness = longitudinal_stiffness;
ctx.lateral_friction_coeff = lateral_friction_coeff;
ctx.lateral_stiffness = lateral_stiffness;
ctx.longitudinal_friction_load = longitudinal_friction_load;
ctx.lateral_friction_load = lateral_friction_load;
ctx.weight = weight;
ctx.driven_wheels = driven_wheels;
ctx.tyre_radius = tyre_radius;
ctx.m = m;

ctx.max_battery_pack_voltage = max_battery_pack_voltage;
ctx.battery_power_limit = battery_power_limit;
ctx.battery_internal_resistance = battery_internal_resistance;

%% Multi-lap solving loop

lapLogs = cell(num_laps, 1);

current_SoC = battery.initial_SoC;
distance_offset_m = 0;
time_offset_s = 0;

for lap_idx = 1:num_laps

    [lapLogs{lap_idx}, current_SoC] = solveSingleLap( ...
        current_SoC, ...
        lap_idx, ...
        distance_offset_m, ...
        time_offset_s, ...
        ctx);

    distance_offset_m = distance_offset_m + Length_of_track;
    time_offset_s = lapLogs{lap_idx}.time_s(end);

end

solverLog = combineLapLogs(lapLogs, constants);

save(solver_log_file_name, "solverLog");

disp(solverLog.lap_time_s);
disp("Solver log saved to: " + solver_log_file_name);

%% Plot result

figure;
plot(solverLog.accel_ms2);
grid on;

xlabel("Track index");
ylabel("Longitudinal acceleration [m/s^2]");
title("Acceleration profile");

%% Local functions

function [lapLog, final_SoC] = solveSingleLap( ...
    initial_SoC, ...
    lap_idx, ...
    distance_offset_m, ...
    time_offset_s, ...
    ctx)

    x = ctx.x;
    X = ctx.X;
    Y = ctx.Y;
    r = ctx.r;
    n = ctx.n;
    mesh_size = ctx.mesh_size;

    Cd_track = ctx.Cd_track;
    Cl_track = ctx.Cl_track;

    speed_points = ctx.speed_points;
    motorMap = ctx.motorMap;
    vehicle = ctx.vehicle;
    battery = ctx.battery;
    driveLimits = ctx.driveLimits;
    currentModel = ctx.currentModel;
    constants = ctx.constants;
    voltageLUT = ctx.voltageLUT;
    braking = ctx.braking;

    frontal_area = ctx.frontal_area;
    Cr = ctx.Cr;

    longitudinal_friction_coeff = ctx.longitudinal_friction_coeff;
    longitudinal_stiffness = ctx.longitudinal_stiffness;
    lateral_friction_coeff = ctx.lateral_friction_coeff;
    lateral_stiffness = ctx.lateral_stiffness;

    longitudinal_friction_load = ctx.longitudinal_friction_load;
    lateral_friction_load = ctx.lateral_friction_load;
    weight = ctx.weight;
    driven_wheels = ctx.driven_wheels;

    tyre_radius = ctx.tyre_radius;
    m = ctx.m;

    max_battery_pack_voltage = ctx.max_battery_pack_voltage;
    battery_internal_resistance = ctx.battery_internal_resistance;

    %% Solving algorithm

    v_max = zeros(n, 1);
    tps_v_max = zeros(n, 1);

    A = zeros(length(r), 1);
    B = zeros(length(r), 1);
    C = zeros(length(r), 1);

    AX_TYRE_MAX_ACC = zeros(length(r), 1);
    AX_ACC = zeros(length(r), 1);
    AX_DEC = zeros(length(r), 1);
    FACTOR = zeros(length(r), 1);

    for i = 1:n

        Cd = Cd_track(i);
        Cl = Cl_track(i);

        D = -1 / 2 * constants.air_density_kgpm3 * Cl * frontal_area;

        if r(i) == 0

            v = max(speed_points);
            tps_v_max(i) = 1;

        else

            a = -sign(r(i)) * lateral_stiffness / 4 * D^2;

            b = ...
                sign(r(i)) * ...
                (lateral_friction_coeff * D + ...
                lateral_stiffness * lateral_friction_load * D - ...
                lateral_stiffness / 2 * weight * D) - ...
                m * r(i);

            c = ...
                sign(r(i)) * ...
                (lateral_friction_coeff * weight + ...
                lateral_stiffness * lateral_friction_load * weight - ...
                lateral_stiffness / 4 * weight^2);

            A(i) = a;
            B(i) = b;
            C(i) = c;

            % Calculate roots
            if a == 0

                v = sqrt(-c / b);

            elseif (b^2 - 4 * a * c) >= 0

                if (-b + sqrt(b^2 - 4 * a * c)) / 2 / a >= 0

                    v = sqrt((-b + sqrt(b^2 - 4 * a * c)) / 2 / a);

                elseif (-b - sqrt(b^2 - 4 * a * c)) / 2 / a >= 0

                    v = sqrt((-b - sqrt(b^2 - 4 * a * c)) / 2 / a);

                else

                    disp("No real roots");

                end

            else

                disp("discriminant negative");

            end

        end

        v = min(v, max(speed_points));

        %% x-acceleration adjustment for drag

        adjust_speed = true;
        v_to_be_editted = v;

        while adjust_speed

            downforce = D * v^2;
            drag = D * Cd / Cl * v^2;

            rolling_resistance = -(weight - downforce) * Cr;
            Wd = (weight - downforce) / driven_wheels;

            ax_drag = -1 / m * drag;

            ay_max = ...
                (lateral_friction_coeff + ...
                lateral_stiffness * ...
                (lateral_friction_load - (weight + downforce) / 2)) * ...
                (weight + downforce) / m * sign(r(i));

            ay_needed = v^2 * r(i);

            if i > 1

                factor = v - v_max(i - 1);

                if factor < 0
                    factor = -1;
                else
                    factor = 1;
                end

            else

                factor = 1;

            end

            if factor >= 0

                ax_tyre_max_acc = ...
                    1 / m * ...
                    (longitudinal_friction_coeff + ...
                    longitudinal_stiffness * ...
                    (longitudinal_friction_load - (weight + downforce) / 2)) * ...
                    (weight + downforce) * 2 * sign(r(i));

                AX_TYRE_MAX_ACC(i) = ax_tyre_max_acc;

                ax_limit = getmaxaccel( ...
                    max_battery_pack_voltage, ...
                    v, ...
                    motorMap, ...
                    vehicle, ...
                    battery, ...
                    driveLimits, ...
                    currentModel, ...
                    constants);

                ax_tyre_max_acc = min(ax_tyre_max_acc, ax_limit);

                ay = ay_max * sqrt(1 - (ax_drag / ax_tyre_max_acc)^2);

                ax_acc = ax_tyre_max_acc * sqrt(1 - (ay_needed / ay_max)^2);

                scale = min([-ax_drag, ax_acc]) / ax_limit;

                tps_v_max(i) = min([1, scale]);
                tps_v_max(i) = max([tps_v_max(i), 0]);

            else

                ax_tyre_max_dec = ...
                    -1 / m * ...
                    (longitudinal_friction_coeff + ...
                    longitudinal_stiffness * ...
                    (longitudinal_friction_load - (weight + downforce) / 4)) * ...
                    (weight + downforce) * sign(r(i));

                ay = ay_max * sqrt(1 - (ax_drag / ax_tyre_max_dec)^2);

                ax_dec = ax_tyre_max_dec * sqrt(1 - (ay_needed / ay_max)^2);

                fx_tyre = max([ax_drag, -ax_dec]) * m;

                tps_v_max(i) = 0;

            end

            if ay / ay_needed < 1

                v = sqrt(ay / r(i)) - 1;

            else

                if factor == 1
                    ax_acc = ax_tyre_max_acc * sqrt(1 - (ay_needed / ay_max)^2);
                end

                if factor == -1
                    ax_dec = ax_tyre_max_dec * sqrt(1 - (ay_needed / ay_max)^2);
                end

                adjust_speed = false;

            end

        end

        if factor >= 0
            tq = ax_acc * m * tyre_radius;
        else
            tq = ax_dec * m * tyre_radius;
        end

        v_max(i) = v;

    end

    disp("Max speed calculation complete for lap " + lap_idx + ".");

    %% Apex detection

    [v_apex, apex_number] = findpeaks(-v_max);
    v_apex = -v_apex;

    v_dummy = zeros(length(v_max), 1);

    for i = 1:length(v_apex)

        if i == 1

            v_dummy(1:apex_number(i) - 1) = 0;
            v_dummy(apex_number(i)) = v_apex(i);

        else

            v_dummy(apex_number(i - 1) + 1:apex_number(i) - 1) = 0;
            v_dummy(apex_number(i)) = v_apex(i);

        end

    end

    %% Forward acceleration pass

    v_dummy_accel = zeros(length(v_dummy), 1);
    v_dummy_decel = zeros(length(v_dummy), 1);

    v_dummy_accel(1) = 0;

    SoC = initial_SoC;
    capacity = battery.capacity_Ah;

    %% Solver logging setup

    lapLog = struct();

    lapLog.distance_m = x(:) + distance_offset_m;
    lapLog.X_m = X(:);
    lapLog.Y_m = Y(:);
    lapLog.lap_index = lap_idx * ones(length(v_dummy), 1);

    lapLog.v_max_ms = v_max(:);
    lapLog.v_apex_ms = v_dummy(:);

    lapLog.forward_speed_ms = nan(length(v_dummy), 1);
    lapLog.final_speed_ms = nan(length(v_dummy), 1);
    lapLog.accel_ms2 = nan(length(v_dummy), 1);

    lapLog.SoC = nan(length(v_dummy), 1);
    lapLog.voltage_V = nan(length(v_dummy), 1);
    lapLog.current_A = nan(length(v_dummy), 1);

    lapLog.battery_power_W = nan(length(v_dummy), 1);
    lapLog.terminal_power_W = nan(length(v_dummy), 1);
    lapLog.heat_loss_W = nan(length(v_dummy), 1);
    lapLog.accumulator_heat_loss_W = nan(length(v_dummy), 1);

    lapLog.current_source = "getdccurrent polynomial model";
    lapLog.voltage_source = "interpvoltage lookup table";
    lapLog.internal_resistance_ohm = battery.internal_resistance_ohm;
    lapLog.power_limit_W = battery.power_limit_W;
    lapLog.capacity_Ah = capacity;

    lapLog.initial_SoC = initial_SoC;

    lapLog.SoC(1) = SoC;
    lapLog.voltage_V(1) = interpvoltage(SoC, voltageLUT);
    lapLog.current_A(1) = 0;
    lapLog.battery_power_W(1) = 0;
    lapLog.terminal_power_W(1) = 0;
    lapLog.heat_loss_W(1) = 0;
    lapLog.accumulator_heat_loss_W(1) = 0;
    lapLog.forward_speed_ms(1) = v_dummy_accel(1);

    for i = 2:length(v_dummy)

        voltage = interpvoltage(SoC, voltageLUT);

        accel = getmaxaccel( ...
            voltage, ...
            v_dummy(i - 1), ...
            motorMap, ...
            vehicle, ...
            battery, ...
            driveLimits, ...
            currentModel, ...
            constants);

        if v_dummy(i) == 0

            v_dummy_accel(i) = ...
                (v_dummy_accel(i - 1)^2 + 2 * accel * mesh_size)^0.5;

            current = getdccurrent(accel, v_dummy_accel(i), voltage, vehicle, currentModel, constants);

            SoC = SoC - ...
                current * ...
                (v_dummy_accel(i) - v_dummy_accel(i - 1)) / ...
                (accel * capacity * constants.seconds_per_minute^2);

            lapLog.forward_speed_ms(i) = v_dummy_accel(i);
            lapLog.SoC(i) = SoC;
            lapLog.voltage_V(i) = voltage;
            lapLog.current_A(i) = current;
            lapLog.battery_power_W(i) = voltage * current;
            lapLog.terminal_power_W(i) = ...
                (voltage - current * battery.internal_resistance_ohm) * current;
            lapLog.heat_loss_W(i) = current^2 * battery.internal_resistance_ohm;
            lapLog.accumulator_heat_loss_W(i) = lapLog.heat_loss_W(i);

        else

            voltage = interpvoltage(SoC, voltageLUT);

            accel = getmaxaccel( ...
                voltage, ...
                v_dummy(i - 1), ...
                motorMap, ...
                vehicle, ...
                battery, ...
                driveLimits, ...
                currentModel, ...
                constants);

            v_dummy_accel(i) = min( ...
                v_dummy(i), ...
                (v_dummy_accel(i - 1)^2 + 2 * accel * mesh_size)^0.5);

            if v_dummy_accel(i) == ...
                    (v_dummy_accel(i - 1)^2 + 2 * accel * mesh_size)^0.5

                current = getdccurrent(accel, v_dummy_accel(i), voltage, vehicle, currentModel, constants);

                SoC = SoC - ...
                    current * ...
                    (v_dummy_accel(i) - v_dummy_accel(i - 1)) / ...
                    (accel * capacity * constants.seconds_per_minute^2);

                lapLog.forward_speed_ms(i) = v_dummy_accel(i);
                lapLog.SoC(i) = SoC;
                lapLog.voltage_V(i) = voltage;
                lapLog.current_A(i) = current;
                lapLog.battery_power_W(i) = voltage * current;
                lapLog.terminal_power_W(i) = ...
                    (voltage - current * battery.internal_resistance_ohm) * current;
                lapLog.heat_loss_W(i) = current^2 * battery.internal_resistance_ohm;
                lapLog.accumulator_heat_loss_W(i) = lapLog.heat_loss_W(i);

            else

                lapLog.forward_speed_ms(i) = v_dummy_accel(i);
                lapLog.SoC(i) = SoC;
                lapLog.voltage_V(i) = voltage;
                lapLog.current_A(i) = 0;
                lapLog.battery_power_W(i) = 0;
                lapLog.terminal_power_W(i) = 0;
                lapLog.heat_loss_W(i) = 0;
                lapLog.accumulator_heat_loss_W(i) = 0;

            end

        end

    end

    %% Backward braking pass

    v_dummy_decel = v_dummy;

    for i = 1:apex_number(end) - 1

        decel = getmaxdecel(v_dummy_decel(i), braking);

        if v_dummy(apex_number(end) - i) == 0

            v_dummy_decel(apex_number(end) - i) = ...
                (v_dummy_decel(apex_number(end) - i + 1)^2 + ...
                2 * decel * mesh_size)^0.5;

        else

            v_dummy_decel(apex_number(end) - i) = min( ...
                v_dummy(apex_number(end) - i), ...
                (v_dummy_decel(apex_number(end) - i + 1)^2 + ...
                2 * decel * mesh_size)^0.5);

        end

    end

    %% Combine acceleration and braking limits

    for i = 1:apex_number(end)

        v_practice = min(v_dummy_decel, v_dummy_accel);

        accel_new(i) = ...
            (v_practice(i + 1)^2 - v_practice(i)^2) / (2 * mesh_size);

        lapLog.final_speed_ms(i) = v_practice(i);
        lapLog.accel_ms2(i) = accel_new(i);

    end

    for i = apex_number(end) + 1:length(v_dummy)

        voltage = interpvoltage(SoC, voltageLUT);

        accel = getmaxaccel( ...
            voltage, ...
            v_practice(i - 1), ...
            motorMap, ...
            vehicle, ...
            battery, ...
            driveLimits, ...
            currentModel, ...
            constants);

        v_practice(i) = ...
            (v_practice(i - 1)^2 + 2 * accel * mesh_size)^0.5;

        current = getdccurrent(accel, v_practice(i), voltage, vehicle, currentModel, constants);

        SoC = SoC - ...
            current * ...
            (v_practice(i) - v_practice(i - 1)) / ...
            (accel * capacity * constants.seconds_per_minute^2);

        lapLog.final_speed_ms(i) = v_practice(i);
        lapLog.SoC(i) = SoC;
        lapLog.voltage_V(i) = voltage;
        lapLog.current_A(i) = current;
        lapLog.battery_power_W(i) = voltage * current;
        lapLog.terminal_power_W(i) = ...
            (voltage - current * battery.internal_resistance_ohm) * current;
        lapLog.heat_loss_W(i) = current^2 * battery.internal_resistance_ohm;
        lapLog.accumulator_heat_loss_W(i) = lapLog.heat_loss_W(i);

    end

    for i = apex_number(end):length(v_dummy) - 1

        accel_new(i) = ...
            (v_practice(i + 1)^2 - v_practice(i)^2) / (2 * mesh_size);

        lapLog.final_speed_ms(i) = v_practice(i);
        lapLog.accel_ms2(i) = accel_new(i);

    end

    lapLog.final_speed_ms(length(v_practice)) = v_practice(end);
    lapLog.accel_ms2(length(v_practice)) = accel_new(end);

    %% Lap time calculation

    for i = 1:length(v_practice) - 1
        dt(i) = (v_practice(i + 1) - v_practice(i)) / accel_new(i);
    end

    time = cumsum(dt);

    lapLog.dt_s = dt(:);
    lapLog.time_s = [0; time(:)] + time_offset_s;

    lapLog.final_speed_ms = v_practice(:);
    lapLog.accel_ms2 = accel_new(:);

    lapLog.SoC = fillmissing(lapLog.SoC, "previous");
    lapLog.voltage_V = fillmissing(lapLog.voltage_V, "previous");
    lapLog.current_A = fillmissing(lapLog.current_A, "constant", 0);
    lapLog.battery_power_W = fillmissing(lapLog.battery_power_W, "constant", 0);
    lapLog.terminal_power_W = fillmissing(lapLog.terminal_power_W, "constant", 0);
    lapLog.heat_loss_W = fillmissing(lapLog.heat_loss_W, "constant", 0);
    lapLog.accumulator_heat_loss_W = fillmissing(lapLog.accumulator_heat_loss_W, "constant", 0);

    lapLog.final_SoC = SoC;
    lapLog.SoC_used = initial_SoC - SoC;

    lapLog.lap_time_s = time(end);
    lapLog.peak_speed_ms = max(v_practice);
    lapLog.peak_speed_kmh = max(v_practice) * constants.mps_to_kmph;
    lapLog.peak_current_A = max(lapLog.current_A);
    lapLog.peak_battery_power_W = max(lapLog.battery_power_W);
    lapLog.peak_terminal_power_W = max(lapLog.terminal_power_W);
    lapLog.peak_heat_loss_W = max(lapLog.heat_loss_W);

    lapLog.energy_battery_J = trapz(lapLog.time_s, lapLog.battery_power_W);
    lapLog.energy_terminal_J = trapz(lapLog.time_s, lapLog.terminal_power_W);
    lapLog.energy_heat_loss_J = trapz(lapLog.time_s, lapLog.heat_loss_W);
    lapLog.energy_accumulator_heat_loss_J = ...
        trapz(lapLog.time_s, lapLog.accumulator_heat_loss_W);

    lapLog.energy_battery_kWh = lapLog.energy_battery_J / constants.joule_per_kWh;
    lapLog.energy_terminal_kWh = lapLog.energy_terminal_J / constants.joule_per_kWh;
    lapLog.energy_heat_loss_kWh = lapLog.energy_heat_loss_J / constants.joule_per_kWh;
    lapLog.energy_accumulator_heat_loss_kWh = ...
        lapLog.energy_accumulator_heat_loss_J / constants.joule_per_kWh;

    final_SoC = SoC;

end

function solverLog = combineLapLogs(lapLogs, constants)

    num_laps = length(lapLogs);

    solverLog = struct();

    vector_fields = [
        "distance_m"
        "X_m"
        "Y_m"
        "lap_index"
        "v_max_ms"
        "v_apex_ms"
        "forward_speed_ms"
        "final_speed_ms"
        "accel_ms2"
        "SoC"
        "voltage_V"
        "current_A"
        "battery_power_W"
        "terminal_power_W"
        "heat_loss_W"
        "accumulator_heat_loss_W"
        "time_s"
    ];

    for field_idx = 1:length(vector_fields)

        field_name = vector_fields(field_idx);
        solverLog.(field_name) = [];

        for lap_idx = 1:num_laps
            solverLog.(field_name) = [
                solverLog.(field_name);
                lapLogs{lap_idx}.(field_name)
            ];
        end

    end

    solverLog.num_laps = num_laps;

    solverLog.current_source = lapLogs{1}.current_source;
    solverLog.voltage_source = lapLogs{1}.voltage_source;
    solverLog.internal_resistance_ohm = lapLogs{1}.internal_resistance_ohm;
    solverLog.power_limit_W = lapLogs{1}.power_limit_W;
    solverLog.capacity_Ah = lapLogs{1}.capacity_Ah;

    solverLog.initial_SoC = lapLogs{1}.initial_SoC;
    solverLog.final_SoC = lapLogs{end}.final_SoC;
    solverLog.SoC_used = solverLog.initial_SoC - solverLog.final_SoC;

    solverLog.lap_time_s = solverLog.time_s(end);
    solverLog.peak_speed_ms = max(solverLog.final_speed_ms);
    solverLog.peak_speed_kmh = max(solverLog.final_speed_ms) * constants.mps_to_kmph;
    solverLog.peak_current_A = max(solverLog.current_A);
    solverLog.peak_battery_power_W = max(solverLog.battery_power_W);
    solverLog.peak_terminal_power_W = max(solverLog.terminal_power_W);
    solverLog.peak_heat_loss_W = max(solverLog.heat_loss_W);
    solverLog.peak_accumulator_heat_loss_W = max(solverLog.accumulator_heat_loss_W);

    solverLog.energy_battery_J = trapz(solverLog.time_s, solverLog.battery_power_W);
    solverLog.energy_terminal_J = trapz(solverLog.time_s, solverLog.terminal_power_W);
    solverLog.energy_heat_loss_J = trapz(solverLog.time_s, solverLog.heat_loss_W);
    solverLog.energy_accumulator_heat_loss_J = ...
        trapz(solverLog.time_s, solverLog.accumulator_heat_loss_W);

    solverLog.energy_battery_kWh = solverLog.energy_battery_J / constants.joule_per_kWh;
    solverLog.energy_terminal_kWh = solverLog.energy_terminal_J / constants.joule_per_kWh;
    solverLog.energy_heat_loss_kWh = solverLog.energy_heat_loss_J / constants.joule_per_kWh;
    solverLog.energy_accumulator_heat_loss_kWh = ...
        solverLog.energy_accumulator_heat_loss_J / constants.joule_per_kWh;

    solverLog.lap_summary = struct();

    for lap_idx = 1:num_laps

        lap_mask = solverLog.lap_index == lap_idx;
        lap_points = find(lap_mask);

        lap_start = lap_points(1);
        lap_end = lap_points(end);

        solverLog.lap_summary(lap_idx).start_index = lap_start;
        solverLog.lap_summary(lap_idx).end_index = lap_end;
        solverLog.lap_summary(lap_idx).start_distance_m = solverLog.distance_m(lap_start);
        solverLog.lap_summary(lap_idx).end_distance_m = solverLog.distance_m(lap_end);
        solverLog.lap_summary(lap_idx).start_time_s = solverLog.time_s(lap_start);
        solverLog.lap_summary(lap_idx).end_time_s = solverLog.time_s(lap_end);
        solverLog.lap_summary(lap_idx).lap_time_s = ...
            solverLog.time_s(lap_end) - solverLog.time_s(lap_start);
        solverLog.lap_summary(lap_idx).start_SoC = solverLog.SoC(lap_start);
        solverLog.lap_summary(lap_idx).end_SoC = solverLog.SoC(lap_end);
        solverLog.lap_summary(lap_idx).SoC_used = ...
            solverLog.SoC(lap_start) - solverLog.SoC(lap_end);

    end

end

function Voltage = interpvoltage(SoC, voltageLUT)
    % Lookup-table based pack open-circuit voltage estimate.
    %
    % SoC is expected from 0 to 1.
    % Voltage output is full-pack voltage [V].

    SoC = max( ...
        min(SoC, max(voltageLUT.soc_points)), ...
        min(voltageLUT.soc_points));

    cell_voltage = interp1( ...
        voltageLUT.soc_points, ...
        voltageLUT.cell_voltage_points, ...
        SoC, ...
        "linear");

    Voltage = cell_voltage * voltageLUT.series_cells;
end

function max_accel = getmaxaccel( ...
    Voltage, ...
    initial_v, ...
    motorMap, ...
    vehicle, ...
    battery, ...
    driveLimits, ...
    currentModel, ...
    constants)

    rpm_points = motorMap.rpm_points;
    voltage_points = motorMap.voltage_points;
    torque_map = motorMap.torque_map;

    tyre_radius = vehicle.tyre_radius;
    fdr = vehicle.fdr;
    m = vehicle.mass;

    power_limit = battery.power_limit_W;
    internal_resistance = battery.internal_resistance_ohm;
    motor_torque_limit = driveLimits.motor_torque_limit_Nm;

    % Clamp voltage and vehicle speed inside lookup table range
    speed_points = rpm_points ./ ...
        (2 * pi * tyre_radius * constants.seconds_per_minute * fdr);

    Voltage = max(min(Voltage, max(voltage_points)), min(voltage_points));
    initial_v = max(min(initial_v, max(speed_points)), min(speed_points));

    % Interpolate available motor torque from motor map
    max_torque = interp2( ...
        speed_points, ...
        voltage_points, ...
        torque_map, ...
        initial_v, ...
        Voltage, ...
        "linear");

    % Apply explicit motor torque limit
    max_torque = min(max_torque, motor_torque_limit);

    % Convert motor torque to vehicle acceleration
    max_accel = max_torque * fdr / (tyre_radius * m);

    % Apply battery power limit by reducing acceleration
    current = getdccurrent(max_accel, initial_v, Voltage, vehicle, currentModel, constants);
    power_used = (Voltage - current * internal_resistance) * current;

    if power_used > power_limit

        while max_accel > 0

            max_accel = max_accel - 0.01;

            current = getdccurrent(max_accel, initial_v, Voltage, vehicle, currentModel, constants);
            power_used = (Voltage - current * internal_resistance) * current;

            if power_used <= power_limit
                break;
            end

        end

    end

    max_accel = max(max_accel, 0);

end

function current = getdccurrent( ...
    accel, ...
    v_dummy_accel, ...
    voltage, ...
    vehicle, ...
    currentModel, ...
    constants)
    % Estimate DC current from voltage, motor speed, and motor torque.
    %
    % Inputs:
    %   accel          - longitudinal acceleration [m/s^2]
    %   v_dummy_accel  - vehicle speed [m/s]
    %   voltage        - DC bus voltage [V]
    %   vehicle        - struct with tyre_radius, fdr, and mass
    %
    % Output:
    %   current        - estimated DC current [A]

    c = currentModel.coeffs;

    x = voltage;

    % Motor speed [rpm]
    wheel_rpm = ...
        v_dummy_accel * constants.seconds_per_minute / ...
        (2 * pi * vehicle.tyre_radius);

    y = wheel_rpm * vehicle.fdr;

    % Motor torque [Nm]
    wheel_torque = vehicle.mass * accel * vehicle.tyre_radius;
    z = wheel_torque / vehicle.fdr;

    current = ...
        c(1) ...
        + c(2) * x ...
        + c(3) * y ...
        + c(4) * z ...
        + c(5) * x^2 ...
        + c(6) * x * y ...
        + c(7) * x * z ...
        + c(8) * y^2 ...
        + c(9) * y * z ...
        + c(10) * z^2 ...
        + c(11) * x^3 ...
        + c(12) * x^2 * y ...
        + c(13) * x^2 * z ...
        + c(14) * x * y^2 ...
        + c(15) * x * y * z ...
        + c(16) * x * z^2 ...
        + c(17) * y^3 ...
        + c(18) * y^2 * z ...
        + c(19) * y * z^2 ...
        + c(20) * z^3;

    % Current should not be negative during traction.
    current = abs(current);
end

function max_decel = getmaxdecel(future_v, braking)
    max_decel = braking.max_decel_mps2;
end
