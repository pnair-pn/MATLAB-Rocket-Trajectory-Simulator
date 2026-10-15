% MATLAB Rocket Trajectory Simulator
% Pranav Nair 2026

clear;
clc;
close all;

%% ============================================================
%  1. ROCKET PARAMETERS
%  ============================================================

% Rocket mass
dryMass = 20;             
fuelMass = 10;             

% Engine
thrust = 500;              
burnTime = 5;              

% Rocket geometry
diameter = 0.10;           
Cd = 0.30;                 

% Launch conditions
launchAngle = input('Input a desired launch angle between 0 and 90 degrees. ');      
initialVelocity = 0;      
initialAltitude = 0;    

%% ============================================================
%  2. CONSTANTS
%  ============================================================

g = 9.81;                  % gravitational acceleration (m/s^2)

rho0 = 1.225;              % sea-level air density (kg/m^3)
scaleHeight = 8500;        % atmospheric scale height (m)

% Cross-sectional area
A = pi * (diameter / 2)^2;

% Convert angle to radians
theta = deg2rad(launchAngle);

%% ============================================================
%  3. SIMULATION SETTINGS
%  ============================================================

dt = 0.01;                 % time step (seconds)
maxTime = 300;             % maximum simulation time (seconds)

t = 0:dt:maxTime;

% Number of time steps
N = length(t);

%% ============================================================
%  4. PREALLOCATE ARRAYS
%  ============================================================

x = zeros(1,N);
y = zeros(1,N);

vx = zeros(1,N);
vy = zeros(1,N);

ax = zeros(1,N);
ay = zeros(1,N);

speed = zeros(1,N);

mass = zeros(1,N);
fuelRemaining = zeros(1,N);

thrustHistory = zeros(1,N);
dragHistory = zeros(1,N);
airDensity = zeros(1,N);

%% ============================================================
%  5. INITIAL CONDITIONS
%  ============================================================

x(1) = 0;
y(1) = initialAltitude;

vx(1) = initialVelocity * cos(theta);
vy(1) = initialVelocity * sin(theta);

mass(1) = dryMass + fuelMass;
fuelRemaining(1) = fuelMass;

%% ============================================================
%  6. MAIN SIMULATION LOOP
%  ============================================================

lastIndex = N;

for i = 1:N-1

    %% --------------------------------------------------------
    %  A. CHECK IF ROCKET IS STILL BURNING
    %  --------------------------------------------------------

    if t(i) <= burnTime && fuelRemaining(i) > 0

        % Thrust is active
        currentThrust = thrust;

        % Constant fuel burn rate
        fuelBurnRate = fuelMass / burnTime;

        % Fuel used during this time step
        fuelUsed = fuelBurnRate * dt;

        % Prevent fuel from going negative
        fuelUsed = min(fuelUsed, fuelRemaining(i));

        fuelRemaining(i+1) = fuelRemaining(i) - fuelUsed;

    else

        % Engine is off
        currentThrust = 0;

        % No more fuel
        fuelRemaining(i+1) = 0;

    end

    %% --------------------------------------------------------
    %  B. CALCULATE CURRENT MASS
    %  --------------------------------------------------------

    mass(i) = dryMass + fuelRemaining(i);

    %% --------------------------------------------------------
    %  C. CALCULATE SPEED
    %  --------------------------------------------------------

    speed(i) = sqrt(vx(i)^2 + vy(i)^2);

    %% --------------------------------------------------------
    %  D. CALCULATE AIR DENSITY
    %  --------------------------------------------------------

    % Prevent negative altitude from affecting density
    currentAltitude = max(y(i),0);

    airDensity(i) = rho0 * exp(-currentAltitude / scaleHeight);

    %% --------------------------------------------------------
    %  E. CALCULATE DRAG
    %  --------------------------------------------------------

    if speed(i) > 0

        drag = 0.5 * airDensity(i) * speed(i)^2 * Cd * A;

    else

        drag = 0;

    end

    %% --------------------------------------------------------
    %  F. BREAK THRUST INTO X AND Y COMPONENTS
    %  --------------------------------------------------------

    thrustX = currentThrust * cos(theta);
    thrustY = currentThrust * sin(theta);

    %% --------------------------------------------------------
    %  G. BREAK DRAG INTO X AND Y COMPONENTS
    %  --------------------------------------------------------

    if speed(i) > 0

        dragX = -drag * vx(i) / speed(i);
        dragY = -drag * vy(i) / speed(i);

    else

        dragX = 0;
        dragY = 0;

    end

    %% --------------------------------------------------------
    %  H. CALCULATE NET FORCES
    %  --------------------------------------------------------

    forceX = thrustX + dragX;

    forceY = thrustY + dragY - mass(i) * g;

    %% --------------------------------------------------------
    %  I. CALCULATE ACCELERATION
    %  --------------------------------------------------------

    ax(i) = forceX / mass(i);

    ay(i) = forceY / mass(i);

    %% --------------------------------------------------------
    %  J. UPDATE VELOCITY
    %  --------------------------------------------------------

    vx(i+1) = vx(i) + ax(i) * dt;

    vy(i+1) = vy(i) + ay(i) * dt;

    %% --------------------------------------------------------
    %  K. UPDATE POSITION
    %  --------------------------------------------------------

    x(i+1) = x(i) + vx(i) * dt + ...
             0.5 * ax(i) * dt^2;

    y(i+1) = y(i) + vy(i) * dt + ...
             0.5 * ay(i) * dt^2;

    %% --------------------------------------------------------
    %  L. STORE DATA
    %  --------------------------------------------------------

    thrustHistory(i) = currentThrust;
    dragHistory(i) = drag;

    %% --------------------------------------------------------
    %  M. CHECK IF ROCKET HAS LANDED
    %  --------------------------------------------------------

    if y(i+1) < 0 && t(i+1) > 0

        % Interpolate approximately to ground level
        lastIndex = i + 1;

        break;

    end

end

%% ============================================================
%  7. TRIM ARRAYS
%  ============================================================

t = t(1:lastIndex);

x = x(1:lastIndex);
y = y(1:lastIndex);

vx = vx(1:lastIndex);
vy = vy(1:lastIndex);

ax = ax(1:lastIndex);
ay = ay(1:lastIndex);

speed = speed(1:lastIndex);

mass = mass(1:lastIndex);
fuelRemaining = fuelRemaining(1:lastIndex);

thrustHistory = thrustHistory(1:lastIndex);
dragHistory = dragHistory(1:lastIndex);
airDensity = airDensity(1:lastIndex);

%% ============================================================
%  8. CALCULATE RESULTS
%  ============================================================

[maxAltitude, altitudeIndex] = max(y);

[maxSpeed, speedIndex] = max(speed);

[maxRange, ~] = max(x);

flightTime = t(end);

peakAltitudeTime = t(altitudeIndex);
peakSpeedTime = t(speedIndex);

%% ============================================================
%  9. DISPLAY RESULTS
%  ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('       ROCKET TRAJECTORY SIMULATOR\n');
fprintf('============================================\n');

fprintf('\nROCKET PARAMETERS\n');
fprintf('Dry Mass:              %.2f kg\n', dryMass);
fprintf('Initial Fuel Mass:     %.2f kg\n', fuelMass);
fprintf('Total Initial Mass:    %.2f kg\n', dryMass + fuelMass);
fprintf('Thrust:                %.2f N\n', thrust);
fprintf('Burn Time:             %.2f s\n', burnTime);
fprintf('Diameter:              %.3f m\n', diameter);
fprintf('Drag Coefficient:      %.2f\n', Cd);
fprintf('Launch Angle:          %.2f degrees\n', launchAngle);

fprintf('\nSIMULATION RESULTS\n');
fprintf('Maximum Altitude:      %.2f m\n', maxAltitude);
fprintf('Maximum Altitude:      %.2f km\n', maxAltitude / 1000);

fprintf('Maximum Speed:         %.2f m/s\n', maxSpeed);
fprintf('Maximum Speed:         %.2f km/h\n', maxSpeed * 3.6);

fprintf('Maximum Range:         %.2f m\n', maxRange);
fprintf('Maximum Range:         %.2f km\n', maxRange / 1000);

fprintf('Flight Time:           %.2f s\n', flightTime);

fprintf('Time of Peak Altitude: %.2f s\n', peakAltitudeTime);
fprintf('Time of Peak Speed:    %.2f s\n', peakSpeedTime);


%% ============================================================
%  10. ROCKET TRAJECTORY GRAPH
%  ============================================================

figure;

plot(x, y, 'LineWidth', 2);

grid on;

xlabel('Horizontal Distance (m)');
ylabel('Altitude (m)');

title('Rocket Trajectory');

%% ============================================================
%  11. ALTITUDE VS TIME GRAPH
%  ============================================================

figure;

plot(t, y, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Altitude (m)');

title('Altitude vs. Time');

%% ============================================================
%  12. SPEED VS TIME GRAPH
%  ============================================================

figure;

plot(t, speed, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Speed (m/s)');

title('Speed vs. Time');

%% ============================================================
%  13. ACCELERATION VS TIME GRAPH
%  ============================================================

acceleration = sqrt(ax.^2 + ay.^2);

figure;

plot(t, acceleration, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

title('Acceleration vs. Time');

%% ============================================================
%  14. THRUST & DRAG GRAPH
%  ============================================================

figure;

plot(t, thrustHistory, 'LineWidth', 2);

hold on;

plot(t, dragHistory, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Force (N)');

title('Thrust vs. Aerodynamic Drag');

legend('Thrust', 'Drag');

%% ============================================================
%  15. MASS VS TIME GRAPH
%  ============================================================

figure;

plot(t, mass, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Mass (kg)');

title('Rocket Mass vs. Time');

%% ============================================================
%  16. AIR DENSITY VS ALTITUDE GRAPH
%  ============================================================

figure;

plot(y, airDensity, 'LineWidth', 2);

grid on;

xlabel('Altitude (m)');
ylabel('Air Density (kg/m^3)');

title('Atmospheric Density vs. Altitude');