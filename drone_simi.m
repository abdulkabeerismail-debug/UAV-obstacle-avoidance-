%% 1. Initialization: Set up the UAV Environment
disp('Initializing Waypoints and UAV Parameters...');
altitude = 15; 

% Define 3D waypoints [X, Y, Z] to prevent array bounds errors
waypoints = [ 0   0   altitude; 
             10  20   altitude; 
             30  40   altitude; 
             50  10   altitude];

%% 2. Execution: Run the Simulink Model
disp('Starting Simulation Engine...');
% This runs the model and traps the output cleanly
simOut = sim('drone_sim'); 

%% 3. Post-Processing: Extract and Format Data
disp('Processing Flight Data...');
rawData = simOut.actualFlight;

% Clean up Simulink formatting
if isa(rawData, 'timeseries')
    rawData = rawData.Data;
end

% Squeeze and transpose matrix to ensure [N x 3] shape
flightMatrix = squeeze(rawData);
if size(flightMatrix, 1) == 3 && size(flightMatrix, 2) > 3
    flightMatrix = flightMatrix'; 
end

% Isolate axes
flownX = flightMatrix(:, 1);
flownY = flightMatrix(:, 2);
flownZ = flightMatrix(:, 3);

%% 4. Visualization: Plot the Flight Path
disp('Generating Flight Plot...');
figure('Name', 'Emergency RTH Test');

% Plot Planned Route
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), '--b', 'LineWidth', 2);
hold on; grid on;

% Plot Actual Flight Path
plot3(flownX, flownY, flownZ, '-r', 'LineWidth', 3);

% Formatting the Graph
title('Battery Failsafe: Return to Home Triggered');
legend('Planned Route', 'Actual Drone Flight (RTH)');
xlabel('X (meters)'); 
ylabel('Y (meters)'); 
zlabel('Z (meters)');
axis([0 50 0 50 0 30]); % Locks the graph view
view(3);
hold off;

disp('Simulation and Plotting Complete!');