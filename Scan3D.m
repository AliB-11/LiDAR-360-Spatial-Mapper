%% Clean Workspace and Prepare Environment
clc;
clear;

%% Setup Serial Communication
% Define serial port and baud rate, then open the connection.
serialPort = "COM5";
baudRateVal = 115200;
s = serialport(serialPort, baudRateVal, "Timeout", 30);
flush(s);
fprintf("Connected to %s at %d baud.\n", s.Port, baudRateVal);
fprintf("Waiting for the START_CYCLE command...\n");

%% Wait for Start Trigger
% Loop until we detect the trigger string from the external device.
while true
    incomingStr = readline(s);
    if contains(incomingStr, "START_CYCLE")
        disp("Trigger received: START_CYCLE. Commencing measurement capture...");
        break;
    end
end

% Prepare an empty matrix to accumulate measurements
% Each record holds: [distance, angle (deg), depth]
results = zeros(0,3);

%% Acquire Measurement Data
% Continuously read serial data until an end-of-cycle command is received.
while true
    incomingStr = readline(s);
    
    % Check for special commands to manage cycle boundaries.
    if contains(incomingStr, "END_CYCLE")
        disp("END_CYCLE detected. Halting data capture.");
        break;
    elseif contains(incomingStr, "NEW_CYCLE")
        disp("Cycle complete; starting a new cycle. Advance one step.");
        continue;  % Skip the rest and wait for next data line
    elseif isempty(incomingStr)
        continue;  % Ignore blank lines
    end
    
    % Parse the incoming measurement string to extract values.
    parsedValues = customParse(incomingStr);
    % Save only the distance, computed angle, and depth (in order).
    newEntry = parsedValues([2, 3, 4]);
    results = vertcat(results, newEntry);
end

%% Convert to 3D Coordinates & Apply Zigzag Correction
% The sensor's polar data is converted to Cartesian coordinates.
% Note: The conversion uses a depth scaling factor.
depthScaling = 300;
% results columns: 1 = distance, 2 = angle (deg), 3 = depth.
[calcX, calcY] = pol2cart(deg2rad(results(:,2)), results(:,1));
calcZ = results(:,3) * depthScaling;
% Remap to our coordinate system:
%   New X: depth (calcZ)
%   New Y: negative of x (for width)
%   New Z: negative of y (for height)
cartesianData = [calcZ.'; -calcX.'; -calcY.'];

% Group points into layers using the raw depth values (from results) to avoid floating-point issues.
uniqueDepths = unique(results(:,3));
numLayers = length(uniqueDepths);
% Prepare cell array to hold each re-ordered layer.
reorderedLayers = cell(numLayers, 1);
for i = 1:numLayers
    % Get indices for the current depth layer.
    indices = find(results(:,3) == uniqueDepths(i));
    % Extract the corresponding 3D points.
    layerPts = cartesianData(:, indices);
    % Sort points by their angle in the Y-Z plane (i.e. using atan2).
    angles = atan2(layerPts(3,:), layerPts(2,:));
    [~, sortOrder] = sort(angles);
    reorderedLayers{i} = layerPts(:, sortOrder);
end

%% Plotting: Create the 3D Visualization with Corrected Alignment
figure;
hold on;

% Plot closed-loop contours for each layer.
for i = 1:numLayers
    pts = reorderedLayers{i};
    % For a slight offset, apply a cyclic shift to the first layer.
    if i == 1
        pts = circshift(pts, [0, 2]);
    end
    % Draw a closed polygon by connecting all points in sequence.
    plot3(pts(1, [1:end 1]), pts(2, [1:end 1]), pts(3, [1:end 1]), 'Color', [0, 0, 0], 'LineWidth', 1.5);
end

% Overlay the raw scattered data points using red markers.
scatter3(cartesianData(1,:), cartesianData(2,:), cartesianData(3,:), 36, 'r', 'o');

% Draw vertical connections between corresponding points in adjacent layers.
% Here, we assume that each layer contains the same number of measurements.
numPoints = size(reorderedLayers{1}, 2);
for i = 1:(numLayers-1)
    currLayer = reorderedLayers{i};
    nextLayer = reorderedLayers{i+1};
    for j = 1:numPoints
        plot3([currLayer(1,j), nextLayer(1,j)], ...
              [currLayer(2,j), nextLayer(2,j)], ...
              [currLayer(3,j), nextLayer(3,j)], 'Color', [1, 0, 0], 'LineWidth', 1);
    end
end

% Finalize plot appearance.
axis equal;
grid on;
grid minor;
xlabel('X Depth', 'FontSize', 12);
ylabel('Y Width', 'FontSize', 12);
zlabel('Z Height', 'FontSize', 12);
title('2DX3 Project Scan - Ali Bandali - 400532826', 'FontSize', 14);
set(gca, 'FontSize', 10);
set(gca, 'YDir', 'reverse');
hold off;

%% Custom Parsing Function
% Extract numeric values from an incoming comma-separated string.
function parsedData = customParse(inputStr)
    % Print the received string (without a line break) for debugging.
    fprintf("Data received: %s  ", inputStr);
    
    % Expected format: five comma-separated numbers.
    nums = sscanf(inputStr, '%f,%f,%f,%f,%f');
    
    checkVal = nums(1);
    distVal = nums(2);
    rawAngle = nums(3);
    depthVal = nums(4);
    spadVal = nums(5);
    
    % Calculate the effective angle in degrees.
    angleDeg = mod(rawAngle, 2048) * (11.25 / 64);
    
    % Optionally convert to Cartesian coordinates for quick verification.
    [xVal, yVal] = pol2cart(deg2rad(angleDeg), distVal);
    % Multiply the depth by the scaling factor (300) to get height.
    zVal = depthVal * 300;
    fprintf("[X = %.2f, Y = %.2f, Z = %.2f]\n", zVal, -xVal, -yVal);
    
    % Return the parsed data: [check value, distance, angle, depth, spad number]
    parsedData = [checkVal, distVal, angleDeg, depthVal, spadVal];
end