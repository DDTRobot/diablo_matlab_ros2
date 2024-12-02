%% Set environment and initialize node
% setenv("ROS_DOMAIN_ID", "42");% use your DOMIAN_ID to replace 42
matlab_diablo_ception_show_node = ros2node("/matlab_diablo_ception_show_node");
pause(3);% Ensure connection is established


%% Initialize global state 
global fig imu_text motor_text battery_text isFigureOpen;
isFigureOpen = true; 

% Create figures and layouts
fig = figure('Name', 'Robot Status', 'NumberTitle', 'off', 'CloseRequestFcn', @closeFigureCallback);
tiledlayout(3, 1); % IMU, Motor, Battery

% IMU
nexttile;
imu_text = text(0.1, 0.2, '', 'FontSize', 8, 'Interpreter', 'none');
title('IMU Data');
axis off;
% Motor
nexttile;
motor_text = text(0.1, 0.2, '', 'FontSize', 8, 'Interpreter', 'none');
title('Motor Data');
axis off;
% Battery
nexttile;
battery_text = text(0.1, 0.5, '', 'FontSize', 8, 'Interpreter', 'none');
title('Battery Data');
axis off;

% ROS Subscribers
ImuSub = ros2subscriber(matlab_diablo_ception_show_node, "/diablo/sensor/Imu", @ImuCallback);
MotorSub = ros2subscriber(matlab_diablo_ception_show_node, "/diablo/sensor/Motors", @motorStatusCallback);
BatterySub = ros2subscriber(matlab_diablo_ception_show_node, "/diablo/sensor/Battery", @batteryStatusCallback);

% To remove the subscribers and node, input the following command into the command window
% clear ImuSub MotorSub BatterySub matlab_diablo_ception_show_node; 

%% Callback functions
function ImuCallback(msg)
    global imu_text isFigureOpen;
    if ~isFigureOpen
        return; 
    end
    x = msg.orientation.x;
    y = msg.orientation.y;
    z = msg.orientation.z;
    w = msg.orientation.w;
    EulerZYX = quat2eul([w x y z], "ZYX");
    imu_data = sprintf(['Quaternion:\n x: %f\n y: %f\n z: %f\n w: %f\n', ...
                        'EulerZYX: %f %f %f\n'], ...
                        x, y, z, w, EulerZYX(1), EulerZYX(2), EulerZYX(3));
    set(imu_text, 'String', imu_data); 
end

function motorStatusCallback(msg)
    global motor_text isFigureOpen;
    if ~isFigureOpen
        return; 
    end
    left_hip_pos = msg.left_hip_pos;
    left_knee_pos = msg.left_knee_pos;
    left_wheel_pos = msg.left_wheel_pos;

    right_hip_pos = msg.right_hip_pos;
    right_knee_pos = msg.right_knee_pos;
    right_wheel_pos = msg.right_wheel_pos;

    motor_data = sprintf(['Left Motor Status:\n left_hip_pos: %f\n left_knee_pos: %f\n left_wheel_pos: %f\n', ...
                          'Right Motor Status:\n right_hip_pos: %f\n right_knee_pos: %f\n right_wheel_pos: %f\n'], ...
                          left_hip_pos, left_knee_pos, left_wheel_pos, ...
                          right_hip_pos, right_knee_pos, right_wheel_pos);
    set(motor_text, 'String', motor_data); 
end

function batteryStatusCallback(msg)
    global battery_text isFigureOpen;
    if ~isFigureOpen
        return; 
    end
    battery_data = sprintf('Battery Voltage: %f V\nBattery Percentage: %f%%\n', ...
                           msg.voltage, msg.percentage);
    set(battery_text, 'String', battery_data); 
end

%% Close figure callback
function closeFigureCallback(~, ~)
    global isFigureOpen ;
    disp('Closing figure');
    isFigureOpen = false; 
    delete(gcf); 
end
