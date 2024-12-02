%% Set environment and initialize node
% setenv("ROS_DOMAIN_ID", "42");% use your DOMIAN_ID to replace 42
matlab_diablo_interact_node = ros2node("/matlab_diablo_interact_node");
pause(3); % Ensure connection is established

%% Initialize global state in a structured way
global state;
state = struct(...
    'isFigureOpen', true, ...
    'key_input', '', ...
    'imu', struct('counter', 0, 'data', zeros(0, 3), 'plots', gobjects(1, 3),'plot_buffer',200), ...
    'motor', struct('counter', 0, 'data', zeros(0, 6), 'plots', struct('left', gobjects(1, 3), 'right', gobjects(1, 3)),'plot_buffer',200), ...
    'battery', struct('counter', 0, 'voltage', [], 'plot', gobjects(1),'plot_buffer',50) ...
);

% Create figures and layouts
createFiguresAndLayouts();

% ROS Subscribers
ImuSub = ros2subscriber(matlab_diablo_interact_node, "/diablo/sensor/Imu", @ImuCallback);
MotorSub = ros2subscriber(matlab_diablo_interact_node, "/diablo/sensor/Motors", @MotorStatusCallback);
BatterySub = ros2subscriber(matlab_diablo_interact_node, "/diablo/sensor/Battery", @BatteryStatusCallback);

% Keyboard listener for teleoperation
clc;
fprintf('Teleop start now!\n');
disp('you can press Esc to exit the teleop control node');
keyboard_listener_fig = figure('KeyPressFcn', @(src, event) keyboardListener(src, event));
set(keyboard_listener_fig, 'Name', 'Teleop Control', 'NumberTitle', 'off');
text_handle = text(0.5, 0.5, '', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Interpreter', 'none');
axis off;

% Control message initialization
ctrlMsgs = initializeMotionCtrlMsg();
diablo_ctrl_topic = "/diablo/MotionCmd";
DiabloCmdPub = ros2publisher(matlab_diablo_interact_node, diablo_ctrl_topic, "motion_msgs/MotionCtrl");

%% Main loop
subscribe_draw_counter = 0;
while true
    if ~isempty(state.key_input)
        key = state.key_input;
        disp(key);
        if key == "escape"
            break;  % Exit the loop
        else
            ctrlMsgs = generateMsgs(ctrlMsgs, key);
            send(DiabloCmdPub, ctrlMsgs);  % Publish the message
            state.key_input = '';
        end
    else
        % Default message when no key pressed
        ctrlMsgs.mode_mark = false;
        ctrlMsgs.mode.split_mode = false;
        ctrlMsgs.value.forward = 0.0;
        ctrlMsgs.value.left = 0.0;
        ctrlMsgs.value.leg_split = 0.0;
        send(DiabloCmdPub, ctrlMsgs);
    end
    msg_str = struct2str(ctrlMsgs); 
    set(text_handle, 'String', msg_str); 

    if mod(subscribe_draw_counter, 5) == 0
        updateImuPlots();
        updateMotorPlots();
        updateBatteryPlots();
        drawnow;
    end
    subscribe_draw_counter = subscribe_draw_counter + 1;

    pause(0.04);  % 40 ms sleep
end

fprintf('exit!\n');
clear 

%% Function to create figures and layouts
function createFiguresAndLayouts()
    global state;
    fig = figure('Name', 'Robot Status', 'NumberTitle', 'off', 'CloseRequestFcn', @closeFigureCallback);
    tiledlayout(4, 3);  % 4 rows, 3 columns layout
    initializePlots();
end

%% Function to initialize plots
function initializePlots()
    global state;
    titles = {'IMU Roll', 'IMU Pitch', 'IMU Yaw', 'Left Hip Position', 'Left Knee Position', 'Left Wheel Position', ...
              'Right Hip Position', 'Right Knee Position', 'Right Wheel Position', 'Battery Data'};
    for i = 1:9
        nexttile;
        if i <= 3
            state.imu.plots(i) = plot(nan, nan);
        elseif i <= 6
            state.motor.plots.left(i-3) = plot(nan, nan);
        else
            state.motor.plots.right(i-6) = plot(nan, nan);
        end
        title(titles{i}, 'FontSize', 8);
        ylabel('Position (rad)');
        xlabel('Message Number');
    end
    nexttile(10, [1, 3]);
    state.battery.plot = plot(nan, nan);
    title(titles{10}, 'FontSize', 8);
    ylabel('Voltage (V)');
    xlabel('Message Number');
end

%% Callback functions
function ImuCallback(msg)
    global state;
    if ~state.isFigureOpen
        return;
    end
    updateImuData(msg);
end

function MotorStatusCallback(msg)
    global state;
    if ~state.isFigureOpen
        return;
    end
    updateMotorData(msg);
end

function BatteryStatusCallback(msg)
    global state;
    if ~state.isFigureOpen
        return;
    end
    updateBatteryData(msg);
end

%% Close figure callback
function closeFigureCallback(~, ~)
    global state;
    disp('Closing figure');
    state.isFigureOpen = false;
    delete(gcf);
end

%% Update IMU data
function updateImuData(msg)
    global state;
    x = msg.orientation.x;
    y = msg.orientation.y;
    z = msg.orientation.z;
    w = msg.orientation.w;
    EulerZYX = quat2eul([w x y z], "ZYX");
    state.imu.data = [state.imu.data; EulerZYX];
    if size(state.imu.data, 1) > state.imu.plot_buffer
        state.imu.data = state.imu.data(end-(state.imu.plot_buffer-1):end, :);
    end
    state.imu.counter = state.imu.counter + 1;
end

%% Update IMU plots
function updateImuPlots()
    global state;
    for i = 1:3
        set(state.imu.plots(i), 'XData', max(1, state.imu.counter-(state.imu.plot_buffer-1)):state.imu.counter, 'YData', state.imu.data(:, 3-i+1));
    end
%     drawnow;
end

%% Update motor data
function updateMotorData(msg)
    global state;
    new_data = [msg.left_hip_pos, msg.left_knee_pos, msg.left_wheel_pos, msg.right_hip_pos, msg.right_knee_pos, msg.right_wheel_pos];
    state.motor.data = [state.motor.data; new_data];
    if size(state.motor.data, 1) > state.motor.plot_buffer
        state.motor.data = state.motor.data(end-(state.motor.plot_buffer-1):end, :);
    end
    state.motor.counter = state.motor.counter + 1;
end

%% Update motor plots
function updateMotorPlots()
    global state;
    for i = 1:3
        set(state.motor.plots.left(i), 'XData', max(1, state.motor.counter-(state.motor.plot_buffer-1)):state.motor.counter, 'YData', state.motor.data(:, i));
        set(state.motor.plots.right(i), 'XData', max(1, state.motor.counter-(state.motor.plot_buffer-1)):state.motor.counter, 'YData', state.motor.data(:, i+3));
    end
%     drawnow;
end

%% Update battery data
function updateBatteryData(msg)
    global state;
    state.battery.voltage = [state.battery.voltage; msg.voltage];
    if length(state.battery.voltage) > state.battery.plot_buffer
        state.battery.voltage = state.battery.voltage(end-(state.battery.plot_buffer-1):end);
    end
    state.battery.counter = state.battery.counter + 1;
end

%% Update battery plots
function updateBatteryPlots()
    global state;
    set(state.battery.plot, 'XData', max(1, state.battery.counter-(state.battery.plot_buffer-1)):state.battery.counter, 'YData', state.battery.voltage);
%     drawnow;
end

%% Keyboard listener
function keyboardListener(src, event)
    global state;
    if src == gcf
        state.key_input = event.Key;
        if strcmp(event.Key, 'escape')
            disp('Press Esc, exit the node...');
            close(src);  % Close figure window
        end
    end
end

%%
function ctrlMsgs = initializeMotionCtrlMsg()
    % Initialize the MotionCtrl message structure
    ctrlMsgs = ros2message("motion_msgs/MotionCtrl");
    ctrlMsgs.mode_mark = false;
    ctrlMsgs.mode = struct('jump_mode', false, 'split_mode', false, ...
                           'height_ctrl_mode', false, 'pitch_ctrl_mode', false, ...
                           'roll_ctrl_mode', false, 'stand_mode', false);
    ctrlMsgs.value = struct('forward', 0.0, 'left', 0.0,'leg_split',0.0, ...
                            'pitch', 0.0, 'roll', 0.0, 'up', 0.0);
end


function ctrlMsgs = generateMsgs(ctrlMsgs, key)
    % Update control message based on the input key
    switch key
        case 'w'
            ctrlMsgs.value.forward = 1.0;
        case 's'
            ctrlMsgs.value.forward = -1.0;
        case 'a'
            ctrlMsgs.value.left = 1.0;
        case 'd'
            ctrlMsgs.value.left = -1.0;
        case 'e'
            ctrlMsgs.value.roll = 0.1;
        case 'q'
            ctrlMsgs.value.roll = -0.1;
        case 'r'
            ctrlMsgs.value.roll = 0.0;
        case 'h'
            ctrlMsgs.value.up = -0.5;
        case 'j'
            ctrlMsgs.value.up = 1.0;
        case 'k'
            ctrlMsgs.value.up = 0.5;
        case 'l'
            ctrlMsgs.value.up = 0.0;
        case 'u'
            ctrlMsgs.value.pitch = 0.5;
        case 'i'
            ctrlMsgs.value.pitch = 0.0;
        case 'o'
            ctrlMsgs.value.pitch = -0.5;
        case 'z'
            ctrlMsgs.mode_mark = true;
            ctrlMsgs.mode.stand_mode = true;
        case 'x'
            ctrlMsgs.mode_mark = true;
            ctrlMsgs.mode.stand_mode = false;
        case 'f'
            ctrlMsgs.mode_mark = true;
            ctrlMsgs.mode.split_mode = true;
        case 'g'
            ctrlMsgs.mode_mark = true;
            ctrlMsgs.mode.split_mode = false;
        otherwise
            % Do nothing for unrecognized keys
    end
end

%% Convert struct to string for display
function str = struct2str(structVar)
    fields = fieldnames(structVar);
    str = '';
    for i = 1:length(fields)
        field = fields{i};
        value = structVar.(field);
        if isstruct(value)
            subfields = struct2str(value);
            str = [str, sprintf('%s:\n%s\n', field, subfields)];
        else
            str = [str, sprintf('%s: %s\n', field, mat2str(value))];
        end
    end
end