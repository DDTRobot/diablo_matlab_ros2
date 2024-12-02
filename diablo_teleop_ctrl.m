%% Set environment and initialize node
% setenv("ROS_DOMAIN_ID", "42");% use your DOMIAN_ID to replace 42
matlab_diablo_teleop_node = ros2node("/matlab_diablo_teleop_node");
pause(3) % Ensure connection is established

%%
global key_input ;
key_input = '';

% Keyboard listener for teleoperation
clc;
fprintf('Teleop start now!\n');
disp('you can press Esc to exit the teleop control node');
keyboard_listener_fig = figure('KeyPressFcn', @(src, event) keyboardListener(src, event));
set(keyboard_listener_fig, 'Name', 'Teleop Control', 'NumberTitle', 'off');
text_handle = text(0.5, 0.5, '', 'HorizontalAlignment', 'center', 'FontSize', 10,'Interpreter', 'none');
axis off;

% Control message structure
ctrlMsgs = initializeMotionCtrlMsg();
diablo_ctrl_topic = "/diablo/MotionCmd";
DiabloCmdPub = ros2publisher(matlab_diablo_teleop_node,diablo_ctrl_topic,"motion_msgs/MotionCtrl");


%% Main loop
while true
    if ~isempty(key_input)
        key = key_input;
        disp(key);
        if key == "escape"
            break;  % Exit the loop
        else
            ctrlMsgs = generateMsgs(ctrlMsgs, key);
            send(DiabloCmdPub, ctrlMsgs);  % Publish the message
            key_input = '';
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
    pause(0.04);  % 40 ms sleep
end

fprintf('exit!\n');
clear DiabloCmdPub matlab_diablo_teleop_node

%% Keyboard listener
function keyboardListener(src, event)
    global key_input ;
    key_input= event.Key;
    if src == gcf
        if strcmp(event.Key, 'escape')
            disp('press Esc, exit the teleop control node...');
            close(src);  % Close figure window
        end
    end
end

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