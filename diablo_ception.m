%% Set environment and initialize node
% setenv("ROS_DOMAIN_ID", "42");% use your DOMIAN_ID to replace 42
matlab_diablo_ception_node = ros2node("/matlab_diablo_ception_node");
pause(3);% Ensure connection is established

ImuSub = ros2subscriber(matlab_diablo_ception_node,"/diablo/sensor/Imu",@ImuCallback);
MotorSub = ros2subscriber(matlab_diablo_ception_node,"/diablo/sensor/Motors",@motorStatusCallback);
BatterySub = ros2subscriber(matlab_diablo_ception_node,"/diablo/sensor/Battery",@batteryStatusCallback);

% To remove the subscribers and node, input the following command into the command window
% clear ImuSub MotorSub BatterySub matlab_diablo_ception_node; 
%%
function ImuCallback(msg)
    x = msg.orientation.x;
    y = msg.orientation.y;
    z = msg.orientation.z;
    w = msg.orientation.w;
    EulerZYX = quat2eul([w x y z],"ZYX");
    fprintf('Quaternion:\n x: %f\n y: %f\n z: %f\n w: %f\n',x,y,z,w);
    fprintf('EulerZYX: %f %f %f\n',EulerZYX(1), EulerZYX(2), EulerZYX(3));
end

function motorStatusCallback(msg)
    left_hip_pos = msg.left_hip_pos ;
    left_knee_pos = msg.left_knee_pos ;
    left_wheel_pos = msg.left_wheel_pos ;
    
    right_hip_pos = msg.right_hip_pos ;
    right_knee_pos = msg.right_knee_pos ;
    right_wheel_pos = msg.right_wheel_pos ;
    fprintf('LeftMotorStatus:\n left_hip_pos: %f\n left_knee_pos: %f\n left_wheel_pos: %f\n',left_hip_pos,left_knee_pos,left_wheel_pos);
    fprintf('RightMotorStatus:\n right_hip_pos: %f\n right_knee_pos: %f\n right_wheel_pos: %f\n',right_hip_pos,right_knee_pos,right_wheel_pos);
end

function batteryStatusCallback(msg)
    fprintf('Battery Voltage: %f V\n', msg.voltage);
    fprintf('Battery Percentage: %f%%\n', msg.percentage);
end

