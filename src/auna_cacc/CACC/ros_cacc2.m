clc; clear all;
pyenv('Version','/usr/bin/python3.7');
%ros.ros2.internal.createOrGetLocalPython(true)

if ~any(strcmp(ros2("msg","list"),'auna_its_msgs/CAM'))
    ros2genmsg("~/AuNa/src")
end

% simulation parameters
num_robots = 4;

cIN = 4;

if cIN==0
    Tsim = inf;
elseif cIN == 1
    Tsim = 30;
elseif cIN == 2
    Tsim = 45;
elseif cIN == 3
    Tsim = 30
elseif cIN == 4
    Tsim = 10;
end

% vehicle parameters
L = 0.32;   %wheelbase
v_0 = 0.5;
v_thr = 0.1;
v_threshold = v_thr;


% 1 = 30
% 2 = 45
% 3 = 30

% controller parameters
h = 1.0;                 % time gap, 0.2s
r = 1;                   % standstill distance, 0.5m

k1 = 1    ; k2 = 1;            % (16,9) (3.5,3.5)
kp_x = k1;
kd_x = k2;
kp_y = k1;
kd_y = k2;

x_0 = -1;
y_0 = 0;
theta_0 = 0;

model = 'CACC_Center';
load_system(model);

simIn(1:num_robots-1) = Simulink.SimulationInput(model);

for idx = 1:num_robots-1
    simIn(idx) = simIn(idx).setVariable('Tsim',Tsim);
    
    simIn(idx) = simIn(idx).setVariable('L',L);
    simIn(idx) = simIn(idx).setVariable('v_0',v_0);
    simIn(idx) = simIn(idx).setVariable('v_thr',v_thr);
    simIn(idx) = simIn(idx).setVariable('v_threshold',v_threshold);
    
    simIn(idx) = simIn(idx).setVariable('h',h);
    simIn(idx) = simIn(idx).setVariable('r',r);
    
    simIn(idx) = simIn(idx).setVariable('k1',k1);
    simIn(idx) = simIn(idx).setVariable('k2',k2);
    simIn(idx) = simIn(idx).setVariable('kp_x',kp_x);
    simIn(idx) = simIn(idx).setVariable('kd_x',kd_x);
    simIn(idx) = simIn(idx).setVariable('kp_y',kp_y);
    simIn(idx) = simIn(idx).setVariable('kd_y',kd_y);
    
    simIn(idx) = simIn(idx).setVariable('x_0',x_0);
    simIn(idx) = simIn(idx).setVariable('y_0',y_0);
    simIn(idx) = simIn(idx).setVariable('theta_0',theta_0);
    
    simIn(idx) = simIn(idx).setBlockParameter([model '/subscriber/Subscribe'],'Topic',['/robot' num2str(idx) '/caccCam']);
    simIn(idx) = simIn(idx).setBlockParameter([model '/publisher/Subscribe'],'Topic',['/robot' num2str(idx) '/localization_pose']);
    simIn(idx) = simIn(idx).setBlockParameter([model '/publisher/Subscribe1'],'Topic',['/robot' num2str(idx) '/odom']);
    simIn(idx) = simIn(idx).setBlockParameter([model '/publisher/Publish'],'Topic',['/robot' num2str(idx) '/cmd_vel']);
    simIn(idx) = simIn(idx).setBlockParameter([model '/ToWorkspace'],'VariableName',['robot' num2str(idx)]);
end

tmp_model_name = 'tmp_model';
open_system(tmp_model_name);

allblocks = find_system(tmp_model_name);
allblocks = setdiff(allblocks, {tmp_model_name});

for ii = 1:numel(allblocks)
    try
        delete_block(allblocks{ii})
    catch
        disp(allblocks{ii})
        disp('Some objects couldn''t be deleted')
    end
end


for idx = 1:num_robots-1
    add_block('simulink/Ports & Subsystems/Model',[tmp_model_name '/robot' num2str(idx)]);
    set_param([tmp_model_name '/robot' num2str(idx)], 'ModelName','CACC_Center');
    
    %ModelParameterNames = fieldnames(get_param([tmp_model_name '/robot' num2str(idx)],'ObjectParameters'))
    instanceParams = get_param([tmp_model_name '/robot' num2str(idx)],'InstanceParameters');
    
    instanceParams(1).Name = 'Topic';
    instanceParams(1).Value = ['CACC_Center' '/caccCam'];
    instanceParams(1).Path = Simulink.BlockPath('/subscriber/Subscribe');
    instanceParams(1).Argument = true;
    
    
    setBlockParameter([tmp_model_name '/robot' num2str(idx) '/subscriber/Subscribe'],'Topic',['/robot' num2str(idx) '/caccCam']);
    
    set_param([tmp_model_name '/robot' num2str(idx)],'InstanceParameters',instanceParams);
    return
    
    %set_param([tmp_model_name '/robot' num2str(idx) '/subscriber/Subscribe'], 'Topic',['/robot' num2str(idx) '/caccCam']);
    %set_param([tmp_model_name '/robot' num2str(idx) '/publisher/Subscribe'], 'Topic',['/robot' num2str(idx) '/localization_pose']);
    %set_param([tmp_model_name '/robot' num2str(idx) '/publisher/Subscribe1'], 'Topic',['/robot' num2str(idx) '/odom']);
    %set_param([tmp_model_name '/robot' num2str(idx) '/publisher/Publish'], 'Topic',['/robot' num2str(idx) '/cmd_vel']);
    %set_param([tmp_model_name '/robot' num2str(idx) '/ToWorkspace'], 'VariableName',['robot' num2str(idx)]);
end



return
simOut = parsim(simIn);

figure(1)
