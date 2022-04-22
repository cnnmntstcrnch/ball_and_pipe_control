% A MATLAB script to control Rowans Systems & Control Floating Ball 
% Apparatus designed by Mario Leone, Karl Dyer and Michelle Frolio. 
% The current control system is a PID controller.
%
% Created by Kyle Naddeo, Mon Jan 3 11:19:49 EST 
% Modified by Adriana Fasino 4/15/2022

%% Start fresh
close all; clc; clear device;

%% Connect to device
device = serialport("COM3",19200); %open serial communication in the proper COM port

%% Parameters
target      = 0.5;   % Desired height of the ball [m]
sample_rate = 0.01;  % Amount of time between controll actions [s]

%% Give an initial burst to lift ball and keep in air
set_pwm(device,4000); % Initial burst to pick up ball
pause(0.9) % Wait 0.1 seconds
%set_pwm(device,4000); % Set to lesser value to level out somewhere in the pipe
%pause(0.9)
%set_pwm(device,2000);
%% Initialize variables
action      = 4000; % Same value of last set_pwm   
error       = 0;
error_sum   = 0;


target_pwm = 4000; %find unknown
prev_action = target_pwm;
%% Feedback loop
while true
    %% Read current height
    [distance,pwm,~,deadpan] = read_data(device);
    [y,~] = ir2y(distance); % Convert from IR reading to distance from bottom [m]
    
    %% Calculate errors for PID controller
    error_prev = error;             % D
    error      = target - y       % P
    error_sum  = error + error_sum % I
    
    %% Control
    prev_action = action;

    kp = 40;
    kd = 2000;
    ki = 0;
    fkp = kp * error;
    fkd = kd * (error - error_prev);
    fki = ki * error_sum;
    sum = fkp + fkd + fki;
    %action = target_pwm + sum;
    action = prev_action + sum
    set_pwm(device,action); 
    % Implement action    
    % Wait for next sample
    pause(sample_rate)
end

