filename = "recorded_data.csv";
startIdx = 1000;
A = readmatrix(filename);
dt = 2000/1e6;
pos1 = A(startIdx:end,1);
pos2 = A(startIdx:end,2);
in1 = A(startIdx:end,3);
in2 = A(startIdx:end,4);

speed1 = [0;diff(pos1)/dt];
speed2 = [0;diff(pos2)/dt];
time = linspace(0, (length(pos1)) * dt, length(pos1));

data1 = iddata(speed1,in1,dt);
sys1 = tfest(data1,1);

data2 = iddata(speed2,in2,dt);
sys2 = tfest(data2,1);


subplot(1,2,1)
compare(data1,sys1)

subplot(1,2,2)
compare(data2,sys2)

T1 = 1/sys1.Denominator(2)
Lambda1 = sys1.Numerator * T1

T2 = 1/sys2.Denominator(2)
Lambda2 = sys2.Numerator * T1

Ts = 0.35;
[kp1,ki1,kd2] = getControlConstants(T1, Lambda1, Ts)

[kp2,ki2,kd2] = getControlConstants(T2, Lambda2, Ts)

function [kp, ki, kd] = getControlConstants(T, Lambda, Ts)
    % getControlConstants calculates control constants for a control system.
    % 
    % Inputs:
    %   T      - Settling time (plant)
    %   Lambda - Control tuning parameter
    %   Ts     - Desired closed-loop settling time
    %
    % Outputs:
    %   kp - Proportional gain
    %   ki - Integral gain
    %   kd - Derivative gain

    % Calculate closed-loop natural frequency
    w_cl = 7.55 / Ts;

    % Calculate control constants
    kp = (3 * w_cl^2 * T) / Lambda;
    ki = (w_cl^3 * T) / Lambda;
    kd = (3 * w_cl * T - 1) / Lambda;
end