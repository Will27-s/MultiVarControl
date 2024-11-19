filename1 = "recorded_data.csv";
filename2 = "recorded_data.csv";

kp = 20;
ki = 129;
kd = 0.79;
Gc = tf([kd, kp, ki], [1, 0])

startIdx = 1000;
A = readmatrix(filename1);
B = readmatrix(filename2);

dt = 2000/1e6;
pos1 = A(startIdx:end,1);
pos2 = B(startIdx:end,2);
in1 = A(startIdx:end,3);
in2 = B(startIdx:end,4);

speed1 = [0;diff(pos1)/dt];
speed2 = [0;diff(pos2)/dt];
time = linspace(0, (length(pos1)) * dt, length(pos1));

data1 = iddata(speed1,in1,dt);
sys1 = tfest(data1,3,2);

overS = tf(1,[1,0]);
G1 = sys1 / (Gc * (1 - sys1 ))

data2 = iddata(speed2,in2,dt);
sys2 = tfest(data2,1);
%G2 = overS * sys2 / (Gc * (1 - sys2 )); % Finding plant system from closed loop response


subplot(1,2,1)
compare(data1,sys1)

subplot(1,2,2)
compare(data2,sys2)

T1 = 1/sys1.Denominator(2)
Lambda1 = sys1.Numerator * T1

T2 = 1/sys2.Denominator(2)
Lambda2 = sys2.Numerator * T1

Ts = 0.35;
[kp1,ki1,kd1] = getControlConstants(T1, Lambda1, Ts)

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