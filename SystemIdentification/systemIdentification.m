filename1 = "systemIdentification.csv";
filename2 = "systemIdentification.csv";

startIdx = 700;
A = readmatrix(filename1);
B = readmatrix(filename2);

dt = 2000/1e6;
pos1 = A(startIdx:end,2);
pos2 = B(startIdx:end,2);
in1 = A(startIdx:end,1);
in2 = B(startIdx:end,4);

smoothed_position = sgolayfilt(pos1, 3, 15); % Polynomial order 3, window size 15
speed1 = [0;diff(smoothed_position)/dt];

speed2 = [0;diff(pos2)/dt];
time = linspace(0, (length(pos1)) * dt, length(pos1));

data1 = iddata(speed1,in1,dt);
sys1 = tfest(data1,1);

data2 = iddata(speed2,in2,dt);
sys2 = tfest(data2,1);
%G2 = overS * sys2 / (Gc * (1 - sys2 )); % Finding plant system from closed loop response

time = 0:dt:2613*dt;

figure
plot(time,in1,'Linewidth',1.5)
xlabel('Time (s)')
ylabel('Input u (PWM)')

figure
compare(data1,sys1);
ylabel('Motor speed (counts/s)')
xlabel('Time')

T1 = 1/sys1.Denominator(2)
Lambda1 = sys1.Numerator * T1

%T2 = 1/sys2.Denominator(2)
%Lambda2 = sys2.Numerator * T1

Ts = 0.35;
u_max = max(in1)
[kp1,ki1,kd1] = getControlConstants(T1, Lambda1, Ts)


%[kp2,ki2,kd2] = getControlConstants(T2, Lambda2, Ts)
Gc = tf([kd1,kp1,ki1],[1,0,0])
G = sys1 * Gc;

% margin(G)

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