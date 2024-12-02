filename1 = "Trianglebest.csv";

% order: 
% motor 1 | motor 2
% u | pos | error | ref

A = readmatrix(filename1);
dt = 2000/1e6;

startIdx = 800;
endIdx = 3000;

time = 0:dt:(endIdx-startIdx)*dt

u1 = A(startIdx:endIdx,1);
pos1 = A(startIdx:endIdx,2);
error1 = A(startIdx:endIdx,3);
ref1 = A(startIdx:endIdx,4);
u2 = A(startIdx:endIdx,5);
pos2 = A(startIdx:endIdx,6);
error2 = A(startIdx:endIdx,7);
ref2 = A(startIdx:endIdx,8);


meanerror1 = mean(abs(error1))
meanerror2 = mean(abs(error2))

figure;
 hold on;
 
 plot(time,pos1,'LineWidth',1)
  plot(time,ref1,'LineWidth',1)
  plot(time,pos2,'LineWidth',1)
  plot(time,ref2,'LineWidth',1)
  xlabel('Time (s)')
  ylabel('Motor Position (counts)')
  lgd1 = legend('Motor 1 Position','Motor 1 Reference', 'Motor 2 Position', 'Motor 2 Reference');
 hold off;

figure
hold on
plot(time,error1)
plot(time,error2)
xlabel('Time (s)')
ylabel('Error (counts)')
lgd2 = legend('Motor 1 Error', 'Motor 2 Error');
hold off

figure
hold on
plot(time,u1)
plot(time, u2)
xlabel('Time (s)')
ylabel('Input (PWM)')
lgd3 = legend('Motor 1 Input', 'Motor 2 Input');


