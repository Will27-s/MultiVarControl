% file simple_inverse_kinematics.m

clear; close all; clc

h = 90;
l1 = 145;
l2 = 140;
lp = h;

do_pause = 1;

max_accel = 2400; % mm/s^2
max_speed = 320; % mm/s
time_step = 0.002; % s
gearbox_backlash = 1.5*pi/180;  % radians
shapes = strvcat('square','circle','spiral','lissajous');
shape_num = 1;
shape_to_draw = deblank(shapes(shape_num,:));
animate = 1; % 1 = animate motion
animate_step = 3;
arm_colour = 'k';
switch shape_to_draw
    case 'square'
        side_length = 80; x_centre = 150; y_centre = 0;
        t0 = 0; t1 = ceil(max_speed/(max_accel*time_step))*time_step;
        t2 = ceil(side_length/(max_speed*time_step))*time_step;
        t3 = t2+t1;
        t0123_dt = round([t0 t1 t2 t3]/time_step,0)'+1;
        nt = t0123_dt(end);
        t = (0:nt-1)'*time_step;
        max_speed = side_length/t2;        
        max_accel = max_speed/t1;
        v = zeros(nt,1);
        v(t0123_dt(1):t0123_dt(2)) = max_accel*t(t0123_dt(1):t0123_dt(2));
        v(t0123_dt(2):t0123_dt(3)) = max_speed;
        v(t0123_dt(3):t0123_dt(4)) = max_speed - max_accel*(t(t0123_dt(3):t0123_dt(4))-t(t0123_dt(3)));
        s = cumsum(([0 v(1:end-1)']+[0 v(2:end)'])')*time_step/2;
        x = zeros(nt*5,1); y = zeros(nt*5,1);
        x(1:nt) = s+x_centre-side_length/2;
        x(nt+1:2*nt) = x_centre+side_length/2;
        x(2*nt+1:3*nt) = x_centre+side_length/2-s;
        x(3*nt+1:4*nt) = x_centre-side_length/2;
        x(4*nt+1:5*nt) = x(1:nt);        
        y(1:nt) = y_centre-side_length/2;
        y(nt+1:2*nt) = s+y_centre-side_length/2;
        y(2*nt+1:3*nt) = y_centre+side_length/2;
        y(3*nt+1:4*nt) = y_centre+side_length/2-s;
        y(4*nt+1:5*nt) = y(1:nt);
        v = [v; v; v; v; v];
        nt_draw = 5*nt;
        
    case 'circle'
        radius = 50; x_centre = 150; y_centre = 0;
        perimeter = 2.3*pi*radius;  % slightly more than 2*pi to show effect of different backlash state at start of loop 2
        t0 = 0; t1 = ceil(max_speed/(max_accel*time_step))*time_step;
        t2 = ceil(perimeter/(max_speed*time_step))*time_step;
        t3 = t2+t1;
        t0123_dt = round([t0 t1 t2 t3]/time_step,0)'+1;
        nt = t0123_dt(end);
        t = (0:nt-1)'*time_step;
        max_speed = perimeter/t2;
        max_accel = max_speed/t1;
        v = zeros(nt,1);
        v(t0123_dt(1):t0123_dt(2)) = max_accel*t(t0123_dt(1):t0123_dt(2));
        v(t0123_dt(2):t0123_dt(3)) = max_speed;
        v(t0123_dt(3):t0123_dt(4)) = max_speed - max_accel*(t(t0123_dt(3):t0123_dt(4))-t(t0123_dt(3)));
        s = cumsum(([0 v(1:end-1)']+[0 v(2:end)'])')*time_step/2;
        psi = s/radius;
        x = x_centre + radius * cos(psi);
        y = y_centre + radius * sin(psi);
        nt_draw = nt;
        
    case 'spiral'
        r_start = 10; r_end = 60; n_loops = 5;
        x_centre = 130; y_centre = 0;
        drdth = (r_end - r_start)/(2*pi*n_loops); k = drdth;
        s_length = k*(2*pi*n_loops)^2/2 + r_start*2*pi*n_loops; perimeter = s_length;
        t0 = 0; t1 = ceil(max_speed/(max_accel*time_step))*time_step;
        t2 = ceil(perimeter/(max_speed*time_step))*time_step;
        t3 = t2+t1;
        t0123_dt = round([t0 t1 t2 t3]/time_step,0)'+1;
        nt = t0123_dt(end);
        t = (0:nt-1)'*time_step;
        max_speed = perimeter/t2;
        max_accel = max_speed/t1;
        v = zeros(nt,1);
        v(t0123_dt(1):t0123_dt(2)) = max_accel*t(t0123_dt(1):t0123_dt(2));
        v(t0123_dt(2):t0123_dt(3)) = max_speed;
        v(t0123_dt(3):t0123_dt(4)) = max_speed - max_accel*(t(t0123_dt(3):t0123_dt(4))-t(t0123_dt(3)));
        s = cumsum(([0 v(1:end-1)']+[0 v(2:end)'])')*time_step/2;
        th = (-r_start+sqrt(r_start^2+2*k*s))/k;
        r = r_start + k*th;
        x = x_centre + r.*cos(th);
        y = y_centre + r.*sin(th);
        nt_draw = nt;
        
    case 'lissajous'
        radius = 50; x_centre = 130; y_centre = 0;
        x_mult = 3; y_mult = 1;
        perimeter_mult = sqrt(x_mult*y_mult); % approximately
        perimeter = 2*pi*radius*perimeter_mult;
        t0 = 0; t1 = ceil(max_speed/(max_accel*time_step))*time_step;
        t2 = ceil(perimeter/(max_speed*time_step))*time_step;
        t3 = t2+t1;
        t0123_dt = round([t0 t1 t2 t3]/time_step,0)'+1;
        nt = t0123_dt(end);
        t = (0:nt-1)'*time_step;
        max_speed = perimeter/t2;
        max_accel = max_speed/t1;
        v = zeros(nt,1);
        v(t0123_dt(1):t0123_dt(2)) = max_accel*t(t0123_dt(1):t0123_dt(2));
        v(t0123_dt(2):t0123_dt(3)) = max_speed;
        v(t0123_dt(3):t0123_dt(4)) = max_speed - max_accel*(t(t0123_dt(3):t0123_dt(4))-t(t0123_dt(3)));
        s = cumsum(([0 v(1:end-1)']+[0 v(2:end)'])')*time_step/2;
        psi = s/radius/perimeter_mult;
        x = x_centre + radius * cos(x_mult*psi);
        y = y_centre + radius * sin(y_mult*psi);
        nt_draw = nt;
        
    otherwise
end

t_draw = (1:nt_draw)'*time_step;
z = lp*ones(nt_draw,1);

phi = atan2(y,x);
cos_th2 = (x.^2+y.^2-l1^2-l2^2)./(2*l1*l2);
if max(abs(cos_th2)) > 1
    disp('Path is outside the range of the arm')
    pause
end
sin_th2 = sqrt(1-cos_th2.^2);
th2 = atan2(-sin_th2,cos_th2);
A = x;
B = l1+l2*cos(th2);
C = -l2*sin(th2);
u_m = (C-sqrt(C.^2-A.^2+B.^2))./(A+B);
y_pve = find(y>0);
u_m(y_pve) = (C(y_pve)+sqrt(C(y_pve).^2-A(y_pve).^2+B(y_pve).^2))./(A(y_pve)+B(y_pve));
u_m = real(u_m);
th1 = 2*atan(u_m);
l1_x = l1*cos(th1);
l2_x = l1_x + l2*cos(th1+th2);
l1_y = l1*sin(th1);
l2_y = l1_y + l2*sin(th1+th2);
l1_z = h*ones(size(l2_y));
l2_z = h*ones(size(l2_y));

x_fk = l2_x;
y_fk = l2_y;
z_fk = l2_z - lp;

dphi_dt = [0; diff(phi)]/time_step;
dth1_dt = [0; diff(th1)]/time_step;
dth2_dt = [0; diff(th2)]/time_step;
d2phi_dt2 = [0; diff(dphi_dt)]/time_step;
d2th1_dt2 = [0; diff(dth1_dt)]/time_step;
d2th2_dt2 = [0; diff(dth2_dt)]/time_step;

i = 1;
figure(1)
arm_h = plot3([0 0 l1_x(i) l2_x(i) l2_x(i)],[0 0 l1_y(i) l2_y(i) l2_y(i)],[0 h l1_z(i) l2_z(i) l2_z(i)-lp], ...
    'linewidth',4,'color',arm_colour); hold on
plot3(x,y,z-lp); axis equal; grid on
set(gca,'xlim',[-0.2 1]*300,'ylim',[-1 1]*140,'zlim',[-10 170],'view',[0 90])
title('Ideal shape to draw & arm kinematics'); xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')
plot3(x_fk,y_fk,z_fk,'g')
if animate == 1
    for i = 1:animate_step:nt_draw
        set(arm_h,'xdata',[0 0 l1_x(i) l2_x(i) l2_x(i)],'ydata',[0 0 l1_y(i) l2_y(i) l2_y(i)], ...
            'zdata',[0 h l1_z(i) l2_z(i) l2_z(i)-lp]);
        drawnow
    end
end

if do_pause ~= 0, pause, end

figure(2); plot(t_draw,[x y]); legend('x','y')
grid on; title('x & y vs time'); xlabel('time [s]'); ylabel('x & y [mm]'); if do_pause ~= 0, pause, end
figure(3); plot(t_draw, [th1 th2]); legend('th_1','th_2')
grid on; title('Arm angles vs time'); xlabel('time [s]'); ylabel('Arm angles [rad]'); if do_pause ~= 0, pause, end
figure(4); plot(t_draw, [dth1_dt dth2_dt]); legend('dth_1/dt','dth_2/dt')
grid on; title('Arm angular velocities vs time'); xlabel('time [s]'); ylabel('Arm angular velocities [rad/s]'); if do_pause ~= 0, pause, end
figure(5); plot(t_draw, [d2th1_dt2 d2th2_dt2]); legend('d^2th_1/dt^2','d^2th_2/dt^2')
grid on; title('Arm angular accelerations vs time'); xlabel('time [s]'); ylabel('Arm angular accelerations [rad/s^2]'); if do_pause ~= 0, pause, end
speeds = [0; sqrt(diff(x).^2+diff(y).^2)]/time_step;
figure(6); plot(t_draw,[v speeds]); legend('Target','Actual')
grid on; title('Target and actual pen speed vs time'); xlabel('time [s]'); ylabel('Pen speed [mm/s]'); if do_pause ~= 0, pause, end
accels = [0; abs(diff(speeds))]/time_step;
figure(7); plot(t_draw,[max_accel*ones(nt_draw,1) accels]); legend('Target','Actual')
grid on; title('Target and actual pen absolute acceleration vs time'); xlabel('time [s]'); ylabel('Pen acceleration [mm/s^2]'); if do_pause ~= 0, pause, end

% quantisation
counts_per_rotation = 1440;
q = 2*pi / counts_per_rotation;
phi_q = round(phi/q)*q;
th1_q = round(th1/q)*q;
th2_q = round(th2/q)*q;

l1_x_bl = l1*cos(th1_q);
l2_x_q = l1_x_bl + l2*cos(th1_q+th2_q);
l1_y_q = l1*sin(th1_q);
l2_y_q = l1_y_q + l2*sin(th1_q+th2_q);

x_fk_q = l2_x_q;
y_fk_q = l2_y_q;

figure(8)
plot(t_draw,[x_fk_q-x_fk y_fk_q-y_fk]); legend('x','y')
grid on; title('Errors in x & y due to encoder quantisation'); xlabel('time [s]'); ylabel('x & y errors [mm]')
if do_pause ~= 0, pause, end

% backlash
if gearbox_backlash ~= 0    
    th1_bl = gear_backlash(th1, gearbox_backlash);
    th2_bl = gear_backlash(th2, gearbox_backlash);    
    l1_x_bl = l1*cos(th1_bl);
    l2_x_bl = l1_x_bl + l2*cos(th1_bl+th2_bl);    
    l1_y_bl = l1*sin(th1_bl);
    l2_y_bl = l1_y_bl + l2*sin(th1_bl+th2_bl);
    
    x_fk_bl = l2_x_bl;
    y_fk_bl = l2_y_bl;
    z_fk_bl = 0*l2_y_bl;
    
    x_err_1 = x_fk_bl-x_fk;
    y_err_1 = y_fk_bl-y_fk;
    pen_fun_1 = mean(sqrt(abs(x_err_1.^2+y_err_1.^2)));
    
    figure(9)
    plot(t_draw,x_err_1,'r',t_draw,y_err_1,'r--'); legend('x','y'); hold on
    grid on; title('Errors in x & y due to backlash in gearboxes'); xlabel('time [s]'); ylabel('x & y errors [mm]')
    if do_pause ~= 0, pause, end

    figure(1)
    plot3(x_fk_bl,y_fk_bl,z_fk_bl,'r')
    if do_pause ~= 0, pause, end

    xt = x; yt = y; zt = lp*ones(nt_draw,1);
    gamma = 1;
    x = x + gamma*(x-x_fk_bl);
    y = y + gamma*(y-y_fk_bl);
    % z = lp*ones(nt_draw,1);
    z = z + gamma*(z-z_fk_bl)-lp;
    
    phi = atan2(y,x);
    cos_th2 = (x.^2+y.^2-l1^2-l2^2)./(2*l1*l2);
    if max(abs(cos_th2)) > 1
        disp('Path is outside the range of the arm')
        pause
    end
    sin_th2 = sqrt(1-cos_th2.^2);
    th2 = atan2(-sin_th2,cos_th2);
    A = x;
    B = l1+l2*cos(th2);
    C = -l2*sin(th2);
    u_m = (C-sqrt(C.^2-A.^2+B.^2))./(A+B);
    y_pve = find(y>0);
    u_m(y_pve) = (C(y_pve)+sqrt(C(y_pve).^2-A(y_pve).^2+B(y_pve).^2))./(A(y_pve)+B(y_pve));
    u_m = real(u_m);
    th1 = 2*atan(u_m);
    th3 = -th2-th1;
    th1_bl = gear_backlash(th1, gearbox_backlash);
    th2_bl = gear_backlash(th2, gearbox_backlash);

    l1_x_bl = l1*cos(th1_bl);
    l2_x_bl = l1_x_bl + l2*cos(th1_bl+th2_bl);
    l1_y_bl = l1*sin(th1_bl);
    l2_y_bl = l1_y_bl + l2*sin(th1_bl+th2_bl);
    
    x_fk_bl_2 = l2_x_bl;
    y_fk_bl_2 = l2_y_bl;
    z_fk_bl_2 = 0*(l2_y_bl - lp);
    x_err_2 = x_fk_bl_2-xt;
    y_err_2 = y_fk_bl_2-yt;
    z_err_2 = z_fk_bl_2-zt+lp;
    pen_fun_2 = mean(sqrt(abs(x_err_2.^2+y_err_2.^2)));
    figure(9)
    plot(t_draw,x_err_2,'b',t_draw,y_err_2,'b--'); legend('x_1','y_1','x_2','y_2')
    if do_pause ~= 0, pause, end

    figure(1)
    plot3(x_fk_bl_2,y_fk_bl_2,z_fk_bl_2,'b')
    if do_pause ~= 0, pause, end

    figure(10)    
    plot(xt,yt,'g'); grid on; hold on; axis equal
    plot(x_fk_bl,y_fk_bl,'r')
    title('Backlash errors')    
    legend('target','backlash')
    if do_pause ~= 0, pause, end
    % plot(x_fk_bl_2,y_fk_bl_2,'b')
    % title('Backlash errors & correction')    
    % legend('target','backlash','backlash with correction')
    % drawnow
    % if do_pause ~= 0, pause, end
    
    if shape_num ~= 3 % can't find area of spiral
        x_grade = x_fk_bl; y_grade = y_fk_bl; pc = '.r';
        rs = ceil(length(x_grade)/300); % 10; % speed up by reducing number of perimeter points used 
        x_grade = [x_grade(1:rs:end); x_grade(1)];
        y_grade = [y_grade(1:rs:end); y_grade(1)];
        xt_l = [xt(1:rs:end); xt(1)];
        yt_l = [yt(1:rs:end); yt(1)];        
        min_x = floor(min([x_grade; xt_l]));
        max_x = ceil(max([x_grade; xt_l]));
        min_y = floor(min([y_grade; yt_l]));
        max_y = ceil(max([y_grade; yt_l]));
        dx = 0.25; dy = 0.25;
        xdv = (min_x:dx:max_x)';
        ydv = (min_y:dy:max_y)';
        [xgr,ygr] = meshgrid(xdv,ydv);
        in_bl = inpolygon(xgr,ygr,x_grade,y_grade);
        in_t = inpolygon(xgr,ygr,xt_l,yt_l);
        in_one = xor(in_bl,in_t);
        error_metric_1 = sum(sum(in_one))/sum(sum(in_t));
        figure(10)
        plot(xgr(in_one),ygr(in_one),pc)
        title(['Backlash errors.  Error metric: ' num2str(error_metric_1*100,'%0.2g') '%'])        
        legend('target','backlash',['initial error: ' num2str(error_metric_1*100,'%0.2g') '%'])
        if do_pause ~= 0, pause, end
        
        x_grade = x_fk_bl_2; y_grade = y_fk_bl_2; pc = '.b';
        rs = ceil(length(x_grade)/300); % 10; % speed up by reducing number of perimeter points used 
        x_grade = [x_grade(1:rs:end); x_grade(1)];
        y_grade = [y_grade(1:rs:end); y_grade(1)];
        xt_l = [xt(1:rs:end); xt(1)];
        yt_l = [yt(1:rs:end); yt(1)];        
        min_x = floor(min([x_grade; xt_l]));
        max_x = ceil(max([x_grade; xt_l]));
        min_y = floor(min([y_grade; yt_l]));
        max_y = ceil(max([y_grade; yt_l]));
        dx = 0.25; dy = 0.25;
        xdv = (min_x:dx:max_x)';
        ydv = (min_y:dy:max_y)';
        [xgr,ygr] = meshgrid(xdv,ydv);
        in_bl = inpolygon(xgr,ygr,x_grade,y_grade);
        in_t = inpolygon(xgr,ygr,xt_l,yt_l);
        in_one = xor(in_bl,in_t);
        error_metric_2 = sum(sum(in_one))/sum(sum(in_t));
        figure(10)
        plot(x_fk_bl_2,y_fk_bl_2,'b')
        plot(xgr(in_one),ygr(in_one),pc)
        title(['Backlash errors.  Error metric: ' num2str(error_metric_2*100,'%0.2g') '%'])        
        legend('target','backlash',['initial error: ' num2str(error_metric_1*100,'%0.2g') '%'], ...
            'backlash with correction',['improved error: ' num2str(error_metric_2*100,'%0.2g') '%'])
    end    
end

if 1 == 0
    min_th1 = -pi/2; max_th1 = pi/2; min_th2 = -pi; max_th2 = pi;
    n_th1 = 181; n_th2 = 361;
    th1_v = linspace(min_th1, max_th1,n_th1)';
    th2_v = linspace(min_th2, max_th2,n_th2)';
    dth1 = (th1_v(2)-th1_v(1))/10;
    dth2 = (th2_v(2)-th2_v(1))/10;

    [th1_m, th2_m] = meshgrid(th1_v, th2_v);
    x2_m = l1*cos(th1_m) + l2*cos(th1_m+th2_m);
    y2_m = l1*sin(th1_m) + l2*sin(th1_m+th2_m);
    [th1_p1, th2_p1] = meshgrid(th1_v+dth1, th2_v);
    x2_p1 = l1*cos(th1_p1) + l2*cos(th1_p1+th2_p1);
    y2_p1 = l1*sin(th1_p1) + l2*sin(th1_p1+th2_p1);
    dxdth1 = (x2_p1-x2_m)/dth1;
    dydth1 = (y2_p1-y2_m)/dth1;
    [th1_p2, th2_p2] = meshgrid(th1_v, th2_v+dth2);
    x2_p2 = l1*cos(th1_p2) + l2*cos(th1_p2+th2_p2);
    y2_p2 = l1*sin(th1_p2) + l2*sin(th1_p2+th2_p2);
    dxdth2 = (x2_p2-x2_m)/dth2;
    dydth2 = (y2_p2-y2_m)/dth2;

    figure
    % surf(th1_v,th2_v,dxdth1)
    % shading interp
    % xlabel('theta_1'); ylabel('theta_2')
    plot3(x2_m,y2_m,dxdth1,'.b')
    xlabel('x'); ylabel('y')
    title('dx/dth_1 [mm/rad]')
    grid on

    figure
    plot3(x2_m,y2_m,dxdth2,'.b')
    xlabel('x'); ylabel('y')
    title('dx/dth_2 [mm/rad]')
    grid on
end