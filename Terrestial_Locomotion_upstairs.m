clc; clear all; close all;
%% Question A - Finding Joint Trajectories
%% Initial Conditions
M = 100; L = 0.65; D_0 = 0.07; D_1 = 0.21; D = D_0 + D_1; H = 0.21; t_step = 40e-3; g = 9.81;

%Position of Center of Mass(COM)
x_com(1) =0; y_com(1) = 2*L;

th_hr(1) = 0; th_kr(1) = 0; %Chosing clockwise direction as +ve
th_hl(1) = 30; th_kl(1) = (360-60); % Chosing anticlockwise direction as +ve

%Velocity of COM
vx_com(1) = 0.20; vy_com(1) = 0;

% From the expression of vx_com we get 0.2/L = w_kr + 2*w_hr for initial condition
% Thus assuming w_kr = 0, we get w_hr = 0.2/2*L
w_hr(1) = 0.2/(2*L); w_kr(1) = 0;

%Putting initial conditions in the ZMP equation we get acc_kr +2acc_hr = 0
%But choosing acc_hr(1) = 0.1; acc_kr(1) = -2*acc_hr(1); gives -ve value
%for w_hr which will mean that after the knee is at rest at w_hr(1) = 0 it will start
%rotating in anti-clockwise direction instead of clockwise, which is not
%possible. Hence we choose
acc_hr(1) = 0; acc_kr(1) = 0;

%By putting the initial values we get ay_com = -1/2L
ax_com(1) = 0; ay_com(1) = -1/(2*L);

w_hr(2) = w_hr(1) + acc_hr(1)*t_step;
w_kr(2) = w_kr(1) + acc_kr(1)*t_step; 
acc_hr(2) = (w_hr(2) - w_hr(1))/t_step;
acc_kr(2) = (w_kr(2) - w_kr(1))/t_step;

th_hr(2) = th_hr(1) + w_hr(1)*t_step;
th_kr(2) = th_kr(1) + w_kr(1)*t_step;

x_com(2) = L*( sin(th_kr(2)+ th_hr(2)) + sin(th_hr(2)));
y_com(2) = L*( cos(th_kr(2)+ th_hr(2)) + cos(th_hr(2)));

i = 3;

%% Calculating the trajectory of th_hr and th_kr when the right leg is in contact with the ground

while( x_com(i-1) <= 0.28)
    
    th_hr(i) = sind(i);
    w_hr(i) = (th_hr(i) - th_hr(i-1))/t_step;
    acc_hr(i) = (w_hr(i) - w_hr(i-1))/t_step;
    
    %From the formula for ZMP we find acc_kr, we get
    w_kr(i) = w_kr(i-1) + acc_kr(i-1)*t_step;
    th_kr(i) = th_kr(i-1) + w_kr(i-1)*t_step;
    
    
    %acc_kr(i) = ( (sin(th_kr(i)) + sin(th_hr(i)))*( -acc_hr(i)*sin(th_kr(i)) - ((w_kr(i) + w_hr(i))^(2))*(cos(th_kr(i))) - acc_hr(i)*sin(th_hr(i)) - ((w_hr(i))^(2))*cos(th_hr(i)) + g/L )...
    %         - (cos(th_kr(i)) + cos(th_hr(i)))*( acc_hr(i)*cos(th_kr(i)) - ((w_kr(i) + w_hr(i))^(2))*( sin(th_kr(i))) - acc_hr(i)*cos(th_hr(i)) - ((w_hr(i))^(2))*sin(th_hr(i)) ) )/ ( cos(th_kr(i))*(cos(th_kr(i)) + cos(th_hr(i))) + sin(th_kr(i))*( sin(th_kr(i)) + sin(th_hr(i)) ) );
    
    acc_kr(i) = ( (sin(th_kr(i) + th_hr(i)) + sin(th_hr(i)))*( -acc_hr(i)*sin(th_kr(i) + th_hr(i)) - ((w_kr(i) + w_hr(i))^(2))*(cos(th_kr(i) + th_hr(i))) - acc_hr(i)*sin(th_hr(i)) - ((w_hr(i))^(2))*cos(th_hr(i)) + g/L )...
              - (cos(th_kr(i) + th_hr(i)) + cos(th_hr(i)))*( acc_hr(i)*cos(th_kr(i) + th_hr(i)) - ((w_kr(i) + w_hr(i))^(2))*(sin(th_kr(i) + th_hr(i))) - acc_hr(i)*cos(th_hr(i)) + ((w_hr(i))^(2))*sin(th_hr(i)) ) )/ ( cos(th_kr(i) + th_hr(i))*(cos(th_kr(i) + th_hr(i)) + cos(th_hr(i))) + sin(th_kr(i) + th_hr(i))*( sin(th_kr(i) + th_hr(i)) + sin(th_hr(i)) ) );
    
    % Now we can also calculate the position,velocity and acceleration of center of mass
    x_com(i) = L*( sin(th_kr(i)+ th_hr(i)) + sin(th_hr(i)));
    y_com(i) = L*( cos(th_kr(i)+ th_hr(i)) + cos(th_hr(i)));
    
    vx_com(i) =  L*((w_kr(i) +w_hr(i))*cos(th_kr(i) + th_hr(i)) + w_hr(i)*cos(th_hr(i)));
    %The negative sign here means the velocity is in downwards direction
    vy_com(i) = -L*((w_kr(i) +w_hr(i))*sin(th_kr(i) + th_hr(i)) +w_hr(i)*sin(th_hr(i)));
    
    ax_com(i) =  L *((acc_kr(i) + acc_hr(i))*cos(th_kr(i))  -(w_kr(i)+w_hr(i))^2*sin(th_kr(i))  +acc_hr(i)*cos(th_hr(i))  -w_hr(i)^2*sin(th_hr(i)) );
    ay_com(i) = -L *((acc_kr(i) + acc_hr(i))*sin(th_kr(i))  +(w_kr(i)+w_hr(i))^2*cos(th_kr(i))  +acc_hr(i)*sin(th_hr(i))  +w_hr(i)^2*cos(th_hr(i)) );
    
    i = i + 1;
    
end




%% Trajectory for angles of left leg, assuming that the th_hl and th_kl vary linearly till they touch the ground

%The hip angle varies from 30 degrees to 35.5038 degrees
for j = 1:12
    %Using formula y - y1 = m(x - x1)
    th_hl(j) = (30 + ((35.5038 - 30)/(12-1))*(j-1))*(pi/180);
    if j==1
        w_hl(j) = 0;
        w_kl(j) = 0;
        acc_kl = 0;
        acc_hl = 0;
    end
    if j > 1
        w_hl(j) = (th_hl(j) - th_hl(j-1))/t_step;
        acc_hl(j) = (w_hl(j) - w_hl(j-1))/t_step;
    end
    
    %According to actual experiments on Biomechanical analysis of height
    %and knee flexion in walking stairs, it can be seen before the rising
    %feet and COM reaches the stair the knee bends more than required to prevent
    %collision of foot with stair, and after that the bend in the knee
    %decreases
    
    %According to the ZMP calculations for the right leg the COM reaches
    %the stair at i = 9, hence bending the knee to (360 - 80) till i = 9 instead
    %of (360 - 71.0076) to prevent collision of foot with the stair and
    %from i = 9 to i = 12 decreasing the bend in the knee to (360 - 71.0076), so that the foot
    % lands on the stair
    
    %Since we are taking the anti-clockwise direction as positive the th_kl
    %varies from (360-(180 - 120)) = (360 - 60) to (360 - (180 - 108.9924))
    %= (360 - 71.0076)
        
    if j < 10
        th_kl(j) = (300 + ((280 - 300)/(9-1))*(j-1))*(pi/180);
        if j > 1
            w_kl(j) = (th_kl(j) - th_kl(j-1))/t_step;
            acc_kl(j) = (w_kl(j) - w_kl(j-1))/t_step;
        end
    elseif j >= 10
        th_kl(j) = (280 + ((288.9924 - 280)/(12-10))*(j-10))*(pi/180);
        w_kl(j) = (th_kl(j) - th_kl(j-1))/t_step;
        acc_kl(j) = (w_kl(j) - w_kl(j-1))/t_step;
    end
    
end


%{


%% At this point both the legs are in contact with the ground

%Assuming that the left leg remains straight as the COM moves from the
%center of step length i.e. 0.75/2 to center of right half of the step i.e.
%(0.75/2)*(1/2), the hip angle th_hl varies from 16.75 to 8.292 linearly

syms hr kr hl kl;
while( x_com(i-1) < (0.75/2 + 0.75/4))
    
    th_hr(i) = sind(i);
    w_hr(i) = (th_hr(i) - th_hr(i-1))/t_step;
    acc_hr(i) = (w_hr(i) - w_hr(i-1))/t_step;
    
    %From the formula for ZMP we find acc_kr, we get
    w_kr(i) = w_kr(i-1) + acc_kr(i-1)*t_step;
    th_kr(i) = th_kr(i-1) + w_kr(i-1)*t_step;
    
    acc_kr(i) = ( (sin(th_kr(i) + th_hr(i)) + sin(th_hr(i)))*( -acc_hr(i)*sin(th_kr(i) + th_hr(i)) - ((w_kr(i) + w_hr(i))^(2))*(cos(th_kr(i) + th_hr(i))) - acc_hr(i)*sin(th_hr(i)) - ((w_hr(i))^(2))*cos(th_hr(i)) + g/L )...
              - (cos(th_kr(i) + th_hr(i)) + cos(th_hr(i)))*( acc_hr(i)*cos(th_kr(i) + th_hr(i)) - ((w_kr(i) + w_hr(i))^(2))*(sin(th_kr(i) + th_hr(i))) - acc_hr(i)*cos(th_hr(i)) + ((w_hr(i))^(2))*sin(th_hr(i)) ) )/ ( cos(th_kr(i) + th_hr(i))*(cos(th_kr(i) + th_hr(i)) + cos(th_hr(i))) + sin(th_kr(i) + th_hr(i))*( sin(th_kr(i) + th_hr(i)) + sin(th_hr(i)) ) );
    
    %% Calculating angles for left leg from Physical realism
    cos_hr = cos(th_hr(i) + th_kr(i)) + cos(th_hr(i));
    sin_hr = sin(th_hr(i) + th_kr(i)) + sin(th_hr(i));
    %[sol_hr sol_kr] = solve(cos(kr)*cos(hr) - sin(kr)*sin(hr) + cos(hr) == cos_hl, sin(kr)*cos(hr) + cos(kr)*sin(hr) + sin(hr) == D/L + 2*sin_hl)
    [sol_hl, sol_kl] = solve(cos(kl + hl) + cos(hl) == cos_hr, sin(kl + hl) + sin(hl) == sin_hr );
    
    
    for s = 1:length(sol_hl)
        sol_hl = double(sol_hl)
        [idx idx] = min(sol_hl - th_hl(i-1));
        th_hl(i) = sol_hl(idx);
        w_hl(i) = (th_hl(i) - th_hl(i-1))/t_step;
        acc_hl(i) = (w_hl(i) - w_hl(i-1))/t_step;
    end
    
    for s = 1:length(sol_kl)
        sol_kl = double(sol_kl)
        [idx idx] = min(sol_kl - th_kl(i-1));
        th_kl(i) = sol_kl(idx);
        if th_kl(i) > (360*pi/180) || th_kl(i) < 0
            th_kl(i) = th_kl(i-1) + w_kl(i-1)*t_step;
        end
        w_kl(i) = (th_kl(i) - th_kl(i-1))/t_step;
        acc_kl(i) = (w_kl(i) - w_kl(i-1))/t_step;
    end
    
          
    %% Now we can also calculate the position,velocity and acceleration of center of mass
    x_com(i) = L*( sin(th_kr(i)+ th_hr(i)) + sin(th_hr(i)));
    y_com(i) = L*( cos(th_kr(i)+ th_hr(i)) + cos(th_hr(i)));
    
    vx_com(i) =  L*((w_kr(i) +w_hr(i))*cos(th_kr(i) + th_hr(i)) + w_hr(i)*cos(th_hr(i)));
    vy_com(i) = -L*((w_kr(i) +w_hr(i))*sin(th_kr(i) + th_hr(i)) +w_hr(i)*sin(th_hr(i)));
    
    ax_com(i) =  L *((acc_kr(i) + acc_hr(i))*cos(th_kr(i))  -(w_kr(i)+w_hr(i))^2*sin(th_kr(i))  +acc_hr(i)*cos(th_hr(i))  -w_hr(i)^2*sin(th_hr(i)) );
    ay_com(i) = -L *((acc_kr(i) + acc_hr(i))*sin(th_kr(i))  +(w_kr(i)+w_hr(i))^2*cos(th_kr(i))  +acc_hr(i)*sin(th_hr(i))  +w_hr(i)^2*cos(th_hr(i)) );
    
    i = i + 1;
    
end

%}


%{
%% Now assuming that at this point the right leg comes off the ground and the body
% moves to left leg on single support phase
while( th_hl(i-1) >= 0 )
    th_hl(i) = (35.5038 + ((0 - 35.5038)/((12+12)-12))*(i-12))*(pi/180);
    
    %code to calculate the th_kl
    w_hl(i) = (th_hl(i) - th_hl(i-1))/t_step;
    acc_hl(i) = (w_hl(i) - w_hl(i-1))/t_step;
    
    w_kl(i) = w_kl(i-1) + acc_kl(i-1)*t_step;
    th_kl(i) = th_kl(i-1) + w_kl(i-1)*t_step;
    
    if th_kl(i) > (360*pi/180)
        th_kl(i) = (360*pi/180);
    end
   
    acc_kl(i) = ( (sin(th_kl(i) + th_hl(i)) + sin(th_hl(i)))*( -acc_hl(i)*sin(th_kl(i) + th_hl(i)) - ((w_kl(i) + w_hl(i))^(2))*(cos(th_kl(i) + th_hl(i))) - acc_hl(i)*sin(th_hl(i)) - ((w_hl(i))^(2))*cos(th_hl(i)) + g/L )...
              - (cos(th_kl(i) + th_hl(i)) + cos(th_hl(i)))*( acc_hl(i)*cos(th_kl(i) + th_hl(i)) - ((w_kl(i) + w_hl(i))^(2))*(sin(th_kl(i) + th_hl(i))) - acc_hl(i)*cos(th_hl(i)) + ((w_hl(i))^(2))*sin(th_hl(i)) ) )/ ( cos(th_kl(i) + th_hl(i))*(cos(th_kl(i) + th_hl(i)) + cos(th_hl(i))) + sin(th_kl(i) + th_hl(i))*( sin(th_kl(i) + th_hl(i)) + sin(th_hl(i)) ) );
    
    
    x_com(i) =  0.28 -  L*( sin(- th_kl(i)+ th_hl(i)) + sin(th_hl(i)));
    y_com(i) = L*( cos( th_kl(i)+ th_hl(i)) + cos(th_hl(i)));
    
    vx_com(i) =  L*((w_kl(i) +w_hl(i))*cos(th_kl(i) + th_hl(i)) + w_hl(i)*cos(th_hl(i)));
    vy_com(i) = -L*((w_kl(i) +w_hl(i))*sin(th_kl(i) + th_hl(i)) +w_hl(i)*sin(th_hl(i)));
    
    ax_com(i) =  L *((acc_kl(i) + acc_hl(i))*cos(th_kl(i))  -(w_kl(i)+w_hl(i))^2*sin(th_kl(i))  +acc_hl(i)*cos(th_hl(i))  -w_hl(i)^2*sin(th_hl(i)) );
    ay_com(i) = -L *((acc_kl(i) + acc_hl(i))*sin(th_kl(i))  +(w_kl(i)+w_hl(i))^2*cos(th_kl(i))  +acc_hl(i)*sin(th_hl(i))  +w_hl(i)^2*cos(th_hl(i)) );
    
    i = i + 1;
    
end


%}

%Left leg after contact with the ground
while(th_hl > 0)
    %Using formula y - y1 = m(x - x1)
    th_hl(i) = (35.5038 + ((0 - 35.5038)/((12+12)-12))*(i-12))*(pi/180);
    w_hl(i) = (th_hl(i) - th_hl(i-1))/t_step;
    acc_hl(i) = (w_hl(i) - w_hl(i-1))/t_step;
    
    %Since we are taking the anti-clockwise direction as positive the th_kl
    %varies from (360-(180 - 120)) = (360 - 60) to (360 - (180 - 108.9924))
    %= (360 - 71.0076)
    th_kl(i) = (288.94 + ((360 - 288.9924)/((12+12)-12))*(i-12))*(pi/180);
    w_kl(i) = (th_kl(i) - th_kl(i-1))/t_step;
    acc_kl(i) = (w_kl(i) - w_kl(i-1))/t_step;
    
    x_com(i) =  0.2829;  %  L*( sin(- th_kl(i)+ th_hl(i)) + sin(th_hl(i)));
    y_com(i) = L*( cos( th_kl(i)+ th_hl(i)) + cos(th_hl(i))) + H;
    
    vx_com(i) =  L*((w_kl(i) +w_hl(i))*cos(th_kl(i) + th_hl(i)) + w_hl(i)*cos(th_hl(i)));
    vy_com(i) = -L*((w_kl(i) +w_hl(i))*sin(th_kl(i) + th_hl(i)) +w_hl(i)*sin(th_hl(i)));
    
    ax_com(i) =  L *((acc_kl(i) + acc_hl(i))*cos(th_kl(i))  -(w_kl(i)+w_hl(i))^2*sin(th_kl(i))  +acc_hl(i)*cos(th_hl(i))  -w_hl(i)^2*sin(th_hl(i)) );
    ay_com(i) = -L *((acc_kl(i) + acc_hl(i))*sin(th_kl(i))  +(w_kl(i)+w_hl(i))^2*cos(th_kl(i))  +acc_hl(i)*sin(th_hl(i))  +w_hl(i)^2*cos(th_hl(i)) );
    
    i = i+1;
    
end


%% We assume angles of right leg vary linearly till the final position
for k = (j-1):i
    th_hr(k) = th_hr(j-1) + (((-30*pi/180) - th_hr(j-1))/(i - (j-1)))*(k - (j-1));
    w_hr(k) = (th_hr(k) - th_hr(k-1))/t_step;
    acc_hr(k) = (w_hr(k) - w_hr(k-1))/t_step;
    
    th_kr(k) = th_kr(j-1) + (((60*pi/180) - th_kr(j-1))/(i - (j-1)))*(k - (j-1));
    w_kr(k) = (th_kr(k) - th_kr(k))/t_step;
    acc_kr(k) = (w_kr(k) - w_kr(k))/t_step;
end



figure(1)
x_offset = 0; y_offset = 0;
 %change 1:1 to 1:3
 x_steps = [0, D_1, D_1, D_0+2*D_1, D_0+2*D_1,  2*D_0+3*D_1, 2*D_0+3*D_1, 3*D_0+4*D_1  ];
 y_steps = [0,   0,   H,         H,       2*H,          2*H,         3*H,         3*H  ];
for z = 1:3
    
    for s = 1:i-1
    plot(x_steps,y_steps);
    hold on;
    line(x_steps, y_steps,'Color','b','Linewidth',2)
    
    plot(x_com(s) + x_offset, y_com(s)+y_offset,'ob','MarkerSize',9)
    axis([0 2 0 2])
    hold on;
    
    %right knee position
    x_kr = x_com(s)+x_offset - L*sin(th_hr(s));
    y_kr = y_com(s)+y_offset - L*cos(th_hr(s));
    
    plot(x_kr, y_kr, 'or','MarkerSize',9);
    
    %Drawing line from the hip to right knee
    line([x_com(s)+x_offset x_kr],[y_com(s)+y_offset y_kr],'Color','k','Linewidth',2);
    
    
    %right foot position
    x_fr = x_kr - L*sin(th_hr(s)+ th_kr(s));
    y_fr = y_kr - L*cos(th_hr(s)+ th_kr(s));
    
    %Drawing line from the knee to the right foot
    line([x_kr x_fr],[y_kr y_fr],'Color','k','Linewidth',2);
    
    %left knee position
     x_kl = x_com(s)+x_offset + L*sin(th_hl(s));
     y_kl = y_com(s)+y_offset - L*cos(th_hl(s));
    
    plot(x_kl, y_kl, 'or','MarkerSize',9);
    
    %drawing line from the hip to left knee
    line([x_com(s)+x_offset x_kl],[y_com(s)+y_offset y_kl],'Color','k','Linewidth',2);
    
    x_fl = x_kl + L*sin(th_hl(s)+ th_kl(s));
    y_fl = y_kl - L*cos(th_hl(s)+ th_kl(s));
    
   
    
    %{
    %left foot position
    if s > 14 && s < 19
        x_fl = 0.75 + x_offset; y_fl = 0;
        %x_fl = x_kl - L*sin(th_hl(s)+ th_kl(s));
        %y_fl = y_kl - L*cos(th_hl(s)+ th_kl(s));
    else
        x_fl = x_kl + L*sin(th_hl(s)+ th_kl(s));
        y_fl = y_kl - L*cos(th_hl(s)+ th_kl(s));
    end
    %}
    
    %Drawing line from left knee to the left foot
    line([x_kl x_fl],[y_kl y_fl],'Color','k','Linewidth',2);
    
    
    pause(0.25);
    clf;
    end
x_offset = x_offset + x_com(s);
y_offset = y_offset + H;
end




  




