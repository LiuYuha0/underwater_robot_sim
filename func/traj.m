function point = traj(t)

point = zeros(3,1);
% x = pi*1.092/18*sin(pi/18*t);
% y = 0.36;
% z = pi*1.092/18*cos(pi/18*t);

if t <= 1800
    x = 1.092*(1-cos(pi/18*t))-0.3;
    y = 0.36;
    z = 1.092*sin(pi/18*t)+0.1;

    point(1,1) = x;
    point(2,1) = y;
    point(3,1) = z;
else
    point(1,1) = -0.3;
    point(2,1) = 0.36;
    point(3,1) = 1.192;
end