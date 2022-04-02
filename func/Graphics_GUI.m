%Graphics_GUI

screen_size = get(0,'ScreenSize');
win_startx = round(.2*screen_size(3)/2);
win_dimx = screen_size(3)-2*win_startx;
win_starty = round(.2*screen_size(4)/2);
win_dimy = screen_size(4)-2*win_starty;

h0 = figure('Name','SIMURV simulation results', 'Position', [win_startx win_starty win_dimx win_dimy]);

but_dimx = 80;
but_dimy = 40;
but_startx = 10;
but_starty = win_dimy - but_dimy - 10;
h1 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'close all', 'callback', 'close');

but_starty = win_dimy - 2*but_dimy - 2*10;
h2 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'accel', 'callback', 'PlotAccelerations(t,dzita)');

but_starty = win_dimy - 3*but_dimy - 3*10;
h3 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'velocities', 'callback', 'PlotVelocities(t,zita)');

but_starty = win_dimy - 4*but_dimy - 4*10;
h4 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'positions', 'callback', 'PlotConfiguration2(t,eta,q,eta_d,q_d)');

but_starty = win_dimy - 5*but_dimy - 5*10;
h5 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'forces', 'callback', 'PlotForces(t,tau)');

but_starty = win_dimy - 6*but_dimy - 6*10;
h6 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'end-eff', 'callback', 'PlotEndEffector(t,eta_ee1_d,eta_ee1,eta_ee_quat_d,eta_ee_quat)');

but_starty = win_dimy - 7*but_dimy - 7*10;
h7 = uicontrol('Parent', h0, 'Position', [but_startx but_starty but_dimx but_dimy], ...
    'Tag','Pushbutton1', 'string', 'tasks', 'callback', 'PlotTasks(t,sigma_tilde_a,sigma_tilde_b,sigma_tilde_c)');


