% Dynamics Simulation of Rover Operating in Snow
% by Austin Lines
% Last updated: 2021-01-15
% Referenced in Chapters 6 & 7 of PhD Thesis

% Shear and normal stress of a rigid tire
% Equations from Wong
% Iterative determination of theta1 given slip ratio
% Computation of resistance and total torque
% Use forces in 8 DOF model

% SHORTCOMINGS OF SIMULATION
% - no lateral bulldozing resistance forces
% - resistance to lateral motion is only based on M_res (0.3) term in f_of_x8
% - negative slips are automatically forced to 0

%%
clear
clear global


dbstop('in',mfilename,'at','1311','if','debug==1')

global g Lf Lr hcg Rw mg m muf mur ms LfpLr Iwf Iwr e track_f tr ho Ixxs Ixzs Ixz Izz mhur mhuf ... 
    hfLrp hrLfm mshcg mshcgg mshchpho tfo2 tro2 Fzf_1 Fzr_1 beta gam ho6 ho8 ho2 ho3 Bw;

%% SETTINGS

% NUMBER OF MONITORS ON WHICH RESULTS WILL BE DISPLAYED
% 1 - Single
% 0 - Double
singleMonitor = 1;

% FORCE-SLIP CURVE
% 1 - ON
% 0 - OFF
slipCalc = 1;

% DEBUG
% 1 - ON
% 0 - OFF
debug = 0;

% PARAMETRIC STUDY
% 1 - ON
% 0 - OFF
parametric = 0;

% PRESSURE-SINKAGE MODEL
% 1 - Bekker model
% 2 - Preston-Thomas model
% 3 - Snow
% 4 - Reece
model = 1;

% SHEAR STRESS-SHEAR DISPLACEMENT MODEL
% 1 - Fresh snow (no hump) (Wong - Theory of Ground Vehicles - pg. 144, A.)
% 2 - Frozen snow (hump)   (Wong - Theory of Ground Vehicles - pg. 149, C.)
shear_model = 2;

% PATCH
% 0 - max pressure at theta = 0, no constant pressure patch
% 1 - constant pressure contact patch
% 2 - Wong-Reece - maximum pressure somewhere in middle of wheel
patch = 2;
z_const = .056;         % meters

% SLIP-SINKAGE
% 0 - OFF
% 1 - Lyasko's model of (1 + i)/(1 - 0.5*i)
% 2 - Lyasko's model of z = z0 + i*S*z0
% 3 - Ding's Model
slip_sink = 2;
S = 1;
% S = 0.6/0.33;

if slip_sink == 2
    % FLATTEN AT HIGH SINKAGE
    % 0 - OFF
    % 1 - Flatten
    flatten = 1;
end
    
% BULLDOZING RESISTANCE
% 0 - OFF
% 1 - ON
bulldozing = 1;

% ROBOT
% 1 - Default
% 2 - Cool Robot
% 3 - FrostyBoy
robot = 3;

% OPEN OR CLOSED LOOP
% 1 - Open-loop mode
% 2 - Closed-loop mode
open_closed = 1;

% TERRAIN DAMPER
% 0 - OFF
% 1 - ON
damp_terrain = 1;
b_terrain = 1/200;
sink_max = .001;

% THETA2 CONTROL
% 0 - OFF
% 1 - ON
% 2 - Ishigami equation
slip_theta2 = 0;
lambda = .11;        % Lambda for Ishigami equation

b_eff = .01;

ss_nom = 0.1;   % nominal stepsize for integration of shear and normal stress

%% List of common terrain parameters
% To use a set of terrain parameters below, copy and paste the terrain
% parameters into the next section

run = false;
if run == true
    
    % Clayey soil
    phi = 13*pi/180;    % angle of internal shearing resistance
    c = 4.14;           % cohesion of soil
    Ks =0.01;           % shear deformation modulus
    kc = 13.19;
    kphi = 692.15;      % pressure sinkage parameters
    n = 0.5;
    c1 = 0.18;
    c2 = 0.32;          % these are probably not needed for deformable tire

    % Dry sand from Iagnemma et al. 2004
    phi = 30*pi/180;    % angle of internal shearing resistance
    c = 1.0;            % cohesion of soil  kPa 
    Ks =0.025;          % shear deformation modulus (m)
    kc = 0.9;           % kPa/m^n-1 
    kphi = 1523.4;      % pressure sinkage parameters  kPa/m*n
    n = 1.1;
    c1 = 0.3;
    c2 = 0.4;           % these are probably not needed for deformable tire

    % Sandy loam  from Wong p. 136
    phi = 33.7*pi/180;  % angle of internal shearing resistance
    c = 3.3;            % cohesion of soil  kPa 
    Ks = 0.025;          % shear deformation modulus (m)
    kc = 74.6;          % kPa/m^n-1 
    kphi = 2080;        % pressure sinkage parameters  kPa/m*n
    n = 1.1;
    c1 = 0.3;
    c2 = 0.4;           % these are probably not needed for deformable tire

    % Clayey soil
    phi = 13*pi/180;    % angle of internal shearing resistance
    c = 4.14;           % cohesion of soil
    Ks = 0.01;           % shear deformation modulus
    kc = 13.19;
    kphi = 692.15;      % pressure sinkage parameters
    n = 0.5;
    c1 = 0.18;
    c2 = 0.32;          % these are probably not needed for deformable tire

    % Snow from Wong p. 136 (Sweden)
    phi = 20.7*pi/180;  % angle of internal shearing resistance
    c = 6;              % cohesion of soil  kPa 
    Ks =0.02;           % shear deformation modulus (m)
    kc = 10.55;         % kPa/m^n-1 
    kphi = 66.08;       % pressure sinkage parameters  kPa/m*n
    n = 1.44;
    c1 = 0.3;
    c2 = 0.4;           % these are probably not needed for deformable tire

    % Snow from Wong p.136 (Harrison)
    phi = 23.2*pi/180;  % angle of internal shearing resistance
    c = .62;            % cohesion of soil  kPa 
    Ks =0.02;           % shear deformation modulus (m)
    kc = 2.49;          % kPa/m^n-1 
    kphi = 245.9;       % pressure sinkage parameters  kPa/m*n
    n = 1.6;
    c1 = 0.3;
    c2 = 0.4;           % these are probably not needed for deformable tire

    % Snow from Wong p.136 (US)
    phi = 19.7*pi/180;  % angle of internal shearing resistance
    c = 1.03;           % cohesion of soil  kPa 
    Ks = 0.02;          % shear deformation modulus (m)
    kc = 4.37;          % kPa/m^n-1 
    kphi = 196.72;      % pressure sinkage parameters  kPa/m*n
    n = 1.6;
    c1 = 0.3;
    c2 = 0.4;           % these are probably not needed for deformable tire

    % Snow from Lever
    phi = 20.7*pi/180;  % angle of internal shearing resistance
    c = .5;             % cohesion of soil  kPa 
    Ks = 0.02;          % shear deformation modulus (m)
    kc = 90;            % kPa/m^n-1 
    kphi = 90;          % pressure sinkage parameters  kPa/m*n
    n = 1.6;
    c1 = 0.3;
    c2 = 0.4;           % these are probably not needed for deformable tire

    % Lean clay
    phi = 20*pi/180;    % angle of internal shearing resistance
    c = 68.95;          % cohesion of soil
    K = 0.0001;         % shear deformation modulus
    Ks = K;
    kc = 16.42;
    kphi = 1724.69;     % pressure sinkage parameters
    n = 0.2;
    c1 = 0.43;
    c2 = 0.32;

end
%% Terrain parameters



% WONG, PRESTON-THOMAS
zm = .54;           % meters

km = 40;            % kPa/m
c1 = 0.18;          % Wong and Reece 1967, loose sand
c2 = 0.32;          % Wong and Reece 1967, loose sand

% SNOW
kp1 = 16.3;         % kPa
kp2 = 0;            % kPa/m
kz1 = .033;         % meters
kz2 = 0;            % meter^2

% REECE
% kc = 50;
% n = 2;
% Ks = 0.061;

% WONG, shear stress-shear displacement, Model C
Kw = 0.022;    % 0.022 standard     %0.055 results in zero drawbar pull at zero slip given above conditions
Kr = 0.66;        % 0.66 standard

% Snow from Wong p.136 (Harrison)
phi_d = 23;
phi = phi_d*pi/180;  % angle of internal shearing resistance
c = 0.75;            % cohesion of soil  kPa 
Ks =0.02;           % shear deformation modulus (m)
kc = 2.49;          % kPa/m^n-1 
kphi = 245.9;       % pressure sinkage parameters  kPa/m*n
n = 1.6;
c1 = 0.3;
c2 = 0.4;           % these are probably not needed for deformable tire
    



% Soil bearing capacity factors
[Kc,Ky] = soil_factors(phi);
gamma = 1961;        % N/m^3

%% Vehicle parameters

if robot == 1   % Default
    g = 9.81;               % Gravity constant, m/s^2
    mv = 9.6071655;         % Vehicle mass, kg
    m = mv;
    Rw = 5/100;             % wheel radius in m
    mw = 0.28;              % Wheel mass, kg
    Iw = mw*Rw^2;           % Wheel moment of inertia, kg m^2
    Iwf = Iw;               % Front wheel moment of interia, kg m^2
    Iwr = Iw;               % Rear wheel moment of interia, kg m^2
    muf = mw*2;             % Total front wheel mass, kg
    mur = mw*2;             % Total rear wheel mass, kg
    ms = m - muf - mur;     % Vehicle body mass (w/o wheels), kg
    tr = 0.43722;           % Track width, m
    track_f = tr;           % Front track width, m, non-trivial to implement as not =
    Lr = 0.184361;          % Distance from CG to rear axle centerline, m
    Lf = 0.216237;          % Distance from CG to front axle centerline
    LfpLr = Lr + Lf;        % Total vehicle length between axle centers, m
    hcg = 0.05;             % Height from wheel axis to vehicle CG
    hf = 0.05;              % Height of front roll center w.r.t. ground, m
    hr = 0.05;              % Height of rear roll center w.r.t. ground, m
    huf = Rw;               % Front unsprung mass height (wheel center) w.r.t. ground, m
    hur = Rw;               % Rear unsprung mass height (wheel center) w.r.t. ground, m
    ho = (hf*Lr + hr*Lf)/LfpLr;
    e = 0;                  % Horizontal distance from body CG to total vehicle CG
    Izz = .23687393;        % Z-axis (yaw) moment of inertia, kg*m^2
    mhur = mur*hur;
    mhuf = muf*huf;
    hfLrp = hf*(Lr + e)/LfpLr;
    hrLfm = hr*(Lf - e)/LfpLr;
    mshcg = ms*hcg;
    mshcgpho = ms*(hcg + ho);
    mshcgg = mshcg*g;
    tfo2 = track_f/2;
    tro2 = tr/2;
    mg = m*g;               % Total vehicle normal force Newtons
    Fz = mv*g;              % Total vehicle normal force Newtons
    
    % Static compontent of forces: Fzf_1 and Fzr_1
    Fzf_1 = Lr*m*9.81/(Lf + Lr);
    Fzr_1 = Lf*m*9.81/(Lf + Lr);
    Fzfl = (Lr*m*9.81/(Lf + Lr))/2; %Front left normal force, N
    Fzfr = (Lr*m*9.81/(Lf + Lr))/2; %Front right normal force, N
    Fzrl = (Lf*m*9.81/(Lf + Lr))/2; %Rear left normal force, N
    Fzrr = (Lf*m*9.81/(Lf + Lr))/2; %Rear right normal force, N
    
    Bw = 0.05; % damping coefficient for tires

elseif robot == 2 % Cool Robot
    g = 9.81;           % Gravity constant, m/s^2
    mv = 60;            % Vehicle mass, kg
    m = mv;
    Rw = .4572/2;       % Wheel radius in m (Cool Robot)
    b = 0.127;          % Wheel width (Cool Robot)
    mw = 11/2.2;        % Wheel mass, kg
    Iw = mw*Rw^2;       % Wheel moment of inertia, kg m^2
    Iwf = Iw;           % Front wheel moment of interia, kg m^2
    Iwr = Iw;           % Rear wheel moment of interia, kg m^2
    muf = mw*2;         % Total front wheel mass, kg
    mur = mw*2;         % Total rear wheel mass, kg
    ms = m - muf - mur; % Vehicle body mass (w/o wheels), kg
    tr = .9745;         % Track width, m
    track_f = tr;       % Front track width, m, non-trivial to implement as not =
    Lr = .3155;         % Distance from CG to rear axle centerline, m
    Lf = .3155;         % Distance from CG to front axle centerline, m
    LfpLr = Lr + Lf;    % Total vehicle length between axle centers, m
    hcg = 0.05;         % Height from wheel axis to vehicle CG
    hf = 0.05;          % Height of front roll center w.r.t. ground, m
    hr = 0.05;          % Height of rear roll center w.r.t. ground, m
    huf = Rw;           % Front unsprung mass height (wheel center) w.r.t. ground, m
    hur = Rw;           % Rear unsprung mass height (wheel center) w.r.t. ground, m
    ho = (hf*Lr + hr*Lf)/LfpLr;
    e = 0;              % Horizontal distance from body CG to total vehicle CG
    Izz = 11.4586;        % Z-axis (yaw) moment of inertia, kg*m^2
    mhur = mur*hur;
    mhuf = muf*huf;
    hfLrp = hf*(Lr + e)/LfpLr;
    hrLfm = hr*(Lf - e)/LfpLr;
    mshcg = ms*hcg;
    mshcgpho = ms*(hcg + ho);
    mshcgg = mshcg*g;
    tfo2 = track_f/2;
    tro2 = tr/2;
    mg = m*g;               % Total vehicle normal force Newtons
    Fz = mv*g;              % Total vehicle normal force Newtons
    
    % Normal forces
    Fzf_1 = Lr*m*9.81/(Lf + Lr);
    Fzr_1 = Lf*m*9.81/(Lf + Lr);
    Fzfl = (Lr*m*9.81/(Lf + Lr))/2; %Front left normal force, N
    Fzfr = (Lr*m*9.81/(Lf + Lr))/2; %Front right normal force, N
    Fzrl = (Lf*m*9.81/(Lf + Lr))/2; %Rear left normal force, N
    Fzrr = (Lf*m*9.81/(Lf + Lr))/2; %Rear right normal force, N
    
    Bw = 0.0005;        % Damping coefficient for tires
    
elseif robot == 3 % FrostyBoy Robot
    g = 9.81;           % Gravity constant, m/s^2
    mv = 90;            % Vehicle mass, kg
    m = mv;
    Rw = .265;          % Wheel radius in m
    b = .45;            % Wheel width
    mw = 13/2.2;        % Wheel mass, kg
    Iw = mw*Rw^2;       % Wheel moment of inertia, kg m^2
    Iwf = Iw;           % Front wheel moment of interia, kg m^2
    Iwr = Iw;           % Rear wheel moment of interia, kg m^2
    muf = mw*2;         % Total front wheel mass, kg
    mur = mw*2;         % Total rear wheel mass, kg
    ms = m - muf - mur; % Vehicle body mass (w/o wheels), kg
    tr = 1.14224;       % Track width, m
    track_f = tr;       % Front track width, m, non-trivial to implement as not =
    wheelbase = .579+.515;
    Lr = wheelbase/2;   % Distance from CG to rear axle centerline, m
    Lf = wheelbase/2; 	% Distance from CG to front axle centerline, m
    LfpLr = Lr + Lf;    % Total vehicle length between axle centers, m
    hcg = 0.140;        % Height from wheel axis to vehicle CG
    hf = 0.05;          % Height of front roll center w.r.t. ground, m
    hr = 0.05;          % Height of rear roll center w.r.t. ground, m
    huf = Rw;           % Front unsprung mass height (wheel center) w.r.t. ground, m
    hur = Rw;           % Rear unsprung mass height (wheel center) w.r.t. ground, m
    ho = (hf*Lr + hr*Lf)/LfpLr;
    e = 0;              % Horizontal distance from body CG to total vehicle CG
    Izz = 36.05;        % Z-axis (yaw) mome=of inertia, kg*m^2
    mhur = mur*hur;
    mhuf = muf*huf;
    hfLrp = hf*(Lr + e)/LfpLr;
    hrLfm = hr*(Lf - e)/LfpLr;
    mshcg = ms*hcg;
    mshcgpho = ms*(hcg + ho);
    mshcgg = mshcg*g;
    tfo2 = track_f/2;
    tro2 = tr/2;
    mg = m*g;           % Total vehicle weight, N
    Fz = mv*g;          % Total vehicle normal force, N

    % Normal forces
    Fzf_1 = Lr*m*9.81/(Lf + Lr);
    Fzr_1 = Lf*m*9.81/(Lf + Lr);
    Fzfl = (Lr*m*9.81/(Lf + Lr))/2; %Front left normal force, N
    Fzfr = (Lr*m*9.81/(Lf + Lr))/2; %Front right normal force, N
    Fzrl = (Lf*m*9.81/(Lf + Lr))/2; %Rear left normal force, N
    Fzrr = (Lf*m*9.81/(Lf + Lr))/2; %Rear right normal force, N
    
    % Motor parameters
    Tw_rad = -10.0507;      % Nm/(rad/sec) at wheel (from 'RP34_MotorCalcs.m')
    V_ref = 48;             % reference voltage
    w_noload = 7000;        % RPM
    w_noload_rad = 18.3260; % no load speed at wheel [rad/sec]
    gearbox_efficiency = 0.8;
    T_max = 88.64;             % 88.64 = 2.77*40*gearbox_efficiency;     
    
    Bw = 0.3269;        % Damping coefficient for tires 
    
end

%% Four Wheel Simulation

% global_permanent_modified;  % Loading our global variable list

% Fehlberg coefficients for 4th order Runge-Kutta integration. Testing
% suggests no advantage is realized from defining these coefficients here
% as opposed to in the integration script itself.
% The Fehlberg coefficients: for rk4 integration
beta  = [ [    1      0      0     0      0    0]/4
          [    3      9      0     0      0    0]/32
          [ 1932  -7200   7296     0      0    0]/2197
          [ 8341 -32832  29440  -845      0    0]/4104
          [-6080  41040 -28352  9295  -5643    0]/20520 ]';
gam = [902880  0  3953664  3855735  -1371249  277020]'/7618050;

% The following lines will define the simulation timestep and simulated duration
ts = 0.02;          
h = ts;
tmax = 3;           % # of seconds we will run the model

duration = tmax/h;  % # of seconds to run the model, divided by the time step
t = [0:h:tmax];     % A vector of the timesteps
nt = max(size(t));  % Value giving the # of timesteps, to help me follow TAURUS code
ho6 = h/6;  ho8 = h/8;
ho2 = h/2;  ho3 = h/3;


%% Initialize vectors

%Initial state vector
%          [dx, dy, r, wFL, wFR, wRL, wRR, vxfl, vyfl, vxfr, vyfr, vxrl, vyrl, vxrr, vyrr]
% [longitudinal velocity of x, and y, yaw rate, angular velocity of the four
% wheels, linear velocity of the four wheels in x and y]
X0 = 0.001*[1  0   0  1/Rw  1/Rw  1/Rw  1/Rw  1     0     1     0     1     0     1     0]';

% Initial slip and slip angle
% slip percentage
sfl = 0;        
sfr = 0;
srl = 0;
srr = 0;
% slip angle
safl = 0;       
safr = 0;
sarl = 0;
sarr = 0;

% Initial dynamic state  - this program cannot start from zero velocity
ax = 0;         % acceleration in x
ay = 0;         % acceleration in y
rdot = 0;       % yaw acceleration
vx = X0(1);
vy = X0(2);
vxa = vx;
vya = vy;
r = X0(3);
wFL = X0(4);
wFR = X0(5);
wRL = X0(6);
wRR = X0(7);

V = sqrt(vx^2 + vy^2);
x = [vx, vy, r, wFL, wFR, wRL, wRR]';

% Initialize vectors
y = zeros(nt,7);            % This matrix is based on the state w/o forces appended
slip = zeros(nt,4);         % We have slip percent at four wheels
slip_angle = slip;          % We have slip angle at four wheels
accout = zeros(nt,3);       % accout = [ax ay rdot] 
u = zeros(nt,4);            % input, u = [Tl, Tr, Tl, Tr]
yaw_angle = zeros(1,nt);
X = yaw_angle;
Y = X;
forces = zeros(nt,24);      % Four wheel Fx, Fy, Fz, M, R, T_r
if slip_sink > 0
sinkage = zeros(nt,15);
else
    sinkage = zeros(nt,11);
end
K_store = zeros(21,8*nt);   % Storing gains for steady state analysis
bigeps = eps*1E9;           % Using floating point relative accuracy command to control accuracy
Tfl = 0;                    % Front left torque
Tfr = 0;                    % Front right torque
Trl = 0;                    % Rear left torque
Trr = 0;                    % Rear right torque
u(1,:) = [Tfl Tfr Trl Trr]; % Wheel torque commands
y(1,:) = x'; 
slip(1,:) = [sfl sfr srl srr]; % Slip
slip_angle(1,:) = [safl safr sarl sarr];
accout(1,:) = [ax ay rdot]; % Acceleration terms
acchat = accout(1,:);
yaw_angle(1) = 0;
X(1) = 0;
Y(1) = 0;
w = zeros(1,8);


% Defining wheel force vector
% Fx Fy Fz Mz
% There are four columns for each force 24 total  Fx, Fy, Fz, Mz, Rx, T
forces(1,:) = [0 0 0 0 0 0 0 0 Fzfl Fzfr Fzrl Fzrr 0 0 0 0 0 0 0 0 0 0 0 0];
Fy_forces(1,:) = [0 0];
Mz_force(1) = 0;
nn = round(h/0.005);
yaw_t =0;
Fxfl = 0;
Fxfr = 0;
Fxrl = 0;
Fxrr = 0;
Fyfl = 0;
Fyfr = 0;
Fyrl = 0;
Fyrr = 0;

uu = zeros(nt,4);

Tfl_prev = 0;
Tfr_prev = 0;
Trl_prev = 0;
Trr_prev = 0;

c_fl = c;
c_fr = c;
c_rl = c;
c_rr = c;

km_fl = km;
km_fr = km;
km_rl = km;
km_rr = km;

zm_fl = zm;
zm_fr = zm;
zm_rl = zm;
zm_rr = zm;

Ks_fl = Ks;
Ks_fr = Ks;
Ks_rl = Ks;
Ks_rr = Ks;

diag_flag = 0;
ex_factor = .01;

% Cohesion - random walk generator
sigma_c = 0.01;
nn = 4000;
% c_rand1 = [c (sigma_c*cumsum(randn(1,n)) + c)];
% c_rand2 = [c (sigma_c*cumsum(randn(1,n)) + c)];
% c_rand3 = [c (sigma_c*cumsum(randn(1,n)) + c)];
% c_rand4 = [c (sigma_c*cumsum(randn(1,n)) + c)];

% Preston-Thomas - random walk generator
sigma_km = 0.5;
% sigma_zm = 0.001;
% km_rand1 = [km (sigma_km*cumsum(randn(1,n)) + km)];
% km_rand2 = [km (sigma_km*cumsum(randn(1,n)) + km)];
% km_rand3 = [km (sigma_km*cumsum(randn(1,n)) + km)];
% km_rand4 = [km (sigma_km*cumsum(randn(1,n)) + km)];
% 
% zm_rand1 = [zm (sigma_zm*cumsum(randn(1,n)) + zm)];
% zm_rand2 = [zm (sigma_zm*cumsum(randn(1,n)) + zm)];
% zm_rand3 = [zm (sigma_zm*cumsum(randn(1,n)) + zm)];
% zm_rand4 = [zm (sigma_zm*cumsum(randn(1,n)) + zm)];

sigma_phi = 0.05;
phi_rand1 = [km (sigma_phi*cumsum(randn(1,nn)) + phi_d)];

rw_num = 1;
rw_num_record = ones(1,nt);
pitch = 0;
roll =   0;
roll_f = 0;
roll_r = 0;

Fzf = Fzfl + Fzfr;
Fzr = Fzrl + Fzrr;
Fx_g = 0;

diag_flag_front = 0;
diag_flag_rear = 0;
front_flag = 0;
rear_flag = 0;

excavation_fl = 0;
excavation_fr = 0;
excavation_rl = 0;
excavation_rr = 0;

vz_fl = 0;
vz_fr = 0;
vz_rl = 0;
vz_rr = 0;

sigma_km = 4;

num_per_meter = 250;
rear_diff = floor(wheelbase*num_per_meter);
rw_num_front = 1;
rw_num_rear = 1;

speed_error_fl = zeros(nt,1);
speed_error_fr = zeros(nt,1);
speed_error_rl = zeros(nt,1);
speed_error_rr = zeros(nt,1);
dist = zeros(1,nt);
v_mag = zeros(1,nt);
dist(1) = 0;
v_mag(1) = 0;


for i = 1:nt-1
     disp(i)
     

     

     
     %%% Adding random walk to terrain parameters
     if i>200
         rw_num = floor((dist(i)-dist(200))*num_per_meter)+1;
         rw_num_record(i) = rw_num;
         rw_num_front = rw_num;
         if rw_num > (rear_diff+1)
             rw_num_rear = rw_num - rear_diff;
         end
         
%          c_fl = c_rand1(rw_num_front);
%          c_fr = c_rand1(rw_num_front);
%          c_rl = c_rand1(rw_num_rear);
%          c_rr = c_rand1(rw_num_rear);
%          
%          km_fl = km_rand1(rw_num_front);
%          km_fr = km_rand1(rw_num_front);
%          km_rl = km_rand1(rw_num_rear);
%          km_rr = km_rand1(rw_num_rear);       
     end
          
        %%% Increase in k
%         if i >= 200
%             km_fl = 65;
%             km_rr = 65;
%             z_fr = z_fl;
%             z_rl = z_rr;
%             if i == 200
%                 z_fr = z_fr_orig;
%                 z_rl = z_rl_orig;
%             end
%             diag_flag_front = 1;
%             diag_flag_rear = 1;
%             front_flag = 1;
%             rear_flag = 1;
%             slip_sink = 0;
%         end
        %%%

        %%% Decrease in cohesion
%         if i >= 200
%             z_fr = z_fl;
%             z_rl = z_rr;
%             if i == 200
%                 z_fr = z_fr_orig;
%                 z_rl = z_rl_orig;
%             end
%             diag_flag_front = 1;
%             diag_flag_rear = 1;
%             front_flag = 1;
%             rear_flag = 1;
%             slip_sink = 0;
%             if c_fr > 0.5
%                 c_fr = c_fr - .01;
%             end
%             if c_rl > 0.5
%                 c_rl = c_rl - .01;
%             end
%         end
        %%%
        
        %%% Weight transfer/excavation, use minimum of each axle as         
%         if i >= 200
%             km_fl = 65;
%             km_rr = 65;
%             if i == 200
%                 z_fr = 2*z_fr_orig - z_fr;
%                 sinkage(i,2) = z_fr;
%                 z_rl = 2*z_rl_orig - z_rl;
%                 sinkage(i,3) = z_rl;
%             end
%             diag_flag_front = 1;
%             diag_flag_rear = 1;
%             front_flag = 1;
%             rear_flag = 1;
%             excavation_fl = 0;
%             excavation_fr = 1;
%             excavation_rl = 1;
%             excavation_rr = 0;
%         end
        %%%
        
    
     disp([num2str(km_fl),' km']);
     disp([num2str(c_fl),' kPa']);
     disp([num2str(sinkage(i,1)),' meters']);
     disp([num2str(rw_num_front),' random #']);
   
    vx = x(1);
    vy = x(2);
    r = x(3);
    wfl = x(4);
    wfr = x(5);
    wrl = x(6);
    wrr = x(7);
    
% Calculate torques from given voltage
V_command = 45;         % out of 100
V_offset = 9.6;         % measured from real data
accel_limit = 1000;     % RPM/sec
V_slope = (accel_limit/w_noload)*h;      % motor controller acceleration converted to (%Voltage)/(time step)

if open_closed == 1
    if i < (V_command-(100*V_offset/V_ref))/(.00286*100)
        V_applied_fl = (V_slope*(i))*V_ref + V_offset;
        V_applied_fr = (V_slope*(i))*V_ref + V_offset;
        V_applied_rl = (V_slope*(i))*V_ref + V_offset;
        V_applied_rr = (V_slope*(i))*V_ref + V_offset;
    elseif i >= (V_command-(100*V_offset/V_ref))/(.00286*100) % && i<300 %DECELERATION
        V_applied_fl = (V_command/100)*V_ref;
        V_applied_fr = (V_command/100)*V_ref;    
        V_applied_rl = (V_command/100)*V_ref;
        V_applied_rr = (V_command/100)*V_ref;    
    end

elseif open_closed == 2
    Kp = 1;
    Ki = 0.01;

    if i < (V_command-(100*V_offset/V_ref))/(.00286*100)
        V_applied_fl = (V_slope*(i))*V_ref + V_offset;
        V_applied_fr = (V_slope*(i))*V_ref + V_offset;
        V_applied_rl = (V_slope*(i))*V_ref + V_offset;
        V_applied_rr = (V_slope*(i))*V_ref + V_offset;  
    elseif i > 200
        speed_setpoint_L = 3.5;
        speed_setpoint_R = 5.8;
        speed_error_fl(i) = speed_setpoint_L - wfl;
        speed_error_fr(i) = speed_setpoint_R - wfr;
        speed_error_rl(i) = speed_setpoint_L - wrl;
        speed_error_rr(i) = speed_setpoint_R - wrr;
        speed_err_i_fl = sum(speed_error_fl(200:i))/(i-200);
        speed_err_i_fr = sum(speed_error_fr(200:i))/(i-200);
        speed_err_i_rl = sum(speed_error_rl(200:i))/(i-200);
        speed_err_i_rr = sum(speed_error_rr(200:i))/(i-200);
        correction_fl = Kp*speed_error_fl(i) + Ki*speed_err_i_fl;
        correction_fr = Kp*speed_error_fr(i) + Ki*speed_err_i_fr;
        correction_rl = Kp*speed_error_rl(i) + Ki*speed_err_i_rl;
        correction_rr = Kp*speed_error_rr(i) + Ki*speed_err_i_rr;
        if abs(correction_fl) > V_slope*V_ref
            V_applied_fl = V_applied_fl + V_slope*V_ref*sign(correction_fl);
        else
            V_applied_fl = V_applied_fl + correction_fl;
        end
        if abs(correction_fr) > V_slope*V_ref
            V_applied_fr = V_applied_fr + V_slope*V_ref*sign(correction_fr);
        else
            V_applied_fr = V_applied_fr + correction_fr;
        end
        if abs(correction_rl) > V_slope*V_ref
            V_applied_rl = V_applied_rl + V_slope*V_ref*sign(correction_rl);
        else
            V_applied_rl = V_applied_rl + correction_rl;
        end
        if abs(correction_rr) > V_slope*V_ref
            V_applied_rr = V_applied_rr + V_slope*V_ref*sign(correction_rr);
        else
            V_applied_rr = V_applied_rr + correction_rr;
        end  
    end
end

T = 0.001;
s = tf('s');

Kfl = Tw_rad*(wfl - w_noload_rad*V_applied_fl/V_ref);
Kfr = Tw_rad*(wfr - w_noload_rad*V_applied_fr/V_ref);
Krl = Tw_rad*(wrl - w_noload_rad*V_applied_rl/V_ref);
Krr = Tw_rad*(wrr - w_noload_rad*V_applied_rr/V_ref);

mtr_c = 1/(T*s+1);
mtr_d = c2d(mtr_c,h);
[num,den] = tfdata(mtr_d);
SD_motor = 2.26;

Tfl = normrnd((num{1}(2)*Kfl - den{1}(2)*Tfl_prev),SD_motor);
Tfr = normrnd((num{1}(2)*Kfr - den{1}(2)*Tfr_prev),SD_motor);
Trl = normrnd((num{1}(2)*Krl - den{1}(2)*Trl_prev),SD_motor);
Trr = normrnd((num{1}(2)*Krr - den{1}(2)*Trr_prev),SD_motor);

if V_applied_fl <= 0
    Tfl = 0;
end
if V_applied_fr <= 0
    Tfr = 0;
end
if V_applied_rl <= 0
    Trl = 0;
end
if V_applied_rr <= 0
    Trr = 0;
end

if Tfl > T_max
    Tfl = T_max;
end
if Tfl < 0
    Tfl = 0;
end
if Tfr > T_max
    Tfr = T_max;
end
if Tfr < 0
    Tfr = 0;
end
if Trl > T_max
    Trl = T_max;
end
if Trl < 0
    Trl = 0;
end
if Trr > T_max
    Trr = T_max;
end
if Trr < 0
    Trr = 0;
end

Tfl_prev = Tfl;
Tfr_prev = Tfr;
Trl_prev = Trl;
Trr_prev = Trr;

u(i,:) = [Tfl Tfr Trl Trr];

% Calculate speeds at each tire
% These calculations incorporate x and y velocity components and yaw (r)
% times moment arm (either track_f, tr for front, rear track width or Lf, Lr for
% front, rear distance from axle line to cg.
vxfl = vx - r*track_f/2;
vyfl = vy + r*Lf;
vxfr = vx + r*track_f/2;
vyfr = vy + r*Lf;
vxrl = vx - r*tr/2;
vyrl = vy - r*Lr;
vxrr = vx + r*tr/2;
vyrr = vy - r*Lr;

z_offset = 0;

%% Calculate slip and slip angle

[alpha_fl, sigma_fl] = get_slip_v3_lines(Rw*wfl,vxfl,vyfl,Tfl);
disp([num2str(sigma_fl),'%'])
[alpha_fr, sigma_fr] = get_slip_v3_lines(Rw*wfr,vxfr,vyfr,Tfr);
[alpha_rl, sigma_rl] = get_slip_v3_lines(Rw*wrl,vxrl,vyrl,Trl);
[alpha_rr, sigma_rr] = get_slip_v3_lines(Rw*wrr,vxrr,vyrr,Trr);

stepsize = ss_nom;
theta1 = 0.18;

W_est = 0;

if debug == 1
    close all
end

diag_flag_mem = diag_flag;

%% Front Left Wheel
recalc_flag = 0;

if i>1
    if z_fl < sinkage(i,1)
%         diag_flag = 2;
    elseif z_fr < sinkage(i,2)
%         diag_flag = 1;
    end
end

if i>1
if front_flag == 0
    diag_flag = 0;
elseif front_flag == 1
    diag_flag = diag_flag_front;
end
end

while diag_flag == 0 || diag_flag == 1 || diag_flag == 2 || diag_flag == 3
min_flag = 0;
W = Fzfl;

if debug == 1
    clear W_error_g theta1_g thetam_g j
    j=1;                                % DEBUG
end

if diag_flag == 3
    W = Fzf - Fzfr_diag;
    if W < 0
        W = 0;
        Fzfl = W;
        Fzfr = Fzf;
    else
        Fzfl = W;
        Fzfr = Fzfr_diag;
    end
end

counter = 0;
stepsize = ss_nom;

if diag_flag == 2
    if z_fl <= 0
        Fzfl_diag = 0;
        theta1 = 0;
        thetam = 0;
        theta2 = 0;
        diag_flag = 2;
    else
    
    theta1 = acos(1-z_fl/Rw);
    
    if patch == 0
        thetam = 0;
    elseif patch == 1
        thetam = real(acos((z_const/Rw)+cos(theta1)));
    elseif patch == 2
        thetam = (c1+c2*sigma_fl/100)*theta1;
        theta2 = deg2rad(-15);
        if (sigma_fl/100 > .5) && (slip_theta2 == 1)
            theta2 = deg2rad(lambda*((sigma_fl/100)-.5)-15);
        elseif slip_theta2 == 2
            theta2 = -acos(1-(lambda*(1 - cos(theta1))));
        else
            theta2 = deg2rad(-15);
        end
    end
    
    W1 = integral(@(xt)normal_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),thetam,theta1');
    W3 = integral(@(xt)shear_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfr,min_flag,shear_model,z_offset),thetam,theta1);
    if patch == 0
        W2 = 0;
        W4 = 0;
    elseif patch == 1
        if model == 2
            W2 = km*zm*(-log(1-(z_const/zm)));
        else
            W2 = (kc + kphi*b)*((Rw/b).^n)*((1 - cos(theta1)).^n);
        end
    elseif patch == 2
        W2 = integral(@(xt)normal_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),theta2,thetam);
        W4 = integral(@(xt)shear_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfr,min_flag,shear_model,z_offset),theta2,thetam);
    end
    
    Fzfl_diag = Rw*b*(W1+W2+W3+W4)*1000;
    if Fzfl_diag < 0
        Fzfl_diag = 0;
    end
    diag_flag = 2;
        
    %%% Compute slip-sinkage - alines 2020-07-14
    z0 = (1 - cos(theta1))*Rw;
    if slip_sink == 1
        Kss = (1+sigma_fl/100)/(1-(0.5*sigma_fl/100));
        z_fl_slip = Kss*z0;
    elseif slip_sink == 2      
        if (sigma_fl/100)>.3 && flatten==1
            factor = ((S*.5)./(1+exp(-8*((sigma_fl/100)-.25))));
        else
            factor = S*(sigma_fl/100);
        end
        if excavation_fl == 1
            z_fl_slip = z0 - factor*z0;
        else
            z_fl_slip = z0 + factor*z0;
        end
        if z_fl_slip < 0
            z_fl_slip = 0;
        end            
    elseif slip_sink == 3
        z_fl_slip = z0 - ding_slip_sink(thetam,theta2,phi,sigma_fl/100)*((wfl*h)/(thetam - theta2));
    end
    if slip_sink > 0
    theta_slip = acos(1 - (z_fl_slip/Rw));
    end
    
    %%% Don't allow immediate increase in sinkage, limit sinkage
    z_fl = Rw*(1 - cos(theta1));
    if i>1
        if slip_sink == 0
            z_fl_slip = z_fl;
        end
        vz_fl = (z_fl_slip - sinkage(i,1))/h;
        z_fl_orig = z_fl;
        if damp_terrain == 1
            sink_add = b_terrain*vz_fl;
            if abs(sink_add) > sink_max
                sink_add = sink_max*(sign(sink_add));
            end
            z_fl = sinkage(i,1) + sink_add;
            theta1 = acos(1-z_fl/Rw);
        else
            if abs(z_fl_slip - sinkage(i,1)) > (sink_max)
                z_fl = sinkage(i,1) + sink_max*sign(vz_fl);
                theta1 = acos(1-z_fl/Rw);
            else
                z_fl = Rw*(1 - cos(theta_slip));
                theta1 = acos(1-z_fl/Rw);
            end
        end
    end
    end
    
else          %%% (diag_flag ~= 2)

    while (W_est > (W + 0.5)/1000 )   || (W_est <  (W - 0.5)/1000)
        counter = counter+1;
        if patch == 0
            thetam = 0;
        elseif patch == 1
            thetam = real(acos((z_const/Rw)+cos(theta1)));
        elseif patch == 2
            thetam = (c1+c2*sigma_fl/100)*theta1;
            if (sigma_fl/100 > .5) && (slip_theta2 == 1)
                theta2 = deg2rad(lambda*((sigma_fl/100)-.5)-15);
            elseif slip_theta2 == 2
                theta2 = -acos(1-(lambda*(1 - cos(theta1))));
            else
                theta2 = deg2rad(-15);
            end
        end
        
        % Find W by integrating normal stress and shear stress, break into two regions theta_1 to theta_m and theta_m to theta_2
        W1 = integral(@(xt)normal_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),thetam,theta1');
        W3 = integral(@(xt)shear_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),thetam,theta1);
        if patch == 0
            W2 = 0;
            W4 = 0;
        elseif patch == 1
            if model == 2
                W2 = km*zm*(-log(1-(z_const/zm)));
            else
                W2 = (kc + kphi*b)*((Rw/b).^n)*((1 - cos(theta1)).^n);
            end
            W4 = 0;
        elseif patch == 2
            W2 = integral(@(xt)normal_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),theta2,thetam);
            W4 = integral(@(xt)shear_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'normal',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),theta2,thetam);
        end
        
        if patch == 1
            W_est = Rw*b*(W1+W3) + W2*b*sin(thetam)*2;
        else
            W_est = Rw*b*(W1+W2+W3+W4);
        end
        W_error = (W/1000 - W_est);
        
        if debug == 1
            W_error_g(j) = W_error;             % DEBUG
            theta1_g(j) = rad2deg(theta1);      % DEBUG
            thetam_g(j) = rad2deg(thetam);      % DEBUG
            theta2_g(j) = rad2deg(theta2);      % DEBUG
            j=j+1;                              % DEBUG
        end
        
        if counter > 200
            stepsize = stepsize/2;
            counter = 0;
        end
        
        theta1new = theta1 + stepsize*W_error;
        while ((theta1new < 0) || imag(theta1new ~= 0))
            stepsize = stepsize/10;
            theta1new = theta1 + stepsize*W_error;
        end
        theta1 = theta1new;
    end
   
    %%% Compute slip-sinkage - alines 2020-07-14
    z0 = (1 - cos(theta1))*Rw;
    if slip_sink == 1
        Kss = (1+sigma_fl/100)/(1-(0.5*sigma_fl/100));
        z_fl_slip = Kss*z0;
    elseif slip_sink == 2      
        if (sigma_fl/100)>.3 && flatten==1
            factor = ((S*.5)./(1+exp(-8*((sigma_fl/100)-.25))));
        else
            factor = S*(sigma_fl/100);
        end
        if excavation_fl == 1
            z_fl_slip = z0 - factor*z0;
        else
            z_fl_slip = z0 + factor*z0;
        end
        if z_fl_slip < 0
            z_fl_slip = 0;
        end  
    elseif slip_sink == 3
        z_fl_slip = z0 - ding_slip_sink(thetam,theta2,phi,sigma_fl/100)*((wfl*h)/(thetam - theta2));
    end
    if slip_sink > 0
        theta_slip = acos(1 - (z_fl_slip/Rw));
    end

    %%% Don't allow immediate increase in sinkage, limit sinkage
    z_fl = Rw*(1 - cos(theta1));
    if i>1
        if slip_sink == 0
            z_fl_slip = z_fl;
        end
        vz_fl = (z_fl_slip - sinkage(i,1))/h;
        z_fl_orig = z_fl;
        if damp_terrain == 1
            sink_add = b_terrain*vz_fl;
            if abs(sink_add) > sink_max
                sink_add = sink_max*(sign(sink_add));
            end
            z_fl = sinkage(i,1) + sink_add;
            theta1 = acos(1-z_fl/Rw);
        else
            if abs(z_fl_slip - sinkage(i,1)) > (sink_max)
                z_fl = sinkage(i,1) + sink_max*sign(vz_fl);
                theta1 = acos(1-z_fl/Rw);
            else
                z_fl = Rw*(1 - cos(theta_slip));
                theta1 = acos(1-z_fl/Rw);
            end 
        end
    end   
end     %%% diag_flag

% Compute resistance
R1 = integral(@(xt)normal_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),thetam,theta1);
R3 = integral(@(xt)shear_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),thetam,theta1);

if patch == 0 || patch == 1
    R2 = 0;
    R4 = 0;
elseif patch == 2
    R2 = integral(@(xt)normal_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),theta2,thetam);
    R4 = integral(@(xt)shear_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),theta2,thetam);
end

% Compute total torque
T1 = integral(@(xt)shear_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'torque',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),thetam,theta1);
if patch == 0 || patch == 1
    T2 = 0;
else
    T2 = integral(@(xt)shear_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,sigma_fl/100,'torque',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),theta2,thetam);
end

Tfl_r = Rw*Rw*b*(T1 + T2);      % Resistive torque

if Tfl_r < 0
    Tfl_r = 0;
end

% Tractive effort
Fxfl = Rw*b*(R3+R4);            % Tractive effort

% Bulldozing Resistance
if bulldozing == 1
    z_b = z_fl - (1-cos(thetam))*Rw;
    Rbfl = b*(0.667*z_b*c*Kc + 0.5*(z_b^2)*gamma*Ky)/1000;   % Gee-Clough, pg 162
else
    Rbfl = 0;
end

% Gravity Resistance
Rgfl = Fx_g*(Fzfl/Fz);

DP = Rw*b*(R3 + R4 - R1 - R2) - Rbfl - Rgfl;
Rxfl = Rw*b*(R1+R2) + Rbfl + Rgfl;

if debug == 1
    k = 1:j-1;                  % DEBUG
    figure(1)                   % DEBUG
    hold on;                    % DEBUG
    plot(k,theta1_g);           % DEBUG
    plot(k,thetam_g);           % DEBUG
    yyaxis right;               % DEBUG
    plot(k,W_error_g);          % DEBUG
    legend('theta_1','theta_m','W_e');  % DEBUG
    if patch == 1
        xtheta2 = linspace(-thetam,thetam,200);
        z2 = ones(size(xtheta2));
        y2 = ones(size(xtheta2));
        z2 = z2*z_const;
        y2 = y2*W2;
    else
        xtheta2 = linspace(theta2,thetam,200);
        z2 = Rw*abs(cos(theta1 - ((xtheta2 - theta2)/(thetam-theta2))*(theta1-thetam)) - cos(theta1));
        if model == 1
            y2 = (kc + kphi*b)*((Rw/b).^n)*((abs((cos(theta1 - xtheta2*(theta1 - thetam)/thetam) - cos(theta1)))).^n);
        elseif model == 2
            y2 = zeros(size(z2));
            y2(z2<zm) = km*zm*(-log(1-(z2(z2<zm)/zm)));
            x21 = zm-0.0002;
            x22 = zm-0.0001;
            y21 = km*zm*(-log(1-(x21/zm)));
            y22 = km*zm*(-log(1-(x22/zm)));
            slope = (y22-y21)/(x22-x21);
            if sum(z2>=zm)>1
                y2(z2>=zm) = (z2(z2>=zm)-x21)*slope+y22;
            end
        elseif model == 4
            y2 = (kc + kphi*b)*((Rw/b)*(cos(theta1 - ((xtheta2 - theta2)/(thetam-theta2))*(theta1-thetam)) - cos(theta1))).^n;
        end
        j2 = Rw*(theta1 - xtheta2 - (1-sigma_fl/100).*(sin(theta1) - sin(xtheta2)));
        tau2 = (c_fl + y2*tan(phi)).*(1-exp(-j2/Ks_fl));
    end
    xtheta = linspace(thetam,theta1,200);
    if model == 1
        y1 = (kc + kphi*b)*((Rw/b).^n)*((cos(xtheta) - cos(theta1)).^n);
    elseif model == 2
        z = Rw*(cos(xtheta) - cos(theta1));
        y1 = zeros(size(z));
        y1(z<zm) = km*zm*(-log(1-(z(z<zm)/zm)));
        x11 = zm-0.0002;
        x12 = zm-0.0001;
        y11 = km*zm*(-log(1-(x11/zm)));
        y12 = km*zm*(-log(1-(x12/zm)));
        slope = (y12-y11)/(x12-x11);
        if sum(z>=zm)>1
            y1(z>=zm) = (z(z>=zm)-x11)*slope+y12;
        end
    elseif model == 4
        y1 = (kc + kphi*b)*(((Rw/b)*(cos(xtheta) - cos(theta1))).^n);
    end
    j1 = Rw*(theta1 - xtheta - (1-sigma_fl/100).*(sin(theta1) - sin(xtheta)));
    tau1 = (c_fl + y1*tan(phi)).*(1-exp(-j1/Ks_fl));

    angle = [xtheta2,xtheta];
    y_new = [y2,y1];
    tau_new = [tau2,tau1];
    if patch == 1
        normal = [y2,y1.*cos(xtheta)];
        resist = [zeros(1,200),y1.*sin(xtheta)];
    else
        normal = [y2.*cos(xtheta2),y1.*cos(xtheta)];
        resist = [y2.*sin(xtheta2),y1.*sin(xtheta)];
    end
    j_theta = [j2,j1];
    
    figure(2)
    hold on
    plot(rad2deg(angle),normal)
    plot(rad2deg(angle),resist)
    legend('z-direction','x-direction');
    xlabel('Degrees')
    ylabel('Normal Stress [kPa]')
    figure(3)
    plot(rad2deg(angle),tau_new);
    xlabel('Degrees')
    ylabel('Shear Stress [kPa]')
end

if slipCalc == 1
    min_flag = 0;
    slip_ct = 0;
    for slip_fl = 0:0.01:1
        slip_ct = slip_ct + 1;
        R1 = integral(@(xt)normal_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),thetam,theta1);
        R3 = integral(@(xt)shear_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,slip_fl,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),thetam,theta1);
        if patch == 0 || patch == 1
            R2 = 0;
            R4 = 0;
        elseif patch == 2
            R2 = integral(@(xt)normal_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,z_offset),theta2,thetam);
            R4 = integral(@(xt)shear_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,slip_fl,'resist',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),theta2,thetam);
        end
        T1 = integral(@(xt)shear_stress_lines(xt,theta1,thetam,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,slip_fl,'torque',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),thetam,theta1);
        if patch == 0 || patch == 1
            T2 = 0;
        else
            T2 = integral(@(xt)shear_stress_lines_2(xt,theta1,thetam,theta2,kc,b,kphi,n,Rw,phi,c_fl,Ks_fl,Kr,Kw,slip_fl,'torque',model,km_fl,zm_fl,kp1,kp2,kz1,kz2,Tfl,min_flag,shear_model,z_offset),theta2,thetam);
        end
        drawbar(slip_ct) = Rw*b*(R3 + R4 - R1 - R2);
        F_slip(slip_ct) = Rw*b*(R3+R4);
        resistance(slip_ct) = Rw*b*(R1 + R2);
        torque(slip_ct) = Rw*Rw*b*(T1+T2);
        slip_range(slip_ct) = slip_fl;
        clear T1 T2 R1 R2 R3 R4;
    end
    slip_range = slip_range*1;
    F_slip = F_slip*1000;
    drawbar = drawbar*1000;
    resistance = resistance*1000;
    torque = torque*1000;
    
    figure(2)
    subplot(1,2,1) 
    plot(slip_range,drawbar,'-','color',[0.8500    0.3250    0.0980],'LineWidth',2);
    
    hold on
    plot(slip_range,F_slip,'--','color',[0 0.4470 0.7410],'LineWidth',2);    
    plot([-1,1],[0,0],'k')

    plot(slip_range,-resistance,':','color', [0.4660 0.6740 0.1880],'LineWidth',2);
    %legend("Net Force","Tractive Force","Resistive Force",'location',"northeast", 'FontSize', 12)
    ylim([-100,180])
    xlim([0,1])
    pbaspect([1.5 1 1])
    xlabel('Slip [0,1]', 'FontSize', 18)
    ylabel('Force [N]', 'FontSize', 18)
    title('Well Bonded', 'FontSize', 16)
    hold off

    figure(3)
    subplot(1,2,1)
    plot(slip_range,torque);
    pbaspect([1.5 1 1])


end
break
end
end
