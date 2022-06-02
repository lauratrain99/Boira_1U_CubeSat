%% BOIRA 1U CubeSat %% 
% Sergio, BOIRA Mission Analysis, MiSE
% 11/04/22

%% Orbit design script
% This script provides an interface to trade-off orbital elements 
% for Sun-synchronous orbits. 

% Orbits in consideration are sun-synchronous from 300 km to 800 km

% All units are in S.I.

set_graphics();

%% Constants
mu = 3.986e14;                                  % Earth gravitational parameter
J2 = 1.08263e-3;                                % Second zonal harmonic of the Earth
J3 =  -2.53881e-6;                              % Third zonal harmonic of the Earth
Re = 6378.14e3;                                 % Mean Earth radius
SolarYear = 365.242199;                         % Solar year
tau = (3600*24)*(SolarYear/(1+SolarYear));      % Sidereal day
Earth_Omega = (2*pi/SolarYear)/(3600*24);       % Earth Mean motion
Sun_Omega = (360)/SolarYear;                    % Earth rate relative to vernal equinox
OmegaE = 7.2921150e-5;                          % Angular speed of the Earth 

%% Instrument characterization 
FOV = deg2rad(5);                               % EO instrument FOV
  
%% Design input and orbit requirements
% Time relationships 
vernalTime = datetime(2023,03,20,22,34,0,0);    % Day of the vernal equinox
vernalTime = datenum(vernalTime);               % Day of the vernal equinox

launchTime = datetime(2023,12,21,0,0,0,0);      % Launch day
time = datenum(launchTime);                     % Launch day

% Sun requirements (this must change as a function of the launch date. See Sun's analemma)
delta = deg2rad(2);                             % Solar declination
eps = 0;                                        % Equation of time

% Orbit requirements 
LTAN = 6;                                       % Mean Local Time of the Ascending Node
phi = deg2rad(40.304665);                       % Latitude of interest for the LTAN (Madrid)

%% Argument of perigee design 
% In general, this element cannot be freely selected and depends on the launcher 

% To freeze the eccentricity, select the AoP as
omega_d = 270;                         

elements(5) = omega_d;

%% Semimajor axis design
% Groundtrack requirements
a_d = Re+522e3;                                 % Desired orbital SMA

elements(1) = a_d; 

%% Inclination selection
% General inclination evaluation
dOmega = Earth_Omega;                           % Orbital precession rate
dh = 100;                                       % Altitude step [m]
a_max = 1000e3;                                 % Orbital altitude (circular orbit) over the geoid
a = Re:dh:Re+a_max;                             % Orbital altitude
e = 0;                                          % Orbital eccentricity

% Inclination-semimajor axis function for SSOs
n = sqrt(mu./a.^3);                             % Orbital mean motion options
p = a*(1-e^2);                                  % Semilatus rectum of the orbit
i = acos((-2*dOmega*p.^2)./(3*J2*Re^2.*n));     % Inclination 

% Desired inclination
n_d = sqrt(mu/a_d^3);                           % Orbital mean motion
p_d = a_d*(1-e^2);                              % Semilatus rectum of the orbit
i_d = acos((-2*dOmega*p_d^2)/(3*J2*Re^2*n_d));  % Desired inclination

% Iteration scheme setup 
tol = 1e-20;                                    % Convergence tolerance
error = 1;                                      % Initial error

% Iterative scheme
while (error >= tol)
    % Frozen AoP eccentricity (J3 perturbed motion)
    e = (-1/2)*(J3/J2)*(Re/a_d)*sin(i_d);

    p_d = a_d*(1-e^2);                                                     % Semilatus rectum of the orbit

    % J2 perturbed mean motion
    n_p  = n_d*(1+(3/2)*J2*(Re/a_d)^2*sqrt(1-e^2)*(1-(3/2)*sin(i_d)^2));   % First order perturbation in the mean motion due to the J2
    i_p = acos((-2*dOmega*p_d^2)/(3*J2*Re^2*n_p));                         % Sun-synchronous condition

    % Convergence criteria
    error = abs(i_p-i_d);
    i_d = i_p;
end

elements(3) = i_d;

%% Eccentricity design 
% In general, this element cannot be freely selected and depends on the launcher and the uncertainty in the injection point 

% To freeze the AoP, select the eccentricity as
e_d = (-1/2)*J3/J2*(Re/a_d)*sin(i_d);

elements(2) = e_d;

%% RAAN design (local illumination conditions) 
% Local time analysis
phi = (1/15)*asin(tan(phi)/tan(i_d));
LTAN = LTAN-phi;
RAAN_d = (360/24)*LTAN-180+(Sun_Omega)*(time-vernalTime);

% Design RAAN
RAAN_d = mod(RAAN_d, 360);

elements(4) = RAAN_d;

%% Groundtrack analysis
% Pertubed nodal period
Po = 2*pi*sqrt(a_d^3/mu)*(1-(3/2)*J2*(Re/a_d)^2*(3-4*sin(i_d)^2)); 

dL = Po*(OmegaE-dOmega);                    % East-West drift or fundamental interval
rev_days = 86400/Po;                        % Revolutions per day 
rep_days = 2*pi/dL;                         % Number of revolutions in the cycle       
cycle_days = floor(rep_days/rev_days);      % Length of the cycle in days 

ratio = Po/86400; 

%% Solar illumination conditions analysis
% SSO beta angle
s = [cos(delta)*cos(eps); cos(delta)*sin(eps); sin(delta)];     % Sun vector 
h = [sin(i_d)*sin(RAAN_d); -sin(i_d)*cos(RAAN_d); cos(i_d)];    % Angular momentum vector 
beta = asin(dot(h,s));                                          % Solar angle 

% Shadow time 
eta = asin(Re/a_d);                        
nu = 2*acos(cos(eta)/cos(beta));            
dTimeShadow = rad2deg(nu)/360*Po;                               % Time in shadow

%% Sampling time 
r(1) = a_d*(1-e_d);             % Nominal perigee altitude
r(2) = a_d*(1+e_d);             % Nominal apogee altitude

amin = Re+250e3;                % Minimum nominal SMA 
r(3) = amin*(1-e_d);            % Minimum perigee altitude
r(4) = amin*(1+e_d);            % Minimum apogee altitude

dcone = tan(FOV)*r;             % FOV cone diameter
sampling_time = dcone./sqrt(mu*(2./r-1./[a_d a_d amin amin]));

curvature = dcone/Re;           % Longitude range 
sampling_time(2,:) = curvature./sqrt(mu./[a_d a_d amin amin].^3);

%% Instantenous Access Area 
% Constants
K = 2.55604187e8;               % Conversion factor to km2

% Spacecraft orbital altitude evolution 
mission_lifetime = SolarYear*2;
t = 0:mission_lifetime-1;
rmax(1,:) = (1+e_d)*linspace(a_d, amin, mission_lifetime);
rmax(2,:) = (1-e_d)*linspace(a_d, amin, mission_lifetime);

% IAA
lambda0 = acos(Re./rmax(1,:));
IAA(1,:) = K*(1-cos(lambda0)); 

lambda0 = acos(Re./rmax(2,:));
IAA(2,:) = K*(1-cos(lambda0));

maxIAA = [IAA(1,1) IAA(2,end)]; 

% Relative IAA
IAA = IAA/IAA(1,1);

% Number of communications 
S = 510e12; 
D = pi*( (min(rmax(2,:))-Re)*tan(deg2rad(5)) )^2;
Communications = 1.15*floor(S/D);

% Number of payload data 
PL_data = 48;
Data = Communications*PL_data/(1024)^2;

%% Results
fprintf("LTAN: %.4f h \n", LTAN);
fprintf("Orbital SMA: %.8f km \n", (a_d/1e3));
fprintf("Orbital Ecc: %.8f \n", e_d);
fprintf("Orbital Inc: %.8f deg \n", rad2deg(i_d));
fprintf("Orbital RAAN: %.8f deg \n", RAAN_d);
fprintf("Orbital AoP: %.8f deg \n", omega_d);

fprintf("Time in shadow: %.4f min \n", dTimeShadow/60);

fprintf("Size of payload data: %.4f MB \n", Data);

figure(1) 
hold on
plot((a-Re)/1e3, rad2deg(i), 'b');
plot((a_d-Re)/1e3, rad2deg(i_d), 'or'); 
hold off 
grid on
xlabel('Orbital altitude over the geoid (km)'); 
ylabel('Orbital inclination (deg)'); 
legend('Sun-synchronous orbit', 'Design point for BOIRA');
title('Orbit design point for BOIRA');

figure(2) 
hold on
plot(t, IAA(1,:), 'b');
plot(t, IAA(2,:), 'r'); 
hold off 
grid on
xlabel('Mission time [days]'); 
ylabel('IAA  [km$^2$]'); 
legend('Max IAA per orbit', 'Min IAA per orbit');
title('IAA evolution in time');

%% Auxiliary functions
%Some cool graphics setup
function set_graphics()
    %Set graphical properties
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); 
    set(groot, 'defaultAxesFontSize', 11); 
    set(groot, 'defaultAxesGridAlpha', 0.3); 
    set(groot, 'defaultAxesLineWidth', 0.75);
    set(groot, 'defaultAxesXMinorTick', 'on');
    set(groot, 'defaultAxesYMinorTick', 'on');
    set(groot, 'defaultFigureRenderer', 'painters');
    set(groot, 'defaultLegendBox', 'off');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultLegendLocation', 'best');
    set(groot, 'defaultLineLineWidth', 1); 
    set(groot, 'defaultLineMarkerSize', 3);
    set(groot, 'defaultTextInterpreter','latex');
end
