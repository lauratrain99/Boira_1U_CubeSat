function [r, v] = coe2rv(mu, a, e, Omega, inc, omega, theta)
% coe2rv converts from Classical Orbit Elements to the components of the state
% vector x = [r,v] (position and velocity)
% vector x = [r,v] (position and velocity) in ECI coordinates
%
% INPUT
%           mu, gravitational parameter [km^3/s]
%        theta, true anomaly [rad]
%
% OUTPUT 
%            r, 3x1 position vector [km]
%            v, 3x1 velocity vector [km/s]
%            r, 3x1 position vector in ECI coords [km]
%            v, 3x1 velocity vector in ECI coords [km/s]
%
%%
    % B0 - Equatorial ECI vector bases. All the vectors are given
    % in terms these coordinates
    i0 = [1;0;0];
    j0 = [0;1;0];
    k0 = [0;0;1];
    
    
    % B1 
    i1 = cos(Omega)*i0 + sin(Omega)*j0;
    j1 = -sin(Omega)*i0 + cos(Omega)*j0;
    k1 = k0;
    
    % B2
    i2 = i1;
    j2 = cos(inc)*j1 + sin(inc)*k1;
    k2 = -sin(inc)*j1 + cos(inc)*k1;
   
    % B3
    i3 = cos(omega)*i2 + sin(omega)*j2;
    j3 = -sin(omega)*i2 + cos(omega)*j2;
    k3 = k2;
    
    
    % Cylindrical vector basis in the plane of motion
    er = cos(theta)*i3 + sin(theta)*j3;
    etheta = -sin(theta)*i3 + cos(theta)*j3;
    
    % Orbit parameter [km]
    p = a*(1 - e^2);
    
    % Angular momentum [km^2/s]
    h = sqrt(p*mu);
    
    % Norm of the position vector [km]
    r_norm = p / (1 + e*cos(theta));
    
    % Radial velocity [km/s]
    rdot = mu/h * e*sin(theta);
    
    % Azimuthal velocity [km/s]
    rthetadot = mu/h*(1 + e*cos(theta)); % can be also computed by h/r
    
    % Position vector [km]
    r = r_norm * er;
    
    % Velocity vector [km/s]
    v = rdot * er + rthetadot * etheta;
    
    % Check angular velocity vector and k3 vectors are parallel
    h = cross(r,v);
    
end