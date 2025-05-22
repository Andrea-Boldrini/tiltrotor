function [R] = Body2Inertial(phi, theta, psi)

% Compute trigonometric terms
Cphi = cos(phi);   Sphi = sin(phi);
Ctheta = cos(theta);  Stheta = sin(theta);
Cpsi = cos(psi);   Spsi = sin(psi);

% Define the rotation matrix R
R = [ Ctheta*Cpsi,  Sphi*Stheta*Cpsi - Cphi*Spsi,  Cphi*Stheta*Cpsi + Sphi*Spsi;
       Ctheta*Spsi,  Sphi*Stheta*Spsi + Cphi*Cpsi,  Cphi*Stheta*Spsi - Sphi*Cpsi;
      -Stheta,       Sphi*Ctheta,                   Cphi*Ctheta ];

end

