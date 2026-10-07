function r = reference_input(t)
% REFERENCE_INPUT  Reference input r(t) of the brief, in rad.
%   r = reference_input(t) returns the sequence of steps in the angle of
%   attack for the time t (scalar or vector, in s). The brief gives the
%   steps in degrees; the model states are in rad, so r is returned in rad.

r_deg = zeros(size(t));
r_deg(t >= 1  & t < 10) = 0.5;
r_deg(t >= 22 & t < 32) = -0.5;
r_deg(t >= 45 & t < 55) = 1;
r_deg(t >= 65 & t < 75) = -1;
r_deg(t >= 85 & t < 95) = 0.5;

r = r_deg * pi/180;

end
