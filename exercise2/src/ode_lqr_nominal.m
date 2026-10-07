function dv = ode_lqr_nominal(t, v, p)
% ODE_LQR_NOMINAL  Part (a): LQR controller on the nominal plant, with the reference model.
%   dv = ode_lqr_nominal(t, v, p) returns the derivative of v = [x; xm], where
%   x is the plant state and xm the reference-model state (3 each).
%   p is the parameter struct built in run_pitch_control (fields A, B, Bm, Am, k_lqr).

x = v(1:3);
xm = v(4:6);

r = reference_input(t);
u = -p.k_lqr * x;

dv = zeros(size(v));
dv(1:3) = p.A * x + p.B * u + p.Bm * r;   % plant
dv(4:6) = p.Am * xm + p.Bm * r;           % reference model

end
