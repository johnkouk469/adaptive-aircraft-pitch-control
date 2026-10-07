function dv = ode_lqr_uncertain(t, v, p)
% ODE_LQR_UNCERTAIN  Part (b): the controller of part (a) on the uncertain plant.
%   dv = ode_lqr_uncertain(t, v, p) returns the derivative of v = [x; xm]. The plant
%   has an actuator loss p.D and a state-dependent disturbance p.theta' * x, which
%   the controller does not know about.
%   p is the parameter struct built in run_pitch_control (fields A, B, Bm, Am, k_lqr, theta, D).

x = v(1:3);
xm = v(4:6);

r = reference_input(t);
u = -p.k_lqr * x;
f = p.theta' * x;

dv = zeros(size(v));
dv(1:3) = p.A * x + p.B * p.D * (u + f) + p.Bm * r;   % plant
dv(4:6) = p.Am * xm + p.Bm * r;                       % reference model

end
