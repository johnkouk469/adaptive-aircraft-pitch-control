function dv = ode_adaptive(t, v, p)
% ODE_ADAPTIVE  Part (c): adaptive controller and update laws on the uncertain plant.
%   dv = ode_adaptive(t, v, p) returns the derivative of
%   v = [x; xm; khat; thetahat] (3 each): the plant state, the reference-model
%   state and the two parameter estimates. The controller does not know D or theta.
%   p is the parameter struct built in run_pitch_control (fields A, B, Bm, Am,
%   theta, D, P, gamma_k, gamma_theta).

x = v(1:3);
xm = v(4:6);
khat = v(7:9);
thetahat = v(10:12);

r = reference_input(t);

f = p.theta' * x;
u = khat'*x - thetahat'*x;
e = x - xm;

dv = zeros(size(v));
dv(1:3) = p.A * x + p.B * p.D * (u + f) + p.Bm * r;   % plant
dv(4:6) = p.Am * xm + p.Bm * r;                       % reference model
dv(7:9) = -p.gamma_k * x * e' * p.P * p.B;            % khat update law
dv(10:12) = p.gamma_theta * x * e' * p.P * p.B;       % thetahat update law

end
