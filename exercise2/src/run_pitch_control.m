%% RUN_PITCH_CONTROL  Exercise 2: pitch control of an aircraft, parts (a), (b) and (c).
% (a) LQR controller with a reference model, (b) the same controller with an
% actuator loss and a disturbance (unstable), (c) the adaptive controller.
% Opens six figures. The plant, controller and reference model share their
% parameters through the struct p, which is passed to the ODE functions.

%% clear everything
clc;
clear;
close all;
%% init variables
% init time variables
tstart = 0;
tfinal = 100;

% the model works in rad; the plots show angles in degrees, the unit of the brief
to_deg = 180/pi;

% plant: state x = [integral of y; y; q], input u (elevator), output y (angle of attack)
Ap = [-0.8060 1 ; -9.1486 -4.59 ];
Bp = [-0.04 ; -4.59];
Cp = [1 0];

np = size(Ap,1);
m = size(Cp,1);
n = np + m;

A = zeros(n);
A(m+1:n,m+1:n) = Ap;
A(1:m,m+1:n) = Cp;

B = zeros(n,size(Bp,2));
B(m+1:n,1:size(Bp,2))= Bp;

Bm = zeros(n,m);
Bm(1:m,1:m) = - ones(m,m);

p.A = A;
p.B = B;
p.Bm = Bm;

%% (a)

Q_lqr = [20 0 0 ; 0 0 0 ; 0 0 0];
R_lqr = 0.1;
k_lqr = lqr(A,B,Q_lqr,R_lqr);
Am = A - B*k_lqr;

p.k_lqr = k_lqr;
p.Am = Am;

% init starting point
vinit = zeros(6, 1); % [x(0) ; xm(0)]

%% run ode
tspan = [tstart tfinal];
[t, v] = ode23(@(t, v) ode_lqr_nominal(t, v, p), tspan, vinit);

%% define ode output variables
x = v(:, 1:3);
xm = v(:, 4:6);

r = reference_input(t);

u = -k_lqr*x';

%% plotting

% plot output
figure(1); clf;
subplot(2, 1, 1)
% plot the evolution of y and ym with respect to time
plot(t, x(:,2)*to_deg, 'r');
hold on
plot(t, xm(:,2)*to_deg, 'g--');
plot(t, r*to_deg, 'b:');
legend("y","ym","r")
ylabel('$y(t), \: y_m(t) \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Input \: signal \: r \: and \: the \: evolution \: of \: y \: and \: y_m \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

% plot controller signal u
subplot(2, 1, 2)
plot(t, u*to_deg);
ylabel('$u(t) \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Control \: signal \: \delta_e$',...
    'Interpreter', 'latex', 'fontsize', 12);

%% (b)
ka = 1.5 * A(3,2);
kq = 0.5 * A(3,3);
theta = zeros(3,1);
theta(2) = ka;
theta(3) = kq;
D = 0.8;

p.theta = theta;
p.D = D;

% init starting point
vinit = zeros(6, 1); % [x(0) ; xm(0)]

%% run ode
[t, v] = ode23(@(t, v) ode_lqr_uncertain(t, v, p), tspan, vinit);

%% define ode output variables
xb = v(:, 1:3);
xmb = v(:, 4:6);

r = reference_input(t);

%% plotting

% plot output
figure(2); clf;
subplot(2, 1, 1)
% |y| and |ym| on a log scale: the exponential growth of y shows as a straight line
semilogy(t, abs(xb(:,2))*to_deg, 'r');
hold on
semilogy(t, abs(xmb(:,2))*to_deg, 'g--');
legend("|y|","|ym|", 'Location', 'southeast')
ylabel('$|y(t)|, \: |y_m(t)| \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$|y| \: and \: |y_m| \: over \: time \: (log \: scale)$',...
    'Interpreter', 'latex', 'fontsize', 12);

% the first seconds on a linear scale: y leaves ym right after the first step of r
subplot(2, 1, 2)
plot(t, xb(:,2)*to_deg, 'r');
hold on
plot(t, xmb(:,2)*to_deg, 'g--');
plot(t, r*to_deg, 'b:');
legend("y","ym","r")
ylabel('$y(t), \: y_m(t) \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 3])
ylim([-1 3])
title('$Input \: signal \: r \: and \: the \: evolution \: of \: y \: and \: y_m \: in \: the \: first \: 3 \: s$',...
    'Interpreter', 'latex', 'fontsize', 12);


%% (c)
% the gains were tuned with all signals in degrees; the states are in rad and the
% adaptive laws are quadratic in the signals, so the gains scale by (180/pi)^2
p.gamma_k = (180/pi)^2 * [2000 0 0 ; 0 2000 0 ; 0 0 200];
p.gamma_theta = (180/pi)^2 * [2000 0 0 ; 0 2000 0 ; 0 0 200];
Q = [100 0 0 ; 0 100 0 ; 0 0 100];
p.P = lyap(Am',Q);

% init starting point
vinit = zeros(12, 1); % [x(0) ; xm(0) ; khat(0) ; thetahat(0)]

%% run ode
[t, v] = ode23(@(t, v) ode_adaptive(t, v, p), tspan, vinit);

%% define ode output variables
xc = v(:, 1:3);
xmc = v(:, 4:6);
khat = v(:, 7:9);
thetahat = v(:, 10:12);

r = reference_input(t);
uc = zeros(size(t));
for i=1:size(t,1)
    uc(i) = khat(i,:)*xc(i,:)' - thetahat(i,:)*xc(i,:)';
end

%% calculate error

% output tracking error e = y - ym
ec = xc(:,2) - xmc(:,2);

% with Phi(x) = x the controller only uses khat - thetahat, so only that
% difference is determined; matching A + B*D*(khat - thetahat + theta)' = Am
% gives its ideal value
g_star = -k_lqr/D - theta';

%% plotting

% plot output
figure(3); clf;
subplot(2, 1, 1)
% plot the evolution of y and ym with respect to time
plot(t, xc(:,2)*to_deg, 'r');
hold on
plot(t, xmc(:,2)*to_deg, 'g--');
plot(t, r*to_deg, 'b:');
legend("y","ym","r")
ylabel('$y(t), \: y_m(t) \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Input \: signal \: r \: and \: the \: evolution \: of \: y \: and \: y_m \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

% plot controller signal u
subplot(2, 1, 2)
plot(t, uc*to_deg);
ylabel('$u(t) \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Control \: signal \: \delta_e$',...
    'Interpreter', 'latex', 'fontsize', 12);

% plot the estimates khat
figure(4); clf;
subplot(3, 1, 1)
plot(t, khat(:,1));
ylabel('$\hat{k_x}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{k_x}(t) \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

subplot(3, 1, 2)
plot(t, khat(:,2));
ylabel('$\hat{k_y}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{k_y}(t) \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

subplot(3, 1, 3)
plot(t, khat(:,3));
ylabel('$\hat{k_z}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{k_z}(t) \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

% plot the estimates thetahat
figure(5); clf;
subplot(3, 1, 1)
plot(t, thetahat(:,1));
ylabel('$\hat{\theta_x}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{\theta_x}(t) \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

subplot(3, 1, 2)
plot(t, thetahat(:,2));
ylabel('$\hat{\theta_y}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{\theta_y}(t) \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

subplot(3, 1, 3)
plot(t, thetahat(:,3));
ylabel('$\hat{\theta_z}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{\theta_z}(t) \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

figure(6); clf;
subplot(2, 1, 1)
plot(t, ec*to_deg);
ylabel('$e(t) \: [deg]$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: error \: e=y-y_m \: over \: time$',...
    'Interpreter', 'latex', 'fontsize', 12);

% khat - thetahat (solid) against its ideal value g_star (dashed, same colours)
subplot(2, 1, 2)
plot(t, khat - thetahat);
hold on
set(gca, 'ColorOrderIndex', 1);
plot([tstart tfinal], [g_star; g_star], '--');
ylabel('$\hat{k}(t) - \hat{\theta}(t)$', 'Interpreter', 'latex', 'fontsize', 12);
xlabel('$t[s]$', 'Interpreter', 'latex', 'fontsize', 12);
xlim([0 tfinal])
title('$Evolution \: of \: \hat{k} - \hat{\theta} \: (solid) \: and \: its \: ideal \: value \: (dashed)$',...
    'Interpreter', 'latex', 'fontsize', 12);
