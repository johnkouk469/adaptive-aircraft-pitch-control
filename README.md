# Adaptive aircraft pitch control

Model reference adaptive control (MRAC) for the pitch dynamics of an aircraft: a Lyapunov-based design, and a MATLAB simulation that shows an LQR controller going unstable under actuator loss and an adaptive controller that keeps tracking.

![Angle of attack y, reference model output ym and reference input r over 100 s with the adaptive controller, and the elevator command below.](exercise2/docs/figures/c-tracking.png)

## Context

Two individual assignments for Intelligent and Adaptive Control Systems at the Aristotle University of Thessaloniki, School of Electrical and Computer Engineering, academic year 2021-22. The course provided the assignment texts and the aircraft model; the design, the derivations, the code and the reports are mine.

| Assignment | What it is |
|---|---|
| [Exercise 1](exercise1/docs/report.md) ([brief](exercise1/docs/brief.md)) | Theory: a state-feedback controller with integral action for a linear system with an unknown loss of control effectiveness D and an unknown nonlinearity $f(x) = \theta^T \Phi(x)$. Adaptive laws from a Lyapunov function, boundedness of all signals and convergence of the tracking error via Barbalat's lemma. |
| [Exercise 2](exercise2/docs/report.md) ([brief](exercise2/docs/brief.md)) | MATLAB: the same design applied to an aircraft's angle of attack, with the elevator as input. |

## What exercise 2 does

- (a) Adds the integral of the output error to the state and uses LQR to build the reference model; the closed loop tracks a sequence of steps in the angle of attack.
- (b) Adds an actuator loss (D = 0.8) and a state-dependent disturbance to the plant and shows that the controller of (a) becomes unstable.
- (c) Replaces it with the adaptive controller from exercise 1, which restores tracking without knowing D or the disturbance.

The model works in rad; the reference input (given in degrees) and all plots are in degrees.

## Getting started

Requirements: MATLAB with the Control System Toolbox (`lqr`, `lyap`). Tested with MATLAB R2019a.

```
cd exercise2/src
run_pitch_control   % runs parts (a) to (c) and opens six figures
make_figures        % runs run_pitch_control and saves the six figures to ../docs/figures/
```

`make_figures` takes about 10 s on the PC it was tested on.

## Results

From a run of `make_figures` in MATLAB R2019a:

| Part | Result |
|---|---|
| (a) LQR reference model | y follows ym (largest difference below 1e-14 deg); peak elevator command 2.9 deg |
| (b) same controller, with actuator loss and disturbance | unstable: abs(y) grows exponentially, to about 1.6e173 deg at t = 100 s |
| (c) adaptive controller | largest tracking error 0.0096 deg, below 0.0031 deg after t = 20 s; peak elevator command 12.9 deg |

The figures and their discussion are in the [exercise 2 report](exercise2/docs/report.md).

## Project structure

```
exercise1/docs/   report.md, brief.md
exercise2/docs/   report.md, brief.md, figures/
exercise2/src/    run_pitch_control.m   main script: parts (a), (b), (c) and the plots
                  ode_lqr_nominal.m     closed loop of part (a)
                  ode_lqr_uncertain.m   part (a) controller on the uncertain plant
                  ode_adaptive.m        adaptive controller and update laws
                  reference_input.m     reference input r(t)
                  make_figures.m        saves the figures used in the report
```

## Limitations

- Simulation only, on the linear model from the brief, without actuator limits, noise or delays.
- The adaptation gains were tuned by trial for this reference input.
- The parameter estimates do not converge to their ideal values. Because $\Phi(x) = x$, the controller only uses their difference, and that settles near, not at, its ideal value. Tracking does not need them to converge.
- Tested only with MATLAB R2019a. Not tested with GNU Octave.

## Future work

- Run the code under GNU Octave (with the `control` package) and note what, if anything, needs changing.
- Add a short script that runs the three parts and checks the numbers in the results table, so a reader can confirm them in about 10 s.
- Add a code map to this README: where the LQR design, the Lyapunov matrix and the update laws are, and which report equations they implement.

## Credits and third-party material

The assignment briefs in `exercise1/docs/brief.md` and `exercise2/docs/brief.md`, including the aircraft picture in the exercise 2 brief and the aircraft model, come from the course and are translated from Greek. They are not covered by the licence below.

## Licence

MIT for the code and the reports; see [LICENSE](LICENSE).
