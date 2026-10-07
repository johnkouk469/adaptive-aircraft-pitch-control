# Intelligent and Adaptive Control Systems: Assignment 1

> The assignment brief as issued by the course staff (academic year 2021-22), translated from Greek. The text and the problem belong to the course staff; only the translation is new. The report that answers it: [report.md](report.md).

Consider the system:

$$
\dot{x}_p = A_p x_p + B_p D (u + f(x_p)) \tag{1}
$$

where $x_p \in \mathbb{R}^{n_p}$ is the state vector of the system and $u \in \mathbb{R}$ the control input. The function $f(*)$ has an unknown form, but we know that it is locally Lipschitz continuous in $x_p$ and, moreover,

$$
f(x_p) = \theta^T \Phi(x_p) \in \mathbb{R}. \tag{2}
$$

In (2), $\theta \in \mathbb{R}^{N \times 1}$ denotes a vector of unknown but constant parameters and $\Phi(x_p) \in \mathbb{R}^N$ a vector of known nonlinear functions, locally Lipschitz continuous in $x_p$.

In (1), the matrices $A_p \in \mathbb{R}^{n_p \times n_p}$ and $B_p \in \mathbb{R}^{n_p \times 1}$ are known and constant. The unknown constant $D$ is strictly positive. The pair $(A_p, B_p)$ is controllable. The output of the system satisfies $y_p = C_p^T x_p \in \mathbb{R}$, with $C_p \in \mathbb{R}^{n_p}$ a known constant vector. The constant $D$ models the loss of control "power" that can appear through actuator degradation, failures and so on. In the ideal case, where everything works correctly, $D = 1$. In this case, in addition to $D = 1$, we also ignore the presence of $f(x_p)$, that is, we set $f(x_p) = 0$. The controlled system then becomes

$$
\begin{aligned}
\dot{x}_p &= A_p x_p + B_p u, \\
y_p &= C_p^T x_p.
\end{aligned} \tag{3}
$$

(a) For the system (3), design a linear state-feedback controller such that all signals in the closed loop are bounded and the output $y_p$ tracks the output $y_m$ of a reference model

$$
\begin{aligned}
\dot{x}_m &= A_m x_m + B_m r, \\
y_m &= C_m^T x_m.
\end{aligned} \tag{4}
$$

The controller should have proportional-integral characteristics. For this purpose, the error $e_y \triangleq y_p - r$ is integrated by forming $e_{yI} \triangleq \int_0^t e_y(\tau) d\tau$.

(b) In the non-ideal case, where $0 < D < 1$ and $f(x_p)$ is not identically zero, modify the controller of part (a) by adding an extra adaptive control stage, so that the same goal is achieved.
