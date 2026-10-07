# Assignment 1: Intelligent and Adaptive Control Systems

27 December 2021

The assignment: [brief.md](brief.md).

## 1 Part (a)

In the ideal case where $D = 1$ and $f(x_p) = 0$, the controlled system becomes:

$$
\begin{aligned}
\dot{x_p} = A_p x_p + B_p u \\
y_p = C_p^T x_p
\end{aligned} \tag{1}
$$

To design a state-feedback controller with the characteristics of a proportional-integral controller, we define the errors:

$$
\begin{aligned}
e_y = y_p - r
\end{aligned} \tag{2}
$$

$$
\begin{aligned}
e_{yI} = \int\limits_0^t e_y({\tau})d{\tau} \Rightarrow \dot{e}_{yI} = y_p - r \Rightarrow e_{yI} = \frac{e_y}{s}
\end{aligned} \tag{3}
$$

and form the augmented state equation:

$$
\begin{aligned}
\dot{x} = Ax + Bu +B_m r
\end{aligned} \tag{4}
$$

with

$$
x = \begin{bmatrix}
e_{yI}^T \\
x_p^T
\end{bmatrix} ,\,
A = \begin{bmatrix}
0 & C_p \\
0 & A_p
\end{bmatrix} ,\,
B = \begin{bmatrix}
0 \\
B_p
\end{bmatrix} ,\,
B_m = \begin{bmatrix}
-I \\
0
\end{bmatrix}
$$

In (4) we add and subtract the term $B k^T x$, where $k$ is an unknown gain matrix:

$$
\begin{aligned}
\dot{x} = Ax + Bu + B_m r + B k^T x - B k^T x
\end{aligned} \tag{5}
$$

or

$$
\begin{aligned}
\dot{x} = \left( A + B k^T \right) x + B \left( u - k^T x \right) + B_m r
\end{aligned} \tag{6}
$$

I choose the controller:

$$
\begin{aligned}
u = \hat{k}^T x
\end{aligned} \tag{7}
$$

where $\hat{k}$ is the estimate of $k$.\
Then (6) becomes:

$$
\begin{aligned}
\dot{x} = A_m x + B \tilde{k}^T x + B_m r
\end{aligned} \tag{8}
$$

where

$$
\tilde{k} = \hat{k} - k
$$

and

$$
A_m = \left( A + B k^T \right)
$$

Based on (8), we consider the reference model:

$$
\begin{aligned}
\dot{x_m} = A_m x_m + B_m r \\
y_m = C_m^T x_m
\end{aligned} \tag{9}
$$

We define the state tracking error:

$$
\begin{aligned}
e = x - x_m
\end{aligned} \tag{10}
$$

Differentiating it and taking the difference between (8) and (9), we get:

$$
\begin{aligned}
\dot{e} = A_m e + B \tilde{k}^T x
\end{aligned} \tag{11}
$$

We consider the Lyapunov function:

$$
\begin{aligned}
V = e^T P e + trace\left(\tilde{k}^T {\Gamma}^{-1} \tilde{k}\right)
\end{aligned} \tag{12}
$$

where ${\Gamma} = {\Gamma}^T > 0$ is a design parameter and $P = P^T > 0$ is the solution of the algebraic Lyapunov equation $P A_m + A_m^T P = -Q$ with $Q = Q^T > 0$. $V$ is positive definite, and differentiating it with respect to time along the solution of (11) we find:

$$
\begin{aligned}
\dot{V} = -e^T Q e + 2 e^T P B \tilde{k}^T x + 2 trace (\tilde{k}^T {\Gamma}^{-1} \dot{\hat{k}} )
\end{aligned} \tag{13}
$$

Using the property $a^T b = trace(b a^T)$, we have:

$$
\begin{aligned}
\dot{V} = -e^T Q e + 2 trace( \tilde{k}^T {\Gamma}^{-1} \dot{\hat{k}} + \tilde{k}^T x e^T P B )
\end{aligned} \tag{14}
$$

If we choose:

$$
\begin{aligned}
\dot{\hat{k}} = - {\Gamma} x e^T P B
\end{aligned} \tag{15}
$$

then (14) becomes:

$$
\begin{aligned}
\dot{V} = -e^T Q e \leq 0
\end{aligned} \tag{16}
$$

Therefore $e , \tilde{k} \in L_{\infty}$. Moreover, $V \geq 0$ and $V$ is a non-increasing function of time, since $\dot{V} \leq 0$. Hence the limit $\lim\limits_{t \to \infty} V = V_{\infty}$ exists. Integrating both sides of (16) from 0 to $\infty$ gives:

$$
\begin{aligned}
\int_0^\infty -e^T Q e \,\mathrm{d}{\tau}= \int_0^\infty \dot{V}\,\mathrm{d}{\tau} = - [ V_{\infty} - V(0) ]
\end{aligned} \tag{17}
$$

Consequently, $e \in L_2$. Also, by assumption $u,r \in L_{\infty}$, so $x_p,x_m \in L_{\infty} \Rightarrow y_p,y_m \in L_{\infty}$ and $e_y,e_{yI} \in L_{\infty}$, hence $x \in L_{\infty}$. From (11) we conclude that $\dot{e} \in L_{\infty}$, since it is an operation on uniformly bounded signals. Therefore, by Barbalat's lemma, since $e \in L_2 \cap L_{\infty}$ and $\dot{e} \in L_{\infty}$, it follows that $\lim\limits_{t \to \infty} e(t) = 0$. Moreover, $\lim\limits_{t \to \infty} \dot{\hat{k}} = 0$, as the product of a uniformly bounded function and a function whose limit is zero. Also, since $\tilde{k} = \hat{k} - k \in L_{\infty}$, it follows that $\hat{k} \in L_{\infty}$.

## 2 Part (b)

In the non-ideal case, where $0 < D < 1$ and $f(x_p)$ is not identically zero, the controlled system becomes:

$$
\begin{aligned}
\dot{x_p} = A_p x_p + B_p D ( u + f(x_p)) \\
y_p = C_p^T x_p
\end{aligned} \tag{18}
$$

As in part (a), to design a state-feedback controller with the characteristics of a proportional-integral controller, we define the errors:

$$
\begin{aligned}
e_y = y_p - r
\end{aligned} \tag{19}
$$

$$
\begin{aligned}
e_{yI} = \int\limits_0^t e_y({\tau})d{\tau} \Rightarrow \dot{e}_{yI} = y_p - r \Rightarrow e_{yI} = \frac{e_y}{s}
\end{aligned} \tag{20}
$$

and form the augmented state equation:

$$
\begin{aligned}
\dot{x} = Ax + B D ( u + f(x_p)) +B_m r
\end{aligned} \tag{21}
$$

with

$$
x = \begin{bmatrix}
e_{yI}^T \\
x_p^T
\end{bmatrix} ,\,
A = \begin{bmatrix}
0 & C_p \\
0 & A_p
\end{bmatrix} ,\,
B = \begin{bmatrix}
0 \\
B_p
\end{bmatrix} ,\,
B_m = \begin{bmatrix}
-I \\
0
\end{bmatrix}
$$

or

$$
\begin{aligned}
\dot{x} = Ax + B D ( u + \theta^T {\Phi}(x_p) ) +B_m r
\end{aligned} \tag{22}
$$

In (22) we add and subtract the term $B D k^T x$, where $k$ is an unknown gain matrix:

$$
\begin{aligned}
\dot{x} = Ax + B D ( u + \theta^T {\Phi}(x_p) ) +B_m r + B D k^T x - B D k^T x
\end{aligned} \tag{23}
$$

or

$$
\begin{aligned}
\dot{x} = \left( A + B D k^T \right) x + B D \left( u - k^T x + \theta^T {\Phi}(x_p) \right) + B_m r
\end{aligned} \tag{24}
$$

I choose the controller:

$$
\begin{aligned}
u = \hat{k}^T x - \hat{\theta}^T {\Phi}(x_p)
\end{aligned} \tag{25}
$$

where $\hat{k}$ is the estimate of $k$ and $\hat{\theta}$ the estimate of $\theta$.\
Then (24) becomes:

$$
\begin{aligned}
\dot{x} = A_m x + B D (\tilde{k}^T x - \tilde{\theta}^T {\Phi}(x_p) ) + B_m r
\end{aligned} \tag{26}
$$

where

$$
\tilde{k} = \hat{k} - k ,\, \tilde{\theta} = \hat{\theta} - \theta
$$

and

$$
A_m = \left( A + B D k^T \right)
$$

Based on (26), we consider the reference model:

$$
\begin{aligned}
\dot{x_m} = A_m x_m + B_m r \\
y_m = C_m^T x_m
\end{aligned} \tag{27}
$$

We define the state tracking error:

$$
\begin{aligned}
e = x - x_m
\end{aligned} \tag{28}
$$

Differentiating it and taking the difference between (26) and (27), we get:

$$
\begin{aligned}
\dot{e} = A_m e + B D (\tilde{k}^T x - \tilde{\theta}^T {\Phi}(x_p) )
\end{aligned} \tag{29}
$$

We consider the Lyapunov function:

$$
\begin{aligned}
V = e^T P e + trace\left(\tilde{k}^T {\Gamma}_x^{-1} \tilde{k} D\right) + trace\left(\tilde{\theta}^T {\Gamma}_\theta^{-1} \tilde{\theta} D\right)
\end{aligned} \tag{30}
$$

where $`{\Gamma}_x = {\Gamma}_x^T > 0`$ and $`{\Gamma}_\theta = {\Gamma}_\theta^T > 0`$ are design parameters and $P = P^T > 0$ is the solution of the algebraic Lyapunov equation $P A_m + A_m^T P = -Q$ with $Q = Q^T > 0$. Since $D > 0$, $V$ is positive definite, and differentiating it with respect to time along the solution of (29) we find:

$$
\begin{aligned}
\dot{V} = -e^T Q e + 2 e^T P B D ( \tilde{k}^T x - \tilde{\theta}^T {\Phi}(x_p) ) + 2 trace (\tilde{k}^T {\Gamma}_x^{-1} \dot{\hat{k}} D ) + 2 trace (\tilde{\theta}^T {\Gamma}_\theta^{-1} \dot{\hat{\theta}} D )
\end{aligned} \tag{31}
$$

Using the property $a^T b = trace(b a^T)$, we have:

$$
\begin{aligned}
\dot{V} = -e^T Q e + 2 trace( \tilde{k}^T {\Gamma}_x^{-1} \dot{\hat{k}} D + \tilde{k}^T x e^T P B D ) + 2 trace( \tilde{\theta}^T {\Gamma}_\theta^{-1} \dot{\hat{\theta}} D - \tilde{\theta}^T {\Phi}(x_p) e^T P B D )
\end{aligned} \tag{32}
$$

If we choose:

$$
\begin{aligned}
\dot{\hat{k}} = - {\Gamma}_x x e^T P B \\
\dot{\hat{\theta}} = {\Gamma}_\theta {\Phi}(x_p) e^T P B
\end{aligned} \tag{33}
$$

then (32) becomes:

$$
\begin{aligned}
\dot{V} = -e^T Q e \leq 0
\end{aligned} \tag{34}
$$

Therefore $e , \tilde{k} , \tilde{\theta} \in L_{\infty}$. We observe that (34) has the same form as (16) of part (a), so the results found in part (a) still hold (all signals in the closed loop are bounded and the state tracking error converges to zero), and also $\lim\limits_{t \to \infty} \dot{\hat{\theta}} = 0$, as the product of a uniformly bounded function and a function whose limit is zero.
