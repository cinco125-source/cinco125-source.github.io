---
title: "SINDy + MPC for Multirotor Collision Avoidance — When Data-Driven Identification Meets Predictive Control"
date: 2026-05-06 14:30:00 +0900
categories: [Paper Notes, Control]
tags: [sindy, mpc, multirotor, system-identification, data-driven, collision-avoidance]
math: true
mermaid: false
image:
  path: /assets/img/posts/sindy-mpc/fig0_hero.png
  alt: SINDy + MPC for multirotor collision avoidance — overview
---

> **Paper**: Lee et al., *Sparse Identification of Nonlinear Dynamics‐Based Model Predictive Control for Multirotor Collision Avoidance*, **IET Control Theory & Applications**, 2025.
> DOI: [10.1049/cth2.70049](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/cth2.70049) · arXiv: [2412.06388](https://arxiv.org/abs/2412.06388)
>
> *한국어 버전은 [여기](/posts/sindy-mpc-multirotor-collision-avoidance/).*

## Why this paper?

Strap a payload to a multirotor and the vehicle is no longer the one you modeled. Mass and inertia change, aerodynamic disturbances enter, and the controller starts to wobble. Now demand that the same vehicle **track a reference trajectory while avoiding obstacles** — and the problem becomes genuinely interesting.

This paper attacks the problem with a **data-driven identification + model predictive control (MPC)** combo. The identifier of choice is **SINDy (Sparse Identification of Nonlinear Dynamics)**.

## Why SINDy?

There is no shortage of ways to learn dynamics from data — neural networks, Gaussian processes, Koopman embeddings. They share one drawback: the resulting model is a **black box**. You cannot easily say *which* term in the dynamics caused a given output, nor can you give the model a physical meaning.

**SINDy is different.**

- It assumes that the governing equations of the system are sparse — only a small number of terms matter.
- A candidate function library $\Psi$ is regressed against state derivatives, and **L1 regularization** keeps only the meaningful coefficients.
- The result is a human-readable ODE that requires modest amounts of data.

For safety-critical aerospace applications, the **interpretability** of SINDy is a real asset.

![SINDy concept](/assets/img/posts/sindy-mpc/fig1_sindy_concept.png){: w="800" }
_Fig. 1. The SINDy regression viewed as a matrix equation. The candidate library $\Psi(X,U)$ is multiplied by a sparse coefficient matrix $\Sigma$ to reproduce the state derivatives $\dot X$. L1 regularization drives most entries of $\Sigma$ to zero, leaving only physically meaningful terms._

## System model: a multirotor with payload

The 6-DOF dynamics considered in the paper are

$$
\dot{\eta} = \nu, \qquad
\dot{\nu} = \tfrac{1}{m_t}(R_{BI} f_t + f_a) - g
$$

$$
\dot{\Omega} = R_\omega \omega, \qquad
\dot{\omega} = I_t^{-1}\left(\tau_t + \tau_a - \omega \times I_t \omega \right)
$$

The catch is that the **payload changes mass and inertia**:

- $m_t = m_m + m_p$
- $I_t = I_m + I_p$

Aerodynamic effects are modeled as linear damping:

- $f_a = [-K_F \dot{x},\ -K_F \dot{y},\ -K_F \dot{z}]^\top$
- $\tau_a = [-K_{M,p}\, p,\ -K_{M,q}\, q,\ -K_{M,r}\, r]^\top$

The **practical issue** is that $m_p$, $I_p$, $K_F$, and $K_M$ are all unknown. They must be estimated from data.

## SINDy in one equation

Stack the state snapshots into $X$ and their numerical derivatives into $\dot X$. Then regress

$$
\dot X = \Psi(X,U)\, \Sigma
$$

with an L1 penalty on the coefficient matrix:

$$
\Sigma^\star = \arg\min_\Sigma \, \| \dot X - \Psi(X,U)\Sigma \|_2^2 + \lambda \|\Sigma\|_1
$$

Larger $\lambda$ produces a sparser $\Sigma$, leaving only the dominant dynamics terms.

### Embedding physical priors in the library

A clever element of the paper is that **physics-informed terms** are baked into the candidate library:

$$
\Psi_{\text{tr}} = \big[\,\Psi_{\text{prior}}(\text{thrust, gravity}) \;\big|\; \Psi_{\text{poly}}(\text{velocity polynomials})\,\big]
$$

$$
\Psi_{\text{ro}} = \big[\,\Psi_{\text{prior}}(\text{control, rotational coupling}) \;\big|\; \Psi_{\text{poly}}(\text{angular-rate polynomials})\,\big]
$$

This guides the regression toward physically plausible solutions even with limited data.

## Where does the data come from?

Hovering yields almost no information. The authors instead

1. fly a **PID baseline controller** along a rectangular reference trajectory,
2. log states and inputs for 100 s at $\Delta t = 0.002\,\text{s}$,
3. cover a wide range of attitudes and velocities.

Translation and rotation models are then identified **separately**.

## SINDy-MPC: closing the loop with the identified model

The identified $\dot x = \hat f(x,u)$ is plugged directly into an MPC formulation:

$$
\min_{u_1,\dots,u_N}\; \sum_{i=1}^{N} \big[\, (r_i - y_i)^\top Q (r_i - y_i) + u_i^\top R\, u_i \,\big]
$$

subject to

- $\dot x = \hat f(x,u)$ — SINDy-identified dynamics,
- $u_{\min} \le u \le u_{\max}$ — input bounds,
- $x_1 = x_{\text{init}}$ — initial state,
- $\sqrt{(x_{\text{ob}}-x)^2 + (y_{\text{ob}}-y)^2 + (z_{\text{ob}}-z)^2} \ge D_{\min}$ — **collision avoidance**.

The last constraint is what makes this work: a simple distance inequality forces the optimizer to bend the trajectory around obstacles.

![SINDy-MPC architecture](/assets/img/posts/sindy-mpc/fig2_block_diagram.png){: w="900" }
_Fig. 2. The full SINDy-MPC architecture. (Top) Offline, baseline-PID flight data feeds an L1-regularized regression that identifies $\hat f$. (Bottom) Online, that identified model goes straight into the MPC, which closes the loop while honoring the collision-avoidance constraint._

## Identification accuracy (Tables II–III)

The paper reports true vs identified coefficients for the dominant terms of both translational and rotational dynamics. The key entries:

| Term | True $\Sigma$ | Identified $\hat\Sigma$ | Error |
|---|---:|---:|---:|
| $R_{(1,3)} F_B \to \dot{x}$ | 0.7692 | 0.7707 | 0.20% |
| $R_{(2,3)} F_B \to \dot{y}$ | 0.7692 | 0.7704 | 0.16% |
| $L \to \dot{p}$ | 32.258 | 32.248 | 0.03% |
| $M \to \dot{q}$ | 26.316 | 26.316 | 0.00% |
| $N \to \dot{r}$ | 15.873 | 15.879 | 0.04% |

Every dominant coefficient is recovered to within 1%. The exception is yaw aerodynamic damping ($-K_{M,r} r$), where the rectangular reference simply does not excite yaw enough to identify the term reliably (>30% error). **Excitation is part of the design problem.**

## Closed-loop performance (Table IV)

![Quantitative results](/assets/img/posts/sindy-mpc/fig4_quantitative.png){: w="900" }
_Fig. 3. (a) Identification error for the dominant coefficients — all under 1%. (b) Closed-loop position RMSE — SINDy-MPC outperforms nominal-model MPC on every axis, with the largest improvement on the z axis (36% reduction, $0.628 \to 0.399$)._

| RMSE [m] | $x$ | $y$ | $z$ |
|---|---:|---:|---:|
| **SINDy-MPC** | **0.813** | **0.381** | **0.399** |
| Nominal-model MPC | 0.909 | 0.452 | 0.628 |

The numbers tell a clear story. The nominal-model MPC, unaware of the payload and aerodynamic effects, accumulates large altitude error, while the SINDy-identified model brings the closed-loop tracking back in line.

![Trajectory schematic](/assets/img/posts/sindy-mpc/fig3_trajectory.png){: w="700" }
_Fig. 4. Top-down view of trajectory tracking with collision avoidance. With a rectangular reference and two $D_{\min}$ keep-out zones, SINDy-MPC (blue) stays close to the reference, while the nominal-model MPC (light red) drifts due to model mismatch._

- The vehicle tracks the reference under payload uncertainty and unmodeled aerodynamics.
- The MPC keeps a safe distance from each obstacle while completing the trajectory.

## Contributions in one paragraph

1. A **meaningful multirotor model** that explicitly accounts for payload and aerodynamic uncertainty.
2. The **first SINDy + MPC framework** specifically targeted at multirotor collision avoidance.
3. Embedding **physical priors** in the candidate library to combine **data efficiency** with **model interpretability**.

## My take

In a world increasingly dominated by neural identifiers, this paper is a reminder that **sparsity and interpretability** still matter — especially in aerospace, where being able to explain *why* a model produces a given output is half the value.

The honest limitations:

- The result depends heavily on the chosen library — domain knowledge is unavoidable.
- Data distribution matters; the yaw case shows what happens when it does not.
- Online re-identification and adaptive SINDy are natural follow-up directions.

A real-flight implementation, plus an adaptive SINDy variant for changing payloads, would be the obvious next step.

## References

- Original (IET CTA): <https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/cth2.70049>
- arXiv preprint: <https://arxiv.org/abs/2412.06388>
- Brunton, S. L., Proctor, J. L., & Kutz, J. N. (2016). *Discovering governing equations from data by sparse identification of nonlinear dynamical systems*. PNAS.
- Kaiser, E., Kutz, J. N., & Brunton, S. L. (2018). *Sparse identification of nonlinear dynamics for model predictive control in the low-data limit*. Proc. R. Soc. A.

## Figure provenance

All figures in this post are schematic illustrations drawn by the author for the purpose of summarizing the paper. They are *not* reproduced from the original publication; please consult the paper for the official figures.

---

*Corrections welcome. As always, the paper itself is the best source.*
