# USBL "Global" Factor

## Covariance derivation

We re-write the measurement model to simplify the notation a bit in the
following form

\begin{equation}
\underbrace{{}^W\mathbf{t}_{B/W}}_{\mathbf{t}} =
  \underbrace{{}^W\mathbf{t}_{L/W}}_{\mathbf{t}_1} +
  \underbrace{{}^{W}_{L}\mathbf{R}}_{\mathbf{R}_1}
    \underbrace{{}^L\mathbf{t}_{L_u/L}}_{\mathbf{t}_{L_x}} +
  \underbrace{{}^{W}_{B}\mathbf{R}}_{\mathbf{R}} \left(
    \underbrace{{}^{B}_{B_u}\mathbf{R}}_{\mathbf{R}_{B_x}}
      \underbrace{{}^{B_u}\mathbf{t}_{B_u/L_u}}_{-\mathbf{t}_z} +
    \underbrace{{}^{B}\mathbf{t}_{B/B_u}}_{-\mathbf{t}_{B_x}}
  \right),
\end{equation}

which yields (after factorizing the two minus signs):

\begin{equation}
\mathbf{t} =
  \mathbf{t}_1 +
  \mathbf{R}_1
    \mathbf{t}_{L_x} -
  \mathbf{R} \left(
    \mathbf{R}_{B_x}
      \mathbf{t}_z +
    \mathbf{t}_{B_x}
  \right).
\end{equation}

We want to model this in a probabilistic way, so we model translations and
rotations as

\begin{align*}
\mathbf{t} &= \mathbf{\bar{t}} + \delta \mathbf{t} \\
\mathbf{R} &= \mathbf{\bar{R}} \text{Exp}(\delta \boldsymbol{\theta}).
\end{align*}

We model everything, except for the extrinsics, as random variables, so we get
the following:

\begin{align}
\mathbf{\bar{t}} + \delta\mathbf{t} =&
  \left( \mathbf{\bar{t}}_1 + \delta\mathbf{t}_1 \right) +
  \mathbf{\bar{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{t}_{L_x} -
  \mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta}) \left(
    \mathbf{R}_{B_x} (\mathbf{\bar{t}}_z + \delta\mathbf{t}_z) +
    \mathbf{t}_{B_x}
  \right)\\
=&
  \left( \mathbf{\bar{t}}_1 + \delta\mathbf{t}_1 \right) \\
&\text{(a) ...}+ \mathbf{\bar{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{t}_{L_x}\\
&\text{(b) ...}- \mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta})
    \mathbf{R}_{B_x} \mathbf{\bar{t}}_z \\
&\text{(c) ...}- \mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta})
    \mathbf{R}_{B_x} \delta\mathbf{t}_z  \\
&\text{(d) ...}- \mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta})
    \mathbf{t}_{B_x}.
\end{align}

We now keep expanding each individual term using the approximation
$\text{Exp}(\delta\boldsymbol{\theta}) \approx I +
\delta\boldsymbol{\theta}^\wedge$, which yields:

\begin{align*}
  \text{(a)} = \mathbf{\bar{R}}_1
     \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{t}_{L_x} &\approx
  \mathbf{\bar{R}}_1
     (I + \delta\boldsymbol{\theta}_1^\wedge) \mathbf{t}_{L_x} \\
  &= (\mathbf{\bar{R}}_1
      + \mathbf{\bar{R}}_1 \delta\boldsymbol{\theta}_1^\wedge)
      \mathbf{t}_{L_x} \\
  &= \mathbf{\bar{R}}_1 \mathbf{t}_{L_x}
      + \mathbf{\bar{R}}_1 \delta\boldsymbol{\theta}_1^\wedge \mathbf{t}_{L_x}
\end{align*}

\begin{align*}
\text{(b)} = -\mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta})
    \mathbf{R}_{B_x} \mathbf{\bar{t}}_z &\approx
  -\mathbf{\bar{R}} (I + \delta\boldsymbol{\theta}^\wedge)
    \mathbf{R}_{B_x} \mathbf{\bar{t}}_z \\
 &=  -\mathbf{\bar{R}} \mathbf{R}_{B_x} \mathbf{\bar{t}}_z
  -\mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
    \mathbf{R}_{B_x} \mathbf{\bar{t}}_z \\
\end{align*}

\begin{align*}
\text{(c)} = - \mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta})
    \mathbf{R}_{B_x} \delta\mathbf{t}_z &\approx
    - \mathbf{\bar{R}} (I + \delta\boldsymbol{\theta}^\wedge)
    \mathbf{R}_{B_x} \delta\mathbf{t}_z \\
 &= - \mathbf{\bar{R}} \mathbf{R}_{B_x} \delta\mathbf{t}_z
    - \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
    \mathbf{R}_{B_x} \delta\mathbf{t}_z
\end{align*}

\begin{align*}
\text{(d)} = - \mathbf{\bar{R}} \text{Exp}(\delta\boldsymbol{\theta})
    \mathbf{t}_{B_x} &\approx
  - \mathbf{\bar{R}} (I + \delta\boldsymbol{\theta}^\wedge)
    \mathbf{t}_{B_x} \\
  &= - \mathbf{\bar{R}} \mathbf{t}_{B_x}
  - \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge \mathbf{t}_{B_x}
\end{align*}

Substituting the expanded terms back into out $\mathbf{\bar{t}} +
\delta\mathbf{t}$ equation, we get:

\begin{align}
\mathbf{\bar{t}} + \delta\mathbf{t} =&
   \underbrace{\mathbf{\bar{t}}_1}_{\text{mean}} +
   \overbrace{\delta\mathbf{t}_1}^{\text{noise}} +
   \underbrace{\mathbf{\bar{R}}_1 \mathbf{t}_{L_x}}_{\text{mean}} +
   \overbrace{\mathbf{\bar{R}}_1 \delta\boldsymbol{\theta}_1^\wedge
     \mathbf{t}_{L_x}}^{\text{noise}} -
   \underbrace{\mathbf{\bar{R}} \mathbf{R}_{B_x} \mathbf{\bar{t}}_z}_{\text{mean}} -
   \overbrace{\mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{R}_{B_x} \mathbf{\bar{t}}_z}^{\text{noise}} \text{ ...} \\
     &\text{...}-
   \overbrace{\mathbf{\bar{R}} \mathbf{R}_{B_x} \delta\mathbf{t}_z -
   \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{R}_{B_x} \delta\mathbf{t}_z}^{\text{noise}} -
   \underbrace{\mathbf{\bar{R}} \mathbf{t}_{B_x}}_{\text{mean}} -
   \overbrace{\mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{t}_{B_x}}^{\text{noise}},
\end{align}

and separating them by noise and mean terms, we can arrange it in the following
way:
\begin{align}
\mathbf{\bar{t}} + \delta\mathbf{t} =&
  \underbrace{
   \mathbf{\bar{t}}_1 +
   \mathbf{\bar{R}}_1 \mathbf{t}_{L_x} -
   \mathbf{\bar{R}} \mathbf{R}_{B_x} \mathbf{\bar{t}}_z -
   \mathbf{\bar{R}} \mathbf{t}_{B_x}}_{\mathbf{\bar{t}}} \text{...}\\
   &\text{...}+
  \underbrace{
   \delta\mathbf{t}_1 +
   \mathbf{\bar{R}}_1 \delta\boldsymbol{\theta}_1^\wedge
     \mathbf{t}_{L_x} -
   \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{R}_{B_x} \mathbf{\bar{t}}_z
     -
   \mathbf{\bar{R}} \mathbf{R}_{B_x} \delta\mathbf{t}_z -
     \overbrace{
   \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{R}_{B_x} \delta\mathbf{t}_z}^{\text{cross term}} -
   \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{t}_{B_x}
  }_{\delta\mathbf{t}}.
\end{align}

For covariance purposes, we are interested solely in the noise term. Using the
following skew-symmetric matrix operator properties

\begin{align}
  \boldsymbol{\theta}^\wedge \mathbf{t} &\triangleq -\mathbf{t}^\wedge \boldsymbol{\theta}, \text{ and } \\
  \left(\boldsymbol{\theta}^\wedge\right)^\top &\triangleq - \boldsymbol{\theta}^\wedge,
\end{align}

we factorize the expression with respect to all our noise variables
$\delta(\cdot)$ (throwing away the cross-terms, since they'll disappear anyway):

\begin{align}
\delta\mathbf{t} &=
   \delta\mathbf{t}_1 +
   \mathbf{\bar{R}}_1 \delta\boldsymbol{\theta}_1^\wedge
     \mathbf{t}_{L_x} -
   \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{R}_{B_x} \mathbf{\bar{t}}_z
     -
   \mathbf{\bar{R}} \mathbf{R}_{B_x} \delta\mathbf{t}_z -
   \mathbf{\bar{R}} \delta\boldsymbol{\theta}^\wedge
     \mathbf{t}_{B_x} \\
 &=
   \delta\mathbf{t}_1 -
   \mathbf{\bar{R}}_1
     \mathbf{t}_{L_x}^\wedge \delta\boldsymbol{\theta}_1 +
   \mathbf{\bar{R}}
     \left( \mathbf{R}_{B_x} \mathbf{\bar{t}}_z \right)^\wedge
     \delta\boldsymbol{\theta}
     -
   \mathbf{\bar{R}} \mathbf{R}_{B_x} \delta\mathbf{t}_z +
   \mathbf{\bar{R}}
     \mathbf{t}_{B_x}^\wedge \delta\boldsymbol{\theta} \\
 &=
   \delta\mathbf{t}_1
     - \mathbf{\bar{R}}_1
     \mathbf{t}_{L_x}^\wedge \delta\boldsymbol{\theta}_1
     - \mathbf{\bar{R}} \mathbf{R}_{B_x} \delta\mathbf{t}_z
   + \left(
   \mathbf{\bar{R}}
     \left( \mathbf{R}_{B_x} \mathbf{\bar{t}}_z \right)^\wedge
   +
   \mathbf{\bar{R}}
     \mathbf{t}_{B_x}^\wedge
   \right) \delta\boldsymbol{\theta}.
\end{align}

To get the covariance, we must compute $\mathbb{E}[\delta\mathbf{t}
\delta\mathbf{t}^\top]$. To do so, we exploit the fact that all measurements are
uncorrelated, so the cross-terms go away, yielding:

\begin{align*}
  \mathbb{E}[\delta\mathbf{t}\delta\mathbf{t}^\top] &=
    \mathbb{E}[\delta\mathbf{t}_1 \delta\mathbf{t}_1^\top] \\
  &- \mathbb{E}[\delta\mathbf{t}_1 \delta\boldsymbol{\theta}_1^\top]
      \left( \mathbf{\bar{R}}_1 \mathbf{t}_{L_x}^\wedge \right)^\top \\
  &- \left( \mathbf{\bar{R}}_1 \mathbf{t}_{L_x}^\wedge \right)
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\mathbf{t}_1^\top] \\
  &+ \left( \mathbf{\bar{R}}_1 \mathbf{t}_{L_x}^\wedge \right)
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{\theta}_1^\top]
      \left( \mathbf{\bar{R}}_1 \mathbf{t}_{L_x}^\wedge \right)^\top \\
  &+ \left( \mathbf{\bar{R}} \mathbf{R}_{B_x} \right)
  \mathbb{E}[\delta\mathbf{t}_z \delta\mathbf{t}_z^\top]
     \left( \mathbf{\bar{R}} \mathbf{R}_{B_x} \right)^\top \\
  &+
  \left( \mathbf{\bar{R}} \left( \mathbf{R}_{B_x} \mathbf{\bar{t}}_z
         \right)^\wedge + \mathbf{\bar{R}} \mathbf{t}_{B_x}^\wedge
   \right)
  \mathbb{E}[\delta\boldsymbol{\theta} \delta\boldsymbol{\theta}^\top]
  \left( \mathbf{\bar{R}} \left( \mathbf{R}_{B_x} \mathbf{\bar{t}}_z
         \right)^\wedge + \mathbf{\bar{R}} \mathbf{t}_{B_x}^\wedge
   \right)^\top
\end{align*}
