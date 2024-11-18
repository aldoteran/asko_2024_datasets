# Optical "Global" Factor

## Measurement Description

Our optical measurements give us an observation of the relative pose between
LoLo's camera frame $L_c$ and the Boat's light fiducial frame $B_l$, taking the
following form:

\begin{equation}
{}^{L_c}_{B_l} \hat{\mathbf{T}}_z = \begin{bmatrix}
  {}^{L_c}_{B_l} \mathbf{R} & {}^{L_c} \mathbf{t}_{B_l / L_c} \\
  0^\top & 1
\end{bmatrix} \in \text{SE}(3).
\end{equation}

## Factor Definition

Our measurement model takes the form

\begin{equation}
  {}^{L_c}_{B_l} \hat{\mathbf{T}} =
    {}^{W}_{L} \mathbf{T} \cdot
    {}^{L}_{L_c} \mathbf{T} \cdot
    {}^{L_c}_{B_l} \mathbf{T} \cdot
    {}^{B_l}_{B} \mathbf{T}
\end{equation}

## Covariance Derivation

To calculate the covariance, we explicitly model our measurement model in a
probabilistic way, with

\begin{align*}
\mathbf{T} &= \bar{\mathbf{T}} \cdot \delta \mathbf{T} \\
  &= \begin{bmatrix}
    \bar{\mathbf{R}} & \bar{\mathbf{t}} \\
    0^\top & 1
  \end{bmatrix}
  \begin{bmatrix}
    \text{Exp}(\delta \boldsymbol{\theta}) & \delta \mathbf{t} \\
    0^\top & 1
  \end{bmatrix}\\\\
  &= \begin{bmatrix}
    \bar{\mathbf{R}} \text{Exp}(\delta \boldsymbol{\theta}) &
    \bar{\mathbf{t}} + \bar{\mathbf{R}} \delta \mathbf{t} \\
    0^\top & 1
  \end{bmatrix}.
\end{align*}

Additionally, we do NOT model our extrinsic transforms as random variables, so
we just take those as evidence. Thus, we get the following pose composition chain:

\begin{align}
  \underbrace{{}^{L_c}_{B_l} \hat{\mathbf{T}}}_{\mathbf{T}} &=
    \underbrace{{}^{W}_{L} \mathbf{T}}_{\mathbf{T}_1} \cdot
    \underbrace{{}^{L}_{L_c} \mathbf{T}}_{\mathbf{T}_2} \cdot
    \underbrace{{}^{L_c}_{B_l} \mathbf{T}}_{\mathbf{T}_3} \cdot
    \underbrace{{}^{B_l}_{B} \mathbf{T}}_{\mathbf{T}_4} \\
\Rightarrow \mathbf{T}\delta\mathbf{T} &=
  \mathbf{T}_1\delta\mathbf{T}_1 \cdot
  \mathbf{T}_2 \cdot
  \mathbf{T}_3\delta\mathbf{T}_3 \cdot
  \mathbf{T}_4.
\end{align}

We compose all poses then to get the following:

\begin{align}
\mathbf{T}\delta\mathbf{T} &=
  \begin{bmatrix}
    \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) &
    \bar{\mathbf{t}}_1 + \bar{\mathbf{R}}_1 \delta \mathbf{t}_1 \\
    0^\top & 1
  \end{bmatrix} \begin{bmatrix}
    \mathbf{R}_2 & \mathbf{t}_2 \\
    0^\top & 1
  \end{bmatrix} \cdot
  \mathbf{T}_3\delta\mathbf{T}_3 \cdot
  \mathbf{T}_4 \\\\
 &= \begin{bmatrix}
    \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2 &
    \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{t}_2 +
    \bar{\mathbf{t}}_1 + \bar{\mathbf{R}}_1 \delta \mathbf{t}_1 \\
    0^\top & 1
  \end{bmatrix} \cdot
  \mathbf{T}_3\delta\mathbf{T}_3 \cdot
  \mathbf{T}_4 \\\\
  &= (\cdots) \cdot
  \begin{bmatrix}
    \bar{\mathbf{R}}_3 \text{Exp}(\delta \boldsymbol{\theta}_3) \mathbf{R}_4 &
    \bar{\mathbf{R}}_3 \text{Exp}(\delta \boldsymbol{\theta}_3) \mathbf{t}_4 +
    \bar{\mathbf{t}}_3 + \bar{\mathbf{R}}_3 \delta \mathbf{t}_3 \\
    0^\top & 1
  \end{bmatrix},
\end{align}

and, since we don't have enough space to write it all out in matrix form, we
just separate the matrix into its rotational and translational components.
Therefore, we end up getting:

\begin{align}
\bar{\mathbf{R}} \text{Exp}(\delta \boldsymbol{\theta}) &=
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2
  \bar{\mathbf{R}}_3 \text{Exp}(\delta \boldsymbol{\theta}_3) \mathbf{R}_4 \\
\bar{\mathbf{t}} + \bar{\mathbf{R}} \delta \mathbf{t} &=
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2
  \left(
    \bar{\mathbf{R}}_3 \text{Exp}(\delta \boldsymbol{\theta}_3) \mathbf{t}_4 +
    \bar{\mathbf{t}}_3 + \bar{\mathbf{R}}_3 \delta \mathbf{t}_3
  \right) +
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{t}_2 +
    \bar{\mathbf{t}}_1 + \bar{\mathbf{R}}_1 \delta \mathbf{t}_1.
\end{align}

### Rotational covariance

For the rotation part of the covariance, we take the rotational expression, and
then try to push all of the exponentials to the RHS of the expression by using
the following useful properties:

\begin{align}
\mathbf{R} \text{Exp}(\delta \boldsymbol{\theta})
  &= \text{Exp}(\mathbf{R} \delta \boldsymbol{\theta}) \mathbf{R}, \text{ and }\\
\text{Exp}(\delta \boldsymbol{\theta}) \mathbf{R}
  &= \mathbf{R} \text{Exp}(\mathbf{R}^\top \delta \boldsymbol{\theta}).\\
\end{align}

We then apply these properties to massage the following expression:

\begin{align}
\bar{\mathbf{R}} \text{Exp}(\delta \boldsymbol{\theta}) &=
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2
  \bar{\mathbf{R}}_3
  \underbrace{\text{Exp}(\delta \boldsymbol{\theta}_3) \mathbf{R}_4} \\
&= \underbrace{\bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1)}
  \mathbf{R}_2 \bar{\mathbf{R}}_3
  \mathbf{R}_4 \text{Exp}(\mathbf{R}_4^\top \delta\boldsymbol{\theta}_3) \\
&= \underbrace{
  \text{Exp}(\bar{\mathbf{R}}_1 \delta\boldsymbol{\theta}_1) \bar{\mathbf{R}}_1
  \mathbf{R}_2 \bar{\mathbf{R}}_3
  \mathbf{R}_4} \text{Exp}(\mathbf{R}_4^\top \delta\boldsymbol{\theta}_3) \\
&= \underbrace{
  \bar{\mathbf{R}}_1 \mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4}_{
    \bar{\mathbf{R}}
  }
  \underbrace{
  \text{Exp}((\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4)^\top
             \delta\boldsymbol{\theta}_1)
  \text{Exp}(\mathbf{R}_4^\top \delta\boldsymbol{\theta}_3)}_{
    \text{Exp}(\delta \boldsymbol{\theta})
  }
\end{align}

### Translational covariance

### Cross-terms covariance


