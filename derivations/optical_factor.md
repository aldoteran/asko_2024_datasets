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
  }.
\end{align}

Ok, so we can express our rotation's noise as:

\begin{align}
  \text{Exp}(\delta \boldsymbol{\theta}) &=
  \text{Exp}((\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4)^\top
             \delta\boldsymbol{\theta}_1)
  \text{Exp}(\mathbf{R}_4^\top \delta\boldsymbol{\theta}_3) \\
  &\approx \text{Exp}((\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4)^\top
             \delta\boldsymbol{\theta}_1 +
  \mathbf{R}_4^\top \delta\boldsymbol{\theta}_3),
\end{align}

then, using the small-angle approximation $\text{Exp}(\delta\boldsymbol{\theta})
\approx I + \delta\boldsymbol{\theta}^\wedge$, we can finally get the
expression:

\begin{align}
  \delta\boldsymbol{\theta} &=
   (\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4)^\top
             \delta\boldsymbol{\theta}_1 +
  \mathbf{R}_4^\top \delta\boldsymbol{\theta}_3.
\end{align}

Now, on to the good part, let's take the expectation. Using the fact that LoLo's
measurements and the optical measurements are uncorrelated, we get the following
expression with only two terms:

\begin{align*}
  \mathbb{E}[\delta\boldsymbol{\theta}\delta\boldsymbol{\theta}^\top]
  =& (\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4)^\top
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{\theta}_1^\top]
     (\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4) \\
  &+
  \mathbf{R}_4^\top
  \mathbb{E}[\delta\boldsymbol{\theta}_3 \delta\boldsymbol{\theta}_3^\top]
  \mathbf{R}_4,
\end{align*}

where $\mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{\theta}_1^\top]$
is the rotational covariance matrix from LoLo's INS measurement, and
$\mathbb{E}[\delta\boldsymbol{\theta}_3 \delta\boldsymbol{\theta}_3^\top]$ is
the rotational covariance matrix from the optical measurement.

### Translational covariance

Now let's deal with the translation part. We can just expand the full
expression:

\begin{align}
\bar{\mathbf{t}} + \bar{\mathbf{R}} \delta \mathbf{t} =&
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2
    \bar{\mathbf{R}}_3 \text{Exp}(\delta \boldsymbol{\theta}_3) \mathbf{t}_4 \\
    &+
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2
    \bar{\mathbf{t}}_3 \\
    &+
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{R}_2
    \bar{\mathbf{R}}_3 \delta \mathbf{t}_3 \\
    &+
  \bar{\mathbf{R}}_1 \text{Exp}(\delta \boldsymbol{\theta}_1) \mathbf{t}_2 \\
    &+
  \bar{\mathbf{t}}_1 \\
    &+
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1,
\end{align}

which we then keep expanding further using the small-angle approximation for the
exponential map to obtain:

\begin{align}
\bar{\mathbf{t}} + \bar{\mathbf{R}} \delta \mathbf{t} \approx&
  \bar{\mathbf{R}}_1 (I + \delta \boldsymbol{\theta}_1^\wedge) \mathbf{R}_2
    \bar{\mathbf{R}}_3 (I + \delta \boldsymbol{\theta}_3^\wedge) \mathbf{t}_4 \\
    &+
  \bar{\mathbf{R}}_1 (I + \delta \boldsymbol{\theta}_1^\wedge) \mathbf{R}_2
    \bar{\mathbf{t}}_3 \\
    &+
  \bar{\mathbf{R}}_1 (I + \delta \boldsymbol{\theta}_1^\wedge) \mathbf{R}_2
    \bar{\mathbf{R}}_3 \delta \mathbf{t}_3 \\
    &+
  \bar{\mathbf{R}}_1 (I + \delta \boldsymbol{\theta}_1^\wedge) \mathbf{t}_2 \\
    &+
  \bar{\mathbf{t}}_1 \\
    &+
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1 \\
 =&
   (\bar{\mathbf{R}}_1 +
     \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge)
     (\mathbf{R}_2\bar{\mathbf{R}}_3 + \mathbf{R}_2\bar{\mathbf{R}}_3
       \delta \boldsymbol{\theta}_3^\wedge) \mathbf{t}_4 \\
    &+
  (\bar{\mathbf{R}}_1 +
    \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge) \mathbf{R}_2
    \bar{\mathbf{t}}_3 \\
    &+
  (\bar{\mathbf{R}}_1 +
   \bar{\mathbf{R}}_1\delta \boldsymbol{\theta}_1^\wedge) \mathbf{R}_2
    \bar{\mathbf{R}}_3 \delta \mathbf{t}_3 \\
    &+
  (\bar{\mathbf{R}}_1 +
    \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge) \mathbf{t}_2 \\
    &+
  \bar{\mathbf{t}}_1 \\
    &+
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1 \\
 =&
 \underbrace{
 \bar{\mathbf{R}}_1 \mathbf{R}_2\bar{\mathbf{R}}_3 \mathbf{t}_4
 }_{\text{mean}}
 +
 \overbrace{
 \bar{\mathbf{R}}_1 \mathbf{R}_2\bar{\mathbf{R}}_3
       \delta \boldsymbol{\theta}_3^\wedge \mathbf{t}_4
  }^{\text{noise}}
 +
 \overbrace{
 \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge
   \mathbf{R}_2\bar{\mathbf{R}}_3 \mathbf{t}_4
  }^{\text{noise}}
 +
 \overbrace{
 \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge
 \mathbf{R}_2\bar{\mathbf{R}}_3
       \delta \boldsymbol{\theta}_3^\wedge \mathbf{t}_4
}^{\text{noise (cross)}} \\
    &+
 \underbrace{
  \bar{\mathbf{R}}_1 \mathbf{R}_2 \bar{\mathbf{t}}_3
 }_{\text{mean}}
  +
 \overbrace{
    \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge
    \mathbf{R}_2 \bar{\mathbf{t}}_3
  }^{\text{noise}} \\
    &+
 \overbrace{
  \bar{\mathbf{R}}_1 \mathbf{R}_2 \bar{\mathbf{R}}_3 \delta \mathbf{t}_3
  }^{\text{noise}}
  +
 \overbrace{
   \bar{\mathbf{R}}_1\delta \boldsymbol{\theta}_1^\wedge \mathbf{R}_2
    \bar{\mathbf{R}}_3 \delta \mathbf{t}_3
  }^{\text{noise (cross)}} \\
    &+
 \underbrace{
  \bar{\mathbf{R}}_1 \mathbf{t}_2
 }_{\text{mean}}
  +
 \overbrace{
    \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge \mathbf{t}_2
  }^{\text{noise}} \\
    &+
 \underbrace{
  \bar{\mathbf{t}}_1
 }_{\text{mean}} \\
    &+
 \overbrace{
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1
  }^{\text{noise}}.
\end{align}

We can now just throw away all the "mean" terms, as well as the "cross" noise
terms (since they'll go away anyways), since we just want to deal with the
perturbation. That leaves us with the expression:

\begin{align}
\bar{\mathbf{R}} \delta \mathbf{t} &=
  \bar{\mathbf{R}}_1 \mathbf{R}_2\bar{\mathbf{R}}_3
       \delta \boldsymbol{\theta}_3^\wedge \mathbf{t}_4
  +
 \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge
   \mathbf{R}_2\bar{\mathbf{R}}_3 \mathbf{t}_4
  +
 \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge
    \mathbf{R}_2 \bar{\mathbf{t}}_3
  +
 \bar{\mathbf{R}}_1 \mathbf{R}_2 \bar{\mathbf{R}}_3 \delta \mathbf{t}_3
  +
 \bar{\mathbf{R}}_1 \delta \boldsymbol{\theta}_1^\wedge \mathbf{t}_2
  +
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1 \\
&=
 - \bar{\mathbf{R}}_1 \mathbf{R}_2\bar{\mathbf{R}}_3
     \mathbf{t}_4^\wedge \delta\boldsymbol{\theta}_3
  -
 \bar{\mathbf{R}}_1
 \left( \mathbf{R}_2\bar{\mathbf{R}}_3 \mathbf{t}_4 \right)^\wedge
  \delta\boldsymbol{\theta}_1
  -
 \bar{\mathbf{R}}_1 \left( \mathbf{R}_2 \bar{\mathbf{t}}_3 \right)^\wedge
   \delta\boldsymbol{\theta}_1
  +
 \bar{\mathbf{R}}_1 \mathbf{R}_2 \bar{\mathbf{R}}_3 \delta \mathbf{t}_3
  -
 \bar{\mathbf{R}}_1 \mathbf{t}_2^\wedge \delta\boldsymbol{\theta}_1
  +
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1 \\
&=
 - \bar{\mathbf{R}}_1 \mathbf{R}_2\bar{\mathbf{R}}_3
     \mathbf{t}_4^\wedge \delta\boldsymbol{\theta}_3
  - \left(
 \bar{\mathbf{R}}_1
 \left(
   \mathbf{R}_2\bar{\mathbf{R}}_3 \mathbf{t}_4 \right)^\wedge +
   \bar{\mathbf{R}}_1 \left( \mathbf{R}_2 \bar{\mathbf{t}}_3 \right)^\wedge +
   \bar{\mathbf{R}}_1 \mathbf{t}_2^\wedge
 \right )
  \delta\boldsymbol{\theta}_1
  +
 \bar{\mathbf{R}}_1 \mathbf{R}_2 \bar{\mathbf{R}}_3 \delta \mathbf{t}_3
  +
  \bar{\mathbf{R}}_1 \delta \mathbf{t}_1.
\end{align}

At this point, I think we can just remove the LHS rotation by multiplying by its
transpose, which will give us

\begin{align}
\delta\mathbf{t} &=
 -
 \underbrace{
 \bar{\mathbf{R}}^\top \bar{\mathbf{R}}_1 \mathbf{R}_2\bar{\mathbf{R}}_3
     \mathbf{t}_4^\wedge
 }_{\triangleq A}
     \delta\boldsymbol{\theta}_3
  -
 \underbrace{
  \bar{\mathbf{R}}^\top \left(
 \bar{\mathbf{R}}_1
 \left(
   \mathbf{R}_2\bar{\mathbf{R}}_3 \mathbf{t}_4 \right)^\wedge +
   \bar{\mathbf{R}}_1 \left( \mathbf{R}_2 \bar{\mathbf{t}}_3 \right)^\wedge +
   \bar{\mathbf{R}}_1 \mathbf{t}_2^\wedge
 \right )
 }_{\triangleq B}
  \delta\boldsymbol{\theta}_1
  +
 \underbrace{
 \bar{\mathbf{R}}^\top \bar{\mathbf{R}}_1
   \mathbf{R}_2 \bar{\mathbf{R}}_3
 }_{\triangleq C}
   \delta\mathbf{t}_3
  +
 \underbrace{
 \bar{\mathbf{R}}^\top \bar{\mathbf{R}}_1
 }_{\triangleq D}
 \delta\mathbf{t}_1,
\end{align}

and then proceed to take the expectation

\begin{align*}
  \mathbb{E}[\delta\mathbf{t}\delta\mathbf{t}^\top]
  =&
  A
  \mathbb{E}[\delta\boldsymbol{\theta}_3 \delta\boldsymbol{\theta}_3^\top]
  A^\top \\
  &+
  A
  \mathbb{E}[\delta\boldsymbol{\theta}_3 \delta\boldsymbol{t}_3^\top]
  C^\top \\
  &+
  C
  \mathbb{E}[\delta\boldsymbol{t}_3 \delta\boldsymbol{t}_3^\top]
  C^\top \\
  &+
  C
  \mathbb{E}[\delta\boldsymbol{t}_3 \delta\boldsymbol{\theta}_3^\top]
  A^\top \\
  &+
  B
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{\theta}_1^\top]
  B^\top \\
  &-
  B
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{t}_1^\top]
  D^\top \\
  &+
  D
  \mathbb{E}[\delta\boldsymbol{t}_1 \delta\boldsymbol{t}_1^\top]
  D^\top \\
  &-
  D
  \mathbb{E}[\delta\boldsymbol{t}_1 \delta\boldsymbol{\theta}_1^\top]
  B^\top,
\end{align*}

which uses LoLo's and the optical measurement's rotational, translational, and
cross-term covariance blocks.

### Cross-terms covariance

This one is pretty easy. We simplify the rotation perturbation expression as

\begin{align}
  \delta\boldsymbol{\theta} &=
   (\mathbf{R}_2 \bar{\mathbf{R}}_3 \mathbf{R}_4)^\top
             \delta\boldsymbol{\theta}_1 +
  \mathbf{R}_4^\top \delta\boldsymbol{\theta}_3 \\
  &=
  \alpha \delta\boldsymbol{\theta}_1 +
  \beta \delta\boldsymbol{\theta}_3,
\end{align}

we can just take the expectation of the cross terms and obtain:

\begin{align*}
  \mathbb{E}[\delta\boldsymbol{\theta}\delta\mathbf{t}^\top]
  =&
  -
  \alpha
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{\theta}_1^\top]
  B^\top \\
  &+
  \alpha
  \mathbb{E}[\delta\boldsymbol{\theta}_1 \delta\boldsymbol{t}_1^\top]
  D^\top \\
  &-
  \beta
  \mathbb{E}[\delta\boldsymbol{\theta}_3 \delta\boldsymbol{\theta}_3^\top]
  A^\top \\
  &+
  \beta
  \mathbb{E}[\delta\boldsymbol{\theta}_3 \delta\boldsymbol{t}_3^\top]
  C^\top.
\end{align*}

