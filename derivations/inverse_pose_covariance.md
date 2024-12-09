# Inverse pose covariance derivation

We define a pose as

\begin{align}
\mathbf{T} = \bar{\mathbf{T}} \delta\mathbf{T} &= \begin{bmatrix}
  \mathbf{R} & \mathbf{t} \\
  0^\top & 1
\end{bmatrix} \begin{bmatrix}
  \text{Exp}(\delta\boldsymbol{\theta}) & \delta\mathbf{t} \\
  0^\top & 1
\end{bmatrix} \\\\
&=
\begin{bmatrix}
  \mathbf{R} \text{Exp}(\delta\boldsymbol{\theta}) &
  \mathbf{R} \delta\mathbf{t} + \mathbf{t} \\
  0^\top & 1
\end{bmatrix}
\in \text{SE}(3),
\end{align}

which then, for an inverse pose, we get

\begin{align}
\mathbf{T}^{-1} = \left( \bar{\mathbf{T}} \delta\mathbf{T} \right)^{-1}
= \begin{bmatrix}
\left(
  \mathbf{R} \text{Exp}(\delta\boldsymbol{\theta})
  \right)^\top &
 - \left(
  \mathbf{R} \text{Exp}(\delta\boldsymbol{\theta})
  \right)^\top \left(\mathbf{R} \delta\mathbf{t} + \mathbf{t}\right) \\
  0^\top & 1
\end{bmatrix}
\in \text{SE}(3).
\end{align}


