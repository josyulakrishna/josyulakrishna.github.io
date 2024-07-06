---
title:  "Notes on non-holonomic control theory"
categories: [control, differential geometry]
---

<script type="text/javascript" src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
<script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    TeX: { equationNumbers: { autoNumber: "all" } }
  });
</script>
<!-- 
* [Introduction](#Introduction)
* [Integrability](#Integrability)
* [Vector Fields](#Vector-Fields)
* [Lie Brackets](#Lie-Brackets)
* [Covectors](#Covectors)
* [Controllability](#Controllability)
* [Reference](#Reference) -->

## Introduction 


A Given system of Pffafian constraints of the form 

$$ J(\theta,x) \theta' = G^T(\theta, x)$$ 

We convert it into a form 

$$\omega_i (q) q' = 0$$ 

where $$q=(\theta, x ) \in \mathbb{R}^n, i=1..k $$ is the configuration of the system.
where the $$\omega_i(q)$$ are row vectors. We assume that the ωi are linearly independent at each point $$q \in \mathbb{R}^n$$ and each \omega_i describes one constraint on the directions in which $$q'$$ is permitted to take values.


A holonomic system restricts the motion on a smooth hypersurface (manifold), Locally a manifold can be represented by a set of equations of the form  

$$ h_i(q) = 0 $$ 

The dimension of the manifold on which this system evolves is $$n-k$$. The system is *integrable* if there exists functions $$h_i: \mathbb{R}^n \to \mathbb{R},   i=1..k$$ such that 

$$h_i(q(t))=0 \iff \omega_i(q)q'=0,  i=1..k$$

An integrable Pffafian constraint is called a holonomic 
constraint. Its called *nonholonomic* if its not equivalent to 
a holonomic sytem.

Therefore the question now is, given a nonholonomic system is there a path between $$q_0$$ to $$q_f$$. This is called reachability.



## Integrability

A system is integrable, if we can find a function $$h:\mathbb{R}^n \to \mathbb{R}$$ such that 

$$\omega(q)q'=0 \iff  h(q)=0$$

Lets take a single constraint on the velocity and differntiate $$h(q)$$ it can be written as 

 $$\sum_{j=1}^{n} \omega_j(q)q_j' =0 \iff \frac{\partial h(q)}{\partial q_j}q_j'=0$$ 
 
 This implies there is a function $$\alpha(q)$$ satisfying 
 
 $$ \alpha(q)\omega_j(q) = \frac{\partial h(q)}{\partial q_j}    \ \ \forall j=1..n$$ 
 
 
Thus, a single Pfaffian constraint is holonomic if and only if there exists an integrating factor $$\alpha(q)$$ such that $$\alpha(q) \omega(q)$$ is the derivative of some function $$h$$.

An integrable constraint restricts the motion of a system to level sets of $$h$$. 

It will be convenient for us to convert problems with nonholonomic constraints into another form. Roughly speaking, we would like to ex- amine the systems not from the point of view of the constraints (namely, the directions that we cannot move), but rather from the viewpoint of the directions in which we are free to move.

Therefore we re-formulate the problem by choosing a rightnull space for $$\omega(q)$$ denoted by $$g_j(q) \in \mathbb{R}^n$$
$$i=1,...,n−k=:m$$ such that 

$$\omega_i(q)g_j(q) = 0 \ \  i= 1..k, j=1..n-k$$

therefore the allowable trajcetories can be written as 


$$ q' = \sum_{j=1}^{m} g_j(q)u_j$$

that is $$q(t)$$ is a feasible trajectory iff it satisfies the above equation, for some choice of controls $$u(t) \in \mathbb{R}^m$$

## Vector Fields 
A vector field assigns a vector in the tangent space of a manifold, i.e. a smooth map $$f(q) \in T_q\mathbb{R}^n$$,  and is represented as a collumn 


$$f = \begin{bmatrix} f_1(q) \\ f_2(q)\\ . \\ . \\ f_n(q) \end{bmatrix} $$


They can be thought of right hand side of the equation 
$$
\begin{equation}
q' = f(q)
\label{eq:one}
\end{equation}
$$
The rate of change of a smooth function $$V:\mathbb{R}^n \to \mathbb{R} $$ along the flow of $$f$$ is given by 


$$ V' = \frac{\partial V}{\partial q_i}f_i $$ 

Denoted by 

$$
\begin{equation}
L_fV = \frac{\partial V}{\partial q}f(q)
\end{equation}
$$


The flow of a vector field is the solution of the differential equation (9) Specifically the equation $$ \Phi_{t}^{f}(q) $$ represents the state of the system starting from the $$q_0$$ at time $$t=0$$ thus 

$$ \frac{d \Phi_{t}^{f}(q)}{dt} = f(\Phi_{t}^{f}(q)) \ \ \  q \in \mathbb{R}^n$$


A vector field is said to be complete if it is defined for all t. 

## Lie Brackets 
Consider the composition $$ \Phi_{t}^{g1}(q) \circ \Phi_{s}^{g2}(q)$$ gives the composition of flow along  g2 for s-seconds with flow of g1 for t-seconds.  

The Lie bracket is thus the infinitesimal motion that results from flowing around a square defined by two vector fields f and g. 

$$[f,g] = \frac{\partial g}{\partial q}f(q) - \frac{\partial f}{\partial q}g(q) $$

A *distribution* assigns a subspace of the tangent space to each point $$\mathbb{R}^n$$ in a smooth way. A special case of the distribution is given when it is defined by a set of smooth vector fields $$g_1, g_2, .. g_m $$ in which a distribtion is defined as 

$$\Delta_q = span\{g_1(q), g_2(q), .. g_m(q) \}$$

A distribution is called **regular** if the dimension of $$\Delta_q $$ doesn't change with q. Its called **involutive** 
if its closed under a lie bracke, i.e. $$[f,g] \in \Delta \ \ \forall \ f, g \in \Delta$$ the closure operation is denoted by $$\bar{\Delta}$$. That is, $$\bar{\Delta}$$ is the smallest distribution containing $$\Delta$$ such that if $$f, g \in \bar{\Delta}$$  then $$[f, g]  \in  \bar{\Delta}$$.

## Lie Algebra 

A vectorspace V (over $$\mathbb{R}$$) is a Lie Algebra if there exists a bilinear operation $$ V \times V \to V$$  denoted by $$[,]$$ satisfying (i)Skew Symmetry $$([f,g] = -[g,f])$$ and (ii) Jacobi Identity. The set of smooth vector fields on Rn with the Lie bracket is a Lie algebra and is denoted $$ \mathscr{X} $$ $$ (\mathbb{R}^n)$$. For a set of smooth fields $$ g_1, g_2, .. g_m $$, the involutive closure $$\bar{\Delta}$$ is a Lie algebra, called the lie algebra generate by $$ g_1, g_2, .. g_m $$ and is often denoted by $$ \mathscr{L}(g_1, g_2, .. g_m )$$

A distribution $$\Delta$$  of constant dimension k, is said to be integrable if there exists a set of smooth function $$  h_i: \mathbb{R}^n \to \mathbb{R}$$ such that the row vectors $$\frac{\partial h_i}{\partial q}$$ are linearly independent for at q for every $$ f \in \Delta $$. 

The hypersurfaces given by 

$$\{q: h_1(q)=c_1, .. h_{n-k}(q)=c_{n-k} \}$$ 

are called integral manifolds. 

**Frobenius Theorem**: A regular distribution is integrable if and only if it is involutive.

Thus if $$\Delta$$ is integrable then there are $$n-k$$ functions defined locally $$h_i:\mathbb{R}^n \to \mathbb{R} \ \ i=1..n-k$$
with level surfaces of $$ h= (h_1, .. ,h_{n-k})$$, these level surfaces form a *foliation* of $$ \mathbb{R}^n $$, A single level surface is called a *leaf of foliation*.

## Covectors 
The dual space of $$T_p\mathbb{R}^n$$ is denoted by $$T_p^*\mathbb{R}^n$$, analogus to the vector field a *one-form* is a map which assign to each point $$ q$$ a covector $$\omega(q) \in T_p^*\mathbb{R}^n$$ in local coordinates a one-form is represented by a row-vector 

$$\omega(q) = [\omega_1(q), .. \omega_n(q)]  $$

Differential of a smooth function is an example of an one-form for $$ \beta: \mathbb{R}^n \to \mathbb{R} $$


$$ d\beta = [\frac{\partial{\beta}}{\partial q_1} ... \frac{\partial{\beta}}{\partial q_n}]$$


A codistribution assigns a covector to each point in $$q \in T_p^*\mathbb{R}^n $$ 

$$\Omega = span\{ \omega_1(q), ... \omega_n(q) \}$$ 

For a control problem we need to convert an equation given as one-forms $$ \omega_i(q)q'=0 $$ into an equivalent control system. 

**Annihilator**
For a given set of one-forms $$ \omega_i \ ,\  i=1..k$$ there exists a set of smooth linearly independent vector fields $$g_j \ j= 1.. n-k $$ such that $$\omega_i(q) g_j(q) = 0$$

i.e for $$\Omega_q = span\{ \omega_1(q), ... \omega_n(q) \}$$ there exists a  $$\Delta_q = span\{g_1(q), g_2(q), .. g_{n-k}(q) \}$$ such that $$ \Delta = \Omega^{\perp} $$.  We say that the Distribution $$\Delta $$  annihilates the codistribution $$\Omega$$ 

The control system associated with the distribution is of the form 

$$q' = g_1(q)u_1+..+g_{n-k}u_{n-k} $$ 

Can be written since $$\omega(q)q'=0 $$  and $$ g_1.. g_{n-k}$$ is an annihilator (rightnull space of $$\omega(q)$$)


## Controllability
Consider the control systems of the form


$$ \sum :  \ \ \ q' = g_1(q)u_1+..+g_{m}u_{m} \\ U \in \mathbb{R}^m \\  q \in \mathbb{R}^n $$ 


This system is said to be drift-free when under zero input u the system doesn't drift. The $$g_i,\ i=1..m$$ are assumed to be linearly independent vector fields on $$\mathbb{R}^n$$

A system $$\sigma$$ is said to be controllable if for any  $$q_0, q_f$$ there exists a set of controls $$u:[0,T] \to U$$ for T>0 which satisfies $$q(0)=q_0$$ and $$q(T)=q_f$$

Given an openset $$V \subseteq \mathbb{R}^n$$ define $$\mathscr{R}^v(q_0, t)$$ to be the set of states q that there exists $$u:[0,T ] \to U$$ that steers $$\sum$$  from $$q(0)=q_0$$ to $$q(T)=q_f$$  and satisfies $$ q(t) \subseteq V$$ for $$ 0 \leqslant t \leqslant T$$ and define the set of states reachable up to time T  to be 

$$\mathscr{R}^v(q_0, \leqslant T)  = \bigcup_{0<\tau \leqslant T}\mathscr{R}^v(q_0, \tau)$$ 

Let $$ \bar{\Delta} = \mathscr{L}(g1,...,gm) $$ be the Lie algebra generated by $$g_1,...,g_m$$. Referred to as the the controllability Lie algebra. A sequence of inputs 

$$ u_1 = +1 \  u_2 = 0 \ \  for\  0\leqslant t \leqslant \epsilon  
\\
u_1 = 0 \  u_2 = +1 \ \  for \ \epsilon \leqslant t \leqslant 2\epsilon 
\\
u_1 = -1 \  u_2 = 0 \ \  for \  2\epsilon \leqslant t \leqslant 3\epsilon 
\\
u_1 = 0 \  u_2 = -1 \ \  for \  3\epsilon \leqslant t \leqslant 4\epsilon 
$$
 
 Generates motion in the direction of Lie Bracket $$[g_1, g_2]$$ If we were to iterate on this sequence, it should be possible to generate motion along directions given by all the other Lie products associated with the $$g_i$$
 
 **Chow's Theorem**: The control system $$\sum$$ is locally controllable at $$q \in \mathbb{R}^n$$ if $$ \Delta_q = T_q\mathbb{R}^n$$.
## Reference
A Mathematical Introduction to Robot Manipulation R.M Murray