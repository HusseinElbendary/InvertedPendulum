## Abstract

This is a simulation project of a physical pendulum mounted on a moving cart. The cart is moving using belt drive that is connected to gearbox and DC motor.
The input of the system is motor voltage ranging from +18v to -18v scaled to +1 to -1. Cart position is in meters and pendulum angle is in radians.
The control and simulation is done using matlab and simulink. 

## System Model
To model this the pendulum on cart, its better to use lagrange equation.
First we obtain the equation of potintial energy of the system which is just the hight of the CoG (center of gravity) of the pendulum
$$V=\frac{1}{2}m_pglcos(\theta)$$
second, we obtain the kinetic energy which consist of three parts
$$T=T_{Cart} + T_{Pendulum\,linear} + T_{Pendulum\,Rotational}$$

Kinetic energy of the cart is just the energy from the linear velocity
$$T_{Cart} = \frac{1}{2}m_cv_c^2 = \frac{1}{2}m_c\dot x^2$$
kinetic energy of the linear motion of the pendulum is the linear velocity of the CoG which is vector sum of rotational speed of the pendulum and the linear speed of the cart
$$T_{Pendulum\,linear} = \frac{1}{2}mv_p^2$$
$$v_p^2 = (\frac{d}{dt}\frac{1}{2}lcos(\theta))^2 + (\frac{d}{dt}(x - \frac{1}{2}lsin(\theta))^2$$
$$v_p^2 = (-\frac{1}{2}lsin(\theta)\dot\theta)^2 + (\dot x - \frac{1}{2}lcos(\theta)\dot\theta)^2$$
$$v_p^2 = \dot x^2 + \frac{1}{4}l^2\dot\theta^2 - \dot xlcos(\theta)\dot\theta$$

kinetic energy of the rotional motion of the pendulum
$$T_{Pendulum\,Rotational} = \frac{1}{2}I_p\dot\theta^2$$
therefore the Lagrangian is equals to
$$L = T - V = \frac{1}{2}I_p\dot\theta^2+\frac{1}{2}m_c\dot x^2 + \frac{1}{2}m_p\dot x^2 + \frac{1}{8}m_pl^2\dot\theta^2 - \frac{1}{2}m_p l\dot xcos(\theta)\dot\theta - \frac{1}{2}m_pglcos(\theta) $$ 

lagrange equation states that $$\frac{d}{dt}\frac{\partial L}{\partial \dot x} -\frac{\partial L}{\partial x} = Q_x$$
$$\frac{d}{dt}\frac{\partial L}{\partial \dot \theta} -\frac{\partial L}{\partial \theta} = Q_\theta$$

thus subtituting and simplifying 
- for variabe $x$
$$\frac{\partial L}{\partial x} = 0$$
$$\frac{\partial L}{\partial \dot x}= (m_c+m_p)\dot x - \frac{1}{2}m_plcos(\theta)\dot\theta$$

$$\frac{d}{dt}\frac{\partial L}{\partial \dot x}=(m_c+m_p)\ddot x- \frac{1}{2}m_pl[cos(\theta)\ddot\theta -sin(\theta)\dot\theta^2] $$
$$Q_x=F(t)-b_c\dot x$$

$$(m_c+m_p)\ddot x- \frac{1}{2}m_pl[cos(\theta)\ddot\theta -sin(\theta)\dot\theta^2] =F(t)-b_c\dot x \dots(1)$$

- for variable $\theta$ 
$$\frac{\partial L}{\partial \theta} = \frac{1}{2}m_p[\dot xlsin(\theta)\dot\theta+lgsin(\theta)]$$

$$\frac{\partial L}{\partial \dot\theta} = I_p\dot\theta-\frac{1}{2}m_p\dot xlcos(\theta)+\frac{1}{4}m_pl^2\dot\theta$$

$$\frac{d}{dt}\frac{\partial L}{\partial \dot\theta} = I_p\ddot\theta-\frac{1}{2}m_p\ddot xlcos(\theta)+\frac{1}{2}m_p\dot xlsin(\theta)\dot\theta+\frac{1}{4}m_pl^2\ddot\theta$$

$$(I_p+\frac{m_pl^2}{4})\ddot\theta-\frac{1}{2}m_pl[\ddot x cos(\theta)+g sin(\theta)] = -b_p\dot\theta \dots(2)$$


(1) and (2) are non-linear equations that describe the inverted pendulum on cart system, we would like to add the motor model to the system model as well.

The model of DC model is very well known

$$J_m\ddot\theta_m+b_m\dot\theta_m = K_mi(t)$$

$$L_m\dot i(t) +R_mi(t)=V-K_m\dot\theta$$

$$K_mi(t)=\tau$$

Knowing the gear ratio and the belt bulley radius, we can convert between motor torque and applied force to the cart $F(t)$
$$F(t)=\frac{nK_mi(t)}{r}$$

Therefore the final system model is
$$(m_c+m_p)\ddot x- \frac{1}{2}m_pl[cos(\theta)\ddot\theta -sin(\theta)\dot\theta^2] =\frac{nK_mi(t)}{r}-b_c\dot x \dots(1)$$
$$(I_p+\frac{m_pl^2}{4})\ddot\theta-\frac{1}{2}m_pl[\ddot x cos(\theta)+g sin(\theta)] = -b_p\dot\theta \dots(2)$$
$$L_m\dot i(t) +R_mi(t)=V-K_m\dot\theta_m \dots(3)$$



## Linearization

let $$x= \begin{pmatrix} x_1 \newline x_2 \newline x_3 \newline x_4 \end{pmatrix} = \begin{pmatrix} x \newline \dot x \newline \theta \newline \dot \theta \end{pmatrix}$$
be the state vector of the system. hence 

$$\dot x= \begin{pmatrix} \dot x_1 \newline \dot x_2 \newline \dot x_3 \newline \dot x_4 \end{pmatrix} = \begin{pmatrix} \dot x \newline \ddot x \newline \dot \theta \newline \ddot \theta \end{pmatrix} $$

It's clear that 
$$\dot x_1 = x_2 \dots f_1$$ and $$\dot x_3 = x_4 \dots f_3$$

now we need to calculate $\dot x_2$ and $\dot x_4$, but first lets rewrite the system model using the state variables 

$$(m_c+m_p)\dot x_2- \frac{1}{2}m_pl[cos(x_3)\dot x_4 -sin(x_3)x_4^2] =u(t)-b_c x_2 \dots(1)$$

$$(I_p+\frac{m_pl^2}{4})\dot x_4+b_p x_4-\frac{1}{2}m_pl[ cos(x_3) \dot x_2+g sin(x_3)] =0 \dots(2)$$

rearranging terms

$$ (m_c+m_p)\dot x_2 =\frac{1}{2}m_plcos(x_3)\dot x_4 - \frac{1}{2}m_plsin(x_3)x_4^2 -b_c x_2 + u(t) \dots(1)$$

$$(I_p+\frac{m_pl^2}{4})\dot x_4=\frac{1}{2}m_pl cos(x_3) \dot x_2+\frac{1}{2}m_plg sin(x_3) -b_p x_4 \dots(2)$$

solving for $\dot x_2$ and $\dot x_4$  (second and forth functions)

$$ \dot x_2 = \frac{\frac{1}{8}m_p^2l^2gsin(x_3) -0.5m_pb_plx_4cos(x_3)-0.5(I_p +0.25m_pl^2)m_plx_4^2sin(x_3)-(I_p+0.25m_pl^2)b_cx_2+u(t)}{I_p(m_c+m_p)+0.25m_cm_pl^2+0.25m_p^2l^2-0.25m_pl^2cos(x_3)^2} \dots f_2$$

$$\dot x_4=\frac{-\frac{1}{8}m_p^2l^2sin(2x_3)x_4^2-0.5m_plb_ccos(x_3)x_2+0.5m_plcos(x_3)u(t)+0.5m_plgsin(x_3)-b_px_4}{I_p(m_c+m_p)+0.25m_cm_pl^2+0.25m_p^2l^2-0.25m_pl^2cos(x_3)^2} \dots f_4$$

To linearize the system, we need to find the the jacobian of the state vector $\dot x$ at the equilibirum point 
$$x_e= \begin{pmatrix} 0 \newline 0 \newline 0 \newline 0 \end{pmatrix}$$
 
The jacobian is defined as 
$$J_\dot x(x_e)={\begin{bmatrix}{\dfrac {\partial f_{1}}{\partial x_{1}}}&{\dfrac {\partial f_{1}}{\partial x_{2}}}&{\dfrac {\partial f_{1}}{\partial x_{3}}}&{\dfrac {\partial f_{1}}{\partial x_{4}}}\newline{\dfrac {\partial f_{2}}{\partial x_{1}}}&{\dfrac {\partial f_{2}}{\partial x_{2}}}&{\dfrac {\partial f_{2}}{\partial x_{3}}}&{\dfrac {\partial f_{2}}{\partial x_{4}}} \newline{\dfrac {\partial f_{3}}{\partial x_{1}}}&{\dfrac {\partial f_{3}}{\partial x_{2}}}&{\dfrac {\partial f_{3}}{\partial x_{3}}}&{\dfrac {\partial f_{3}}{\partial x_{4}}}\newline {\dfrac {\partial f_{4}}{\partial x_{1}}}&{\dfrac {\partial f_{4}}{\partial x_{2}}}&{\dfrac {\partial f_{4}}{\partial x_{3}}}&{\dfrac {\partial f_{4}}{\partial x_{4}}}\end{bmatrix}}$$
evaluating at equalibrum point gives us the state transistion matrix:
$$A=J_\dot x={\begin{bmatrix}{0}&{1}&{0}&{0}\newline{0}&{-\frac{(I_p+0.25m_pl^2)b_c}{\gamma}}&{\frac{0.25m_p^2l^2g}{\gamma}}&{-\frac{0.5m_plb_p}{\gamma}}\newline{0}&{0}&{0}&{1}\newline{0}&{-\frac{0.5m_plb_c}{\gamma}}&{\frac{0.5(m_c+m_p)m_plg}{\gamma}}&{-\frac{b_p(m_c+m_p)}{\gamma}}\end{bmatrix}}$$
where $$\gamma = I_p(m_c+m_p)+0.25m_cm_pl^2$$
The input matrix $B$ is the parital derivatives of state vector W.R.T to input $u$ 

$$B=\begin{bmatrix}{\dfrac {\partial f_{1}}{\partial u}}\newline{\dfrac {\partial f_{2}}{\partial u}}\newline{\dfrac {\partial f_{3}}{\partial u}}\newline{\dfrac {\partial f_{4}}{\partial u}} \end{bmatrix}=\begin{bmatrix}{0}\newline{\frac{I_p+0.25m_p*l^2}{\gamma}}\newline{0}\newline {\frac{0.5m_p*l}{\gamma}} \end{bmatrix}$$

The output matrix $C$  is just $x_1$ and $x_3$
$$C=\begin{bmatrix} {1}& {0} &{0} &{0} \newline {0}&{0}&{1}&{0} \end{bmatrix}$$


The linear state state model is
$$\dot x=Ax+Bu$$
$$y=Cx$$


## Controller Design 





## Refrences
- [JIBSAO (philarchive.org)](https://philarchive.org/archive/JIBSAO) 
- [homework and exercises - Lagrangian of an inverted pendulum on a moving cart - Physics Stack Exchange](https://physics.stackexchange.com/questions/550033/lagrangian-of-an-inverted-pendulum-on-a-moving-cart) 
- [Control Tutorials for MATLAB and Simulink - Home (umich.edu)](https://ctms.engin.umich.edu/CTMS/index.php?aux=Home)
