==============================================================================
 THREE-TANK MODEL PREDICTIVE CONTROLLER (MPC)
==============================================================================

OVERVIEW
--------

This project implements a Model Predictive Controller (MPC) for the nonlinear
three-tank system.

The controller uses:

    • Successive linearization
    • Affine discrete-time model
    • Absolute state prediction
    • Absolute control input optimization
    • Projected Gradient Descent (PGD) solver

Unlike the original implementation, this controller does NOT use deviation
variables or steady-state operating points.

Instead, it directly predicts the actual tank levels and pump flow rates.


==============================================================================
1. NONLINEAR MODEL
==============================================================================

Continuous-time model

    x_dot = f(x,u)

where

    x = [h1 h2 h3]^T

    u = [q1 q2]^T

Euler discretization gives

    x(k+1) = x(k) + Ts*f(x(k),u(k))


==============================================================================
2. LINEARIZATION
==============================================================================

At every sampling instant the nonlinear model is linearized around

    xbar = current measured state

and

    ubar = [0 0]^T

The continuous Jacobians are

    Ac = df/dx

    Bc = df/du

The discrete-time model is

    Ad = I + Ts*Ac

    Bd = Ts*Bc

Since the operating point is generally not the origin, an affine offset is
required

    d = xbar
        + Ts*f(xbar,ubar)
        - Ad*xbar
        - Bd*ubar

The resulting affine model becomes

    x(k+1) = Ad*x(k) + Bd*u(k) + d


==============================================================================
3. PREDICTION MODEL
==============================================================================

Future states are predicted using

    X = Phi*x + Gamma*U + D

where

    X =
        [ x(k+1)
          x(k+2)
            ...
          x(k+H) ]

and

    U =
        [ u(k)
          u(k+1)
            ...
          u(k+H-1) ]

Phi, Gamma and D are generated recursively.


==============================================================================
4. FREE RESPONSE
==============================================================================

If every future control input is zero

    U = 0

the prediction becomes

    Xfree = Phi*x + D

This is the natural gravity-driven evolution of the system.


==============================================================================
5. COST FUNCTION
==============================================================================

The MPC minimizes

    J = (X-R)'*Sbar*(X-R)
        + U'*Rbar*U

Since

    X = Xfree + Gamma*U

the optimization problem becomes

    min

        0.5*U'*G*U + F'*U

where

    G = Gamma'*Sbar*Gamma + Rbar

    F = Gamma'*Sbar*(Xfree-R)


==============================================================================
6. INPUT CONSTRAINTS
==============================================================================

Each pump satisfies

    0 <= qi <= Qmax

Therefore

    0 <= U <= Qmax

The constraints are enforced by projection after every gradient step.


==============================================================================
7. OPTIMIZATION
==============================================================================

The quadratic program is solved using Projected Gradient Descent (PGD).

Each iteration performs

    Gradient

        grad = G*U + F

    Gradient step

        U = U - alpha*grad

    Projection

        0 <= U <= Qmax

The iteration is repeated a fixed number of times.


==============================================================================
8. RECEDING HORIZON STRATEGY
==============================================================================

Only the first optimized control input

    u(k)

is applied to the plant.

At the next sampling instant

    1. Measure current tank levels.

    2. Linearize the nonlinear model.

    3. Build a new prediction model.

    4. Solve the optimization problem.

    5. Apply only the first control input.

The procedure is repeated every sampling period.


==============================================================================
9. FILE STRUCTURE
==============================================================================

MPC()

    Main controller.

linearModelAffine()

    Computes

        Ad
        Bd
        d

from the nonlinear model.

nonlinearDynamics()

    Computes the nonlinear three-tank dynamics.


==============================================================================
10. MAIN EQUATIONS
==============================================================================

Nonlinear model

    x_dot = f(x,u)

Affine discrete model

    x(k+1) = Ad*x(k) + Bd*u(k) + d

Prediction model

    X = Phi*x + Gamma*U + D

Free response

    Xfree = Phi*x + D

Quadratic objective

    J = (X-R)'*Sbar*(X-R) + U'*Rbar*U

QP form

    min 0.5*U'*G*U + F'*U

Projected Gradient update

    U = Projection(U - alpha*(G*U + F))


==============================================================================
NOTES
==============================================================================

• Linearization is performed at every sampling instant.

• The controller optimizes absolute pump flow rates instead of control
  increments.

• No steady-state operating point is required.

• The affine offset guarantees that the linear model exactly matches the
  nonlinear model at the current operating point.

• Only the first optimized control move is applied, following the standard
  receding-horizon MPC strategy.

==============================================================================