# MPC Hydrofoil Control

### Description

This is a repository hosting code that was developed during my thesis on Model Predictive Control used to control a Hydrofoil-equipped vessel in 3 degrees of freedom.

The work is divided in two parts, with an intermediate step:
- Parametric model of a hydrofoil vessel
- Automatic linearization of reduced DoF model
- Controller design for heave, roll, pitch

A SIMULINK model is used for simulations, logging data for post-processing and assessing performance. 

#### Control Methods
Currently, *LQR* augmented with integral action, saturation and anti-windup is the simple, baseline model.
Variants of *MPC* developed using the Matlab MPC Toolbox were tested and compared against the baseline model.

Tested MPC models include
- Input disturbance model to account for lifting surface offsets (induced by speed changes or modeling error)
- Delay prediction (linear) to account for realistic actuator operation
- Reference previewing (anticipative action)
- Linear Parameter Varying (LPV) prediction models - Adaptive MPC for changing dynamics


### Future work
- [ ] Improve the hydrodynamic modelling capabilities by either moving to a physics engine, or introducing dynamic lifting line
- [ ] Improve project with C/C++ implementation of control algorithms (using acados for example)
- [ ] Combine both methods using tools such as ROS to handle modeling of sensor devices etc.

_The parametric model is currently set as a usual design appearing in the Open Class of Solar Boat competitions such as the Monaco Energy Boat Challenge._
