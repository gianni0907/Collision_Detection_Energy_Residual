# Collision Detection Energy Residual
In this work we tackle the issue of collision detection, a key topic in the context of safe physical human-robot interaction. The problem is addressed in case of rigid robots via a scalar monitoring signal of the kinetic energy of the system. The proposed solution relies on proprioceptive sensors only and assumes that only the robot joint positions are available, while the velocities are estimated using a reduced-order observer.

## Energy-based residual
It is a scalar residual defined as
```math
        \sigma(t)=k_{\sigma} \left( \hat{T}(t)-\int_0^t{(\dot{\mathbf{q}}^T(\mathbf{\tau}_m-\hat{\mathbf{g}}(\mathbf{q}))+\sigma)ds}-\hat{T}(0) \right)
```
where $\hat{T}(t)$ is the estimate of the robot kinetic energy at instant $t\geq0$, $\hat{\mathbf{g}}(\mathbf{q})$ is the estimate of the gravity vector. In ideal condition its dynamics is
```math
        \dot{\sigma}(t)=k_{\sigma}\left( P_{ext}-\sigma \right)
```
where $P_{ext}=\dot{\mathbf{q}}^T\mathbf{\tau}_{ext}$ is the external power.

## Momentum-based residual
The vectorial residual signal is defined as
```math
        \mathbf{r}(t)=\ \mathbf{K}_r \left( \hat{\mathbf{p}}(t)-\int_0^t (\mathbf{\tau}_m-\hat{\mathbf{\beta}}(\mathbf{q},\dot{\mathbf{q}})+\mathbf{r})ds - \mathbf{p}(0) \right)
```
where $\mathbf{\beta}(\mathbf{q},\dot{\mathbf{q}}) \coloneqq \mathbf{g}(\mathbf{q})-\mathbf{C}^T(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}}$ and $\hat{\mathbf{p}}(t)=\hat{\mathbf{M}}(\mathbf{q})\mathbf{\dot{q}}$.
Assuming ideal conditions, the dynamics is
```math
        \dot{\mathbf{r}}(t)=\mathbf{K}_r(\mathbf{\tau}_{ext}-\mathbf{r}(t)) \ ,
```
so it will be used as a virtual sensor of the torque generated in a collision.

## Reduced-order observer
Assuming $\dot{\mathbf{q}}$ not available for measurement, we detail a reduced-order observer following "[Andrea Cristofaro and Alessandro De Luca. Reduced-order observer design for robot manipulators](https://ieeexplore.ieee.org/document/9849836)".

## Overall block scheme
The experiments are performed following the scheme - also in `media/block_scheme.jpg`

![Block scheme](media/block_scheme.jpg)

Further details on the components of the scheme are available in `report.pdf`.

## Repository structure
The structure of the code in `source/` is detailed in the following.
* `DHMatrix.m`: computes the DH transformation matrices from the DH table
* `check_in_workspace.m`: given the link lengths l and the point p, checks if the point is inside the workspace.
* `compute_eeJacobian.m`: script that generates the following functions
	- `get_pee.m` get the end-effector position
 	- `get_Jee.m` get linear part of the end effector jacobian
  	- `get_dJee.m` get the derivative of the linear part of the end effector jacobian
* `compute_vel_obs_gain.m`: script that computes the gain to use in the velocity observer using $v_{max}=4\ [rad/s],\ \eta=1\ [rad/s]$. It generates the following functions
 	- `get_inertia_matrix.m` returns the matrix $\mathbf{M}(\mathbf{q})$
  	- `get_c_factorization_matrix.m` returns a factorization matrix $\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})$ that ensures the skew-symmetry of $\dot{\mathbf{M}}(\mathbf{q})-2\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})$
* `elem_rot_mat.m`: returns the rotation matrix given an axis (x,y,z) and an angle
* `inverse_kinematics.m`: compute inverse kinematics of 3R spatial manipulator from the end effector Cartesian position. A parameter allows to select among the 4 closed-form solutions. This is used for the initial configuration, while online the Levenberg_Marquardt solver offered by the Robotics Toolbox is used.
* `modNE.m`: implementation of the modified Newton Euler algorithm from "[Alessandro De Luca and Lorenzo Ferrajoli. A modified newton-euler method for dynamic computations in robot fault detection and control](https://ieeexplore.ieee.org/document/5152618)". The implemented variant ensures the skew-symmetry of $\dot{\mathbf{M}}(\mathbf{q})-2\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})$
* `plot_data.m`, `plot_joint_space.m`: plotting functions for data coming from the simulation
* `robot_motion.m`: visualize the robot motion
* `run_experiment.m`: **main file. It defines all the robot parameters and runs the simulation**.
* `simulation.slx`: **simulink file**. It contains the block scheme for the simulation. It is run by `run_experiment.m`

### How to run
Simply clone the repository and run `run_experiment.m`. Relevant parameters that are free to tweak are
* `estimate_velocity`: choose whether to estimate $\dot{\mathbf{q}}$ via numerical differentiation (`estimate_velocity=2`), use the velocity observer (`estimate_velocity=1`), or use the ground truth value (`estimate_velocity=0`).
* `residual_threshold`: choose the threshold for $\sigma(t)$ for collision detection
* `enable_push`: choose whether to push the robot (`enable_push=1`) or not
* `mass_percent_uncertainty`: choose the percentage by which increasing the mass in the robot dynamics with respect to the value used elsewhere. This is to test the robustness of the collision detection pipeline to model uncertainties.

At the end of the simulation, all the plots will appear along with the visualization of the robot.

## Simulations
The simulations carried out compare:
* reduced-order velocity observer vs numerical differentiation method, in two scenarios:
	- ideal case
	- model uncertainties (link masses increased of 5%)
* stiff controller vs compliant controller.

The following video - also in `media/video.mp4` file - shows the obtained results for the second comparison

https://github.com/gianni0907/Collision_Detection_Energy_Residual/assets/72447693/35204628-dff1-4199-9ac4-466ea9dc3637


Further details are available in the report.
