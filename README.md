# KUKA 6 - Simulation

Plan a trajectory and simulate a Kuka 6 in Simulink, using the Simulink 3D Animation Toolbox.

The trajectory planning can be done in the task space as well as in the joint space.
The inverse kinematic is calculated using the Gauss-Newton with Levenberg-Marquadt method [[1]](#1).
For the inverse dynamic the recursive Newton-Euler method has been implemented.

## Folder structure
```
.
├── ...
├── srcs                    
│   ├── matlab                          
|   │   ├── +core                       # Package for all the Matlab object
|   │   ├── +solver                     # Package for the ik, id and fd solver
|   │   └── +trajectory                 # Package for the quintic and cubic polynomial
│   ├── simulink
|   │   ├── KukaSimModel.slx            # Kuka Simulation model
|   │   ├── KukaVisualizationModel.slx  # Model for the 3D simulation
|   │   └── lib_Solver.slx              # library containing blocks for ik, id and fk solvers
│   └── wrl                             # wrl model
└── startup.m                           # script used for the setup
```

## Kuka6.wrl
Source: _Rohit Mishra (2021). Modelling and Simulation of Kuka kr6 robot. (https://www.mathworks.com/matlabcentral/fileexchange/41451-modelling-and-simulation-of-kuka-kr6-robot), MATLAB Central File Exchange. Retrieved September 14, 2021._

## startup.m
In order to setup the environment and allow the simulation, the script `startup.m` shall be run:
* set the needed Matlab Path
* load the DH Paramaters, as well as the Mass, Inertia and the external Forces vector
* load the definied waypoints
* load the bus elements from the `busObjects.mat` file
* open the model and set the space mode:
    * `TP_MODE=1` for task space
    * `TP_MODE=2` for joint space

To simulate the model, just press F5 or click on the Run button.

## References
<a id="1">[1]</a> 
https://people.duke.edu/~hpgavin/ce281/lm.pdf