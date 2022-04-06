# underwater_robot_sim
A simulator for kinematics, dynamics and control on UVDMS(Underwater Vehicle Dual-Manipulator System).It uses DH method for establishing kinematic model and Newton-Euler method for dynamic model. It can be used for further research on robotics, control theory and so on. This project provides a demo for underwater grasping operation of UVDMS. 

## Demo
Underwater operation without control

<img src="https://github.com/LiuYuha0/underwater_robot_sim/blob/master/flash/Demo_dynamics.gif" alt="demo" width="480" height="360" border="10" />

PID feedback control on robot's position and pose
<img src="https://github.com/LiuYuha0/underwater_robot_sim/blob/master/flash/Demo_PIDcontrol.gif" alt="demo" width="480" height="360" border="10" /></a>

## Run
Clone the repository
```sh
cd YOUR_WORKSPACE/
git clone https://github.com/LiuYuha0/underwater_robot_sim.git
```

Opne matlab and run
```sh
simurv.m
```

chose model(You can establish your own robot model)
```sh
data_hust_uvdms.m
```

chose simulation demo
```sh
core_UVDMS_demo2_grasp.m
```

If you want to generate video as demo above, run `GenerateMovie(0)` in MATLAB terminal and chose `xx.mat`.

## Result
The PID control results are as follows, the variation of robot's position/orientation show in Fig.1, and forces/torques in Fig.2.
<img src="https://github.com/LiuYuha0/underwater_robot_sim/blob/master/flash/control_result_pos.png" alt="result" width="480" height="360" border="10" /></a>
<img src="https://github.com/LiuYuha0/underwater_robot_sim/blob/master/flash/control_result_tau.png" alt="result" width="480" height="360" border="10" /></a>

