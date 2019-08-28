# Optimal Turn-Right Trajectory using Pontryagin's Maximum Principle

## Description

Small python script that solve the optimization problem to generate the time optimal trajectory given a maximum allowed acceleration, initial and final conditions. It is based on the application of Pontryagin's Maximum Principle.

**Initial Velocity: 10 m/s**

<img src="img/opti_traj_pos_10.png" alt="Position" width="400"/>
<img src="img/opti_traj_accel_10.png" alt="Acceleration" width="400"/>

**Initial Velocity: 2 m/s**

<img src="img/opti_traj_pos_2.png" alt="Position" width="400"/>
<img src="img/opti_traj_accel_2.png" alt="Acceleration" width="400"/>

## Run the code

Run the code:

```
	python main.py --help

    Optimal Turn-Right Trajectory using Pontryagin's Maximum Principle

    optional arguments:
      -h, --help            show this help message and exit
      -max_accel MAX_ACCEL  Maximum Acceleration
      -init_pos INIT_POS    Initial position along Y axis
      -init_vel [INIT_VEL [INIT_VEL ...]]
                            Initial velocities
      -show_figures         Plot results on figures
      -no_save_csv          Do not save results
      -output_dir OUTPUT_DIR
                            Path of the output
```