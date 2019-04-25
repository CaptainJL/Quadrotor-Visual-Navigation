# Quadrotor-Visual-Navigation

This repository contains all of the code relating to the visual navigation of a quadrotor in complex environments.
This is based on using optical flow as a cue for motion.


# Directory Structure

* **IROS_2018**: 	This is the code used for the obstacle avoidance flying experiments. 
					This included a conference paper submission called "Vision Based Forward Sensitive Reactive Control for a Quadrotor VTOL" (url= https://ieeexplore.ieee.org/abstract/document/8593606)
* **VISUAL_SERVO**: This is the code used to perform a visual hover control over a textured surface using optical flow.
					It then can also uses a target of a form, to reduce drift error caused by optical flow.
					
Each directory is broken down in "GPU" and "PX4", where the "GPU" part is the code that was used on an NVIDIA Tegra TX2, while the "PX4" part is the code that was modified for use on the Pixhawk 2.1 flight controller.