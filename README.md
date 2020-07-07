# Motion planning with contraint relaxation

This project aims at solving the problem of flipping heavy objects with robot arms equipped by two-jaw simple grippers. I wanted to introduce a way to avoid the narrow passage planning problem by finding a way to relax the constraints when unneeded. This is accomplished by calculating the expected gravity torque based on the prior knowledge of the grasped object model. 

## Problem
When trying to flip heavy object using the conventional shortest path planning, they may slip in hand. The in-hand slip is undesirable because it can cause unintended change of the grasp or totally grasp loss in the worst case.

<img src=project_images/failure_gif15.gif>

## Slip model
Modeling the in-hand slip phenomina is interesting. It relates the inclination angle to the slip-driven in-hand angle. For more on this, please refer to the [paper](https://ieeexplore.ieee.org/document/9044335).

<img src=project_images/model.gif>

## Solution
Three different sizes of boards were used in the experiments, large, medium, and small. The following videos show the successful motion sequences found by the planner. The planner applies constraint relaxation according to the expected gavity torque that every specific object may experience during manipulation.

### Big board flipping
The planner determines tight constraint angle (10°) as the board has large dimentions.
<img src=project_images/big_board1.gif>

### Medium board flipping
The planner determines more relaxed constraint angle (40°) for the medium board.
<img src=project_images/med_board1.gif>

### Small board flipping 
The planner found that there is no need at all for limiting the constraint for the small board.
<img src=project_images/small_board1.gif>


## Real application
The above method is deployed in a dual-arm manipulation system to enhance its capabaility of manipulating heavy objects.

<img src=project_images/dual_arm_manipulation.gif>

The details of this project are described in [this paper](https://ieeexplore.ieee.org/document/9044335). This module was integrated in my lab's robotics development environemt as a method for 3D pose estimation. 

## Contact
If you are interested in the presented work/ideas, or if you have any questions, feel free to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can disuss about this project and other interesting projects.

