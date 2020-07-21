# Motion planning with constraints

This project aims at solving the in-hand problem associated with flipping heavy objects with robot arms. Upon the derivation of the slip model, the problem is formulated as an angle constraint problem. The constraint is integrated into the RRT motion planner. The solution is satisfactory but not efficient. It suffered from over constraining which let to narrow passage problem. In this improvement extension, I implement a constraint relaxation to avoid over constraining and decrease the planning time.

## Problem
When trying to flip heavy object using the shortest path planning, they may slip in hand. The in-hand slip is undesirable because it can cause unintended change of the grasp or totally grasp loss in the worst case.

<img src=project_images/failure_gif15.gif>

## Slip model
Modeling the in-hand slip is interesting. It relates the inclination angle of the gripper to the slip-driven in-hand angle (object angle with respect to the gripper). For more details on this, please refer to the [paper](https://ieeexplore.ieee.org/document/9044335).

<img src=project_images/model.gif>

## Solution
Three different sizes of boards were used in the experiments, large, medium, and small. The following videos show the successful motion sequences found by the planner with constraint relaxation applied. The relaxation takes into consideration the expected gravity torque that every specific object may experience during manipulation and defines a critical limit beyond which the in-hand slip happens. The limit is observed allover the generated motion sequence for a safe and efficient manipulation process.

### Big board flipping
The planner determines tight constraint angle (10°) as the board has large dimensions.
<img src=project_images/big_board1.gif>

### Medium board flipping
The planner determines more relaxed constraint angle (40°) for the medium board.
<img src=project_images/med_board1.gif>

### Small board flipping 
The planner found that there is no need at all for limiting the constraint for the small light board.
<img src=project_images/small_board1.gif>


## Real application
The above method is deployed in a dual-arm manipulation system to enhance its capability of manipulating heavy objects.

<img src=project_images/dual_arm_manipulation.gif>

The details of this project are described in [this paper](https://ieeexplore.ieee.org/document/9044335). This module was integrated in my lab's robotics development environment as a method for 3D pose estimation. 

## Contact
If you are interested in the presented work/ideas, or if you have any questions, feel free to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can discuss about this project and other interesting projects.

