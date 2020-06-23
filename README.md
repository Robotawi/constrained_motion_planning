# Motion planning with contraint relaxation

This project aims at solving the problem of flipping heavy objects with robot arms equipped by two-jaw simple grippers. I wanted to introduce a way to avoid the narrow passage planning problem by finding a way to relax the constraints when unneeded. This is accomplished by calculating the expected gravity torque based on the prior knowledge of the grasped object model. 

## The problem
The problem that I wanted to solve is shown below. When trying to flip heavy object, they may slip in hand. This is problematic, and it may even cause loss of grasp!

<img src=project_images/failure_gif15.gif>

## The model
Modeling the in-hand slip phenomina is interesting. It relates the inclination angle to the slip-driven in-hand angle. For more on this, please refer to the [paper]().
<img src=project_images/model.gif>

## The solution
Three different sizes of boards were used in the experiments, big, medium, and small. The following videos show the successful motion sequences found by the planner. The planner applies constraint relaxation according to the expected gavity torque that every specific object may experience during manipulation.

### The big board flipping
<img src=project_images/big_board1.gif>

### The medium board flipping 
<img src=project_images/med_board1.gif>

### The small board flipping 
<img src=project_images/small_board1.gif>


## Real application
