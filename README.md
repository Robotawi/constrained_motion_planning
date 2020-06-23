#Motion planning with contraint relaxation 
This project aims at solving the problem of flipping heavy objects with robot arms equipped by two-jaw simple grippers. I wanted to introduce a way to avoid the narrow passage planning problem by finding a way to relax the constraints when unneeded. This is accomplished by calculating the expected gravity torque based on the prior knowledge of the grasped object model. 

The problem that I wanted to solve is shown below. When trying to flip heavy object, they may slip in hand. This is problematic, and it may even cause loss of grasp!

<img src=data/failure_gif15.gif>