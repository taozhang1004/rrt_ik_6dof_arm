# Path Planning for GalaXea A1 Robotic Arm Based on RRT Algorithm

## After completing this tutorial, you will:

- Understand how the RRT path planning algorithm is implemented on a 6DOF robotic arm.
- Learn about the shortcomings of the original RRT algorithm and how to improve it.
- Learn how to use Inverse Kinematics (IK) to solve for joint angles.
- Learn how to use RRT to perform object grasping in MuJoCo simulation and generate data.

---

## Your Task

- Use RRT (and its potential enhancements) to implement object grasping with the A1 robotic arm and generate data.
- Task Outline:
  - Initial robotic arm position ![start](/media/start.png)
  - Target robotic arm position ![end](/media/end.png)
  - Without path planning, the arm will collide with obstacles ![no_plan](/media/no_plan.gif)
  - After applying RRT path planning ![with_plan](/media/with_plan.gif)

---

## Task Steps

### Step 1: Understand the RRT Algorithm and Run the Code
- Read through `rrt_a1.py` and understand how the algorithm is implemented.
- Start the conda environment set up in Project 1: `conda activate act_a1`
- Run RRT until a successful path is found: `python3 rrt_a1.py`
- View the path generated: `rrt_robot_motion.mp4`

### Step 2: Improve the RRT Algorithm
- Problem: Currently, the probability of RRT finding a valid path is quite low, or it takes too long to find one.
- Solution: Enhance the existing RRT algorithm to improve planning efficiency, such as using RRT* or RRT-Connect algorithms.

### Step 3: Implement Object Grasping and Record Data
- Set up a scene in MuJoCo for the robotic arm to grasp an object.
- The current target position is defined in the robotic arm's 6D joint space. In other words, the goal is to find a joint state, but what we want is to provide the object’s pose and have the arm directly grasp it. You need to refer to the code in `ik.py` to solve for the joint angles based on the target end-effector pose using IK. Install the additional dependencies: `pip install ikpy transformations`, then run `python3 ik.py` to generate a sample IK video. The coarse coordinates represent the actual end-effector pose, while the fine coordinates are the target pose. If the IK solution is successful, both poses will perfectly overlap, as shown in the image: ![ik](/media/ik.gif)
- Combine the previous steps to build an automated path planning and object grasping data recording pipeline for future training use.

---
