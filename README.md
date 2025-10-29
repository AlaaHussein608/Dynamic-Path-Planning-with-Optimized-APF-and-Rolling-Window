# Dynamic-Path-Planning-with-Optimized-APF-and-Rolling-Window

## Overview
This project implements three variants of the Artificial Potential Field (APF) algorithm for dynamic path planning in robotic systems. The algorithms are designed to navigate dynamic environments where obstacles may move during execution.

## Algorithms

### 1. APF1: Adaptive Repulsive Force Field with Rolling Window
- Combines a rolling window approach with an adaptive repulsive force field.
- Uses a heuristic function to select local sub-goals within a rolling window.
- Adapts repulsive force based on the robot's proximity to the goal.

### 2. APF2: Optimized APF with Rolling Window
- Uses the rolling window method but with a fixed repulsive force field.
- Focuses on generating shorter paths but may struggle in complex environments.

### 3. APF3: Optimized APF without Rolling Window
- Relies solely on the optimized APF without local window decomposition.
- Balances path length and success rate but lacks the adaptability of the rolling window.

## Key Features

### Rolling Window Method
- Decomposes global path planning into local sub-problems.
- Selects local sub-targets on the window boundary using a heuristic based on the global goal.
- Helps avoid local minima and adapt to dynamic changes.

### Adaptive Repulsive Force Field
- Modifies the repulsive force based on the robot's distance to the goal.
- Ensures the robot can reach the goal even in the presence of nearby obstacles.


## Results Summary
- **APF1:** Highest success rate but longer paths on average.  
- **APF2:** Shortest paths but lowest success rate.  
- **APF3:** Balanced performance with moderate path length and high success rate.

## Conclusion
The rolling window method combined with adaptive repulsive fields significantly improves robustness in dynamic environments. APF1 is the most reliable, while APF2 is the most efficient when it succeeds. APF3 offers a middle ground between reliability and efficiency.
