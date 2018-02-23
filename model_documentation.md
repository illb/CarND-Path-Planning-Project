# Model Documentation

## Loop
* collect data
* create a vehicle list with sensor fusion data
* generate vehicle predictions
* choose next state and select the trajectory
* smoothes trajectory and generate next_x and next_y values and puts them into the result json.

## Speed Control
* target_speed controls the maximum speed to less than 40mph
* if the vehicle ahead is close, mix the velocity of my vehicle and the velocity of vehicle ahead in proportion to distance.
  - close : The distance from the preceding vehicle is less than 15
* else if the vehicle ahead is too close, use the velocity of vehicle ahead in inverse proportion to distance.
  - too close : The distance from the preceding vehicle is less than 7
* else speed up with max acceleration

## Finite State Machine States
* KL (Keep Lane)
  - start state
  - if the vehicle ahead is close (distance 15 or less), consider whether the next lane is empty.
    - if the next lane is empty, consider change state to the PLCL and PLCR
      - there's no vehicle within 10 of the next lane, consider it as an empty lane.
* PLCL (Prepare Lane Change Left), PLCR (Prepare Lane Change Left)
  - PLCL can change to PLCL and LCL in next step
  - PLCR can change to PLCR and LCR in next step
  - compare cost keep lane trajectory
  - If too close (distance 5 or less) vehicle behind or a next lane is not empty (there's a vehicle distance 10 or less) select the current lane trajectory.
  - else select the next lane trajectory.

* LCL (Lane Change Left), LCR (Lane Change Right)
  - lane change and change state to KL

## Cost Functions
* Among the result trajectories, the lowest cost trajectory is seleted
* cost functions
  - goal_distance_cost : both intended_lane and final_lane should be close to goal lane to reduce cost.
  - inefficiency_cost : both speeds of intended_lane and final_lane should be close to target speed to reduce cost.

  