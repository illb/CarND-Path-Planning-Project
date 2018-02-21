# Model Documentation

## Loop
* collect data
* create a vehicle list with sensor fusion data
* generate vehicle predictions
* choose next state and select the trajectory
* smoothes trajectory and generate next_x and next_y values and puts them into the result json.

## Speed Control
* checkSpeedLimit function controls the maximum speed to less than 49.5mph
* if the vehicle ahead is too close, slow down 0.224 mph
* else, speed up 0.224 mph

## Finite State Machine States
* KL (Keep Lane)
  - start state
  - if the vehicle ahead is too close, consider whether the next lane is empty.
    - if the next lane is empty, consider change state to the PLCL and PLCR
* PLCL (Prepare Lane Change Left), PLCR (Prepare Lane Change Left)
  - PLCL can change to PLCL and LCL in next step
  - PLCR can change to PLCR and LCR in next step
  - compare cost keep lane trajectory
  - If too close vehicle behind select the current lane trajectory.
  - else compare current lane trajectory to next lane trajectory to select slow trajectory.

* LCL (Lane Change Left), LCR (Lane Change Right)
  - lane change and change state to KL
