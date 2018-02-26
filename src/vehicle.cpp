#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>

#include "const.h"
#include "cost.h"

const float BUFFER_AHEAD = 15.0;
const float BUFFER_BEHIND = 15.0;
const float BUFFER_NEXT_LANE = 12.0;
const float BUFFER_TOO_CLOSE = 8.0;
const float DISTANCE_COLISION = 3.5;

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {
}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

	this->lane = lane;
	this->s = s;
	this->v = v;
	this->a = a;
	this->state = state;
	this->max_acceleration = -1;
}

Vehicle::~Vehicle() {
}

vector<Vehicle> Vehicle::choose_next_state(
		map<int, vector<Vehicle>> predictions) {
	/*
	 Here you can implement the transition_function code from the Behavior Planning Pseudocode
	 classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
	 to the next state.

	 INPUT: A predictions map. This is a map of vehicle id keys with predicted
	 vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
	 the vehicle at the current timestep and one timestep in the future.
	 OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

	 */
	vector<string> states = successor_states(predictions);

//	cout << "successor_states.size=" << states.size() << "\n";

	float cost;
	vector<float> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (vector<string>::iterator it = states.begin(); it != states.end();
			++it) {
		vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
		if (trajectory.size() != 0) {
			cost = calculate_cost(*this, predictions, trajectory);
			costs.push_back(cost);
			final_trajectories.push_back(trajectory);
		}
	}

	vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);
	return final_trajectories[best_idx];
}

const int PREPARE_STATE_RESET_TICK_COUNT = 60;

vector<string> Vehicle::successor_states(map<int, vector<Vehicle>> predictions) {
	/*
	 Provides the possible next states given the current state for the FSM
	 discussed in the course, with the exception that lane changes happen
	 instantaneously, so LCL and LCR can only transition back to KL.
	 */
	vector<string> states;
	string state = this->state;
	if (state.compare("KL") == 0) {
		Vehicle vehicle_ahead;
		if (get_vehicle_ahead(predictions, lane, BUFFER_AHEAD, vehicle_ahead)) {
			if (lane > 0) {
				if (can_change_lane(predictions, lane - 1)) {
					states.push_back("PLCL");
				}
			}
			if (lane < lanes_available - 1) {
				if (can_change_lane(predictions, lane + 1)) {
					states.push_back("PLCR");
				}
			}
		}
	} else if (state.compare("PLCL") == 0) {
		if (state_tick < PREPARE_STATE_RESET_TICK_COUNT) {
			if (lane > 0) {
				states.push_back("PLCL");
				states.push_back("LCL");
			}
		}
	} else if (state.compare("PLCR") == 0) {
		if (state_tick < PREPARE_STATE_RESET_TICK_COUNT) {
			if (lane < lanes_available - 1) {
				states.push_back("PLCR");
				states.push_back("LCR");
			}
		}
	}

	if (states.empty()) {
		states.push_back("KL");
	}

	//If state is "LCL" or "LCR", then just return "KL"
	return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
		map<int, vector<Vehicle>> predictions) {
	/*
	 Given a possible next state, generate the appropriate trajectory to realize the next state.
	 */
	vector<Vehicle> trajectory;
	if (state.compare("CS") == 0) {
		trajectory = constant_speed_trajectory();
	} else if (state.compare("KL") == 0) {
		trajectory = keep_lane_trajectory(predictions);
	} else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
		trajectory = lane_change_trajectory(state, predictions);
	} else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
		trajectory = prep_lane_change_trajectory(state, predictions);
	}
	return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions,
		int lane) {
	/*
	 Gets next timestep kinematics (position, velocity, acceleration)
	 for a given lane. Tries to choose the maximum velocity and acceleration,
	 given other vehicle positions and accel/velocity constraints.
	 */
	float max_velocity_accel_limit = this->v + (this->max_acceleration * FRAME_SEC);
	float min_velocity_accel_limit = this->v - (this->max_acceleration * FRAME_SEC);

	float new_position;
	float new_velocity;
	float new_accel;
	Vehicle vehicle_ahead;
	Vehicle vehicle_behind;

	if (get_vehicle_ahead(predictions, lane, BUFFER_AHEAD, vehicle_ahead)) {
//		if (get_vehicle_behind(predictions, lane, BUFFER_BEHIND, vehicle_behind)) {
//			new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
//		} else {
			float distance = vehicle_ahead.s - this->s;

//			cout << "distance : " << distance << "\n";
//			cout << "vehicle_ahead.v : " << vehicle_ahead.v << "\n";
//			cout << "this->v : " << this->v << "\n";
//			cout << "this->a : " << this->a << "\n";

			float max_velocity_in_front = 0.0;
			float too_close_distance = BUFFER_TOO_CLOSE;
			if (distance <= too_close_distance) {
				cout << " TOO CLOSE !!!\n";
				float weight = distance / too_close_distance;
				max_velocity_in_front = vehicle_ahead.v * weight * weight;
				max_velocity_in_front = min(max_velocity_in_front, max_velocity_accel_limit);
				if (weight > 0.5) {
					max_velocity_in_front = max(max_velocity_in_front, min_velocity_accel_limit);
				}
			} else {
				float weight = (distance - too_close_distance) / (BUFFER_AHEAD - too_close_distance);
				max_velocity_in_front = (v * weight) + (vehicle_ahead.v * (1.0 - weight));
				max_velocity_in_front = min(max_velocity_in_front, max_velocity_accel_limit);
				max_velocity_in_front = max(max_velocity_in_front, min_velocity_accel_limit);
			}

			new_velocity = min(max_velocity_in_front, this->target_speed);
//		}
	} else {
		new_velocity = min(max_velocity_accel_limit, this->target_speed);
	}

	new_accel = (new_velocity - this->v) / FRAME_SEC; //Equation: (v_1 - v_0)/t = acceleration
	new_position = this->s + (new_velocity * FRAME_SEC)
			+ (new_accel * FRAME_SEC * FRAME_SEC) / 2.0;

	return {new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
	/*
	 Generate a constant speed trajectory.
	 */
	float next_pos = position_at(FRAME_SEC);
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->v,
			this->a, this->state), Vehicle(this->lane, next_pos, this->v, 0,
			this->state) };
	return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(
		map<int, vector<Vehicle>> predictions) {
	/*
	 Generate a keep lane trajectory.
	 */
	vector<Vehicle> trajectory = { Vehicle(lane, this->s, this->v, this->a,
			state) };
	vector<float> kinematics = get_kinematics(predictions, this->lane);
	float new_s = kinematics[0];
	float new_v = kinematics[1];
	float new_a = kinematics[2];
	trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
	return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
		map<int, vector<Vehicle>> predictions) {
	/*
	 Generate a trajectory preparing for a lane change.
	 */
	float new_s;
	float new_v;
	float new_a;
	Vehicle vehicle_ahead;
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->v,
			this->a, this->state) };
	vector<float> curr_lane_new_kinematics = get_kinematics(predictions,
			this->lane);

	if (!can_change_lane(predictions, new_lane) ||
			get_vehicle_ahead(predictions, this->lane, BUFFER_TOO_CLOSE, vehicle_ahead)) {

		new_s = curr_lane_new_kinematics[0];
		new_v = curr_lane_new_kinematics[1];
		new_a = curr_lane_new_kinematics[2];

		cout << "PLCT : CURRENT LANE\n";
	} else {
		vector<float> best_kinematics;
		vector<float> next_lane_new_kinematics = get_kinematics(predictions,
				new_lane);
		//Choose kinematics with lowest velocity.
		if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
			best_kinematics = next_lane_new_kinematics;
		} else {
			best_kinematics = curr_lane_new_kinematics;
		}
		new_s = best_kinematics[0];
		new_v = best_kinematics[1];
		new_a = best_kinematics[2];

//		cout << "PLCT : NEXT LANE\n";
	}

	trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
		map<int, vector<Vehicle>> predictions) {
	/*
	 Generate a lane change trajectory.
	 */
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory;

	if (!can_change_lane(predictions, new_lane)) {
		//If lane change is not possible, return empty trajectory.
		return trajectory;
	}

	trajectory.push_back(
			Vehicle(this->lane, this->s, this->v, this->a, this->state));
	vector<float> kinematics = get_kinematics(predictions, new_lane);
	trajectory.push_back(
			Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2],
					state));
	return trajectory;
}

float Vehicle::position_at(double t) {
	return this->s + this->v * t + this->a * t * t / 2.0;
}

bool Vehicle::can_change_lane(map<int, vector<Vehicle>> predictions, int lane) {
	float min_dist = BUFFER_NEXT_LANE;
	Vehicle temp_vehicle;

	//Check if a lane change is possible (check if another vehicle occupies that spot).
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
			it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == lane) {
			float distance = temp_vehicle.s - this->s;
			float diff_velocity = temp_vehicle.v - this->v;
			if (distance >= BUFFER_AHEAD + 2.0 || distance <= -BUFFER_BEHIND) {
				// pass
			} else {
				if (distance > 0.0) {
					// ahead
					if (distance < BUFFER_NEXT_LANE) {
						cout << "CANNOT CHANGE: TOO CLOSE AHEAD\n";
						return false;
					} else if (diff_velocity < 3.0) {
						cout << "CANNOT CHANGE: TOO SLOW AHEAD\n";
						return false;
					}
				} else {
					// behind
					if (abs(distance) < BUFFER_NEXT_LANE - 3.0) {
						cout << "CANNOT CHANGE: TOO CLOSE BEHIND\n";
						return false;
					} else if (diff_velocity > 10.0) {
						cout << "CANNOT CHANGE: TOO FAST BEHIND\n";
						return false;
					}
				}
			}

			min_dist = abs(temp_vehicle.s - this->s);
		}
	}

	return true;
}


bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, float bufferS,
		Vehicle & rVehicle) {
	/*
	 Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
	 rVehicle is updated if a vehicle is found.
	 */
	float max_s = this->s - bufferS;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
			it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s
				&& temp_vehicle.s > max_s) {

			max_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, float bufferS,
		Vehicle & rVehicle) {
	/*
	 Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
	 rVehicle is updated if a vehicle is found.
	 */
	float min_s = this->s + bufferS;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
			it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s
				&& temp_vehicle.s < min_s) {
			min_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
	/*
	 Generates predictions for non-ego vehicles to be used
	 in trajectory generation for the ego vehicle.
	 */
	vector<Vehicle> predictions;
	for (int i = 0; i < horizon; i++) {
		float next_s = position_at(FRAME_SEC * i);
		predictions.push_back(Vehicle(this->lane, next_s, v, 0));
	}
	return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
	/*
	 Sets state and kinematics for ego vehicle using the last state of the trajectory.
	 */
	Vehicle next_state = trajectory[1];
	if (this->state != next_state.state) {
		state_tick = 0;
	} else {
		state_tick++;
	}

	this->state = next_state.state;
	this->lane = next_state.lane;
	this->s = next_state.s;
	this->v = next_state.v;
	this->a = next_state.a;
}

void Vehicle::configure(vector<float> road_data) {
	/*
	 Called by simulator before simulation begins. Sets various
	 parameters which will impact the ego vehicle.
	 */
	target_speed = road_data[0];
	lanes_available = road_data[1];
	goal_s = road_data[2];
	goal_lane = road_data[3];
	max_acceleration = road_data[4];
}
