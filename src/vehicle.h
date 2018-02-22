#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

	map<string, int> lane_direction = { { "PLCL", -1 }, { "LCL", -1 }, { "LCR",
			1 }, { "PLCR", 1 } };

	struct collider {

		bool collision; // is there a collision?
		int time; // time collision happens

	};

	int L = 1;

	int lane = 1;
	int s = 0;
	float v = 0.0;
	float a = 0.0;
	float target_speed = 0.0;
	int lanes_available = 0;
	float max_acceleration = 0.0;
	int goal_lane = 1;
	int goal_s = 0;

	string state = "CS";

	/**
	 * Constructor
	 */
	Vehicle();
	Vehicle(int lane, float s, float v, float a, string state = "CS");

	/**
	 * Destructor
	 */
	virtual ~Vehicle();

	vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

	vector<string> successor_states(map<int, vector<Vehicle>> predictions);

	vector<Vehicle> generate_trajectory(string state,
			map<int, vector<Vehicle>> predictions);

	vector<float> get_kinematics(map<int, vector<Vehicle>> predictions,
			int lane);

	vector<Vehicle> constant_speed_trajectory();

	vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

	vector<Vehicle> lane_change_trajectory(string state,
			map<int, vector<Vehicle>> predictions);

	vector<Vehicle> prep_lane_change_trajectory(string state,
			map<int, vector<Vehicle>> predictions);

	float position_at(double t);

	bool get_vehicle_nearest(map<int, vector<Vehicle>> predictions, int lane, int bufferS,
			Vehicle & rVehicle);

	bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, int bufferS,
			Vehicle & rVehicle);

	bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, int bufferS,
			Vehicle & rVehicle);

	vector<Vehicle> generate_predictions(int horizon = 2);

	void realize_next_state(vector<Vehicle> trajectory);

	void configure(vector<float> road_data);

};

#endif
