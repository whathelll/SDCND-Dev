#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}


double calculateCost(Vehicle &vehicle, Vehicle &clone, map<int,vector < vector<int> > > predictions) {

  vector<vector<int>> trajectory = clone.generate_predictions(10);

  //lane cost function
  double cost = 0.0;

  //target lane cost function
  double lane_cost = 0.0;
  for(auto &s: trajectory){
      lane_cost += (s[0] - 0.) * (s[0] - 0.0); //0 is target lane
  }
  std::cout << "lane cost: " << lane_cost;
  cost += lane_cost;

  //velocity cost function
  double velocity_cost = 0.0;
  double velocity_weight = 200.0;
  if(vehicle.v > vehicle.target_speed) {
      velocity_cost = 1.0;
  } else if(vehicle.v < vehicle.target_speed) {
      velocity_cost = 1.0 * (vehicle.target_speed - vehicle.v) / vehicle.target_speed;
  }
  std::cout << " velocity cost: " << velocity_cost*velocity_weight;
  cost += velocity_cost*velocity_weight;

  //acceleration cost function
  double acceleration_cost = 0.0;
  double acceleration_weight = 50.0;
  if(vehicle.v < vehicle.target_speed) {
      acceleration_cost += -clone._max_accel_for_lane(predictions, clone.lane, clone.s);
  }
  //accel cost for next time period
  acceleration_cost += -clone._max_accel_for_lane(predictions, clone.lane, clone.s+clone.v);

  std::cout << " acceleration cost: " << acceleration_cost*acceleration_weight;
  cost += acceleration_cost*acceleration_weight;


  //collision cost
  double collision_cost = 0.0;
  double collision_weight = 10.0;
  for(auto &prediction: predictions) {
    if(prediction.first == -1) {
      continue;
    }
    vector<vector<int>> p = prediction.second;
    //for step 1, if lane equals and
    int p_lane = p[1][0];
    int p_s = p[1][1];
    if(p_lane == clone.lane && p_s > clone.s && p_s < clone.s+clone.v ) {
	collision_cost = 100;
    }
  }
  std::cout << " collision cost: " << collision_cost;
  cost += collision_cost;

  std::cout << " total cost: " << cost;
  std::cout << endl;
  return cost;
}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */

  vector<vector<int>> trajectory;

  for(auto &prediction: predictions) {
//    std::cout << "Car: " << prediction.first << std::endl;
    for(auto &s: prediction.second) {
      if(prediction.first == -1) {
	trajectory = prediction.second;
      }
//      std::cout << "s:" << s[1] << ", lane:" << s[0] << std::endl;
    }
  }
  std::cout << "Current lane:" << this->lane << " v:" << this->v << std::endl;

  int dmax = 3;
  int dmin = 0;

  vector<string> possible_successor_states(0);
  vector<Vehicle> clones;

  //nasty lane selection code but oh well
  possible_successor_states.push_back("KL");  //can always keep lane
  Vehicle keep_left_car = *this;
  clones.push_back(keep_left_car);
  if(this->lane > dmin) {
      possible_successor_states.push_back("LCR");
      Vehicle clone = *this;
      clone.lane -= 1;
      clones.push_back(clone);
  }
  if(this->lane-1 > dmin) {
      possible_successor_states.push_back("LCR");
      Vehicle clone = *this;
      clone.lane -= 2;
      clones.push_back(clone);
  }
  if(this->lane < dmax) {
      possible_successor_states.push_back("LCL");
      Vehicle clone = *this;
      clone.lane += 1;
      clones.push_back(clone);
  }
  if(this->lane+1 < dmax) {
      possible_successor_states.push_back("LCL");
      Vehicle clone = *this;
      clone.lane += 2;
      clones.push_back(clone);
  }


  vector<double> costs(0);

  for(int i=0; i < possible_successor_states.size(); i++) {
      state = possible_successor_states[i];
      // generate trajectory
      Vehicle clone = clones[i];
      vector<vector<int>> trajectory_for_state = clone.generate_predictions(10);

      std::cout << "state:" << state << " new lane:" << trajectory_for_state[0][0] << " old lane:" << trajectory[0][0] << std::endl;

      //calculate cost for trajectory
      double cost_for_state = 0;

      cost_for_state += calculateCost(*this, clone, predictions);
      costs.push_back(cost_for_state);

  }

  string best_next_state = "";
  double min_cost = 999999999999.0;
  for(int i=0; i<possible_successor_states.size(); i++) {
      if(costs[i] < min_cost) {
	  std::cout << "best cost:" << costs[i] << std::endl;
	  min_cost = costs[i];
	  best_next_state = possible_successor_states[i];
      }
  }
  std::cout << "best state: " << best_next_state << std::endl;
  state = best_next_state;

//  state = "KL"; // this is an example of how you change state.


}

void Vehicle::configure(vector<int> road_data) {
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

string Vehicle::display() {

	ostringstream oss;

	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";

    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t;
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {

	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {

    	int v_id = it->first;

        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }

    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}

    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }

    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}
