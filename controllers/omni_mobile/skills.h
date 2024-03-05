#ifndef SKILLS_H
#define SKILLS_H
#include "navigate.h"
#include "RRT_simple.h"
#include <cassert>
#include <webots/supervisor.h>

#define SHOOT_FIXED_MODE 0
#define DEBUG_RRT 0

Dot default_bx = Dot(-11.6, 11.6);
Dot default_by = Dot(-8, 8);

Dot extend_bx = Dot(-12.3, 12.3);
Dot extend_by = Dot(-8.8, 8.8);

bool BALL_OBS_ON = 0;
//-------SOCCER_FIELD_VARIABLE----------------------------------
int robot_encrypted_id;
int map_id[12] = {-1, 0, 1, -1, 3, 2, -1, -1, -1, 4, 5, 6};
inline int robot_decrypt(int en_id) { return (en_id >= 500) ? map_id[en_id - 500] : (map_id[en_id] + 7); }
#define ROBOTS 7*2  // number of robots
double ACTIVE_RANGE = 0.64; // 2*sqrt(2)*HALF_LENGTH_LEG

double ball_position[3] = {0, 0, 0.2};
double ball_predict_pos[3] = {0, 0, 0.2}; 
double old_ball_position[3] = {0, 0, 0.2};
double ball_moving_direction[2] = {0, 0};
double ball_velo;
//-----------SENSOR_VARIABLE--------------
#define NUMBER_OF_INFRARED_SENSORS 12
  // IR
double sensor_values[NUMBER_OF_INFRARED_SENSORS];
//   // KEYBOARD
// int old_key = -1;
  //  TRAPPED
int bumper_value;
//     // ROBOT DIR Value
double robot_dir_x, robot_dir_y, robot_dir_z;
int springer_counter = 0;
//     // ROBOT GPS Value
double gps_value[3];  
// ball goal distance
double distance_query = -1;
double dir_shot_x = 0, dir_shot_y = 0;
WbNodeRef ball_node;

bool missing_player[ROBOTS] = {0};
WbNodeRef player_def[ROBOTS];
WbNodeRef NAN_DEF;
double player_position[ROBOTS][3];

//---------------SHOOT
    // connector 
WbDeviceTag magnetic_sensor;
WbNodeRef magnetic_node;
//-------------------ball_kick_state
int ball_hold_counter = 0;
int ball_release_counter = 1;
int ball_possesion = 0, old_ball_possesion = -1; // 0-none, 1-hold, 2-shooted
    // linear motor
WbDeviceTag springer;


int time_step_counter = 0;


void check_counter_springer(){
  if (springer_counter-- < 0)
  {
    wb_motor_set_position(springer, 0.1);
    springer_counter = 0;
  }
}


#define FORCE2DISTANCE 0.7 //0.0926
#define MAX_FORCE 10

void on_springer(int on_timer, double distance_travel, double dir_shot_x, double dir_shot_y)  
{ 
  springer_counter = on_timer;  
  // double applied_force = FORCE2DISTANCE*distance_travel+0.56; 
  double applied_force = FORCE2DISTANCE*distance_travel+0.65; 


  // applied_force = 0.9*distance_travel;

  applied_force = MIN(applied_force, MAX_FORCE);  
  applied_force = MAX(applied_force, 3);

  std::cout << "                    APPLIED FORCE " << applied_force << " request_travel " << distance_travel << '\n'; 
  wb_motor_set_position(springer, 0.128); 
  // double mono_vec = length_vector(robot_dir_y, robot_dir_x);
  double cheat_vec = length_vector(dir_shot_y, dir_shot_x);

  // const double force[3] = {applied_force*robot_dir_y/mono_vec, -applied_force*robot_dir_x/mono_vec, 0};
  const double force[3] = {applied_force*dir_shot_x/cheat_vec, applied_force*dir_shot_y/cheat_vec, 0};

  // const double force[3] = {11, 9, 0};
  // const double torque[3] = {0.9, 0, 25};


  // const double force[3] = {9, 3, 20};
  // const double torque[3] = {0.1, 0, 8};



  // const double spin[3] = {0, 2, 0};
  // const double oset[3] = {0, -0.2, 0};

// cout << force[0] << ' ' << force[1] << '\n';
  // chip shot available
  // wb_supervisor_node_add_torque(ball_node, torque,0);

  wb_supervisor_node_add_force(ball_node, force,0);
  // wb_supervisor_node_add_force_with_offset(ball_node, spin, oset,1);


} 

//-------------PREDICT


void predicted()  
{ 
  // ball_velo; ball_moving_direction, ball_possesion; => ball_final_pos  
  // double friction = TIME_STEP/(1000*FRICTION); // NEED  TO BE CALCULATE BY HAND 
  double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 
  double travel_distance = ( (ball_velo - 0.55)/0.13 + 1.74  )/delta_ball_dir;  
  // printf("BALL TRAVEL_DIST %f and BALL_VELO %f\n", travel_distance, ball_velo);  
  // if (fabs(ball_velo) < 0.0001) travel_distance = 0.0001;  
  if (isinf(travel_distance)) travel_distance = 0;
  if (isnan(travel_distance)) travel_distance = 0;
  ball_predict_pos[0] =  ball_position[0] + ball_moving_direction[0]*travel_distance; 
  ball_predict_pos[1] =  ball_position[1] + ball_moving_direction[1]*travel_distance; 
  // if (SPEC_DEBUG_MODE) printf("TF %f BALL_VELO %f BALL FUTURE %f %f \n", ball_velo/friction, ball_velo, ball_predict_pos[0], ball_predict_pos[1]); 
} 

bool should_use_predict(){
  return 0;
   // length_vector(temp_vector[0], temp_vector[1]) > 2*ACTIVE_RANGE
   // fabs(angle_difference( atan2( temp_vector[1], temp_vector[0] ), atan2(ball_moving_direction[1], ball_moving_direction[0]) )) > degToRad(80) 
   // ball_velo > 0.8 
}

#define SMALL_RANGE 0.5
Omni_Vector react_move_to_position(double  current_x, double current_y, double t_pos_x, double t_pos_y, \
									double t_dir_x, double t_dir_y, bool special_offset = 0)
{

	// assert(use_ball_pos*dir_to_ball == 0);

//  double temp_vector[2] = { target_x - gps_value[0], target_y - gps_value[1]}; 
    double t_pos_vector[2] = { t_pos_x - current_x, t_pos_y - current_y}; 
 //  if (use_ball_pos){
 //    temp_vector[0] = target_x - ball_position[0];
 //    temp_vector[1] = target_y - ball_position[1];
 //  }

  // double Vec_Dir = atan2(temp_vector[1], temp_vector[0]); 
  double Vec_Dir = atan2(t_dir_y - current_y, t_dir_x - current_x); 

 //  if (dir_to_ball) Vec_Dir = atan2(ball_position[1] - gps_value[1], ball_position[0] - gps_value[0] );

  double Robot_Dir = current_robot_dir;
  double return_direction = angle_difference(Vec_Dir, Robot_Dir);


  	if (special_offset) {
		double amount = angle_difference(atan2(ball_position[1] - gps_value[1], ball_position[0] - gps_value[0]), Robot_Dir);
		// cout << "SPECIAL OFFSET " << amount << ' ' << radToDeg(amount) << '\n';
		return_direction += amount;
	}
	if (fabs(return_direction) < 0.03) return_direction = 0; 



  double temp_len = length_vector(t_pos_vector[0], t_pos_vector[1] );

  // *return_magnitude = 0;

  if (temp_len < 1.75 && temp_len > 0.001){
  	double temp_dir = atan2(t_pos_vector[1], t_pos_vector[0]);
  	double X = 1.7*(1.2-exp(-3*temp_len));
  		// cout << "                    ATAN2 " << temp_dir << "    LEN " << X << '\n';


  	t_pos_vector[0] = X * cos(temp_dir);
  	t_pos_vector[1] = X * sin(temp_dir);
  }
 //  else if (temp_len <){
 // //  	if (ball_velo > 2)
	// //     // t_pos_vector[0]*=-0.5, t_pos_vector[1] *=-0.5;
	// //     t_pos_vector[0]*= 0.1, t_pos_vector[1] *= 0.1;
	// // else
	//     t_pos_vector[0]*=0.8, t_pos_vector[1] *=0.8;
 //  }

  // cout << "ROBOT " << robot_encrypted_id << '\n';
  // cout << "rob vector " << t_pos_vector[0] << ' ' << t_pos_vector[1] << ' ' << atan2(t_pos_vector[1], t_pos_vector[0]) + degToRad(180) << ' ' << atan2(t_pos_vector[1], t_pos_vector[0]) - degToRad(180) << '\n';
  // // cout << "hooker " << gps_value[0] + 0.21 * cos(current_robot_dir) << " " << gps_value[1] + 0.21 * sin(current_robot_dir) << '\n';
  // cout << "ball vector " << ball_moving_direction[0] << ' ' << ball_moving_direction[1] << ' ' << atan2(ball_moving_direction[1], ball_moving_direction[0]) << '\n';
  // cout << "ball speed " << ball_velo << '\n';

  	// cout << "     ROBOT " << robot_encrypted_id << "  vx vy " << t_pos_vector[0] << ' ' << t_pos_vector[1] << "   command " << t_pos_x << ' ' << current_x << '\n';
	return Omni_Vector{t_pos_vector[0], t_pos_vector[1], -return_direction};
}




struct CachedPath{
	Path cached_p;
	int cached_time = -100;
	Dot path_goal;

	bool is_keiken(int current_time, Dot current_goal)
	{ return (abs(cached_time - current_time) < 5 && essentiallyEqual(current_goal.first, path_goal.first, 0.005) && essentiallyEqual(current_goal.second, path_goal.second, 0.005)); }

} cached;

Omni_Vector plan_rrt(Dot q_f, Dot bx = default_bx, Dot by = default_by, int dir_to_ball = 1, double EXTEND_RADIUS = 0){
	int this_id = robot_decrypt(robot_encrypted_id);
    Dot q_s = Dot(gps_value[0], gps_value[1]);
    if (dir_to_ball == 1) q_s = Dot(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir));

    if (stuck_with_goal(q_s)) q_f = Dot(0, 0);

    Env Cir;
 	if (BALL_OBS_ON) Cir.push_back(Circle{Dot(ball_position[0], ball_position[1]),1.5});

    for (int i = 0; i < ROBOTS; i++)
    	if (!missing_player[i] && i != this_id) 
    		Cir.push_back(Circle{Dot(player_position[i][0], player_position[i][1]),DISTANCE_WHEEL_TO_ROBOT_CENTRE+EXTEND_RADIUS+0.02});

    Path use_cached;
    if (cached.is_keiken(time_step_counter, q_f)) use_cached = cached.cached_p;

    int retrys = 0;
    Path r_path = path_plan(q_s, q_f, Cir, bx, by, use_cached, 0);
    Dot q_roll_back_f = Dot(-1000, -1000);
    Dot q_org_f = q_f;

  	double org_dir = atan2(q_s.second - q_f.second, q_s.first - q_f.first);
	// double org_dist = length_dist_vector(q_f.first, q_f.second, q_s.first, q_s.second);

    while (r_path.size() < 2 && retrys++ < 5){

	  	double temp_dist = retrys * 1;
	  	// temp_dist = MAX(temp_dist - 0.3, 0.001);
	  	q_f = Dot(temp_dist*cos(org_dir) + q_org_f.first, temp_dist*sin(org_dir) + q_org_f.second);

	  	if (!list_circle_collision_check(Cir, q_f) && q_roll_back_f.first < -500) q_roll_back_f = q_f;

    	r_path = path_plan(q_s, q_f, Cir, bx, by, use_cached, 0);

    	if (r_path.size() < 2){
	       	cout << "				 Robot " << this_id << " next_point " << r_path[1].first << ' ' << r_path[1].second << '\n';
    		cout << q_s.first << ' ' << q_s.second << " and q_f " << q_f.first << ' ' << q_f.second << '\n';
	    }
    	// for (auto i:Cir)
    	// 	cout << "Center " << i.Center.first << ' ' << i.Center.second << " radius " << i.radius << '\n';
	    // for (auto i : r_path)
	    //     cout << i.first << ' ' << i.second << ' ' << '\n';

	    // cout << "FUCK ENV " << Cir[0].Center.first << ' ' << Cir[0].Center.second << ' ' << Cir[0].radius << " Q_F " << q_f.first << ' ' << q_f.second << " Q__S" << q_s.first << ' ' << q_s.second << ' ' << '\n';

    }
    if (r_path.size() >= 2){
    	cached.cached_p= r_path;
    	cached.cached_time= time_step_counter;
    	cached.path_goal= q_f;

    	// cout << "CURRENT QUERIES\n";
    	// cout << "   RRT Robot " << this_id << " next_point " << r_path[1].first << ' ' << r_path[1].second << '\n';
    	// cout << q_s.first << ' ' << q_s.second << " and q_f " << q_f.first << ' ' << q_f.second << '\n';
    	// for (auto i:Cir)
    	// 	cout << "Center " << i.Center.first << ' ' << i.Center.second << " radius " << i.radius << '\n';
	    // for (auto i : r_path)
	    //     cout << i.first << ' ' << i.second << ' ' << '\n';

	    // cout << "FUCK ENV " << Cir[0].Center.first << ' ' << Cir[0].Center.second << ' ' << Cir[0].radius << " Q_F " << q_f.first << ' ' << q_f.second << " Q__S" << q_s.first << ' ' << q_s.second << ' ' << '\n';
    }
    else {
    	r_path.push_back(q_roll_back_f);
    	cached.cached_time = -100;
    }
    // else r_path = path_plan(q_s, q_f, Cir, bx, by, use_cached);

    	// cout << "    CHACED SIZE " << cached.cached_p.size() << '\n';

	// if (DEBUG_RRT) cout << "CURRENT QUERIES\n";
	// if (DEBUG_RRT)  
		// if (isnan(r_path[1].first) || cached.cached_p.size() == 1){
		// 	cout << "      RRT " << this_id << "r_pathsize " << r_path.size() << " next_point " << r_path[1].first << ' ' << r_path[1].second << " from " << gps_value[0] + 0.21 * cos(current_robot_dir) << " " << gps_value[1] + 0.21 * sin(current_robot_dir)<< '\n';
		// 	cout << "   RRT q_s q_f " << q_s.first << ' ' << q_s.second << " and q_f " << q_f.first << ' ' << q_f.second << '\n';
	 //    	for (auto i:Cir)
	 //    		cout << "Center " << i.Center.first << ' ' << i.Center.second << " radius " << i.radius << '\n';
	 //    	for (auto i:use_cached)
	 //    		cout << "cached " << i.first << ' ' << i.second << '\n';
	 //    	// std::exit(1);

	 //    	cout << " PATH NAN \n";
	 //    	for (auto i:r_path)
	 //    		cout << i.first << ' ' << i.second << '\n';
	 //    	cout << '\n';
		// }

    if (dir_to_ball == 1)
	    return react_move_to_position(q_s.first, q_s.second,  bound_x(r_path[1].first, bx.first, bx.second), bound_y(r_path[1].second, by.first, by.second), ball_position[0], ball_position[1]);
	else if (dir_to_ball == 0)
		return react_move_to_position(q_s.first, q_s.second,  bound_x(r_path[1].first, bx.first, bx.second), bound_y(r_path[1].second, by.first, by.second), q_f.first, q_f.second);
	else if (dir_to_ball == 2)
		return react_move_to_position(q_s.first, q_s.second,  bound_x(r_path[1].first, bx.first, bx.second), bound_y(r_path[1].second, by.first, by.second), 0, 0);
	else {
			cout << "                       WRONG CALL RRT \n";
		return Omni_Vector{0, 0, 0};
	}
}

Omni_Vector move_to_position(Dot q_f, Dot bx = default_bx, Dot by = default_by)
{  
  // if (INFO_MODE) printf("track_object position %f %f %f \n", temp_post[0], temp_post[1], temp_post[2]);
  return plan_rrt(q_f, bx, by, 1);
}

Omni_Vector move_to_object(WbNodeRef track_object, Dot bx = default_bx, Dot by = default_by)
{  
  const double *temp_post = wb_supervisor_node_get_position(track_object);
  // if (INFO_MODE) printf("track_object position %f %f %f \n", temp_post[0], temp_post[1], temp_post[2]);
  return plan_rrt(Dot(temp_post[0], temp_post[1]), bx, by, 0);
}

			Omni_Vector rotate_to_position(double target_x, double target_y)
			{
			  Omni_Vector V = react_move_to_position(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir), 0, 0, target_x, target_y);
			  V.vx *= 0.0001; V.vy *= 0.0001;

			  return V;
			}

			Omni_Vector ball_rotate_to_position(double target_x, double target_y)
			{
			  // Omni_Vector X = react_move_to_position(gps_value[0] - 0.21 * cos(current_robot_dir), gps_value[1] - 0.21 * sin(current_robot_dir), ball_position[0], ball_position[1], target_x, target_y);
			  Omni_Vector X = react_move_to_position(gps_value[0], gps_value[1], ball_position[0], ball_position[1], target_x, target_y, 1);
			  X.vx *= 0.01; X.vy *= 0.01;
			  return X;
			}

			Omni_Vector move_ball_rotate_to_position(double target_x, double target_y)
			{
			  Omni_Vector X = react_move_to_position(ball_position[0], ball_position[1], target_x, target_y, target_x, target_y);
			  X.vx *= 0.01; X.vy *= 0.01;
			  return X;
			}


			Omni_Vector rotate_to_object(WbNodeRef track_object)
			{
			  const double *temp_post = wb_supervisor_node_get_position(track_object);
			  return rotate_to_position(temp_post[0], temp_post[1]);
			}
    
double F_dynamic_error(double ball_goal_distance){
  // ball_goal small -> error bigger
		// return 0.015;
  if (ball_goal_distance < 3*ACTIVE_RANGE)
    return 0.11;
  else if (ball_goal_distance < 9*ACTIVE_RANGE)
    return 0.061;
  else if (ball_goal_distance < 18*ACTIVE_RANGE)
    return 0.031;
  else
    return 0.015;
}

void shoot_ball()
{
	// cout << "SHOOTING " << ball_possesion << " counter " << ball_hold_counter << '\n';
    if (wb_connector_get_presence(magnetic_sensor) == 1)
    {
      // wb_connector_lock(magnetic_sensor);
      if (++ball_hold_counter >= 1 && ball_possesion == 1) { // 2*0.032 seconds elapsed 
      // if (ball_possesion == 1){
        wb_connector_unlock(magnetic_sensor);
        on_springer(2, distance_query, dir_shot_x, dir_shot_y);
        ball_release_counter = -2;
        ball_hold_counter = 0;

        // current_state = 0;
        ball_possesion = 2;
      }
    }
}

void idle() { base_reset(); }


void move_to_ball(Dot bx = default_bx, Dot by = default_by, double scale_velo = 10) 
{ 
  // if (SPEC_DEBUG_MODE) printf("DIFF CHECK BALL %f\n", angle_difference( atan2( temp_vector[1], temp_vector[0]), atan2(ball_moving_direction[1], ball_moving_direction[0]); 
	if (wb_connector_get_presence(magnetic_sensor) == 0){

		  bool ENABLE_PREDICT = 0;
		  if (should_use_predict() && ENABLE_PREDICT) 
		  { 
		    predicted();
		    component_vector[0] = plan_rrt(Dot(ball_predict_pos[0], ball_predict_pos[1]), bx, by); 
		    // if (SPEC_DEBUG_MODE) printf("PREDICTED BALL TO VECTOR 0\n"); 
		    // return Omni_Vector{0, 0, 0};
		  } 
		  else  
		  { 

		    // move_to_object(ball_node, &field_vector_magnetude[0], &field_vector_direction[0]);  
		    // if (SPEC_DEBUG_MODE) printf("STRAIGHT BALL TO VECTOR 0\n");  
		  	component_vector[0] = plan_rrt(Dot(ball_position[0], ball_position[1]), bx, by, 1, -0.2);
		  	component_vector[0].ippai(scale_velo);
// WHY HERE 
			// double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 
			// if (ball_velo > 0.02)
			// 	component_vector[1] = Omni_Vector{ball_moving_direction[0] * ball_velo / delta_ball_dir, ball_moving_direction[1] * ball_velo / delta_ball_dir, 0};

		  } 
	}
	else if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
  // field_vector_magnetude[1] = ball_velo*0.5,  
  // field_vector_direction[1] = angle_difference( atan2(ball_moving_direction[1], ball_moving_direction[0]), current_robot_dir); 
  // if (STUCKED_TIME <= 15){
  //   field_vector_magnetude[2] = 0;
  //   field_vector_direction[2] = 0;
  // }
}

void get_ball(){
	if (wb_connector_get_presence(magnetic_sensor) == 0){
		move_to_ball(extend_bx, extend_by, 5);
	}
	else {
		if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
	}
}

// void chase_ball_verGood(double IP_x, double IP_y){
// 	// if (ball_velo > 0.8) 
// 	// 	component_vector[0] = plan_rrt(Dot(IP_x, IP_y), default_bx, default_by),
// 	// 	component_vector[0].ippai(10);
// 	// else 
// 	if (wb_connector_get_presence(magnetic_sensor) == 0){

// 		// if (ball_velo < 1.8){
// 			double current_dist = length_dist_vector(ball_position[0], ball_position[1],   gps_value[0], gps_value[1]);

// 			move_to_ball(extend_bx, extend_by, exp(-current_dist/1.5)*3 );



// 			// double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 
// 			// if (ball_velo > 0.02)
// 			// 	component_vector[1] = Omni_Vector{ball_moving_direction[0] * ball_velo / delta_ball_dir, ball_moving_direction[1] * ball_velo / delta_ball_dir, 0},
// 			// 	component_vector[1].ippai( exp(ball_velo/4)-1 );



// 			component_vector[2] = react_move_to_position(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir), IP_x, IP_y, ball_position[0], ball_position[1]);
// 			component_vector[2].ippai(exp(current_dist/2)-1);

// 	}
// 	else {
// 		if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
// 	}
// }


#define V_MAX_ROB 1.2


void chase_ball_verSafe(double IP_x, double IP_y){
	if (wb_connector_get_presence(magnetic_sensor) == 0){
			double t_ball2IP = length_dist_vector(ball_position[0], ball_position[1],  IP_x, IP_y) / ball_velo;
			double t_rob2IP  = length_dist_vector(gps_value[0], gps_value[1],  IP_x, IP_y) / V_MAX_ROB;
			double current_dist_2ball = length_dist_vector(ball_position[0], ball_position[1],  gps_value[0], gps_value[1]);

			if (t_rob2IP < t_ball2IP && ball_velo > 100){

				component_vector[2] = react_move_to_position(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir), \
				              bound_x(IP_x, default_bx.first, default_bx.second), bound_y(IP_y, default_by.first, default_by.second), ball_position[0], ball_position[1]);
				component_vector[2].ippai(3);
			}
			else {
				move_to_ball(default_bx, default_by, exp(-current_dist_2ball/3)*2);
					double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 
					// if ( delta_ball_dir > 0.5 && fabs( fabs(IP_x) - HIGH_BOUND_X) > 1 && fabs( fabs(IP_y) - HIGH_BOUND_Y) > 1 ){
					if ( delta_ball_dir > 0.02 && fabs( fabs(IP_x) - HIGH_BOUND_X) > 1 && fabs( fabs(IP_y) - HIGH_BOUND_Y) > 1 ){
						component_vector[1] = Omni_Vector{ball_moving_direction[0] * ball_velo / delta_ball_dir, ball_moving_direction[1] * ball_velo / delta_ball_dir, 0};
						component_vector[1].ippai(1.2 );
					}


				if (current_dist_2ball > 0.5){
					component_vector[2] = react_move_to_position(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir), \
				              IP_x, IP_y, ball_position[0], ball_position[1]);
					component_vector[2].ippai(1.3);
				}

			}
	}
	else {
		if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
	}
}


void chase_ball(double IP_x, double IP_y){
	// if (ball_velo > 0.8) 
	// 	component_vector[0] = plan_rrt(Dot(IP_x, IP_y), default_bx, default_by),
	// 	component_vector[0].ippai(10);
	// else 
	if (wb_connector_get_presence(magnetic_sensor) == 0){

		// if (ball_velo < 1.8){
			double t_ball2IP = length_dist_vector(ball_position[0], ball_position[1],  IP_x, IP_y) / ball_velo;
			double t_rob2IP  = length_dist_vector(gps_value[0], gps_value[1],  IP_x, IP_y) / V_MAX_ROB;
			double current_dist_2ball = length_dist_vector(ball_position[0], ball_position[1],  gps_value[0], gps_value[1]);
			// double current_dist_2IP = length_dist_vector(IP_x, IP_y,  gps_value[0], gps_value[1]);
			double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 

			// cout << "CHASE_BALL t__ball " << t_ball2IP << "  t___rob " << t_rob2IP << " ball-velo " << ball_velo << '\n';
			if ( ball_velo > 10000){

				component_vector[2] = react_move_to_position(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir), \
				              bound_x(IP_x, default_bx.first, default_bx.second), bound_y(IP_y, default_by.first, default_by.second), ball_position[0], ball_position[1]);
				// component_vector[2].ippai(exp(current_dist/1)-1);				
				component_vector[2].ippai(3);
			}
			else {
// SAFE CHOICE (3; 2|   1.5 | 1)


				// move_to_ball(default_bx, default_by, exp(-current_dist_2ball*current_dist_2ball/2)*2);

				// cout <<"                 MBALL " <<  component_vector[0].vx << " " << component_vector[0].vy << " here out of bound \n";

				// if (current_dist_2ball > 2)
					move_to_ball(default_bx, default_by, exp(-current_dist_2ball/3)*2 * exp(-0.2*delta_ball_dir*delta_ball_dir)  );
				// else
				// 	move_to_ball(default_bx, default_by, exp(-current_dist_2ball*current_dist_2ball*current_dist_2ball*current_dist_2ball*0.04)*2);

// e^{-0.08\cdot x^{4}}\cdot2

				// double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 
				// if (ball_velo > 0.02)
				// 	component_vector[1] = Omni_Vector{ball_moving_direction[0] * ball_velo / delta_ball_dir, ball_moving_direction[1] * ball_velo / delta_ball_dir, 0},
				// 	component_vector[1].ippai( exp(ball_velo/6)-1 );
		

					// if ( delta_ball_dir > 0.5 && fabs( fabs(IP_x) - HIGH_BOUND_X) > 1 && fabs( fabs(IP_y) - HIGH_BOUND_Y) > 1 ){
					// if ( delta_ball_dir > 0.02 && fabs( fabs(IP_x) - HIGH_BOUND_X) > 1 && fabs( fabs(IP_y) - HIGH_BOUND_Y) > 1 ){
					if (delta_ball_dir > 0.0001 && fabs( fabs(IP_x) - HIGH_BOUND_X) > 1 && fabs( fabs(IP_y) - HIGH_BOUND_Y) > 1 ){
						component_vector[1] = Omni_Vector{ball_moving_direction[0] * ball_velo / delta_ball_dir, ball_moving_direction[1] * ball_velo / delta_ball_dir, 0};
						component_vector[1].ippai(MAX(0, 2-exp(0.1/sqrt(delta_ball_dir))));
					// 	// component_vector[1].ippai(MAX(1, exp(ball_velo/6)/2 ));
					}


				// if (current_dist_2ball > 0.25){
					component_vector[2] = react_move_to_position(gps_value[0] + 0.21 * cos(current_robot_dir), gps_value[1] + 0.21 * sin(current_robot_dir), \
				              IP_x, IP_y, ball_position[0], ball_position[1]);
					component_vector[2].ippai(MAX(0, 0.96-0.38/current_dist_2ball));
					// if (current_dist_2ball > 2) component_vector[2].ippai(1.3);
				// }
					// cout <<"                 MIP " <<  component_vector[2].vx << " " << component_vector[2].vy << " here the problem " << IP_x << ' ' << IP_y << "\n";


				// component_vector[2].ippai(exp(current_dist_2ball*current_dist_2ball/6)-1);

			}




	}
	else {
		if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
	}
}

void cut_ball(double IP_x, double IP_y){
	// if (ball_velo > 0.8) 
	// 	component_vector[0] = plan_rrt(Dot(IP_x, IP_y), default_bx, default_by),
	// 	component_vector[0].ippai(10);
	// else 
	if (wb_connector_get_presence(magnetic_sensor) == 0){
		component_vector[2] = react_move_to_position(gps_value[0], gps_value[1], \
		              bound_x(IP_x, default_bx.first, default_bx.second), bound_y(IP_y, default_by.first, default_by.second), ball_position[0], ball_position[1]);
		component_vector[2].ippai(4);
	}
	else {
		if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
	}
}




// void foo(Dot q_enemy_holder){
// 	if (wb_connector_get_presence(magnetic_sensor) == 0){
// 		Dot cut_position = Dot(q_enemy_holder.first + (ball_position[0] - q_enemy_holder.first) * 2, q_enemy_holder.second + (ball_position[1] - q_enemy_holder.second) * 2);
// 		// cout << "enemy " << q_enemy_holder.first << ' ' << q_enemy_holder.second << '\n';
// 		// cout << "cut_pos " << cut_position.first << " " << cut_position.second << '\n';
// 		// if (los)
// 		component_vector[0] = move_to_position(cut_position ,default_bx, default_by);
// 		component_vector[0].ippai(3);
// 	}
// 	else {
// 		if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
// 	}
// }

		// APPROXIMATE PASS
		void pass_ball(int robot_id, Dot bx = default_bx, Dot by = default_by) // according name field
		{
		  if (wb_connector_get_presence(magnetic_sensor) == 0){
		    move_to_ball(bx, by);
		  }
		  else
		  {
		    if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
		    component_vector[0] = ball_rotate_to_position(player_position[robot_id][0], player_position[robot_id][1]);
		    // distance_query *= 1.3;
		    double error_angle = F_dynamic_error(distance_query);
		    // if (fabs(component_vector[0].vx) + fabs(component_vector[0].vy) + fabs(component_vector[0].vw) > 1.2) error_angle *= 3;
		    // special calc.
		    // double delta_angle = current_robot_dir - atan2( ball_position[1] - gps_value[1], ball_position[0] - gps_value[0] );
		    // field_vector_direction[0] += delta_angle;
		    // cout << "distance " << distance_query << " delta_a " << delta_angle << " permited " << error_angle << " now " << fabs(field_vector_direction[0]) << '\n';
		    // cout << "POD "<< field_vector_direction[2] << " stuck " << STUCKED_TIME << " pod velo " << field_vector_magnetude[2] << '\n';
		    dir_shot_x = player_position[robot_id][0] - ball_position[0];
		    dir_shot_y = player_position[robot_id][1] - ball_position[1];
		    if (fabs(component_vector[0].vw) <= error_angle ) shoot_ball();
		    // if (fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 1.2) shoot_ball();
		  }
		}

// FUCK CHECK 17/OCT ?????
void dribble(double e_radius, Dot bx = default_bx, Dot by = default_by) 
{
  Point mid_goal = get_goal(robot_decrypt(robot_encrypted_id), 0);
  if (e_radius == -1){// dont care 'bout ball though
    // component_vector[2].ippai(2);
    component_vector[0] = plan_rrt(Dot(mid_goal.first, mid_goal.second), bx, by, 1, 0);
	component_vector[0].ippai(4);

  }
  else { // need to keep ball
    // component_vector[2].ippai(e_radius);

    if (wb_connector_get_presence(magnetic_sensor) == 0)
      move_to_ball(bx, by);
    else{
      if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
      component_vector[0] = plan_rrt(Dot(mid_goal.first, mid_goal.second), bx, by, 0, e_radius);
      component_vector[0].ippai(5);
      // if ( fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 2.1 ) shoot_ball();
    }
  }
}

// FUCK CHECK 17/OCT ?????
void x_dribble(double x, double y, Dot bx = default_bx, Dot by = default_by) 
{
    if (wb_connector_get_presence(magnetic_sensor) == 0)
      move_to_ball(bx, by);
    else{
      if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
      component_vector[0] = plan_rrt(Dot(x, y), bx, by, 0, 0.8);
      component_vector[0].ippai(5);
      // if ( fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 2.1 ) shoot_ball();
    }
}


#define ONE_SEC_2_PI 2.05
#define ONE_SEC_2_M  2.5
#define DELTA_PI degToRad(36)
#define NUM_DELTA_PI 360/36

double get_min_dist_offteam(Point Q, int id_team){
	double min_dist = 10000;
	for (int i = 0; i < ROBOTS; i++){
		if (missing_player[i] || get_team(id_team) * get_team(i) > 0) continue;
		min_dist = MIN(min_dist, length_dist_vector(Q.first, Q.second, player_position[i][0], player_position[i][1]));
	}
	if (min_dist > 5000) min_dist = 0;
	return min_dist;
}

double simple_fixed_self(Point Start, Point Sink, Point Target, double offset_z){
	double move_reward = 0; // combine move from Start -> Sink and shoot from Sink -> Target with offset_z
	double shot_reward = 0;

	int this_id = robot_decrypt(robot_encrypted_id);
	move_reward = get_min_dist_offteam(Sink, this_id);

	double minimal_dist = 10000;
	double travel_length = length_dist_point(Sink, Target);
	int SIMPLE_CUT = 4;
	for (int i = 1; i <= SIMPLE_CUT; i++)
	{
		double t_x = Sink.first + (Target.first - Sink.first) * travel_length * i / SIMPLE_CUT;
		double t_y = Sink.second + (Target.second - Sink.second) * travel_length * i / SIMPLE_CUT;

		minimal_dist = MIN(minimal_dist, get_min_dist_offteam(Point{t_x, t_y}, this_id));
	}
	shot_reward = minimal_dist;

	return move_reward + shot_reward;
}

void fixed_self(double delta_w, Point q_goal){
	if (!SHOOT_FIXED_MODE) return;
	if (on_border(Point{gps_value[0], gps_value[1]}, 3) == false) return;

	double t_offset = fabs(delta_w) / ONE_SEC_2_PI;

	Point best_point = q_goal;
	double best_reward = -10000;

	for (int i = 0; i < NUM_DELTA_PI; i++){
		double test_dir = DELTA_PI * i;

		double t_x = bound_x( gps_value[0] + cos(test_dir) * t_offset * ONE_SEC_2_M, default_bx.first, default_bx.second);
		double t_y = bound_y( gps_value[1] + sin(test_dir) * t_offset * ONE_SEC_2_M, default_by.first, default_by.second);
		Point t_p = Point{t_x, t_y};

		// get offset = t_offset;
		double t_reward = simple_fixed_self(Point{gps_value[0], gps_value[1]} , t_p, q_goal, t_offset);

		if (t_reward > best_reward)
			best_reward = t_reward,
			best_point = t_p;
	}

	component_vector[0] = react_move_to_position(gps_value[0], gps_value[1], best_point.first, best_point.second, q_goal.first, q_goal.second);	

	// if (SHOOT_FIXED_MODE) 
	// 	cout << "fixed move from " << gps_value[0] << ' ' << gps_value[1] << " to " << best_point.first << ' ' << best_point.second << " q_dir " << q_goal.first << ' ' << q_goal.second << '\n',
	// 	cout << "         simplified rew " << best_reward << " remainder_delta_q " << delta_w << " t_off " << t_offset << '\n',
	// 	cout << "    compare_delta_q OLD " << delta_w << " NEW " << component_vector[0].vw << '\n';

}

		void field_shoot(double x, double y, Dot bx = default_bx, Dot by = default_by)
		{

		  if (wb_connector_get_presence(magnetic_sensor) == 0){
		    move_to_ball(bx, by);
		  }
		  else
		  {
		  	wb_connector_lock(magnetic_sensor);
		    if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
		    component_vector[0] = ball_rotate_to_position(x, y);
		    distance_query = length_dist_vector(ball_position[0], ball_position[1], x, y) - (0.06+DISTANCE_WHEEL_TO_ROBOT_CENTRE);

		    // cout << " request_distance b4     " << distance_query << '\n';

		    if (distance_query < 3) distance_query *= 1.35;
		    else 
	    	if (distance_query < 6) distance_query *= 1.47;
		    else 
			if (distance_query < 13) distance_query *= 1.2;
		    else distance_query *= 1.2;

		    double error_angle = F_dynamic_error(distance_query);

		    dir_shot_x = x - ball_position[0];
		    dir_shot_y = y - ball_position[1];

		    // distance_query *= 1.3;
		    // if (fabs(component_vector[0].vx) + fabs(component_vector[0].vy) + fabs(component_vector[0].vw) > 1.2) error_angle *= 3;
		    if (SHOOT_FIXED_MODE) cout << "FIELD SHOOT AT " << x << ' ' << y << "\n",
// realy here ?
fixed_self(component_vector[0].vw, Point{x, y});

		    if (fabs(component_vector[0].vw) <= error_angle ) 
		    	shoot_ball();
		  }

		}

		void goal_shoot(int goal_id, bool allow_move, Dot bx = default_bx, Dot by = default_by)
		{
		  if (wb_connector_get_presence(magnetic_sensor) == 0)
		  {
		    move_to_ball(bx, by);
		  }
		  else
		  {
		    if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
		    Point goal = get_goal(robot_decrypt(robot_encrypted_id), goal_id);
		    component_vector[0] = ball_rotate_to_position(goal.first, goal.second);

		    // cout << "GOAL CHECK " << goal.first << ' ' << goal.second << " rotate" <<  radToDeg(component_vector[0].vw) << " \n";
// if (fabs(component_vector[0].vw)> ??? ) component_vector[0] = ball_rotate_to_position(goal.first, goal.second - ???)

// component_vector[0].ippai(100);

		    // for (j = 1; j < 3; j++)
		    // {
		    //   double temp_magnetude, temp_direction;
		    //   rotate_to_position(goal_position_x[j], goal_position_y[j], &temp_magnetude, &temp_direction);
		    //   if (fabs(temp_direction) < fabs(field_vector_direction[0])) field_vector_direction[0] = temp_direction;
		    // }
		    // cout << "damn " << goal.first << ' ' << goal.second << '\n';
		    if (SHOOT_FIXED_MODE) cout << "GOAL SHOOT " << goal.first << ' ' << goal.second << " \n";
		    if (allow_move) fixed_self(component_vector[0].vw, goal);
		    distance_query = 100000;
		    dir_shot_x = goal.first - ball_position[0];
		    dir_shot_y = goal.second - ball_position[1];
		    double error_angle = F_dynamic_error(distance_query)*0.001;
// error_angle = 0.001;
		    if (fabs(component_vector[0].vw) <= error_angle )  shoot_ball();			    
		  }

		}

// 		void manual_shoot(double manual_dir_x,  double manual_dir_y, Dot bx = default_bx, Dot by = default_by)
// 		{
// 	    if (++ball_release_counter > 0) wb_connector_lock(magnetic_sensor);
// 	    component_vector[0] = ball_rotate_to_position(manual_dir_x, manual_dir_y);

// 	    distance_query = 100000;
// 	    dir_shot_x = manual_dir_x - ball_position[0];
// 	    dir_shot_y = manual_dir_y - ball_position[1];
// 	    double error_angle = F_dynamic_error(distance_query)*0.001;
// // error_angle = 0.001;
// 	    if (fabs(component_vector[0].vw) <= error_angle )  shoot_ball();			    

// 		}


void force_shoot(){
    // dangerous, carefull to use
	shoot_ball();
	idle();
}

void force_release(){
	wb_connector_unlock(magnetic_sensor);
	component_vector[0] = plan_rrt(Dot(0, 0), extend_bx, extend_by, 1);
}


void soft_release(){
	wb_connector_unlock(magnetic_sensor);
	component_vector[0] = react_move_to_position(gps_value[0], gps_value[1], ball_position[0], ball_position[1], ball_position[0], ball_position[1]);
}

void pressing(double x, double y, Dot bx = default_bx, Dot by = default_by)
{
  if (BALL_OBS_ON)
	  component_vector[0] = plan_rrt(Dot(x, y), extend_bx, extend_by, 1, 0.1);
  else
	  component_vector[0] = plan_rrt(Dot(x, y), default_bx, default_by, 1, -0.5); //react_move_to_position(gps_value[0], gps_value[1], x, y, ball_position[0], ball_position[1]);
}

		void capture(double x, double y, Dot bx = default_bx, Dot by = default_by)
		{

		  if (ball_possesion == 0){
			  component_vector[0] = plan_rrt(Dot(x, y), bx, by, 1);
			  if (length_dist_vector(x, y, gps_value[0], gps_value[1]) < 0.8) component_vector[0] = rotate_to_object(ball_node);
		  }
		  else{
			  component_vector[0] = plan_rrt(Dot(x, y), bx, by, 0);
		  }
		  // if (STUCKED_TIME <= 5){
		  //   field_vector_magnetude[2] = 0;
		  //   field_vector_direction[2] = 0;
		  // }
		}

		void free_capture(double x, double y)
		{
		  component_vector[0] = plan_rrt(Dot(x, y), extend_bx, extend_by, 2);
		  // if (length_dist_vector(x, y, gps_value[0], gps_value[1]) < 0.04) 
		  	// component_vector[0] = rotate_to_position(0, 0);
		}


// need to fix the cir bounding check
void tackle(int opp_player_id, Dot bx = default_bx, Dot by = default_by)
{
  assert(opp_player_id >= 0);
  if (missing_player[opp_player_id])
    assert(False);
  else
    component_vector[0] = move_to_object(player_def[opp_player_id], bx, by);
  
  // if (STUCKED_TIME <= 15){
  //   field_vector_magnetude[2] = 0;
  //   field_vector_direction[2] = 0;
  // }
}


int bending_mode = 0; // 0: TBA, 1: Manual, 3: Auto-ball, 2: Auto-goal
int counter_shoot = 0;

void manual_control(int navi_command, int bend_command, int ball_command){
	int mask_key_pressed = navi_command - 224;

	if (CHECK_BIT(mask_key_pressed, 0)) component_vector[2].vy += 4;
	if (CHECK_BIT(mask_key_pressed, 1)) component_vector[2].vy -= 4;
	if (CHECK_BIT(mask_key_pressed, 2)) component_vector[2].vx -= 4;
	if (CHECK_BIT(mask_key_pressed, 3)) component_vector[2].vx += 4;

	if (bend_command == 1) component_vector[2].vw = degToRad(30), bending_mode = 1;
	else if (bend_command == 2) component_vector[2].vw = degToRad(-30), bending_mode = 1;
	else if (bend_command == 8 or (bend_command == 0 && bending_mode == 3)){

    bending_mode = 3;
    if (ball_possesion == 0) {
      if (mask_key_pressed == 0){
        component_vector[2] = plan_rrt(Dot(ball_position[0], ball_position[1]), default_bx, default_by, 1, -0.2);
        component_vector[2].ippai(5);
      }
      else {
        component_vector[2].vw = rotate_to_position(ball_position[0], ball_position[1]).vw * (ball_possesion==0); 
      }
    }
    else{
      wb_connector_lock(magnetic_sensor);
      // Point goal = get_goal(robot_decrypt(robot_encrypted_id), 0);
      // component_vector[2].vw = ball_rotate_to_position(goal.first, goal.second).vw;
		  // component_vector[2].vw = rotate_to_position(ball_position[0], ball_position[1]).vw * (ball_possesion==0);
    }
  }
	else if (bend_command == 4 or (bend_command == 0 && bending_mode == 2)){
    bending_mode = 2;

    // still blank
	}

	if (ball_command == 2){
		counter_shoot = 1; // start_counting
	}
	else if (ball_command == 1 && counter_shoot >= 1){		
		counter_shoot += 1;
	}
	else if (ball_command == 3){
		distance_query = MAX(2, 1.6*log(10*counter_shoot + 1));
		double t_vx  = (ball_position[0] - gps_value[0]);
		double t_vy  = (ball_position[1] - gps_value[1]);
		double t_lxy = length_vector(t_vx, t_vy);

		dir_shot_x = ball_position[0] + distance_query * t_vx/t_lxy;
		dir_shot_y = ball_position[1] + distance_query * t_vy/t_lxy;

		cout << counter_shoot << " d x y " << distance_query << ' ' << dir_shot_x << ' ' << dir_shot_y << '\n';

		field_shoot(dir_shot_x, dir_shot_y);
		shoot_ball();

		counter_shoot = 0;
	}
	else counter_shoot = 0;

	// if (ball_command == 2){
	// 	goal_shoot(0, 0);
	// }

}

#endif
