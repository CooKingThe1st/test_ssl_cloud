#ifndef INTER_H
#define INTER_H

#include "Setup_UI.h"
#include <stdlib.h>
#include "..\omni_mobile\geometry.h"

int player_ball[ROBOTS];
// ball variable
int non_last_ball = -1, spn_last_ball = -1;
int player_last_touch = -1, cached_player_last_touch = -1;
bool ball_check_hold = 1;


//  ball and robot pose
WbNodeRef ball_node;
WbNodeRef goal_node[2];
double ball_reset_timer = 0;
double ball_velo = 0;
double ball_position[3] = {0, 0, 0.2};
double old_ball_position[3] = {0, 0, 0.2}; 
double ball_moving_direction[2] = {0, 0};
 
double ball_initial_position[3] = {0, 0, 0.2};
double goal_position_x[2][3] = {0}, goal_position_y[2][3] = {0}; 

//-------------USEFUL-FUNCTION-----------
double GOAL_X_LIMIT = 1000;
double GOAL_Y_LIMIT = 1.3;
double GOAL_Z_LIMIT = 1.37;
double GOAL_WIDTH = 0.85;

int check_goal(){
	if (fabs(ball_position[1]) > GOAL_Y_LIMIT || fabs(ball_position[2]) > GOAL_Z_LIMIT || fabs(ball_position[0]) <= GOAL_X_LIMIT) return 0;
	// if (ball_position[0] > GOAL_X_LIMIT && ball_position[0] < GOAL_X_LIMIT + GOAL_WIDTH+1.5) // blue goal return 1
	if (ball_position[0] > GOAL_X_LIMIT)
		return 1;
	// else if (ball_position[0] < -GOAL_X_LIMIT && ball_position[0] > -GOAL_X_LIMIT - GOAL_WIDTH-1.5) // red goal return -1
	else{ 
		// cout <<  (ball_position[0]) << " " << -GOAL_X_LIMIT ;
		return -1;
	}
}

int check_linesman() // throw-in, touchline, out of bound, goal
{
	if (check_goal() != 0) return -1;
	else if (fabs(ball_position[0]) > HIGH_BOUND_X+0.04)  return 2; // corner-kick
			// cout << "x y z " << fabs(ball_position[0]) << ' ' << fabs(ball_position[1]) << ' ' << fabs(ball_position[2]) << '\n';
			// cout << "lim " << GOAL_X_LIMIT << ' ' << GOAL_Y_LIMIT << ' ' << GOAL_Z_LIMIT << '\n';
	else if (fabs(ball_position[1]) > HIGH_BOUND_Y+0.04 )  return 1; // throw-in
	else return 0;
}

int check_stuck()
{
	int check_non = 0, check_spn = 0;
	for (int i = 0; i < 7; i++) 
		if (player_ball[i] != 0) {
			if (check_non == 0) check_non = -1;
			else check_non = -3;
		}
	for (int i = 7; i < ROBOTS; i++)
		if (player_ball[i] != 0) {
			if (check_spn == 0) check_spn = 2;
			else check_spn = 4;
		}

	if (check_spn != 0 && check_non != 0) return 1; // in dispute
	else return 0;
}

Point get_linesman(int vio_type)
{
	if (vio_type == 2){ // corner kick
		return Point{bound_x(ball_position[0]), bound_set(ball_position[1], HIGH_BOUND_Y)};
	}
	else if (vio_type == 1){
		return Point{ball_position[0], bound_y(ball_position[1])};
	}
	else {
		assert(false); return Point{-10000, -10000};
	}
}

int get_closest_robot_off_team_off_gk(Point ref_point, bool *missing, double player_pos[][3], int query_id){
	int closest_robot = -1;
	for (int i = 0; i < ROBOTS; i++){
		if (missing[i] || i == 0 || i == 7) continue;
		if (query_id != -1 && (get_team(i) * get_team(query_id) > 0) ) continue;// only find opponent, and specify player_team
		if (closest_robot == -1 || length_dist_point( Point{player_pos[i][0], player_pos[i][1]}, ref_point) < \
			length_dist_point( Point{player_pos[closest_robot][0], player_pos[closest_robot][1]}, ref_point) )
			closest_robot = i;
	}
	return closest_robot;
}


void check_interupt(bool *missing, double player_pos[][3], int last_touch){

	if (IntPack.vio_type == -1){


		int freekick_signal = check_linesman();
		int stuck_signal = check_stuck();
		if (freekick_signal != 0) cout << "FREE_CKIK SIGNAL " << freekick_signal << '\n';

		if (freekick_signal == 0 && stuck_signal == 0) interupt_color = 0x6DFF33;
		else if (freekick_signal != 0){ // priotize free kick
			if (freekick_signal == 1) { // border kick
				interupt_color = 0xecfc03;
				assert(last_touch != -1);
				IntPack.VioPos = get_linesman(freekick_signal);
				cout << " CHECK BORDER KICK with last_touch " << last_touch << '\n';
				IntPack.executor = get_closest_robot_off_team_off_gk(Point{ball_position[0], ball_position[1]}, missing, player_pos, last_touch);
				if (IntPack.executor == -1) IntPack.executor = (get_team(last_touch) * get_team(0) > 0) ? 7 : 0;

				cout << "THE executor " << IntPack.executor << '\n';
				cout << "VIO POS " << IntPack.VioPos.first << ' ' << IntPack.VioPos.second << '\n';
				IntPack.vio_type = 1;

				// cout << "FAULTY by " << last_touch << " and exe by " << IntPack.executor  << '\n';

			}
			else if (freekick_signal == 2) { // corner
				assert(last_touch != -1);
				double which_goal_side = -(get_goal(last_touch, 0).first);
				

				// cout << " GOAL SIDE " << which_goal_side << " by who " << last_touch << " ball_pos " << ball_position[0] << '\n';
				if ((which_goal_side > 0 && ball_position[0] > 0) or  (which_goal_side < 0 && ball_position[0] < 0) ){
					interupt_color = 0xfc3003; // corner kick
					IntPack.VioPos = get_linesman(freekick_signal);
					cout << " CHECK CORNER KICK with last_touch " << last_touch << '\n';

					IntPack.executor = get_closest_robot_off_team_off_gk(Point{ball_position[0], ball_position[1]}, missing, player_pos, last_touch);
					IntPack.vio_type = 2;

					if (IntPack.executor == -1) IntPack.executor = (get_team(last_touch) * get_team(0) > 0) ? 7 : 0;
					cout << "THE executor " << IntPack.executor << '\n';

					// cout << "     FAULTY by " << last_touch << " and exe by " << IntPack.executor  << '\n';

				}
				else {
					interupt_color = 0xac03fc; // goal- free kick
					IntPack.VioPos = Point{ HIGH_BOUND_X - 1 , 0};
						if  (ball_position[0] < 0) IntPack.VioPos.first *= -1;

					cout << " CHECK FREE KICK with last_touch " << last_touch << '\n';

					if (ball_position[0] < 0){
						IntPack.executor = 7;
						if (missing[7]) IntPack.executor = get_closest_robot_off_team_off_gk( Point{ball_position[0], ball_position[1]}, missing, player_pos, last_touch );
					}
					else {
						IntPack.executor = 0;
						if (missing[0]) IntPack.executor = get_closest_robot_off_team_off_gk( Point{ball_position[0], ball_position[1]}, missing, player_pos, last_touch);
					}
					cout << "THE executor " << IntPack.executor << '\n';
					IntPack.vio_type = 3;
				}

			}
			else { // goal
				interupt_color = 0x03f8fc;
				// IntPack.vio_type = 100;
			}
		}
		else if (stuck_signal != 0){ // only stuck
			interupt_color = 0xfc03e3;
		}
	}
	else{
		if (length_dist_vector(ball_position[0], ball_position[1], IntPack.VioPos.first, IntPack.VioPos.second) < 1.2){
			if (IntPack.executor == -1 || missing[IntPack.executor]) {
				cout << "hmm get executor false \n";
			} else if (player_ball[IntPack.executor] == 2){
				IntPack.reset();
			}
		}
	}
}

#endif
