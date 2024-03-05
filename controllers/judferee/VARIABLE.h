#ifndef VAR_H
#define VAR_H

#include <webots/display.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>


// SYSTEM VAR
#define ROBOTS (7*2)  // number of robots
int CURRENT_BRAIN_LEVEL = 3;// 0-> random, 1->Naive, 3->mixed
double TIME_STEP;
unsigned int time_step_counter = 1;

WbNodeRef vpoint;

// BALL VAR

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

// PLAYER VAR
WbNodeRef player_def[ROBOTS];
WbNodeRef NAN_DEF;
double player_position[ROBOTS][3], player_rotation[ROBOTS][4];
double player_initial_position[ROBOTS][3], player_initial_rotation[ROBOTS][4];

const char *robot_name[ROBOTS] = {"NON_GK", "NON_CB", "NON_LB", "NON_RB", "NON_ST", "NON_LW", "NON_RW",
                                  "SPN_GK", "SPN_CB", "SPN_LB", "SPN_RB", "SPN_ST", "SPN_LW", "SPN_RW"};
const char *robot_webot_name[ROBOTS] = {"NON_001", "NON_002", "NON_005", "NON_004", "NON_009", "NON_010", "NON_011",
                                        "SPN_001", "SPN_002", "SPN_005", "SPN_004", "SPN_009", "SPN_010", "SPN_011"};
int map_id[12] = {-1, 0, 1, -1, 3, 2, -1, -1, -1, 4, 5, 6};
                //   gk  cb     rb lb             st lw rw
inline int robot_decrypt(int en_id) { return (en_id >= 500) ? map_id[en_id - 500] : (map_id[en_id] + 7); }

bool missing_player[ROBOTS] = {0};


// PLAYER COMMAND

    // SOCCER VARIABLE
int player_state[ROBOTS];
double player_param_main[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};
double player_param_sub[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};

double old_player_param_main[ROBOTS];
double old_player_param_sub[ROBOTS];

int old_player_state[ROBOTS];

int manual_player = -1;

// SETUP_UI VAR


#endif
