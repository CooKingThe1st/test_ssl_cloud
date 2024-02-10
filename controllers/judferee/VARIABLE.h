#ifndef VAR_H
#define VAR_H

// SYSTEM VAR
#define ROBOTS (7*2)  // number of robots
int CURRENT_BRAIN_LEVEL = 0;// 0-> random, 1->Naive, 3->mixed
double TIME_STEP;


// PLAYER VAR


const char *robot_name[ROBOTS] = {"NON_GK", "NON_CB", "NON_LB", "NON_RB", "NON_ST", "NON_LW", "NON_RW",
                                  "SPN_GK", "SPN_CB", "SPN_LB", "SPN_RB", "SPN_ST", "SPN_LW", "SPN_RW"};
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
int old_player_state[ROBOTS];


// SETUP_UI VAR


#endif
