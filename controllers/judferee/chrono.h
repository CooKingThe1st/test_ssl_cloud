#ifndef CHRONO_H
#define CHRONO_H
#include <deque>          

#include "VARIABLE.h"
// #include <file_manipulate.h>

using namespace std;
// void back_logging(){
// 	clear_file("back_log");
// 	for ??
// 		log_to_file(???)
// }

string preset_name[6] = {"0", "1", "2", "3", "4", "5"};
bool name_in_used[6] = {0, 0, 0, 0, 0, 0};
deque<string> saved_state;

double ball_saveVelo[6][6] = {};
double robot_saveVelo[6][ROBOTS][6] = {};
double save_player_param_main[6][ROBOTS];
double save_player_param_sub[6][ROBOTS];
int save_player_state[6][ROBOTS];

void init_chrono(){
	for (int i = 0; i < 6; i++)
		saved_state.push_back("__init__");
}

void world_state_save(string state){

	int id_state = state[0] - '0';
	// save robot state
	for (int i = 0; i < ROBOTS; i++){
		if (missing_player[i]) continue;
		wb_supervisor_node_save_state(player_def[i], state.c_str());
		const double *tempVelo = wb_supervisor_node_get_velocity(player_def[i]);
		for (int j = 0; j < 6; j++) robot_saveVelo[id_state][i][j] = tempVelo[j];
		save_player_state[id_state][i] = player_state[i];
		save_player_param_main[id_state][i] = player_param_main[i];
		save_player_param_sub[id_state][i] = player_param_sub[i];
	}

	// save ball state
		// log_to_file("back_log", ???);
	wb_supervisor_node_save_state(ball_node, state.c_str());

	const double *tempVelo = wb_supervisor_node_get_velocity(ball_node);
	for (int i = 0; i < 6; i++) ball_saveVelo[id_state][i] = tempVelo[i];
}

void world_state_load(string state){
	int id_state = state[0] - '0';

	for (int i = 0; i < ROBOTS; i++)
		if (!missing_player[i]) {
			wb_supervisor_node_load_state(player_def[i], state.c_str()),
		
			wb_supervisor_node_restart_controller(player_def[i]);
		
			wb_supervisor_node_set_velocity(player_def[i], robot_saveVelo[state[0] - '0'][i]);

			player_state[i] = save_player_state[id_state][i];
			player_param_main[i] = save_player_param_main[id_state][i];
			player_param_sub[i] = save_player_param_sub[id_state][i];
		}

	// load ball state 
	wb_supervisor_node_load_state(ball_node, state.c_str());
	wb_supervisor_node_set_velocity(ball_node, ball_saveVelo[id_state]);
}

void chrono_control(){
	if (chrono_control_key[0] == 1){
		WbSimulationMode x = wb_supervisor_simulation_get_mode();
		cout << x << '\n';
		if (x == WB_SUPERVISOR_SIMULATION_MODE_PAUSE)
			wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME);
		else if (x == WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME)
			wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
		// wb_supervisor_simulation_set_mode();
	}


	if (time_step_counter % 100 == 0) { // first log

		string next_state = "";

		string pop_state = saved_state.front();
		saved_state.pop_front();
		if (pop_state.length() == 1)
			// name_in_used[pop_state[0] - '0'] = false,
			next_state = pop_state;
		else 
			for (int i = 0; i < 6; i++)
				if (!name_in_used[i]){
					next_state = preset_name[i];
					name_in_used[i] = 1;
					break;
				}
		cout << next_state << '\n';
		saved_state.push_back(next_state);

		world_state_save(next_state);
	}

	if (chrono_control_key[1] == 1){

		world_state_load(preset_name[1]);
	}

}


#endif
