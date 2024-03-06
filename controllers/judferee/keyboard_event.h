#ifndef KEYEVENT_H
#define KEYEVENT_H
#include <webots/keyboard.h>
#include <VARIABLE.h>

#define FOLLOW_ROBOT 0
#define FOLLOW_CENTER 0
#define FOLLOW_VIEWPOINT 1

  // KEYBOARD
int old_key = -1;
WbNodeRef CAMERA_CENTER_NODE;

static void init_keyboard(){
  old_key = 0;

  for (int i = 0; i < 7; i++)
    if (!missing_player[i]){
      manual_player = i;
      break;
    }

  if (FOLLOW_ROBOT) wb_supervisor_field_set_sf_string(wb_supervisor_node_get_field(vpoint, "follow"), robot_webot_name[manual_player]);
  else if (FOLLOW_CENTER) wb_supervisor_field_set_sf_string(wb_supervisor_node_get_field(vpoint, "follow"), "CAMERA_CENTER");

  CAMERA_CENTER_NODE = wb_supervisor_node_get_from_def("CAMERA_CENTER");

  string SELF_INDI = string(robot_name[manual_player]) + ".INDICATOR";
  wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(SELF_INDI.c_str()) , vpoint, 1);
}

static void check_navi(int key, int &modified){
    switch (key) {
      case WB_KEYBOARD_UP:
        // printf("Go forwards\n");
        modified += 1;
        break;
      case WB_KEYBOARD_DOWN:
        // printf("Go backwards\n");
        modified += 2;
        break;
      case WB_KEYBOARD_LEFT:
        // printf("Strafe left\n");
        modified += 4;
        break;
      case WB_KEYBOARD_RIGHT:
        // printf("Strafe right\n");
        modified += 8;
        break;
    }
}

static void check_bend(int key, double& modified){
  switch (key) {
    case 'Q':
      // printf("Rotate left\n");
      modified = 1;
      break;
    case 'E':
      // printf("Rotate right\n");
      modified = 2;
      break;
    case '2':
      // printf("Rotate Goal\n");
      modified = 4;
      break;
    case 'R':
      modified = 16;
      break;
    case 'W':
      // printf("Rotate Ball\n");
      modified = 8;
      break;
  }
}


int space_event = 0; 
bool last_pressed_keyboard = 0;
static void check_switch(bool flag, double& modified, bool& switch_sign){


  // switch(key) {
  //   case ' ':
  //     // printf("space pressed");
  //     // need counter here
  //     break;
  // }
  // cout << space_event << " " <<last_pressed_keyboard << " BEFORE EVENT SPACEING \n";
  if (flag)
  {
    if (last_pressed_keyboard) space_event = 1; // hold
    else space_event = 2; // rising

    last_pressed_keyboard = 1;
  }
  else{
    if (last_pressed_keyboard) space_event = 3; // dropping
    else space_event = 0; // off

    last_pressed_keyboard = 0;
  }

  // cout << space_event << " EVENT SPACEING \n";
  if (player_ball[manual_player] == 0 && space_event == 2) switch_sign = 1;
  
  if (player_ball[manual_player]){
    if (space_event == 1) modified = 1;
    else if (space_event == 2) modified = 2;
    else if (space_event == 3) modified = 3;
  }

}

static void update_camera_center(){
  double new_center[3] = {(ball_position[0] + player_position[manual_player][0])/2 , (ball_position[1] + player_position[manual_player][1])/2, 0};
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(CAMERA_CENTER_NODE, "translation"), new_center);
}

double old_center[3] = {0, 0, 24};
double smoothness = 0.01; // 0.06

bool auto_mode = 0;

static void update_viewpoint(){
  double true_center[3] = {(ball_position[0] + player_position[manual_player][0])/2 , (ball_position[1] + player_position[manual_player][1])/2, 30};
  double new_center[3] = { old_center[0] + (true_center[0] - old_center[0]) * smoothness , old_center[1] + (true_center[1] - old_center[1]) * smoothness, true_center[2]};
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(vpoint, "position"), new_center);

  old_center[0] = new_center[0];
  old_center[1] = new_center[1];
  old_center[2] = new_center[2];
}

static void check_keyboard() {

  if (FOLLOW_CENTER) update_camera_center();
  else if (FOLLOW_VIEWPOINT) update_viewpoint();

  vector <int> manual_control_key;
  chrono_control_key[0] = false;
  chrono_control_key[1] = false; // = pressed key [F, G]

  while (true){
    int key = wb_keyboard_get_key();
    if (key == -1) break;
    else 
    if (key == 'F')
      chrono_control_key[0] = true;
    else if (key == 'G')
      chrono_control_key[1] = true;
    else
      manual_control_key.push_back(key);
      // cout << " pressed " << key << '\n';
  }

  if (!MANUAL_MODE) return;

  int manual_state = 224;
  double manual_main_param = 0000;
  double manual_sub_param  = 0000;

  bool change_player_sign = 0, exist_space = 0;

  for (auto key : manual_control_key)
  {
    check_navi(key, manual_state);
    check_bend(key, manual_main_param);
    if (key == ' ') exist_space = 1;
  }
  check_switch(exist_space, manual_sub_param, change_player_sign);

  if (!change_player_sign){


    if (int(manual_main_param) == 16 || (auto_mode == 1 && manual_main_param == 0 && manual_state == 224) ){
      ;
      auto_mode = 1;
    }
    else {
      player_state[manual_player] = manual_state;
      player_param_main[manual_player] = manual_main_param;
      player_param_sub[manual_player] = manual_sub_param;

      auto_mode = 0;
    }
  }
  else{

    string SELF_INDI = string(robot_name[manual_player]) + ".INDICATOR";
    wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(SELF_INDI.c_str()) , vpoint, 0);


    // no change on the manual_player behavior, change the manual_id now
    // cout << " CHange \n";
    int first_available = -1; bool found = 0;
    for (int i = 0; i < 7; i++)
      if (!missing_player[i]){
        if (first_available == -1) first_available = i;

        if (i > manual_player){
          manual_player = i;
          found = 1;
          break;
        }
      }
    if (!found) manual_player = first_available;

    SELF_INDI = string(robot_name[manual_player]) + ".INDICATOR";
    wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(SELF_INDI.c_str()) , vpoint, 1);

    if (FOLLOW_ROBOT) wb_supervisor_field_set_sf_string(wb_supervisor_node_get_field(vpoint, "follow"), robot_webot_name[manual_player]);
  }
      // case 'P':
      //   // run_demo();
      //   break;
        // printf("Reset\n");
        // base_reset();

      // default:
        // fprintf(stderr, "Wrong keyboard input\n");
        // player_state[manual_player] = 0;
        // break;

  // old_key = key;
}

#endif
