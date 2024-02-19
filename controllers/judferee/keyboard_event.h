#ifndef KEYEVENT_H
#define KEYEVENT_H
#include <webots/keyboard.h>
#include <VARIABLE.h>

  // KEYBOARD
int old_key = -1;


static void init_keyboard(){
  old_key = 0;
  wb_keyboard_enable(TIME_STEP);
  for (int i = 0; i < 7; i++)
    if (!missing_player[i]){
      manual_player = i;
      break;
    }

  wb_supervisor_field_set_sf_string(wb_supervisor_node_get_field(vpoint, "follow"), robot_webot_name[manual_player]);

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
    case 'R':
      // printf("Rotate Goal\n");
      modified = 4;
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


static void check_keyboard() {
  vector <int> pressed_key;
  while (true){
    int key = wb_keyboard_get_key();
    if (key == -1) break;
    else 
      pressed_key.push_back(key);
      // cout << " pressed " << key << '\n';
  }

  if (pressed_key.size() == 0){
    player_state[manual_player] = 0;
    last_pressed_keyboard = 0;
    return;
  }

  int manual_state = 224;
  double manual_main_param = 0000;
  double manual_sub_param  = 0000;

  bool change_player_sign = 0, exist_space = 0;

  for (auto key : pressed_key)
  {
    check_navi(key, manual_state);
    check_bend(key, manual_main_param);
    if (key == ' ') exist_space = 1;
  }
  check_switch(exist_space, manual_sub_param, change_player_sign);

  if (!change_player_sign){
    player_state[manual_player] = manual_state;
    player_param_main[manual_player] = manual_main_param;
    player_param_sub[manual_player] = manual_sub_param;
  }
  else{
    // no change on the manual_player behavior, change the manual_id now
    cout << " CHange \n";
    for (int i = 0; i < 7; i++)
      if (!missing_player[i] && i != manual_player){
        manual_player = i;
        break;
      }
    wb_supervisor_field_set_sf_string(wb_supervisor_node_get_field(vpoint, "follow"), robot_webot_name[manual_player]);
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
