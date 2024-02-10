#ifndef KEYEVENT_H
#define KEYEVENT_H
#include <webots/keyboard.h>
#include <VARIABLE.h>

  // KEYBOARD
int old_key = -1;

int manual_player = -1;

static void init_keyboard(){
  old_key = 0;
  wb_keyboard_enable(TIME_STEP);
  for (int i = 0; i < 7; i++)
    if (!missing_player[i]){
      manual_player = i;
      break;
    }

  cout << " WOW " << manual_player << '\n';
}

static void check_keyboard() {
  int key = wb_keyboard_get_key();
  if ((key >= 0) && key != old_key) {
    switch (key) {
      case 'W':
        printf("Go forwards\n");
        player_state[manual_player] = 200;
        player_param_main[manual_player] = 0001;
        // base_forwards();
        break;
      case 'X':
        printf("Go backwards\n");
        // base_backwards();
        break;
      case 'A':
        printf("Strafe left\n");
        // base_strafe_left();
        break;
      case 'D':
        printf("Strafe right\n");
        // base_strafe_right();
        break;
      case 'Q':
        printf("Diag Hi Left\n");
        // base_set_speeds(DEMO_SPEED / 2.0,  DEMO_SPEED / 2.0, 0);
        break;
      case 'E':
        printf("Diag Hi Right\n");
        // base_set_speeds(DEMO_SPEED / 2.0,  -DEMO_SPEED / 2.0, 0);
        break;
      case 'Z':
        printf("Diag Low Left\n");
        // base_set_speeds(-DEMO_SPEED / 2.0,  DEMO_SPEED / 2.0, 0);
        break;
      case 'C':
        printf("Diag Low Right\n");
        // base_set_speeds(-DEMO_SPEED / 2.0,  -DEMO_SPEED / 2.0, 0);
        break;

      case 'I':
        printf("Go forward and rotate\n");
        // base_set_speeds(DEMO_SPEED / 2.0,  0,3);
        break;
      case 'K':
        printf("Go backwards and rotate\n");
        // base_set_speeds(-DEMO_SPEED / 2.0,  0,3);
        break;
      case 'J':
        printf("Strafe Lef and rotate\n");
        // base_set_speeds(0, DEMO_SPEED / 2.0 , 3);
        break;
      case 'L':
        printf("Strafe Right and rotate\n");
        // base_set_speeds(0, -DEMO_SPEED / 2.0, 3);
        break;
      case 'O':
        printf("Diag Hi 45 and rotate\n");
        // base_set_speeds(DEMO_SPEED / 2.0, DEMO_SPEED / 2.0, 3);
        break;                      

      case 'M':
        printf("Move to Ball\n");
        // command[ robot_decrypt(robot_encrypted_id) ] = 8;
        break;


      case '1':
        printf("Turn left\n");
        // base_turn_left();
        break;
      case '3':
        printf("Turn right\n");
        // base_turn_right();
        break;
      case 'P':
        // run_demo();
        break;
      case WB_KEYBOARD_END:
      case 'S':
      case ' ':
        // command[ robot_decrypt(robot_encrypted_id) ] = -1;
        printf("Reset\n");
        // base_reset();
        break;
      // case 'D':
      //   if (key != old_key)  // perform this action just once
      //     demo = !demo;
      //   break;
      // case 'A':
      //   if (key != old_key)  // perform this action just once
      //   break;
      default:
        fprintf(stderr, "Wrong keyboard input\n");
        player_state[manual_player] = 0;
        break;
    }
  }
  else if (key == -1)
  {
    player_state[manual_player] = 0;
  }
  // if (autopilot != old_autopilot) {
  //   old_autopilot = autopilot;
  //   if (autopilot)
  //     printf("Auto control\n");
  //   else
  //     printf("Manual control\n");
  // }
  old_key = key;
}

#endif
