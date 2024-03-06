#include <webots/device.h>
#include <webots/keyboard.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <webots/led.h>
#include <vector> 
#include <cstdlib>  
#include <ctime>


#include "skills.h"
#include "update_graphic.h"

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <unordered_map>

//-------------PHYSICAL-LIMIT-STAT------
#define FRICTION 0.2
//-------------ROBOT-RELATED--------

#define TEAM_A "SPN"
#define TEAM_B "NON"
#define INFO_MODE 0
#define BALL_INFO_MODE 0
#define ALL_DEBUG_MODE 0
#define SENSOR_MODE 0
#define DEBUG_WEBOT_API 0
#define SPEC_DEBUG_MODE (robot_encrypted_id == 503)
#define EMITTER_INFO 0

#define ACTION_DEBUG_MODE 0
#define WIRELESS_DEBUG_MODE 0

using namespace std;  
string ROBOT_NAME;
string ROBOT_PREFIX;
int ROBOT_TEAM;
int ROBOT_ID;
double TIME_STEP; 

//-------------STATE

int current_state = 0, old_state = -1;
double time_elapsed = 0;
bool action_done_return = 0;

//-------------MOTION_PLANNED--------
double velo_x_vec = 0.0, velo_y_vec = 0.0, velo_w_vec = 0.0;
int STUCKED_TIME = 0;
//-----------OBJECT_NAME--------------
WbNodeRef goal_node;
double goal_position_x[3] = {0}, goal_position_y[3] = {0}; 

//-----------SENSOR_NAME----------------
    // motor
WbDeviceTag wheels[4];
    // ir sensor
static WbDeviceTag infrared_sensors[NUMBER_OF_INFRARED_SENSORS];
static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {  "ir_0",  "ir_30",   "ir_60",  "ir_90",  "ir_120", "ir_150", "ir_180", \
                                                                         "ir_210", "ir_240",  "ir_270",  "ir_300",  "ir_330"};
    // radio
WbDeviceTag emitter, receiver;
    // gps
WbDeviceTag gps;
    // bumper
WbDeviceTag bumper_sensor;
    
//----------------------------------------
//-------------SENSOR_CONVERT-------------------

double convert_volt_to_meter(WbDeviceTag tag, double V) {
  const char *model = wb_device_get_model(tag);

  if (strcmp(model, "GP2D120"))
    return 0.1594 * pow(V, -0.8533) - 0.02916;
  else if (strcmp(model, "GP2Y0A02YK0F"))
    return 0.7611 * pow(V, -0.9313) - 0.1252;
  else if (strcmp(model, "GP2Y0A41SK0F"))
    return 0.1594 * pow(V, -0.8533) - 0.02916;
  else if (strcmp(model, "GP2Y0A710K0F"))
    return 20.24 * pow(V, -4.76) + 0.6632;
  else {
    printf("This infrared sensor model is not compatible\n");
    return -1;
  }
}
//--------------------SIMULATION_STEP----------------------------
static void single_step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  const double start_time = wb_robot_get_time();
  do {
    single_step();
    base_accelerate();
  } while (start_time + sec > wb_robot_get_time());
}

//-----------MATH_FUNCTION-------------------------

void new_position(double *robot_dir_x, double *robot_dir_y, double *robot_dir_z, double unit_vector_x, double unit_vector_y, double unit_vector_z)
{
    const double *R_Matrix = wb_supervisor_node_get_orientation(wb_supervisor_node_get_self());

    *robot_dir_x = R_Matrix[0] * unit_vector_x + R_Matrix[1] * unit_vector_y + R_Matrix[2] * unit_vector_z;
    *robot_dir_y = R_Matrix[3] * unit_vector_x + R_Matrix[4] * unit_vector_y + R_Matrix[5] * unit_vector_z;
    *robot_dir_z = R_Matrix[6] * unit_vector_x + R_Matrix[7] * unit_vector_y + R_Matrix[8] * unit_vector_z;
}

//  robots vari
const char *robot_name[ROBOTS] = {"NON_GK", "NON_CB", "NON_LB", "NON_RB", "NON_ST", "NON_LW", "NON_RW", 
                                  "SPN_GK", "SPN_CB", "SPN_LB", "SPN_RB", "SPN_ST", "SPN_LW", "SPN_RW"};

int command[ROBOTS] = {0};
double param_main[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};
double param_sub[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};

// -----------TINO3------

// static void run_demo() {
//   printf("Demonstration started\n");

//   // Make a square movement
//   base_forwards();
//   passive_wait(2.0);
//   base_reset();
//   passive_wait(1.0);

//   base_strafe_left();
//   passive_wait(2.0);
//   base_reset();
//   passive_wait(1.0);

//   base_backwards();
//   passive_wait(2.0);
//   base_reset();
//   passive_wait(1.0);

//   printf("Demonstration finished\n");
// }

// static void display_instructions() {
//   printf("Control commands:\n");
//   printf(" - WAXD QEZC:       Move the robot\n");

//   printf(" - J/K: Rotate the robot\n");
//   printf(" - S/Space: Stop/Reset\n");
//   printf("\n");
//   // printf("Demonstration mode: press 'D'\n");
//   // printf("Autopilot mode : press 'A'\n");
// }

//----------------INIT_FUNCTION
void init_robot()
{



  string SELF_DEF_NAME =  wb_supervisor_node_get_def(wb_supervisor_node_get_self());
  // cout << SELF_DEF_NAME << " INIT" << '\n';

  ROBOT_NAME = SELF_DEF_NAME.substr(0, 4);

  static std::unordered_map<std::string,std::string> const squad_numb = { {"GK","001"}, {"CB","002"}, {"LB", "005"}, {"RB", "004"}, {"ST", "009"},{"LW", "010"}, {"RW", "011"} };
  auto it = squad_numb.find(SELF_DEF_NAME.substr(4, 2));
  if (it != squad_numb.end()) {
    ROBOT_NAME = ROBOT_NAME + it->second;
  } else { throw string("unknown squad number"); };

  // cout << ROBOT_NAME << '\n';

  // const char* TEMP_ROBOT_NAME = wb_robot_get_name();
  // ROBOT_NAME = TEMP_ROBOT_NAME;

  // char team_name[3];  
  // memcpy( team_name, ROBOT_NAME.c_str(), 3 ); // first 3 character  
  // team_name[3] = '\0';
  if (strcmp(ROBOT_NAME.substr(0, 3).c_str(),TEAM_A) == 0) 
  {
    ROBOT_TEAM = 0;
    goal_node = wb_supervisor_node_get_from_def("BLUE_GOAL");
  }
  else if (strcmp(ROBOT_NAME.substr(0, 3).c_str(),TEAM_B) == 0) 
  {
    ROBOT_TEAM = 500;
    goal_node = wb_supervisor_node_get_from_def("RED_GOAL");
  }
  else ROBOT_TEAM = -1;
  ROBOT_ID = (ROBOT_NAME[4]-'0')*100 + (ROBOT_NAME[5] - '0')*10 + (ROBOT_NAME[6] - '0');

  robot_encrypted_id = ROBOT_TEAM + ROBOT_ID;
  // cout << "EID " << robot_encrypted_id << '\n';

  // if (INFO_MODE) printf("WEBOTS ROBOT ID %d %d %s \n", ROBOT_TEAM, ROBOT_ID, ROBOT_NAME);
  srand(ROBOT_ID + ROBOT_TEAM);
  const int temp = wb_robot_get_basic_time_step();
  TIME_STEP = temp;
}

void init_player_ball_position()
{
  NAN_DEF = wb_supervisor_node_get_from_def("NAN");


  ball_node = wb_supervisor_node_get_from_def("BALL");
  wb_supervisor_node_enable_pose_tracking(ball_node, TIME_STEP, NULL);

  for (int i = 0; i < ROBOTS; i++) {
    player_def[i] = wb_supervisor_node_get_from_def(robot_name[i]);
    // if (isnan(player_def[i])) missing_player[i] = 1;
    if (player_def[i] == NAN_DEF) missing_player[i] = 1;

    else wb_supervisor_node_enable_pose_tracking(player_def[i], TIME_STEP, NULL);
  }
  const double *tempPost = wb_supervisor_node_get_position(goal_node);

  goal_position_x[0] = tempPost[0]; goal_position_y[0] = tempPost[1];
  goal_position_x[1] = tempPost[0]; goal_position_y[1] = tempPost[1] - 0.6;
  goal_position_x[2] = tempPost[0]; goal_position_y[2] = tempPost[1] + 0.6;

  set_up_goal(goal_position_x[0], goal_position_y[0]);
  // if (ALL_DEBUG_MODE)
  //   printf("nani %f %f %f %f \n", goal_position_y[1], goal_position_y[2], goal_position_y[0], goal_position_x[0]);
}


void init_sensor_actuator()
{
  // get devices

 // Get the motors and set target position to infinity (speed control)
  char wheel_name[16];
  for (int i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d_joint", i);
    int j =(i+1)%4; 
    wheels[j] = wb_robot_get_device(wheel_name);
    wb_motor_set_position(wheels[j], INFINITY);
    wb_motor_set_velocity(wheels[j], 0.0);
  }
  link_wheel(wheels[0], wheels[1], wheels[2], wheels[3]);


  springer = wb_robot_get_device("springer");
  wb_motor_set_velocity(springer, 15);

  magnetic_sensor = wb_robot_get_device("connector");
  wb_connector_enable_presence(magnetic_sensor, TIME_STEP);

  magnetic_node = wb_supervisor_node_get_from_device(magnetic_sensor);
  WbFieldRef tens_field = wb_supervisor_node_get_field(magnetic_node, "tensileStrength");
  WbFieldRef sher_field = wb_supervisor_node_get_field(magnetic_node, "shearStrength");
  WbFieldRef tol_field = wb_supervisor_node_get_field(magnetic_node, "distanceTolerance");


  wb_supervisor_field_set_sf_float(tens_field, 325);
  wb_supervisor_field_set_sf_float(sher_field, 325);
  wb_supervisor_field_set_sf_float(tol_field, 0.05);
  double connector_pose[3] = {0, 0.24, 0.06};
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(magnetic_node, "translation"), connector_pose);

  ROBOT_PREFIX = string(robot_name[robot_decrypt(robot_encrypted_id)]);
  string robot_bumper = ROBOT_PREFIX + ".NAIYOU.Springer.BUMPER";
  WbNodeRef bumper= wb_supervisor_node_get_from_def(robot_bumper.c_str());
  WbFieldRef mat_field = wb_supervisor_node_get_field(bumper, "contactMaterial");
  wb_supervisor_field_set_sf_string(mat_field, "rubber");


  emitter = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);


  // Get and enable the infrared sensors
  for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i) {
    infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
    wb_distance_sensor_enable(infrared_sensors[i], TIME_STEP);
  }
  // Enable the keyboard inputs
  // old_key = 0;
  // wb_keyboard_enable(TIME_STEP);

  //wb_touch_sensor_enable(bumper_sensor, TIME_STEP);
}


//---------------INFO_UPDATE_CYCLE

void update_sensor_value()
{
    //bumper_value = wb_touch_sensor_get_value(bumper_sensor);
    // NOTE reverse value
    // for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i)
    //   sensor_values[(i+6)%NUMBER_OF_INFRARED_SENSORS] = convert_volt_to_meter(infrared_sensors[0], wb_distance_sensor_get_value(infrared_sensors[i]));

    // no obstacle 0.41
    // real close 0.000

    const double *gps_value_temp = wb_gps_get_values(gps);
    gps_value[0] = gps_value_temp[0];
    gps_value[1] = gps_value_temp[1];
    gps_value[2] = gps_value_temp[2];


    if (INFO_MODE) printf("gps %f %f \n", gps_value[0], gps_value[1]);

    if (SENSOR_MODE) {
      for (int i = 7; i < NUMBER_OF_INFRARED_SENSORS; i++)
        printf("sensor %d %f | ", i, sensor_values[i]);
      for (int i = 0; i < 7; i++)
        printf("sensor %d %f | ", i, sensor_values[i]);
      printf("\n");
    }
    ball_possesion = wb_connector_get_presence(magnetic_sensor);
    // cout << "loop " << ball_possesion << '\n';

    // if (ball_possesion == 1)
    //   wb_connector_lock(magnetic_sensor);
}

// ROBOT VELO MAX 3.78

void get_player_ball_position()
{
  new_position(&robot_dir_x, &robot_dir_y, &robot_dir_z, 0, 1, 0);
  robot_angle_velo = fabs(current_robot_dir - atan2(robot_dir_y, robot_dir_x)); 
  current_robot_dir = atan2(robot_dir_y, robot_dir_x)-degToRad(90);

  if (INFO_MODE) printf("robot dir %f %f %f \n", robot_dir_x, robot_dir_y, robot_dir_z);
  if (INFO_MODE) printf("robot dir in deg %f \n", current_robot_dir); 

  const double *tempBall = wb_supervisor_node_get_position(ball_node);
  if (BALL_INFO_MODE) printf("ball position %f %f %f \n", tempBall[0], tempBall[1], tempBall[2]);
  for (int j = 0; j < 3; j++) old_ball_position[j] = ball_position[j], ball_position[j] = tempBall[j];

  ball_moving_direction[0] = ball_position[0] - old_ball_position[0];
  ball_moving_direction[1] = ball_position[1] - old_ball_position[1];
  // if (SPEC_DEBUG_MODE) printf("ball direction %f  \n", atan2(ball_moving_direction[1], ball_moving_direction[0]));

  tempBall = wb_supervisor_node_get_velocity(ball_node);
  if (BALL_INFO_MODE) printf("ball velocity %f x %f y %f z \n", tempBall[0], tempBall[1], tempBall[2]);
  ball_velo = fabs(sqrt(tempBall[0] * tempBall[0] + tempBall[1] * tempBall[1]));


  tempBall = wb_supervisor_node_get_velocity(player_def[ robot_decrypt(robot_encrypted_id) ]);
  // double robot_velo = fabs(sqrt(tempBall[0] * tempBall[0] + tempBall[1] * tempBall[1]));
  // cout << " ROBOT VELO " << robot_velo << '\n';

  for (int i = 0; i < ROBOTS; i++) {
    if (missing_player[i]) continue;
    tempBall = wb_supervisor_node_get_position(player_def[i]);
    // transmit value
    for (int j = 0; j < 3; j++)
      player_position[i][j] = tempBall[j];
    if (BALL_INFO_MODE) printf("PLAYER %d is at %f %f %f\n", i, player_position[i][0], player_position[i][1], player_position[i][2]);
    // robot_rotation_field[i] = wb_supervisor_node_get_field(node, "rotation");
    // robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
  }
}


// static void check_keyboard() {
//   int key = wb_keyboard_get_key();
//   if ((key >= 0) && key != old_key) {
//     switch (key) {
//       case 'W':
//         printf("Go forwards\n");
//         base_forwards();
//         break;
//       case 'X':
//         printf("Go backwards\n");
//         base_backwards();
//         break;
//       case 'A':
//         printf("Strafe left\n");
//         base_strafe_left();
//         break;
//       case 'D':
//         printf("Strafe right\n");
//         base_strafe_right();
//         break;
//       case 'Q':
//         printf("Diag Hi Left\n");
//         base_set_speeds(DEMO_SPEED / 2.0,  DEMO_SPEED / 2.0, 0);
//         break;
//       case 'E':
//         printf("Diag Hi Right\n");
//         base_set_speeds(DEMO_SPEED / 2.0,  -DEMO_SPEED / 2.0, 0);
//         break;
//       case 'Z':
//         printf("Diag Low Left\n");
//         base_set_speeds(-DEMO_SPEED / 2.0,  DEMO_SPEED / 2.0, 0);
//         break;
//       case 'C':
//         printf("Diag Low Right\n");
//         base_set_speeds(-DEMO_SPEED / 2.0,  -DEMO_SPEED / 2.0, 0);
//         break;

//       case 'I':
//         printf("Go forward and rotate\n");
//         base_set_speeds(DEMO_SPEED / 2.0,  0,3);
//         break;
//       case 'K':
//         printf("Go backwards and rotate\n");
//         base_set_speeds(-DEMO_SPEED / 2.0,  0,3);
//         break;
//       case 'J':
//         printf("Strafe Lef and rotate\n");
//         base_set_speeds(0, DEMO_SPEED / 2.0 , 3);
//         break;
//       case 'L':
//         printf("Strafe Right and rotate\n");
//         base_set_speeds(0, -DEMO_SPEED / 2.0, 3);
//         break;
//       case 'O':
//         printf("Diag Hi 45 and rotate\n");
//         base_set_speeds(DEMO_SPEED / 2.0, DEMO_SPEED / 2.0, 3);
//         break;                      

//       case 'M':
//         printf("Move to Ball\n");
//         command[ robot_decrypt(robot_encrypted_id) ] = 8;
//         break;


//       case '1':
//         printf("Turn left\n");
//         base_turn_left();
//         break;
//       case '3':
//         printf("Turn right\n");
//         base_turn_right();
//         break;
//       case 'P':
//         run_demo();
//         break;
//       case WB_KEYBOARD_END:
//       case 'S':
//       case ' ':
//         command[ robot_decrypt(robot_encrypted_id) ] = -1;
//         printf("Reset\n");
//         base_reset();
//         break;
//       // case 'D':
//       //   if (key != old_key)  // perform this action just once
//       //     demo = !demo;
//       //   break;
//       // case 'A':
//       //   if (key != old_key)  // perform this action just once
//       //   break;
//       default:
//         fprintf(stderr, "Wrong keyboard input\n");
//         break;
//     }
//   }
//   // if (autopilot != old_autopilot) {
//   //   old_autopilot = autopilot;
//   //   if (autopilot)
//   //     printf("Auto control\n");
//   //   else
//   //     printf("Manual control\n");
//   // }
//   old_key = key;
// }


void emitter_publish()
{
    int data_send[3] = {robot_encrypted_id, current_state, ball_possesion};

    if ((current_state != old_state) || (ball_possesion != old_ball_possesion))
    { 
      // cout << "    send loop " << ball_possesion << " of " << robot_encrypted_id << '\n';

      if (EMITTER_INFO && SPEC_DEBUG_MODE) printf("Published once with id %d and %d and %d \n", data_send[0], data_send[1], data_send[2]);
      wb_emitter_send(emitter, data_send, sizeof(int) * 3);
    }

    old_state = current_state;
    old_ball_possesion = ball_possesion;
}

void update_wireless_receiver()
{
  while (wb_receiver_get_queue_length(receiver) > 0) {
      const float *message = (float*)wb_receiver_get_data(receiver);
      const int data_length = wb_receiver_get_data_size(receiver) /sizeof(float);

      if (WIRELESS_DEBUG_MODE) {    
        printf("PLAYER COMMAND %d %d %.2f \n", data_length, wb_receiver_get_data_size(receiver), *(message));
        for (int i = 0; i < data_length/4; i++)
          printf("robot_id %.2f command %.2f param_main %.2f param_sub %.2f", *(message + i * 4), *(message + i * 4 + 1), *(message + i * 4 + 2),  *(message + i * 4 + 3));
        printf("\n");
      }    
      for (int i = 0; i < data_length/4; i++)
        command[int(*(message + i * 4))] = int(*(message + i * 4 + 1)),
        param_main[int(*(message + i * 4))] = *(message + i * 4 + 2),
        param_sub[int(*(message + i * 4))] = *(message + i * 4 + 3);
      wb_receiver_next_packet(receiver);
   }
   current_state = command[ robot_decrypt(robot_encrypted_id) ];
   // if (SPEC_DEBUG_MODE) printf("GET CURRENT STATE %d and id %d\n", current_state,  robot_decrypt(robot_encrypted_id) );
}


static void behavior_control(){

    BALL_OBS_ON = 0;
    if (current_state >= 1000){
      current_state -= 1000;
      BALL_OBS_ON = 1;
    }
    int this_id = robot_decrypt(robot_encrypted_id);

    // if (ROBOT_TEAM == 500) wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(INDI.c_str()) , vpoint, 0);
    switch(current_state) {
      case 0:
        idle();
        break;
      case 1:
        pass_ball(int(param_main[this_id]));
        break;
      case 2:
        goal_shoot(int(param_main[this_id]), int(param_sub[this_id]));
        break;
      case 8:
        move_to_ball();
        break;
      case 88:
        get_ball();
        break;
      case 68:
        chase_ball(param_main[this_id], param_sub[this_id]);
        break;
      case 69:
        cut_ball(param_main[this_id], param_sub[this_id]);
        break;
      // case 28:
      // {
      //   int enemy_holder = int(param_main[this_id]);
      //   cout << "who hold now " << enemy_holder << '\n';
      //   assert(this_id != enemy_holder);
      //   foo(Dot(player_position[enemy_holder][0], player_position[enemy_holder][1]));
      //   break;
      // }
      case 9:
        field_shoot(param_main[this_id], param_sub[this_id]);
        break;
      case 18:
        component_vector[0] = rotate_to_object(ball_node);
        break;
      case 4: // 
      case 14: // atak
      case 24: // def
        capture(param_main[this_id], param_sub[this_id]);
        break;
      case 44: // Interupt
        free_capture(param_main[this_id], param_sub[this_id]);
        break;
      case 3: // off-ball
      case 33: // block
      case 13: // guard
      case 63: // fence
        pressing(param_main[this_id], param_sub[this_id]);
        break;
      case 5:
        tackle(int(param_main[this_id]));
        break;
      case 6:
        dribble(int(param_main[this_id]));
        break;
      case 16:
        x_dribble(param_main[this_id], param_sub[this_id]);
        break;
      case 10:
        force_shoot();
        break;
      case 111:
        force_release();
        break;
      case 112:
        soft_release();
        break;


      case 224:
      case 225:
      case 226:
      case 227:
      case 228:
      case 229:
      case 230:
      case 231:
      case 232:
      case 233:
      case 234:
      case 235:
      case 236:
      case 237:
      case 238:
      case 239:
        // wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(INDI.c_str()) , vpoint, 1);
        
        manual_control(current_state, int(param_main[this_id]), int(param_sub[this_id]));
        // base_move(param_main[this_id]);

        break;

      default:
        cout << "unknown command " << current_state << '\n';
    }
}

int self_timer_autounlock = 0;
static void congest_rule(){
  if (self_timer_autounlock == 0){
    if (current_state == 112 || current_state == 113) {
      self_timer_autounlock = 5;
      wb_supervisor_field_set_sf_bool(wb_supervisor_node_get_field(magnetic_node, "autoLock"), 0);
    }

  }
  else {
    self_timer_autounlock --;

    if (self_timer_autounlock == 0)
    {
      wb_supervisor_field_set_sf_bool(wb_supervisor_node_get_field(magnetic_node, "autoLock"), 1);
    }
  }
}

int main(int argc, char **argv) {
  // Initialization
  wb_robot_init();

  init_robot();
  init_player_ball_position();
  init_sensor_actuator();

  // Display instructions to control the robot
  // display_instructions();
  passive_wait(0.1);

  update_self_marker(ROBOT_PREFIX, ROBOT_ID, ROBOT_TEAM);

  // Main loop
  //  o Autopilot mode if nothing else done
  //  o Demonstration mode
  //  o Manual control with keyboard arrows

  // command[ robot_decrypt(robot_encrypted_id) ] = 2;

  // double ROBOT_MIN_Y = 10000;
  // bool STABLE_YET = 0, PEAK_YET = 0;

  single_step();

  while (wb_robot_step(TIME_STEP) != -1) {
    ++time_step_counter;
    // cout << "STATE " << current_state << " TIME " << time_step_counter << " BALL " << ball_possesion << '\n';
    // if (ball_velo*3 < 3.78) cout << "distance " << ball_position[0] << '\n'; else 
    // cout << "OK BALL VELO " << ball_velo << '\n';
    // if (gps_value[1] > ROBOT_MIN_Y && !PEAK_YET) cout << "Peak " << ROBOT_MIN_Y << " at T_PEAK " << "\n", PEAK_YET = 1;
    // else ROBOT_MIN_Y = gps_value[1];
    // if (fabs(gps_value[0] - ball_position[0])/fabs(ball_position[0]) <= 0.6 && PEAK_YET)  cout << " STABLE " << '\n',STABLE_YET = 1;
    // if (time_step_counter > 40) command[ robot_decrypt(robot_encrypted_id) ] = 0;

    update_sensor_value();
    update_wireless_receiver();

    get_player_ball_position();

    check_counter_springer();

    // check_keyboard();
    // component_vector[2].assign_xy( transform_vector(POD(sensor_values, &STUCKED_TIME), current_robot_dir) );
    behavior_control();

    congest_rule();

    // component_vector[0].vx = -2;
    // component_vector[0].vy = 0;
    // component_vector[0].vw = 0;

    emitter_publish();

    // config_dynamic_pod();

    // cout << "               ROBOT " << robot_encrypted_id << " has velo \n";
    finalize_speed();
    base_accelerate();
    // single_step();
  }

  wb_robot_cleanup();
  return 0;
}
