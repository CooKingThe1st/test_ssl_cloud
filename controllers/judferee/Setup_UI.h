#ifndef UI_H
#define UI_H
using namespace std;

#include <string.h>
#include <cassert>  
#define UI_ON 1
#define LAYOUT_STAT 0
#define LAYOUT_BVIOR 1
#define LOG_MODE 0

#include <stdio.h>
#include <iomanip>
int CURRENT_BRAIN_LEVEL = 0;// 0-> random, 1->Naive, 3->mixed


#define ROBOTS (7*2)  // number of robots

//  player vari
const char *robot_name[ROBOTS] = {"NON_GK", "NON_CB", "NON_LB", "NON_RB", "NON_ST", "NON_LW", "NON_RW",
                                  "SPN_GK", "SPN_CB", "SPN_LB", "SPN_RB", "SPN_ST", "SPN_LW", "SPN_RW"};
int map_id[12] = {-1, 0, 1, -1, 3, 2, -1, -1, -1, 4, 5, 6};
                //   gk  cb     rb lb             st lw rw
inline int robot_decrypt(int en_id) { return (en_id >= 500) ? map_id[en_id - 500] : (map_id[en_id] + 7); }

bool missing_player[ROBOTS] = {0};

std::string get_pre_set(bool X = 0){
  string pre_set;
  int num_robot = 0;
  for (int i = 0; i < ROBOTS; i++)
    if (!missing_player[i]) num_robot++;

  // if (num_robot == 4) pre_set = "A_" ;
  // else if (num_robot == 10) pre_set = "B_";
  // else if (num_robot == ROBOTS) pre_set = "C_";
  // else assert(false);
  pre_set =  std::to_string(num_robot) + "_";

  if (CURRENT_BRAIN_LEVEL == 1)  pre_set = pre_set + "NAIVE_";
  else if (CURRENT_BRAIN_LEVEL == 2)  pre_set = pre_set + "INTER_";
  else if (CURRENT_BRAIN_LEVEL == 3)  pre_set = pre_set + "MIXED_";
  else assert(false);
  if (X) return pre_set + "SET";
  return pre_set;
}

//----------------------OUTPUT

std::string get_string_id(){
  int num_red = 0, num_blue = 0;
  for (int i = 0; i < 7; i++)
    if (!missing_player[i]) num_blue++;
  for (int i = 7; i < ROBOTS; i++)
    if (!missing_player[i]) num_red++;
  string mid_fix = std::to_string(num_red)+"R&"+std::to_string(num_blue)+"B";
  return "_"+mid_fix;
}

void STAT_UI(){
  if (!UI_ON) return;

  if (LAYOUT_STAT) 
  {wb_supervisor_set_label(20, "Score", 0.05, 0.01, 0.06, 0xe5ed9f, 0.0, "Arial");
   wb_supervisor_set_label(21, "Posse", 0.05, 0.1, 0.06, 0xe5ed9f, 0.0, "Arial");
   wb_supervisor_set_label(22, "P/Attem", 0.05, 0.2, 0.06, 0xe5ed9f, 0.0, "Arial");
   wb_supervisor_set_label(23, "P/Succ", 0.05, 0.3, 0.06, 0xe5ed9f, 0.0, "Arial");
   wb_supervisor_set_label(24, "S/Attem", 0.05, 0.4, 0.06, 0xe5ed9f, 0.0, "Arial");
  }
  else if (LAYOUT_BVIOR){
    wb_supervisor_set_label(57, get_pre_set(1).c_str(), 0.02, 0.38, 0.08, 0x6DFF33, 0.0, "Arial");
    wb_supervisor_set_label(21, "Posse", 0.039, 0.42, 0.08, 0x6DFF33, 0.0, "Arial");
    wb_supervisor_set_label(22, "P/Attem", 0.039, 0.45, 0.08, 0x6DFF33, 0.0, "Arial");
    wb_supervisor_set_label(23, "P/Succ", 0.039, 0.48, 0.08, 0x6DFF33, 0.0, "Arial");
    wb_supervisor_set_label(24, "S/Attem", 0.039, 0.51, 0.08, 0x6DFF33, 0.0, "Arial");
  }
}

std::string get_file_timestamp(int mode = 0)
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  if (mode == 0) strftime(buffer,sizeof(buffer),"_%d-%m-%Y-%H-%M-%S",timeinfo);
  else if (mode==1) strftime(buffer,sizeof(buffer),"_%d-%m-%Y",timeinfo);
  else assert(false);
  return std::string(buffer);
}

ofstream MyFile;

void write_to_file(string record_file_name, string data)
{
    if (!LOG_MODE) return;
    string suffix = get_file_timestamp(1);
    string midfix = get_string_id();
    string prefix = get_pre_set();
    MyFile.open(prefix+record_file_name+midfix+suffix+".txt", ios::app);
    MyFile << data;
    // for (i = 0; i < ROBOTS; i++) {
    //   if (missing_player[i]) continue;
    //   MyFile << robot_name[i] << " pos ";
    //   for (j = 0; j < 3; j++)
    //     MyFile <<  player_position[i][j] << " ";

    //   MyFile << " rot ";
    //   for (j = 0; j < 4; j++)
    //     MyFile <<  player_rotation[i][j] << " ";

    //   MyFile << '\n';
    // }
    MyFile.close();
}
//------------------------- support function -------------------------

int interupt_color = 0x6DFF33;

char time_string[64];
static void show_current_time(double current_time){
    if (!UI_ON)  return;

    sprintf(time_string, "%02d:%02d", (int)(current_time / 60), (int)current_time % 60);
    if (LAYOUT_STAT) wb_supervisor_set_label(86, time_string, 0.45, 0.01, 0.08, interupt_color, 0.0, "Arial");  // black
    else if (LAYOUT_BVIOR) wb_supervisor_set_label(86, time_string, 0.032, 0.01, 0.08, interupt_color, 0.0, "Arial");  // black
}

static void set_single(double X, double pos_x, double pos_y, int ID){
  if (!UI_ON) return;
  char X_string[16];
  sprintf(X_string, "%02f", X);
  if (LAYOUT_BVIOR) wb_supervisor_set_label(ID, X_string, pos_x, pos_y, 0.08, 0xff0000, 0.0, "Arial");

}

static void set_scores(int a, int b, double pos, int lid, int rid) {
  if (!UI_ON) return;
  char c_score[16];
  // convert int to c_score char
  sprintf(c_score, "%d", a);
  if (LAYOUT_STAT) wb_supervisor_set_label(lid, c_score, 0.92, pos, 0.08, 0xff0000, 0.0, "Arial");  // spn c_score
  else if (LAYOUT_BVIOR) wb_supervisor_set_label(lid, c_score, 0.01, pos, 0.08, 0xff0000, 0.0, "Arial");  // spn c_score

  sprintf(c_score, "%d", b);
  if (LAYOUT_STAT) wb_supervisor_set_label(rid, c_score, 0.22, pos, 0.08, 0x0000ff, 0.0, "Arial");  // non score
  else if (LAYOUT_BVIOR) wb_supervisor_set_label(rid, c_score, 0.1, pos, 0.08, 0x0000ff, 0.0, "Arial");  // non score
}

static void set_percent(double a, double b, double pos, int lid, int rid) {
  if (!UI_ON) return;
  if (LAYOUT_STAT) {
    char c_score[16];
    // convert int to c_score char
    sprintf(c_score, "%.02f %%", a*100);
    wb_supervisor_set_label(lid, c_score, 0.92, pos, 0.1, 0xff0000, 0.0, "Arial");  // spn percent
    sprintf(c_score, "%.02f %%", b*100);
    wb_supervisor_set_label(rid, c_score, 0.22, pos, 0.1, 0x0000ff, 0.0, "Arial");  // non percent
  }
  else if (LAYOUT_BVIOR){
    char c_score[16];
    sprintf(c_score, "%.01f", a);
    wb_supervisor_set_label(lid, c_score, 0.01, pos, 0.08, 0xff0000, 0.0, "Arial");  // spn percent
    sprintf(c_score, "%.01f", b);
    wb_supervisor_set_label(rid, c_score, 0.12, pos, 0.08, 0x0000ff, 0.0, "Arial");  // non percent
  }
}

string double2str(double x, int num_pre = 2)
{
    std::string num_text = std::to_string(x);
    return num_text.substr(0, num_text.find(".")+num_pre);
}

string decipher_b(int behavior_int){
    switch(behavior_int) {
      case 0:
        return "idle";
      case 1:
        return "pass";
      case 2:
        return "shoot";
      case 8:
        return "N_chase";
      case 88:
        return "I_chase";
      case 68:
        return "C_chase";
      case 69:
        return "D_chase";
      case 9:
        return "pass";
      case 18:
        return "capture";
      case 4:
        return "off-ball";
      case 14:
        return "atk_off-ball";
      case 24:
        return "def_get-ball";
      case 44:
        return "b_move";
      case 3:
        return "off-ball";
      case 13:
        return "GUARD";
      case 33:
        return "block";
      case 63:
        return "fence";
      case 5:
        return "tackle";
      case 6:
        return "dribble";
      case 10:
        return "f_shoot";
      case 111:
        return "f_release";
      case 112:
        return "s_release";
      default:{
        if (behavior_int > 1000) return "INT_"+decipher_b(behavior_int-1000);
        else return std::to_string(behavior_int);
      }
    }
}
static void show_command(int *_state, int *ball_state, double *par_main, double *par_sub){
  if (!UI_ON) return;

  if (LAYOUT_STAT){
    string list_command = "";
    for (int i = 0; i < 7; i++){
      if (missing_player[i]) continue;
      list_command = list_command + robot_name[i] + " " + std::to_string(_state[i]) + " " + double2str(par_main[i]) + " " + double2str(par_sub[i]) + "    "; 
    }
    // cout << list_command << '\n';
    wb_supervisor_set_label(37, list_command.c_str(), 0.02, 0.9, 0.1, 0x0000ff, 0.0, "Arial");  // non percent

    list_command = "";
    for (int i = 7; i < ROBOTS; i++){
      if (missing_player[i]) continue;
      list_command = list_command + robot_name[i] + " " + std::to_string(_state[i]) + " " + double2str(par_main[i]) + " " + double2str(par_sub[i]) + "    "; 
    }
    // cout << list_command << '\n';
    wb_supervisor_set_label(38, list_command.c_str(), 0.02, 0.8, 0.1, 0xff0000, 0.0, "Arial");  // non percent
  }
  else if (LAYOUT_BVIOR){
    string list_command = "";
    for (int i = 0; i < 7; i++){
      if (missing_player[i]) list_command += '\n';
      else list_command = list_command + robot_name[i] + "   " + decipher_b(_state[i]) +'\n'; //+ " " + double2str(par_main[i]) + " " + double2str(par_sub[i]) + '\n'; 
    }
    // cout << list_command << '\n';
    wb_supervisor_set_label(37, list_command.c_str(), 0.01, 0.1, 0.072, 0x0000ff, 0.0, "Arial");  // non percent

    list_command = "";
    for (int i = 7; i < ROBOTS; i++){
      if (missing_player[i]) list_command += '\n';
      else list_command = list_command + robot_name[i] + "   " + decipher_b(_state[i]) + '\n'; // + double2str(par_main[i]) + " " + double2str(par_sub[i]) + "    "; 
    }
    // cout << list_command << '\n';
    wb_supervisor_set_label(38, list_command.c_str(), 0.01, 0.69, 0.072, 0xff0000, 0.0, "Arial");  // non percent
  }
}



#endif
