#ifndef RRT_simple   /* Include guard */
#define RRT_simple
 
 
#include <experimental/random>
 
#include <vector>
#include <algorithm>
#include <math.h>
#include <cassert>
#include <iostream>
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;
using namespace std;
#define Dot pair<double, double>
#define Path vector<Dot>
 
struct Circle{
    Dot Center;
    double radius;
};
#define Env vector<Circle>

bool essentiallyEqual(float a, float b, float epsilon)
{    return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * epsilon); }

 
// # return true for collision
bool check_circle(Circle cir, Dot poi, double ext = 0.25){
    double len = sqrt( (poi.first-cir.Center.first)*(poi.first-cir.Center.first)+\
                    (poi.second-cir.Center.second)*(poi.second-cir.Center.second));
    if (len < cir.radius + ext)
        return true;
    return false;
}
 
bool list_circle_collision_check(Env list_circle, Dot q_s){
    for (Circle circle : list_circle)
        if (check_circle(circle, q_s)) return true;
    return false;
}

pair<double, Circle> list_circle_get_min_dist(Env list_circle, Dot q_s){
    double min_dist = 10000000; Circle cir_most;
    for (Circle circle : list_circle)
    {
        double dist2Cir = (circle.Center.first-q_s.first) *  (circle.Center.first-q_s.first) + (circle.Center.second-q_s.second)*(circle.Center.second-q_s.second);
        if (dist2Cir < min_dist)
            min_dist = dist2Cir,
            cir_most = circle;
    }
    return make_pair(min_dist, cir_most);
}


bool circle_collision_check_line(Env list_circle, Dot q_s, Dot q_f){
    for (Circle circle : list_circle){
        // assert(!check_circle(circle, q_s));
        if (check_circle(circle, q_s)) return true;

        if (check_circle(circle, q_f)) return true;
 
        double u_up = ((circle.Center.first-q_s.first)*(q_f.first-q_s.first)+(circle.Center.second-q_s.second)*(q_f.second-q_s.second));
        double u_down = ((q_s.first-q_f.first)*(q_s.first-q_f.first)+(q_s.second-q_f.second)*(q_s.second-q_f.second));
        double u = u_up / u_down;
        double x = q_s.first+u*(q_f.first-q_s.first);
            // double y = q_s.second+u*(q_f.second-q_s.second);
 
        double lower_q = min(q_s.first, q_f.first);
        double upper_q = max(q_s.first, q_f.first);

        // cout << "wow check line " << x << ' ' << y << '\n';
        // cout << "wow checkl line " << lower_q << ' ' << upper_q << ' ' << check_circle(circle, Dot(x, y)) << '\n';
        if ((x > lower_q) && (x < upper_q)) // && check_circle(circle, Dot(x, y)) )
            return true;
    }
    return false;
}

bool out_of_bound(Dot q_q){
    if (q_q.first > 11.62 or q_q.first < -11.62) return true;
    if (q_q.second > 8.5 or q_q.second < -8.5) return true;
    return false;
}

bool stuck_with_goal(Dot q_q, double ext = 0.3){
        if (fabs(q_q.second ) < 1.3 + ext){
            if (q_q.first > 11.56 - ext) // blue goal return 1
                return true;
            else if (q_q.first < -11.56 +ext) // red goal return -1
                return true;
        }
        return false;
}

struct RRT{
    Env circle_obs;
 
    Dot bound_X, bound_Y;
    Dot q_init, q_final;
    Dot q_new = make_pair(-10000, -10000);
    double delta_q = 0.2;
    double dis_num = 100;
 
    vector<Dot> qs; // Store the Dot of node
    vector<int> qs_parent; // Store the parent index of node
 
    RRT(Env circle_, Dot bound_X_, Dot bound_Y_, Dot q_init_, Dot q_final_)
    {
        this->circle_obs = circle_;
        this->bound_X    = bound_X_;
        this->bound_Y    = bound_Y_;
        this->q_init     = q_init_;
        this->q_final    = q_final_;
 
        this->q_new      = q_init;
        this->qs.push_back(q_init);
        this->qs_parent.push_back(-1);
    }
 
    bool collision_check_point(Dot point){
        for (Circle circle : this->circle_obs)
            if (check_circle(circle, point)) return true;

        if (stuck_with_goal(point)) return true;

        return false;
    }

    // Dot bound_line(Dot Start, Dot Stop){
    //   if (fabs(Stop.first) <= HIGH_BOUND_X and fabs(Stop.second) <= HIGH_BOUND_Y){
    //     return Stop;
    //   } 
    //   else {
    //     Dot Answer = Stop;
    //     double B = (Stop.first - Start.first) /  (Stop.second - Start.second);
    //     if (fabs(Answer.first) > HIGH_BOUND_X)
    //       Answer.first = bound_x(Answer.first),
    //       Answer.second = Start.second + (Answer.first - Start.first) / B;

    //     if (fabs(Answer.second) > HIGH_BOUND_Y)
    //       Answer.second = bound_y(Answer.second),
    //       Answer.first = Start.first + B * (Answer.second - Start.second);

    //     return bound_line(Start, Answer);
    //   }
    // }

 
    bool collision_check_line(Dot q_start, Dot q_end){
        if ((q_start.first == q_end.first) && (q_start.second == q_end.second)) return true;

        if (out_of_bound(q_start)){
            double trav_len = sqrt( (q_start.first-q_end.first) *(q_start.first-q_end.first) + (q_start.second-q_end.second) * (q_start.second-q_end.second) );
            for (int dist_step = 0; dist_step < 4; dist_step ++){
                double dist_add = dist_step * 0.5;
                if (dist_add > trav_len) break;
                Dot q_next = Dot(q_start.first + dist_add * (q_end.first - q_start.first) / trav_len, q_start.second + dist_add * (q_end.second - q_start.second) / trav_len);
                if (stuck_with_goal(q_next)) return true;
            }
        }

        return circle_collision_check_line(this->circle_obs, q_start, q_end);
    }
 
    pair<int, Dot> nearest_vertex_check(Dot q_rand){
        // _, _query = self.kdtree.get_nearest((q_rand[0] , q_rand[1], -1 ))
        // # print(_query)
        // return _query[2], (_query[0], _query[1])
 
        double min_length =16 * this->bound_X.second * this->bound_Y.second;
        int index_near = -1;
        Dot q_near = Dot(-1000, -1000);
        for (std::size_t index = 0; index < this->qs.size(); ++index){
            Dot q = this->qs[index];
            double length = (q_rand.first-q.first) *(q_rand.first-q.first) + (q_rand.second-q.second) * (q_rand.second-q.second);
            if (length < min_length){
                min_length = length;
                index_near = index;
                q_near = q;
            }
        }
        return make_pair(index_near, q_near);
    }
 
    void new_point_generate(Dot n_poi, int index_near){
        this->q_new = n_poi;
 
        // this->kdtree.add_point((n_poi[0], n_poi[1], len(this->qs)))
        // # Write into qs & q_parent
        this->qs.push_back(this->q_new);
        this->qs_parent.push_back(index_near);
    }
 
    bool direct_check(Dot new_final){
        q_final = new_final;
 
        if (collision_check_line(this->q_new, q_final) == false){
            // # print("direct check surv from {}, {} to {},{}".format(self.q_new[0], self.q_new[1], self.q_final[0], self.q_final[1]))
 
            this->qs.push_back(q_final);
            this->qs_parent.push_back(this->qs.size()-2);
            return true;
        }
        else{
            return false;
        }
    }
 
    Path get_path(int fid = -1){
        Path path;
        if (fid == -1) fid = this->qs.size() - 1;
 
        while (true){
            path.push_back(this->qs[fid]);
            fid = this->qs_parent[fid];
            if (fid == -1) break;
        }
        return path;
    }
 
    bool extend(Dot q_rand){
        if (collision_check_point(q_rand)) return false;
        pair<int, Dot> tmp = nearest_vertex_check(q_rand);
        int index_near = tmp.first;
        Dot q_near = tmp.second;
 
        double tx = q_near.first + (q_rand.first-q_near.first) * this->delta_q;
        double ty = q_near.second + (q_rand.second-q_near.second) * this->delta_q;
        Dot q_temp = Dot(tx, ty);
 
        if (collision_check_line(q_near, q_temp)) return false;
 
        // # print("new node surv from {}, {} to {},{}".format(q_near[0], q_near[1], q_temp[0], q_temp[1]))
        // # Generate new vertex according to delta_q
        new_point_generate(q_temp, index_near);
        return true;
    }
}; 
 
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0, 1);//uniform distribution between 0 and 1
 
Dot goal_random_vertex_generate(Dot goal, Dot bx, Dot by){
    double dice = dis(gen);
    Dot q_rand = goal;
    if (dice > 0.1)
        q_rand = Dot(bx.first+(bx.second-bx.first)*dis(gen), by.first+(by.second-by.first)*dis(gen));
    return q_rand;
}
 
 
Dot cach_random_vertex_generate(Dot goal, Path waypoint_cached, Dot bx, Dot by){
    double dice = dis(gen);
    Dot q_rand = Dot(-10000, -10000);
    if (waypoint_cached.size() < 2)
        return goal_random_vertex_generate(goal, bx, by);
 
    if (dice < 0.1)
        q_rand = goal;
    else if (dice < 0.7)
        q_rand = waypoint_cached[std::experimental::randint(1, int(waypoint_cached.size()))];
    else
        q_rand = Dot(bx.first+(bx.second-bx.first)*dis(gen), by.first+(by.second-by.first)*dis(gen));
    return q_rand;
}
 
 



Path refined_straight_path(Env env, Path ori){
    if (ori.size() == 2) return ori;

    Path left_p; left_p.push_back(ori[0]);
    Path right_p; right_p.push_back(ori.back());

    int i_l = 0;
    int i_r = ori.size()-1;
    while (true){
        // cout << "iter " << i_l << ' ' << i_r << '\n';
        // cout << "wat " << ori[i_l].first << ' ' << ori[i_l].second << " and " << ori[i_r].first << ' ' << ori[i_r].second << '\n';
        // cout << "env " << env[0].Center.first << ' ' << env[0].Center.second << '\n';
        // cout << " f check " << circle_collision_check_line(env, ori[i_l], ori[i_r]) << '\n';

        if (i_l > i_r) break;
        if (i_l == i_r)break;

        if (!circle_collision_check_line(env, ori[i_l], ori[i_r])){
            left_p.push_back(ori[i_r]);
            break;
        }


        for (int k = i_r; k > i_l; k--)
            if (!circle_collision_check_line(env, ori[i_l], ori[k])){
                i_l = k;
                left_p.push_back(ori[i_l]);
                break;
            }

        for (int k = i_l; k < i_r; k++)
            if (!circle_collision_check_line(env, ori[k], ori[i_r])){
                i_r = k;
                right_p.push_back(ori[i_r]);
                break;
            }

    }
    // cout << left_p.back().first << ' ' << left_p.back().second << '\n';
    // cout << right_p.back().first << ' ' << right_p.back().second << '\n';

    assert(essentiallyEqual(left_p.back().first,right_p.back().first, 0.05));
    assert(essentiallyEqual(left_p.back().second,right_p.back().second, 0.05));

    right_p.pop_back();
    reverse(right_p.begin(), right_p.end());
    left_p.insert(end(left_p), begin(right_p), end(right_p));
    return left_p;
}


#define RRT_RUN true
 
Path path_plan(const Dot q_s,const  Dot q_f,const Env Cir,const  Dot bound_x,const  Dot bound_y,\
 Path cached, bool UI_ENABLE = 0, bool useVoronoi = 0, bool ERRT = 1){

    // bound_extend(bound_x, q_f)

    if (list_circle_collision_check(Cir, q_s)){
// special case, maybe bottle-neck

        Path stupid_path;
        stupid_path.push_back(q_s);

        auto tempMinCir= list_circle_get_min_dist(Cir, q_s);
        Circle most_faulty_circle = tempMinCir.second;

        if (fabs(q_s.first ) < 5 or  (tempMinCir.first <= 1 and fabs(q_s.first) < 8 ) )
            stupid_path.push_back(Dot( -most_faulty_circle.Center.first + 2* q_s.first, -most_faulty_circle.Center.second + 2* q_s.second ));
        else {

            // if (tempMinCir.first > 1) {
            //     Env mod_Cir;
            //     for (Circle circle : Cir)
            //         mod_Cir.push_back(Circle{circle.Center, 1});
            //     return path_plan(q_s, q_f, mod_Cir, bound_x, bound_y, cached, UI_ENABLE, useVoronoi, ERRT);
            // }
            
            double Vx = (q_s.first - most_faulty_circle.Center.first);
            double Vy = (q_s.second - most_faulty_circle.Center.second);
            if (Vy == 0) Vy = 0.1;
            // X*Vx + Y * Vy = right_side
            // double right_side = q_s.first * Vx + q_s.second * Vy;
            if (q_f.first > q_s.first)
                // stupid_path.push_back( Dot(q_s.first + 2, q_s.second+ (right_side - Vx*(2) )/Vy ));
                stupid_path.push_back( Dot(q_s.first + 2, q_s.second+ (- Vx*2 )/Vy ));
            else 
                stupid_path.push_back( Dot(q_s.first - 2, q_s.second+ (+ Vx*2 )/Vy ));

                // stupid_path.push_back( Dot(q_s.first - 2, q_s.second+ (right_side - Vx*(-2) )/Vy ));
        }
        return stupid_path;   


        // double fix_rad = 1.8;

        // // double current_best = -1; Dot q_worst =  make_pair(-10000, -10000);
        // // Path list_avail;
        // int STEP_NUM = 36;
        // double STEP_ANG = 360 / STEP_NUM;
        // for (int i = 0; i < STEP_NUM; i++){

        //     Dot q_test = make_pair(q_s.first + fix_rad * cos(  i*STEP_ANG *  M_PI / 180.0 ),  q_s.second + fix_rad * sin(  i*STEP_ANG *  M_PI / 180.0 ));
        //     // double min_danger_dist = list_circle_get_min_dist(Cir, q_test);

        //     // if (min_danger_dist > 1){
        //     //     // move to a list, with a percent equal
        //     //     list_avail.push_back(q_test);
        //     // }
        //     // else if (min_danger_dist > current_best)
        //     //     current_best = min_danger_dist,
        //     //     q_worst = q_test;
        //     if (list_circle_collision_check(Cir, q_test) == false){
        //         Path stupid_path;
        //         stupid_path.push_back(q_s);
        //         stupid_path.push_back(q_test);
        //         return stupid_path;   
        //     }
        // }
        // list_avail.push_back(q_worst);

        // Path stupid_path;
        // stupid_path.push_back(q_s);
        // stupid_path.push_back(list_avail[ std::experimental::randint(1, int(list_avail.size())) ]);
        // cout << "RRT FAILED " << list_avail.size() << ' ' << stupid_path.back().first << ' ' << stupid_path.back().second << '\n';
        // return stupid_path;     

        // Env mod_Cir;
        // for (Circle circle : Cir)
        //     mod_Cir.push_back(Circle{circle.Center, circle.radius-0.22});
        // cout << "-------------------SELF_CALLED RRT\n";
        // while (true){
        //     Path test_path = path_plan(q_s, q_f, mod_Cir, bound_x, bound_y, cached, UI_ENABLE, useVoronoi, ERRT);
        //     if (list_circle_collision_check(Cir, test_path[1]) == false)
        //         return test_path;
        // }

        // while (true)
        // {
        //     Dot q_rand = goal_random_vertex_generate(q_f, bound_x, bound_y);
        //     if (list_circle_collision_check(Cir, q_rand) == false)
        //     {
        //         Path stupid_path;
        //         stupid_path.push_back(q_s);
        //         stupid_path.push_back(q_rand);
        //         return stupid_path;
        //     }
        // }
    }


    bool BiRRT = false;
    if (list_circle_collision_check(Cir, q_f)) BiRRT = false;

    // fig, ax = plt.subplots()
    // if not (UI_ENABLE): plt::close();
 
    // if (useVoronoi):
    //     vorRegs, cells = gen_voronoi(Cir[:,1:], ax)
    //     delau = Delaunay(Cir[:,1:])
    //     delau_ind, delau_edg = delau.vertex_neighbor_vertices
 
    RRT rrt_a = RRT(Cir, bound_x, bound_y, q_s, q_f);
    RRT rrt_b = RRT(Cir, bound_x, bound_y, q_f, q_s);
    int NODE_LIMITER = 50;
    int num_iterate = 0;
    bool FOUND = false;
    
 
    while (RRT_RUN && FOUND == false) {
        if (max(int(rrt_a.qs.size()), int(rrt_b.qs.size())) > NODE_LIMITER) {
            // cout << "max node read " << rrt_a.qs.size() << ' ' << rrt_b.qs.size() << '\n';
            break;
        }
        if (num_iterate > 100) {
                 // cout << "num_iterate " << num_iterate << '\n';
                 break;
        }
        if (useVoronoi && ERRT){
            // cout << "not yet";
            break;
        }
 
        Dot current_target = rrt_b.q_final;
        bool reversed_tree = false;
        if (BiRRT && rrt_a.qs.size() < rrt_b.qs.size()){
            current_target = rrt_a.q_final;
            reversed_tree = true;
            // RRT C = rrt_a;
            // rrt_a = rrt_b;
            // rrt_b = C;
            // rrt_a, rrt_b = rrt_b, rrt_a
        }
        if (!BiRRT) current_target = rrt_a.q_final;

        Dot q_rand = Dot(-10000, -10000);
        // if (useVoronoi)
        //     q_rand = voro_random_vertex_generate(rrt_b.q_final, vorRegs, cells, delau_ind, delau_edg)
        if (ERRT)
            q_rand = cach_random_vertex_generate(current_target, cached, bound_x, bound_y);
        else
            q_rand = goal_random_vertex_generate(current_target, bound_x, bound_y);

        // cout << "random " << q_rand.first << ' ' << q_rand.second << '\n';
 
        if (BiRRT){
            if (!reversed_tree){
                if (rrt_b.extend(q_rand)){
                    if (rrt_a.extend( rrt_b.q_new )){
                        if (rrt_a.direct_check(rrt_b.q_new) or \
                            rrt_b.direct_check(rrt_a.q_new)){
                            if (UI_ENABLE) cout << "ITERATE K= " << num_iterate;
                            FOUND = 1;
                            break;
                        }
                    }
                }
            }
            else{
                if (rrt_a.extend(q_rand)){
                    if (rrt_b.extend( rrt_a.q_new )){
                        if (rrt_b.direct_check(rrt_a.q_new) or \
                            rrt_a.direct_check(rrt_b.q_new)){
                            if (UI_ENABLE) cout << "ITERATE K= " << num_iterate;
                            FOUND = 1;
                            break;
                        }
                    }           
                }
            }
        }
        else{
            if (rrt_a.direct_check(rrt_a.q_final)){
                FOUND = 1;
                break;
            }
            rrt_a.extend(q_rand);
        }
        num_iterate += 1;
    }
 
    // if (UI_ENABLE): rrt_a.figure_generate(ax)
    // if (UI_ENABLE): rrt_b.figure_generate(ax)

    Path path_a = rrt_a.get_path();
    if (!BiRRT) {
        pair<int, Dot> tmp = rrt_a.nearest_vertex_check(rrt_a.q_final);
        int index_near = tmp.first;
        path_a = rrt_a.get_path(index_near);
    }

    Path path_b = rrt_b.get_path();
    Path path, ref_path;

    if (!(  (path_a.back().first == q_s.first) && (path_a.back().second == q_s.second)  ))
    {
        path = path_a;
        path_a = path_b;
        path_b = path;
    }
    reverse(path_a.begin(), path_a.end());
    path = path_a;
    path.insert(end(path), begin(path_b), end(path_b));
 
    // if (UI_ENABLE): plot_circle(Cir, ax)
    // if (UI_ENABLE): plt.xlim([0, SIZE_X])
    // if (UI_ENABLE): plt.ylim([0, SIZE_Y])
    // if (UI_ENABLE): fig.tight_layout()
 
    if (UI_ENABLE){
        if (!FOUND) 
            cout << "FAILED with " << rrt_a.qs.size()+rrt_b.qs.size() << " nodes at " << num_iterate << " iters\n";
        else 
            cout << "FOUNDED with "<< rrt_a.qs.size()+rrt_b.qs.size() <<" nodes\n";
    }

    if (UI_ENABLE){
        for (auto i : path)
            cout << "ORG RRTPATH " << i.first << ' ' << i.second << '\n';
    }

    if (FOUND) ref_path = refined_straight_path(Cir, path);
    else ref_path = refined_straight_path(Cir, path_a);

    // if (UI_ENABLE){
    //     for (auto i : ref_path)
    //         cout << "REF RRTPATH " << i.first << ' ' << i.second << '\n';
    // }

    if (ref_path.size() == 1){
        ref_path.push_back(Dot(  (q_s.first+q_f.first)/2, (q_s.second+q_f.second)/2   ));
        // cout << "                                                           WHAT \n";
        // for (auto i : ref_path)
        //     cout << "REF RRTPATH " << i.first << ' ' << i.second << '\n';
        // cout << "                                       WwWWWW                 WHAT " << path.size() << "     " << FOUND << '\n';

        // for (auto i : path_a)
        //     cout << "org RRTPATH " << i.first << ' ' << i.second << '\n';

        // pair<int, Dot> tmp = rrt_a.nearest_vertex_check(rrt_a.q_final);
        // int index_near = tmp.first;
        // path_a = rrt_a.get_path(index_near);    

        // for (auto i : path_a)
        //     cout << "again RRTPATH " << i.first << ' ' << i.second << '\n';
        // cout << "index_near " << index_near << '\n';

        // cout << "q_f " << q_f.first << ' ' << q_f.second << '\n';
    }
 
    // for i in range(len(ref_path) - 1):
    //     x = [ref_path[i][0], ref_path[i+1][0]]
    //     y = [ref_path[i][1], ref_path[i+1][1]]
    //     ax.plot(x, y, "b", linewidth=0.3)
 
 
    // # ref_x = []
    // # ref_y = []
    // # for i in range(len(ref_path)):
    // #     ref_x.push_back(ref_path[i][0])
    // #     ref_y.push_back(ref_path[i][1])
 
    // # tck, u = interpolate.splprep([ref_x, ref_y],k=2)
    // # x_i, y_i = interpolate.splev(np.linspace(0, 1, 100), tck)
    // # ax.plot(x_i, y_i)
 
    // # plt.show()
    // # if (UI_ENABLE):
    //     # plt.show()
    //     # if (not FOUND) or (len(rrt_a.qs)+len(rrt_b.qs) > 600): plt.show()
    //     # else: plt.close()
    // return FOUND, path, ref_path, len(rrt_a.qs), len(rrt_b.qs), num_iterate
    return ref_path;
}
 
#endif // BASIC RRT
 
 
// g++ test.cpp -I /usr/include/python3.10 -lpython3.10 -o test
// time ./test
 
 