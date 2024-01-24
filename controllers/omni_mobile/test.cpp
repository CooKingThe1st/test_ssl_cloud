
#include "RRT_simple.h"
using namespace std;
 
int main(){
    // srand(502);
    Dot q_s = Dot( 1.21074, -0.33328 );
    Dot q_f = Dot(2.21047 ,0.556239);
    Env Cir;
    Cir.push_back(Circle{Dot( 2.461 ,0.560928 ), 0.247});
     Cir.push_back(Circle{Dot( 0.388832, 0.206744), 0.247});
    Dot bx = Dot(-11.6, 11.6);
    Dot by = Dot(-8, 8);
 
    Path cached;
    cached.push_back(Dot(1.21074, -0.33328));
 
    Path test = path_plan(q_s, q_f, Cir, bx, by, cached, 1);
    // test = path_plan(q_s, q_f, Cir, bx, by, test, 1);
    // test = path_plan(q_s, q_f, Cir, bx, by, test, 1);
    // test = path_plan(q_s, q_f, Cir, bx, by, test, 1);
    // test = path_plan(q_s, q_f, Cir, bx, by, test, 1);

    for (auto i : test)
        cout << i.first << ' ' << i.second << ' ' << '\n';
}