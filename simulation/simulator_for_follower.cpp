#include <iostream>
#define _USE_MATH_DEFINES
#include "position.hpp"
#include "car.hpp"
#include "simulator.hpp"

const int sim_cnt;

int main(int argc, char *argv[]){
    simulator* sim[sim_cnt = 2]={new simulator(0,0,0,0.1,10,0,0.001),new simulator(0,0,0,0.1,10,0,0.0001)};
    for(int i =0; i < sim_cnt; i++)
        sim[i]->simulate(10);
    delete[] sim;
    return 0;
}