#include <iostream>
#define _USE_MATH_DEFINES
#include "position.hpp"
#include "car.hpp"
#include "simulator.hpp"

using namespace std;

int sim_cnt;

void dumpout(simulator* in){
    int len = 0;
    for(int x = 0; x < len; x++)
        cout << "point" << endl;
}

int main(int argc, char *argv[]){
    cout << "Simulator init..." << endl;
    simulator* sim[sim_cnt = 2] = {new simulator(0,0,0,0.1,10,0,0.001),new simulator(0,0,0,0.1,10,0,0.0001)};
    cout << "Simulate..." << endl;
    for(int i =0; i < sim_cnt; i++)
        sim[i]->simulate(10);
    cout << "Simulation finished" << endl;
    delete[] sim;
    return 0;
}