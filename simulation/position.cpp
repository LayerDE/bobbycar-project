#include <cmath>
#include "position.hpp"


void position::move(){};

void position::move_straight(float lenght, float height){
        x = cos(direction)*lenght;
        y = sin(direction)*lenght;
        x = sin(direction)*height;
        y = cos(direction)*height;
}

position::position(float hx,float hy,float hdirection){
    x = hx;
    y = hy;
    direction = hdirection;
}