#include <cmath>
#include "car.hpp"

car::car(float wb, float rh, float hx, float hy, float hdirection, float halpha) : position(hx, hy,hdirection){
    wheelbase = wb;
    rear2hitch = rh;
    alpha = halpha;
}
void car::calc_curve(float lenght, float alpha_steer, float &x, float &y, float &angle)
{
    if(alpha_steer == 0.0f){
        x=lenght;
        y=0;
        angle = 0;
    }
    else{
        float r_bw = wheelbase/tan(fabs(alpha_steer));
        angle = lenght/r_bw;
        x = sin(angle) * r_bw;
        y = (1 - cos(angle)) * r_bw;
    }
}
void car::move(){
    float x,y,a;
    calc_curve(lenght,alpha,x,y,a);
    move_straight(x,y);
    direction+=a;
};