#pragma once

class position{
    public:
        position(float hx,float hy,float hdirection);
        float x,y,direction;
        void move();
        void move_straight(float lenght, float height);
};