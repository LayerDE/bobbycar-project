#pragma once

class position{
    public:
        position();
        position(float hx,float hy,float hdirection);
        float x,y,direction;
        virtual void move(float move_lenght);
        void move_straight(float lenght, float height);
};