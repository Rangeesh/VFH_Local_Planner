#ifndef _BICYCLE_MODEL_
#define _BICYCLE_MODEL_
#define TIME_STEP 0.2

class KinematicBicycle
{
    float lr; // CG to Rear Axle
    float lf; // CG to Front Axle
    float sr; // Steering Ratio
    public:
    float x; // Inertial X
    float y; // Inertial Y
    float theta; // Inertial Theta - degrees
    float v; // Velocity
    float beta; // Slip Angle - degrees
    float r; // Radius of Curvature
    float s; // searching step
    // If R>999 - It's assumed to be a straight line
    explicit KinematicBicycle(float LF=1.0, float LR = 1.0) : lr(LR), lf(LF),sr(15.4), x(0), y(0), v(0), theta(0), beta(0), r(1000) {update(0,0);} 
    void update(float ax, float del_f);
    float radius() {return r;}
    float searching_step() {return s;}
};



#endif