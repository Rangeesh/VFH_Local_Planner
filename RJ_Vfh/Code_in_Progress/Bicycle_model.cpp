#include<iostream>
#include<cmath>
#include<Bicycle_Model.h>

#define Deg2Rad(x) x*3.14/180

using namespace std;

void KinematicBicycle::update(float ax, float del_f) // Using Forward Euler Method
// del_f is of the road wheel in degrees
{
    float v_next = v + ax*TIME_STEP;
    float x_next = x + v*cos(Deg2Rad(theta+beta))*TIME_STEP;
    float y_next = y + v*cos(Deg2Rad(theta+beta))*TIME_STEP;
    float theta_next = theta + v/lr*sin(Deg2Rad(beta))*TIME_STEP;
    beta = atan2(lr*tan(Deg2Rad(del_f))/(lr+lf));
    r = cot(Deg2Rad(del_f))*(lf+lr)/cos(Deg2Rad(beta));
    if (r>999) r=1000;
    x = x_next;
    y = y_next;
    theta = theta_next;
    v = v_next;

    s = min(5, v);
}