#ifndef _VFH_H_
#define _VFH_H_

#include<vector>
#include<iostream>
#include<unordered_map>

#define CELL_WIDTH 0.4 // Should be non zero!
#define MAX_RANGE_LIDAR 25
#define NUM_CELL int(MAX_RANGE_LIDAR/CELL_WIDTH)
#define SECTOR_ANGLE 5
#define DMIN 3
#define S 0.2 // ? define this !!
#define SMAX 16 // Number of sectors to differentiate between wide and narrow
#define MU_1 5
#define MU_2 3
// #define MU_3 2
#define STEERING_RATIO 16

using namespace std;

class Point
{
    public:
    float x;
    float y;
    Point(float X = 0, float Y = y): x(X), y(Y) {}
};

struct sector
{
    float angle;
    int k;
    float distance;
};
ostream& operator<<(ostream& out, Point& P);

class Histogram
{
    public:

    // friend PolarHistogram;

    float cell_width; // in Metres
    int num_cell; // Number of Cells lengthwise - Must be odd
    Point car_position;
    int sector_angle;
    int num_sector;

    // Binary Parameters

    float low_threshold;
    float high_threshold;

    // Masked Parameters
    float phi_l;
    // float phi_b; // pi + current heading - NOT! zero for us
    float phi_r;

    // Parameters

    float dmin;
    float s;
    float sd; // S*Velocity + dMIN - Safety Distance
    float cr; // Car Radius
    float td; // Cr + sd - Total Distance
    float a;
    float b; // FOr finding magnitude - cv^2 *(a - b*d^2)

    

    // Candidate Directions Parameters
    int k_t;

    // Target Computation Parameters
    float x_target; //! Need to figure out a way to update these values
    float y_target; //! Need to figure out a way to update these values

    // Cost Function Parameters
    float mu_1;
    float mu_2;
    // float mu_3;

    vector<vector<int>> cart_hist;
    vector<vector<int>> cart_hist_now;
    vector<vector<int>> no_obs_hist;

    vector<vector<sector>> Mapping;

    unordered_map<int,float> primary_phist;
    unordered_map<int,float> binary_phist;
    unordered_map<int,float> masked_phist;

    vector<int> candidate_directions;

    explicit Histogram(float cw=CELL_WIDTH, int num=NUM_CELL, float s_angle = SECTOR_ANGLE);

    void update(vector<Point>& LiDAR2D, float velocity);

    void updateBinary();

    float delta(int k, int l);

    void updateMasked(float& s); // function of velocity

    int computetargetdirection();

    float costfunction(int k);

    int findcandidatedirection();

    float computesteeringangle();

    void display_cart_hist_now(); // Use Python for Displaying
    void display_cart_hist(); // Use Python for Displaying
    
};





// class PolarHistogram
// {
//     // float cell_width; // in Metres - must be positive
//     // int num_cell; // Number of Cells lengthwise - Must be odd
//     // Point car_position;

//     public:

//     // vector<vector<sector>> Mapping;

//     // unordered_map<int,float> primary_phist;
//     // unordered_map<int,float> binary_phist;
//     // unordered_map<int,float> masked_phist;

//     // explicit PolarHistogram(float cw=CELL_WIDTH, int num=NUM_CELL);

//     // void Update(const CartesianHistogram& A);
    

    

// };




#endif
