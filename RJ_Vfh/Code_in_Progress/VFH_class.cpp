#include<vector>
#include<iostream>
#include<unordered_map>
#include<cmath>
#include<iomanip>

#include <VFH_class.h>

#define ITER 5
#define Rad2Deg(x) x*180/3.14

using namespace std;

cout<<std::fixed<<std::setprecision(2);

#define SIGN(x) (x<0): -1 ? 1


explicit Histogram::Histogram(float cw=CELL_WIDTH, int num=NUM_CELL, float s_angle = SECTOR_ANGLE)
{
    cell_width=cw;
    num_cell = num;
    car_position.x = (num_cell-1)/2;
    car_position.y = (num_cell-1)/2;
    dmin = DMIN;
    s = S;
    sd = DMIN;
    Cr = 1.5;
    sector_angle=s_angle;
    num_sector = 360/sector_angle;
    //Cost function Parameter Initialization
    mu_1 = MU_1;
    mu_2 = MU_2;
    // mu_3 = MU_3;
    //Resizing - Cartesian Histogram
    cart_hist.resize(num,vector<int>(num));
    cart_hist_now.resize(num,vector<int>(num));
    no_obs_hist.resize(num,vector<int>(num));
    // Resizing - Polar Histogram
    Mapping.resize(num_cell, vector<sector>(num_cell));
    for(int i = 0;i<num_cell;++i)
    {
        for(int j = 0;j<num_cell;++j)
        {
            Mapping[i][j].angle = Rad2Deg(atan2(j - car_position.y,i-car_position.x));
            int signed_k = int (Mapping[i][j].angle/sector_angle);
            Mapping[i][j].k = (signed_k>=0 ? signed_k : signed_k + num_sector;
            Mapping[i][j].distance = cell_width*(sqrt((i-car_position.x),(y-car_position.y)));
        }
    }

    for(int i =0;i<num_sector;++i) // Is it necessary?
    {
        primary_phist[i]=0;
        binary_phist[i]=0;
        masked_phist[i]=0;
    }

}

void Histogram::updateBinary() // Assumes Primary is already done
// 1 - Is it blocked? - TRUE
// 0 - Is it blocked? - FALSE
{
    for(int i =0;i<num_sector; ++i)
    {
        if (primary_phist[i]>high_threshold)
        {
            binary_phist[i] = 1;
        }
        else if (primary_phist[i]<low_threshold)
        {
            binary_phist[i]=0;
        }
    }
}

void Histogram::updateMasked(float& s) // For every active cell, limit - phi_l and phi_r
{ // s - searching step
    float rmin = 7; // Random value taken for now!!
    float theta_s = asin(s/(2*rmin);// searching_step theta
    phi_l = pi/2 + theta_s;
    phi_r = pi/2 - theta_s;
    for(int i =0;i<num_sector;++i)
    {
        if ((sector_angle)*(i)<=phi_r || (sector_angle)*(i+1)>=phi_l)
        {
            masked_phist[i]=1;
        }
        else
        {
            masked_phist[i]=binary_phist[i];
        }
    }

}

int Histogram::computetargetdirection()//! ...!
{
    float ang = atan2 (y_target,x_target);
    return int(ang/sector_angle);
}


float Histogram::delta( int k, int l)
{
    // return min(min(abs(k-l),abs(k-l-)),) // todo
    return abs(k-l);
}

float Histogram::costfunction(int k)
{
    return mu_1*delta(k,k_t) + mu_2*delta(k,0);
}

float Histogram::computesteeringangle() // Output in degrees
{
    int p = findcandidatedirection();

    if (p==-1) // no candidate direction
    {

    }

    float steering = (p*sector_angle + 2.5)*STEERING_RATIO;

    return steering;
}

int Histogram::findcandidatedirection()
{

    k_t = computetargetdirection();
    candidate_directions.clear(); // size zero
    // sector values will be pushed back

    for(int i = int(phi_r/sector_angle) + 1 ; i < int(phi_l/sector_angle); ++i)
    {
        if (masked_phist[i]==0)
        {
            int k_l = i;
            int k_r = i;
            while(true)
            {
                ++i;
                if (masked_phist[i]==0) continue; else break;
            }
            --k_r;

            if (k_l-k_r < SMAX and (k_l-k_r)>6) // Narrow Opening - not checking for target in here -  6 is an estimated value
            {
                candidate_directions.push_back((k_r+k_l)/2);
            }

            else
            {
                if (k_t<=k_r and k_t>=k_l)
                {
                    candidate_directions.push_back(k_t);
                }

                candidate_directions.push_back(int(k_r + SMAX/2));
                candidate_directions.push_back(int(k_l - SMAX/2));
            }
        }

    }


    // Find cost of each candidate function, and tell final sector direction
    // If no sector - write function to reduce speed -- Based on the closest obstacle in front...
    
    if (candidate_directions.empty()) return -1;
    float mincost = 0;
    float mink = 0;
    for (auto it = candidate_directions.begin(); it!=candidate_directions.end();++it)
    {
        if (it==candidate_directions.begin())
        {
            mincost = costfunction(*it); //!
            mink = *it;
            continue;
        }

        if (mincost > costfunction(*it))
        {
            mincost = costfunction(*it);
            mink = *it;
        }

    }
    return mink;
}


void Histogram::update(vector<Point>& LiDAR2D, float v)
{

    sd = s*v + dmin;
    td = sd + cr;


    for (auto it = LiDAR2D.begin(); it!=LiDAR2D.end(); ++it)
    {
        int X = car_position.x - SIGN(it->x)*(roundf((it->x-cell_width/2.0)/cell_width) + 1);
        int Y = car_position.y - SIGN(it->y)*(roundf((it->y-cell_width/2.0)/cell_width) + 1);
        cart_hist_now[X][Y]+=1;
    }

    //! I am not limiting the maximum value of histogram for now

    for(int i =0;i<num_cell; ++i)
    {
        for(int j = 0;j<num_cell;++j)
        {
            cart_hist[i][j]+=cart_hist_now[i][j];
            if (cart_hist_now[i][j]==0)
            {
               if (no_obs_hist[i][j]==ITER) 
               {
                   cart_hist[i][j]==0;
               }
               else
               {
                   no_obs_hist[i][j]++;
               }
               
            }

            // updating Primary Polar Histogram
            {
                //! Equation from notes
                float enlarge_angle = Rad2Deg(asin(td,Mapping[i][j].distance));
                float angle_min = Mapping[i][j].angle - enlarge_angle; // Crossover at 180/ -180 unlikely - hence omitted
                int k_min = int (angle_min/SECTOR_ANGLE);
                int n_sectors = int(2*enlarge_angle/sector_angle);
                float mag = (cart_hist[i][j])*(cart_hist[i][j])*(a - b*Mapping[i][j].distance*Mapping[i][j].distance);
                int ct = 0;
                while (ct<n_sectors)
                {
                    int k_signed = k_min + ct;
                    int k_actual = (k_signed>=0 ? k_signed : k_signed + 360/SECTOR_ANGLE;
                    primary_phist[k_actual]+= mag;
                }

            }

        }
    }

    updateBinary();

}

void Histogram::display_cart_hist_now()
{
    cout<<"----------Current Cartesian Histogram-----------\n";
    for (int i =0;i<num_cell;++i)
    {
        for(int j = 0;j<num_cell;++j)
        {
            cout<<cart_hist_now[i][j]<<" ";
        }
        cout<<endl;
    }

}

void Histogram::display_cart_hist()
{
    cout<<"----------Cartesian Histogram-----------\n";
    for (int i =0;i<num_cell;++i)
    {
        for(int j = 0;j<num_cell;++j)
        {
            cout<<cart_hist[i][j]<<" ";
        }
        cout<<endl;
    }

}


ostream& operator<<(ostream& out, Point& P)
{
    cout<<"Point: x = "<<P.x<<", y = "<<P.y<<" \n";
}

// explicit PolarHistogram::PolarHistogram(float cw=CELL_WIDTH, int num=NUM_CELL)
// {
    // cell_width=cw;
    // num_cell = num;
    // car_position.x = (num_cell-1)/2;
    // car_position.y = (num_cell-1)/2;

    // Resizing

    // Mapping.resize(num_cell, vector<sector>(num_cell));
    // for(int i = 0;i<num_cell;++i)
    // {
    //     for(int j = 0;j<num_cell;++j)
    //     {
    //         float angle = atan2(i-car_position.x,j - car_position.y);
    //         int signed_k = int (angle/SECTOR_ANGLE);
    //         Mapping[i][j].k = (signed_k>=0 ? signed_k : signed_k + 360/SECTOR_ANGLE;
    //         Mapping[i][j].distance = cell_width*(sqrt((i-car_position.x),(y-car_position.y)));
    //     }
    // }

    // for(int i =0;i<360/SECTOR_ANGLE;++i) // Is it necessary?
    // {
    //     primary_phist[i]=0;
    //     binary_phist[i]=0;
    //     masked_phist[i]=0;
    // }
// }


