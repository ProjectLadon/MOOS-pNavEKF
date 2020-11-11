/************************************************************/
/*    NAME: Pierce Nichols                                    */
/*    ORGN: Ladon Robotics                                             */
/*    FILE: NavEKF_increment.h                                        */
/*    DATE:                                                 */
/************************************************************/

#pragma once

extern "C" {
    #include "roboticscape.h"
}

using namespace std;

enum state_axis_t : uint8_t {
    x           = 0,
    y           = 1,
    theta       = 2,
    v           = 3,
    theta_dot   = 4,
    v_dot       = 5
};

class NavState2D
{
public:
    NavState2D(rc_matrix_t sensor_matrix, float time_step);
    ~NavState2D();

    void tick(rc_vector_t *last_x);
    const rc_matrix_t &getF() {return F;};
    const rc_matrix_t &getH() {return H;};
    const rc_vector_t &getXPrediction() {return x_predict;};
    const rc_vector_t &getYPrediction() {return y_predict;};
    static const int getStateCount();
private:
    const float dt;
    rc_matrix_t H;
    rc_matrix_t F;
    rc_vector_t x_predict;
    rc_vector_t y_predict;

    void calcF(rc_vector_t *x);
};
