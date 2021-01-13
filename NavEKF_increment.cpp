/************************************************************/
/*    NAME: Pierce Nichols                                    */
/*    ORGN: Ladon Robotics                                             */
/*    FILE: NavEKF_increment.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include "NavEKF_increment.h"
#include <cmath>

const uint8_t state_count = 6;
#define PI 3.14159265
#define DEG2RAD (180/PI)

const int NavState2D::getStateCount() {return state_count;}

NavState2D::NavState2D(rc_matrix_t sensor_matrix, float time_step):
dt(time_step),
H(rc_matrix_empty()), 
F(rc_matrix_empty()),
x_predict(rc_vector_empty()),
y_predict(rc_vector_empty())
{
    rc_matrix_duplicate(sensor_matrix, &H);
    rc_vector_zeros(&x_predict, state_count);
    rc_vector_zeros(&y_predict, H.rows);
    rc_matrix_zeros(&F, state_count, state_count);
}

NavState2D::~NavState2D()
{
    rc_vector_free(&x_predict);
    rc_vector_free(&y_predict);
    rc_matrix_free(&F);
    rc_matrix_free(&H);
}

void NavState2D::tick(rc_vector_t *last_x)
{
    // propagate x_k
    x_predict.d[state_axis_t::x] = last_x->d[state_axis_t::x] +
        (dt * last_x->d[state_axis_t::v] * cos(last_x->d[state_axis_t::theta] * DEG2RAD)) +
        (0.5 * dt * dt * last_x->d[state_axis_t::v_dot] * cos(last_x->d[state_axis_t::theta] * DEG2RAD));
    // propagate y_k
    x_predict.d[state_axis_t::y] = last_x->d[state_axis_t::y] +
        (dt * last_x->d[state_axis_t::v] * sin(last_x->d[state_axis_t::theta] * DEG2RAD)) +
        (0.5 * dt * dt * last_x->d[state_axis_t::v_dot] * sin(last_x->d[state_axis_t::theta] * DEG2RAD));
    // propagate theta_k (heading)
    x_predict.d[state_axis_t::theta] = last_x->d[state_axis_t::theta] +
        (dt * last_x->d[state_axis_t::theta_dot]);
    // propagate v_k (velocity)
    x_predict.d[state_axis_t::v] = last_x->d[state_axis_t::v] +
        (dt * last_x->d[state_axis_t::v_dot]);
    // propagate theta_dot_k (yaw rate)
    x_predict.d[state_axis_t::theta_dot] = last_x->d[state_axis_t::theta_dot];
    // propagate v_dot_k (acceleration)
    x_predict.d[state_axis_t::v_dot] = last_x->d[state_axis_t::v_dot];
    rc_matrix_times_col_vec(H, x_predict, &y_predict);  // predict sensor values
    calcF(&x_predict);                                  // compute Jacobian
}

void NavState2D::calcF(rc_vector_t *x)
{
    rc_matrix_zeros(&F, state_count, state_count);
    F.d[state_axis_t::x][state_axis_t::x] = 1;
    F.d[state_axis_t::x][state_axis_t::theta] = -(x->d[state_axis_t::v] * dt * sin(x->d[state_axis_t::theta] * DEG2RAD)) -
                (0.5 * dt * dt * x->d[state_axis_t::v_dot] * sin(x->d[state_axis_t::theta] * DEG2RAD));
    F.d[state_axis_t::x][state_axis_t::v] = (dt * cos(x->d[state_axis_t::theta] * DEG2RAD));
    F.d[state_axis_t::x][state_axis_t::v_dot] = (0.5 * dt * dt * cos(x->d[state_axis_t::theta] * DEG2RAD));
    F.d[state_axis_t::y][state_axis_t::y] = 1;
    F.d[state_axis_t::y][state_axis_t::theta] = (x->d[state_axis_t::v] * dt * cos(x->d[state_axis_t::theta] * DEG2RAD)) +
                (0.5 * dt * dt * x->d[state_axis_t::v_dot] * cos(x->d[state_axis_t::theta] * DEG2RAD));
    F.d[state_axis_t::y][state_axis_t::v] = (dt * sin(x->d[state_axis_t::theta] * DEG2RAD));
    F.d[state_axis_t::y][state_axis_t::v_dot] = (0.5 * dt * dt * sin(x->d[state_axis_t::theta] * DEG2RAD));
    F.d[state_axis_t::theta][state_axis_t::theta] = 1;
    F.d[state_axis_t::theta][state_axis_t::theta_dot] = dt;
    F.d[state_axis_t::v][state_axis_t::v] = 1;
    F.d[state_axis_t::v][state_axis_t::v_dot] = dt;
    F.d[state_axis_t::theta_dot][state_axis_t::theta_dot] = 1;
    F.d[state_axis_t::v_dot][state_axis_t::v_dot] = 1;
}
