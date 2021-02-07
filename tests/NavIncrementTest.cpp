#include "../NavEKF_increment.h"
#include "gtest/gtest.h"
#include <random>
#include <cmath>
#include <iostream>
#include <chrono>

extern "C" {
    #include "roboticscape.h"
}

#define STDTOL              (0.01)
#define LINTOL              (0.1)
#define STDTS               (0.1)
#define XY_MIN              (-10000)
#define XY_MAX              (10000.1)
#define XY_STEP             (100)
#define THETA_MIN           (0)
#define THETA_MAX           (360)
#define THETA_STEP          (45)
#define THETA_DOT_MIN       (-40)
#define THETA_DOT_MAX       (40.1)
#define THETA_DOT_STEP      (10)
#define V_MIN               (-5)
#define V_MAX               (5.1)
#define V_STEP              (2.5)
#define V_DOT_MIN           (-2)
#define V_DOT_MAX           (2.1)
#define V_DOT_STEP          (0.5)
#define DEG2RAD             (M_PI/180)

class TickTestFramework : public ::testing::Test
{
    protected:
    void SetUp ()
    {
        re.seed(chrono::system_clock::now().time_since_epoch().count());
        sensor_matrix = rc_matrix_empty();
        input_vector = rc_vector_empty();
        output_vector = rc_vector_empty();
        rc_matrix_identity(&sensor_matrix, NavState2D::getStateCount());
        rc_vector_zeros(&input_vector, NavState2D::getStateCount());
        rc_vector_zeros(&output_vector, NavState2D::getStateCount());

        test_obj = new NavState2D(sensor_matrix, STDTS);
    }

    void TearDown()
    {
        delete test_obj;
        rc_matrix_free(&sensor_matrix);
        rc_vector_free(&input_vector);
        rc_vector_free(&output_vector);
    }

    bool equalWithTol (const double a, const double b, const double tol = STDTOL)
    {
        if (tol > fabs(a - b)) {
            return true;
        } else {
            cerr << "a " << to_string(a) << "\tb " << to_string(b);
            cerr << "\tdifference is " << to_string(fabs(a - b));
            cerr << "\ttolerance is " << to_string(tol) << endl;
            return false;
        }
        return true;
    }

    void reset()
    {
        rc_vector_zeros(&input_vector, NavState2D::getStateCount());
        rc_vector_zeros(&output_vector, NavState2D::getStateCount());
        test_obj->reset();
    }

    string printMatrix(const rc_matrix_t* m, bool sci=false, string sep="\n")
    {
        stringstream out;
        out << fixed << setprecision(5);
        if (sci) out << scientific << setprecision(3);
        out << "[";
        for (int i = 0; i < (m->rows - 1); i++)
        {
            out << "[";
            for (int j = 0; j < (m->cols - 1); j++)
            {
                out << m->d[i][j] << ", ";
            }
            out << m->d[i][m->cols - 1] << "]," << sep;
        }
        out << "[";
        for (int j = 0; j < (m->cols - 1); j++)
        {
            out << m->d[m->rows - 1][j] << ", ";
        }
        out << m->d[m->rows - 1][m->cols - 1] << " ]]";
        return out.str();
    }

    string printVector(const rc_vector_t* v)
    {
        stringstream out;
        out << "[ ";
        for (int i = 0; i < (v->len - 1); i++) out << to_string(v->d[i]) << ", ";
        out << to_string(v->d[v->len - 1]) << " ]";
        return out.str();
    }

    default_random_engine re;
    NavState2D *test_obj;
    rc_matrix_t sensor_matrix;
    rc_vector_t input_vector;
    rc_vector_t output_vector;
};

TEST_F(TickTestFramework, x_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        input_vector.d[state_axis_t::x] = x;
        ASSERT_NO_THROW(test_obj->tick(&input_vector));
        rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], x, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
    }
}

TEST_F(TickTestFramework, y_test)
{
    for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
    {
        input_vector.d[state_axis_t::y] = y;
        ASSERT_NO_THROW(test_obj->tick(&input_vector));
        rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], y, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
    }
}

TEST_F(TickTestFramework, theta_test)
{
    for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
    {
        input_vector.d[state_axis_t::theta] = theta;
        ASSERT_NO_THROW(test_obj->tick(&input_vector));
        rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
    }
}

TEST_F(TickTestFramework, v_test)
{
    for (double v = V_MIN; v < V_MAX; v += V_STEP)
    {
        input_vector.d[state_axis_t::v] = v;
        ASSERT_NO_THROW(test_obj->tick(&input_vector));
        rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], v * STDTS));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], v * STDTS));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], v * STDTS, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
    }
}

TEST_F(TickTestFramework, theta_dot_test)
{
    for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
    {
        input_vector.d[state_axis_t::theta_dot] = theta_dot;
        ASSERT_NO_THROW(test_obj->tick(&input_vector));
        rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta_dot * STDTS));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta_dot * STDTS));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta_dot * STDTS, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], theta_dot, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
    }
}

TEST_F(TickTestFramework, v_dot_test)
{
    for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
    {
        input_vector.d[state_axis_t::v_dot] = v_dot;
        ASSERT_NO_THROW(test_obj->tick(&input_vector));
        rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0.5 * STDTS * STDTS * v_dot));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
        EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0.5 * STDTS * STDTS * v_dot));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
        EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0.5 * STDTS * STDTS * v_dot, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v_dot * STDTS, LINTOL));
        EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], v_dot, LINTOL));
    }
}

TEST_F(TickTestFramework, x_y_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
        {
            input_vector.d[state_axis_t::x] = x;
            input_vector.d[state_axis_t::y] = y;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], x, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], y, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, x_theta_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
        {
            input_vector.d[state_axis_t::x] = x;
            input_vector.d[state_axis_t::theta] = theta;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], x, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, x_v_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double v = V_MIN; v < V_MAX; v += V_STEP)
        {
            input_vector.d[state_axis_t::x] = x;
            input_vector.d[state_axis_t::v] = v;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (v * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (v * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], x + (v * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, x_theta_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
        {
            input_vector.d[state_axis_t::x] = x;
            input_vector.d[state_axis_t::theta_dot] = theta_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], x, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], theta_dot, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, x_v_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
        {
            input_vector.d[state_axis_t::x] = x;
            input_vector.d[state_axis_t::v_dot] = v_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], x + (v_dot * 0.5 * STDTS * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], v_dot, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, y_theta_test)
{
    for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
    {
        for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
        {
            input_vector.d[state_axis_t::y] = y;
            input_vector.d[state_axis_t::theta] = theta;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], y, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, y_v_test)
{
    for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
    {
        for (double v = V_MIN; v < V_MAX; v += V_STEP)
        {
            input_vector.d[state_axis_t::y] = y;
            input_vector.d[state_axis_t::v] = v;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], (v * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], (v * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], (v * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], y, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, y_theta_dot_test)
{
    for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
    {
        for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
        {
            input_vector.d[state_axis_t::y] = y;
            input_vector.d[state_axis_t::theta_dot] = theta_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], y, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], theta_dot, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, y_v_dot_test)
{
    for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
    {
        for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
        {
            input_vector.d[state_axis_t::y] = y;
            input_vector.d[state_axis_t::v_dot] = v_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], (v_dot * 0.5 * STDTS * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], y, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], v_dot, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, theta_v_test)
{
    for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
    {
        for (double v = V_MIN; v < V_MAX; v += V_STEP)
        {
            input_vector.d[state_axis_t::theta] = theta;
            input_vector.d[state_axis_t::v] = v;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], (v * STDTS * cos(theta * DEG2RAD))));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], (v * STDTS * sin(theta * DEG2RAD))));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], (v * STDTS * cos(theta * DEG2RAD))));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], (v * STDTS * sin(theta * DEG2RAD))));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], (v * STDTS * cos(theta * DEG2RAD)), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], (v * STDTS * sin(theta * DEG2RAD)), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, theta_theta_dot_test)
{
    for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
    {
        for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
        {
            input_vector.d[state_axis_t::theta] = theta;
            input_vector.d[state_axis_t::theta_dot] = theta_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta + (theta_dot * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta + (theta_dot * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta + (theta_dot * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], theta_dot, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, theta_v_dot_test)
{
    for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
    {
        for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
        {
            input_vector.d[state_axis_t::theta] = theta;
            input_vector.d[state_axis_t::v_dot] = v_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], (cos(theta * DEG2RAD) * (v_dot * 0.5 * STDTS * STDTS))));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], (sin(theta * DEG2RAD) * (v_dot * 0.5 * STDTS * STDTS))));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], (cos(theta * DEG2RAD) * (v_dot * 0.5 * STDTS * STDTS))));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], (sin(theta * DEG2RAD) * (v_dot * 0.5 * STDTS * STDTS))));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], (cos(theta * DEG2RAD) * (v_dot * 0.5 * STDTS * STDTS)), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], (sin(theta * DEG2RAD) * (v_dot * 0.5 * STDTS * STDTS)), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], v_dot, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, v_theta_dot_test)
{
    for (double v = V_MIN; v < V_MAX; v += V_STEP)
    {
        for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
        {
            input_vector.d[state_axis_t::v] = v;
            input_vector.d[state_axis_t::theta_dot] = theta_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], v * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], (theta_dot * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], v * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], (theta_dot * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], v * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], (theta_dot * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], theta_dot, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], 0, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, v_v_dot_test)
{
    for (double v = V_MIN; v < V_MAX; v += V_STEP)
    {
        for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
        {
            input_vector.d[state_axis_t::v] = v;
            input_vector.d[state_axis_t::v_dot] = v_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], (v * STDTS) + (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v + v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], (v * STDTS) + (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v + v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], (v * STDTS) + (v_dot * 0.5 * STDTS * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v + v_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], v_dot, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, theta_dot_v_dot_test)
{
    for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
    {
        for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
        {
            input_vector.d[state_axis_t::theta_dot] = theta_dot;
            input_vector.d[state_axis_t::v_dot] = v_dot;
            ASSERT_NO_THROW(test_obj->tick(&input_vector));
            rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], (v_dot * 0.5 * STDTS * STDTS)));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
            EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], (v_dot * 0.5 * STDTS * STDTS), LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], 0, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], theta_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], theta_dot, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], v_dot * STDTS, LINTOL));
            EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], v_dot, LINTOL));
        }
    }
}

TEST_F(TickTestFramework, x_y_theta_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
        {
            for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::y] = y;
                input_vector.d[state_axis_t::theta] = theta;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_y_v_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
        {
            for (double v = V_MIN; v < V_MAX; v += V_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::y] = y;
                input_vector.d[state_axis_t::v] = v;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (v * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (v * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_y_theta_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
        {
            for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::y] = y;
                input_vector.d[state_axis_t::theta_dot] = theta_dot;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_y_v_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double y = XY_MIN; y < XY_MAX; y += XY_STEP)
        {
            for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::y] = y;
                input_vector.d[state_axis_t::v_dot] = v_dot;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (0.5 * v_dot * STDTS * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (0.5 * v_dot * STDTS * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], y));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_theta_v_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
        {
            for (double v = V_MIN; v < V_MAX; v += V_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::theta] = theta;
                input_vector.d[state_axis_t::v] = v;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (v * STDTS * cos(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], (v * STDTS * sin(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (v * STDTS * cos(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], (v * STDTS * sin(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_theta_theta_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
        {
            for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::theta] = theta;
                input_vector.d[state_axis_t::theta_dot] = theta_dot;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta + (theta_dot * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta + (theta_dot * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_theta_v_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
        {
            for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::theta] = theta;
                input_vector.d[state_axis_t::v_dot] = v_dot;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (0.5 * v_dot * STDTS * STDTS * cos(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], (0.5 * v_dot * STDTS * STDTS * sin(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (0.5 * v_dot * STDTS * STDTS * cos(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], (0.5 * v_dot * STDTS * STDTS * sin(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_v_theta_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double v = V_MIN; v < V_MAX; v += V_STEP)
        {
            for (double theta_dot = THETA_DOT_MIN; theta_dot < THETA_DOT_MAX; theta_dot += THETA_DOT_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::v] = v;
                input_vector.d[state_axis_t::theta_dot] = theta_dot;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (v * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], (theta_dot * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], theta_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (v * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], (theta_dot * STDTS)));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], theta_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], 0));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

TEST_F(TickTestFramework, x_theta_v_dot_test)
{
    for (double x = XY_MIN; x < XY_MAX; x += XY_STEP)
    {
        for (double theta = THETA_MIN; theta < THETA_MAX; theta += THETA_STEP)
        {
            for (double v_dot = V_DOT_MIN; v_dot < V_DOT_MAX; v_dot += V_DOT_STEP)
            {
                input_vector.d[state_axis_t::x] = x;
                input_vector.d[state_axis_t::theta] = theta;
                input_vector.d[state_axis_t::v_dot] = v_dot;
                ASSERT_NO_THROW(test_obj->tick(&input_vector));
                rc_matrix_times_col_vec(test_obj->getF(), input_vector, &output_vector);
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::x], x + (0.5 * v_dot * STDTS * STDTS * cos(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::y], (0.5 * v_dot * STDTS * STDTS * sin(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v], v_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getXPrediction().d[state_axis_t::v_dot], v_dot));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::x], x + (0.5 * v_dot * STDTS * STDTS * cos(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::y], (0.5 * v_dot * STDTS * STDTS * sin(DEG2RAD * theta))));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta], theta));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::theta_dot], 0));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v], v_dot * STDTS));
                EXPECT_TRUE(equalWithTol(test_obj->getYPrediction().d[state_axis_t::v_dot], v_dot));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::x], test_obj->getXPrediction().d[state_axis_t::x], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::y], test_obj->getXPrediction().d[state_axis_t::y], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta], test_obj->getXPrediction().d[state_axis_t::theta], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::theta_dot], test_obj->getXPrediction().d[state_axis_t::theta_dot], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v], test_obj->getYPrediction().d[state_axis_t::v], LINTOL));
                EXPECT_TRUE(equalWithTol(output_vector.d[state_axis_t::v_dot], test_obj->getYPrediction().d[state_axis_t::v_dot], LINTOL));
            }
        }
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
