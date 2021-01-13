/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: NavEKF.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef NavEKF_HEADER
#define NavEKF_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NavEKF_increment.h"
#include <vector>
#include <string>

using namespace std;

class NavEKF : public AppCastingMOOSApp
{
public:
    NavEKF();
    ~NavEKF();

protected: // Standard MOOSApp functions to overload
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnDisconnectFromServer();
    bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
    bool buildReport();

protected:
    void registerVariables();
    bool buildSensorMatrix();
    string printMatrix(const rc_matrix_t* m, bool sci=false, string sep="\n");
    string printVector(const rc_vector_t* v);
    void debug_ekf_update(rc_kalman_t* kf, rc_matrix_t F, rc_matrix_t H, rc_vector_t x_pre, rc_vector_t y, rc_vector_t h);

private: // Configuration variable
    vector<string> input_vars;
    vector<state_axis_t> input_types;
    vector<string> output_vars;
    string p_matrix_var;

private: // State variables
    double proc_noise;
    double meas_noise;
    rc_kalman_t kf;
    rc_vector_t sensor_inputs;
    rc_matrix_t sensor_estimation_matrix;
    NavState2D *nav_state;
    uint64_t data_received;
    bool data_good;
    bool server_connected;
    bool debug_enabled;
};

#endif
