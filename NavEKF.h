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

private: // Configuration variable
    vector<string> input_vars;
    vector<state_axis_t> input_types;
    vector<string> output_vars;

private: // State variables
    double proc_noise;
    double meas_noise;
    rc_kalman_t kf;
    rc_vector_t sensor_inputs;
    rc_matrix_t sensor_estimation_matrix;
    NavState2D *nav_state;
};

#endif
