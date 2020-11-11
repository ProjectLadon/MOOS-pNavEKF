/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: NavEKF.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "NavEKF.h"
#include "NavEKF_increment.h"

using namespace std;

//---------------------------------------------------------
// Constructor

NavEKF::NavEKF()
{
    output_vars.resize(NavState2D::getStateCount(), "");
    // Give the output variables default names
    output_vars[state_axis_t::x] = "EKF_X";
    output_vars[state_axis_t::y] = "EKF_Y";
    output_vars[state_axis_t::theta] = "EKF_THETA";
    output_vars[state_axis_t::v] = "EKF_V";
    output_vars[state_axis_t::theta_dot] = "EKF_THETA_DOT";
    output_vars[state_axis_t::v_dot] = "EKF_V_DOT";
    nav_state = nullptr;
}

//---------------------------------------------------------
// Destructor

NavEKF::~NavEKF()
{
    // Free allocated stuff (kalman filter freed on OnDisconnectFromServer)
    rc_vector_free(&sensor_inputs);
    rc_matrix_free(&sensor_estimation_matrix);
    if (nav_state) delete nav_state;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool NavEKF::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);

    MOOSMSG_LIST::iterator p;
    for(p=NewMail.begin(); p!=NewMail.end(); p++)
    {
        CMOOSMsg &msg = *p;
        string key    = msg.GetKey();
        for (int i = 0; i < input_vars.size(); i++)
        {
            // search our inputs for the supplied message name and slot
            // the received value into the appropriate element of the
            // sensor input vector.
            if ((key == input_vars[i]) && msg.IsDouble())
            {
                sensor_inputs.d[i] = msg.GetDouble();
            }
            else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
                reportRunWarning("Unhandled Mail: " + key);
        }
    }

    return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool NavEKF::OnConnectToServer()
{
    rc_matrix_t meas_noise_m;
    rc_matrix_t proc_noise_m;
    rc_matrix_t Pi;
    // Our assumption here is that the process and measurement covariance
    // matrices are both equal to lambda * I, where I is the identity matrix
    // of the correct size and lambda is any real number and is provided
    // by the configuration file..
    rc_matrix_identity(&meas_noise_m, sensor_estimation_matrix.rows);
    rc_matrix_identity(&proc_noise_m, NavState2D::getStateCount());
    rc_matrix_times_scalar(&meas_noise_m, meas_noise);
    rc_matrix_times_scalar(&proc_noise_m, proc_noise);
    // Our initial noise estimate is just the identity matrix.
    rc_matrix_identity(&Pi, NavState2D::getStateCount());
    rc_kalman_alloc_ekf(&kf, proc_noise_m, meas_noise_m, Pi);
    // These matrices have no further purpose after initializing the EKF
    rc_matrix_free(&proc_noise_m);
    rc_matrix_free(&meas_noise_m);
    rc_matrix_free(&Pi);
    registerVariables();
    return(true);
}

//---------------------------------------------------------
// Procedure: OnDisconnectFromServer

bool NavEKF::OnDisconnectFromServer()
{
    rc_kalman_free(&kf);    // If we disconnect and reconnect we need to re-init anyways.
    return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool NavEKF::Iterate()
{
    AppCastingMOOSApp::Iterate();
    if (!nav_state) return false; // This could a nullptr if initialization failed, so avoid the crash.
    nav_state->tick(&(kf.x_est)); // Run the state incrementer
    // update the Kalman filter
    rc_kalman_update_ekf(&kf, nav_state->getF(), nav_state->getH(),
        nav_state->getXPrediction(), sensor_inputs, nav_state->getYPrediction());
    // Publish our outputs.
    for (int i = 0; i < NavState2D::getStateCount(); i++)
    {
        Notify(output_vars[i], kf.x_est.d[i]);
    }
    AppCastingMOOSApp::PostReport();
    return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool NavEKF::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();

    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
        reportConfigWarning("No config block found for " + GetAppName());

    STRING_LIST::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++)
    {
        string orig  = *p;
        string line  = *p;
        string param = toupper(biteStringX(line, '='));
        string value = line;

        bool handled = false;
        if (param == "INPUT")
        {
            input_vars.push_back(value);
            handled = true;
        }
        else if (param == "INPUT_TYPE")
        {
            string val = toupper(value);
            int8_t type_len = input_types.size();
            if (value == "X")            input_types.push_back(state_axis_t::x);
            else if (value == "Y")       input_types.push_back(state_axis_t::y);
            else if (value == "THETA")   input_types.push_back(state_axis_t::theta);
            else if (value == "V")       input_types.push_back(state_axis_t::v);
            else if (value == "THETA_DOT") input_types.push_back(state_axis_t::theta_dot);
            else if (value == "V_DOT")   input_types.push_back(state_axis_t::v_dot);
            // This is only true if the input type was a valid one.
            if (input_types.size() > type_len) handled = true;
        }
        else if (param == "PROCESS_NOISE")
        {
            proc_noise = stof(value);
            handled = true;
        }
        else if (param == "MEASUREMENT_NOISE")
        {
            meas_noise = stof(value);
            handled = true;
        }
        else if (param == "X_OUT")
        {
            output_vars[state_axis_t::x] = value;
        }
        else if (param == "Y_OUT")
        {
            output_vars[state_axis_t::y] = value;
        }
        else if (param == "THETA_OUT")
        {
            output_vars[state_axis_t::theta] = value;
        }
        else if (param == "V_OUT")
        {
            output_vars[state_axis_t::v] = value;
        }
        else if (param == "THETA_DOT_OUT")
        {
            output_vars[state_axis_t::theta_dot] = value;
        }
        else if (param == "V_DOT_OUT")
        {
            output_vars[state_axis_t::v_dot] = value;
        }

        if(!handled) reportUnhandledConfigWarning(orig);
    }

    // If the sensor matrix doesn't populate, nothing else will work, so bail.
    if (!buildSensorMatrix()) return false;
    // Initialize the state object
    nav_state = new NavState2D(sensor_estimation_matrix, (1/GetAppFreq()));
    registerVariables();
    return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void NavEKF::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();
    for (auto &var : input_vars)
    {
        Register(var, 0);
    }
}

//---------------------------------------------------------
// Procedure: registerVariables

bool NavEKF::buildSensorMatrix()
{
    // The assumption here is that all sensor inputs represent
    // exactly one state and are in the same units with no offsets.
    // Therefore, each row of the sensor matrix H is assumed to have
    // a single 1 and state_count - 1 zeros.
    if (input_vars.size() != input_types.size()) return false;
    rc_matrix_zeros(&sensor_estimation_matrix, input_vars.size(),
        NavState2D::getStateCount());
    for (int i = 0; i < input_vars.size(); i++)
    {
        sensor_estimation_matrix.d[i][input_types[i]] = 1;
    }
    return true;
}


//------------------------------------------------------------
// Procedure: buildReport()

bool NavEKF::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "File:                                        \n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}
