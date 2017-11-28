#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

//const double tolerance = 0.01;

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = d_error = i_error = prev_cte = 0.0;
    
    // Twiddling parameters
    p = {Kp,Ki,Kd};
    dp = {0.1*Kp,0.1*Ki,0.1*Kd};
    step = 1;
    param_index = 2;
    total_error = 0;
    increased_P = false;
    reduced_P = false;

}

void PID::UpdateError(double cte) {
    
    d_error = cte - prev_cte;
    p_error = cte;
    i_error += cte;
    prev_cte = cte;
            
        if (total_error < best_err) {
            cout << "If better Error, increase dp go to next param  " << param_index << endl;
            best_err = total_error;
            dp[param_index] *= 1.1;

            param_index = (param_index + 1) % 3;
            increased_P = reduced_P = false;
        }
        
        if (!increased_P && !reduced_P) {
            cout << "First time for a parameter - Update P with dp   " << param_index << endl;
            UpdateP(param_index, dp[param_index]);
            increased_P = true;
        }
        else if (increased_P && !reduced_P) {
            cout << "Increased P but still not better Error, reduce P by twice dp" << endl;
            UpdateP(param_index, -2 * dp[param_index]);
            reduced_P = true;
            
        }
        else {
            cout << "Reducing P didnt work, error still high, Revert back to original p, reduce dp, go to next parameter" << endl;
            UpdateP(param_index, dp[param_index]);
            dp[param_index] *= 0.9;
            param_index = (param_index + 1) % 3;
            increased_P = reduced_P = false;
        }
        total_error = 0;
        cout << "Step: " << step << "  new parameters" << endl;
        cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << endl;
        cout << "Dp: " << dp[0] << ", Di: " << dp[1] << ", Dd: " << dp[2] << endl;
        cout << "Pterm: " << -Kp*p_error << ", Iterm: " << -Ki*i_error << ", Dterm: " << -Kd*d_error << endl;
    
    step++;
}

double PID::TotalSteer() {
    return -Kp * p_error
    - Kd * d_error
    - Ki * i_error;
}

double PID::TotalError() {
    return fabs(p_error) + fabs(i_error) + fabs(d_error);
}


void PID::UpdateP(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Ki += amount;
    }
    else if (index == 2) {
        Kd += amount;
    }
    else {
        std::cout << "UpdateP index out of bounds";
    }
}
