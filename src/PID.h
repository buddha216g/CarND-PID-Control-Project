#ifndef PID_H
#define PID_H
#include <vector>



class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

    
    /*
     * Twiddle variables
     */
    std::vector<double> p,dp;
    int step, param_index;
    double total_error, best_err;
    bool increased_P, reduced_P;
    
    
    double best_error = 999.9;    
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

    /*
     * Calculate the steer.
     */
    double TotalSteer();
    
  /*
  * Calculate the total PID error.
  */
  double TotalError();
    
    
    /*
     * Convenience function for adding amount (dp) to a PID controller parameter based on index
     */
    void UpdateP(int index, double amount);
};

#endif /* PID_H */
