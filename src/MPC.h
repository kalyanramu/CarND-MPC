#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

#define LF 2.67

#define ref_v 70
#define DEG25_RAD 0.436332
#define MAX_VAL 1.0e19
#define MAX_ACCEL 1.0
#define NUM_STEPS 10
#define DELTA_TIME 0.05

#define A_WEIGHT 40 //5000 --- vehicle doesn't move much
#define DELTA_WEIGHT 0.0001 //Increasing this made vehicle truen even at high speed properly

#define A_DIFF_WEIGHT 1
#define DELTA_DIFF_WEIGHT 5000

#define VREF_WEIGHT 1
#define CTE_WEIGHT 0.1 //Keeps this tight to center when turning 0.01
#define PSI_WEIGHT 50

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  //MPC predicted X and Y tracjectories
  vector<double> x_vals;
  vector<double> y_vals;
};

#endif /* MPC_H */
