#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <cmath>

using CppAD::AD;

size_t N = 10;
double dt = 0.1;
const double Lf = 2.67;     // Lf is the length from front to CoG that has a similar radius
const double cte_ref = 0.0;
const double epsi_ref = 0.0;
const double v_ref = 100;    // increase v_ref value to increase speed of the car
// set up indices
// [x1, x2, ..xN, y1, y2, ..yN, psi1, psi2, ..psiN, v1, v2, ..vN, 
// cte1, cte2, ..cteN, epsi1, epsi2, ...epsiN,
// delta1, delta2, ..deltaN, a1, a2, ..aN]
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


// FG_eval class defines the cost/objective function and the constraints for the IPOPT solver.
// IPOPT solver minimizes the value cost function while adhering to the constraints
class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  // `fg` a vector of the cost constraints
  //`vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars) {

    // weigths for cost function
    double w_cte = 2000;
    double w_epsi = 2000;
    double w_v = 1;
    double w_delta = 10;
    double w_a = 10;
    double w_delta_diff = 100;
    double w_a_diff = 10;

    // set up cost/objective function, FG_eval expects cost function in fg[0]
    fg[0] = 0.0;
    // penalize cross track error, orientation erro and velocity error
    for(uint32_t t = 0; t < N; ++t) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + t] - cte_ref, 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t] - epsi_ref, 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + t] - v_ref, 2);
    }
    // penalize actuations
    for(uint32_t t = 0; t < N-1; ++t) {
      fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }
    // penalize change in actuations
    for(uint32_t t = 0; t < N-2; ++t) {
      fg[0] += w_delta_diff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_a_diff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // set up constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    for(uint32_t t = 1; t < N; ++t) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t -1];
      // f(x) = c0 + c1*x + c2*x*x + c3*x*x*x;
      AD<double> f_x = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      // f'(x) = c1 + 2*c2*x + 3*c3*x*x
      AD<double> f_x_dash = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - (v0 * delta0 * dt)/Lf);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (f_x - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] =  epsi1 - (psi0 - f_x_dash - (v0 * delta0 * dt)/Lf); 
    }
  }
};


//
// MPC class
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  size_t n_vars = N * 6 + (N-1) * 2;  // number of variables
  size_t n_constraints = N * 6;       // number of constraints

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (uint32_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // set initial value of the variables
  // vars[x_start] = x;
  // vars[y_start] = y;
  // vars[psi_start] = psi;
  // vars[v_start] = v;
  // vars[cte_start] = cte;
  // vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // set lower and upper limits for variables.
  for(uint32_t i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // set lower and upper limits for delta
  double steer_limit_deg = 25;
  for(uint32_t i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -steer_limit_deg / 180.0 * M_PI;
    vars_upperbound[i] = steer_limit_deg / 180.0 * M_PI;
  }

  // set lower and upper limit for acceleration
  for(uint32_t i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // set lower and upper limit for constraints
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint32_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  
  // return solution  
  std::vector<double> result;

  // add steering angle and acceleration to result
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  // add all x and y values to plot predicted path in the simulator
  for(uint32_t i = 0; i < N; ++i) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}
