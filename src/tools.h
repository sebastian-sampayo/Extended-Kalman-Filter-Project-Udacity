#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>

#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to convert from radar measurement to model state x 
  * (Polar to Cartesian coordinates).
  * @param radar_measurement Polar coordinates in the format: {ro, phi, ro_dot}
  * @return Cartesian coordinates in the format: {p_x, p_y, v_x, v_y}
  */
  Eigen::VectorXd ConvertRadar2State(const Eigen::VectorXd& radar_measurement);
  
private:
  float init_velocity_scale = 0.2;
};

#endif /* TOOLS_H_ */
