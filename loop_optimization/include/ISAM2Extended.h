#ifndef ISAM2_EXTENDED_H
#define ISAM2_EXTENDED_H

#include <gtsam/nonlinear/ISAM2.h>
#include <memory>

namespace gtsam {

/**
 * Extended ISAM2 class with backup and recover functionality
 * for False Positive Rejection (FPR) in LTA-OM
 */
class ISAM2Extended : public ISAM2 {
public:
  using ISAM2::ISAM2;  // Inherit constructors

  /**
   * Backup the current state of ISAM2
   * Used for False Positive Rejection - saves state before adding potential loop closure
   */
  void backup() {
    theta_bkq_ = theta_;
    variableIndex_bkq_ = variableIndex_;
    delta_bkq_ = delta_;
    deltaNewton_bkq_ = deltaNewton_;
    RgProd_bkq_ = RgProd_;
    deltaReplacedMask_bkq_ = deltaReplacedMask_;
    nonlinearFactors_bkq_ = nonlinearFactors_;
    linearFactors_bkq_ = linearFactors_;
    doglegDelta_bkq_ = doglegDelta_;
    fixedVariables_bkq_ = fixedVariables_;
    update_count_bkq_ = update_count_;
    params_bkq_ = params_;

    // Backup BayesTree state (not in original patch but added for completeness)
    nodes_bkq_ = this->nodes_;
    roots_bkq_ = this->roots_;
  }

  /**
   * Recover the backed up state of ISAM2
   * Used when a loop closure is determined to be a false positive
   */
  void recover() {
    theta_ = theta_bkq_;
    variableIndex_ = variableIndex_bkq_;
    delta_ = delta_bkq_;
    deltaNewton_ = deltaNewton_bkq_;
    RgProd_ = RgProd_bkq_;
    deltaReplacedMask_ = deltaReplacedMask_bkq_;
    nonlinearFactors_ = nonlinearFactors_bkq_;
    linearFactors_ = linearFactors_bkq_;
    doglegDelta_ = doglegDelta_bkq_;
    fixedVariables_ = fixedVariables_bkq_;
    update_count_ = update_count_bkq_;
    params_ = params_bkq_;

    // Recover BayesTree state (not in original patch but added for completeness)
    this->nodes_ = nodes_bkq_;
    this->roots_ = roots_bkq_;
  }

private:
  // Backup storage variables
  Values theta_bkq_;
  VariableIndex variableIndex_bkq_;
  VectorValues delta_bkq_;
  VectorValues deltaNewton_bkq_;
  VectorValues RgProd_bkq_;
  KeySet deltaReplacedMask_bkq_;
  NonlinearFactorGraph nonlinearFactors_bkq_;
  GaussianFactorGraph linearFactors_bkq_;
  boost::optional<double> doglegDelta_bkq_;
  KeySet fixedVariables_bkq_;
  int update_count_bkq_;
  ISAM2Params params_bkq_;

  // BayesTree backup (not in original patch but added for completeness)
  typename Base::Nodes nodes_bkq_;
  typename Base::Roots roots_bkq_;
};

}  // namespace gtsam

#endif  // ISAM2_EXTENDED_H
