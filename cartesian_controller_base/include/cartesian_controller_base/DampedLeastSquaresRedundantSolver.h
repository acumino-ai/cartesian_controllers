////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    DampedLeastSquaresRedundantSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \author  Gal Gorjup <galg@acumino.ai>
 * \date    2025/07/04
 *
 */
//-----------------------------------------------------------------------------

#ifndef DAMPED_LEAST_SQUARES_REDUNDANT_SOLVER_H_INCLUDED
#define DAMPED_LEAST_SQUARES_REDUNDANT_SOLVER_H_INCLUDED

#include <cartesian_controller_base/IKSolver.h>

#include <kdl/jacobian.hpp>
#include <memory>

#include "rclcpp/node.hpp"

namespace cartesian_controller_base
{
/**
   * \brief A damped least squares IK solver for Cartesian controllers, 
   * with joint centering for redundant manipulators
   *
   *
   *  Implementation is based on the DampedLeastSquaresSolver, with an added term that 
   *  tries to center the joints in their allowed range through null space projection.
   */
class DampedLeastSquaresRedundantSolver : public IKSolver
{
public:
  DampedLeastSquaresRedundantSolver();
  ~DampedLeastSquaresRedundantSolver();

  /**
     * \brief Compute joint target commands with damped least squares
     *
     * \param period The duration in sec for this simulation step
     * \param net_force The applied net force, expressed in the root frame
     *
     * \return A point holding positions, velocities and accelerations of each joint
     */
  trajectory_msgs::msg::JointTrajectoryPoint getJointControlCmds(
    rclcpp::Duration period, const ctrl::Vector6D & net_force) override;

  /**
     * \brief Initialize the solver
     *
     * \param nh A node handle for namespace-local parameter management
     * \param chain The kinematic chain of the robot
     * \param upper_pos_limits Tuple with max positive joint angles
     * \param lower_pos_limits Tuple with max negative joint angles
     *
     * \return True, if everything went well
     */

  bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh, const KDL::Chain & chain,
            const KDL::JntArray & upper_pos_limits,
            const KDL::JntArray & lower_pos_limits) override;

private:
  std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
  KDL::Jacobian m_jnt_jacobian;

  // Dynamic parameters
  const std::string m_params =
    "solver.damped_least_squares_redundant";  ///< namespace for parameter access
  double m_alpha;                             ///< damping coefficient
};

}  // namespace cartesian_controller_base

#endif
