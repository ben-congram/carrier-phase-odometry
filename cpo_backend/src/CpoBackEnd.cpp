#include <TdcpErrorEval.hpp>
#include <UnicycleErrorEval.hpp>

#include <cpo_backend/CpoBackEnd.hpp>

using TransformStateVar = steam::se3::TransformStateVar;
using TransformStateEvaluator = steam::se3::TransformStateEvaluator;
using TransformEvaluator = steam::se3::TransformEvaluator;
using SteamTrajVar = steam::se3::SteamTrajVar;
using VectorSpaceStateVar = steam::VectorSpaceStateVar;
using PositionEvaluator = steam::se3::PositionEvaluator;

CpoBackEnd::CpoBackEnd() : Node("cpo_back_end") {

  subscription_ = this->create_subscription<cpo_interfaces::msg::TDCP>(
      "tdcp", 10, std::bind(&CpoBackEnd::_tdcpCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("cpo_odometry", 10);

  // set up receiver-vehicle transform. todo: hard-coded for now but eventually make this configurable
  Eigen::Matrix4d T_gps_vehicle_eigen = Eigen::Matrix4d::Identity();
  T_gps_vehicle_eigen(0, 3) = -0.60;
  T_gps_vehicle_eigen(2, 3) = -0.52;
  auto T_gps_vehicle = lgmath::se3::TransformationWithCovariance(T_gps_vehicle_eigen);
  T_gps_vehicle.setZeroCovariance();
  tf_gps_vehicle_ = steam::se3::FixedTransformEvaluator::MakeShared(T_gps_vehicle);

  getParams();
}

void CpoBackEnd::_tdcpCallback(const cpo_interfaces::msg::TDCP::SharedPtr msg_in) {

  uint n = msg_in->pairs.size();
  std::cout << "Found " << n << " sat pairs." << std::endl;

  addMsgToWindow(msg_in);

  // don't attempt optimization if we don't have a full window
  if (msgs_.size() < window_size_)
    return;

  if (n >= 4) {
    resetProblem();

    // set up steam problem

    Eigen::Matrix<double, 6, 1> plausible_vel;       // temporary way to initialize velocity state variable
    plausible_vel << -0.9, 0.0, 0.0, 0.0, 0.0, 0.0;

    // setup state variables using initial condition
    std::vector<SteamTrajVar> traj_states;
    std::vector<TransformStateVar::Ptr> statevars;

    // keep track of states composed with frame transform for use in TDCP terms
    std::vector<TransformEvaluator::ConstPtr> vehicle_poses;
    std::vector<TransformEvaluator::ConstPtr> enu_poses;

    // set up T_0g state
    TransformStateVar::Ptr T_0g_statevar(new TransformStateVar(init_pose_));
    TransformEvaluator::ConstPtr T_0g = TransformStateEvaluator::MakeShared(T_0g_statevar);

    { // first pose in window gets locked
      TransformStateVar::Ptr temp_statevar_a(new TransformStateVar(lgmath::se3::Transformation()));
      TransformStateEvaluator::Ptr temp_pose_a = TransformStateEvaluator::MakeShared(temp_statevar_a);
      VectorSpaceStateVar::Ptr temp_velocity_a = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      temp_statevar_a->setLock(true);   // lock the first pose (but not the first velocity)
      SteamTrajVar temp_a(steam::Time((int64_t)msgs_.front().first.t_a), temp_pose_a, temp_velocity_a);
      statevars.push_back(temp_statevar_a);
      traj_states.push_back(temp_a);

      vehicle_poses.emplace_back(temp_pose_a);
      enu_poses.emplace_back(steam::se3::compose(temp_pose_a, T_0g));
    }

    // loop over window to add states for other poses
    for (uint i = 0; i < msgs_.size(); ++i) {
      Eigen::Matrix<double, 6, 1> plaus_Tba_vec;       // temporary way to initialize pose state variable
      plaus_Tba_vec << -1.0 * (i + 1), 0.0, 0.0, 0.0, 0.0, 0.0;

      TransformStateVar::Ptr temp_statevar(new TransformStateVar(lgmath::se3::Transformation(plaus_Tba_vec)));  // todo: replace with msgs_[i].second
      TransformStateEvaluator::Ptr temp_pose = TransformStateEvaluator::MakeShared(temp_statevar);
      VectorSpaceStateVar::Ptr temp_velocity = VectorSpaceStateVar::Ptr(new VectorSpaceStateVar(plausible_vel));
      SteamTrajVar temp(steam::Time((int64_t)msgs_[i].first.t_b), temp_pose, temp_velocity);
      statevars.push_back(temp_statevar);
      traj_states.push_back(temp);

      vehicle_poses.emplace_back(temp_pose);                                        // T_k0
      enu_poses.emplace_back(steam::se3::compose(temp_pose, T_0g));   // T_kg = T_k0 * T_0g
    }

    // add TDCP terms
    for (uint k = 0; k < msgs_.size(); ++k) {

      TransformEvaluator::ConstPtr T_k1k = steam::se3::compose(vehicle_poses[k + 1], steam::se3::inverse(vehicle_poses[k]));
      PositionEvaluator::ConstPtr r_ba_ina(new PositionEvaluator(T_k1k));

      // using constant covariance here for now
      steam::BaseNoiseModel<1>::Ptr tdcp_noise_model(new steam::StaticNoiseModel<1>(tdcp_cov_));

      // iterate through satellite pairs in msg and add TDCP costs
      for (const auto &pair : msgs_[k].first.pairs) {
        Eigen::Vector3d r_1a_ing_ata{pair.r_1a_a.x, pair.r_1a_a.y, pair.r_1a_a.z};
        Eigen::Vector3d r_1a_ing_atb{pair.r_1a_b.x, pair.r_1a_b.y, pair.r_1a_b.z};
        Eigen::Vector3d r_2a_ing_ata{pair.r_2a_a.x, pair.r_2a_a.y, pair.r_2a_a.z};
        Eigen::Vector3d r_2a_ing_atb{pair.r_2a_b.x, pair.r_2a_b.y, pair.r_2a_b.z};

        steam::TdcpErrorEval::Ptr tdcp_error(new steam::TdcpErrorEval(pair.phi_measured,
                                                                      r_ba_ina,
                                                                      enu_poses[k],   // T_kg
                                                                      r_1a_ing_ata,
                                                                      r_1a_ing_atb,
                                                                      r_2a_ing_ata,
                                                                      r_2a_ing_atb));
        auto tdcp_factor = steam::WeightedLeastSqCostTerm<1, 6>::Ptr(new steam::WeightedLeastSqCostTerm<1, 6>(
            tdcp_error,
            tdcp_noise_model,
            tdcp_loss_function_));
        tdcp_cost_terms_->add(tdcp_factor);
      }
    }

    // add prior on initial pose to deal with roll uncertainty and constrain r^0g_g to zero
    steam::BaseNoiseModel<6>::Ptr
        sharedNoiseModel(new steam::StaticNoiseModel<6>(pose_prior_cov_));
    steam::TransformErrorEval::Ptr prior_error_func(new steam::TransformErrorEval(init_pose_, T_0g));
    pose_prior_cost_ = steam::WeightedLeastSqCostTerm<6, 6>::Ptr(new steam::WeightedLeastSqCostTerm<6, 6>(
        prior_error_func,
        sharedNoiseModel,
        pp_loss_function_));

    steam::BaseNoiseModel<4>::Ptr nonholonomic_noise_model(new steam::StaticNoiseModel<4>(nonholonomic_cov_));
    steam::se3::SteamTrajInterface traj(smoothing_factor_information_, true);

    // loop through velocity state variables
    for (const auto &traj_state : traj_states) {
      if (!traj_state.getVelocity()->isLocked()) {
        // add nonholonomic costs
        steam::UnicycleErrorEval::Ptr non_holo_error_func(new steam::UnicycleErrorEval(traj_state.getVelocity()));
        auto non_holonomic_factor = steam::WeightedLeastSqCostTerm<4, 6>::Ptr(new steam::WeightedLeastSqCostTerm<4, 6>(
            non_holo_error_func,
            nonholonomic_noise_model,
            nonholonomic_loss_function_));
        nonholonomic_cost_terms_->add(non_holonomic_factor);

        // add smoothing costs
        steam::Time temp_time = traj_state.getTime();
        const TransformEvaluator::Ptr &temp_pose = traj_state.getPose();
        const steam::VectorSpaceStateVar::Ptr &temp_velocity = traj_state.getVelocity();
        traj.add(temp_time, temp_pose, temp_velocity);

        // also add velocity state to problem
        problem_->addStateVariable(traj_state.getVelocity());
      }
    }
    traj.appendPriorCostTerms(smoothing_cost_terms_);

    problem_->addCostTerm(tdcp_cost_terms_);
    problem_->addCostTerm(nonholonomic_cost_terms_);
    problem_->addCostTerm(smoothing_cost_terms_);
    problem_->addCostTerm(pose_prior_cost_);

    problem_->addStateVariable(T_0g_statevar);
    for (const auto &state : statevars) {
      problem_->addStateVariable(state);
    }

    // print initial costs (for debugging/development)
    printCosts();

    // setup solver and optimize
    steam::DoglegGaussNewtonSolver::Params params;
    params.verbose = true;      // todo: make configurable
    params.maxIterations = 3;
    solver_.reset(new steam::DoglegGaussNewtonSolver(problem_.get(), params));
    solver_->optimize();

    // print final costs (for debugging/development)
    printCosts();

    // get optimized transform and publish
    lgmath::se3::Transformation pose = statevars.back()->getValue();
    auto gn_solver = std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(solver_);
    Eigen::Matrix<double, 6, 6> pose_covariance = gn_solver->queryCovariance(statevars.back()->getKey());
    geometry_msgs::msg::PoseWithCovariance pose_msg = toPoseMsg(pose, pose_covariance);
    publisher_->publish(pose_msg);

    std::cout << "r_ba_ina: " << pose.r_ba_ina().transpose() << std::endl;
    std::cout << "T_ba vec: " << pose.vec().transpose() << std::endl;

    for (uint i = 1; i < msgs_.size(); ++i) {
      msgs_[i].second = statevars[i]->getValue() * statevars[i - 1]->getValue().inverse();       // T_21 = T_20 * inv(T_10)
      std::cout << "Set edge " << i << " to (vec) " << msgs_[i].second.vec().transpose() << std::endl;
    }

    // update our orientation estimate
    init_pose_ = T_0g_statevar->getValue();   // todo: may need to update

    std::cout << "init_pose_ vec: " << init_pose_.vec().transpose() << std::endl;

  }

}
void CpoBackEnd::getParams() {
  // todo: eventually want to setup as ROS2 params (this->declare_parameter<...) but for now will hard code

  double tdcp_cov = 0.1;
  double non_holo_y = 0.1;
  double non_holo_z = 0.1;
  double non_holo_roll = 0.1;
  double non_holo_pitch = 0.1;
  double lin_acc_std_dev_x = 1.0;
  double lin_acc_std_dev_y = 0.1;
  double lin_acc_std_dev_z = 0.1;
  double ang_acc_std_dev_x = 0.1;
  double ang_acc_std_dev_y = 0.1;
  double ang_acc_std_dev_z = 0.1;
  double roll_cov_x = 0.001;
  double roll_cov_y = 0.001;
  double roll_cov_z = 0.001;
  double roll_cov_ang1 = 0.01;
  double roll_cov_ang2 = 0.01;
  double roll_cov_ang3 = 1.0;
  uint window_size = 10;

  tdcp_cov_ << tdcp_cov;

  Eigen::Array<double, 1, 4> non_holo_diag;
  non_holo_diag << non_holo_y, non_holo_z, non_holo_roll, non_holo_pitch;
  nonholonomic_cov_.setZero();
  nonholonomic_cov_.diagonal() = non_holo_diag;

  Eigen::Array<double, 1, 6> Qc_diag;
  Qc_diag << lin_acc_std_dev_x, lin_acc_std_dev_y, lin_acc_std_dev_z,
      ang_acc_std_dev_x, ang_acc_std_dev_y, ang_acc_std_dev_z;
  smoothing_factor_information_.setZero();
  smoothing_factor_information_.diagonal() = 1.0 / Qc_diag;

  pose_prior_cov_ = Eigen::Matrix<double, 6, 6>::Identity();
  pose_prior_cov_(0, 0) = roll_cov_x;
  pose_prior_cov_(1, 1) = roll_cov_y;
  pose_prior_cov_(2, 2) = roll_cov_z;
  pose_prior_cov_(3, 3) = roll_cov_ang1;
  pose_prior_cov_(4, 4) = roll_cov_ang2;
  pose_prior_cov_(5, 5) = roll_cov_ang3;

  window_size_ = window_size;
}

void CpoBackEnd::resetProblem() {
  // setup loss functions
  tdcp_loss_function_.reset(new steam::DcsLossFunc(2.0));   // todo: try different loss functions
  nonholonomic_loss_function_.reset(new steam::L2LossFunc());
  pp_loss_function_.reset(new steam::L2LossFunc());

  // setup cost terms
  tdcp_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  nonholonomic_cost_terms_.reset(new steam::ParallelizedCostTermCollection());
  smoothing_cost_terms_.reset(new steam::ParallelizedCostTermCollection());

  // set up the steam problem
  problem_.reset(new steam::OptimizationProblem());
}

void CpoBackEnd::printCosts() {
  std::cout << " === Costs === " << std::endl;
  std::cout << "Carrier Phase:       " << tdcp_cost_terms_->cost() << "        Terms:  "
            << tdcp_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Nonholonomic:        " << nonholonomic_cost_terms_->cost() << "        Terms:  "
            << nonholonomic_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Smoothing:           " << smoothing_cost_terms_->cost() << "        Terms:  "
            << smoothing_cost_terms_->numCostTerms() << std::endl;
  std::cout << "Pose Prior:          " << pose_prior_cost_->cost() << "        Terms:  1" << std::endl;
}

void CpoBackEnd::addMsgToWindow(const cpo_interfaces::msg::TDCP::SharedPtr &msg) {

  if (!msgs_.empty() && msg->t_a != msgs_.back().first.t_b) {
    // times don't align, so we've likely missed a msg. To be safe we will clear it for now
    std::cout << "Warning: mismatched times. Clearing msgs_. Current t_a: " << msg->t_a << ". Previous t_b: "
              << msgs_.back().first.t_b << std::endl;
    std::deque<std::pair<cpo_interfaces::msg::TDCP, lgmath::se3::Transformation>>().swap(msgs_);
  }

  // add latest message
  lgmath::se3::Transformation new_T_estimate = lgmath::se3::Transformation();   // todo - initialize better

  msgs_.emplace_back(*msg, new_T_estimate);

  // if we have a full queue, discard the oldest msg
  while (msgs_.size() > window_size_) {
    msgs_.pop_front();
  }
}

geometry_msgs::msg::PoseWithCovariance CpoBackEnd::toPoseMsg(const lgmath::se3::Transformation &T,
                                                             const Eigen::Matrix<double, 6, 6> &cov) {
  Eigen::Quaterniond q(T.matrix().topLeftCorner<3, 3>());
  Eigen::Vector3d r = T.matrix().topRightCorner<3, 1>();

  geometry_msgs::msg::PoseWithCovariance msg;
  msg.pose.position.set__x(r[0]);
  msg.pose.position.set__y(r[1]);
  msg.pose.position.set__z(r[2]);
  msg.pose.orientation.set__x(q.x());
  msg.pose.orientation.set__y(q.y());
  msg.pose.orientation.set__z(q.z());
  msg.pose.orientation.set__w(q.w());

  std::array<double, 36> temp{};
  Eigen::Matrix<double, 6, 6>::Map(temp.data()) = cov;  // todo: not sure if this cov is valid
  msg.set__covariance(temp);

  return msg;
}
