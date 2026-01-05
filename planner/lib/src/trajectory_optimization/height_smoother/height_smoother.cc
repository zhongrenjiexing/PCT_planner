#include "trajectory_optimization/height_smoother/height_smoother.h"

#include <iostream>

#include "common/smoothing/osqp_spline1d_solver.h"

Eigen::VectorXd HeightSmoother::Smooth(const Eigen::VectorXd& coarse_height,
                                       const Eigen::VectorXd& upper_bound,
                                       const double dt, const int N,
                                       const double knot_interval) {
  std::vector<double> lbs;
  std::vector<double> ubs;
  std::vector<double> refs;
  std::vector<double> ts;
  std::vector<double> knots;

  for (int i = 0; i < N; ++i) {
    knots.emplace_back(i * knot_interval);
  }

  for (int i = 0; i < coarse_height.size(); ++i) {
    // 使用A*路径高度减去一个安全余量作为下界，而不是固定-1.0
    // 这样可以防止轨迹过度偏离地面
    lbs.emplace_back(coarse_height(i) - 0.8);  // 允许最多下降0.8m
    ubs.emplace_back(std::max(coarse_height(i) - 0.8, upper_bound(i) - 0.3));
    refs.emplace_back(coarse_height(i));
    ts.emplace_back(i * dt);
  }

  common::OsqpSpline1dSolver solver(knots, 5);
  auto kernel = solver.mutable_kernel();
  kernel->AddRegularization(1e-5);
  // kernel->AddSecondOrderDerivativeMatrix(5);
  kernel->AddThirdOrderDerivativeMatrix(30);
  // 关键修改：大幅增加参考线权重，从1提高到100
  // 这样优化器会更紧密跟随A*路径，尤其是在楼梯等大高度变化区域
  kernel->AddReferenceLineKernelMatrix(ts, refs, 1); // pervious:1 / sonnet: 100
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint();
  constraint->AddPointConstraint(ts.front(), coarse_height(0));
  // constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]);
  // constraint->AddPointSecondDerivativeConstraint(t_knots_.front(),
  // init_s_[2]);
  constraint->AddBoundary(ts, lbs, ubs);
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  if (!solver.Solve()) {
    // std::cout << "Fail to solve the spline" << std::endl;
    return coarse_height;
  }

  auto spline = solver.spline();
  Eigen::VectorXd smooth_height(coarse_height.size());
  for (int i = 0; i < coarse_height.size(); ++i) {
    smooth_height(i) = spline(ts[i]);
  }
  return smooth_height;
}
