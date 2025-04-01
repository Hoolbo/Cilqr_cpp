// #include <mat.h>
#include <vector>
// #include "mclmcrrt.h"
#include "matplotlibcpp.h"
#include "Eigen/Eigen"
#include "ilqr.h"

void my_plot(const std::vector<std::vector<double>>& global_plan_log,
             const std::vector<std::vector<double>>& ego_log,
             const Trajectory& obs_traj,
             const Solution& solution);

std::vector<std::vector<double>> load_map();    



