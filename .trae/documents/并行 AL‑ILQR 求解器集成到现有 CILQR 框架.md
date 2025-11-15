## 背景与目标
- 在现有使用障碍函数处理约束的 CILQR 框架中，增量开发一套并行 AL‑ILQR（Augmented Lagrangian + iLQR）求解器，支持非线性状态/控制不等式约束；并在 `main` 中提供两种算法的切换入口。
- 参考 ALTRO 算法思想：外层增广拉格朗日更新乘子与罚因子，内层用 iLQR 求解带约束的拉格朗日子问题，并在必要时进行并行线搜索与时间步并行的导数计算。[1][2][4]

## 现有代码结构梳理
- 求解器与模型：`include/ilqr.h:292` 定义 `CILQRSolver`；状态/控制/轨迹/车辆模型等数据结构均已完备。
- 约束的障碍函数实现与导数：
  - 障碍物 c 与 ∂c 计算在 `src/ilqr.cpp:650-668`（椭圆安全区，c>0 视为违反）。
  - 车道左右边界 c 与 ∂c 在 `src/ilqr.cpp:675-686`。
  - 状态 `gamma` 上下限在 `src/ilqr.cpp:692-700`；控制 `gamma_dot` 上下限在 `src/ilqr.cpp:721-728`。
- CILQR 主流程：
  - 导数计算 `compute_df`/`compute_cost_derivatives`：`src/ilqr.cpp:596-764`。
  - 反向传递与正则化：`src/ilqr.cpp:766-879`。
  - 正向线搜索：`src/ilqr.cpp:881-1023`。
  - 主迭代：`src/ilqr.cpp:160-324`。
- `main` 中求解与绘制：
  - 求解器实例化与循环位置：`src/main.cpp:176-183`、调用求解：`src/main.cpp:189-193`。

## 算法设计（AL‑iLQR）
- 目标函数：`J(x,u)` 保留原状态/控制代价；约束改用增广拉格朗日项替代障碍函数。
- 不等式约束统一记为 `g_k(x_k,u_k) ≤ 0`，定义 `φ(g)=max(0,g)`。增广拉格朗日：
  - `L(x,u;λ,ρ) = J(x,u) + Σ_k Σ_i [ λ_{k,i} φ(g_{k,i}) + ρ/2 φ(g_{k,i})^2 ]`
  - 梯度近似：当 `g>0` 时，`∂L/∂z += (λ + ρ·g)·∂g/∂z`，`∂²L/∂z² += ρ·(∂g ∂gᵀ)`（z 为 x 或 u）。
- 约束映射：复用现有 `c` 与梯度计算（替换障碍函数为 AL 项）：
  - 障碍物安全椭圆：`c_obs = 1 - x'^2/a^2 - y'^2/b^2`、`dc_obs` 见 `src/ilqr.cpp:661-666`。
  - 车道左右：`c_left = l - w_left`、`c_right = -l - w_right`、梯度 `dc_left/right` 见 `src/ilqr.cpp:677-684`。
  - 状态上/下限：`c_γ_max = γ - γ_max`、`c_γ_min = γ_min - γ`、`dc = e4`（第 4 分量），`src/ilqr.cpp:692-700`。
  - 控制上/下限：`c_γ̇_max = γ̇ - γ̇_max`、`c_γ̇_min = γ̇_min - γ̇`、`dc = P2`（第 2 控制），`src/ilqr.cpp:721-728`。
- 外层循环（AL 更新）：
  - 计算最大违反 `c_max = max_k,i φ(g_{k,i})`；若 `c_max ≤ tol_c` 则外层收敛。
  - 乘子更新（保持非负）：`λ_{k,i} ← max(0, λ_{k,i} + ρ·φ(g_{k,i}))`。
  - 罚因子调度：若违反未改善则 `ρ ← min(ρ_max, ρ·γ)`（如 `γ=10`）。
- 内层 iLQR：
  - 用现有 `compute_df`，并将 `compute_cost_derivatives` 替换为 `compute_al_derivatives`，把 AL 项累加到 `lx/lxx/lu/luu/lux`。
  - 反向与线搜索流程复用现有实现。

## 并行化设计
- 时间步并行导数：`compute_al_derivatives` 按 k 独立，采用并行 for：优先 OpenMP（可选编译器支持），否则使用 `std::async` 或并行算法拆分时间步批次。
- 并行线搜索：同时评估多组 `alpha` 候选（如 `1.0,0.5,0.25,0.125`）的前向回滚与代价，择优更新；若并行不可用保持串行退化。
- 线程安全：每个候选写入独立 `Solution` 缓冲，避免数据竞争。

## 代码改动与文件布局
- 新增 `ALILQRSolver` 类（与 `CILQRSolver` 对齐的接口）：
  - 成员：`lambda_obs/lane/gamma/gamma_dot`（按时间步向量）、`rho`、`tol_c`、`max_outer_iter`、日志统计。
  - 方法：`solve`（外层 AL 循环）、`compute_al_derivatives`、共享 `backward`/`forward`。
- 代码组织：
  - 复用数据结构：`Solution/Vehicle/SystemModel/Trajectory/ControlSequence`（无需改动）。
  - 为减少大范围重构，`ALILQRSolver` 内部复制并最小改动 `backward/forward` 实现，接口与返回值完全一致。
- 文件与构建：
  - 建议新增 `src/al_ilqr.cpp`（实现）与在 `include/ilqr.h` 末尾声明 `class ALILQRSolver`；`CMakeLists.txt:47-53` 将新源加入 `MAIN_SOURCES`。

## main 集成与切换
- 命令行参数解析添加 `--solver [cilqr|al]`，默认 `cilqr`。
- 当 `al`：实例化 `ALILQRSolver`，并在外层 `ITER` 循环中调用其 `solve`；其余日志与可视化复用现有接口。
- 具体改动位置：构造与调用处 `src/main.cpp:176-183`、`src/main.cpp:189-193` 增加分支与选择逻辑；打印 CPU 时间保持一致（参考 `src/main.cpp:193`）。

## 关键参数与接口
- 新增 AL 选项（可放入 `Arg` 或 AL 专用结构）：`penalty_initial`（如 10）、`penalty_scaling`（如 10）、`penalty_max`、`constraint_tolerance`（如 1e‑3）、`max_outer_iter`（如 10）。
- 在 AL 模式下，关闭障碍函数代价 `arg.if_cal_* = false`，避免双重计数；约束仅由 AL 项处理。

## 验证与日志
- 统计项：总迭代、内/外层迭代、代价变化 `dJ`、最大约束违反 `c_max`、平均梯度、最终 `rho` 与 `lambda` 范围（参考 [1] 的统计集合）。
- 脚本验证：使用默认地图 `B201` 与命令行切换，观察 `c_max` 降至 `≤ tol_c` 且轨迹无碰撞；绘制与 CILQR 一致。
- 性能对比：记录 `CPU time` 与迭代次数，评估并行线搜索带来的加速。

## 风险与回退
- 数值稳定：当 `Quu` 近奇异时继续沿用现有 LLT→SVD 回退路径（见 `src/ilqr.cpp:818-840`）。
- 若 AL 外层收敛缓慢：限制 `rho` 增长，维持较小步长，并在内层放宽中间收敛阈值（参考 [1] 的 `cost_tolerance_intermediate` 思路）。
- 暂不实现 ALTRO 的“投影牛顿”精修阶段；若后续需要，可在 AL 收敛后追加活跃集投影精修。

## 交付内容汇总
- 新增并行 AL‑ILQR 求解器类与实现；导数计算复用现有约束 c/∂c；外层 AL 更新逻辑完整。
- `main` 支持 `CILQR` 与 `AL‑ILQR` 切换；默认保持现状，命令行可选。
- 完整日志与可视化保持一致，约束违反/代价序列可输出到 CSV。

[1] Altro.jl 说明与选项：https://github.com/RoboticExplorationLab/Altro.jl 
[2] ALTRO 论文（IROS）：https://bjack205.github.io/assets/ALTRO.pdf 
[4] Altro 文档（算法综述）：https://docs.juliahub.com/Altro/GnnwV/0.4.0/