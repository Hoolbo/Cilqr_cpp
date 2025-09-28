#include "utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <array>
#include <memory>
#include <random>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <array>
#include <memory>
// 保留原有的matplotlib支持（仅在需要时）
#include "matplotlibcpp.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include <map>
#include <queue>
#include <unordered_map>
#include <limits>

#include "hybrid_astar.h"

// ---- Grid A* fallback helpers to avoid obstacle penetration ----
struct AStarNode {
    int x; int y; double g; double f; int px; int py;
};

static inline double astar_heuristic(int x, int y, int gx, int gy){
    return std::hypot((double)(x-gx), (double)(y-gy));
}

static bool astar_search(const OccupancyGrid& grid,
                         const Eigen::Vector2d& start_w,
                         const Eigen::Vector2d& goal_w,
                         std::vector<Eigen::Vector2d>& out_path){
    auto to_grid = [&](const Eigen::Vector2d& p){
        int x = (int)std::floor((p.x() - grid.origin_x) / grid.resolution);
        int y = (int)std::floor((p.y() - grid.origin_y) / grid.resolution);
        return std::pair<int,int>(x,y);
    };
    auto to_world = [&](int x, int y){
        double wx = grid.origin_x + (x + 0.5) * grid.resolution;
        double wy = grid.origin_y + (y + 0.5) * grid.resolution;
        return Eigen::Vector2d(wx, wy);
    };
    auto [sx, sy] = to_grid(start_w);
    auto [gx, gy] = to_grid(goal_w);
    auto idx = [&](int x, int y){ return (size_t)y * grid.width + (size_t)x; };
    auto in_bounds = [&](int x, int y){ return x>=0 && x<grid.width && y>=0 && y<grid.height; };
    if (!in_bounds(sx,sy) || !in_bounds(gx,gy)) return false;
    if (grid.cells[idx(sx,sy)]==1 || grid.cells[idx(gx,gy)]==1) return false;

    struct QNode { double f; int x; int y; };
    struct Cmp { bool operator()(const QNode& a, const QNode& b) const { return a.f > b.f; } };
    std::priority_queue<QNode, std::vector<QNode>, Cmp> open;
    std::vector<double> gscore(grid.width * grid.height, std::numeric_limits<double>::infinity());
    std::vector<int> parent(grid.width * grid.height, -1);

    gscore[idx(sx,sy)] = 0.0;
    open.push({astar_heuristic(sx,sy,gx,gy), sx, sy});

    const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const double dcost[8] = {1,1,1,1,std::sqrt(2),std::sqrt(2),std::sqrt(2),std::sqrt(2)};

    while(!open.empty()){
        auto cur = open.top(); open.pop();
        int cx = cur.x, cy = cur.y;
        if (cx == gx && cy == gy){
            std::vector<Eigen::Vector2d> path;
            int x = gx, y = gy;
            while(!(x==sx && y==sy)){
                path.push_back(to_world(x,y));
                int pid = parent[idx(x,y)];
                if (pid < 0) break;
                int px = pid % grid.width; int py = pid / grid.width;
                x = px; y = py;
            }
            path.push_back(to_world(sx,sy));
            std::reverse(path.begin(), path.end());
            out_path.swap(path);
            return true;
        }
        for(int k=0;k<8;k++){
            int nx = cx + dirs[k][0];
            int ny = cy + dirs[k][1];
            if (!in_bounds(nx,ny)) continue;
            if (grid.cells[idx(nx,ny)]==1) continue;
            // prevent cutting corners when diagonal
            if (dirs[k][0]!=0 && dirs[k][1]!=0){
                int ax = cx + dirs[k][0]; int ay = cy;
                int bx = cx; int by = cy + dirs[k][1];
                if (!in_bounds(ax,ay) || !in_bounds(bx,by)) continue;
                if (grid.cells[idx(ax,ay)]==1 || grid.cells[idx(bx,by)]==1) continue;
            }
            double tentative = gscore[idx(cx,cy)] + dcost[k];
            if (tentative < gscore[idx(nx,ny)]){
                gscore[idx(nx,ny)] = tentative;
                parent[idx(nx,ny)] = cy * grid.width + cx;
                double f = tentative + astar_heuristic(nx,ny,gx,gy);
                open.push({f,nx,ny});
            }
        }
    }
    return false;
}

// 路径平滑函数（增强版）：
// 1) 随机捷径（shortcut）去除之字形与多余折点（保持无碰撞与缩短长度）
// 2) 拉普拉斯式渐进平滑（带碰撞约束）
// 3) 按弧长统一重采样，确保点间距一致有利于车辆跟踪
std::vector<Eigen::Vector2d> smooth_path(const std::vector<Eigen::Vector2d>& path,
                                        const OccupancyGrid& grid,
                                        int iterations) {
    if (path.size() <= 2) return path;

    std::vector<Eigen::Vector2d> pts = path;

    // 1) 随机捷径（shortcut）
    {
        int attempts = std::min<int>(200, std::max(20, iterations * 40));
        if ((int)pts.size() > 3 && attempts > 0) {
            std::random_device rd;
            std::mt19937 rng(rd());
            for (int a = 0; a < attempts; ++a) {
                int n = (int)pts.size();
                if (n <= 3) break;
                // 使用当前尺寸构造分布，避免越界
                std::uniform_int_distribution<int> uid(0, n - 1);
                int i = uid(rng);
                int j = uid(rng);
                if (i > j) std::swap(i, j);
                if (j - i < 2) continue; // 至少隔一个点
                if (is_collision_free_segment(grid, pts[i], pts[j])) {
                    double old_len = 0.0;
                    for (int k = i; k < j; ++k) old_len += (pts[k + 1] - pts[k]).norm();
                    double new_len = (pts[j] - pts[i]).norm();
                    if (new_len + 1e-6 < old_len) {
                        // 删除中间点，直接连边
                        pts.erase(pts.begin() + i + 1, pts.begin() + j);
                    }
                }
            }
        }
    }

    // 2) 渐进式平滑（带碰撞约束）
    {
        int smooth_iters = std::max(1, iterations);
        double alpha = 0.45; // 平滑强度（0-1）
        for (int it = 0; it < smooth_iters; ++it) {
            if (pts.size() <= 2) break;
            std::vector<Eigen::Vector2d> tmp = pts;
            for (size_t k = 1; k + 1 < pts.size(); ++k) {
                Eigen::Vector2d candidate = 0.5 * (pts[k - 1] + pts[k + 1]);
                Eigen::Vector2d newpos = (1.0 - alpha) * pts[k] + alpha * candidate;
                // 计算占用网格的最近障碍物方向（局部斥力），鼓励远离障碍物边界
                int gx = static_cast<int>(std::floor((newpos.x() - grid.origin_x) / grid.resolution));
                int gy = static_cast<int>(std::floor((newpos.y() - grid.origin_y) / grid.resolution));
                Eigen::Vector2d repel(0.0, 0.0);
                double repel_gain = 0.2;
                int R = 5; // 搜索邻域半径（格点）
                for (int dy = -R; dy <= R; ++dy) {
                    for (int dx = -R; dx <= R; ++dx) {
                        int nx = gx + dx;
                        int ny = gy + dy;
                        if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height) {
                            size_t nidx = static_cast<size_t>(ny) * grid.width + static_cast<size_t>(nx);
                            if (grid.cells[nidx] == 1) {
                                double wx = grid.origin_x + (nx + 0.5) * grid.resolution;
                                double wy = grid.origin_y + (ny + 0.5) * grid.resolution;
                                Eigen::Vector2d d = newpos - Eigen::Vector2d(wx, wy);
                                double dist = std::max(d.norm(), 1e-6);
                                repel += d / (dist * dist); // 1/r^2 斥力
                            }
                        }
                    }
                }
                newpos += repel_gain * repel;
                if (is_collision_free_segment(grid, pts[k - 1], newpos) &&
                    is_collision_free_segment(grid, newpos, pts[k + 1])) {
                    tmp[k] = newpos;
                }
            }
            pts.swap(tmp);
        }
    }

    // 3) 按弧长统一重采样（固定间距有利于控制器跟踪与曲率稳定）
    std::vector<Eigen::Vector2d> resampled;
    {
        // 累积弧长
        std::vector<double> acc; acc.reserve(pts.size());
        acc.push_back(0.0);
        for (size_t k = 1; k < pts.size(); ++k) {
            acc.push_back(acc.back() + (pts[k] - pts[k - 1]).norm());
        }
        double total = acc.back();
        if (total < 1e-6) return pts;

        // 重采样间距（米）：取较小固定值，保证足够平滑且点数不过多
        double ds = 0.12; // 更密的重采样间距以提升圆滑度
        int M = std::max(2, (int)std::ceil(total / ds));
        resampled.reserve(M + 1);
        size_t seg = 0;
        for (int m = 0; m <= M; ++m) {
            double s = (double)m * (total / M);
            while (seg + 1 < acc.size() && s > acc[seg + 1]) seg++;
            if (seg + 1 >= acc.size()) { resampled.push_back(pts.back()); break; }
            double t = (s - acc[seg]) / (acc[seg + 1] - acc[seg] + 1e-12);
            Eigen::Vector2d p = pts[seg] + t * (pts[seg + 1] - pts[seg]);
            resampled.push_back(p);
        }
    }

    // 4) 预平滑：Chaikin剪切（最多2次），在不碰撞的前提下消除尖角
    {
        int chaikin_passes = 3;
        double w = 0.25; // Chaikin权重
        for (int p = 0; p < chaikin_passes; ++p) {
            if (resampled.size() <= 2) break;
            std::vector<Eigen::Vector2d> cut;
            cut.reserve(resampled.size() * 2);
            cut.push_back(resampled.front());
            Eigen::Vector2d prev = resampled.front();
            for (size_t k = 0; k + 1 < resampled.size(); ++k) {
                Eigen::Vector2d P = resampled[k];
                Eigen::Vector2d Q = resampled[k + 1];
                Eigen::Vector2d q1 = (1.0 - w) * P + w * Q;
                Eigen::Vector2d q2 = w * P + (1.0 - w) * Q;
                if (is_collision_free_segment(grid, prev, q1)) {
                    cut.push_back(q1);
                    prev = q1;
                }
                if (is_collision_free_segment(grid, prev, q2)) {
                    cut.push_back(q2);
                    prev = q2;
                }
            }
            if (is_collision_free_segment(grid, prev, resampled.back())) {
                cut.push_back(resampled.back());
            } else {
                cut.push_back(prev);
            }
            resampled.swap(cut);
        }
    }

    // 4) 角点圆弧倒角（车辆最小转弯半径约束），替换尖锐拐角为圆弧
    {
        auto rot90 = [](const Eigen::Vector2d& v){ return Eigen::Vector2d(-v.y(), v.x()); };
        auto clamp01 = [](double x){ return std::max(-1.0, std::min(1.0, x)); };
        // 自适应角点倒角：根据车辆最小转弯半径与局部障碍距离调整
        double Rmin_vehicle = [&](){
            SystemModel tmp_model; // 使用默认车型参数
            ArticulatedLimits lims = compute_articulated_limits(tmp_model, 0.35); // 与 main.cpp 保持一致的机械上限
            return std::max(lims.R_min, 0.5); // 下限保护，避免过小
        }();
        auto nearest_obstacle_distance = [&](const Eigen::Vector2d& p){
            int gx = static_cast<int>(std::floor((p.x() - grid.origin_x) / grid.resolution));
            int gy = static_cast<int>(std::floor((p.y() - grid.origin_y) / grid.resolution));
            int Rcells = std::min(25, std::max(8, (int)std::round(6.0 / std::max(grid.resolution, 1e-3))));
            double best = 1e9;
            for (int dy = -Rcells; dy <= Rcells; ++dy) {
                for (int dx = -Rcells; dx <= Rcells; ++dx) {
                    int nx = gx + dx;
                    int ny = gy + dy;
                    if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) continue;
                    size_t nidx = static_cast<size_t>(ny) * grid.width + static_cast<size_t>(nx);
                    if (grid.cells[nidx] == 1) {
                        double wx = grid.origin_x + (nx + 0.5) * grid.resolution;
                        double wy = grid.origin_y + (ny + 0.5) * grid.resolution;
                        double d = std::hypot(p.x() - wx, p.y() - wy);
                        if (d < best) best = d;
                    }
                }
            }
            return (best < 1e8) ? best : 1e6;
        };
        // 角度阈值与倒角半径将按环境在循环中动态计算
        double angle_thresh = 0.0;
        double R = Rmin_vehicle;
        std::vector<Eigen::Vector2d> rounded; rounded.reserve(resampled.size()*2);
        rounded.push_back(resampled.front());
        for (size_t k = 1; k + 1 < resampled.size(); ++k) {
            Eigen::Vector2d p0 = resampled[k - 1];
            Eigen::Vector2d p1 = resampled[k];
            Eigen::Vector2d p2 = resampled[k + 1];
            Eigen::Vector2d d1 = (p1 - p0); double L1 = std::max(d1.norm(), 1e-9); d1 /= L1;
            Eigen::Vector2d d2 = (p2 - p1); double L2 = std::max(d2.norm(), 1e-9); d2 /= L2;
            double cosang = clamp01(d1.dot(d2));
            double phi = std::acos(cosang); // [0,pi]
            // 判断是否需要倒角
            if (phi > angle_thresh) {
                double t = R * std::tan(phi / 2.0);
                if (L1 > t + 0.15 && L2 > t + 0.15) {
                    Eigen::Vector2d T1 = p1 - d1 * t; // 第一段的切点
                    Eigen::Vector2d T2 = p1 + d2 * t; // 第二段的切点
                    // 计算转向方向（左/右）
                    double cross = d1.x() * d2.y() - d1.y() * d2.x();
                    double sgn = (cross >= 0.0) ? 1.0 : -1.0;
                    Eigen::Vector2d n1 = rot90(d1) * sgn; // 指向圆心的法向
                    Eigen::Vector2d n2 = rot90(d2) * sgn;
                    Eigen::Vector2d C1 = T1 + R * n1;
                    Eigen::Vector2d C2 = T2 + R * n2;
                    Eigen::Vector2d C = 0.5 * (C1 + C2); // 取平均以抑制数值误差
                    // 采样圆弧
                    double a1 = std::atan2(T1.y() - C.y(), T1.x() - C.x());
                    double a2 = std::atan2(T2.y() - C.y(), T2.x() - C.x());
                    // 保证按照转向方向采样
                    auto angle_diff = [&](double from, double to){
                        double d = to - from; while (d > M_PI) d -= 2*M_PI; while (d < -M_PI) d += 2*M_PI; return d; };
                    double delta = angle_diff(a1, a2);
                    int m = std::max(8, (int)std::ceil(std::abs(delta) / (10.0 * M_PI / 180.0))); // 每~10度一个点
                    // 检查与插入：保证与前/后段及圆弧段都不碰撞
                    bool ok = true;
                    // 前段到T1
                    if (!is_collision_free_segment(grid, rounded.back(), T1)) ok = false;
                    // 圆弧分段检查
                    Eigen::Vector2d prev = T1;
                    for (int i = 1; i <= m && ok; ++i) {
                        double th = a1 + sgn * (std::abs(delta) * i / m);
                        Eigen::Vector2d q(C.x() + R * std::cos(th), C.y() + R * std::sin(th));
                        if (!is_collision_free_segment(grid, prev, q)) { ok = false; break; }
                        prev = q;
                    }
                    // 最后段T2到p2
                    if (ok && !is_collision_free_segment(grid, prev, p2)) ok = false;
                    if (ok) {
                        // 插入：rounded.back() 已经是上一点，这里插入圆弧替代尖角
                        rounded.push_back(T1);
                        prev = T1;
                        for (int i = 1; i <= m; ++i) {
                            double th = a1 + sgn * (std::abs(delta) * i / m);
                            Eigen::Vector2d q(C.x() + R * std::cos(th), C.y() + R * std::sin(th));
                            rounded.push_back(q);
                        }
                        // 不添加 p1（移除尖角），继续由循环添加后续点
                        continue; // 跳过常规添加p1
                    }
                }
            }
            // 默认不倒角：保留原点
            rounded.push_back(p1);
        }
        rounded.push_back(resampled.back());
        resampled.swap(rounded);
    }

    // 5) 终端平滑（增加迭代并对尖角自适应加权）
    {
        int final_iters = std::max(1, iterations * 2);
        double alpha2 = 0.50; // 提高终端平滑强度
        std::vector<Eigen::Vector2d> tmp = resampled;
        for (int it = 0; it < final_iters; ++it) {
            for (size_t k = 1; k + 1 < resampled.size(); ++k) {
                // 基于夹角自适应：尖角越大，平滑权重越高
                Eigen::Vector2d v1 = resampled[k] - resampled[k - 1];
                Eigen::Vector2d v2 = resampled[k + 1] - resampled[k];
                double n1 = std::max(v1.norm(), 1e-9);
                double n2 = std::max(v2.norm(), 1e-9);
                double cosang = std::clamp((v1.dot(v2)) / (n1 * n2), -1.0, 1.0);
                double ang = std::acos(cosang); // [0, pi]
                double ang_thresh = 45.0 * M_PI / 180.0;
                double extra = 0.20 * std::max(0.0, (ang - ang_thresh) / (M_PI - ang_thresh));
                double local_alpha = std::min(0.95, alpha2 + extra);

                Eigen::Vector2d c = 0.5 * (resampled[k - 1] + resampled[k + 1]);
                Eigen::Vector2d newpos = (1.0 - local_alpha) * resampled[k] + local_alpha * c;
                if (is_collision_free_segment(grid, resampled[k - 1], newpos) &&
                    is_collision_free_segment(grid, newpos, resampled[k + 1])) {
                    tmp[k] = newpos;
                }
            }
            resampled.swap(tmp);
        }
    }

    // 5) 基于弧长参数的自然三次样条（C2连续），进一步提升曲率连续性
    std::vector<Eigen::Vector2d> final_path;
    {
        size_t N = resampled.size();
        if (N < 3) {
            return resampled;
        }
        // 弧长参数 s_k
        std::vector<double> S(N, 0.0);
        for (size_t k = 1; k < N; ++k) {
            S[k] = S[k - 1] + (resampled[k] - resampled[k - 1]).norm();
        }
        double totalS = S.back();
        if (totalS < 1e-9) return resampled;
        // 分别对 x(s), y(s) 拟合自然三次样条，求二阶导数 m_k
        std::vector<double> X(N), Y(N);
        for (size_t k = 0; k < N; ++k) { X[k] = resampled[k].x(); Y[k] = resampled[k].y(); }
        auto compute_m = [&](const std::vector<double>& s, const std::vector<double>& v){
            size_t n = s.size();
            std::vector<double> m(n, 0.0);
            if (n < 3) return m;
            std::vector<double> a(n, 0.0), b(n, 0.0), c(n, 0.0), r(n, 0.0);
            for (size_t i = 1; i + 1 < n; ++i) {
                double h0 = s[i] - s[i - 1];
                double h1 = s[i + 1] - s[i];
                a[i] = h0;
                b[i] = 2.0 * (h0 + h1);
                c[i] = h1;
                r[i] = 6.0 * ((v[i + 1] - v[i]) / std::max(h1, 1e-12) - (v[i] - v[i - 1]) / std::max(h0, 1e-12));
            }
            // 前向消元
            for (size_t i = 2; i + 1 < n; ++i) {
                double w = a[i] / std::max(b[i - 1], 1e-12);
                b[i] -= w * c[i - 1];
                r[i] -= w * r[i - 1];
            }
            // 回代（自然边界 m0=mn-1=0）
            m[0] = 0.0; m[n - 1] = 0.0;
            m[n - 2] = r[n - 2] / std::max(b[n - 2], 1e-12);
            for (int i = (int)n - 3; i >= 1; --i) {
                m[i] = (r[i] - c[i] * m[i + 1]) / std::max(b[i], 1e-12);
            }
            return m;
        };
        std::vector<double> Mx = compute_m(S, X);
        std::vector<double> My = compute_m(S, Y);
        // 统一按固定 ds 采样样条曲线
        double ds_spline = 0.12;
        int M2 = std::max(2, (int)std::ceil(totalS / ds_spline));
        final_path.clear();
        final_path.reserve(M2 + 1);
        auto eval_xy = [&](double s){
            // 找到段索引 k 使得 S[k] <= s <= S[k+1]
            size_t k = 0;
            while (k + 1 < N && s > S[k + 1]) k++;
            if (k + 1 >= N) return resampled.back();
            double h = std::max(S[k + 1] - S[k], 1e-12);
            double A = (S[k + 1] - s) / h;
            double B = (s - S[k]) / h;
            double x = A * X[k] + B * X[k + 1] + ((A * A * A - A) * h * h / 6.0) * Mx[k] + ((B * B * B - B) * h * h / 6.0) * Mx[k + 1];
            double y = A * Y[k] + B * Y[k + 1] + ((A * A * A - A) * h * h / 6.0) * My[k] + ((B * B * B - B) * h * h / 6.0) * My[k + 1];
            return Eigen::Vector2d(x, y);
        };
        // 逐段生成并做碰撞约束：若样条点与上一点连线碰撞，则退回到原重采样点
        Eigen::Vector2d last = resampled.front();
        final_path.push_back(last);
        for (int m = 1; m <= M2; ++m) {
            double s = (double)m * (totalS / M2);
            Eigen::Vector2d p = eval_xy(s);
            if (is_collision_free_segment(grid, last, p)) {
                final_path.push_back(p);
                last = p;
            } else {
                // 使用对应索引的原始重采样点作为退化
                size_t idx = std::min((size_t)N - 1, (size_t)std::llround((double)m * (N - 1) / M2));
                Eigen::Vector2d q = resampled[idx];
                if (is_collision_free_segment(grid, last, q)) {
                    final_path.push_back(q);
                    last = q;
                } // 如果仍然不行，则跳过该步（保持连通性，不插入）
            }
        }
    }

    // 6) 局部角点强化圆滑（针对用户指定区域 X∈[20,30], Y∈[80,125]，留出一定裕度）
    {
        double bx_min = 20.0, bx_max = 30.0;
        double by_min = 80.0, by_max = 125.0;
        double margin = 5.0; // 裕度，避免边界外仍有影响的尖角
        auto in_box = [&](const Eigen::Vector2d& p){
            return (p.x() >= bx_min - margin && p.x() <= bx_max + margin &&
                    p.y() >= by_min - margin && p.y() <= by_max + margin);
        };
        int i0 = -1, i1 = -1;
        for (int i = 0; i < (int)final_path.size(); ++i) {
            if (in_box(final_path[i])) { i0 = i; break; }
        }
        for (int i = (int)final_path.size() - 1; i >= 0; --i) {
            if (in_box(final_path[i])) { i1 = i; break; }
        }
        if (i0 >= 1 && i1 >= i0 + 2) {
            std::vector<Eigen::Vector2d> sub(final_path.begin() + i0, final_path.begin() + i1 + 1);
            // 局部Chaikin：最多2次，权重0.25，带碰撞约束
            int chaikin_passes = 3;
            double w = 0.25;
            for (int p = 0; p < chaikin_passes; ++p) {
                if (sub.size() <= 2) break;
                std::vector<Eigen::Vector2d> cut;
                cut.reserve(sub.size() * 2);
                cut.push_back(sub.front());
                Eigen::Vector2d prev = sub.front();
                for (size_t k = 0; k + 1 < sub.size(); ++k) {
                    Eigen::Vector2d P = sub[k];
                    Eigen::Vector2d Q = sub[k + 1];
                    Eigen::Vector2d q1 = (1.0 - w) * P + w * Q;
                    Eigen::Vector2d q2 = w * P + (1.0 - w) * Q;
                    if (is_collision_free_segment(grid, prev, q1)) { cut.push_back(q1); prev = q1; }
                    if (is_collision_free_segment(grid, prev, q2)) { cut.push_back(q2); prev = q2; }
                }
                if (is_collision_free_segment(grid, prev, sub.back())) { cut.push_back(sub.back()); }
                else { cut.push_back(prev); }
                sub.swap(cut);
            }
            // 局部终端平滑：更强的alpha与迭代（带角度自适应，且碰撞约束）
            int local_iters = 40;
            double alpha_local = 0.60;
            std::vector<Eigen::Vector2d> tmp = sub;
            for (int it = 0; it < local_iters; ++it) {
                for (size_t k = 1; k + 1 < sub.size(); ++k) {
                    Eigen::Vector2d v1 = sub[k] - sub[k - 1];
                    Eigen::Vector2d v2 = sub[k + 1] - sub[k];
                    double n1 = std::max(v1.norm(), 1e-9);
                    double n2 = std::max(v2.norm(), 1e-9);
                    double cosang = std::clamp((v1.dot(v2)) / (n1 * n2), -1.0, 1.0);
                    double ang = std::acos(cosang);
                    double ang_thresh = 40.0 * M_PI / 180.0;
                    double extra = 0.25 * std::max(0.0, (ang - ang_thresh) / (M_PI - ang_thresh));
                    double local_alpha = std::min(0.95, alpha_local + extra);
                    Eigen::Vector2d c = 0.5 * (sub[k - 1] + sub[k + 1]);
                    Eigen::Vector2d newpos = (1.0 - local_alpha) * sub[k] + local_alpha * c;
                    if (is_collision_free_segment(grid, sub[k - 1], newpos) &&
                        is_collision_free_segment(grid, newpos, sub[k + 1])) {
                        tmp[k] = newpos;
                    }
                }
                sub.swap(tmp);
            }
            // 写回（注意Chaikin会增加点数，因此采用拼接替换区间）
            std::vector<Eigen::Vector2d> refined;
            refined.insert(refined.end(), final_path.begin(), final_path.begin() + i0);
            refined.insert(refined.end(), sub.begin(), sub.end());
            refined.insert(refined.end(), final_path.begin() + i1 + 1, final_path.end());
            final_path.swap(refined);
        }
    }

    return final_path;
}

// 使用开放均匀三次B样条对折线路径进行拟合，返回拟合后按弧长重采样的点集；
// 若拟合后任一段与占据栅格碰撞，则回退到原始平滑路径。
static std::vector<Eigen::Vector2d> try_bspline_cubic(const std::vector<Eigen::Vector2d>& polyline,
                                                     const OccupancyGrid& grid) {
    if (polyline.size() < 4) return polyline;
    auto clamp = [](double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); };

    // 次数 p=3，控制点 n+1
    const int p = 3;
    const int n = static_cast<int>(polyline.size()) - 1;
    const int m = n + p + 1; // 节点索引最大值
    std::vector<double> U(m + 1, 0.0);
    // 开放均匀节点：首尾各重复 p+1，中间均匀
    for (int i = 0; i <= p; ++i) U[i] = 0.0;
    for (int i = p + 1; i <= n; ++i) U[i] = double(i - p) / double(n - p);
    for (int i = n + 1; i <= m; ++i) U[i] = 1.0;

    // Cox–de Boor 递推
    std::function<double(int,int,double)> N_ip = [&](int i, int pp, double u){
        if (pp == 0) {
            if (U[i] <= u && u < U[i+1]) return 1.0;
            if (u == 1.0 && U[i] == 1.0) return 1.0; // 尾点包含
            return 0.0;
        }
        double denom1 = U[i+pp] - U[i];
        double denom2 = U[i+1+pp] - U[i+1];
        double term1 = 0.0, term2 = 0.0;
        if (denom1 > 1e-12) term1 = (u - U[i]) / denom1 * N_ip(i, pp-1, u);
        if (denom2 > 1e-12) term2 = (U[i+1+pp] - u) / denom2 * N_ip(i+1, pp-1, u);
        return term1 + term2;
    };
    auto eval = [&](double u){
        u = clamp(u, 0.0, 1.0);
        Eigen::Vector2d C(0.0, 0.0);
        for (int i = 0; i <= n; ++i) {
            double Ni = N_ip(i, p, u);
            C += Ni * polyline[i];
        }
        return C;
    };

    // 估计平均间距
    double total_len = 0.0;
    for (size_t i = 1; i < polyline.size(); ++i) total_len += (polyline[i] - polyline[i-1]).norm();
    double avg_ds = (polyline.size() > 1) ? total_len / (polyline.size()-1) : 0.5;
    avg_ds = clamp(avg_ds, 0.2, 3.0);

    // 初步细分参数点
    int initial_samples = std::max<int>(static_cast<int>(total_len / (avg_ds * 0.5)), 20);
    std::vector<Eigen::Vector2d> coarse; coarse.reserve(initial_samples+1);
    for (int k = 0; k <= initial_samples; ++k) {
        double u = double(k) / initial_samples;
        coarse.push_back(eval(u));
    }

    // 弧长重采样
    std::vector<double> s(coarse.size(), 0.0);
    for (size_t i = 1; i < coarse.size(); ++i) s[i] = s[i-1] + (coarse[i] - coarse[i-1]).norm();
    double S = s.back();
    int target_count = std::max<int>(static_cast<int>(S / avg_ds), 2);
    std::vector<Eigen::Vector2d> fitted; fitted.reserve(target_count+1);
    for (int j = 0; j <= target_count; ++j) {
        double sj = double(j) * S / target_count;
        size_t idx = std::lower_bound(s.begin(), s.end(), sj) - s.begin();
        if (idx == 0) { fitted.push_back(coarse.front()); continue; }
        if (idx >= s.size()) { fitted.push_back(coarse.back()); continue; }
        double t = (sj - s[idx-1]) / (s[idx] - s[idx-1] + 1e-12);
        Eigen::Vector2d P = coarse[idx-1] * (1.0 - t) + coarse[idx] * t;
        fitted.push_back(P);
    }

    // 碰撞检查：逐段
    for (size_t i = 1; i < fitted.size(); ++i) {
        if (!is_collision_free_segment(grid, fitted[i-1], fitted[i])) {
            return polyline; // 回退
        }
    }
    return fitted;
}

std::vector<std::vector<double>> load_map(double startx, double starty, double theta) {
    // 返回一个示例地图数据，实际应该从文件加载
    std::vector<std::vector<double>> map_data(3);
    double costheta = cos(theta);
    double sintheta = sin(theta); 
    // 生成一条简单的直线路径作为示例
    for (int i = 0; i < 1000; ++i) {
        map_data[0].push_back(startx + i*0.1*costheta);  // x坐标
        map_data[1].push_back(starty + i*0.1*sintheta);      // y坐标
        map_data[2].push_back(theta);      // heading
    }
    
    return map_data;
}

MapData load_bitmap_map(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open bitmap map file: " << file_path << std::endl;
        return MapData{};
    }

    MapData map_data;
    std::string json_content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
    
    // 简单的JSON解析 - 查找metadata部分
    size_t metadata_pos = json_content.find("\"metadata\"");
    if (metadata_pos == std::string::npos) {
        std::cerr << "Error: Could not find metadata in JSON file" << std::endl;
        return MapData{};
    }
    
    // 解析width
    size_t width_pos = json_content.find("\"width\"", metadata_pos);
    if (width_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", width_pos);
        size_t comma_pos = json_content.find(",", colon_pos);
        std::string width_str = json_content.substr(colon_pos + 1, comma_pos - colon_pos - 1);
        map_data.width = std::stoi(width_str);
    }
    
    // 解析height
    size_t height_pos = json_content.find("\"height\"", metadata_pos);   
    if (height_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", height_pos);
        size_t comma_pos = json_content.find(",", colon_pos);
        std::string height_str = json_content.substr(colon_pos + 1, comma_pos - colon_pos - 1);
        map_data.height = std::stoi(height_str);
    }
    
    // 解析resolution
    size_t res_pos = json_content.find("\"resolution\"", metadata_pos);
    if (res_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", res_pos);
        size_t end_pos = json_content.find("}", colon_pos);
        std::string res_str = json_content.substr(colon_pos + 1, end_pos - colon_pos - 1);
        map_data.resolution = std::stod(res_str);
    }
    
    // 解析origin
    size_t origin_pos = json_content.find("\"origin\"", metadata_pos);
    if (origin_pos != std::string::npos) {
        size_t bracket_start = json_content.find("[", origin_pos);
        size_t bracket_end = json_content.find("]", bracket_start);
        std::string origin_str = json_content.substr(bracket_start + 1, bracket_end - bracket_start - 1);
        size_t comma_pos = origin_str.find(",");
        double origin_x = std::stod(origin_str.substr(0, comma_pos));
        double origin_y = std::stod(origin_str.substr(comma_pos + 1));
        map_data.origin = {origin_x, origin_y};
    }
    
    // 解析max_elevation
    size_t max_elev_pos = json_content.find("\"max_elevation\"", metadata_pos);
    if (max_elev_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", max_elev_pos);
        size_t end_pos = json_content.find(",", colon_pos);
        if (end_pos == std::string::npos) end_pos = json_content.find("}", colon_pos);
        std::string max_elev_str = json_content.substr(colon_pos + 1, end_pos - colon_pos - 1);
        map_data.max_elevation = std::stod(max_elev_str);
    }
    
    // 解析data数组
    size_t data_pos = json_content.find("\"data\"");
    if (data_pos != std::string::npos) {
        size_t data_start = json_content.find("[", data_pos);
        size_t data_end = json_content.rfind("]");
        std::string data_str = json_content.substr(data_start + 1, data_end - data_start - 1);
        
        map_data.data.resize(map_data.height);
        
        // 解析每一行
        size_t row_start = 0;
        for (int i = 0; i < map_data.height && row_start < data_str.length(); ++i) {
            map_data.data[i].resize(map_data.width);
            
            // 找到这一行的开始和结束
            size_t row_bracket_start = data_str.find("[", row_start);
            size_t row_bracket_end = data_str.find("]", row_bracket_start);
            
            if (row_bracket_start != std::string::npos && row_bracket_end != std::string::npos) {
                std::string row_str = data_str.substr(row_bracket_start + 1, row_bracket_end - row_bracket_start - 1);
                
                // 解析这一行的数据
                std::istringstream row_stream(row_str);
                std::string value;
                int j = 0;
                while (std::getline(row_stream, value, ',') && j < map_data.width) {
                    // 去除空格和引号
                    value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
                    if (value.front() == '"') value = value.substr(1, value.length() - 2);
                    
                    if (value == "-1") {
                        map_data.data[i][j] = -1.0;
                    } else {
                        map_data.data[i][j] = std::stod(value);
                    }
                    j++;
                }
                
                row_start = row_bracket_end + 1;
            } else {
                break;
            }
        }
    }
    
    std::cout << "Bitmap map loaded: " << map_data.width << " x " << map_data.height 
              << ", resolution: " << map_data.resolution << ", max_elevation: " << map_data.max_elevation << std::endl;
    return map_data;
}

SemanticMapData load_semantic_map(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open map file" << std::endl;
        return SemanticMapData{};
    }

    SemanticMapData map_data;
    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        SemanticMapPoint point;
        if (iss >> point.x >> point.y >> point.type) {
            map_data.points.push_back(point);
        }
    }
    
    std::cout << "Semantic map loaded. Size: " << map_data.points.size() << std::endl;
    return map_data;
}

void fill_global_path_points(std::vector<std::vector<double>>& global_plan_log) {
    global_plan_log.clear();
    global_plan_log.resize(2);
    
    double start_x = 450.0;
    double start_y = 30.0;
    double end_x = 550.0;
    double end_y = 50.0;
    
    int num_points = 100;
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double x = start_x + t * (end_x - start_x);
        double y = start_y + t * (end_y - start_y);
        
        global_plan_log[0].push_back(x);
        global_plan_log[1].push_back(y);
    }
    
    std::cout << "Global path points filled: " << global_plan_log[0].size() << " points" << std::endl;
}

void set_global_path(GlobalPlan& global_plan, const std::vector<std::vector<double>>& global_plan_log) {
    // Clear existing path by setting empty vector
    global_plan.set_plan(std::vector<Point>());
    
    if (global_plan_log.size() >= 2 && !global_plan_log[0].empty()) {
        std::vector<Point> points;
        for (size_t i = 0; i < global_plan_log[0].size(); ++i) {
            Point point(global_plan_log[0][i], global_plan_log[1][i], 0.0);
            points.push_back(point);
        }
        global_plan.set_plan(points);
    }
    
    std::cout << "Global path set with " << global_plan.get_points().size() << " points" << std::endl;
}

void init_params(Params& params) {
    // 使用 Eigen::MatrixXd（若禁用Eigen，则使用在 utils.h 中提供的轻量级 stub）
    params.Q = Eigen::MatrixXd::Identity(4, 4);
    params.Q(0, 0) = 10.0;
    params.Q(1, 1) = 10.0;
    params.Q(2, 2) = 1.0;
    params.Q(3, 3) = 1.0;

    params.R = Eigen::MatrixXd::Identity(2, 2);
    params.R(0, 0) = 0.1;
    params.R(1, 1) = 0.1;

    params.Qf = Eigen::MatrixXd::Identity(4, 4);
    params.Qf(0, 0) = 100.0;
    params.Qf(1, 1) = 100.0;
    params.Qf(2, 2) = 100.0;
    params.Qf(3, 3) = 100.0;

    params.dt = 0.1;
    params.N = 50;
    params.max_iter = 20;
    params.tol = 1e-4;

    std::cout << "Parameters initialized" << std::endl;
}

void init_vehicle_model(VehicleModel& vehicle_model) {
    vehicle_model.L1 = 2.7;
    vehicle_model.L2 = 2.7;
    vehicle_model.width = 2.0;
    
    std::cout << "Vehicle model initialized: L1=" << vehicle_model.L1 
              << ", L2=" << vehicle_model.L2 << ", width=" << vehicle_model.width << std::endl;
}

void init_obstacle_trajectory(Trajectory& obs_traj) {
    obs_traj.states.clear();
    State obs_state(480.0, 35.0, 0.0, 0.0);
    int N = 50;
    for (int i = 0; i < N; ++i) {
        obs_traj.states.push_back(obs_state);
    }
    std::cout << "Obstacle trajectory initialized with " << obs_traj.states.size() << " states" << std::endl;
}

void draw_bitmap_debug(const MapData& map_data, const std::string& output_path) {
    if (map_data.width <= 0 || map_data.height <= 0 || map_data.data.empty()) {
        throw std::runtime_error("draw_bitmap_debug: invalid map_data");
    }
    
    std::cout << "Drawing bitmap map: " << map_data.width << "x" << map_data.height << std::endl;
    
    // 设置matplotlib后端为Agg（非交互式）
    matplotlibcpp::backend("Agg");
    
    int height = map_data.height;
    int width = map_data.width;
    std::vector<float> map_for_display(height * width);
    
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double v = (i < (int)map_data.data.size() && j < (int)map_data.data[i].size())
                        ? map_data.data[i][j] : 0.0;
            
            if (v == -1.0) {
                map_for_display[i * width + j] = 0.0f;
            } else {
                map_for_display[i * width + j] = static_cast<float>(
                    map_data.max_elevation > 0.0 ? v / map_data.max_elevation : 0.0);
            }
        }
    }
    
    double x_min = map_data.origin.size() > 0 ? map_data.origin[0] : 0.0;
    double y_min = map_data.origin.size() > 1 ? map_data.origin[1] : 0.0;
    double x_max = x_min + width * map_data.resolution;
    double y_max = y_min + height * map_data.resolution;
    
    std::cout << "Map coordinates: x=[" << x_min << ", " << x_max << "], y=[" << y_min << ", " << y_max << "]" << std::endl;
    std::cout << "Map origin size: " << map_data.origin.size() << std::endl;
    if (map_data.origin.size() >= 2) {
        std::cout << "Origin: [" << map_data.origin[0] << ", " << map_data.origin[1] << "]" << std::endl;
    }

    std::map<std::string, std::string> keywords;
    keywords["cmap"] = "gray";
    keywords["extent"] = std::to_string(x_min) + "," + std::to_string(x_max) + "," +
                         std::to_string(y_min) + "," + std::to_string(y_max);

    matplotlibcpp::imshow(map_for_display.data(), height, width, 1, keywords);
    matplotlibcpp::colorbar();
    matplotlibcpp::title("Bitmap Debug");
    matplotlibcpp::xlabel("X (m)");
    matplotlibcpp::ylabel("Y (m)");
    
    std::string save_path = output_path.empty() ? "bitmap_debug.png" : output_path;
    matplotlibcpp::save(save_path);
    std::cout << "Map saved to: " << save_path << std::endl;
}

void my_plot(const std::vector<std::vector<double>>& global_plan_log,
             const std::vector<std::vector<double>>& ego_log,
             const Trajectory& obs_traj,
             const Solution& solution,
             const MapData* map_data) {
    std::cout << "\n============================" << std::endl;
    std::cout << "my_plot called - generating visualization" << std::endl;
    
    if (!ego_log.empty() && !ego_log[0].empty()) {
        std::cout << "Current vehicle state:" << std::endl;
        std::cout << "  Position: (" << ego_log[0].back() << ", " << ego_log[1].back() << ")" << std::endl;
        if (ego_log.size() > 2) {
            std::cout << "  Heading: " << ego_log[2].back() << " rad" << std::endl;
        }
        if (ego_log.size() > 3) {
            std::cout << "  Articulation angle: " << ego_log[3].back() << " rad" << std::endl;
        }
    }
    
    if (!solution.ego_trj.states.empty()) {
        std::cout << "Planned trajectory: " << solution.ego_trj.states.size() << " points" << std::endl;
        std::cout << "  Start: (" << solution.ego_trj.states[0][0] << ", " << solution.ego_trj.states[0][1] << ")" << std::endl;
        std::cout << "  End: (" << solution.ego_trj.states.back()[0] << ", " << solution.ego_trj.states.back()[1] << ")" << std::endl;
    }
    
    if (!obs_traj.states.empty()) {
        std::cout << "Obstacle position: (" << obs_traj.states[0][0] << ", " << obs_traj.states[0][1] << ")" << std::endl;
    }
    
    if (map_data && map_data->width > 0 && map_data->height > 0) {
        std::cout << "Drawing bitmap map..." << std::endl;
        draw_bitmap_debug(*map_data, "");
    } else {
        std::cout << "No valid map data available for visualization" << std::endl;
    }
    
    std::cout << "============================\n" << std::endl;
}

void save_map_data(const MapData* map_data) {
    if (!map_data || map_data->width <= 0 || map_data->height <= 0 || map_data->data.empty()) {
        std::cout << "No valid map data to save" << std::endl;
        return;
    }
    
    // 获取当前工作目录并构建地图文件路径
    std::string map_filename;
    #ifdef _WIN32
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        std::string exe_path(buffer);
        std::string exe_dir = exe_path.substr(0, exe_path.find_last_of("\\"));
        // 从可执行文件目录向上找到项目根目录
        size_t build_pos = exe_dir.find("\\build");
        if (build_pos != std::string::npos) {
            std::string project_root = exe_dir.substr(0, build_pos);
            map_filename = project_root + "\\outputs\\maps\\map_data.json";
        } else {
            map_filename = "outputs/maps/map_data.json";
        }
    #else
        map_filename = "../outputs/maps/map_data.json";
    #endif
    
    std::ofstream map_file(map_filename);
    if (map_file.is_open()) {
        map_file << "{\n";
        map_file << "  \"width\": " << map_data->width << ",\n";
        map_file << "  \"height\": " << map_data->height << ",\n";
        map_file << "  \"resolution\": " << map_data->resolution << ",\n";
        map_file << "  \"max_elevation\": " << map_data->max_elevation << ",\n";
        map_file << "  \"origin\": [";
        if (map_data->origin.size() >= 2) {
            map_file << map_data->origin[0] << ", " << map_data->origin[1];
        } else {
            map_file << "0, 0";
        }
        map_file << "],\n";
        map_file << "  \"data\": [";
        for (int i = 0; i < map_data->height; ++i) {
            map_file << "[";
            for (int j = 0; j < map_data->width; ++j) {
                double v = (i < (int)map_data->data.size() && j < (int)map_data->data[i].size())
                            ? map_data->data[i][j] : 0.0;
                map_file << v;
                if (j < map_data->width - 1) map_file << ", ";
            }
            map_file << "]";
            if (i < map_data->height - 1) map_file << ", ";
        }
        map_file << "]\n";
        map_file << "}\n";
        map_file.close();
        
        std::cout << "Map data saved to: " << map_filename << std::endl;
    } else {
        std::cout << "Error: Could not open file " << map_filename << " for writing" << std::endl;
    }
}

struct RRTNode {
    Eigen::Vector2d pos;
    int parent = -1;
    double cost = 0.0;
};

static int nearest_index(const std::vector<RRTNode>& nodes, const Eigen::Vector2d& q) {
    int best = -1; double bestd = 1e18;
    for (int i = 0; i < (int)nodes.size(); ++i) {
        double d = (nodes[i].pos - q).norm();
        if (d < bestd) { bestd = d; best = i; }
    }
    return best;
}

static Eigen::Vector2d steer_towards(const Eigen::Vector2d& from, const Eigen::Vector2d& to, double step) {
    Eigen::Vector2d dir = to - from; double d = dir.norm();
    if (d <= step) return to;
    return from + dir * (step / std::max(d, 1e-9));
}

bool rrt_star_plan(const MapData& map,
                   const Eigen::Vector3d& start,
                   const Eigen::Vector3d& goal,
                   std::vector<Point>& out_points,
                   const RRTStarParams& params) {
    std::cout << "RRT* starting with params: max_iters=" << params.max_iters 
              << ", step_size=" << params.step_size 
              << ", goal_tolerance=" << params.goal_tolerance 
              << ", inflation=" << params.inflation_radius << std::endl;
    
    // 1) 构建占用栅格
    OccupancyGrid grid = make_occupancy_grid(map, 0.1, params.inflation_radius);
    auto collision_free = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b){
        return is_collision_free_segment(grid, a, b);
    };
    // 点自由性检查（拒绝采样非法/占用点）
    auto is_free_point = [&](const Eigen::Vector2d& p){
        int gx = static_cast<int>(std::floor((p.x() - grid.origin_x) / grid.resolution));
        int gy = static_cast<int>(std::floor((p.y() - grid.origin_y) / grid.resolution));
        if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) return false;
        size_t idx = static_cast<size_t>(gy) * grid.width + static_cast<size_t>(gx);
        return grid.cells[idx] == 0;
    };

    // 2) 初始化节点集合
    std::vector<RRTNode> nodes;
    nodes.reserve(10000);
    nodes.push_back({Eigen::Vector2d(start[0], start[1]), -1, 0.0});

    // 3) 随机采样器（支持随机种子与“直线路径走廊”采样）
    unsigned int seed = std::random_device{}();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(grid.origin_x, grid.origin_x + grid.width * grid.resolution);
    std::uniform_real_distribution<double> uy(grid.origin_y, grid.origin_y + grid.height * grid.resolution);
    std::uniform_real_distribution<double> ur(0.0, 1.0);
    std::uniform_real_distribution<double> ut(0.0, 1.0);
    std::uniform_real_distribution<double> uoff(-1.0, 1.0); // 走廊偏移比例

    Eigen::Vector2d start2(start[0], start[1]);
    Eigen::Vector2d goal2(goal[0], goal[1]);
    Eigen::Vector2d dir = goal2 - start2;
    double L = std::max(dir.norm(), 1e-9);
    Eigen::Vector2d tangent = dir / L;
    Eigen::Vector2d normal(-tangent.y(), tangent.x());

    std::cout << "RRT* search space: x[" << grid.origin_x << ", " << (grid.origin_x + grid.width * grid.resolution) 
              << "], y[" << grid.origin_y << ", " << (grid.origin_y + grid.height * grid.resolution) << "]" << std::endl;

    // 4) 主循环
    for (int iter = 0; iter < params.max_iters; ++iter) {
        if (iter % 1000 == 0) {
            std::cout << "RRT* iteration " << iter << ", nodes: " << nodes.size() << std::endl;
        }
        
        // 4.1) 采样（目标采样 + 走廊采样 + 全局均匀采样）
        Eigen::Vector2d qrand;
        double r = ur(rng);
        if (r < params.goal_sample_rate) {
            qrand = goal2;
        } else if (r < params.goal_sample_rate + params.corridor_sample_rate) {
            double t = ut(rng); // [0,1] 在线段上取点
            double offset = uoff(rng) * (params.corridor_width * 0.5); // [-w/2, w/2]
            qrand = start2 + t * dir + normal * offset;
        } else {
            qrand = Eigen::Vector2d(ux(rng), uy(rng));
        }
        // 若采样点不可用，则有限次重试
        int retry = 0;
        while (!is_free_point(qrand) && retry < 10) {
            qrand = Eigen::Vector2d(ux(rng), uy(rng));
            retry++;
        }
        if (!is_free_point(qrand)) continue;

        // 4.2) 最近邻
        int idx = nearest_index(nodes, qrand);
        if (idx < 0) continue;
        Eigen::Vector2d qnear = nodes[idx].pos;

        // 4.3) 扩展
        Eigen::Vector2d qnew = steer_towards(qnear, qrand, params.step_size);
        if (!collision_free(qnear, qnew)) continue;

        // 4.4) 插入新节点（简化版，不做重连）
        double line_cost = (qnew - qnear).norm();
        double obs_dist = nearest_obstacle_distance_world(grid, qnew);
        double obs_cost = params.max_clearance / (obs_dist + 1e-6);
        double new_cost = nodes[idx].cost + line_cost + params.cost_obstacle_weight * obs_cost;
        nodes.push_back({qnew, idx, new_cost});
        int new_idx = (int)nodes.size() - 1;

        // 4.5) RRT* 重连：检查邻域，寻找更优父节点 + 重布线邻居
        double radius = params.rewire_radius_factor * std::pow(std::log(nodes.size()) / nodes.size(), 1.0 / 2.0);
        std::vector<int> neighbor_indices;
        for (int i = 0; i < (int)nodes.size(); ++i) {
            if (i != new_idx && (nodes[i].pos - qnew).norm() < radius) {
                neighbor_indices.push_back(i);
            }
        }

        // 寻找更优父节点
        for (int neighbor_idx : neighbor_indices) {
            if (is_collision_free_segment(grid, nodes[neighbor_idx].pos, qnew)) {
                double temp_line_cost = (qnew - nodes[neighbor_idx].pos).norm();
                double temp_obs_dist = nearest_obstacle_distance_world(grid, qnew);
                double temp_obs_cost = params.max_clearance / (temp_obs_dist + 1e-6);
                double temp_cost = nodes[neighbor_idx].cost + temp_line_cost + params.cost_obstacle_weight * temp_obs_cost;
                if (temp_cost < new_cost) {
                    new_cost = temp_cost;
                    nodes[new_idx].parent = neighbor_idx;
                    nodes[new_idx].cost = new_cost;
                }
            }
        }

        // 重布线邻居
        for (int neighbor_idx : neighbor_indices) {
            if (is_collision_free_segment(grid, qnew, nodes[neighbor_idx].pos)) {
                double rewire_line_cost = (nodes[neighbor_idx].pos - qnew).norm();
                double rewire_obs_dist = nearest_obstacle_distance_world(grid, nodes[neighbor_idx].pos);
                double rewire_obs_cost = params.max_clearance / (rewire_obs_dist + 1e-6);
                double potential_new_cost = new_cost + rewire_line_cost + params.cost_obstacle_weight * rewire_obs_cost;
                if (potential_new_cost < nodes[neighbor_idx].cost) {
                    nodes[neighbor_idx].parent = new_idx;
                    nodes[neighbor_idx].cost = potential_new_cost;
                }
            }
        }

        // 4.5) 终止判定
        double dist_to_goal = (qnew - goal2).norm();
        if (dist_to_goal <= params.goal_tolerance) {
            std::cout << "RRT* found path to goal! Distance: " << dist_to_goal << ", nodes: " << nodes.size() << std::endl;
            // 回溯路径
            std::vector<Eigen::Vector2d> path2;
            int cur = new_idx;
            while (cur >= 0) {
                path2.push_back(nodes[cur].pos);
                cur = nodes[cur].parent;
            }
            std::reverse(path2.begin(), path2.end());
            std::vector<Eigen::Vector2d> smoothed_path = smooth_path(path2, grid, 24);
            // 优先使用QP走廊约束平滑；若碰撞则回退到B样条
            std::vector<Eigen::Vector2d> fitted_path = optimize_path_qp(smoothed_path, grid, 35, 1200.0);
            if (!is_collision_free_polyline(grid, fitted_path)) {
                std::cout << "[QP->BSpline] RRT*: QP后仍碰撞，回退到B样条" << std::endl;
                fitted_path = try_bspline_cubic(smoothed_path, grid);
            } else {
                std::cout << "[QP] RRT*: QP优化成功" << std::endl;
            }
            
            // 输出路径点（补 heading）
            out_points.clear();
            out_points.reserve(fitted_path.size());
            for (size_t i = 0; i < fitted_path.size(); ++i) {
                double heading = 0.0;
                if (i + 1 < fitted_path.size()) {
                    Eigen::Vector2d d = fitted_path[i+1] - fitted_path[i];
                    heading = std::atan2(d.y(), d.x());
                } else if (i > 0) {
                    Eigen::Vector2d d = fitted_path[i] - fitted_path[i-1];
                    heading = std::atan2(d.y(), d.x());
                }
                out_points.emplace_back(fitted_path[i].x(), fitted_path[i].y(), heading);
            }
            // 追加目标点确保接口一致
            out_points.emplace_back(goal[0], goal[1], goal[2]);
            return true;
        }
    }

    // 失败
    std::cout << "RRT* failed after " << params.max_iters << " iterations, final nodes: " << nodes.size() << std::endl;
    return false;
}
std::string resolve_resource_path(const std::string& relative_path) {
#ifdef _WIN32
    char buffer[MAX_PATH];
    GetModuleFileNameA(NULL, buffer, MAX_PATH);
    std::string exe_path(buffer);
    std::string exe_dir = exe_path.substr(0, exe_path.find_last_of("\\"));
    // 从可执行文件目录向上找到项目根目录
    size_t build_pos = exe_dir.find("\\build");
    if (build_pos != std::string::npos) {
        std::string project_root = exe_dir.substr(0, build_pos);
        return project_root + "\\" + relative_path;
    }
    return relative_path;
#else
    return relative_path;
#endif
}

void save_m_map_info(const std::vector<std::vector<double>>& m_map_info) {
#ifdef _WIN32
    char buffer[MAX_PATH];
    GetModuleFileNameA(NULL, buffer, MAX_PATH);
    std::string exe_path(buffer);
    std::string exe_dir = exe_path.substr(0, exe_path.find_last_of("\\"));
    size_t build_pos = exe_dir.find("\\build");
    std::string output_path;
    if (build_pos != std::string::npos) {
        std::string project_root = exe_dir.substr(0, build_pos);
        output_path = project_root + "\\outputs\\data\\m_map_info.json";
    } else {
        output_path = "outputs/data/m_map_info.json";
    }
#else
    std::string output_path = "../outputs/data/m_map_info.json";
#endif
    std::ofstream out(output_path);
    if (!out.is_open()) {
        std::cerr << "Failed to open m_map_info output: " << output_path << std::endl;
        return;
    }
    out << "{\n";
    out << "  \"x\": [";
    for (size_t i = 0; i < m_map_info[0].size(); ++i) {
        out << m_map_info[0][i];
        if (i + 1 < m_map_info[0].size()) out << ", ";
    }
    out << "],\n";
    out << "  \"y\": [";
    for (size_t i = 0; i < m_map_info[1].size(); ++i) {
        out << m_map_info[1][i];
        if (i + 1 < m_map_info[1].size()) out << ", ";
    }
    out << "],\n";
    out << "  \"heading\": [";
    for (size_t i = 0; i < m_map_info[2].size(); ++i) {
        out << m_map_info[2][i];
        if (i + 1 < m_map_info[2].size()) out << ", ";
    }
    out << "]\n";
    out << "}\n";
    out.close();
    std::cout << "m_map_info exported to: " << output_path << std::endl;
}


bool plan_global_path(const MapData& bitmap_map,
                      const Eigen::Vector3d& start,
                      const Eigen::Vector3d& goal,
                      GlobalPlan& out_plan,
                      bool enable_rrt) {
    std::vector<Point> points;
    if (enable_rrt) {
        // 使用混合A*替代RRT*作为全局规划器
        HybridAStarParams hparams;
        hparams.grid_resolution = std::max(0.05, bitmap_map.resolution);
        hparams.inflation_radius = 2.0;
        hparams.turning_radius = 5.0;
        hparams.move_step = 0.5; // was 0.4, speed up progress per expansion
        hparams.move_step_backwards = 0.4; // was 0.3, keep ratio similar
        hparams.num_steering_angles = 7; // was 5, finer angular options
        hparams.heading_resolution = 5.0; // was default 10 deg, finer heading bins
        hparams.allow_reverse = false; // enable reverse to escape local traps
        hparams.heuristic_weight = 2.0; // make search more goal-directed
        hparams.max_iterations = 200000; // was 20000, allow more iterations if needed
        hparams.num_nodes_to_keep = 60000; // was 30000, reduce pruning pressure
        hparams.goal_tolerance_xy = 0.8;
        hparams.goal_tolerance_heading = 15.0;
        // 清距偏好（让路径更远离障碍物）
        hparams.clearance_weight = 12;     // 清距代价权重（越大越偏好远离障碍）
        hparams.desired_clearance = 3;    // 期望与障碍的最小距离（米），小于该值增加代价
        hparams.max_clearance = 12.0;       // 清距归一化的上限（米），避免过大距离影响代价
        hparams.min_rs_clearance = 3;     // RS解析路径的最小允许清距（米），低于则拒绝解析连接
        std::cout << "Attempting Hybrid A* planning..." << std::endl;
        if (hybrid_astar_plan(bitmap_map, start, goal, points, hparams)) {
            std::cout << "Hybrid A* planning succeeded! Raw path points: " << points.size() << std::endl;
            std::cout << "[RAW PATH] 直接使用混合A*原始路径，跳过所有平滑处理" << std::endl;
            
            // 直接使用混合A*的原始路径，不进行任何平滑处理
            // 混合A*已经考虑了车辆运动学约束，路径应该是可行的
            
            // 确保目标点被正确添加（避免重复添加）
            if (points.empty() || std::hypot(points.back().x - goal[0], points.back().y - goal[1]) > 1e-3) {
                points.emplace_back(goal[0], goal[1], goal[2]);
            }
            
            out_plan.set_plan(points);
            return true;
        } else {
            std::cout << "Hybrid A* planning failed, falling back to direct line check..." << std::endl;
        }
    }
    // 简化版直线可行性检查
    OccupancyGrid grid = make_occupancy_grid(bitmap_map, 0.05, 1.0);  // 使用与RRT*相同的膨胀距离
    Eigen::Vector2d a(start[0], start[1]);
    Eigen::Vector2d b(goal[0], goal[1]);
    std::cout << "Checking direct line feasibility from (" << start[0] << ", " << start[1] << ") to (" << goal[0] << ", " << goal[1] << ")" << std::endl;
    if (is_collision_free_segment(grid, a, b)) {
        std::cout << "Direct line is collision-free, using straight path" << std::endl;
        double dx = goal[0] - start[0];
        double dy = goal[1] - start[1];
        double dist = std::hypot(dx, dy);
        double heading = std::atan2(dy, dx);
        int steps = std::max(200, (int)std::round(dist / 0.1));
        steps = std::min(steps, 1500);
        points.reserve(steps + 1);
        for (int i = 0; i <= steps; ++i) {
            double t = (steps > 0) ? (double)i / steps : 1.0;
            double x = start[0] + t * dx;
            double y = start[1] + t * dy;
            points.emplace_back(x, y, heading);
        }
        out_plan.set_plan(points);
        return true;
    } else {
        std::cout << "Direct line has collisions, using fallback path" << std::endl;
    }
    // 安全回退：在栅格上用A*搜索一条无碰撞路径
    std::cout << "Fallback to A* grid search path" << std::endl;
    std::vector<Eigen::Vector2d> astar_path;
    if (!astar_search(grid, a, b, astar_path)) {
        std::cerr << "A* fallback failed: no path found on inflated grid" << std::endl;
        return false;
    }
    // 将A*结果平滑与重采样，保证跟踪友好；随后优先进行QP优化（自适应走廊约束），若不通过碰撞则回退到三次B样条拟合
    {
        std::vector<Eigen::Vector2d> smoothed = smooth_path(astar_path, grid, 3);
        std::vector<Eigen::Vector2d> fitted = optimize_path_qp(smoothed, grid, 35, 1200.0);
        if (!is_collision_free_polyline(grid, fitted)) {
            std::cout << "[QP->BSpline] A*: QP后仍碰撞，回退到B样条" << std::endl;
            fitted = try_bspline_cubic(smoothed, grid);
        } else {
            std::cout << "[QP] A*: QP优化成功" << std::endl;
        }
        astar_path = fitted;
    }
    std::vector<Eigen::Vector2d> smooth_astar = smooth_path(astar_path, grid, 24);
    std::vector<Eigen::Vector2d> fitted_astar = optimize_path_qp(smooth_astar, grid, 35, 1200.0);
    if (!is_collision_free_polyline(grid, fitted_astar)) {
        fitted_astar = try_bspline_cubic(smooth_astar, grid);
    }
    points.clear(); points.reserve(fitted_astar.size());
    for (size_t i = 0; i < fitted_astar.size(); ++i) {
        double hx = 0.0, hy = 0.0;
        if (i + 1 < fitted_astar.size()) { hx = fitted_astar[i+1].x() - fitted_astar[i].x(); hy = fitted_astar[i+1].y() - fitted_astar[i].y(); }
        else if (i > 0) { hx = fitted_astar[i].x() - fitted_astar[i-1].x(); hy = fitted_astar[i].y() - fitted_astar[i-1].y(); }
        double heading = std::atan2(hy, hx);
        points.emplace_back(fitted_astar[i].x(), fitted_astar[i].y(), heading);
    }
    std::cout << "Fallback A* path generated with " << points.size() << " points (after B-spline fitting check)" << std::endl;
    out_plan.set_plan(points);
    return true;
}

ArticulatedLimits compute_articulated_limits(const SystemModel& model, double gamma_max_mech) {
    // 保守推导：使用简单近似 kappa = tan(gamma)/L_eff，其中 L_eff = lr + lf*cos(gamma)
    // 取机械上限 gamma_max_mech 的保守值，计算最大曲率与最小半径
    double lf = model.lf;
    double lr = model.lr;
    double gamma = std::min(std::max(gamma_max_mech, 0.0), 0.6); // 保守：不超过约0.6rad
    double L_eff = lr + lf * std::cos(gamma);
    double kappa = std::abs(std::tan(gamma) / std::max(L_eff, 1e-3));
    ArticulatedLimits limits;
    limits.kappa_max = kappa;
    limits.R_min = (kappa > 1e-6) ? 1.0 / kappa : 1e6;
    return limits;
}

OccupancyGrid make_occupancy_grid(const MapData& map, double elevation_threshold_ratio, double inflate_radius_m) {
    OccupancyGrid grid;
    grid.width = map.width;
    grid.height = map.height;
    grid.resolution = map.resolution;
    grid.origin_x = map.origin.size() > 0 ? map.origin[0] : 0.0;
    grid.origin_y = map.origin.size() > 1 ? map.origin[1] : 0.0;
    grid.cells.resize(grid.width * grid.height, 0);
    
    // 使用严格的占用分类：-1 视为障碍，正数视为可通行（与地图定义一致）
    int obstacle_count = 0;  // 障碍单元数量
    int free_count = 0;      // 可通行单元数量
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            double v = map.data[y][x];
            size_t idx = static_cast<size_t>(y) * grid.width + static_cast<size_t>(x);
            if (v < 0) {
                grid.cells[idx] = 1;  // 障碍
                obstacle_count++;
            } else {
                grid.cells[idx] = 0;  // 可通行
                free_count++;
            }
        }
    }
    std::cout << "Grid statistics: obstacles=" << obstacle_count 
              << ", free=" << free_count << std::endl;
    // 简单膨胀：基于曼哈顿邻域的半径近似
    int inflate_cells = static_cast<int>(std::round(inflate_radius_m / std::max(grid.resolution, 1e-3)));
    if (inflate_cells > 0) {
        std::vector<uint8_t> inflated = grid.cells;
        for (int y = 0; y < grid.height; ++y) {
            for (int x = 0; x < grid.width; ++x) {
                size_t idx = static_cast<size_t>(y) * grid.width + static_cast<size_t>(x);
                if (grid.cells[idx] == 1) {
                    for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                        for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height) {
                                size_t nidx = static_cast<size_t>(ny) * grid.width + static_cast<size_t>(nx);
                                inflated[nidx] = 1;
                            }
                        }
                    }
                }
            }
        }
        grid.cells.swap(inflated);
    }
    return grid;
}

bool is_collision_free_segment(const OccupancyGrid& grid, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    double length = (b - a).norm();
    if (length < 1e-6) return true;
    int samples = static_cast<int>(std::ceil(length / (grid.resolution * 0.25)));
    for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples;
        double xw = a.x() + t * (b.x() - a.x());
        double yw = a.y() + t * (b.y() - a.y());
        int gx = static_cast<int>(std::floor((xw - grid.origin_x) / grid.resolution));
        int gy = static_cast<int>(std::floor((yw - grid.origin_y) / grid.resolution));

        // Check a 3x3 neighborhood for collision for a more conservative check
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = gx + dx;
                int ny = gy + dy;

                if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) {
                    return false; // Out of bounds is considered a collision
                }
                size_t idx = static_cast<size_t>(ny) * grid.width + static_cast<size_t>(nx);
                if (grid.cells[idx] == 1) {
                    return false;
                }
            }
        }
    }
    return true;
}

// 折线整体碰撞检查（逐段采样）
bool is_collision_free_polyline(const OccupancyGrid& grid, const std::vector<Eigen::Vector2d>& pts) {
    if (pts.size() < 2) return true;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (!is_collision_free_segment(grid, pts[i-1], pts[i])) return false;
    }
    return true;
}

// 查询世界坐标点到最近障碍栅格中心的距离（米）；若附近无障碍则返回一个大数
double nearest_obstacle_distance_world(const OccupancyGrid& grid, const Eigen::Vector2d& p) {
    int gx = static_cast<int>(std::floor((p.x() - grid.origin_x) / grid.resolution));
    int gy = static_cast<int>(std::floor((p.y() - grid.origin_y) / grid.resolution));
    int Rcells = std::min(35, std::max(8, (int)std::round(8.0 / std::max(grid.resolution, 1e-3))));
    double best = 1e9;
    for (int dy = -Rcells; dy <= Rcells; ++dy) {
        for (int dx = -Rcells; dx <= Rcells; ++dx) {
            int nx = gx + dx;
            int ny = gy + dy;
            if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) continue;
            size_t nidx = static_cast<size_t>(ny) * grid.width + static_cast<size_t>(nx);
            if (grid.cells[nidx] == 1) {
                double wx = grid.origin_x + (nx + 0.5) * grid.resolution;
                double wy = grid.origin_y + (ny + 0.5) * grid.resolution;
                double d = std::hypot(p.x() - wx, p.y() - wy);
                if (d < best) best = d;
            }
        }
    }
    return (best < 1e8) ? best : 1e6;
}

// QP走廊约束优化：在原始折线基础上进行二次平滑，并将偏移限制在依据最近障碍距离自适应的走廊内
std::vector<Eigen::Vector2d> optimize_path_qp(const std::vector<Eigen::Vector2d>& path,
                                              const OccupancyGrid& grid,
                                              int iterations,
                                              double curvature_weight) {
    const int M = static_cast<int>(path.size());
    if (M <= 2) return path;

    std::vector<Eigen::Vector2d> ref = path;
    std::vector<Eigen::Vector2d> X = path;

    // Cost function weights (tunable)
    const double w_track = 6.0;     // Adherence to the original polyline
    const double w_length = 4.0;     // First-order smoothing (suppresses jitter)
    const double w_anchor = 1e6;     // Anchor endpoints

    const double safety = std::max(0.5 * grid.resolution, 0.05);
    const double min_w = std::max(1.0 * grid.resolution, 0.06);
    const double max_w = 6.0; // Max corridor half-width (meters)

    // Pre-calculate tangents, normals, and corridor half-widths
    std::vector<Eigen::Vector2d> T(M), N(M);
    std::vector<double> W(M);
    for (int i = 0; i < M; ++i) {
        Eigen::Vector2d d;
        if (i == 0) d = ref[1] - ref[0];
        else if (i == M - 1) d = ref[M - 1] - ref[M - 2];
        else d = ref[i + 1] - ref[i - 1];
        double L = d.norm();
        if (L < 1e-9) d = Eigen::Vector2d(1.0, 0.0); else d /= L;
        T[i] = d;
        N[i] = Eigen::Vector2d(-d.y(), d.x());
        double wi = nearest_obstacle_distance_world(grid, ref[i]) - safety;
        W[i] = std::min(max_w, std::max(min_w, wi));
    }
    W[0] = std::min(W[0], min_w);
    W[M-1] = std::min(W[M-1], min_w);

    // Build Hessian and gradient (x and y dimensions are separable)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(M, M);
    Eigen::VectorXd bx = Eigen::VectorXd::Zero(M);
    Eigen::VectorXd by = Eigen::VectorXd::Zero(M);

    // Tracking term
    for (int i = 0; i < M; ++i) {
        H(i,i) += w_track;
        bx(i) += w_track * ref[i].x();
        by(i) += w_track * ref[i].y();
    }
    // Second-order smoothing term (curvature) Σ||x_{i+1} - 2x_i + x_{i-1}||^2
    for (int i = 1; i <= M - 2; ++i) {
        H(i-1, i-1) += curvature_weight * 1.0;
        H(i-1, i)   += curvature_weight * (-2.0);
        H(i-1, i+1) += curvature_weight * 1.0;

        H(i,   i-1) += curvature_weight * (-2.0);
        H(i,   i)   += curvature_weight * 4.0;
        H(i,   i+1) += curvature_weight * (-2.0);

        H(i+1, i-1) += curvature_weight * 1.0;
        H(i+1, i)   += curvature_weight * (-2.0);
        H(i+1, i+1) += curvature_weight * 1.0;
    }
    // First-order smoothing term (length) Σ||x_{i+1} - x_i||^2
    for (int i = 0; i <= M - 2; ++i) {
        H(i,   i)   += w_length;
        H(i,   i+1) += -w_length;
        H(i+1, i)   += -w_length;
        H(i+1, i+1) += w_length;
    }
    // Anchor endpoints with a very high weight
    H(0,0) += w_anchor; bx(0) += w_anchor * ref[0].x(); by(0) += w_anchor * ref[0].y();
    H(M-1,M-1) += w_anchor; bx(M-1) += w_anchor * ref[M-1].x(); by(M-1) += w_anchor * ref[M-1].y();

    // Iteratively solve the quadratic program and project back to the corridor
    for (int it = 0; it < std::max(1, iterations); ++it) {
        Eigen::VectorXd solx = H.ldlt().solve(bx);
        Eigen::VectorXd soly = H.ldlt().solve(by);
        for (int i = 0; i < M; ++i) {
            X[i].x() = solx(i);
            X[i].y() = soly(i);
        }
        // Project to corridor and fix endpoints
        X[0] = ref[0];
        X[M-1] = ref[M-1];
        for (int i = 1; i <= M - 2; ++i) {
            Eigen::Vector2d d = X[i] - ref[i];
            double tcomp = d.dot(T[i]);
            double scomp = d.dot(N[i]);
            double sproj = std::clamp(scomp, -W[i], W[i]);
            double tlimit = std::min(std::max(-1.5*W[i], tcomp), 1.5*W[i]); // Allow more freedom tangentially
            Eigen::Vector2d xi = ref[i] + T[i] * tlimit + N[i] * sproj;
            X[i] = 0.8 * xi + 0.2 * X[i]; // Strengthen projection influence
        }
    }

#include <fstream>

// ... existing code ...

    // 若路径段有碰撞，直接返回原始路径（在调用处会回退到B样条）
    if (!is_collision_free_polyline(grid, X)) {
        std::cout << "[QP] 优化结果有碰撞，返回原始折线以触发B样条回退" << std::endl;
        std::ofstream log_file("qp_debug.log", std::ios_base::app);
        log_file << "QP optimization failed. Path has collision." << std::endl;
        for (size_t i = 0; i < X.size() - 1; ++i) {
            if (!is_collision_free_segment(grid, X[i], X[i+1])) {
                std::cout << "  -> Collision on segment: (" << X[i].x() << ", " << X[i].y() 
                          << ") -> (" << X[i+1].x() << ", " << X[i+1].y() << ")" << std::endl;
                log_file << "  -> Collision on segment: (" << X[i].x() << ", " << X[i].y() 
                         << ") -> (" << X[i+1].x() << ", " << X[i+1].y() << ")" << std::endl;
            }
        }
        return ref;
    }
    std::cout << "[QP] 优化完成，点数=" << X.size() << std::endl;
    return X;
}

void dynamic_plot(const std::vector<std::vector<double>>& global_plan_log,
                  const std::vector<std::vector<double>>& ego_log,
                  const std::vector<Trajectory>& obs_trajectories,
                  const Solution& solution,
                  const MapData* map_data,
                  const GlobalPlan& global_plan,
                  const SystemModel& vehicle_model,
                  const Arg& arg) {
    
    // 保存数据到文件供Python脚本使用
    static int frame_count = 0;
    
    // 获取当前工作目录并构建数据文件路径
    std::string data_filename;
    #ifdef _WIN32
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        std::string exe_path(buffer);
        std::string exe_dir = exe_path.substr(0, exe_path.find_last_of("\\"));
        // 从可执行文件目录向上找到项目根目录
        size_t build_pos = exe_dir.find("\\build");
        if (build_pos != std::string::npos) {
            std::string project_root = exe_dir.substr(0, build_pos);
            data_filename = project_root + "\\outputs\\data\\cilqr_data_" + std::to_string(frame_count++) + ".json";
        } else {
            data_filename = "outputs/data/cilqr_data_" + std::to_string(frame_count++) + ".json";
        }
    #else
        data_filename = "../outputs/data/cilqr_data_" + std::to_string(frame_count++) + ".json";
    #endif
    
    std::ofstream data_file(data_filename);
    if (data_file.is_open()) {
        data_file << "{\n";
        
        // 保存算法收敛信息（放在开头）
        data_file << "  \"convergence_info\": {\n";
        data_file << "    \"converged\": " << (solution.converged ? "true" : "false") << ",\n";
        data_file << "    \"iterations\": " << solution.iterations << ",\n";
        data_file << "    \"final_cost\": " << solution.final_cost << ",\n";
        data_file << "    \"solve_time_ms\": " << solution.solve_time_ms << "\n";
        data_file << "  },\n";
        
        // 保存控制序列数据
        data_file << "  \"control_sequence\": {\n";
        if (!solution.control_sequence.controls.empty()) {
            data_file << "    \"velocity\": [";
            for (size_t i = 0; i < solution.control_sequence.controls.size(); ++i) {
                data_file << solution.control_sequence.controls[i][0];
                if (i < solution.control_sequence.controls.size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"steering\": [";
            for (size_t i = 0; i < solution.control_sequence.controls.size(); ++i) {
                data_file << solution.control_sequence.controls[i][1];
                if (i < solution.control_sequence.controls.size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"velocity\": [],\n";
            data_file << "    \"steering\": []\n";
        }
        data_file << "  },\n";
        
        // 保存全局路径数据
        data_file << "  \"global_plan\": {\n";
        std::vector<Point> global_points = global_plan.get_points();
        if (!global_points.empty()) {
            data_file << "    \"x\": [";
            for (size_t i = 0; i < global_points.size(); ++i) {
                data_file << global_points[i].x;
                if (i < global_points.size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"y\": [";
            for (size_t i = 0; i < global_points.size(); ++i) {
                data_file << global_points[i].y;
                if (i < global_points.size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"x\": [],\n";
            data_file << "    \"y\": []\n";
        }
        data_file << "  },\n";
        
        // 保存全局路径日志数据
        data_file << "  \"global_plan_log\": {\n";
        if (!global_plan_log.empty() && !global_plan_log[0].empty()) {
            data_file << "    \"x\": [";
            for (size_t i = 0; i < global_plan_log[0].size(); ++i) {
                data_file << global_plan_log[0][i];
                if (i < global_plan_log[0].size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"y\": [";
            for (size_t i = 0; i < global_plan_log[1].size(); ++i) {
                data_file << global_plan_log[1][i];
                if (i < global_plan_log[1].size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"x\": [],\n";
            data_file << "    \"y\": []\n";
        }
        data_file << "  },\n";
        
        // 保存规划轨迹数据
        data_file << "  \"planned_trajectory\": {\n";
        if (!solution.ego_trj.states.empty()) {
            data_file << "    \"x\": [";
            for (size_t i = 0; i < solution.ego_trj.states.size(); ++i) {
                data_file << solution.ego_trj.states[i][0];
                if (i < solution.ego_trj.states.size() - 1) data_file << ", ";
            }
            data_file << "],\n";
            
            data_file << "    \"y\": [";
            for (size_t i = 0; i < solution.ego_trj.states.size(); ++i) {
                data_file << solution.ego_trj.states[i][1];
                if (i < solution.ego_trj.states.size() - 1) data_file << ", ";
            }
            data_file << "]\n";
        } else {
            data_file << "    \"x\": [],\n";
            data_file << "    \"y\": []\n";
        }
        data_file << "  },\n";
        
        // 保存车辆状态数据
        data_file << "  \"ego_vehicle\": {\n";
        if (!ego_log.empty() && !ego_log[0].empty()) {
            double x = ego_log[0].back();
            double y = ego_log[1].back();
            double theta = ego_log[2].back();
            double gamma = ego_log[3].back();
            
            data_file << "    \"x\": " << x << ",\n";
            data_file << "    \"y\": " << y << ",\n";
            data_file << "    \"theta\": " << theta << ",\n";
            data_file << "    \"gamma\": " << gamma << "\n";
        } else {
            data_file << "    \"x\": 0,\n";
            data_file << "    \"y\": 0,\n";
            data_file << "    \"theta\": 0,\n";
            data_file << "    \"gamma\": 0\n";
        }
        data_file << "  },\n";
        
        // 保存障碍物数据
        data_file << "  \"obstacles\": [\n";
        for (size_t obs_idx = 0; obs_idx < obs_trajectories.size(); ++obs_idx) {
            data_file << "    {\n";
            const auto& obs_traj = obs_trajectories[obs_idx];
            if (!obs_traj.states.empty()) {
                data_file << "      \"x\": " << obs_traj.states[0][0] << ",\n";
                data_file << "      \"y\": " << obs_traj.states[0][1] << ",\n";
                data_file << "      \"theta\": " << obs_traj.states[0][2] << ",\n";
                data_file << "      \"length\": " << arg.obs_length << ",\n";
                data_file << "      \"width\": " << arg.obs_width << "\n";
            } else {
                data_file << "      \"x\": 0,\n";
                data_file << "      \"y\": 0,\n";
                data_file << "      \"theta\": 0,\n";
                data_file << "      \"length\": " << arg.obs_length << ",\n";
                data_file << "      \"width\": " << arg.obs_width << "\n";
            }
            data_file << "    }";
            if (obs_idx < obs_trajectories.size() - 1) {
                data_file << ",";
            }
            data_file << "\n";
        }
        data_file << "  ],\n";
        
        // 保存车辆模型参数
        data_file << "  \"vehicle_model\": {\n";
        data_file << "    \"lf\": " << vehicle_model.lf << ",\n";
        data_file << "    \"lr\": " << vehicle_model.lr << ",\n";
        data_file << "    \"len\": " << vehicle_model.len << ",\n";
        data_file << "    \"width\": " << vehicle_model.width << ",\n";
        data_file << "    \"box_length\": " << vehicle_model.box_length << "\n";
        data_file << "  }\n";
        
        // 移除地图数据保存逻辑，地图数据现在单独保存到maps目录
        
        data_file << "}\n";
        data_file.close();
        
        std::cout << "Data saved to: " << data_filename << std::endl;
    } else {
        std::cout << "Error: Could not open file " << data_filename << " for writing" << std::endl;
    }
}

// 障碍物预测函数实现
Trajectory predict_obstacle_trajectory(const State& initial_state, double dt, int N) {
    Trajectory predicted_trajectory;
    
    // 从初始状态提取位置、朝向和速度信息
    double x = initial_state[0];      // x位置
    double y = initial_state[1];      // y位置
    double theta = initial_state[2];  // 朝向角
    double v = initial_state[3];      // 速度
    
    // 使用匀速模型预测轨迹
    for (int i = 0; i < N; i++) {
        // 计算当前时刻的状态
        double current_time = i * dt;
        
        // 匀速直线运动模型
        double pred_x = x + v * cos(theta) * current_time;
        double pred_y = y + v * sin(theta) * current_time;
        double pred_theta = theta;  // 假设朝向不变
        double pred_v = v;          // 假设速度不变
        
        // 创建预测状态并添加到轨迹中
        State predicted_state(pred_x, pred_y, pred_theta, pred_v);
        predicted_trajectory.push_back(predicted_state);
    }
    
    return predicted_trajectory;
}