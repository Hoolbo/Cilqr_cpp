#include <matplot/matplot.h>
#include <Eigen/Eigen>

int main() {
    // matplot::backend("wxt");  // 官方标准后端设置方式
    auto x = matplot::linspace(0, 10);
    Eigen::Matrix2d Q(2,2);
    Q<<1,2,3,4;
    std::cout<<Q<<std::endl;
    matplot::plot(x, matplot::transform(x, [](double x) { return sin(x); }));
    matplot::show();
}