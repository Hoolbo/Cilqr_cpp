#include <matplot/matplot.h>
#include <Eigen/Eigen>
#include <iostream>
using namespace Eigen;

Vector4d dynamics( Vector4d X, Vector2d U){
    double lr = 1.133;
    double lf = 1.6;
    double len = 2.73;
    double beta = atan((lr / (lr + lf)) * tan(U[1]));
    double dt = 0.1;
    Vector4d X_next;
    X_next <<  
        X[0] + X[3] * cos(X[2] + beta) * dt,
        X[1] + X[3] * sin(X[2] + beta) * dt,
        X[2] + (X[3] / len) * tan(U[1]) * cos(beta) * dt,
        X[3] + U[0] * dt;
    return X_next;
}
int main() {
    Vector4d X,X1,X2;
    X << 1.111,1.111,1.111,1.111;
    Vector2d U0,U1;
    U0 << 0.111,0.111;
    Vector4d X_next = dynamics(X,U0);
    Vector4d X_next1 = dynamics(X,U0);
    std::cout<<X_next<<std::endl;
    std::cout<<X_next1<<std::endl;

}
