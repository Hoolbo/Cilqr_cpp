#include "utils.h"
#include <iostream>

int main() {
    try {
        std::cout << "Simple test starting..." << std::endl;
        
        // 测试基本的State创建
        State test_state(1, 2, 3, 4);
        std::cout << "State created: x=" << test_state[0] << ", y=" << test_state[1] << std::endl;
        
        // 测试Trajectory创建
        Trajectory test_traj;
        test_traj.push_back(test_state);
        std::cout << "Trajectory size: " << test_traj.states.size() << std::endl;
        
        std::cout << "Simple test completed successfully!" << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cout << "Unknown error occurred" << std::endl;
        return 1;
    }
    
    return 0;
}