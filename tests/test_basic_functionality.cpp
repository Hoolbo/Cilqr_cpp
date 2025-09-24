#include <iostream>
#include "../include/ilqr.h"
#include "../include/utils.h"

int main() {
    std::cout << "Starting basic test..." << std::endl;
    
    try {
        // Test load_map function
        std::cout << "Loading map data..." << std::endl;
        std::vector<std::vector<double>> m_map_info = load_map();
        std::cout << "Map loaded successfully. Size: " << m_map_info.size() << std::endl;
        
        if (!m_map_info.empty()) {
            std::cout << "First dimension size: " << m_map_info[0].size() << std::endl;
        }
        
        // Test bitmap map loading
        std::cout << "Loading bitmap map..." << std::endl;
        MapData bitmap_map = load_bitmap_map("Maps/bitmap/B206_global_map.json");
        std::cout << "Bitmap map loaded: " << bitmap_map.width << " x " << bitmap_map.height << std::endl;
        
        // Test GlobalPlan
        std::cout << "Testing GlobalPlan..." << std::endl;
        GlobalPlan global_plan;
        std::vector<std::vector<double>> global_plan_log(3);
        
        // Add some test points
        for (int i = 0; i < 10; ++i) {
            global_plan_log[0].push_back(i * 0.1);
            global_plan_log[1].push_back(0.0);
            global_plan_log[2].push_back(0.0);
        }
        
        set_global_path(global_plan, global_plan_log);
        std::cout << "GlobalPlan test completed successfully." << std::endl;
        
        std::cout << "All tests passed!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}