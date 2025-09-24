#include <iostream>
#include "../include/utils.h"
#include "../include/ilqr.h"

int main() {
    std::cout << "Testing basic functionality..." << std::endl;
    
    try {
        // Test load_map function
        std::cout << "Loading map..." << std::endl;
        std::vector<std::vector<double>> map_info = load_map();
        std::cout << "Map loaded successfully. Size: " << map_info.size() << std::endl;
        if (!map_info.empty()) {
            std::cout << "First dimension size: " << map_info[0].size() << std::endl;
        }
        
        // Test load_bitmap_map function
        std::cout << "Loading bitmap map..." << std::endl;
        MapData bitmap_map = load_bitmap_map("Maps/bitmap/B206_global_map.json");
        std::cout << "Bitmap map loaded. Width: " << bitmap_map.width << ", Height: " << bitmap_map.height << std::endl;
        
        // Test basic vehicle initialization
        std::cout << "Initializing vehicle..." << std::endl;
        Vehicle ego;
        if (map_info.size() >= 3 && !map_info[0].empty()) {
            ego.set_state(map_info[0][300], map_info[1][300], map_info[2][300], 0);
            std::cout << "Vehicle initialized at: (" << map_info[0][300] << ", " << map_info[1][300] << ")" << std::endl;
        }
        
        std::cout << "All tests passed!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return 1;
    }
    
    return 0;
}