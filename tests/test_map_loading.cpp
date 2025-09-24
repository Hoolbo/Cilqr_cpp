#include "../include/utils.h"
#include <iostream>

int main() {
    std::cout << "Testing map loading..." << std::endl;
    
    // 测试加载地图
    MapData bitmap_map = load_bitmap_map("Maps/bitmap/B206_global_map.json");
    
    std::cout << "Map dimensions: " << bitmap_map.width << " x " << bitmap_map.height << std::endl;
    std::cout << "Resolution: " << bitmap_map.resolution << std::endl;
    std::cout << "Max elevation: " << bitmap_map.max_elevation << std::endl;
    std::cout << "Origin: [" << (bitmap_map.origin.size() > 0 ? bitmap_map.origin[0] : 0) 
              << ", " << (bitmap_map.origin.size() > 1 ? bitmap_map.origin[1] : 0) << "]" << std::endl;
    
    if (!bitmap_map.data.empty() && !bitmap_map.data[0].empty()) {
        std::cout << "Data loaded successfully. First few values: ";
        for (int i = 0; i < std::min(5, (int)bitmap_map.data[0].size()); ++i) {
            std::cout << bitmap_map.data[0][i] << " ";
        }
        std::cout << std::endl;
        
        // 计算有效数据点数量
        int valid_points = 0;
        int invalid_points = 0;
        for (const auto& row : bitmap_map.data) {
            for (double val : row) {
                if (val == -1.0) {
                    invalid_points++;
                } else {
                    valid_points++;
                }
            }
        }
        std::cout << "Valid points: " << valid_points << ", Invalid points: " << invalid_points << std::endl;
    } else {
        std::cout << "No data loaded!" << std::endl;
    }
    
    return 0;
}