#pragma once

#include <string>
#include "ilqr.h"
#include "hybrid_astar.h"
#include "utils.h" // For VehicleModel if needed, but we focus on SystemModel

// Load configuration from a JSON file and populate the structures
void load_config(const std::string& main_config_file, 
                 const std::string& ilqr_config_file,
                 const std::string& ha_config_file,
                 Arg& arg, 
                 SystemModel& system_model, 
                 HybridAStarParams& ha_params,
                 RunConfig& run_config);
