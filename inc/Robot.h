/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <deque>
#include <Eigen/Dense>
#include <GBP/Variable.h>
#include <GBP/FactorGraph.h>
#include <GBP/Factor.h>
#include <raylib.h>
#include "Simulator.h"
#include "json.hpp"

class Robot : public FactorGraph
{
public:
    Robot(Simulator *sim,
          int rid,
          std::deque<Eigen::VectorXd> waypoints,
          float size,
          Color color);
    ~Robot();

    void updateCurrent();
    void updateHorizon();
    void updateInterrobotFactors();
    void createInterrobotFactors(std::shared_ptr<Robot> other_robot);
    void deleteInterrobotFactors(std::shared_ptr<Robot> other_robot);
    void draw();
    std::vector<int> getVariableTimesteps(int lookahead_horizon, int lookahead_multiple);
    void decrementBattery(); // Method to decrement battery level
    void writeBatteryToJSON(); // Method to write battery level to JSON

    Eigen::VectorXd position_; // Real position
    float height_3D_;
    float robot_radius_;
    std::deque<Eigen::VectorXd> waypoints_;
    Color color_;
    Simulator *sim_;
    int rid_;

    // Battery variables
    int battery_level;
    int battery_decrement;
    int decrement_interval;

    // Inter-robot communication
    bool interrobot_comms_active_ = true;
    std::vector<int> connected_r_ids_;
    std::vector<int> neighbours_;

private:
    int num_variables_;
};
