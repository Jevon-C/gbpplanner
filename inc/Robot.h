#pragma once

#include <deque>
#include <Eigen/Dense>
#include <gbp/Variable.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/FactorGraph.h> // Ensure this include is present and correct
#include <raylib.h>
#include "json.hpp"
#include <memory>
#include <vector>
#include <Utils.h>

// Forward declaration of Simulator to avoid circular dependency
class Simulator;

class Robot : public FactorGraph // Ensure FactorGraph is defined before this point
{
public:
    // Constructors and Destructor
    Robot(Simulator *sim, int rid, std::deque<Eigen::VectorXd> waypoints, float size, Color color);
    Robot(Simulator *sim, int rid, std::string entity_type, float x, float y, float x_dot, float y_dot,
          int battery_level, int battery_decrement, int decrement_interval, int assigned_task,
          int capacity, int capacity_interval);
    ~Robot();

    // Member functions
    void updateCurrent();
    void updateHorizon();
    void updateInterrobotFactors();
    void createInterrobotFactors(std::shared_ptr<Robot> other_robot);
    void deleteInterrobotFactors(std::shared_ptr<Robot> other_robot);
    void draw();
    std::vector<int> getVariableTimesteps(int lookahead_horizon, int lookahead_multiple);
    void decrementBattery();
    void incrementBattery(int amount);
    void writeBatteryToJSON();
    void writeCapacityToJSON();
    bool isWithinProximity(const Eigen::Vector2f &task_location) const;
    int getId() const;
    int getAssignedTask() const;
    int getCapacityInterval() const;
    int getCapacity() const;
    int getDecrementInterval() const;
    int getBatteryLevel() const;
    Eigen::Vector2f getPosition() const;  // Define getPosition method
    void setCharging(bool charging);
    bool isCharging() const;

    nlohmann::json toJSON() const;

    // Member variables
    Eigen::VectorXd position_; // Real position
    Eigen::VectorXd velocity_; // Velocity vector
    float height_3D_;
    float robot_radius_;
    std::deque<Eigen::VectorXd> waypoints_;
    Color color_;
    Simulator *sim_; // Pointer to Simulator instance
    int rid_;
    std::string entity_type_; // Entity type of the robot

    // Battery variables
    int battery_level_;
    int battery_decrement_;
    int decrement_interval_;

    // Task and capacity variables
    int assigned_task_;
    int capacity_;
    int capacity_interval_;

    // Inter-robot communication
    bool interrobot_comms_active_ = true;
    std::vector<int> connected_r_ids_;
    std::vector<int> neighbours_;

private:
    int num_variables_;
    bool charging_; 
};
