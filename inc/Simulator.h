#pragma once

#include <map>
#include <memory>
#include <algorithm>
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <Graphics.h>
#include <gbp/Variable.h>
#include <gbp/FactorGraph.h>
#include <nanoflann.h>
#include <raylib.h>
#include <rlights.h>
#include <KDTreeMapOfVectorsAdaptor.h>
#include <random>
#include <string>
#include <vector>
#include "Task.h"
#include "ChargingStation.h"
#include "json.hpp" // Added to ensure JSON support
#include <iostream> // Added for error logging
#include "Robot.h"

class Robot;
class Graphics;
class TreeOfRobots;

/************************************************************************************/
// The main Simulator. This is where the magic happens.
/************************************************************************************/
class Simulator
{
public:
    friend class Robot;
    friend class Factor;

    // Constructor
    Simulator();
    ~Simulator();

    // Pointer to Graphics class which holds all the camera, graphics, and models for display
    Graphics *graphics;

    // kd-tree to store the positions of the robots at each timestep.
    // This is used for calculating the neighbours of robots blazingly fast.
    typedef KDTreeMapOfVectorsAdaptor<std::map<int, std::vector<double>>> KDTree;
    std::map<int, std::vector<double>> robot_positions_{{0, {0., 0.}}};
    KDTree *treeOfRobots_;

    // Image representing the obstacles in the environment
    Image obstacleImg;

    int next_rid_ = 0;              // New robots will use this rid. It should be ++ incremented when this happens
    int next_vid_ = 0;              // New variables will use this vid. It should be ++ incremented when this happens
    int next_fid_ = 0;              // New factors will use this fid. It should be ++ incremented when this happens
    uint32_t clock_ = 0;            // Simulation clock (timesteps)
    bool new_robots_needed_ = true; // Whether or not to create new robots. (Some formations are dynamically changing)
    bool symmetric_factors = false; // If true, when inter-robot factors need to be created between two robots,
                                    // a pair of factors is created (one belonging to each robot). This becomes a redundancy.

    // Function to load tasks from JSON file
    void loadTasks(const std::string &filePath);

    // Function to load robots from JSON file
    void loadRobots(const std::string &filePath);

    // Function to load charging stations from JSON file
    void loadChargingStations(const std::string &filePath);

    // Function to increment the intensity of tasks at their respective intervals
    void incrementTaskIntensity();

    // Function to check proximity of robots to tasks and decrement task intensity accordingly
    void checkAndDecrementTaskIntensity();

    // Function to save the current state of tasks and robots to JSON files
    void saveStateToJSON();

    // Methods for handling dynamic tasks
    void loadDynamicTasks(const std::string &filePath);
    void checkAndIntroduceDynamicTasks();

    /*******************************************************************************/
    // Create new robots if needed. Handles deletion of robots out of bounds.
    // New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
    // by appending (push_back()) a shared pointer to a Robot class.
    /*******************************************************************************/
    void createOrDeleteRobots();

    /*******************************************************************************/
    // Set a proportion of robots to not perform inter-robot communications
    /*******************************************************************************/
    void setCommsFailure(float failure_rate = globals.COMMS_FAILURE_RATE);

    /*******************************************************************************/
    // Timestep loop of simulator.
    /*******************************************************************************/
    void timestep();

    /*******************************************************************************/
    // Drawing graphics.
    /*******************************************************************************/
    void draw();

    /*******************************************************************************/
    // Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
    // (Updates the neighbours_ of a robot)
    /*******************************************************************************/
    void calculateRobotNeighbours(std::map<int, std::shared_ptr<Robot>> &robots);

    /*******************************************************************************/
    // Handles keypresses and mouse input, and updates camera.
    /*******************************************************************************/
    void eventHandler();

    /*******************************************************************************/
    // Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
    /*******************************************************************************/
    void deleteRobot(std::shared_ptr<Robot> robot);

    /*******************************************************************************/
    // Method to decrement the battery level of all robots at their respective intervals
    /*******************************************************************************/
    void decrementBatteries();

    /*******************************************************************************/
    // Method to update the RIC of all robots
    /*******************************************************************************/
    void updateRIC();

    // Method to check proximity of robots to charging stations and increment battery level accordingly
    void checkAndChargeRobots();

    void initializeTemporalHistory();
    void recordTemporalHistory();
    nlohmann::json captureCurrentState(); // Helper function
    void removeCompletedTasks();          // Handler for the removal of completed tasks
    ChargingStation findNearestChargingStation(const Robot &robot);

    /*******************************************************************************/
    // RANDOM NUMBER GENERATOR.
    // Usage: random_number("normal", mean, sigma) or random_number("uniform", lower, upper)
    /*******************************************************************************/
    std::mt19937 gen_normal = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform_int = std::mt19937(globals.SEED);
    template <typename T>
    T random_number(std::string distribution, T param1, T param2)
    {
        if (distribution == "normal")
            return std::normal_distribution<T>(param1, param2)(gen_normal);
        if (distribution == "uniform")
            return std::uniform_real_distribution<T>(param1, param2)(gen_uniform);
        return (T)0;
    }
    int random_int(int lower, int upper)
    {
        return std::uniform_int_distribution<int>(lower, upper)(gen_uniform_int);
    }

    std::map<int, std::shared_ptr<Robot>> &getRobots() { return robots_; } // Getter function for robots_

private:
    std::vector<Task> tasks_;                      // Ensure the Task class is defined and included properly
    std::map<int, std::shared_ptr<Robot>> robots_; // Map containing smart pointers to all robots, accessed by their rid.
    std::vector<ChargingStation> charging_stations_;
    std::vector<Task> dynamic_tasks_;
};
