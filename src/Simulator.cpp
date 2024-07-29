#include "Simulator.h"
#include <fstream>
#include "json.hpp"
#include <iostream>
#include <gbp/GBPCore.h>
#include <Graphics.h>
#include <Robot.h>
#include <nanoflann.h>
#include <iomanip>

using json = nlohmann::json;

/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator()
{
    SetTraceLogLevel(LOG_ERROR);
    if (globals.DISPLAY)
    {
        SetTargetFPS(60);
        InitWindow(globals.SCREEN_SZ, globals.SCREEN_SZ, globals.WINDOW_TITLE);
    }

    // Initialise kdtree for storing robot positions (needed for nearest neighbour check)
    treeOfRobots_ = new KDTree(2, robot_positions_, 50);

    // Load tasks from JSON file
    loadTasks("../config/task_information_centre.json");

    // Load charging stations from JSON file
    loadChargingStations("../config/charging_stations.json");

  

    if (globals.use_dynamic_tasks)
    {
        loadDynamicTasks("../config/dynamic_tasks.json");
    }

    // For display only
    // User inputs an obstacle image where the obstacles are BLACK and background is WHITE.
    obstacleImg = LoadImage(globals.OBSTACLE_FILE.c_str());
    if (obstacleImg.width == 0)
        obstacleImg = GenImageColor(globals.WORLD_SZ, globals.WORLD_SZ, WHITE);

    // However, for calculation purposes the image needs to be inverted.
    ImageColorInvert(&obstacleImg);
    graphics = new Graphics(obstacleImg, tasks_, charging_stations_, robots_);
}

/*******************************************************************************/
// Destructor
/*******************************************************************************/
Simulator::~Simulator()
{
    delete treeOfRobots_;
    int n = robots_.size();
    for (int i = 0; i < n; ++i)
        robots_.erase(i);
    if (globals.DISPLAY)
    {
        delete graphics;
        CloseWindow();
    }
}

/*******************************************************************************/
// Function to load tasks from JSON file
/*******************************************************************************/
void Simulator::loadTasks(const std::string &filePath)
{
    std::ifstream taskFile(filePath);
    if (!taskFile.is_open())
    {
        std::cerr << "Error opening task file: " << filePath << std::endl;
        return;
    }

    json taskData;
    try
    {
        taskFile >> taskData;
    }
    catch (json::parse_error &e)
    {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return;
    }
    taskFile.close();

    for (const auto &taskEntry : taskData["tasks"].items())
    {
        int id = std::stoi(taskEntry.key());
        std::string description = taskEntry.value()["description"];
        float x = taskEntry.value()["location"]["x"];
        float y = taskEntry.value()["location"]["y"];
        int intensity = taskEntry.value()["intensity"];
        int task_intensity_increment = taskEntry.value()["task_intensity_increment"];
        int increment_interval = taskEntry.value()["increment_interval"];

        // Use the default value for introduction_time for these tasks
        tasks_.emplace_back(id, description, x, y, intensity, task_intensity_increment, increment_interval);
    }
}

/*******************************************************************************/
// Function to load robots from JSON file
/*******************************************************************************/
void Simulator::loadRobots(const std::string &filePath)
{
    std::ifstream robotFile(filePath);
    if (!robotFile.is_open())
    {
        std::cerr << "Error opening robot file: " << filePath << std::endl;
        return;
    }

    nlohmann::json robotData;
    try
    {
        robotFile >> robotData;
    }
    catch (nlohmann::json::parse_error &e)
    {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return;
    }
    robotFile.close();

    for (const auto &robotEntry : robotData["robots"].items())
    {
        int id = std::stoi(robotEntry.key());
        std::string entity_type = robotEntry.value()["entity_type"];
        float x = robotEntry.value()["location"]["x"];
        float y = robotEntry.value()["location"]["y"];
        float x_dot = robotEntry.value()["location"]["x_dot"];
        float y_dot = robotEntry.value()["location"]["y_dot"];
        int battery_level = robotEntry.value()["battery_level"];
        int battery_decrement = robotEntry.value()["battery_decrement"];
        int decrement_interval = robotEntry.value()["decrement_interval"];
        int assigned_task = robotEntry.value()["assigned_task"];
        int capacity = robotEntry.value()["capacity"];
        int capacity_interval = robotEntry.value()["capacity_interval"];

        robots_[id] = std::make_shared<Robot>(this, id, entity_type, x, y, x_dot, y_dot, battery_level, battery_decrement, decrement_interval, assigned_task, capacity, capacity_interval);
    }
}

/*******************************************************************************/
// Function to load charging stations from JSON file
/*******************************************************************************/
void Simulator::loadChargingStations(const std::string &filePath)
{
    std::ifstream chargingStationFile(filePath);
    if (!chargingStationFile.is_open())
    {
        std::cerr << "Error opening charging station file: " << filePath << std::endl;
        return;
    }

    json chargingStationData;
    try
    {
        chargingStationFile >> chargingStationData;
    }
    catch (json::parse_error &e)
    {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return;
    }
    chargingStationFile.close();

    for (const auto &stationEntry : chargingStationData["charging_stations"].items())
    {
        int id = std::stoi(stationEntry.key());
        float x = stationEntry.value()["location"]["x"];
        float y = stationEntry.value()["location"]["y"];
        int charge_rate_interval = stationEntry.value()["charge_rate_interval"];
        int charge_increment = stationEntry.value()["charge_increment"];

        charging_stations_.emplace_back(id, x, y, charge_rate_interval, charge_increment);
    }
}

// Method to load dynamic tasks from JSON file
void Simulator::loadDynamicTasks(const std::string &filePath)
{
    std::ifstream taskFile(filePath);
    if (!taskFile.is_open())
    {
        std::cerr << "Error opening dynamic task file: " << filePath << std::endl;
        return;
    }

    json taskData;
    try
    {
        taskFile >> taskData;
    }
    catch (json::parse_error &e)
    {
        std::cerr << "Error parsing dynamic tasks JSON: " << e.what() << std::endl;
        return;
    }
    taskFile.close();

    for (const auto &taskEntry : taskData["dynamic_tasks"])
    {
        int id = taskEntry["id"];
        std::string description = taskEntry["description"];
        float x = taskEntry["location"]["x"];
        float y = taskEntry["location"]["y"];
        int intensity = taskEntry["intensity"];
        int task_intensity_increment = taskEntry["task_intensity_increment"];
        int increment_interval = taskEntry["increment_interval"];
        int introduction_time = taskEntry["introduction_time"];

        dynamic_tasks_.emplace_back(id, description, x, y, intensity, task_intensity_increment, increment_interval, introduction_time);
    }
}

// Method to check and introduce dynamic tasks
void Simulator::checkAndIntroduceDynamicTasks()
{
    auto it = dynamic_tasks_.begin();
    while (it != dynamic_tasks_.end())
    {
        if (clock_ >= it->getIntroductionTime())
        {
            tasks_.emplace_back(*it);
            it = dynamic_tasks_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}
/*******************************************************************************/
// Increment the intensity of tasks at their respective intervals
/*******************************************************************************/
void Simulator::incrementTaskIntensity()
{
    for (auto &task : tasks_)
    {
        if (clock_ % task.getIncrementInterval() == 0 && !task.isBeingDecremented())
        { // Check if task is being decremented
            task.incrementIntensity();
            // std::cout << "Task ID: " << task.getId() << " Intensity incremented to: " << task.getIntensity() << std::endl;
        }
    }
}

/*******************************************************************************/
// Check proximity of robots to tasks and decrement task intensity accordingly
/*******************************************************************************/
void Simulator::checkAndDecrementTaskIntensity()
{
    for (auto &task : tasks_)
    {
        task.setBeingDecremented(false); // Reset the flag at the beginning of each check
    }

    // Set the flag for tasks that are within proximity of assigned robots with sufficient battery
    for (auto &[rid, robot] : robots_)
    {
        for (auto &task : tasks_)
        {
            if (robot->isWithinProximity(task.getLocation()) && robot->getAssignedTask() == task.getId() && robot->getBatteryLevel() > 0)
            {
                task.setBeingDecremented(true);
            }
        }
    }

    // Decrement the intensity for tasks being interacted with by robots with sufficient battery
    for (auto &[rid, robot] : robots_)
    {
        for (auto &task : tasks_)
        {
            if (task.isBeingDecremented() && robot->isWithinProximity(task.getLocation()) && robot->getAssignedTask() == task.getId())
            {
                if (clock_ % robot->getCapacityInterval() == 0)
                {
                    task.decrementIntensity(robot->getCapacity());
                    // std::cout << "Task ID: " << task.getId() << " Intensity decremented to: " << task.getIntensity()
                    // << " by Robot ID: " << robot->getId() << std::endl;
                }
            }
        }
    }
}

/*******************************************************************************/
// Check proximity of robots to charging stations and increment battery level accordingly
/*******************************************************************************/
void Simulator::checkAndChargeRobots()
{
    for (auto &[rid, robot] : robots_)
    {
        bool isCharging = false;
        for (const auto &station : charging_stations_)
        {
            if (robot->isWithinProximity(station.getLocation()))
            {
                isCharging = true;
                if (clock_ % station.getChargeRateInterval() == 0)
                {
                    robot->incrementBattery(station.getChargeIncrement());
                    // std::cout << "Robot ID: " << robot->getId() << " Battery incremented to: " << robot->getBatteryLevel() << std::endl;
                }
                break;
            }
        }
        robot->setCharging(isCharging); // Set charging status based on proximity to a charging station
    }
}

/*******************************************************************************/
// Save the current state of tasks and robots to JSON files
/*******************************************************************************/
void Simulator::saveStateToJSON()
{
    nlohmann::json taskData;
    for (const auto &task : tasks_)
    {
        taskData["tasks"][std::to_string(task.getId())] = task.toJSON();
    }

    std::ofstream taskFile("../config/task_information_centre.json");
    if (taskFile.is_open())
    {
        taskFile << std::setw(4) << taskData << std::endl;
        taskFile.close();
    }

    nlohmann::json robotData;
    for (const auto &[rid, robot] : robots_)
    {
        robotData["robots"][std::to_string(rid)] = robot->toJSON();
    }

    std::ofstream robotFile("../config/robot_information_centre.json");
    if (robotFile.is_open())
    {
        robotFile << std::setw(4) << robotData << std::endl;
        robotFile.close();
    }
}

/*******************************************************************************/
// Drawing graphics.
/*******************************************************************************/
void Simulator::draw()
{
    if (!globals.DISPLAY)
        return;

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(graphics->camera3d);

    // Draw Ground
    DrawModel(graphics->groundModel_, graphics->groundModelpos_, 1., WHITE);

    // Draw Tasks first
    for (const auto &task : tasks_)
    {
        if (task.getDescription() == "fire")
        {
            DrawSphere(Vector3{task.getLocation().x(), 0.5f, task.getLocation().y()}, 2.0f, graphics->fireColor_);
        }
        else if (task.getDescription() == "robbery")
        {
            DrawSphere(Vector3{task.getLocation().x(), 0.5f, task.getLocation().y()}, 2.0f, graphics->robberyColor_);
        }
        else if (task.getDescription() == "accident")
        {
            DrawSphere(Vector3{task.getLocation().x(), 0.5f, task.getLocation().y()}, 2.0f, graphics->accidentColor_);
        }
    }

    // Draw Charging Stations
    for (const auto &station : charging_stations_)
    {
        DrawSphere(Vector3{station.getLocation().x(), 0.5f, station.getLocation().y()}, 2.0f, GREEN);
    }

    // Draw Robots after tasks to render them above the tasks
    for (auto &[rid, robot] : robots_)
    {
        robot->draw();
    }

    EndMode3D();
    draw_info(clock_);
    EndDrawing();
}

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
void Simulator::timestep()
{
    if (globals.SIM_MODE != Timestep)
        return;

    // Create and/or destroy factors depending on a robot's neighbours
    calculateRobotNeighbours(robots_);
    for (auto &[r_id, robot] : robots_)
    {
        robot->updateInterrobotFactors();
    }

    // If the communications failure rate is non-zero, activate/deactivate robot comms
    setCommsFailure(globals.COMMS_FAILURE_RATE);

    // Perform iterations of GBP. Ideally the internal and external iterations
    // should be interleaved better. Here it is assumed there are an equal number.
    for (int i = 0; i < globals.NUM_ITERS; i++)
    {
        iterateGBP(1, INTERNAL, robots_);
        iterateGBP(1, EXTERNAL, robots_);
    }

    // Update the robot current and horizon states by one timestep
    for (auto &[r_id, robot] : robots_)
    {
        robot->updateHorizon();
        robot->updateCurrent();
    }

    // Decrement battery levels at the appropriate intervals
    decrementBatteries();

    // Increment task intensities at their respective intervals
    incrementTaskIntensity();

    // Check proximity of robots to tasks and decrement task intensity accordingly
    checkAndDecrementTaskIntensity();

    // Check proximity of robots to charging stations and increment battery level accordingly
    checkAndChargeRobots(); // Add this call

    // Update RIC at the specified intervals
    updateRIC();

    // Check and introduce dynamic tasks
    if (globals.use_dynamic_tasks)
    {
        checkAndIntroduceDynamicTasks();
    }

    // Increase simulation clock by one timestep
    clock_++;
    if (clock_ >= globals.MAX_TIME)
        globals.RUN = false;

    if (globals.record_temporal_history && clock_ % globals.temporal_history_interval == 0)
    {
        recordTemporalHistory();
    }

    // Save state to JSON after each timestep
    // saveStateToJSON();
}

/*******************************************************************************/
// Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
// (Updates the neighbours_ of a robot)
/*******************************************************************************/
void Simulator::calculateRobotNeighbours(std::map<int, std::shared_ptr<Robot>> &robots)
{
    for (auto &[rid, robot] : robots)
    {
        robot_positions_.at(rid) = std::vector<double>{robot->position_(0), robot->position_(1)};
    }
    treeOfRobots_->index->buildIndex();

    for (auto &[rid, robot] : robots)
    {
        // Find nearest neighbors in radius
        robot->neighbours_.clear();
        std::vector<double> query_pt = std::vector<double>{robots[rid]->position_(0), robots[rid]->position_(1)};
        const float search_radius = pow(globals.COMMUNICATION_RADIUS, 2.);
        std::vector<nanoflann::ResultItem<size_t, double>> matches;
        nanoflann::SearchParameters params;
        params.sorted = true;
        const size_t nMatches = treeOfRobots_->index->radiusSearch(&query_pt[0], search_radius, matches, params);
        for (size_t i = 0; i < nMatches; i++)
        {
            auto it = robots_.begin();
            std::advance(it, matches[i].first);
            if (it->first == rid)
                continue;
            robot->neighbours_.push_back(it->first);
        }
    }
}

/*******************************************************************************/
// Set a proportion of robots to not perform inter-robot communications
/*******************************************************************************/
void Simulator::setCommsFailure(float failure_rate)
{
    if (failure_rate == 0)
        return;
    // Get all the robot ids and then shuffle them
    std::vector<int> range{};
    for (auto &[rid, robot] : robots_)
        range.push_back(rid);
    std::shuffle(range.begin(), range.end(), gen_uniform);
    // Set a proportion of the robots as inactive using their interrobot_comms_active_ flag.
    int num_inactive = round(failure_rate * robots_.size());
    for (int i = 0; i < range.size(); i++)
    {
        robots_.at(range[i])->interrobot_comms_active_ = (i >= num_inactive);
    }
}

/*******************************************************************************/
// Handles keypresses and mouse input, and updates camera.
/*******************************************************************************/
void Simulator::eventHandler()
{
    // Deal with Keyboard key press
    int key = GetKeyPressed();
    switch (key)
    {
    case KEY_ESCAPE:
        globals.RUN = false;
        break;
    case KEY_H:
        globals.LAST_SIM_MODE = (globals.SIM_MODE == Help) ? globals.LAST_SIM_MODE : globals.SIM_MODE;
        globals.SIM_MODE = (globals.SIM_MODE == Help) ? globals.LAST_SIM_MODE : Help;
        break;
    case KEY_SPACE:
        graphics->camera_transition_ = !graphics->camera_transition_;
        break;
    case KEY_P:
        globals.DRAW_PATH = !globals.DRAW_PATH;
        break;
    case KEY_R:
        globals.DRAW_INTERROBOT = !globals.DRAW_INTERROBOT;
        break;
    case KEY_W:
        globals.DRAW_WAYPOINTS = !globals.DRAW_WAYPOINTS;
        break;
    case KEY_ENTER:
        globals.SIM_MODE = (globals.SIM_MODE == Timestep) ? SimNone : Timestep;
        break;
    default:
        break;
    }

    // Mouse input handling
    Ray ray = GetMouseRay(GetMousePosition(), graphics->camera3d);
    Vector3 mouse_gnd = Vector3Add(ray.position, Vector3Scale(ray.direction, -ray.position.y / ray.direction.y));
    Vector2 mouse_pos{mouse_gnd.x, mouse_gnd.z}; // Position on the ground plane
    // Do stuff with mouse here using mouse_pos .eg:
    // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
    //     do_code
    // }

    // Update the graphics if the camera has moved
    graphics->update_camera();
}

/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds.
// New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
// by appending (push_back()) a shared pointer to a Robot class.
/*******************************************************************************/
void Simulator::createOrDeleteRobots()
{
    if (!new_robots_needed_)
        return;

    std::vector<std::shared_ptr<Robot>> robots_to_create{};
    std::vector<std::shared_ptr<Robot>> robots_to_delete{};
    Eigen::VectorXd starting, turning, ending; // Waypoints: [x, y, xdot, ydot].

    // Read the JSON configuration file
    std::ifstream config_file("../config/robot_information_centre.json");
    if (!config_file.is_open())
    {
        std::cerr << "Error opening config file." << std::endl;
        return;
    }

    if (config_file.peek() == std::ifstream::traits_type::eof())
    {
        std::cerr << "Error: Config file is empty!" << std::endl;
        return;
    }

    nlohmann::json config_data;
    try
    {
        config_file >> config_data;
    }
    catch (nlohmann::json::parse_error &e)
    {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return;
    }

    if (globals.FORMATION == "circle")
    {
        // Robots must travel to opposite sides of the circle
        new_robots_needed_ = false;
        float min_circumference_spacing = 5. * globals.ROBOT_RADIUS;
        double min_radius = 0.25 * globals.WORLD_SZ;
        Eigen::VectorXd centre{{0., 0., 0., 0.}};
        for (int i = 0; i < globals.NUM_ROBOTS; i++)
        {
            // Select radius of large circle to be at least min_radius,
            // Also ensures that robots in the circle are at least min_circumference_spacing away from each other
            float radius_circle = (globals.NUM_ROBOTS == 1) ? min_radius : std::max(min_radius, sqrt(min_circumference_spacing / (2. - 2. * cos(2. * PI / (double)globals.NUM_ROBOTS))));
            Eigen::VectorXd offset_from_centre = Eigen::VectorXd{{radius_circle * cos(2. * PI * i / (float)globals.NUM_ROBOTS)},
                                                                 {radius_circle * sin(2. * PI * i / (float)globals.NUM_ROBOTS)},
                                                                 {0.},
                                                                 {0.}};
            starting = centre + offset_from_centre;
            ending = centre - offset_from_centre;
            std::deque<Eigen::VectorXd> waypoints{starting, ending};

            // Define robot radius and colour here.
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = ColorFromHSV(i * 360. / (float)globals.NUM_ROBOTS, 1., 0.75);
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color));
        }
    }
    else if (globals.FORMATION == "junction")
    {
        // Robots in a cross-roads style junction. There is only one-way traffic, and no turning.
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 20 == 0)
        { // Arbitrary condition on the simulation time to create new robots
            int n_roads = 2;
            int road = random_int(0, n_roads - 1);
            Eigen::Matrix4d rot;
            rot.setZero();
            rot.topLeftCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);
            rot.bottomRightCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);

            int n_lanes = 2;
            int lane = random_int(0, n_lanes - 1);
            double lane_width = 4. * globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5 * (1 - n_lanes) + lane) * lane_width;
            starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., lane_v_offset, globals.MAX_SPEED, 0.}};
            ending = rot * Eigen::VectorXd{{(double)globals.WORLD_SZ, lane_v_offset, 0., 0.}};
            std::deque<Eigen::VectorXd> waypoints{starting, ending};
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = DARKGREEN;
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color));
        }

        // Delete robots if out of bounds
        for (auto &[rid, robot] : robots_)
        {
            if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2)
            {
                robots_to_delete.push_back(robot);
            }
        }
    }
    else if (globals.FORMATION == "junction_twoway")
    {
        // Robots in a two-way junction, turning LEFT (RED), RIGHT (BLUE) or STRAIGHT (GREEN)
        new_robots_needed_ = true; // This is needed so that more robots can be created as the simulation progresses.
        if (clock_ % 20 == 0)
        { // Arbitrary condition on the simulation time to create new robots
            int n_roads = 4;
            int road = random_int(0, n_roads - 1);
            // We will define one road (the one going left) and then we can rotate the positions for other roads.
            Eigen::Matrix4d rot;
            rot.setZero();
            rot.topLeftCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);
            rot.bottomRightCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road), sin(PI / 2. * road), cos(PI / 2. * road);

            int n_lanes = 2;
            int lane = random_int(0, n_lanes - 1);
            int turn = random_int(0, 2);
            double lane_width = 4. * globals.ROBOT_RADIUS;
            double lane_v_offset = (0.5 * (1 - 2. * n_lanes) + lane) * lane_width;
            double lane_h_offset = (1 - turn) * (0.5 + lane - n_lanes) * lane_width;
            starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., lane_v_offset, globals.MAX_SPEED, 0.}};
            turning = rot * Eigen::VectorXd{{lane_h_offset, lane_v_offset, (turn % 2) * globals.MAX_SPEED, (turn - 1) * globals.MAX_SPEED}};
            ending = rot * Eigen::VectorXd{{lane_h_offset + (turn % 2) * globals.WORLD_SZ * 1., lane_v_offset + (turn - 1) * globals.WORLD_SZ * 1., 0., 0.}};
            std::deque<Eigen::VectorXd> waypoints{starting, turning, ending};
            float robot_radius = globals.ROBOT_RADIUS;
            Color robot_color = ColorFromHSV(turn * 120., 1., 0.75);
            robots_to_create.push_back(std::make_shared<Robot>(this, next_rid_++, waypoints, robot_radius, robot_color));
        }

        // Delete robots if out of bounds
        for (auto &[rid, robot] : robots_)
        {
            if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 || abs(robot->position_(1)) > globals.WORLD_SZ / 2)
            {
                robots_to_delete.push_back(robot);
            }
        }
    }
    else if (globals.FORMATION == "delegator")
    {
        new_robots_needed_ = false;
        for (int i = 0; i < globals.NUM_ROBOTS; i++)
        {
            // Assign current_rid to next_rid_ before incrementing it
            int current_rid = next_rid_++;

            // Extract starting and ending waypoint parameters from the JSON file using current_rid
            std::string current_rid_str = std::to_string(current_rid);
            // std::cout << "Debug: Checking Robot ID " << current_rid_str << " in JSON." << std::endl;

            if (config_data["robots"].contains(current_rid_str))
            {
                auto robot_data = config_data["robots"][current_rid_str];

                // Extract starting waypoint parameters
                double starting_waypoint_x = robot_data["starting_waypoint"]["x"].get<double>();
                double starting_waypoint_y = robot_data["starting_waypoint"]["y"].get<double>();
                double starting_waypoint_x_dot = robot_data["starting_waypoint"]["x_dot"].get<double>();
                double starting_waypoint_y_dot = robot_data["starting_waypoint"]["y_dot"].get<double>();

                // Debugging lines to print the extracted starting waypoint parameters
                /*
                std::cout << "Starting Waypoint - X: " << starting_waypoint_x
                          << ", Y: " << starting_waypoint_y
                          << ", X_dot: " << starting_waypoint_x_dot
                          << ", Y_dot: " << starting_waypoint_y_dot << std::endl;
                */

                // Extract ending waypoint parameters
                double ending_waypoint_x = robot_data["ending_waypoint"]["x"].get<double>();
                double ending_waypoint_y = robot_data["ending_waypoint"]["y"].get<double>();
                double ending_waypoint_x_dot = robot_data["ending_waypoint"]["x_dot"].get<double>();
                double ending_waypoint_y_dot = robot_data["ending_waypoint"]["y_dot"].get<double>();

                // Debugging lines to print the extracted ending waypoint parameters
                /*
                std::cout << "Ending Waypoint - X: " << ending_waypoint_x
                          << ", Y: " << ending_waypoint_y
                          << ", X_dot: " << ending_waypoint_x_dot
                          << ", Y_dot: " << ending_waypoint_y_dot << std::endl;
                */

                // Define starting waypoint
                Eigen::VectorXd starting = Eigen::VectorXd(4);
                starting << starting_waypoint_x,
                    starting_waypoint_y,
                    starting_waypoint_x_dot,
                    starting_waypoint_y_dot;

                // Define ending waypoint
                Eigen::VectorXd ending = Eigen::VectorXd(4);
                ending << ending_waypoint_x,
                    ending_waypoint_y,
                    ending_waypoint_x_dot,
                    ending_waypoint_y_dot;

                std::deque<Eigen::VectorXd> waypoints{starting, ending};

                // Define robot radius and colour here.
                float robot_radius = globals.ROBOT_RADIUS;
                Color robot_color = ColorFromHSV(i * 360.0 / (float)globals.NUM_ROBOTS, 1.0, 0.75);

                robots_to_create.push_back(std::make_shared<Robot>(this, current_rid, waypoints, robot_radius, robot_color));
            }
            else
            {
                std::cerr << "Error: Robot ID " << current_rid_str << " not found in JSON." << std::endl;
                return;
            }
        }
    }
    else
    {
        std::cerr << "Shouldn't reach here, formation not defined!" << std::endl;
    }

    // Create and/or delete the robots as necessary.
    for (auto robot : robots_to_create)
    {
        robot_positions_[robot->rid_] = std::vector<double>{robot->waypoints_[0](0), robot->waypoints_[0](1)};
        robots_[robot->rid_] = robot;
    }
    for (auto robot : robots_to_delete)
    {
        deleteRobot(robot);
    }
}

/*******************************************************************************/
// Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
/*******************************************************************************/
void Simulator::deleteRobot(std::shared_ptr<Robot> robot)
{
    auto connected_rids_copy = robot->connected_r_ids_;
    for (auto r : connected_rids_copy)
    {
        robot->deleteInterrobotFactors(robots_.at(r));
        robots_.at(r)->deleteInterrobotFactors(robot);
    }
    robots_.erase(robot->rid_);
    robot_positions_.erase(robot->rid_);
}

/*******************************************************************************/
// Method to decrement the battery level of all robots at their respective intervals
/*******************************************************************************/
void Simulator::decrementBatteries()
{
    for (auto &[rid, robot] : robots_)
    {
        if (clock_ % robot->getDecrementInterval() == 0)
        {
            robot->decrementBattery();
        }
    }
}

/*******************************************************************************/
// Method to update the RIC of all robots
/*******************************************************************************/
void Simulator::updateRIC()
{
    if (globals.real_time_updates && clock_ % globals.RIC_UPDATE_INTERVAL == 0)
    {
        std::ifstream infile("../config/robot_information_centre.json");
        if (!infile.is_open())
        {
            std::cerr << "Error opening config file for reading." << std::endl;
            return;
        }

        nlohmann::json j;
        try
        {
            infile >> j;
        }
        catch (nlohmann::json::parse_error &e)
        {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            return;
        }
        infile.close();

        for (auto &[rid, robot] : robots_)
        {
            std::string rid_str = std::to_string(rid);
            if (j["robots"].contains(rid_str))
            {
                j["robots"][rid_str]["battery_level"] = robot->getBatteryLevel();
            }
            else
            {
                std::cerr << "Error: Robot ID " << rid_str << " not found in JSON." << std::endl;
            }
        }

        std::ofstream outfile("../config/robot_information_centre.json");
        if (!outfile.is_open())
        {
            std::cerr << "Error opening config file for writing." << std::endl;
            return;
        }
        outfile << std::setw(4) << j << std::endl;
        outfile.close();
    }
}

/*******************************************************************************/
// Temporal history functions
/*******************************************************************************/

void Simulator::initializeTemporalHistory()
{
    nlohmann::json initialData;
    initialData["0"] = captureCurrentState();

    std::ofstream historyFile("../config/temporal_history.json");
    if (historyFile.is_open())
    {
        historyFile << std::setw(4) << initialData << std::endl;
        historyFile.close();
    }
}

// Function to capture the current state of robots and tasks
nlohmann::json Simulator::captureCurrentState()
{
    nlohmann::json state;

    // Capture the state of robots
    for (const auto &[rid, robot] : robots_)
    {
        state["robots"][std::to_string(rid)] = {
            {"battery_level", robot->getBatteryLevel()},
            {"location", {{"x", robot->getPosition().x()}, {"y", robot->getPosition().y()}}}};
    }

    // Capture the state of tasks
    for (const auto &task : tasks_)
    {
        state["tasks"][std::to_string(task.getId())] = {
            {"intensity", task.getIntensity()}};
    }

    return state;
}

// Function to record the state to the temporal history JSON file at regular intervals
void Simulator::recordTemporalHistory()
{
    std::ifstream infile("../config/temporal_history.json");
    nlohmann::json history;

    // Read the existing history if the file exists
    if (infile.is_open())
    {
        infile >> history;
        infile.close();
    }

    // Add the new record with the current timestamp as the key
    history[std::to_string(clock_)] = captureCurrentState();

    // Write the updated history back to the file
    std::ofstream outfile("../config/temporal_history.json");
    if (outfile.is_open())
    {
        outfile << std::setw(4) << history << std::endl;
        outfile.close();
    }
}