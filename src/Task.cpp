#include "Task.h"
#include <fstream>
#include <iostream>
#include <iomanip>


Task::Task(int id, const std::string &description, float x, float y, int intensity, int task_intensity_increment, int increment_interval, int introduction_time)
    : id_(id), description_(description), location_({x, y}), intensity_(intensity), task_intensity_increment_(task_intensity_increment), increment_interval_(increment_interval), introduction_time_(introduction_time) {}
int Task::getId() const { return id_; }

Eigen::Vector2f Task::getLocation() const { return location_; }

int Task::getIncrementInterval() const { return increment_interval_; }

int Task::getIntroductionTime() const
{
    return introduction_time_;
}
void Task::incrementIntensity()
{
    intensity_ += task_intensity_increment_;
    writeToJSON();
}

void Task::decrementIntensity(int amount)
{
    intensity_ -= amount;
    if (intensity_ < 0)
    {
        intensity_ = 0; // Ensure intensity does not go below 0
    }
    writeToJSON();
}

nlohmann::json Task::toJSON() const
{
    nlohmann::json j;
    j["id"] = id_;
    j["description"] = description_;
    j["location"]["x"] = location_.x();
    j["location"]["y"] = location_.y();
    j["intensity"] = intensity_;
    j["task_intensity_increment"] = task_intensity_increment_;
    j["increment_interval"] = increment_interval_;
    return j;
}

std::string Task::getDescription() const { return description_; }

int Task::getIntensity() const { return intensity_; }

void Task::setBeingDecremented(bool flag) { being_decremented_ = flag; }

bool Task::isBeingDecremented() const { return being_decremented_; }

void Task::writeToJSON() const
{
    std::ifstream infile("../config/task_information_centre.json");
    nlohmann::json taskData;
    if (infile.is_open())
    {
        try
        {
            infile >> taskData;
        }
        catch (nlohmann::json::parse_error &e)
        {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            return;
        }
        infile.close();
    }

    taskData["tasks"][std::to_string(id_)] = toJSON();

    std::ofstream outfile("../config/task_information_centre.json");
    if (outfile.is_open())
    {
        outfile << std::setw(4) << taskData << std::endl;
        outfile.close();
    }
    else
    {
        std::cerr << "Error opening task file for writing: ../config/task_information_centre.json" << std::endl;
    }
}
