#include "Task.h"

Task::Task(int id, std::string description, float x, float y, int intensity, int task_intensity_increment, int increment_interval)
    : id_(id), description_(description), location_(x, y), intensity_(intensity), task_intensity_increment_(task_intensity_increment), increment_interval_(increment_interval) {}

int Task::getId() const { return id_; }

Eigen::Vector2f Task::getLocation() const { return location_; }

int Task::getIncrementInterval() const { return increment_interval_; }

void Task::incrementIntensity() { intensity_ += task_intensity_increment_; }

void Task::decrementIntensity(int amount) { intensity_ -= amount; }

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

int Task::getIntensity() const { return intensity_; } // Add this method
