#pragma once
#include <string>
#include <Eigen/Dense>
#include "json.hpp"

class Task
{
public:
    Task(int id, const std::string &description, float x, float y, int intensity, int task_intensity_increment, int increment_interval, int introduction_time = 0);

    int getId() const;
    Eigen::Vector2f getLocation() const;
    int getIncrementInterval() const;
    void incrementIntensity();
    void decrementIntensity(int amount);
    nlohmann::json toJSON() const;
    std::string getDescription() const;
    int getIntensity() const;
    void setBeingDecremented(bool flag);
    bool isBeingDecremented() const;
    void writeToJSON() const; 
    int getIntroductionTime() const;
    int introduction_time_;

private:
    int id_;
    std::string description_;
    Eigen::Vector2f location_;
    int intensity_;
    int task_intensity_increment_;
    int increment_interval_;
    bool being_decremented_;
};
