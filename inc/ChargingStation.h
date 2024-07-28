#pragma once

#include <Eigen/Dense>
#include "json.hpp"

class ChargingStation
{
public:
    ChargingStation(int id, float x, float y, int charge_rate_interval, int charge_increment);

    int getId() const;
    Eigen::Vector2f getLocation() const;
    int getChargeRateInterval() const;
    int getChargeIncrement() const;
    nlohmann::json toJSON() const;

private:
    int id_;
    Eigen::Vector2f location_;
    int charge_rate_interval_;
    int charge_increment_;
};
