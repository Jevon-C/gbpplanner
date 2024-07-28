#include "ChargingStation.h"

ChargingStation::ChargingStation(int id, float x, float y, int charge_rate_interval, int charge_increment)
    : id_(id), location_(x, y), charge_rate_interval_(charge_rate_interval), charge_increment_(charge_increment) {}

int ChargingStation::getId() const { return id_; }
Eigen::Vector2f ChargingStation::getLocation() const { return location_; }
int ChargingStation::getChargeRateInterval() const { return charge_rate_interval_; }
int ChargingStation::getChargeIncrement() const { return charge_increment_; }

nlohmann::json ChargingStation::toJSON() const
{
    nlohmann::json j;
    j["id"] = id_;
    j["location"]["x"] = location_.x();
    j["location"]["y"] = location_.y();
    j["charge_rate_interval"] = charge_rate_interval_;
    j["charge_increment"] = charge_increment_;
    return j;
}
