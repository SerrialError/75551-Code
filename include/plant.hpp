#ifndef PLANT_HPP
#define PLANT_HPP

#include <vector>
#include <memory>
#include "pros/device.hpp"

struct DevicesAndPlants {
    std::vector<pros::v5::Device> devices;
    std::vector<std::shared_ptr<class Plant>> subplants;

    DevicesAndPlants(
        std::vector<pros::v5::Device> devices_ = {},
        std::vector<std::shared_ptr<class Plant>> subplants_ = {}
    ) : devices(std::move(devices_)), subplants(std::move(subplants_)) {}
};

class Plant : public std::enable_shared_from_this<Plant> {
public:
    // Constructor from devices/subplants
    explicit Plant(const DevicesAndPlants& input)
        : devices(input.devices), subplants(input.subplants) {}

    // Default empty plant
    Plant() = default;

protected:
    std::vector<pros::v5::Device> devices;
    std::vector<std::shared_ptr<Plant>> subplants;
};

#endif // plant.hpp
