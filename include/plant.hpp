#ifndef PLANT_HPP
#define PLANT_HPP

#include <vector>
#include <memory>
#include "pros/device.hpp"

class Plant : public std::enable_shared_from_this<Plant> {
public:
    // default empty plant
    Plant() = default;

    // construct with devices only
    explicit Plant(std::vector<pros::v5::Device> devices)
        : devices_(std::move(devices)) {}

    // construct with subplants only
    explicit Plant(std::vector<std::shared_ptr<Plant>> subplants)
        : subplants_(std::move(subplants)) {}

    // construct with both devices and subplants
    Plant(std::vector<pros::v5::Device> devices,
          std::vector<std::shared_ptr<Plant>> subplants)
        : devices_(std::move(devices)), subplants_(std::move(subplants)) {}

    virtual ~Plant() = default;

    // interface
    virtual void update() = 0;

protected:
    std::vector<pros::v5::Device> devices_;
    std::vector<std::shared_ptr<Plant>> subplants_;
};

#endif // plant.hpp
