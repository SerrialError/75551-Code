#ifndef PLANT_HPP
#define PLANT_HPP

#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include "motion.hpp"

class Plant : public std::enable_shared_from_this<Plant> {
public:
    // default empty plant
    Plant() = default;

    // construct with devices only
    explicit Plant(std::vector<pros::v5::Device> devices_)
        : devices(std::move(devices_)) {}

    // construct with subplants only
    explicit Plant(std::vector<std::shared_ptr<Plant>> subplants_)
        : subplants(std::move(subplants_)) {}

    // construct with both devices and subplants
    Plant(std::vector<pros::v5::Device> devices_,
          std::vector<std::shared_ptr<Plant>> subplants_)
        : devices(std::move(devices_)), subplants(std::move(subplants_)) {}

    virtual ~Plant() = default;

    void update() {
        std::shared_ptr<Motion> current;
        {
            std::lock_guard<pros::Mutex> lock(mutex);
            if (!motions.empty()) current = motions.front();
        }

        if (current) {
            if (!started_current_) {
                current->onStart();
                started_current_ = true;
            }

            auto res = current->step();

            if (res == Motion::Result::Completed || res == Motion::Result::Cancelled) {
                current->onEnd();
                // remove the front motion if it's still the same (race-safe)
                std::lock_guard<pros::Mutex> lock(mutex);
                if (!motions.empty() && motions.front() == current) {
                    motions.pop_front();
                }
                started_current_ = false;
            }
        } else {
            // no active motion
            started_current_ = false;
        }
	}
    
	void addMotion(std::shared_ptr<Motion> motion) {
        std::lock_guard<pros::Mutex> lock(mutex);
        motions.push_back(std::move(motion));
    }

    // Push in front
    void pushMotionFront(std::shared_ptr<Motion> motion) {
        std::lock_guard<pros::Mutex> lock(mutex);
        motions.push_front(std::move(motion));
    }

    void cancelAllMotions() {
        std::lock_guard<pros::Mutex> lock(mutex);
        motions.clear();
    }

protected:
    std::vector<pros::v5::Device> devices;
    std::vector<std::shared_ptr<Plant>> subplants;

private:
    std::deque<std::shared_ptr<Motion>> motions;
    pros::Mutex mutex;
    bool started_current_ = false;
};

#endif // plant.hpp
