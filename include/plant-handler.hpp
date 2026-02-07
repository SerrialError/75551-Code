#ifndef PLANT_HANDLER_HPP
#define PLANT_HANDLER_HPP

#include <vector>
#include <mutex>
#include "pros/rtos.hpp"
#include "plant.hpp"

class PlantHandler {
private:
	std::vector<std::shared_ptr<Plant>> plants;
	pros::Mutex mutex;

	PlantHandler() {
		pros::Task([this] {
			while (true) {               
				// take a snapshot while locked          
				std::vector<std::shared_ptr<Plant>> snapshot;
               	{
                	std::lock_guard<pros::Mutex> lock(mutex);
                    snapshot = plants; // copies shared_ptrs cheaply
                }
				for (const auto& plant : snapshot) {
                	plant->update();
                }
				pros::delay(10);
			}
		});
	}
public:
	// Singleton Accessor
	static PlantHandler& get() {
		static PlantHandler instance;
       	return instance;
	}
	void addPlant(std::shared_ptr<Plant> plant) {
		std::lock_guard<pros::Mutex> lock(mutex);
		plants.push_back(std::move(plant));
	}
	// Prevents copying
	PlantHandler(const PlantHandler&) = delete;
	// Idiomatic deletion
	PlantHandler& operator=(const PlantHandler&) = delete;
	// Deletes move operations
	PlantHandler(PlantHandler&&) = delete;
	PlantHandler& operator=(PlantHandler&&) = delete;
};

#endif // plant-handler.hpp
