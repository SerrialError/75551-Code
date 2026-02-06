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
					{
						std::lock_guard<pros::Mutex> lock(mutex);
						for (const auto& plant : plants) {
                        	plant->update();
                    	}
					}
					pros::delay(10);
				}
			});
		}
	public:
		static PlantHandler& get() {
			static PlantHandler instance;
        	return instance;
		}
		void addPlant(std::shared_ptr<Plant> plant) {
			std::lock_guard<pros::Mutex> lock(mutex);
			plants.push_back(plant);
		}
		PlantHandler(const PlantHandler&) = delete;
    	void operator=(const PlantHandler&) = delete;
};

#endif // plant-handler.hpp
