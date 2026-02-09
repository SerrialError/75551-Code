#ifndef MOTION_HPP
#define MOTION_HPP

#include <memory>

class Plant; // forward

class Motion {
public:
    enum class Result { Running, Completed, Cancelled };
    virtual ~Motion() = default;

    virtual void onStart() {}

    virtual Result step() = 0;

    virtual void onEnd() {}
private:
	std::weak_ptr<Plant> plant;
};
#endif // MOTION_HPP
