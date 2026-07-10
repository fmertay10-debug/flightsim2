#pragma once

#include <memory>
#include <vector>

#include "environment/Environment.h"
#include "sim/Entity.h"
#include "sim/SimObserver.h"
#include "sim/WorldView.h"

struct SimConfig {
    double dt          = 0.005;   // fixed timestep [s]
    double duration    = 60.0;    // max sim time [s]
    double groundLevel = 0.0;     // entities die below this altitude [m]
};

// Result of an intercept watch: closest approach between two entities, and
// whether it came inside the hit radius (which ends the run).
struct InterceptResult {
    bool    watching     = false;
    bool    hit          = false;
    double  missDistance = 1e30;   // closest approach so far [m]
    double  time         = 0.0;    // time of closest approach [s]
    Vector3 relPos;                // target - pursuer at closest approach [m, NED]
};

// The world: clock + environment + N entities + observers.
//
// step() is two-phase for multi-vehicle determinism:
//   1. snapshot every entity's state into a WorldView
//   2. every alive entity propagates from that same snapshot
//   3. all next-states commit together
// then observers are notified and ground impacts kill entities.
class Simulation {
public:
    Simulation(SimConfig config, std::unique_ptr<Environment> environment);

    int  addEntity(std::unique_ptr<Entity> entity);            // returns id
    void addObserver(std::unique_ptr<SimObserver> observer);

    // Track closest approach between two entities; end the run (and kill
    // both) when they pass inside hitRadius.
    void watchIntercept(int pursuerId, int targetId, double hitRadius);

    Entity& entity(int id) { return *entities_.at(static_cast<std::size_t>(id)); }

    bool step();   // one timestep; false when the run is over
    void run();    // step() until done, then notify observers via onFinish()

    double time() const { return time_; }
    const std::vector<std::unique_ptr<Entity>>& entities() const { return entities_; }
    const InterceptResult& interceptResult() const { return intercept_; }

private:
    WorldView snapshot() const;

    SimConfig config_;
    std::unique_ptr<Environment> environment_;
    std::vector<std::unique_ptr<Entity>>      entities_;
    std::vector<std::unique_ptr<SimObserver>> observers_;
    double time_ = 0.0;

    int             pursuerId_ = -1, targetId_ = -1;
    double          hitRadius_ = 0.0;
    InterceptResult intercept_;
};
