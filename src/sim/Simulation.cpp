#include "sim/Simulation.h"

#include <cmath>
#include <cstdio>
#include <utility>

namespace {

// Every scalar the integrator advances; one NaN here means the physics blew
// up (bad table data, unstable integration) and would otherwise fly silently
// to the end of the run.
bool finiteState(const State& s) {
    const auto ok3 = [](const Vector3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    return ok3(s.position) && ok3(s.velocity) && ok3(s.angularRate) &&
           std::isfinite(s.attitude.w) && std::isfinite(s.attitude.x) &&
           std::isfinite(s.attitude.y) && std::isfinite(s.attitude.z);
}

} // namespace

Simulation::Simulation(SimConfig config, std::unique_ptr<Environment> environment)
    : config_(config), environment_(std::move(environment)) {}

int Simulation::addEntity(std::unique_ptr<Entity> entity) {
    const int id = static_cast<int>(entities_.size());
    entity->setId(id);
    entities_.push_back(std::move(entity));
    return id;
}

void Simulation::addObserver(std::unique_ptr<SimObserver> observer) {
    observers_.push_back(std::move(observer));
}

void Simulation::watchIntercept(int pursuerId, int targetId, double hitRadius) {
    pursuerId_ = pursuerId;
    targetId_  = targetId;
    hitRadius_ = hitRadius;
    intercept_.watching = true;
}

WorldView Simulation::snapshot() const {
    WorldView view;
    for (const auto& e : entities_)
        view.add({ e->id(), e->name(), e->state(), e->alive() });
    return view;
}

bool Simulation::step() {
    if (time_ >= config_.duration) return false;

    bool anyAlive = false;
    for (const auto& e : entities_)
        if (e->alive()) { anyAlive = true; break; }
    if (!anyAlive) return false;

    // Phase 1: everyone computes their next state from the same snapshot.
    const WorldView view = snapshot();
    std::vector<State> nextStates(entities_.size());
    for (std::size_t i = 0; i < entities_.size(); ++i)
        if (entities_[i]->alive())
            nextStates[i] = entities_[i]->propagate(*environment_, view, config_.dt);

    // Phase 2: commit all together.
    for (std::size_t i = 0; i < entities_.size(); ++i)
        if (entities_[i]->alive())
            entities_[i]->commit(nextStates[i]);

    time_ += config_.dt;

    // Divergence tripwire: kill loudly on the first non-finite committed
    // state instead of propagating NaN to a clean-looking end of run.
    for (const auto& e : entities_)
        if (e->alive() && !finiteState(e->state())) {
            std::fprintf(stderr,
                         "sim: entity '%s' diverged (non-finite state) at "
                         "t=%.3f s -- killed\n", e->name().c_str(), time_);
            e->kill();
        }

    // Notify observers with each entity's telemetry (recorded pre-commit,
    // so control/state/time are consistent).
    for (const auto& e : entities_) {
        if (!e->alive()) continue;
        for (const auto& obs : observers_)
            obs->onStep(e->telemetry());
    }

    // Intercept watch FIRST (before ground impact): a missile diving onto a
    // low or ground target reaches it at nearly ground level, so the hit must
    // register before the ground-impact kill would remove the pursuer.
    if (intercept_.watching && !intercept_.hit) {
        Entity& p = entity(pursuerId_);
        Entity& t = entity(targetId_);
        if (p.alive() && t.alive()) {
            const Vector3 rel = t.state().position - p.state().position;
            const double range = rel.norm();
            if (range < intercept_.missDistance) {
                intercept_.missDistance = range;
                intercept_.time = time_;
                intercept_.relPos = rel;
            }
            if (range <= hitRadius_) {
                intercept_.hit = true;
                p.kill();
                t.kill();
                return false;
            }
        }
    }

    // Ground impact.
    for (const auto& e : entities_)
        if (e->alive() && e->state().altitude() <= config_.groundLevel &&
            e->state().time > 0.0)
            e->kill();

    return true;
}

void Simulation::run() {
    while (step()) {}
    for (const auto& obs : observers_)
        obs->onFinish();
}
