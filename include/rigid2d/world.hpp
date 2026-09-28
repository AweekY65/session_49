#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "rigid2d/body.hpp"
#include "rigid2d/collision.hpp"

namespace rigid2d {

struct WorldConfig {
    // Constant acceleration applied to every dynamic body each step.
    Vec2 gravity = {0.0, -9.81};

    // Fixed simulation step (seconds).
    double fixedDt = 1.0 / 60.0;

    // Velocity constraint iterations (normal impulse + friction).
    int velocityIterations = 8;

    // Positional correction iterations (Baumgarte stabilization).
    int positionIterations = 4;

    // Baumgarte stabilization: fraction of penetration corrected per
    // positional iteration.
    double baumgarte = 0.2;

    // Penetration slop: small overlap left in place so resting contacts
    // stay alive and the solver does not jitter. Also used as the contact
    // persistence distance (speculative contacts stay active across gaps
    // up to this size).
    double slop = 0.005;

    // Restitution only "kicks in" above this approach speed. Below it the
    // contact is treated as inelastic, which removes bounce from resting
    // contacts and prevents perpetual tiny bounces.
    double restitutionThreshold = 1.0;  // m/s

    // Adaptive substepping: each fixed step is subdivided so that a body
    // moves at most `velocityFraction * minShapeSize` per substep. This
    // bounds tunneling for fast objects while keeping slow stacks cheap.
    bool adaptiveSubsteps = true;
    int maxSubsteps = 64;
    double velocityFraction = 0.25;

    // Sleeping: a dynamic body whose speed stays below restSpeed for
    // sleepTime seconds is put to sleep (zero velocity, skipped by the
    // solver). Any collision with an awake moving body wakes it again.
    bool allowSleep = true;
    double restSpeed = 0.05;  // m/s
    double sleepTime = 0.5;   // seconds
};

// The simulation world owns all bodies. Everything lives in process
// memory; there is no I/O and no external engine dependency.
class World {
public:
    explicit World(WorldConfig config = {});

    // Adds a body to the world; returns a stable pointer (bodies are held
    // by unique_ptr, so adding more bodies never invalidates pointers).
    Body* add(Body body);

    std::vector<std::unique_ptr<Body>>& bodies() { return bodies_; }
    const std::vector<std::unique_ptr<Body>>& bodies() const {
        return bodies_;
    }

    const WorldConfig& config() const { return cfg_; }
    WorldConfig& mutableConfig() { return cfg_; }

    // Accumulates an external force for the next fixed step. Integrated
    // over every substep and cleared at the end of the step. Applying a
    // force to a static body is ignored.
    void applyForce(Body* b, const Vec2& force);

    // Applies an instantaneous velocity change J / m. Wakes the body.
    // Ignored for static bodies.
    void applyImpulse(Body* b, const Vec2& impulse);

    // Advances the simulation by exactly cfg.fixedDt seconds (internally
    // subdivided into substeps). Deterministic: identical state and calls
    // always produce identical results.
    void step();

    // Last manifolds found during the step (for inspection/tests).
    const std::vector<Contact>& contacts() const { return contacts_; }

    double totalKineticEnergy() const;
    Vec2 totalMomentum() const;

    void wakeAll();

private:
    // Persistent solver contact, keyed by body pair, surviving across
    // substeps and fixed steps while the pair stays within slop distance.
    // Carries accumulated impulses for warm starting and a firstImpact
    // flag ensuring restitution fires exactly once per collision episode.
    struct PersistentContact {
        Body* a;
        Body* b;
        Vec2 normal;           // a -> b
        Vec2 tangent;
        bool touching;         // currently overlapping (gap <= 0)
        bool firstImpact;      // restitution still owed this episode
        double normalImpulse;  // non-penetration support (warm-started)
        double biasImpulse;    // one-shot restitution impulse (warm-started)
        double tangentImpulse;
    };

    void integrateForces(Body& b, double dt) const;
    void integrateVelocity(Body& b, double dt) const;

    // Builds contacts_ from broad-phase pairs, using the contact threshold.
    void detectContacts();
    // Registers new collision episodes from the current manifolds (sets
    // their first-impact restitution bias) and refreshes surviving ones.
    void registerContacts();
    // Drops episodes whose pair no longer has a manifold (separated beyond
    // the speculative threshold). Never creates episodes.
    void pruneContacts();
    void warmStart();
    void solveVelocityImpulses();
    void solvePenetration();
    void updateSleepState(double dt);
    void wakeContactPartners();
    void clearForces();

    double minDynamicFeature() const;
    PersistentContact* findPersistent(Body* a, Body* b);

    WorldConfig cfg_;
    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<Contact> contacts_;
    std::vector<PersistentContact> persistent_;
};

}  // namespace rigid2d
