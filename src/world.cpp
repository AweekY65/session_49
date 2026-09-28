#include "rigid2d/world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigid2d {

namespace {

// Sleeping bodies are treated as infinite mass inside the solver.
inline double effectiveInverseMass(const Body& b) {
    return (!b.movable() || b.asleep) ? 0.0 : b.inverseMass;
}

}  // namespace

World::World(WorldConfig config) : cfg_(config) {}

Body* World::add(Body body) {
    bodies_.push_back(std::make_unique<Body>(std::move(body)));
    return bodies_.back().get();
}

void World::applyForce(Body* b, const Vec2& force) {
    if (b == nullptr || !b->movable()) return;
    b->wake();
    b->force += force;
}

void World::applyImpulse(Body* b, const Vec2& impulse) {
    if (b == nullptr || !b->movable()) return;
    b->wake();
    b->velocity += impulse * b->inverseMass;
}

void World::wakeAll() {
    for (auto& b : bodies_) b->wake();
}

void World::clearForces() {
    for (auto& b : bodies_) b->force = {0.0, 0.0};
}

// Semi-implicit (symplectic) Euler with a fixed step:
//   v += (g + F/m) * h
//   x += v * h
// The operation order is fixed, so identical inputs always yield
// identical (deterministic) results.
void World::integrateForces(Body& b, double dt) const {
    if (!b.movable() || b.asleep) return;
    b.velocity += (cfg_.gravity + b.force * b.inverseMass) * dt;
}

void World::integrateVelocity(Body& b, double dt) const {
    if (!b.movable() || b.asleep) return;
    b.position += b.velocity * dt;
}

double World::minDynamicFeature() const {
    double result = std::numeric_limits<double>::infinity();
    for (const auto& b : bodies_) {
        if (!b->movable() || b->asleep) continue;
        const double feature =
            b->type == ShapeType::Circle
                ? 2.0 * b->radius
                : 2.0 * std::min(b->half.x, b->half.y);
        result = std::min(result, feature);
    }
    return result;
}

void World::detectContacts() {
    contacts_.clear();
    const std::size_t n = bodies_.size();
    // Speculative-contact distance: manifolds survive gaps up to `slop`,
    // so a resting body that positional correction leaves a hair above the
    // surface does not look like a brand-new collision next substep.
    const double threshold = cfg_.slop;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            Body& a = *bodies_[i];
            Body& b = *bodies_[j];
            if (!a.movable() && !b.movable()) continue;
            Contact c;
            if (collide(a, b, c, threshold)) contacts_.push_back(c);
        }
    }
}

World::PersistentContact* World::findPersistent(Body* a, Body* b) {
    for (auto& pc : persistent_) {
        if ((pc.a == a && pc.b == b) || (pc.a == b && pc.b == a)) {
            return &pc;
        }
    }
    return nullptr;
}

// Synchronizes persistent solver state with the fresh manifolds:
//   * new pairs start a new collision episode (firstImpact = true) and get
//     a restitution *velocity bias* computed from the pre-warm-start
//     approach speed;
//   * surviving pairs keep their accumulated impulses for warm starting
//     and their used-up firstImpact flag (persistent contacts never
//     re-bounce);
//   * pairs that separated beyond the slop threshold are pruned.
void World::registerContacts() {
    for (const Contact& c : contacts_) {
        PersistentContact* pc = findPersistent(c.a, c.b);
        // Normal in the persistent record always points pc.a -> pc.b;
        // fresh manifolds from collide() already point a -> b, but the
        // stored pair may be in the opposite order.
        const Vec2 n = (pc != nullptr && pc->a == c.b) ? -c.normal : c.normal;
        const bool touching = c.gap <= 0.0;

        if (pc == nullptr) {
            PersistentContact fresh;
            fresh.a = c.a;
            fresh.b = c.b;
            fresh.normal = n;
            const Vec2 rv = c.b->velocity - c.a->velocity;
            const Vec2 tv = rv - n * dot(rv, n);
            fresh.tangent =
                lengthSquared(tv) > 0.0 ? normalized(tv) : Vec2{1.0, 0.0};
            fresh.touching = touching;
            fresh.firstImpact = true;
            fresh.normalImpulse = 0.0;
            fresh.biasImpulse = 0.0;
            fresh.tangentImpulse = 0.0;
            persistent_.push_back(fresh);
        } else {
            pc->normal = n;
            pc->touching = touching;
            // Surviving episode: no new bounce is owed.
            pc->firstImpact = false;
        }
    }
}

void World::pruneContacts() {
    persistent_.erase(
        std::remove_if(persistent_.begin(), persistent_.end(),
                       [this](const PersistentContact& pc) {
                           // Keep the episode while a manifold exists for
                           // the pair, even across a small speculative gap;
                           // drop it once they separate beyond the slop.
                           return std::none_of(
                               contacts_.begin(), contacts_.end(),
                               [&](const Contact& c) {
                                   return (c.a == pc.a && c.b == pc.b) ||
                                          (c.a == pc.b && c.b == pc.a);
                               });
                       }),
        persistent_.end());
}

// A sleeping body in contact with an awake body moving above the rest
// speed must wake (e.g. a falling object landing on a sleeping stack).
// Propagates to a fixed point so whole stacks wake correctly.
void World::wakeContactPartners() {
    bool changed = true;
    int guard = 0;
    const int maxPasses = static_cast<int>(bodies_.size()) + 1;
    while (changed && guard < maxPasses) {
        changed = false;
        ++guard;
        for (const Contact& c : contacts_) {
            if (!c.a->asleep && c.b->asleep && c.b->movable() &&
                length(c.a->velocity) > cfg_.restSpeed) {
                c.b->wake();
                changed = true;
            }
            if (!c.b->asleep && c.a->asleep && c.a->movable() &&
                length(c.b->velocity) > cfg_.restSpeed) {
                c.a->wake();
                changed = true;
            }
        }
    }
}

void World::warmStart() {
    for (PersistentContact& pc : persistent_) {
        Body& A = *pc.a;
        Body& B = *pc.b;
        const double invMA = effectiveInverseMass(A);
        const double invMB = effectiveInverseMass(B);
        // Only support and friction are warm-started; the one-shot
        // restitution impulse is not re-applied.
        const Vec2 imp = pc.normal * pc.normalImpulse +
                         pc.tangent * pc.tangentImpulse;
        if (invMA > 0.0) A.velocity -= imp * invMA;
        if (invMB > 0.0) B.velocity += imp * invMB;
    }
}

// Sequential-impulse velocity solve (two normal accumulators).
//
// Let rv = vB - vA and n point a -> b, so vn = dot(rv, n) < 0 while the
// bodies approach. Each contact keeps:
//   Pn   non-penetration SUPPORT impulse, warm-started, free to unload to
//        zero (a separating contact never sustains a compressive load);
//   Pb   one-shot RESTITUTION impulse, clamped >= 0, never unloaded, and
//        deliberately NOT warm-started (re-applying it would re-bounce a
//        resting contact);
//   Pt   Coulomb friction impulse, clamped |Pt| <= mu * Pn.
//
// Per iteration:
//   support : dPn = max(-Pn, -vn / (invMA + invMB))      (vn >= 0 -> 0)
//   friction: dPt = clamp(Pt - vt/(invMA+invMB), ...) - Pt
// Restitution is added once, on iteration 0 of a new episode approaching
// faster than restitutionThreshold:
//   Pb = max(0, (e*(-vn0) - vn) / (invMA + invMB))
// Velocity updates: vA -= P*invMA*axis ; vB += P*invMB*axis.
//
// Warm starting re-applies the previous substep's total impulses, so a
// resting stack already carries its weight support and converges in very
// few iterations without creeping downward or bouncing.
void World::solveVelocityImpulses() {
    for (int iter = 0; iter < cfg_.velocityIterations; ++iter) {
        for (PersistentContact& pc : persistent_) {
            Body& A = *pc.a;
            Body& B = *pc.b;
            const double invMA = effectiveInverseMass(A);
            const double invMB = effectiveInverseMass(B);
            const double invSum = invMA + invMB;
            if (invSum == 0.0) continue;

            const Vec2 rv = B.velocity - A.velocity;
            const double vn = dot(rv, pc.normal);

            // ---- Restitution: one-shot, iteration 0 of a new episode ----
            // Only a genuinely bouncy material (e > 0) and an impact above
            // the cutoff receive a bias impulse; e = 0 contacts get no
            // restitution at all (the support pass handles them).
            if (iter == 0 && pc.firstImpact &&
                -vn >= cfg_.restitutionThreshold) {
                const double e = std::max(A.restitution, B.restitution);
                if (e > 0.0) {
                    const double target = e * (-vn);
                    double dPb = (target - vn) / invSum;
                    dPb = std::max(0.0 - pc.biasImpulse, dPb);
                    pc.biasImpulse += dPb;
                    if (invMA > 0.0)
                        A.velocity -= pc.normal * (dPb * invMA);
                    if (invMB > 0.0)
                        B.velocity += pc.normal * (dPb * invMB);
                }
            }

            // ---- Non-penetration support (can unload, never pull) ----
            {
                const double vnNow =
                    dot(B.velocity - A.velocity, pc.normal);
                double dPn = -vnNow / invSum;
                dPn = std::max(-pc.normalImpulse, dPn);
                pc.normalImpulse += dPn;
                if (invMA > 0.0)
                    A.velocity -= pc.normal * (dPn * invMA);
                if (invMB > 0.0)
                    B.velocity += pc.normal * (dPn * invMB);
            }

            // ---- Coulomb friction (only while geometrically touching) ----
            if (pc.touching) {
                const double vt =
                    dot(B.velocity - A.velocity, pc.tangent);
                double dPt = -vt / invSum;
                const double mu =
                    std::sqrt(A.friction * B.friction);  // geometric mean
                const double maxFriction = mu * pc.normalImpulse;
                const double newPt =
                    std::max(-maxFriction,
                             std::min(maxFriction,
                                      pc.tangentImpulse + dPt));
                dPt = newPt - pc.tangentImpulse;
                pc.tangentImpulse = newPt;
                if (invMA > 0.0)
                    A.velocity -= pc.tangent * (dPt * invMA);
                if (invMB > 0.0)
                    B.velocity += pc.tangent * (dPt * invMB);
            }
        }
    }

    // Restitution for this episode has now been paid.
    for (PersistentContact& pc : persistent_) pc.firstImpact = false;
}

// Baumgarte positional correction, split from velocity so it never adds
// momentum ("split impulse"):
//   correctionMag = max(pen - slop, 0) / (invMA + invMB) * beta
//   xA -= correctionMag * invMA * n ;  xB += correctionMag * invMB * n
// Static/sleeping bodies (inverse mass 0) absorb none of the correction.
void World::solvePenetration() {
    for (int iter = 0; iter < cfg_.positionIterations; ++iter) {
        detectContacts();  // penetrations changed after bodies moved
        for (const Contact& c : contacts_) {
            if (c.gap >= 0.0) continue;  // speculative gap: nothing to fix
            Body& A = *c.a;
            Body& B = *c.b;
            const double invMA =
                A.movable() && !A.asleep ? A.inverseMass : 0.0;
            const double invMB =
                B.movable() && !B.asleep ? B.inverseMass : 0.0;
            const double invSum = invMA + invMB;
            if (invSum == 0.0) continue;

            const double correctionMag =
                std::max(-c.gap - cfg_.slop, 0.0) / invSum * cfg_.baumgarte;
            const Vec2 correction = c.normal * correctionMag;
            if (invMA > 0.0) A.position -= correction * invMA;
            if (invMB > 0.0) B.position += correction * invMB;
        }
    }
    // Final manifold (with speculative threshold) for post-step state.
    detectContacts();
}

// Freezes bodies that have been near rest long enough. A single body is
// slept when it stays slow; more importantly, a whole connected contact
// island that is supported (its contact graph reaches a static body) and
// whose slowest members have rested long enough is put to sleep together.
// This eliminates the residual micro-velocities that sequential impulses
// leave in a resting stack and prevents them from re-energizing it.
void World::updateSleepState(double dt) {
    if (!cfg_.allowSleep) return;

    const std::size_t n = bodies_.size();

    // Build contact islands with a simple union-find, tracking whether an
    // island touches static geometry (i.e. is supported from below).
    std::vector<int> parent(n);
    for (std::size_t i = 0; i < n; ++i) parent[i] = static_cast<int>(i);
    auto find = [&](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };
    auto unite = [&](int x, int y) {
        x = find(x);
        y = find(y);
        if (x != y) parent[y] = x;
    };
    auto indexOf = [&](const Body* t) {
        for (std::size_t i = 0; i < n; ++i)
            if (bodies_[i].get() == t) return static_cast<int>(i);
        return -1;
    };

    for (const Contact& c : contacts_) {
        const int ia = indexOf(c.a);
        const int ib = indexOf(c.b);
        if (ia >= 0 && ib >= 0) unite(ia, ib);
    }
    std::vector<bool> rootSupported(n, false);
    for (const Contact& c : contacts_) {
        if (c.a->isStatic || c.b->isStatic) {
            const int idx = indexOf(c.a->isStatic ? c.b : c.a);
            if (idx >= 0) rootSupported[find(idx)] = true;
        }
    }

    // Aggregate island properties (dynamic members only).
    std::vector<bool> hasDynamic(n, false);
    std::vector<double> maxSpeed(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        Body& b = *bodies_[i];
        if (!b.movable()) {
            b.asleep = b.isStatic;
            continue;
        }
        const int r = find(static_cast<int>(i));
        hasDynamic[r] = true;
        maxSpeed[r] = std::max(maxSpeed[r], length(b.velocity));
    }

    // Advance rest timers. A supported island accumulates rest whenever its
    // whole island is slow; a brief numerical spike merely *pauses* the
    // timer instead of wiping it, so jittery stacks still settle. A
    // genuinely unsupported body resets as usual.
    for (std::size_t i = 0; i < n; ++i) {
        Body& b = *bodies_[i];
        if (!b.movable() || b.asleep) continue;
        const int r = find(static_cast<int>(i));
        if (maxSpeed[r] <= cfg_.restSpeed) {
            b.restTimer += dt;
        } else if (!rootSupported[r]) {
            b.restTimer = 0.0;
        }
    }

    // Sleep whole supported islands once every member is slow and the
    // member with the shortest history has rested long enough. Bodies not
    // part of a supported island fall back to an individual timer check.
    std::vector<double> minTimer(n, std::numeric_limits<double>::infinity());
    for (std::size_t i = 0; i < n; ++i) {
        Body& b = *bodies_[i];
        if (!b.movable() || b.asleep) continue;
        minTimer[find(static_cast<int>(i))] =
            std::min(minTimer[find(static_cast<int>(i))], b.restTimer);
    }
    for (std::size_t i = 0; i < n; ++i) {
        Body& b = *bodies_[i];
        if (!b.movable() || b.asleep) continue;
        const int r = find(static_cast<int>(i));
        const bool sleepIsland = hasDynamic[r] && rootSupported[r] &&
                                 maxSpeed[r] <= cfg_.restSpeed &&
                                 minTimer[r] >= cfg_.sleepTime;
        const bool sleepSolo = !rootSupported[r] &&
                               length(b.velocity) <= cfg_.restSpeed &&
                               b.restTimer >= cfg_.sleepTime;
        if (sleepIsland || sleepSolo) {
            b.asleep = true;
            b.velocity = {0.0, 0.0};
            b.force = {0.0, 0.0};
        }
    }
}

void World::step() {
    // ---- Adaptive substeps ----
    // Bound per-substep travel so fast bodies do not tunnel. The substep
    // count depends only on positions/velocities (never on wall-clock or
    // randomness), preserving determinism.
    int substeps = 1;
    if (cfg_.adaptiveSubsteps) {
        double maxTravel = 0.0;
        for (const auto& b : bodies_) {
            if (b->movable() && !b->asleep) {
                maxTravel =
                    std::max(maxTravel, length(b->velocity) * cfg_.fixedDt);
            }
        }
        const double feature = minDynamicFeature();
        if (std::isfinite(feature) && feature > 0.0 && maxTravel > 0.0) {
            substeps = static_cast<int>(std::ceil(
                maxTravel / (cfg_.velocityFraction * feature)));
            substeps = std::max(1, std::min(substeps, cfg_.maxSubsteps));
        }
    }
    const double h = cfg_.fixedDt / substeps;

    for (int s = 0; s < substeps; ++s) {
        // 1. Gravity + accumulated external forces act on last substep's
        //    post-solve velocities.
        for (auto& b : bodies_) integrateForces(*b, h);

        // 3. Manifolds (speculative) + wake bodies struck by moving ones.
        detectContacts();
        wakeContactPartners();

        // 4. Open new collision episodes (their restitution bias is fixed
        //    from the pre-solve approach velocity).
        registerContacts();

        // 5. Warm-start from carried impulses (resting bodies re-inherit
        //    last substep's support), then refine normal + friction.
        warmStart();
        solveVelocityImpulses();

        // 6. Integrate positions.
        for (auto& b : bodies_) integrateVelocity(*b, h);

        // 7. Split-impulse positional stabilization (no added momentum),
        //    then drop episodes whose pair separated beyond the threshold.
        solvePenetration();
        pruneContacts();
    }

    updateSleepState(cfg_.fixedDt);
    clearForces();  // forces are single-step requests
}

double World::totalKineticEnergy() const {
    double total = 0.0;
    for (const auto& b : bodies_) {
        if (b->movable() && !b->asleep) {
            total += 0.5 * b->mass * lengthSquared(b->velocity);
        }
    }
    return total;
}

Vec2 World::totalMomentum() const {
    Vec2 p{0.0, 0.0};
    for (const auto& b : bodies_) {
        if (b->movable() && !b->asleep) p += b->velocity * b->mass;
    }
    return p;
}

}  // namespace rigid2d
