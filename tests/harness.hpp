// Test helpers for src/sim.hpp. Builds levels out of blocks, spikes, saws
// and slopes, gives each mode typical physics, and checks results.
//
// Numbers are approximate Geometry Dash 2.2 values at 240 physics steps per
// second (level units: 30 = one block). They only need to be plausible: the
// tests check the prediction's own geometry and decisions, not the game.
#pragma once

#include "../src/sim.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace test {

using namespace sim;

inline int g_failures = 0;
inline int g_checks = 0;

#define CHECK(cond, ...) do { \
    ++test::g_checks; \
    if (!(cond)) { \
        ++test::g_failures; \
        std::printf("FAIL %s:%d: %s\n  ", __FILE__, __LINE__, #cond); \
        std::printf(__VA_ARGS__); \
        std::printf("\n"); \
    } \
} while (0)

inline int finish(char const* name) {
    std::printf("%s: %d checks, %d failures\n", name, g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

constexpr float kBlock = 30.f;
constexpr float kSpeed1x = 311.58f / 240.f; // x per step at normal speed

// ---- level pieces ----

inline Obstacle block(float x, float y, float w = kBlock, float h = kBlock) {
    Obstacle o;
    o.rect = Rect(x, y, w, h);
    return o;
}

// A spike standing on y (pointing up), with the game's small hitbox: about
// 6 x 12 units in the middle of its block (a rough approximation).
inline Obstacle spike(float x, float y) {
    Obstacle o;
    o.hazard = true;
    o.rect = Rect(x + 12.f, y, 6.f, 12.f);
    return o;
}

// A spike hanging from a ceiling at y (pointing down).
inline Obstacle spikeDown(float x, float y) {
    Obstacle o;
    o.hazard = true;
    o.rect = Rect(x + 12.f, y - 12.f, 6.f, 12.f);
    return o;
}

inline Obstacle saw(float cx, float cy, float radius) {
    Obstacle o;
    o.hazard = true;
    o.radius = radius;
    o.center = { cx, cy };
    o.rect = Rect(cx - radius, cy - radius, radius * 2, radius * 2);
    return o;
}

// A slope in the rect (x, y, w, h). `rising`: the surface goes up to the
// right (else down). `floor`: solid below the surface (else above).
inline Obstacle slope(float x, float y, float w, float h, bool rising, bool floor, bool deadly = false) {
    Obstacle o;
    o.slope = true;
    o.floorTop = floor;
    o.deadly = deadly;
    o.rect = Rect(x, y, w, h);
    o.y0 = rising ? y : y + h;
    o.y1 = rising ? y + h : y;
    return o;
}

// ---- physics per mode ----

inline Physics physicsFor(Mode mode, bool mini = false) {
    Physics p;
    auto set = [](Estimate& e, float v) { e.value = v; e.count = 50; };
    float scale = mini ? 0.8f : 1.f;
    switch (mode) {
        case Mode::Ship:
            set(p.accelHeld, 0.02f); set(p.accelFree, -0.025f);
            p.maxUp = 1.6f; p.maxDown = 1.8f;
            break;
        case Mode::Ufo:
            set(p.impulse, 1.6f * scale); set(p.accelFree, -0.03f);
            p.maxUp = 2.f; p.maxDown = 2.f;
            break;
        case Mode::Swing:
            set(p.accelFree, -0.03f);
            p.maxUp = 1.8f; p.maxDown = 1.8f;
            break;
        case Mode::Cube:
            // About a 2-block-high, 100-step jump.
            set(p.impulse, 2.4f * scale); set(p.accelFree, -0.047f);
            p.maxUp = 2.4f; p.maxDown = 3.f;
            break;
        case Mode::Ball:
            set(p.impulse, 0.5f); set(p.accelFree, -0.047f);
            p.maxUp = 2.f; p.maxDown = 3.f;
            break;
        case Mode::Robot:
            set(p.impulse, 1.8f * scale); set(p.accelFree, -0.047f); set(p.accelHeld, -0.02f);
            p.maxUp = 2.4f; p.maxDown = 3.f;
            break;
        default: break;
    }
    return p;
}

// A player at (x, y) (hitbox center). Wave: hitbox about 10 x 10 (mini 6),
// others 30 x 30 (mini 18).
inline Player player(Mode mode, float x, float y, Physics const* physics, bool mini = false) {
    Player p;
    p.mode = mode;
    p.center = { x, y };
    float half = mode == Mode::Wave ? 5.f : 15.f;
    if (mini) half *= 0.6f;
    p.halfW = p.halfH = half;
    p.dx = kSpeed1x;
    p.waveSlope = mini ? 2.f : 1.f;
    p.physics = physics;
    return p;
}

// Runs a simulation and returns the path too.
struct Run {
    Result result;
    std::vector<Vec> path;
};

inline Run run(Player const& p, std::vector<Obstacle> const& level, bool heldBefore, int switchAt, int switchBackAt, int steps) {
    Run r;
    r.result = simulate(p, level, heldBefore, switchAt, switchBackAt, steps, &r.path);
    return r;
}

inline std::string describe(Run const& r) {
    char buf[128];
    std::snprintf(buf, sizeof buf, "survived %d (%s), end (%.1f, %.1f)", r.result.survived, r.result.cause,
        r.path.empty() ? 0.f : r.path.back().x, r.path.empty() ? 0.f : r.path.back().y);
    return buf;
}

// Largest vertical jump between two path points (teleports show up here).
inline float maxVerticalStep(Run const& r) {
    float m = 0.f;
    for (size_t i = 1; i < r.path.size(); ++i) m = std::max(m, std::abs(r.path[i].y - r.path[i - 1].y));
    return m;
}

} // namespace test
