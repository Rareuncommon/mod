// Path prediction for the lookahead, kept free of Geode and the game so it
// can be compiled and tested on its own (see tests/). main.cpp turns the
// game's objects and players into these types and calls simulate() and
// choose().
//
// Units are level units (30 = one block) and physics steps (240 per second).
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {

struct Vec {
    float x = 0.f, y = 0.f;
};

struct Size {
    float width = 0.f, height = 0.f;
};

struct Rect {
    Vec origin;
    Size size;

    Rect() = default;
    Rect(float x, float y, float w, float h) : origin { x, y }, size { w, h } {}

    float getMinX() const { return origin.x; }
    float getMaxX() const { return origin.x + size.width; }
    float getMinY() const { return origin.y; }
    float getMaxY() const { return origin.y + size.height; }
    float getMidX() const { return origin.x + size.width / 2; }
    float getMidY() const { return origin.y + size.height / 2; }

    // Same rule as cocos2d's CCRect::intersectsRect: touching edges count.
    bool intersectsRect(Rect const& o) const {
        return !(getMaxX() < o.getMinX() || o.getMaxX() < getMinX()
            || getMaxY() < o.getMinY() || o.getMaxY() < getMinY());
    }
};

inline float distance(Vec a, Vec b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

enum class Mode { None, Wave, Ship, Ufo, Swing, Cube, Ball, Robot };

// Modes where a click acts at once wherever the player is.
inline bool isFlyingMode(Mode mode) {
    return mode == Mode::Wave || mode == Mode::Ship || mode == Mode::Ufo || mode == Mode::Swing;
}

// Modes where a click acts at once on the ground (in the air, the normal
// buffer waits for landing instead).
inline bool isGroundMode(Mode mode) {
    return mode == Mode::Cube || mode == Mode::Ball || mode == Mode::Robot;
}

// A running estimate of one number, from measurements. Measurements far off
// the estimate (a bump into a block, a portal) are ignored once there are
// enough of them to trust the estimate. If many measurements in a row
// disagree with it, the estimate itself was wrong (its first measurements
// were bad), so it starts over from the new ones.
struct Estimate {
    float value = 0.f;
    int count = 0;
    int rejectedInRow = 0;
    void add(float v) {
        if (count >= 8 && std::abs(v - value) > std::max(0.5f * std::abs(value), 0.05f)) {
            if (++rejectedInRow < 6) return;
            count = 0;
        }
        rejectedInRow = 0;
        value = count == 0 ? v : value + (v - value) * 0.2f;
        count++;
    }
};

// Vertical physics learned while playing. All values are in level units per
// step, in the gravity frame (positive = away from gravity).
struct Physics {
    Estimate accelHeld;      // ship, robot: speed change per step while held
    Estimate accelFree;      // gravity / pull toward gravity (negative)
    Estimate impulse;        // UFO: speed on the step of a click; cube, robot:
                             // jump speed; ball: speed after a gravity flip
    float maxUp = 0.f;       // fastest speed seen away from gravity
    float maxDown = 0.f;     // fastest speed seen toward gravity

    bool ready(Mode mode) const {
        switch (mode) {
            case Mode::Ship: return accelHeld.count >= 5 && accelFree.count >= 5;
            case Mode::Ufo: return impulse.count >= 2 && accelFree.count >= 10;
            case Mode::Swing: return accelFree.count >= 10;
            case Mode::Cube:
            case Mode::Ball:
            case Mode::Robot: return impulse.count >= 2 && accelFree.count >= 10;
            default: return true;
        }
    }
};

// An object the player can hit.
struct Obstacle {
    Rect rect;
    Vec center;              // round hazards
    float radius = 0.f;      // > 0: round hazard
    bool hazard = false;     // else solid
    bool slope = false;      // a slope: solid on one side of a line
    bool floorTop = false;   // slope: solid below the line (else above)
    bool deadly = false;     // slope: kills on touch
    float y0 = 0.f, y1 = 0.f;// slope: line height at rect min x / max x

    float surfaceAt(float x) const {
        float w = rect.size.width;
        if (w <= 0.f) return y0;
        float t = std::clamp((x - rect.getMinX()) / w, 0.f, 1.f);
        return y0 + (y1 - y0) * t;
    }
};

constexpr float kInnerScale = 0.3f; // inner box used for block hits
constexpr float kFloorY = 90.f;     // top of the level's ground

// One player at the start of the prediction.
struct Player {
    Mode mode = Mode::None;
    Vec center;              // hitbox center
    float halfW = 0.f, halfH = 0.f;
    bool upsideDown = false;
    float dx = 0.f;          // x movement per step (measured last step)
    float lastDy = 0.f;      // y movement last step
    float waveSlope = 1.f;   // wave |dy| per |dx|
    bool grounded = false;   // ground modes: standing on something
    Physics const* physics = nullptr;
};

struct Result {
    int survived = 0;            // steps survived (== steps: no crash)
    char const* cause = "";      // what the crash hit
};

// Which way the wave is moving now: measured from the last step, or, while
// sliding (no vertical movement), from the button and gravity.
inline bool waveGoingUp(Player const& p, bool held) {
    if (p.lastDy > 0.01f) return true;
    if (p.lastDy < -0.01f) return false;
    return held != p.upsideDown;
}

// Simulates one player for `steps` steps. The button is `heldBefore` until
// step `switchAt`, then the other way; with `switchBackAt` >= 0 it goes back
// on that step.
//   wave:  moves diagonally, up or down by the button (its current
//          direction is the measured one until the first switch).
//   ship:  speed changes by the learned held / released acceleration.
//   UFO:   a press sets the learned jump speed, then gravity.
//   swing: a press flips gravity, then the learned pull.
//   cube, robot: holding on the ground jumps (again on every landing).
//   ball:  holding on the ground flips gravity.
// Speeds are capped at the fastest seen, and a non-wave player pushed out
// of a block, slope or the floor stops moving vertically.
inline Result simulate(
    Player const& p, std::vector<Obstacle> const& obstacles,
    bool heldBefore, int switchAt, int switchBackAt, int steps,
    std::vector<Vec>* path = nullptr
) {
    float dx = p.dx;
    float dy = std::abs(dx) * p.waveSlope;
    auto mode = p.mode;
    auto physics = p.physics;
    bool startUp = waveGoingUp(p, heldBefore);
    float g = p.upsideDown ? -1.f : 1.f;   // gravity frame
    float speed = g * p.lastDy;            // in the gravity frame
    bool grounded = isGroundMode(mode) && p.grounded;
    if (grounded) speed = 0.f;

    float halfW = p.halfW, halfH = p.halfH;
    float innerW = halfW * kInnerScale, innerH = halfH * kInnerScale;
    Vec center = p.center;
    if (path) path->push_back(center);

    for (int i = 0; i < steps; ++i) {
        bool up = startUp;
        if (i >= switchAt) up = !startUp;
        if (switchBackAt >= 0 && i >= switchBackAt) up = startUp;

        bool held = heldBefore;
        if (i >= switchAt) held = !heldBefore;
        if (switchBackAt >= 0 && i >= switchBackAt) held = heldBefore;
        bool pressed = held && ((i == switchAt) || (switchBackAt >= 0 && i == switchBackAt));

        float moveY;
        if (mode == Mode::Wave || !physics) {
            moveY = up ? dy : -dy;
        }
        else {
            switch (mode) {
                case Mode::Ship:
                    speed += held ? physics->accelHeld.value : physics->accelFree.value;
                    break;
                case Mode::Ufo:
                    if (pressed) speed = physics->impulse.value;
                    else speed += physics->accelFree.value;
                    break;
                case Mode::Swing:
                    if (pressed) {
                        g = -g;
                        speed = -speed;
                    }
                    speed += physics->accelFree.value;
                    break;
                case Mode::Cube:
                case Mode::Robot:
                    if (grounded && held) {
                        speed = physics->impulse.value;
                        grounded = false;
                    }
                    else if (mode == Mode::Robot && held && physics->accelHeld.count >= 5) {
                        speed += physics->accelHeld.value;
                    }
                    else speed += physics->accelFree.value;
                    break;
                case Mode::Ball:
                    if (grounded && held) {
                        g = -g;
                        speed = physics->impulse.value;
                        grounded = false;
                    }
                    else speed += physics->accelFree.value;
                    break;
                default: break;
            }
            if (physics->maxUp > 0.f) speed = std::min(speed, physics->maxUp);
            if (physics->maxDown > 0.f) speed = std::max(speed, -physics->maxDown);
            moveY = g * speed;
            up = moveY > 0.f;
        }

        float prevX = center.x, prevY = center.y;
        center.x += dx;
        center.y += moveY;
        float movedY = center.y;

        // Blocks the player came onto from above or below: last step it was
        // level with them in x and clear of them in y, so this step it
        // reached their top or bottom and slides along it. One it was already
        // beside (overlapping in y) is a side hit, handled below.
        for (auto const& o : obstacles) {
            if (o.hazard || o.slope) continue;
            bool wasOver = prevX + halfW > o.rect.getMinX() && prevX - halfW < o.rect.getMaxX();
            if (!wasOver) continue;
            Rect outer(center.x - halfW, center.y - halfH, halfW * 2, halfH * 2);
            if (!outer.intersectsRect(o.rect)) continue;
            bool wasBelow = prevY + halfH <= o.rect.getMinY() + 0.01f;
            bool wasAbove = prevY - halfH >= o.rect.getMaxY() - 0.01f;
            if (wasBelow) center.y = std::min(center.y, o.rect.getMinY() - halfH);
            else if (wasAbove) center.y = std::max(center.y, o.rect.getMaxY() + halfH);
        }
        center.y = std::max(center.y, kFloorY + halfH);

        // Blocks it reaches from the side: a crash if the inner box is level
        // with them, a corner graze otherwise.
        Rect inner(center.x - innerW, center.y - innerH, innerW * 2, innerH * 2);
        for (auto const& o : obstacles) {
            if (o.hazard || o.slope) continue;
            if (inner.intersectsRect(o.rect)) return { i, "a block" };
        }

        // Slopes: slides along one it moves parallel to, or one it lands on
        // that is flat or falling away in its direction (like the floor).
        // Running into the rising face crashes once the inner box is in;
        // touching a hazard slope crashes.
        {
            float waveRise = moveY / std::max(std::abs(dx), 0.001f);
            float dir = dx < 0.f ? -1.f : 1.f;
            for (auto const& o : obstacles) {
                if (!o.slope) continue;
                Rect box(center.x - halfW, center.y - halfH, halfW * 2, halfH * 2);
                if (!box.intersectsRect(o.rect)) continue;
                float left = center.x - halfW, right = center.x + halfW;
                // Surface rise per unit moved in the player's direction.
                float rise = (o.y1 - o.y0) / o.rect.size.width * dir;
                bool parallel = std::abs(waveRise - rise) < 0.2f;
                // Only a surface the player reached this step (within one
                // step's vertical move) can be landed on; deeper means it ran
                // into the slope's face from the side.
                float reach = std::abs(moveY) + 1.5f;
                bool centerOver = center.x >= o.rect.getMinX() && center.x <= o.rect.getMaxX();
                float mid = o.surfaceAt(center.x);
                if (o.floorTop) {
                    float surface = std::max(o.surfaceAt(left), o.surfaceAt(right));
                    if (center.y - halfH >= surface) continue;
                    if (o.deadly) return { i, "a hazard slope" };
                    bool landing = surface + halfH - center.y <= reach;
                    if (landing && (parallel || rise <= 0.05f)) {
                        center.y = surface + halfH;
                        continue;
                    }
                    if (centerOver && center.y - innerH < mid) return { i, "a slope" };
                }
                else {
                    float surface = std::min(o.surfaceAt(left), o.surfaceAt(right));
                    if (center.y + halfH <= surface) continue;
                    if (o.deadly) return { i, "a hazard slope" };
                    bool landing = center.y + halfH - surface <= reach;
                    if (landing && (parallel || rise >= -0.05f)) {
                        center.y = surface - halfH;
                        continue;
                    }
                    if (centerOver && center.y + innerH > mid) return { i, "a slope" };
                }
            }
        }

        // Pushed out by a block, slope or the floor: stop moving that way.
        if (mode != Mode::Wave && center.y != movedY) {
            // Pushed back against gravity: landed on something.
            grounded = g * moveY <= 0.f;
            speed = 0.f;
        }
        else grounded = false;
        if (path) path->push_back(center);

        Rect outer(center.x - halfW, center.y - halfH, halfW * 2, halfH * 2);
        for (auto const& o : obstacles) {
            if (!o.hazard) continue;
            if (o.radius > 0.f) {
                if (distance(o.center, center) <= halfW + o.radius) return { i, "a saw" };
            }
            else if (outer.intersectsRect(o.rect)) return { i, "a spike" };
        }
    }
    return { steps, "" };
}

struct Choice {
    int delay = 0;           // steps to wait before firing
    int survivedNow = 0;     // steps survived after firing now
    int survivedBest = 0;    // steps survived after firing at `delay`
    char const* causeNow = "";
    int lookahead = 0;       // steps looked at after the input
};

// A delay must survive at least this many more steps than firing now.
constexpr int kMinGain = 3;

// Picks when to fire an input (a press if `push`, else a release) for the
// players it drives. For each delay it measures how long the players survive
// after the input fires. Holding the button forever crashes in most parts,
// which moves with the delay and is no reason to wait; a delay is only worth
// it when it survives clearly longer than firing now. Delays where waiting
// itself crashes are skipped. With `followGap` > 0 the opposite input
// follows that many steps after the input (a tap).
inline Choice choose(
    std::vector<Player> const& players, std::vector<Obstacle> const& obstacles,
    bool push, int maxDelay, int followGap, int lookahead
) {
    bool held = !push; // button state until the input fires
    Choice choice;
    choice.lookahead = lookahead;
    int bestDelay = 0, best = -1;
    for (int delay = 0; delay <= maxDelay; ++delay) {
        int steps = lookahead + delay;
        int switchBack = followGap > 0 ? delay + followGap : -1;
        int survived = steps;
        char const* cause = "";
        for (auto const& player : players) {
            auto result = simulate(player, obstacles, held, delay, switchBack, steps);
            if (result.survived < survived) {
                survived = result.survived;
                cause = result.cause;
            }
        }
        if (survived < delay) break; // waiting this long crashes first
        int after = std::min(survived - delay, lookahead);
        if (delay == 0) {
            choice.survivedNow = after;
            choice.causeNow = cause;
        }
        if (after > best) {
            best = after;
            bestDelay = delay;
        }
        if (after >= lookahead) break; // can't do better
    }
    choice.survivedBest = std::max(best, 0);
    if (choice.survivedNow >= lookahead || best < choice.survivedNow + kMinGain) choice.delay = 0;
    else choice.delay = bestDelay;
    return choice;
}

} // namespace sim
