// Ground modes (cube, ball, robot) clicking on the ground: the jump / flip
// prediction and the delay chosen for a click that is a few steps early.
//
// Levels are built relative to a ground height G. Each scenario runs on:
//   floor    the level floor (y = kFloorY), G = kFloorY
//   raised   a wide ground block with its top at G = 300
//   mirror   the raised level mirrored about y = 500 with the player upside
//            down (must give the mirrored path exactly)
// in normal and mini size.
//
// Where the model's rules are known to differ from GD 2.2 (or GD's behaviour
// is uncertain) the comment says so; such checks test GD's behaviour when it
// is clear, else the model's own rule.
#include "harness.hpp"

#include <functional>

using namespace test;

namespace {

constexpr int kLook = 120;    // ground lookahead (kGroundLookahead in main.cpp)
constexpr int kMaxDelay = 10; // buffer frames
constexpr float kRaisedG = 300.f;
constexpr float kMirror = 500.f;

enum class Variant { Floor, Raised, Mirror };
char const* name(Variant v) { return v == Variant::Floor ? "floor" : v == Variant::Raised ? "raised" : "mirror"; }
char const* name(Mode m) { return m == Mode::Cube ? "cube" : m == Mode::Ball ? "ball" : m == Mode::Robot ? "robot" : "?"; }

Obstacle mirrored(Obstacle o, float m) {
    float y = o.rect.getMinY(), h = o.rect.size.height;
    o.rect.origin.y = 2 * m - (y + h);
    o.center.y = 2 * m - o.center.y;
    if (o.slope) {
        o.floorTop = !o.floorTop;
        o.y0 = 2 * m - o.y0;
        o.y1 = 2 * m - o.y1;
    }
    return o;
}

// Level pieces relative to the ground G (y offsets from G).
using Build = std::function<std::vector<Obstacle>(float G)>;

struct World {
    std::vector<Obstacle> level;
    Player p;
    float G;          // ground height in the world's own frame
    float sign;       // +1 normal, -1 mirrored (y = mirror(G + sign * h))
    float worldY(float h) const { return sign > 0 ? G + h : 2 * kMirror - (G + h); }
};

// A player standing on the ground of `build` at x (or at height `h` above G).
World makeWorld(Variant v, Mode mode, bool mini, Physics const* phys, Build const& build, float x, float h = 0.f) {
    World w;
    float G = v == Variant::Floor ? kFloorY : kRaisedG;
    w.G = G;
    w.sign = v == Variant::Mirror ? -1.f : 1.f;
    w.level = build(G);
    if (v != Variant::Floor) w.level.push_back(block(-2000.f, G - 60.f, 8000.f, 60.f));
    float half = mini ? 9.f : 15.f;
    w.p = player(mode, x, G + h + half, phys, mini);
    w.p.grounded = true;
    if (v == Variant::Mirror) {
        for (auto& o : w.level) o = mirrored(o, kMirror);
        w.p.center.y = 2 * kMirror - w.p.center.y;
        w.p.upsideDown = true;
    }
    return w;
}

Player at(Player p, int steps) {
    p.center.x += p.dx * steps;
    return p;
}

float maxStepAllowed(Physics const& ph) { return std::max(ph.maxUp, ph.maxDown) + 0.05f; }

// "Height above the ground" along a path in the world's gravity frame.
float heightOf(World const& w, float y) { return w.sign * (y - (w.sign > 0 ? w.G : 2 * kMirror - w.G)); }

struct EarlyResult { int earliest = -1; };

// The first jump step (from p) that survives `kLook` steps after the click
// and follows at least 6 steps whose jumps crash.
int earliestJump(Player const& p, std::vector<Obstacle> const& level, int followGap, int maxS = 250) {
    int crashesInRow = 0;
    for (int s = 0; s <= maxS; ++s) {
        int back = followGap > 0 ? s + followGap : -1;
        auto r = simulate(p, level, false, s, back, s + kLook);
        if (r.survived < s) return -1; // walked into something first
        if (r.survived == s + kLook) {
            if (crashesInRow >= 6) return s;
            crashesInRow = 0;
        }
        else crashesInRow++;
    }
    return -1;
}

// The core check for one scenario: a click 1..6 steps before the earliest
// surviving one crashes, and choose() delays it exactly onto that step; a
// click on that step fires at once.
void checkEarlyClicks(char const* what, World const& w, Physics const& ph, int followGap) {
    int s = earliestJump(w.p, w.level, followGap);
    CHECK(s >= 7, "%s: setup: earliest surviving jump step %d (need >= 7)", what, s);
    if (s < 7) return;
    for (int e = 0; e <= 6; ++e) {
        auto p = at(w.p, s - e);
        int back = followGap > 0 ? followGap : -1;
        auto now = run(p, w.level, false, 0, back, kLook);
        auto c = choose({ p }, w.level, true, kMaxDelay, followGap, kLook);
        CHECK(maxVerticalStep(now) <= maxStepAllowed(ph), "%s e=%d: teleport %.2f", what, e, maxVerticalStep(now));
        if (e == 0) {
            CHECK(now.result.survived == kLook, "%s on time: %s", what, describe(now).c_str());
            CHECK(c.delay == 0, "%s on time: delay %d (now %d)", what, c.delay, c.survivedNow);
        }
        else {
            CHECK(now.result.survived < kLook, "%s %d early should crash: %s", what, e, describe(now).c_str());
            CHECK(c.delay == e && c.survivedBest == kLook,
                "%s %d early: delay %d best %d now %d (%s)", what, e, c.delay, c.survivedBest, c.survivedNow, c.causeNow);
            if (c.delay > 0) {
                int b = followGap > 0 ? c.delay + followGap : -1;
                auto late = run(p, w.level, false, c.delay, b, kLook + c.delay);
                CHECK(late.result.survived == kLook + c.delay, "%s %d early, delayed: %s", what, e, describe(late).c_str());
            }
        }
    }
}

// The mirrored world must give the mirrored path.
void checkMirror(char const* what, Mode mode, bool mini, Physics const* ph, Build const& build, float x,
                 bool held, int switchAt, int back, int steps) {
    auto a = makeWorld(Variant::Raised, mode, mini, ph, build, x);
    auto b = makeWorld(Variant::Mirror, mode, mini, ph, build, x);
    auto ra = run(a.p, a.level, held, switchAt, back, steps);
    auto rb = run(b.p, b.level, held, switchAt, back, steps);
    CHECK(ra.result.survived == rb.result.survived, "%s mirror: survived %d vs %d", what, ra.result.survived, rb.result.survived);
    float worst = 0.f;
    for (size_t i = 0; i < std::min(ra.path.size(), rb.path.size()); ++i)
        worst = std::max(worst, std::abs(ra.path[i].y - (2 * kMirror - rb.path[i].y)));
    CHECK(worst < 0.01f, "%s mirror: paths differ by %.3f", what, worst);
}

// ---- level builders (x in level units, y relative to G) ----

Build spikes(int n, float x = 150.f) {
    return [=](float G) {
        std::vector<Obstacle> l;
        for (int i = 0; i < n; ++i) l.push_back(spike(x + i * kBlock, G));
        return l;
    };
}

} // namespace

int main() {
    Mode groundJumpers[] = { Mode::Cube, Mode::Robot };
    Variant variants[] = { Variant::Floor, Variant::Raised, Variant::Mirror };
    char what[160];

    // ---- standing still on the ground: no click survives at the same height ----
    for (Mode mode : { Mode::Cube, Mode::Robot, Mode::Ball }) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            for (Variant v : variants) {
                auto w = makeWorld(v, mode, mini, &ph, spikes(0), 0.f);
                auto r = run(w.p, w.level, false, 100000, -1, 300);
                float drift = 0.f;
                for (auto pt : r.path) drift = std::max(drift, std::abs(pt.y - w.p.center.y));
                std::snprintf(what, sizeof what, "%s%s %s idle", name(mode), mini ? " mini" : "", name(v));
                CHECK(r.result.survived == 300 && drift == 0.f, "%s: %s drift %.3f", what, describe(r).c_str(), drift);
            }
        }
    }

    // ---- early clicks before spikes / gaps / steps ----
    struct Scenario { char const* name; Build build; float x; };
    std::vector<Scenario> scenarios = {
        { "1 spike", spikes(1), 20.f },
        { "2 spikes", spikes(2), 20.f },
        { "3 spikes", spikes(3), 20.f },
        // A 1-block step with a spike on it a little way in.
        { "step + spike", [](float G) {
            return std::vector<Obstacle> { block(200.f, G, 600.f, 30.f), spike(330.f, G + 30.f) };
        }, 20.f },
        // Spikes right where a jump over a block lands.
        { "block, spikes after", [](float G) {
            return std::vector<Obstacle> { block(150.f, G, 30.f, 30.f), spike(240.f, G), spike(270.f, G), spike(300.f, G) };
        }, -100.f },
        // A gap of 3 blocks between 2-block-high platforms with spikes below.
        { "gap", [](float G) {
            std::vector<Obstacle> l { block(-400.f, G, 500.f, 60.f), block(190.f, G, 600.f, 60.f) };
            for (float x = 100.f; x < 190.f; x += kBlock) l.push_back(spike(x, G));
            return l;
        }, 20.f },
    };
    for (auto const& sc : scenarios) {
        for (Mode mode : groundJumpers) {
            for (bool mini : { false, true }) {
                auto ph = physicsFor(mode, mini);
                for (Variant v : variants) {
                    float h = std::string(sc.name) == "gap" ? 60.f : 0.f;
                    auto w = makeWorld(v, mode, mini, &ph, sc.build, sc.x, h);
                    std::snprintf(what, sizeof what, "%s %s%s %s", sc.name, name(mode), mini ? " mini" : "", name(v));
                    // Robot: a tap (held forever floats over everything).
                    checkEarlyClicks(what, w, ph, mode == Mode::Robot ? 25 : 0);
                }
                checkMirror(sc.name, mode, mini, &ph, sc.build, sc.x, false, 40, -1, 250);
            }
        }
    }

    // ---- the raised ground behaves like the floor ----
    for (Mode mode : groundJumpers) {
        auto ph = physicsFor(mode);
        auto a = makeWorld(Variant::Floor, mode, false, &ph, spikes(3), 20.f);
        auto b = makeWorld(Variant::Raised, mode, false, &ph, spikes(3), 20.f);
        for (int s : { 0, 40, 80, 100 }) {
            auto ra = run(a.p, a.level, false, s, -1, 250);
            auto rb = run(b.p, b.level, false, s, -1, 250);
            CHECK(ra.result.survived == rb.result.survived, "%s floor vs raised, jump at %d: %d vs %d",
                name(mode), s, ra.result.survived, rb.result.survived);
        }
    }

    // ---- a jump into an unavoidable wall: no delay ----
    for (Mode mode : groundJumpers) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            for (Variant v : variants) {
                auto w = makeWorld(v, mode, mini, &ph, [](float G) {
                    return std::vector<Obstacle> { block(150.f, G, 30.f, 150.f) };
                }, 20.f);
                std::snprintf(what, sizeof what, "%s%s %s wall", name(mode), mini ? " mini" : "", name(v));
                auto r = run(w.p, w.level, false, 0, -1, kLook);
                CHECK(r.result.survived < kLook && std::string(r.result.cause) == "a block", "%s: %s", what, describe(r).c_str());
                auto c = choose({ w.p }, w.level, true, kMaxDelay, 0, kLook);
                CHECK(c.delay == 0, "%s: delay %d", what, c.delay);
                // Walking into a 1-block wall without clicking crashes too
                // (GD: a cube running into a block side dies).
                auto w1 = makeWorld(v, mode, mini, &ph, [](float G) {
                    return std::vector<Obstacle> { block(150.f, G, 30.f, 30.f) };
                }, 20.f);
                auto r1 = run(w1.p, w1.level, false, 100000, -1, 200);
                CHECK(r1.result.survived < 200 && std::string(r1.result.cause) == "a block", "%s 1-block, walking: %s", what, describe(r1).c_str());
            }
        }
    }

    // A spike right in front: waiting crashes, clicking now is the only hope.
    {
        auto ph = physicsFor(Mode::Cube);
        auto w = makeWorld(Variant::Floor, Mode::Cube, false, &ph, spikes(1, 20.f), 0.f);
        auto c = choose({ w.p }, w.level, true, kMaxDelay, 0, kLook);
        CHECK(c.delay == 0, "spike in front: delay %d", c.delay);
    }

    // ---- walking off a ledge falls, then lands; held jumps again on landing ----
    for (Mode mode : { Mode::Cube, Mode::Robot, Mode::Ball }) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            for (Variant v : variants) {
                auto build = [](float G) { return std::vector<Obstacle> { block(-400.f, G, 500.f, 60.f) }; };
                auto w = makeWorld(v, mode, mini, &ph, build, 50.f, 60.f);
                std::snprintf(what, sizeof what, "%s%s %s ledge", name(mode), mini ? " mini" : "", name(v));
                auto r = run(w.p, w.level, false, 100000, -1, 200);
                CHECK(r.result.survived == 200, "%s: %s", what, describe(r).c_str());
                CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "%s: teleport %.2f", what, maxVerticalStep(r));
                float half = w.p.halfH;
                // Off the edge once the hitbox is clear of x = 100.
                int offStep = static_cast<int>((100.f + half - 50.f) / w.p.dx) + 1;
                float h0 = heightOf(w, w.p.center.y);
                float hLater = heightOf(w, r.path[std::min<size_t>(offStep + 20, r.path.size() - 1)].y);
                CHECK(hLater < h0 - 5.f, "%s: not falling 20 steps after the edge (%.2f -> %.2f)", what, h0, hLater);
                float hEnd = heightOf(w, r.path.back().y);
                CHECK(std::abs(hEnd - half) < 0.01f, "%s: should end on the ground, height %.2f", what, hEnd);
            }
        }
    }

    // ---- held: lands on a block top and jumps again; stairs ----
    for (Mode mode : groundJumpers) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            for (Variant v : variants) {
                auto build = [](float G) {
                    return std::vector<Obstacle> { block(100.f, G, 90.f, 15.f), block(190.f, G, 90.f, 30.f),
                        block(280.f, G, 1000.f, 45.f) };
                };
                auto w = makeWorld(v, mode, mini, &ph, build, 0.f);
                std::snprintf(what, sizeof what, "%s%s %s stairs held", name(mode), mini ? " mini" : "", name(v));
                auto r = run(w.p, w.level, false, 0, -1, 500);
                CHECK(r.result.survived == 500, "%s: %s", what, describe(r).c_str());
                CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "%s: teleport %.2f", what, maxVerticalStep(r));
                // Grounded on a step top at some point and left it upwards
                // at once (held), ending up on the top step.
                int groundedOnStep = 0, rejumps = 0;
                for (size_t i = 1; i + 1 < r.path.size(); ++i) {
                    float h = heightOf(w, r.path[i].y) - w.p.halfH;
                    float hp = heightOf(w, r.path[i - 1].y), hn = heightOf(w, r.path[i + 1].y);
                    bool landed = (std::abs(h - 15.f) < 0.01f || std::abs(h - 30.f) < 0.01f || std::abs(h - 45.f) < 0.01f);
                    float hc = heightOf(w, r.path[i].y);
                    // A landing: came down onto a step height and did not go
                    // on falling. Held, it jumps again at once (one step later
                    // if the fall ended exactly flush with the top: no push).
                    if (landed && hp > hc && hn >= hc) {
                        groundedOnStep++;
                        float hn2 = i + 2 < r.path.size() ? heightOf(w, r.path[i + 2].y) : hn;
                        if (hn > hc || hn2 > hc) rejumps++;
                    }
                }
                CHECK(groundedOnStep >= 2 && rejumps == groundedOnStep, "%s: landings on steps %d, rejumps %d",
                    what, groundedOnStep, rejumps);
                CHECK(heightOf(w, r.path.back().y) >= 45.f + w.p.halfH - 0.01f, "%s: end height %.2f", what,
                    heightOf(w, r.path.back().y));
            }
            checkMirror("stairs", mode, mini, &ph, [](float G) {
                return std::vector<Obstacle> { block(100.f, G, 90.f, 15.f), block(190.f, G, 90.f, 30.f),
                    block(280.f, G, 1000.f, 45.f) };
            }, 0.f, false, 0, -1, 500);
        }
    }

    // ---- jumping into a low ceiling ----
    // Model: a head-hit stops the jump (pushed below the block) and the cube
    // falls back. GD 2.2: a cube does not slide on a block bottom; it dies
    // if its inner box reaches the block (head-hit deeper than about
    // halfH * 0.7 = 10.5 units), and survives a shallower bonk. Here the
    // ceiling is 1 block above the cube's head, so the full jump would go
    // ~30 units into it: in GD that kills. Checked against GD (see report).
    for (Mode mode : groundJumpers) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            for (Variant v : variants) {
                float half = mini ? 9.f : 15.f;
                float gapAbove = 30.f;
                auto build = [=](float G) { return std::vector<Obstacle> { block(-400.f, G + 2 * half + gapAbove, 2000.f, 30.f) }; };
                auto w = makeWorld(v, mode, mini, &ph, build, 0.f);
                std::snprintf(what, sizeof what, "%s%s %s low ceiling", name(mode), mini ? " mini" : "", name(v));
                auto r = run(w.p, w.level, false, 0, -1, 200);
                CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "%s: teleport %.2f", what, maxVerticalStep(r));
                float top = 0.f;
                for (auto pt : r.path) top = std::max(top, heightOf(w, pt.y));
                // Either it crashed (GD) or it stayed below the ceiling
                // (the model's slide): never through it.
                CHECK(r.result.survived < 200 || top <= 2 * half + gapAbove - half + 0.01f,
                    "%s: went into the ceiling, height %.2f", what, top);
                CHECK(top <= 2 * half + gapAbove + 0.01f, "%s: went past the ceiling, height %.2f", what, top);
                // GD: deep head-hit with the inner box kills a cube.
                CHECK(r.result.survived < 200, "%s (GD: cube dies on a deep head-hit): %s", what, describe(r).c_str());
            }
        }
    }

    // ---- landing on a block's corner from above ----
    // Falling onto a block where the first step that overlaps it in x also
    // takes the bottom below its top: the player was above the block last
    // step, so it lands (GD snaps onto the top; at most one step's fall).
    for (Mode mode : groundJumpers) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            float half = mini ? 9.f : 15.f;
            std::vector<Obstacle> level { block(100.f, 90.f, 300.f, 60.f) };
            auto p = player(mode, 100.f - half - 0.5f, 150.f + half + 1.f, &ph, mini);
            p.lastDy = -3.f;
            auto r = run(p, level, false, 100000, -1, 100);
            std::snprintf(what, sizeof what, "%s%s corner landing", name(mode), mini ? " mini" : "");
            CHECK(r.result.survived == 100 && std::abs(r.path.back().y - (150.f + half)) < 0.01f, "%s: %s", what, describe(r).c_str());
        }
    }

    // ---- grazing a block's side on the way up: no teleport ----
    {
        auto ph = physicsFor(Mode::Cube);
        // Block whose top is 4 units above the cube's bottom at the moment
        // the hitbox reaches it, approached while rising: outer box grazes,
        // inner box clear.
        for (float x0 : { 60.f, 65.f, 70.f, 75.f, 80.f }) {
            std::vector<Obstacle> level { block(100.f, 90.f, 30.f, 45.f) };
            auto p = player(Mode::Cube, x0, 105.f, &ph);
            p.grounded = true;
            auto r = run(p, level, false, 0, -1, 200);
            CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "graze x0=%.0f: teleport %.2f (%s)", x0, maxVerticalStep(r), describe(r).c_str());
        }
    }

    // ---- slopes ----
    // GD: cube, robot and ball run up 1:1 and 1:2 slopes and can land on
    // them. Model check: walking onto a rising slope then a block at its top.
    for (Mode mode : { Mode::Cube, Mode::Robot, Mode::Ball }) {
        for (bool mini : { false, true }) {
            auto ph = physicsFor(mode, mini);
            for (Variant v : variants) {
                for (float w : { 30.f, 60.f }) {
                    auto build = [=](float G) {
                        return std::vector<Obstacle> { slope(150.f, G, w, 30.f, true, true), block(150.f + w, G, 600.f, 30.f) };
                    };
                    auto wd = makeWorld(v, mode, mini, &ph, build, 20.f);
                    std::snprintf(what, sizeof what, "%s%s %s walk up slope %.0f", name(mode), mini ? " mini" : "", name(v), w);
                    auto r = run(wd.p, wd.level, false, 100000, -1, 300);
                    CHECK(r.result.survived == 300, "%s: %s", what, describe(r).c_str());
                    CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "%s: teleport %.2f", what, maxVerticalStep(r));
                    CHECK(std::abs(heightOf(wd, r.path.back().y) - (30.f + wd.p.halfH)) < 0.01f, "%s: end height %.2f",
                        what, heightOf(wd, r.path.back().y));
                }
                // Downhill: walk off a 1-block block onto a slope falling to the ground.
                auto build = [](float G) {
                    return std::vector<Obstacle> { block(-400.f, G, 500.f, 30.f), slope(100.f, G, 60.f, 30.f, false, true) };
                };
                auto wd = makeWorld(v, mode, mini, &ph, build, 50.f, 30.f);
                std::snprintf(what, sizeof what, "%s%s %s walk down slope", name(mode), mini ? " mini" : "", name(v));
                auto r = run(wd.p, wd.level, false, 100000, -1, 200);
                CHECK(r.result.survived == 200, "%s: %s", what, describe(r).c_str());
                CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "%s: teleport %.2f", what, maxVerticalStep(r));
                CHECK(std::abs(heightOf(wd, r.path.back().y) - wd.p.halfH) < 0.01f, "%s: end height %.2f", what, heightOf(wd, r.path.back().y));
            }
        }
    }
    // Jumping onto a rising slope (cube): lands on it and survives.
    for (Variant v : variants) {
        auto ph = physicsFor(Mode::Cube);
        auto build = [](float G) {
            return std::vector<Obstacle> { slope(120.f, G, 120.f, 60.f, true, true), block(240.f, G, 600.f, 60.f) };
        };
        auto wd = makeWorld(v, Mode::Cube, false, &ph, build, 20.f);
        std::snprintf(what, sizeof what, "cube %s jump onto slope", name(v));
        auto r = run(wd.p, wd.level, false, 30, 31, 300);
        CHECK(r.result.survived == 300, "%s: %s", what, describe(r).c_str());
        CHECK(maxVerticalStep(r) <= maxStepAllowed(ph), "%s: teleport %.2f", what, maxVerticalStep(r));
    }

    // ---- ball ----
    // Floor and a ceiling 3 blocks above the ground.
    for (bool mini : { false, true }) {
        auto ph = physicsFor(Mode::Ball, mini);
        for (Variant v : variants) {
            float half = mini ? 9.f : 15.f;
            auto build = [](float G) { return std::vector<Obstacle> { block(-400.f, G + 90.f, 3000.f, 30.f) }; };
            auto w = makeWorld(v, Mode::Ball, mini, &ph, build, 0.f);
            std::snprintf(what, sizeof what, "ball%s %s ceiling", mini ? " mini" : "", name(v));
            // Tap: flips once, rolls along the ceiling.
            auto tap = run(w.p, w.level, false, 0, 1, 300);
            CHECK(tap.result.survived == 300, "%s tap: %s", what, describe(tap).c_str());
            CHECK(std::abs(heightOf(w, tap.path.back().y) - (90.f - half)) < 0.01f, "%s tap: end height %.2f", what, heightOf(w, tap.path.back().y));
            CHECK(maxVerticalStep(tap) <= maxStepAllowed(ph), "%s tap: teleport %.2f", what, maxVerticalStep(tap));
            // The first step of a flip must not go into the ground it leaves.
            CHECK(heightOf(w, tap.path[1].y) >= half - 0.01f, "%s tap: step 1 height %.2f", what, heightOf(w, tap.path[1].y));
            // Held: model flips again on every landing (ping-pong).
            auto held = run(w.p, w.level, false, 0, -1, 400);
            int touchCeil = 0, touchFloor = 0;
            bool onCeil = false, onFloor = true;
            for (auto pt : held.path) {
                float h = heightOf(w, pt.y);
                bool c = std::abs(h - (90.f - half)) < 0.01f, f = std::abs(h - half) < 0.01f;
                if (c && !onCeil) touchCeil++;
                if (f && !onFloor) touchFloor++;
                onCeil = c; onFloor = f;
            }
            CHECK(held.result.survived == 400 && touchCeil >= 1 && touchFloor >= 1,
                "%s held: %s, ceiling %d floor %d", what, describe(held).c_str(), touchCeil, touchFloor);
        }
        checkMirror("ball ceiling", Mode::Ball, mini, &ph, [](float G) {
            return std::vector<Obstacle> { block(-400.f, G + 90.f, 3000.f, 30.f) };
        }, 0.f, false, 10, 11, 300);
        // Spikes hanging from the ceiling where an early flip lands.
        for (Variant v : variants) {
            auto build = [](float G) {
                std::vector<Obstacle> l { block(-400.f, G + 90.f, 3000.f, 30.f) };
                for (float x = 60.f; x < 300.f; x += kBlock) l.push_back(spikeDown(x, G + 90.f));
                return l;
            };
            auto w = makeWorld(v, Mode::Ball, mini, &ph, build, 0.f);
            std::snprintf(what, sizeof what, "ball%s %s ceiling spikes", mini ? " mini" : "", name(v));
            checkEarlyClicks(what, w, ph, 1);
        }
    }

    // ---- robot: short tap vs long hold ----
    for (bool mini : { false, true }) {
        auto ph = physicsFor(Mode::Robot, mini);
        for (Variant v : variants) {
            auto w = makeWorld(v, Mode::Robot, mini, &ph, spikes(0), 0.f);
            auto tap = run(w.p, w.level, false, 0, 5, 200);
            auto hold = run(w.p, w.level, false, 0, 60, 200);
            float pTap = 0.f, pHold = 0.f;
            for (auto pt : tap.path) pTap = std::max(pTap, heightOf(w, pt.y));
            for (auto pt : hold.path) pHold = std::max(pHold, heightOf(w, pt.y));
            std::snprintf(what, sizeof what, "robot%s %s", mini ? " mini" : "", name(v));
            CHECK(pHold > pTap + 15.f, "%s: hold peak %.1f vs tap %.1f", what, pHold, pTap);
            // A saw at the long jump's peak: the tap clears under it, the hold hits it.
            float peakX = 0.f;
            for (auto pt : hold.path) if (heightOf(w, pt.y) == pHold) peakX = pt.x;
            float sawY = w.worldY(pHold + 10.f);
            std::vector<Obstacle> level = w.level;
            level.push_back(saw(peakX, sawY, 10.f));
            auto tap2 = run(w.p, level, false, 0, 5, 200);
            auto hold2 = run(w.p, level, false, 0, 60, 200);
            CHECK(tap2.result.survived == 200, "%s tap under saw: %s", what, describe(tap2).c_str());
            CHECK(hold2.result.survived < 200, "%s hold into saw: %s", what, describe(hold2).c_str());
            // choose() with the tap's length follows the tap.
            auto c = choose({ w.p }, level, true, kMaxDelay, 5, kLook);
            CHECK(c.delay == 0 && c.survivedNow == kLook, "%s tap choose: delay %d now %d", what, c.delay, c.survivedNow);
        }
        // Early taps over 3 spikes.
        for (Variant v : variants) {
            auto w = makeWorld(v, Mode::Robot, mini, &ph, spikes(3), 20.f);
            std::snprintf(what, sizeof what, "robot%s %s tap 3 spikes", mini ? " mini" : "", name(v));
            checkEarlyClicks(what, w, ph, 25);
        }
    }

    return finish("test_ground");
}
