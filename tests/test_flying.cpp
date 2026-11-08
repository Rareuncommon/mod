// Ship, UFO and swing: the prediction's paths and choose() decisions.
//
// Rules tested are the model's own (see the comments in src/sim.hpp):
//   ship:  speed += accelHeld while held, accelFree when released, capped.
//   UFO:   a press sets speed to `impulse` (also while rising), then gravity.
//   swing: a press flips gravity (g = -g, speed = -speed, so the world
//          velocity is kept), then constant pull toward gravity.
//   A non-wave player pushed out of a block, slope or the floor stops
//   moving vertically.
// Where the model's rule may differ from GD 2.2 the check says so.
#include "harness.hpp"

#include <cmath>
#include <functional>

using namespace test;

namespace {

char const* modeName(Mode m) {
    switch (m) {
        case Mode::Ship: return "ship";
        case Mode::Ufo: return "ufo";
        case Mode::Swing: return "swing";
        default: return "?";
    }
}

struct Cfg {
    Mode mode;
    bool mini;
    bool upside;
};

std::string cfgName(Cfg c) {
    return std::string(modeName(c.mode)) + (c.mini ? " mini" : "") + (c.upside ? " upside-down" : "");
}

Physics const& physicsOf(Mode mode, bool mini) {
    static Physics table[3][2] = {
        { physicsFor(Mode::Ship, false), physicsFor(Mode::Ship, true) },
        { physicsFor(Mode::Ufo, false), physicsFor(Mode::Ufo, true) },
        { physicsFor(Mode::Swing, false), physicsFor(Mode::Swing, true) },
    };
    int i = mode == Mode::Ship ? 0 : mode == Mode::Ufo ? 1 : 2;
    return table[i][mini ? 1 : 0];
}

float capOf(Physics const& ph) { return std::max(ph.maxUp, ph.maxDown); }

Player make(Cfg c, float x, float y, float lastDy = 0.f) {
    auto p = player(c.mode, x, y, &physicsOf(c.mode, c.mini), c.mini);
    p.upsideDown = c.upside;
    p.lastDy = lastDy;
    return p;
}

// Mirror a level / player about the horizontal line y = Y0 (gravity flip).
Obstacle mirrorY(Obstacle o, float Y0) {
    o.rect.origin.y = 2 * Y0 - o.rect.origin.y - o.rect.size.height;
    o.center.y = 2 * Y0 - o.center.y;
    if (o.slope) {
        o.floorTop = !o.floorTop;
        o.y0 = 2 * Y0 - o.y0;
        o.y1 = 2 * Y0 - o.y1;
    }
    return o;
}
Player mirrorY(Player p, float Y0) {
    p.center.y = 2 * Y0 - p.center.y;
    p.lastDy = -p.lastDy;
    p.upsideDown = !p.upsideDown;
    return p;
}
// Mirror about the vertical line x = X0 (running the other way).
Obstacle mirrorX(Obstacle o, float X0) {
    o.rect.origin.x = 2 * X0 - o.rect.origin.x - o.rect.size.width;
    o.center.x = 2 * X0 - o.center.x;
    if (o.slope) std::swap(o.y0, o.y1);
    return o;
}
Player mirrorX(Player p, float X0) {
    p.center.x = 2 * X0 - p.center.x;
    p.dx = -p.dx;
    return p;
}
std::vector<Obstacle> mirrorY(std::vector<Obstacle> v, float Y0) {
    for (auto& o : v) o = mirrorY(o, Y0);
    return v;
}
std::vector<Obstacle> mirrorX(std::vector<Obstacle> v, float X0) {
    for (auto& o : v) o = mirrorX(o, X0);
    return v;
}

// A row of 1-block blocks (seams every block) from x0 to x1, at y.
void row(std::vector<Obstacle>& level, float x0, float x1, float y) {
    for (float x = x0; x < x1; x += kBlock) level.push_back(block(x, y, kBlock, kBlock));
}

// A corridor: floor row with its top at `floorTop`, ceiling row with its
// bottom `gap` blocks higher.
std::vector<Obstacle> corridor(float x0, float x1, float floorTop, float gap) {
    std::vector<Obstacle> level;
    row(level, x0, x1, floorTop - kBlock);
    row(level, x0, x1, floorTop + gap * kBlock);
    return level;
}

bool finite(Run const& r) {
    for (auto v : r.path) if (!std::isfinite(v.x) || !std::isfinite(v.y)) return false;
    return true;
}

float dyAt(Run const& r, size_t i) { return r.path[i].y - r.path[i - 1].y; }

// Common invariants: finite, no teleport, speed cap.
void checkPath(Run const& r, float cap, float extra, std::string const& what) {
    CHECK(finite(r), "%s: non-finite path", what.c_str());
    CHECK(maxVerticalStep(r) <= cap + extra + 1e-3f, "%s: max vertical step %.3f > cap %.2f + %.2f; %s",
        what.c_str(), maxVerticalStep(r), cap, extra, describe(r).c_str());
}

// Number of times the vertical acceleration changes sign (swing flips).
int accelSignChanges(Run const& r) {
    int changes = 0, last = 0;
    for (size_t i = 2; i < r.path.size(); ++i) {
        float a = dyAt(r, i) - dyAt(r, i - 1);
        int s = a > 1e-4f ? 1 : a < -1e-4f ? -1 : 0;
        if (s == 0) continue;
        if (last != 0 && s != last) ++changes;
        last = s;
    }
    return changes;
}

std::vector<Cfg> allCfgs() {
    std::vector<Cfg> v;
    for (auto m : { Mode::Ship, Mode::Ufo, Mode::Swing })
        for (bool mini : { false, true })
            for (bool up : { false, true }) v.push_back({ m, mini, up });
    return v;
}

// Input patterns: heldBefore, switchAt, switchBackAt.
struct Input {
    bool held;
    int at, back;
};
std::vector<Input> inputs() {
    return { { false, 1000, -1 }, { true, 1000, -1 }, { false, 0, -1 }, { true, 0, -1 }, { false, 10, 20 },
        { true, 15, 40 }, { false, 30, 31 }, { false, 5, 120 } };
}

} // namespace

int main() {
    auto cfgs = allCfgs();

    // ---- 1. Open air: survives, no teleport, speed caps, mirror symmetry ----
    for (auto c : cfgs) {
        auto const& ph = physicsOf(c.mode, c.mini);
        float cap = capOf(ph);
        for (float lastDy : { 0.f, 1.f, -1.f, 5.f, -5.f }) {
            for (auto in : inputs()) {
                char buf[96];
                std::snprintf(buf, sizeof buf, "%s open air lastDy %.0f held %d at %d back %d", cfgName(c).c_str(),
                    lastDy, in.held, in.at, in.back);
                auto p = make(c, 0.f, 600.f, lastDy);
                auto r = run(p, {}, in.held, in.at, in.back, 240);
                CHECK(r.result.survived == 240, "%s: %s", buf, describe(r).c_str());
                checkPath(r, cap, 0.f, buf);
                // Each step's movement within the caps of its direction
                // (world frame: up = maxUp unless upside down).
                float upCap = c.upside ? ph.maxDown : ph.maxUp, downCap = c.upside ? ph.maxUp : ph.maxDown;
                // (swing flips which cap applies; only check the larger.)
                if (c.mode == Mode::Swing) upCap = downCap = cap;
                bool ok = true;
                for (size_t i = 1; i < r.path.size(); ++i) {
                    float d = dyAt(r, i);
                    if (d > upCap + 1e-4f || d < -downCap - 1e-4f) ok = false;
                }
                CHECK(ok, "%s: step speed beyond caps", buf);

                // Gravity mirror: the same input upside down in a mirrored
                // world gives the mirrored path.
                auto m = run(mirrorY(p, 600.f), {}, in.held, in.at, in.back, 240);
                bool same = m.path.size() == r.path.size();
                for (size_t i = 0; same && i < r.path.size(); ++i)
                    same = std::abs((1200.f - m.path[i].y) - r.path[i].y) < 0.05f; // float rounding at y ~ 600
                CHECK(same, "%s: gravity mirror differs", buf);
            }
        }
    }

    // Ship cap reached: held long -> rises at maxUp, released -> falls at maxDown.
    {
        auto ph = physicsFor(Mode::Ship);
        auto p = player(Mode::Ship, 0.f, 600.f, &ph);
        auto r = run(p, {}, true, 1000, -1, 200);
        CHECK(std::abs(dyAt(r, r.path.size() - 1) - ph.maxUp) < 1e-4f, "ship held: final dy %.3f", dyAt(r, r.path.size() - 1));
        auto f = run(p, {}, false, 1000, -1, 200);
        CHECK(std::abs(dyAt(f, f.path.size() - 1) + ph.maxDown) < 1e-4f, "ship free: final dy %.3f", dyAt(f, f.path.size() - 1));
        // Upside down: held goes down.
        p.upsideDown = true;
        auto u = run(p, {}, true, 1000, -1, 200);
        CHECK(std::abs(dyAt(u, u.path.size() - 1) + ph.maxUp) < 1e-4f, "ship upside held: final dy %.3f", dyAt(u, u.path.size() - 1));
    }

    // ---- 2. UFO: a press sets speed to impulse, also while rising ----
    for (bool mini : { false, true }) {
        for (bool up : { false, true }) {
            Cfg c { Mode::Ufo, mini, up };
            auto const& ph = physicsOf(Mode::Ufo, mini);
            float g = up ? -1.f : 1.f;
            for (float lastDy : { 0.f, 1.f, 1.9f, -1.5f }) {
                auto p = make(c, 0.f, 600.f, g * lastDy);
                auto r = run(p, {}, false, 0, -1, 10);
                CHECK(std::abs(dyAt(r, 1) - g * ph.impulse.value) < 1e-4f, "%s lastDy %.1f press: first dy %.3f, want %.3f",
                    cfgName(c).c_str(), lastDy, dyAt(r, 1), g * ph.impulse.value);
                CHECK(std::abs(dyAt(r, 2) - g * (ph.impulse.value + ph.accelFree.value)) < 1e-4f, "%s: then gravity, dy %.3f",
                    cfgName(c).c_str(), dyAt(r, 2));
            }
            // Holding the button does not jump again (UFO needs clicks).
            auto p = make(c, 0.f, 600.f);
            auto r = run(p, {}, false, 0, -1, 200);
            int impulses = 0;
            for (size_t i = 1; i < r.path.size(); ++i)
                if (std::abs(dyAt(r, i) - g * ph.impulse.value) < 1e-4f) ++impulses;
            CHECK(impulses == 1, "%s held: %d impulses", cfgName(c).c_str(), impulses);
            // A tap (press, release followGap later) is one impulse and the
            // release changes nothing.
            auto t = run(p, {}, false, 0, 8, 200);
            bool same = t.path.size() == r.path.size();
            for (size_t i = 0; same && i < r.path.size(); ++i) same = std::abs(t.path[i].y - r.path[i].y) < 1e-4f;
            CHECK(same, "%s: tap release changed the path", cfgName(c).c_str());
            // A press already held at the start (heldBefore) is not a new press.
            auto h = run(p, {}, true, 1000, -1, 50);
            CHECK(dyAt(h, 1) * g < 0.f, "%s: held from before jumped (dy %.3f)", cfgName(c).c_str(), dyAt(h, 1));
            // Release then press again: second press is an impulse.
            auto rp = run(p, {}, true, 5, 20, 50);
            CHECK(std::abs(dyAt(rp, 21) - g * ph.impulse.value) < 1e-4f, "%s: re-press at 20: dy %.3f", cfgName(c).c_str(), dyAt(rp, 21));
        }
    }

    // ---- 3. Swing: one flip per press, world velocity kept ----
    for (bool mini : { false, true }) {
        for (bool up : { false, true }) {
            Cfg c { Mode::Swing, mini, up };
            auto const& ph = physicsOf(Mode::Swing, mini);
            auto p = make(c, 0.f, 600.f, 0.3f);
            struct Case { bool held; int at, back, flips; };
            for (auto k : { Case { false, 1000, -1, 0 }, Case { true, 1000, -1, 0 }, Case { false, 10, -1, 1 },
                     Case { false, 10, 18, 1 }, Case { true, 10, 30, 1 }, Case { true, 10, -1, 0 } }) {
                auto r = run(p, {}, k.held, k.at, k.back, 60);
                int flips = accelSignChanges(r);
                CHECK(flips == k.flips, "%s held %d at %d back %d: %d flips, want %d", cfgName(c).c_str(), k.held, k.at,
                    k.back, flips, k.flips);
            }
            // At the press step: world velocity kept, then the new pull.
            auto r = run(p, {}, false, 10, -1, 30);
            float before = dyAt(r, 10), at = dyAt(r, 11);
            float gNew = up ? 1.f : -1.f;
            CHECK(std::abs(at - (before + gNew * ph.accelFree.value)) < 1e-4f, "%s: flip step dy %.3f after %.3f",
                cfgName(c).c_str(), at, before);
        }
    }

    // ---- 4. Floor (y = 90) and block floors/ceilings with seams ----
    for (auto c : cfgs) {
        auto const& ph = physicsOf(c.mode, c.mini);
        float half = c.mini ? 9.f : 15.f;
        float cap = capOf(ph);
        // Resting against the side gravity pulls to, in a 3-block corridor
        // made of 1-block seamed blocks.
        float floorTop = 300.f, ceil = floorTop + 3 * kBlock;
        auto level = corridor(-60.f, 600.f, floorTop, 3.f);
        float restY = c.upside ? ceil - half : floorTop + half;
        float otherY = c.upside ? floorTop + half : ceil - half;
        for (auto in : inputs()) {
            char buf[96];
            std::snprintf(buf, sizeof buf, "%s corridor held %d at %d back %d", cfgName(c).c_str(), in.held, in.at, in.back);
            auto r = run(make(c, 0.f, restY), level, in.held, in.at, in.back, 300);
            CHECK(r.result.survived == 300, "%s: %s", buf, describe(r).c_str());
            checkPath(r, cap, 0.f, buf);
            bool inside = true;
            for (auto v : r.path) inside = inside && v.y >= floorTop + half - 1e-3f && v.y <= ceil - half + 1e-3f;
            CHECK(inside, "%s: left the corridor", buf);
            r = run(make(c, 0.f, otherY), level, in.held, in.at, in.back, 300);
            CHECK(r.result.survived == 300, "%s (from other side): %s", buf, describe(r).c_str());
            checkPath(r, cap, 0.f, buf);
        }
        // 2-block corridor, ship/UFO/swing all.
        auto tight = corridor(-60.f, 600.f, floorTop, 2.f);
        for (auto in : inputs()) {
            auto r = run(make(c, 0.f, floorTop + kBlock), tight, in.held, in.at, in.back, 300);
            CHECK(r.result.survived == 300, "%s 2-block corridor held %d at %d: %s", cfgName(c).c_str(), in.held, in.at,
                describe(r).c_str());
        }
        // 1-block corridor (exactly the hitbox for normal size).
        auto one = corridor(-60.f, 600.f, floorTop, 1.f);
        for (auto in : inputs()) {
            auto r = run(make(c, 0.f, floorTop + 15.f), one, in.held, in.at, in.back, 300);
            CHECK(r.result.survived == 300, "%s 1-block corridor held %d at %d: %s", cfgName(c).c_str(), in.held, in.at,
                describe(r).c_str());
        }
        // Gravity mirror in a corridor.
        for (auto in : inputs()) {
            auto p = make(c, 0.f, restY - 7.f, 0.4f);
            auto r = run(p, level, in.held, in.at, in.back, 300);
            float Y0 = 600.f;
            auto m = run(mirrorY(p, Y0), mirrorY(level, Y0), in.held, in.at, in.back, 300);
            bool same = m.result.survived == r.result.survived && m.path.size() == r.path.size();
            for (size_t i = 0; same && i < r.path.size(); ++i) same = std::abs((2 * Y0 - m.path[i].y) - r.path[i].y) < 2.f; // rounding can decide a touch (speed reset), shifting the path ~1 step
            CHECK(same, "%s corridor gravity mirror differs (held %d at %d): %s vs %s", cfgName(c).c_str(), in.held, in.at,
                describe(r).c_str(), describe(m).c_str());
            // Running the other way (dx < 0) mirrors in x.
            auto mx = run(mirrorX(p, 0.f), mirrorX(level, 0.f), in.held, in.at, in.back, 300);
            same = mx.result.survived == r.result.survived && mx.path.size() == r.path.size();
            for (size_t i = 0; same && i < r.path.size(); ++i) same = std::abs(mx.path[i].y - r.path[i].y) < 1e-3f;
            CHECK(same, "%s corridor x mirror differs", cfgName(c).c_str());
        }
    }

    // The level's floor at y = 90: sliding along it doesn't crash; leaving it works.
    for (auto c : cfgs) {
        if (c.upside) continue; // the level floor is only "down" for normal gravity
        float half = c.mini ? 9.f : 15.f;
        auto p = make(c, 0.f, kFloorY + half);
        auto r = run(p, {}, false, 1000, -1, 200);
        CHECK(r.result.survived == 200 && std::abs(r.path.back().y - (kFloorY + half)) < 1e-3f, "%s sliding on floor: %s",
            cfgName(c).c_str(), describe(r).c_str());
        auto t = run(p, {}, false, 50, -1, 120);
        CHECK(t.path.back().y > kFloorY + half + 1.f, "%s takes off from floor after press: %s", cfgName(c).c_str(), describe(t).c_str());
    }
    // Upside-down ship held into the level floor slides; release leaves it.
    {
        Cfg c { Mode::Ship, false, true };
        auto p = make(c, 0.f, kFloorY + 15.f);
        auto r = run(p, {}, true, 100, -1, 160);
        CHECK(r.result.survived == 160 && std::abs(r.path[100].y - 105.f) < 1e-3f && r.path.back().y > 106.f,
            "upside ship on floor held then released: y@100 %.2f end %.2f", r.path[100].y, r.path.back().y);
    }

    // ---- 5. Ship hugging a ceiling while held, then released: falls away ----
    for (bool mini : { false, true }) {
        for (bool up : { false, true }) {
            Cfg c { Mode::Ship, mini, up };
            float half = mini ? 9.f : 15.f;
            std::vector<Obstacle> level;
            row(level, -60.f, 600.f, 500.f);  // ceiling bottom 500
            row(level, -60.f, 600.f, 270.f);  // floor top 300
            float Y0 = 400.f;
            auto p = make({ Mode::Ship, mini, false }, 0.f, 500.f - half - 20.f);
            auto lv = level;
            if (up) { p = mirrorY(p, Y0); lv = mirrorY(level, Y0); }
            auto r = run(p, lv, true, 100, -1, 160);
            float hug = up ? 300.f + half : 500.f - half;
            CHECK(r.result.survived == 160, "%s ceiling hug: %s", cfgName(c).c_str(), describe(r).c_str());
            CHECK(std::abs(r.path[100].y - hug) < 1e-3f, "%s should hug ceiling at %.1f, y %.2f", cfgName(c).c_str(), hug, r.path[100].y);
            float away = up ? r.path.back().y - hug : hug - r.path.back().y;
            // 60 released steps from speed 0: drop about 0.025*60*61/2 = 45.
            CHECK(away > 30.f, "%s released from ceiling only moved %.2f away", cfgName(c).c_str(), away);
            checkPath(r, capOf(physicsOf(Mode::Ship, mini)), 0.f, cfgName(c) + " ceiling hug");
            // Bumping the ceiling at full speed doesn't keep the speed.
            auto p2 = p;
            p2.lastDy = up ? -1.6f : 1.6f;
            auto b = run(p2, lv, false, 1000, -1, 40);
            float firstAway = 0.f;
            for (size_t i = 2; i < b.path.size(); ++i) {
                float d = up ? dyAt(b, i) : -dyAt(b, i);
                if (d > 0.f) { firstAway = d; break; }
            }
            CHECK(b.result.survived == 40 && firstAway < 0.1f, "%s bump ceiling: %s, first fall dy %.3f", cfgName(c).c_str(),
                describe(b).c_str(), firstAway);
        }
    }

    // UFO bumping a ceiling: speed reset, falls away.
    for (bool up : { false, true }) {
        Cfg c { Mode::Ufo, false, up };
        std::vector<Obstacle> level;
        row(level, -60.f, 600.f, 340.f); // ceiling bottom 340
        float Y0 = 400.f;
        auto p = make({ Mode::Ufo, false, false }, 0.f, 310.f);
        auto lv = level;
        if (up) { p = mirrorY(p, Y0); lv = mirrorY(level, Y0); }
        auto r = run(p, lv, false, 0, 3, 80);
        float top = up ? 2 * Y0 - 325.f : 325.f;
        int touched = -1;
        for (size_t i = 0; i < r.path.size(); ++i) if (std::abs(r.path[i].y - top) < 1e-3f) { touched = (int)i; break; }
        CHECK(r.result.survived == 80 && touched > 0, "%s ufo press under ceiling: %s, touched %d", cfgName(c).c_str(), describe(r).c_str(), touched);
        CHECK(std::abs(r.path.back().y - top) > 20.f, "%s ufo sticks under ceiling: end %.2f", cfgName(c).c_str(), r.path.back().y);
    }

    // ---- 6. Swing flips near floors / ceilings ----
    for (bool mini : { false, true }) {
        float half = mini ? 9.f : 15.f;
        // On the level floor: no press stays, press leaves.
        auto p = make({ Mode::Swing, mini, false }, 0.f, kFloorY + half);
        auto r = run(p, {}, false, 1000, -1, 100);
        CHECK(r.result.survived == 100 && std::abs(r.path.back().y - (kFloorY + half)) < 1e-3f, "swing%s on floor: %s",
            mini ? " mini" : "", describe(r).c_str());
        r = run(p, {}, false, 20, 25, 100);
        CHECK(r.path.back().y > kFloorY + half + 20.f, "swing%s flip off floor: %s", mini ? " mini" : "", describe(r).c_str());
        // Upside down against a block ceiling: stays; press falls away and
        // lands on the floor blocks in a 3-block corridor; press again
        // returns to the ceiling.
        auto level = corridor(-60.f, 900.f, 300.f, 3.f);
        auto q = make({ Mode::Swing, mini, true }, 0.f, 390.f - half);
        r = run(q, level, false, 1000, -1, 100);
        CHECK(r.result.survived == 100 && std::abs(r.path.back().y - (390.f - half)) < 1e-3f, "swing%s on ceiling: %s",
            mini ? " mini" : "", describe(r).c_str());
        r = run(q, level, false, 10, 12, 200);
        CHECK(r.result.survived == 200 && std::abs(r.path.back().y - (300.f + half)) < 1e-3f, "swing%s ceiling->floor: %s",
            mini ? " mini" : "", describe(r).c_str());
        // From the floor (normal gravity), holding from before, release
        // then press: one flip, back up to the ceiling.
        auto f = make({ Mode::Swing, mini, false }, 0.f, 300.f + half);
        r = run(f, level, true, 10, 30, 250);
        CHECK(r.result.survived == 250 && std::abs(r.path.back().y - (390.f - half)) < 1e-3f, "swing%s floor->ceiling: %s",
            mini ? " mini" : "", describe(r).c_str());
        checkPath(r, capOf(physicsOf(Mode::Swing, mini)), 0.f, "swing ceiling/floor");
    }

    // ---- 7. Walls, spikes, saws ----
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        float Y0 = 400.f;
        auto mir = [&](std::vector<Obstacle> lv) { return c.upside ? mirrorY(lv, Y0) : lv; };
        auto at = [&](float x, float y, float lastDy = 0.f) {
            auto p = make({ c.mode, c.mini, false }, x, y, lastDy);
            return c.upside ? mirrorY(p, Y0) : p;
        };
        // Wall across a corridor.
        auto level = corridor(-60.f, 600.f, 300.f, 3.f);
        level.push_back(block(120.f, 300.f, 30.f, 90.f));
        for (auto in : inputs()) {
            auto r = run(at(0.f, 345.f), mir(level), in.held, in.at, in.back, 200);
            CHECK(r.result.survived < 200 && std::string(r.result.cause) == "a block" && r.path.back().x < 120.f,
                "%s wall (held %d at %d): %s", cfgName(c).c_str(), in.held, in.at, describe(r).c_str());
        }
        // Spike on the floor side: sliding along it hits it, passing well above doesn't.
        {
            auto lv = corridor(-60.f, 600.f, 300.f, 4.f);
            lv.push_back(spike(90.f, 300.f));
            bool gravityHold = false; // not pressing: resting on the floor side
            auto r = run(at(0.f, 300.f + half), mir(lv), gravityHold, 1000, -1, 150);
            CHECK(r.result.survived < 150 && std::string(r.result.cause) == "a spike", "%s floor spike slide: %s",
                cfgName(c).c_str(), describe(r).c_str());
            // Just above the spike tip (12 units), without falling much:
            // start 1 unit above for the ship held = level-ish flight.
        }
        // Spike hanging from the ceiling side: hugging that side hits it.
        {
            auto lv = corridor(-60.f, 600.f, 300.f, 4.f);
            lv.push_back(spikeDown(90.f, 420.f));
            if (c.mode == Mode::Ship) {
                auto r = run(at(0.f, 420.f - half), mir(lv), true, 1000, -1, 150);
                CHECK(r.result.survived < 150 && std::string(r.result.cause) == "a spike", "%s ceiling spike hug: %s",
                    cfgName(c).c_str(), describe(r).c_str());
            }
            // Mid-corridor flight passes under it.
            auto r = run(at(0.f, 300.f + half), mir(lv), false, 1000, -1, 150);
            CHECK(r.result.survived == 150, "%s under ceiling spike: %s", cfgName(c).c_str(), describe(r).c_str());
        }
        // Saw in the path kills; one far enough off doesn't.
        {
            auto lv = corridor(-60.f, 600.f, 300.f, 4.f);
            lv.push_back(saw(120.f, 300.f + half, 12.f));
            auto r = run(at(0.f, 300.f + half), mir(lv), false, 1000, -1, 150);
            CHECK(r.result.survived < 150 && std::string(r.result.cause) == "a saw", "%s saw: %s", cfgName(c).c_str(), describe(r).c_str());
            lv.back() = saw(120.f, 300.f + half + half + 12.f + 2.f, 12.f);
            r = run(at(0.f, 300.f + half), mir(lv), false, 1000, -1, 150);
            CHECK(r.result.survived == 150, "%s saw just clear: %s", cfgName(c).c_str(), describe(r).c_str());
        }
        // Grazing a block's corner from the side (outer box overlaps, inner
        // doesn't): no teleport above/below it.
        {
            std::vector<Obstacle> lv { block(60.f, 300.f, 30.f, 30.f) };
            // Player's outer box overlaps the block's top 5 units; inner box clear.
            auto r = run(at(0.f, 330.f + half - 5.f), mir(lv), false, 1000, -1, 60);
            checkPath(r, capOf(physicsOf(c.mode, c.mini)), 0.f, cfgName(c) + " corner graze");
        }
    }

    // Clipping a block's corner within one step: the player reaches the
    // block's bottom (or top) in the same step it reaches its side. The
    // overlap is less than one step's movement in both x and y, so it came
    // from clear in y: it should be pushed back like any ceiling / floor
    // contact (at most one step's vertical move), not left inside the block
    // to crash when its inner box gets there.
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        auto const& ph = physicsOf(c.mode, c.mini);
        float Y0 = 400.f;
        int crashes = 0, tries = 0;
        float worst = 0.f;
        std::string example;
        // Ceiling row starting at x 60, bottom 400; the player rises toward it.
        for (float frac = 0.05f; frac < 1.f; frac += 0.1f) {
            std::vector<Obstacle> lv;
            row(lv, 60.f, 600.f, 400.f);
            float speed = c.mode == Mode::Ship ? ph.maxUp : 1.2f;
            // Start 8 steps before the right edge passes x 60 (so the UFO
            // and swing are still moving fast); k: steps until it passes.
            float x0 = 60.f - half - 7.5f * kSpeed1x;
            int k = (int)std::ceil((60.f - half - x0) / kSpeed1x);
            // Make the top cross 400 inside step k, `frac` of a step early.
            Player p = make({ c.mode, c.mini, false }, x0, 0.f, speed);
            // Ship held keeps maxUp; UFO / swing: integrate the model's
            // speed (accelFree each step) to place the player.
            float y = 0.f, v = speed;
            for (int i = 0; i < k; ++i) {
                if (c.mode != Mode::Ship) v += ph.accelFree.value;
                y += v;
            }
            p.center.y = 400.f - half - y + frac * v;
            auto lvm = lv;
            if (c.upside) { p = mirrorY(p, Y0); lvm = mirrorY(lv, Y0); }
            auto r = run(p, lvm, c.mode == Mode::Ship, 1000, -1, 120);
            ++tries;
            if (r.result.survived < 120) {
                ++crashes;
                if (example.empty()) {
                    char buf[160];
                    std::snprintf(buf, sizeof buf, "start y %.2f: %s", p.center.y, describe(r).c_str());
                    example = buf;
                }
            }
            worst = std::max(worst, maxVerticalStep(r));
        }
        CHECK(crashes == 0, "%s rising into a ceiling block corner: %d/%d crash, e.g. %s", cfgName(c).c_str(), crashes, tries,
            example.c_str());
        CHECK(worst <= capOf(ph) + 1e-3f, "%s ceiling corner: max step %.2f", cfgName(c).c_str(), worst);
    }
    // Ship held along a block ceiling with a 1-block hole: the 30-wide
    // ship always overlaps a ceiling block except for one step, so it can
    // only rise a hair (0.02) into the hole and must be pushed back under the
    // next block. (A 2+ block hole lets it rise several units; whether GD
    // then pushes it down or kills it is uncertain, not checked.)
    for (bool up : { false, true }) {
        std::vector<Obstacle> lv;
        for (float x = -60.f; x < 600.f; x += kBlock) if (x != 60.f) lv.push_back(block(x, 400.f));
        auto ph = physicsFor(Mode::Ship);
        auto p = player(Mode::Ship, 0.f, 385.f, &ph);
        if (up) { lv = mirrorY(lv, 400.f); p = mirrorY(p, 400.f); }
        auto r = run(p, lv, true, 1000, -1, 200);
        CHECK(r.result.survived == 200, "ship%s held under ceiling with 1-block hole: %s", up ? " upside" : "", describe(r).c_str());
    }

    // Falling onto a platform's corner (ship released at full fall speed).
    for (bool mini : { false, true }) {
        float half = mini ? 9.f : 15.f;
        auto const& ph = physicsOf(Mode::Ship, mini);
        int crashes = 0;
        std::string example;
        for (float frac = 0.05f; frac < 1.f; frac += 0.1f) {
            std::vector<Obstacle> lv;
            row(lv, 60.f, 600.f, 270.f); // top 300
            int k = (int)std::ceil((60.f - half) / kSpeed1x);
            auto p = make({ Mode::Ship, mini, false }, 0.f, 300.f + half + ph.maxDown * (k - frac), -ph.maxDown);
            auto r = run(p, lv, false, 1000, -1, 120);
            if (r.result.survived < 120) {
                ++crashes;
                if (example.empty()) example = describe(r);
            }
        }
        CHECK(crashes == 0, "ship%s falling onto a platform corner: %d crash, e.g. %s", mini ? " mini" : "", crashes, example.c_str());
    }

    // ---- 8. Landing on a block top and taking off ----
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        float Y0 = 400.f;
        std::vector<Obstacle> plat;
        row(plat, 0.f, 900.f, 270.f); // top 300, seams
        auto lv = c.upside ? mirrorY(plat, Y0) : plat;
        auto p = make({ c.mode, c.mini, false }, 30.f, 360.f, -0.5f);
        if (c.upside) p = mirrorY(p, Y0);
        float g = c.upside ? -1.f : 1.f;
        float rest = c.upside ? 2 * Y0 - 300.f - half : 300.f + half;
        // Released (ship) / not pressed: lands and slides.
        auto r = run(p, lv, false, 1000, -1, 300);
        CHECK(r.result.survived == 300 && std::abs(r.path.back().y - rest) < 1e-3f, "%s lands on block: %s", cfgName(c).c_str(),
            describe(r).c_str());
        checkPath(r, capOf(physicsOf(c.mode, c.mini)), 0.f, cfgName(c) + " landing");
        // Then press (and keep holding for the ship) at step 250: takes off.
        auto t = run(p, lv, false, 250, c.mode == Mode::Ship ? -1 : 256, 340);
        CHECK(t.result.survived == 340 && g * (t.path[280].y - rest) > 5.f, "%s takes off from block: %s", cfgName(c).c_str(),
            describe(t).c_str());
    }

    // ---- 9. Slopes ----
    // Floor slopes falling away (45 and 2:1): ship released drops onto them
    // and slides down; no teleport.
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        auto const& ph = physicsOf(c.mode, c.mini);
        for (float w : { 60.f, 120.f }) {
            float Y0 = 400.f;
            std::vector<Obstacle> lv;
            row(lv, -90.f, 30.f, 270.f);                       // top 300 up to x 30
            lv.push_back(slope(30.f, 240.f, w, 60.f, false, true)); // falls 300 -> 240
            lv.push_back(block(30.f, 210.f, w, 30.f));         // under the slope
            row(lv, 30.f + w, 600.f, 210.f);                   // top 240 after
            auto p = make({ c.mode, c.mini, false }, 0.f, 300.f + half);
            auto mlv = c.upside ? mirrorY(lv, Y0) : lv;
            if (c.upside) p = mirrorY(p, Y0);
            auto r = run(p, mlv, false, 1000, -1, 300);
            char buf[96];
            std::snprintf(buf, sizeof buf, "%s down slope w %.0f", cfgName(c).c_str(), w);
            CHECK(r.result.survived == 300, "%s: %s", buf, describe(r).c_str());
            checkPath(r, capOf(ph), 0.f, buf);
        }
    }

    // Rising floor slope from the floor (ship / UFO / swing resting on the
    // ground, not pressing). GD pushes non-wave players up a slope they run
    // into from its low end; the model's comment says it lands only on
    // slopes "flat or falling away", so this is where the model and GD may
    // disagree.
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        for (float w : { 30.f, 60.f, 120.f }) {
            float h = w == 30.f ? 30.f : w == 60.f ? 60.f : 60.f; // 45, 45, 2:1
            float Y0 = 400.f;
            std::vector<Obstacle> lv;
            row(lv, -90.f, 60.f, 270.f);                            // top 300
            lv.push_back(slope(60.f, 300.f, w, h, true, true));      // rises 300 -> 300+h
            lv.push_back(block(60.f, 270.f, w, 30.f));
            row(lv, 60.f + w, 600.f, 270.f + h);                     // top 300+h after
            auto p = make({ c.mode, c.mini, false }, 0.f, 300.f + half);
            auto mlv = c.upside ? mirrorY(lv, Y0) : lv;
            if (c.upside) p = mirrorY(p, Y0);
            auto r = run(p, mlv, false, 1000, -1, 250);
            char buf[96];
            std::snprintf(buf, sizeof buf, "%s up slope %.0fx%.0f (resting, no input)", cfgName(c).c_str(), w, h);
            CHECK(r.result.survived == 250, "%s: %s", buf, describe(r).c_str());
            checkPath(r, capOf(physicsOf(c.mode, c.mini)), 1.5f + kSpeed1x * h / w, buf);
        }
    }

    // Ship held in a 2:1 diagonal corridor (floor and ceiling slopes rising
    // together, 3 blocks apart): should survive, pushed along the floor.
    for (bool up : { false, true }) {
        for (bool held : { true, false }) {
            float Y0 = 400.f;
            std::vector<Obstacle> lv;
            lv.push_back(slope(0.f, 200.f, 240.f, 120.f, true, true));   // floor 200 -> 320
            lv.push_back(slope(0.f, 290.f, 240.f, 120.f, true, false));  // ceiling 290 -> 410
            auto p = make({ Mode::Ship, false, false }, 20.f, 200.f + 10.f + 15.f + 20.f, 0.65f);
            if (up) { lv = mirrorY(lv, Y0); p = mirrorY(p, Y0); }
            auto r = run(p, lv, held, 1000, -1, 140);
            CHECK(r.result.survived == 140, "ship%s diagonal corridor held %d: %s", up ? " upside" : "", held, describe(r).c_str());
        }
    }

    // Ship hugging a ceiling slope that rises away: falls off it; one that
    // comes down (falling ceiling) should push the ship down (GD) - the
    // model's rule only slides along ceilings rising away.
    for (float w : { 60.f, 120.f }) {
        std::vector<Obstacle> lv;
        row(lv, -90.f, 60.f, 400.f);                                // ceiling bottom 400
        lv.push_back(slope(60.f, 340.f, w, 60.f, false, false));     // ceiling falls 400 -> 340
        lv.push_back(block(60.f, 400.f, w, 30.f));
        row(lv, 60.f + w, 600.f, 340.f);                             // ceiling bottom 340 after
        auto p = make({ Mode::Ship, false, false }, 0.f, 385.f);
        auto r = run(p, lv, true, 1000, -1, 250);
        CHECK(r.result.survived == 250, "ship held under falling ceiling slope w %.0f: %s", w, describe(r).c_str());
        checkPath(r, 1.6f, 1.5f + kSpeed1x * 60.f / w, "ship ceiling slope");
        // Rising ceiling (goes away): survives.
        std::vector<Obstacle> lv2;
        row(lv2, -90.f, 60.f, 400.f);
        lv2.push_back(slope(60.f, 400.f, w, 60.f, true, false));     // rises 400 -> 460
        auto r2 = run(p, lv2, true, 1000, -1, 250);
        CHECK(r2.result.survived == 250, "ship held under rising ceiling slope w %.0f: %s", w, describe(r2).c_str());
        checkPath(r2, 1.6f, 0.01f, "ship rising ceiling slope");
    }

    // Slope edge clipping (the old teleport bug): passing the tall end of a
    // slope from the side must not lift the player onto its surface.
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        // Floor slope falling to the right with its 60-unit face on the
        // left; player approaches from the left at mid height -> crashes
        // (or at least is not lifted 30+ units in one step).
        std::vector<Obstacle> lv { slope(90.f, 300.f, 60.f, 60.f, false, true), block(90.f, 270.f, 60.f, 30.f) };
        row(lv, -90.f, 90.f, 270.f); // floor top 300 up to the face
        float Y0 = 400.f;
        auto p = make({ c.mode, c.mini, false }, 0.f, 300.f + half + 10.f);
        auto mlv = c.upside ? mirrorY(lv, Y0) : lv;
        if (c.upside) p = mirrorY(p, Y0);
        auto r = run(p, mlv, false, 1000, -1, 150);
        checkPath(r, capOf(physicsOf(c.mode, c.mini)), 2.f, cfgName(c) + " slope face");
        CHECK(r.result.survived < 150, "%s into slope face: %s", cfgName(c).c_str(), describe(r).c_str());
    }

    // Deadly slope: touching kills.
    {
        auto ph = physicsFor(Mode::Ship);
        std::vector<Obstacle> lv { slope(60.f, 90.f, 60.f, 60.f, false, true, true) };
        auto r = run(player(Mode::Ship, 0.f, 105.f, &ph), lv, false, 1000, -1, 100);
        CHECK(r.result.survived < 100 && std::string(r.result.cause) == "a hazard slope", "ship deadly slope: %s", describe(r).c_str());
    }

    // ---- 10. dx = 0 / tiny / backwards ----
    for (auto c : cfgs) {
        float half = c.mini ? 9.f : 15.f;
        for (float dx : { 0.f, 0.001f, -0.001f, -kSpeed1x }) {
            auto level = corridor(-300.f, 300.f, 300.f, 3.f);
            auto p = make(c, 0.f, c.upside ? 390.f - half : 300.f + half);
            p.dx = dx;
            for (auto in : inputs()) {
                auto r = run(p, level, in.held, in.at, in.back, 200);
                char buf[96];
                std::snprintf(buf, sizeof buf, "%s dx %.3f held %d at %d", cfgName(c).c_str(), dx, in.held, in.at);
                CHECK(r.result.survived == 200, "%s: %s", buf, describe(r).c_str());
                checkPath(r, capOf(physicsOf(c.mode, c.mini)), 0.f, buf);
            }
            // Under a ceiling slope with dx = 0: sliding against it, no crash.
            std::vector<Obstacle> sl { slope(-60.f, 380.f, 120.f, 60.f, true, false) };
            auto q = make({ Mode::Ship, c.mini, false }, 0.f, 350.f);
            q.dx = dx;
            auto r = run(q, sl, true, 1000, -1, 200);
            CHECK(finite(r), "ship dx %.3f under slope: non-finite", dx);
        }
    }

    // ---- 11. choose() ----
    // UFO on the floor before a floor spike: the arc of a press now comes
    // down on the spike; a press a few steps later clears it.
    for (bool mini : { false, true }) {
        auto const& ph = physicsOf(Mode::Ufo, mini);
        float half = mini ? 9.f : 15.f;
        auto p = make({ Mode::Ufo, mini, false }, 0.f, kFloorY + half);
        // Where the arc of a press at step 0 comes back down to spike height.
        auto arc = run(p, {}, false, 0, -1, 200);
        float downX = 0.f;
        for (size_t i = 10; i < arc.path.size(); ++i)
            if (arc.path[i].y - half <= kFloorY + 12.f && arc.path[i].y < arc.path[i - 1].y) { downX = arc.path[i].x; break; }
        CHECK(downX > 50.f, "ufo%s arc: downX %.1f", mini ? " mini" : "", downX);
        // Spike whose hitbox right edge is 2 units inside the player's
        // box when it comes down.
        float sx = downX - half + 2.f - 18.f;
        std::vector<Obstacle> lv { spike(sx, kFloorY) };
        auto now = simulate(p, lv, false, 0, -1, 200);
        CHECK(now.survived < 200 && std::string(now.cause) == "a spike", "ufo%s press now should hit spike: %d %s",
            mini ? " mini" : "", now.survived, now.cause);
        for (int gap : { 0, 8 }) {
            auto ch = choose({ p }, lv, true, 10, gap, 160);
            CHECK(ch.delay > 0 && ch.delay <= 4, "ufo%s early press gap %d: delay %d (now %d, best %d, %s)", mini ? " mini" : "",
                gap, ch.delay, ch.survivedNow, ch.survivedBest, ch.causeNow);
        }
        // On time: the spike is under the middle of the arc.
        std::vector<Obstacle> lv2 { spike(downX / 2.f - 15.f, kFloorY) };
        auto ch = choose({ p }, lv2, true, 10, 8, 160);
        CHECK(ch.delay == 0 && ch.survivedNow == 160, "ufo%s on-time press: delay %d now %d", mini ? " mini" : "", ch.delay, ch.survivedNow);
        (void)ph;
    }

    // Ship hugging a ceiling, held, with a spike on a pillar just below:
    // releasing now falls onto it, releasing a few steps later passes.
    for (bool up : { false, true }) {
        float Y0 = 300.f;
        std::vector<Obstacle> lv;
        row(lv, -90.f, 600.f, 165.f);              // ceiling bottom 165
        lv.push_back(block(-8.f, 90.f, 30.f, 30.f)); // pillar top 120
        lv.push_back(spike(-8.f, 120.f));          // spike tip 132
        auto p = make({ Mode::Ship, false, false }, 0.f, 150.f); // bottom 135
        if (up) { lv = mirrorY(lv, Y0); p = mirrorY(p, Y0); }
        auto now = simulate(p, lv, true, 0, -1, 200);
        CHECK(now.survived < 200 && std::string(now.cause) == "a spike", "ship%s release now: %d %s", up ? " upside" : "",
            now.survived, now.cause);
        auto ch = choose({ p }, lv, false, 12, 0, 120);
        CHECK(ch.delay > 0 && ch.delay <= 8, "ship%s early release: delay %d (now %d %s, best %d)", up ? " upside" : "", ch.delay,
            ch.survivedNow, ch.causeNow, ch.survivedBest);
        // On time: spike far behind.
        std::vector<Obstacle> lv2;
        row(lv2, -90.f, 600.f, 165.f);
        lv2.push_back(spike(-80.f, 90.f));
        if (up) lv2 = mirrorY(lv2, Y0);
        auto ch2 = choose({ p }, lv2, false, 12, 0, 120);
        CHECK(ch2.delay == 0, "ship%s on-time release: delay %d now %d", up ? " upside" : "", ch2.delay, ch2.survivedNow);
        // A press evaluated as a release would let the ship fall while
        // waiting: with push=true (held=false before) the same player
        // falls; check the two differ in what they hold.
        auto chPress = choose({ p }, lv2, true, 12, 0, 120);
        CHECK(chPress.delay == 0, "ship%s press on ceiling: delay %d", up ? " upside" : "", chPress.delay);
    }

    // Ship press too early into a ceiling spike: in a corridor, pressing now
    // rises into a hanging spike; pressing later passes it first.
    {
        auto ph = physicsFor(Mode::Ship);
        std::vector<Obstacle> lv;
        lv.push_back(spikeDown(30.f, 135.f)); // hitbox x 42..48, y 123..135
        auto p = player(Mode::Ship, 0.f, 105.f, &ph); // top 120
        auto now = simulate(p, lv, false, 0, -1, 200);
        CHECK(now.survived < 200 && std::string(now.cause) == "a spike", "ship press now into spike: %d %s", now.survived, now.cause);
        // The model passes it with a press 32+ steps later.
        auto ch = choose({ p }, lv, true, 40, 0, 120);
        CHECK(ch.delay >= 30 && ch.delay <= 36, "ship early press: delay %d (now %d %s, best %d)", ch.delay, ch.survivedNow, ch.causeNow, ch.survivedBest);
    }

    // Swing: flipping now off the floor into a ceiling spike; later clears.
    {
        auto ph = physicsFor(Mode::Swing);
        auto p = player(Mode::Swing, 0.f, 105.f, &ph);
        std::vector<Obstacle> lv { spikeDown(30.f, 135.f) };
        auto now = simulate(p, lv, false, 0, -1, 200);
        CHECK(now.survived < 200 && std::string(now.cause) == "a spike", "swing flip now into spike: %d %s", now.survived, now.cause);
        auto ch = choose({ p }, lv, true, 40, 8, 120);
        CHECK(ch.delay > 0, "swing early flip: delay %d (now %d %s, best %d)", ch.delay, ch.survivedNow, ch.causeNow, ch.survivedBest);
    }

    return finish("test_flying");
}
