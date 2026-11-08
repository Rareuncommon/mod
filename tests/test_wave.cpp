// Wave path prediction on the structures wave parts are built from: open
// air, the floor, block floors / ceilings (with seams), walls, gaps, spikes,
// saws, 45 and 2:1 slopes, zigzag corridors, diamonds, staircases and the
// transitions between them. Each scenario checks invariants (no teleports,
// never inside a solid without a crash, mirror symmetry) and the outcome the
// model's own rules give (see the comments in src/sim.hpp).
//
// Where real Geometry Dash behaviour is uncertain (when exactly a wave dies
// vs slides on a slope or block corner), the expected outcome is the one the
// model's comments state, and the test says so.
#include "harness.hpp"

#include <cstring>
#include <functional>

using namespace test;

namespace {

using Level = std::vector<Obstacle>;

constexpr int kForever = 1 << 30;

// A wave at (x, y) moving up or down (in world terms).
Player wave(float x, float y, bool up, bool mini = false, bool upsideDown = false) {
    auto p = player(Mode::Wave, x, y, nullptr, mini);
    p.upsideDown = upsideDown;
    p.lastDy = (up ? 1.f : -1.f) * p.dx * p.waveSlope;
    return p;
}

// A wave sliding (no vertical movement last step): direction from the button.
Player slider(float x, float y, bool mini = false, bool upsideDown = false) {
    auto p = player(Mode::Wave, x, y, nullptr, mini);
    p.upsideDown = upsideDown;
    p.lastDy = 0.f;
    return p;
}

// A wave whose center reaches (x, y) after `steps` steps (fractional steps
// shift where within a step it crosses things: the phase).
Player waveTo(float x, float y, bool up, float steps, bool mini = false) {
    auto p = wave(0.f, 0.f, up, mini);
    p.center.x = x - steps * p.dx;
    p.center.y = y - (up ? 1.f : -1.f) * steps * p.dx * p.waveSlope;
    return p;
}

bool heldFor(bool up, bool upsideDown) { return up != upsideDown; }

float dyOf(Player const& p) { return std::abs(p.dx) * p.waveSlope; }

// Keeps the current direction for `steps`.
Run hold(Player const& p, Level const& level, int steps) {
    bool up = p.lastDy > 0.f;
    return run(p, level, heldFor(up, p.upsideDown), kForever, -1, steps);
}

// Holds the button (`held`) the whole time (for sliders).
Run holdButton(Player const& p, Level const& level, bool held, int steps) {
    return run(p, level, held, kForever, -1, steps);
}

// Current direction, then the other one from step `at`.
Run turnAt(Player const& p, Level const& level, int at, int steps) {
    bool up = p.lastDy > 0.f;
    return run(p, level, heldFor(up, p.upsideDown), at, -1, steps);
}

// Runs a sequence of (goingUp, steps) segments, each continuing from where
// the last one ended. Returns the combined run (survived counts all steps).
Run segments(Player p, Level const& level, std::vector<std::pair<bool, int>> const& segs) {
    Run total;
    total.path.push_back(p.center);
    int done = 0;
    for (auto [up, steps] : segs) {
        p.lastDy = (up ? 1.f : -1.f) * dyOf(p);
        auto r = run(p, level, heldFor(up, p.upsideDown), kForever, -1, steps);
        for (size_t i = 1; i < r.path.size(); ++i) total.path.push_back(r.path[i]);
        done += r.result.survived;
        if (r.result.survived < steps) {
            total.result = { done, r.result.cause };
            return total;
        }
        p.center = r.path.back();
    }
    total.result = { done, "" };
    return total;
}

bool survived(Run const& r, int steps) { return r.result.survived == steps; }

// ---- invariants ----

// The path never jumps vertically more than one step's move, and no path
// point is inside a solid (inner box in a block, below the floor, inner box
// past a slope's line while over it). Points after which the model reported
// a crash are not in the path.
void checkInvariants(char const* name, Player const& p, Level const& level, Run const& r) {
    float dy = dyOf(p);
    float step = maxVerticalStep(r);
    CHECK(step <= dy + 0.02f, "%s: vertical jump %.2f > dy %.2f (%s)", name, step, dy, describe(r).c_str());
    float innerW = p.halfW * kInnerScale, innerH = p.halfH * kInnerScale;
    for (size_t i = 1; i < r.path.size(); ++i) {
        auto c = r.path[i];
        CHECK(c.y - p.halfH >= kFloorY - 0.01f, "%s: below floor at step %zu (%.2f, %.2f)", name, i, c.x, c.y);
        for (auto const& o : level) {
            if (o.hazard) continue;
            if (!o.slope) {
                // Inner box overlapping a block by more than a hair.
                float ox = std::min(c.x + innerW, o.rect.getMaxX()) - std::max(c.x - innerW, o.rect.getMinX());
                float oy = std::min(c.y + innerH, o.rect.getMaxY()) - std::max(c.y - innerH, o.rect.getMinY());
                CHECK(!(ox > 0.01f && oy > 0.01f), "%s: inner box inside block (%.0f,%.0f %.0fx%.0f) at step %zu (%.2f, %.2f), %s",
                    name, o.rect.getMinX(), o.rect.getMinY(), o.rect.size.width, o.rect.size.height, i, c.x, c.y,
                    describe(r).c_str());
                continue;
            }
            if (c.x < o.rect.getMinX() || c.x > o.rect.getMaxX()) continue;
            float mid = o.surfaceAt(c.x);
            bool inRect = c.y >= o.rect.getMinY() - innerH && c.y <= o.rect.getMaxY() + innerH;
            if (!inRect) continue;
            bool inside = o.floorTop ? c.y - innerH < mid - 0.01f : c.y + innerH > mid + 0.01f;
            CHECK(!inside, "%s: inner box past slope line at step %zu (%.2f, %.2f), line %.2f, %s",
                name, i, c.x, c.y, mid, describe(r).c_str());
            if (o.deadly) {
                Rect box(c.x - p.halfW, c.y - p.halfH, p.halfW * 2, p.halfH * 2);
                float l = o.surfaceAt(c.x - p.halfW), rr = o.surfaceAt(c.x + p.halfW);
                bool touching = box.intersectsRect(o.rect)
                    && (o.floorTop ? c.y - p.halfH < std::max(l, rr) : c.y + p.halfH > std::min(l, rr));
                CHECK(!touching, "%s: touching a hazard slope at step %zu without a crash", name, i);
            }
        }
    }
}

// ---- mirror symmetry ----

Obstacle mirror(Obstacle o, float axis) {
    o.rect.origin.y = 2 * axis - o.rect.origin.y - o.rect.size.height;
    o.center.y = 2 * axis - o.center.y;
    if (o.slope) {
        o.floorTop = !o.floorTop;
        o.y0 = 2 * axis - o.y0;
        o.y1 = 2 * axis - o.y1;
    }
    return o;
}

Level mirror(Level const& level, float axis) {
    Level out;
    for (auto const& o : level) out.push_back(mirror(o, axis));
    return out;
}

Player mirror(Player p, float axis) {
    p.center.y = 2 * axis - p.center.y;
    p.lastDy = -p.lastDy;
    p.upsideDown = !p.upsideDown;
    return p;
}

// Upside down in the mirrored level must give the mirrored path. The level
// must stay clear of the game floor (y = 90) in both versions.
void checkMirror(char const* name, Player const& p, Level const& level, bool held, int switchAt, int steps) {
    float axis = 600.f;
    auto a = run(p, level, held, switchAt, -1, steps);
    auto b = run(mirror(p, axis), mirror(level, axis), held, switchAt, -1, steps);
    CHECK(a.result.survived == b.result.survived && std::strcmp(a.result.cause, b.result.cause) == 0,
        "%s mirror: %s vs mirrored %s", name, describe(a).c_str(), describe(b).c_str());
    float worst = 0.f;
    size_t n = std::min(a.path.size(), b.path.size());
    for (size_t i = 0; i < n; ++i) worst = std::max(worst, std::abs(a.path[i].y - (2 * axis - b.path[i].y)));
    CHECK(worst < 0.01f, "%s mirror: paths differ by %.3f", name, worst);
}

// ---- level pieces ----

// A row of `n` blocks starting at x (seams between them).
void row(Level& l, float x, float y, int n) {
    for (int i = 0; i < n; ++i) l.push_back(block(x + i * kBlock, y));
}

// Step at which a run's path first reaches x (or -1).
int stepAtX(Run const& r, float x) {
    for (size_t i = 0; i < r.path.size(); ++i) if (r.path[i].x >= x) return static_cast<int>(i);
    return -1;
}

// A zigzag corridor from x0 with `n` 45-degree sections of width w (one
// slope triangle each, floor and ceiling), the first one rising. The floor
// line starts at y0, the ceiling line is `gap` above it. Blocks fill below
// the floor slopes and above the ceiling slopes.
Level zigzag(float x0, float y0, float w, int n, float gap, bool fill = true) {
    Level l;
    for (int i = 0; i < n; ++i) {
        float x = x0 + i * w;
        bool rising = i % 2 == 0;
        l.push_back(slope(x, y0, w, w, rising, true));
        l.push_back(slope(x, y0 + gap, w, w, rising, false));
        if (fill) {
            if (y0 > kFloorY) l.push_back(block(x, kFloorY, w, y0 - kFloorY));
            l.push_back(block(x, y0 + gap + w, w, 300.f));
        }
    }
    return l;
}

// A diamond of 4 slope triangles centered on (cx, cy) with half-size s.
Level diamond(float cx, float cy, float s, bool deadly = false) {
    return {
        slope(cx - s, cy, s, s, true, true, deadly),     // upper left (floor: solid below)
        slope(cx, cy, s, s, false, true, deadly),        // upper right
        slope(cx - s, cy - s, s, s, false, false, deadly), // lower left (ceiling: solid above)
        slope(cx, cy - s, s, s, true, false, deadly),    // lower right
    };
}

} // namespace

int main() {
    // =====================================================================
    // Open air
    // =====================================================================
    for (bool mini : { false, true }) {
        for (bool up : { false, true }) {
            auto p = wave(0.f, 400.f, up, mini);
            auto r = hold(p, {}, 100);
            CHECK(survived(r, 100), "open air: %s", describe(r).c_str());
            float want = 400.f + (up ? 1.f : -1.f) * 100 * dyOf(p);
            CHECK(std::abs(r.path.back().y - want) < 0.05f, "open air straight line: end y %.2f want %.2f mini %d",
                r.path.back().y, want, mini);
            checkInvariants("open air", p, {}, r);
            checkMirror("open air", p, {}, heldFor(up, false), 30, 100);
        }
    }

    // =====================================================================
    // Game floor (y = 90): sliding along it holding down
    // =====================================================================
    for (bool mini : { false, true }) {
        auto p = wave(0.f, 110.f, false, mini);
        auto r = hold(p, {}, 120);
        CHECK(survived(r, 120), "floor slide: %s", describe(r).c_str());
        CHECK(std::abs(r.path.back().y - (kFloorY + p.halfH)) < 0.01f, "floor slide end y %.2f", r.path.back().y);
        checkInvariants("floor slide", p, {}, r);
        // Sliding then pressing: leaves the floor diagonally right away.
        auto s = slider(0.f, kFloorY + p.halfH, mini);
        auto r2 = run(s, {}, false, 10, -1, 30);
        CHECK(survived(r2, 30), "floor slide then up: %s", describe(r2).c_str());
        CHECK(std::abs(r2.path.back().y - (kFloorY + p.halfH + 20 * dyOf(p))) < 0.01f,
            "floor slide then up: end y %.2f", r2.path.back().y);
    }

    // =====================================================================
    // Flat block floors and ceilings with seams
    // =====================================================================
    for (bool mini : { false, true }) {
        // Floor of 10 blocks (top 210), wave lands on it and slides across
        // every seam.
        Level floor;
        row(floor, 0.f, 180.f, 10);
        auto p = wave(0.f, 240.f, false, mini);
        auto r = hold(p, floor, 200);
        CHECK(survived(r, 200), "block floor slide (mini %d): %s", mini, describe(r).c_str());
        float land = 210.f + p.halfH;
        bool flat = true;
        for (auto c : r.path) if (c.x > 60.f && c.x < 290.f && std::abs(c.y - land) > 0.01f) flat = false;
        CHECK(flat, "block floor slide stays flat on top (mini %d)", mini);
        checkInvariants("block floor slide", p, floor, r);
        checkMirror("block floor slide", p, floor, false, kForever, 200);

        // Ceiling of 10 blocks (bottom 300), wave rises into it and slides.
        Level ceil;
        row(ceil, 0.f, 300.f, 10);
        auto q = wave(0.f, 260.f, true, mini);
        auto rc = hold(q, ceil, 200);
        CHECK(survived(rc, 200), "block ceiling slide (mini %d): %s", mini, describe(rc).c_str());
        bool flatC = true;
        for (auto c : rc.path) if (c.x > 60.f && c.x < 290.f && std::abs(c.y - (300.f - q.halfH)) > 0.01f) flatC = false;
        CHECK(flatC, "block ceiling slide stays flat (mini %d)", mini);
        checkInvariants("block ceiling slide", q, ceil, rc);

        // Starting exactly on a seam, already sliding.
        auto s = slider(30.f - 0.3f, 210.f + p.halfH, mini);
        auto rs = holdButton(s, floor, false, 100);
        CHECK(survived(rs, 100), "slide starting on a seam: %s", describe(rs).c_str());
        CHECK(maxVerticalStep(rs) < 0.01f, "slide starting on a seam stays flat: %.2f", maxVerticalStep(rs));

        // Floor and ceiling a 2-block tunnel: zigzag inside survives.
        Level tunnel = floor;
        row(tunnel, 0.f, 270.f, 10);
        auto t = wave(0.f, 240.f, true, mini);
        auto rt = segments(t, tunnel, { { true, 20 }, { false, 30 }, { true, 30 }, { false, 30 }, { true, 30 } });
        CHECK(survived(rt, 140), "zigzag in a block tunnel (mini %d): %s", mini, describe(rt).c_str());
        checkInvariants("zigzag in a block tunnel", t, tunnel, rt);
    }

    // Seam where the second block is a hair lower (editor misalignment):
    // keeps sliding.
    {
        Level l = { block(0.f, 180.f), block(30.f, 179.99f, 30.f, 30.f), block(60.f, 180.f) };
        auto p = slider(5.f, 215.f);
        auto r = holdButton(p, l, false, 60);
        CHECK(survived(r, 60), "seam with tiny height difference: %s", describe(r).c_str());
        checkInvariants("seam with tiny height difference", p, l, r);
    }

    // =====================================================================
    // Sliding off a block's end and the user's screenshot case: descending
    // diagonally onto the top-left corner area of a block, sliding flat
    // along the top, then continuing diagonally down after the block.
    // =====================================================================
    for (bool mini : { false, true }) {
        Level l = { block(60.f, 250.f, 90.f, 30.f) }; // top 280, x 60..150
        float half = mini ? 3.f : 5.f;
        // Aim the bottom-right corner 3 units above the block's corner.
        auto p = waveTo(60.f - half, 283.f + half, false, 30.f, mini);
        float dy = dyOf(p);
        auto r = hold(p, l, 150);
        CHECK(survived(r, 150), "screenshot case (mini %d): %s", mini, describe(r).c_str());
        checkInvariants("screenshot case", p, l, r);
        // Flat at 280 + halfH while the box is over the block.
        bool flat = true;
        for (auto c : r.path) if (c.x - half > 62.f && c.x - half < 148.f && std::abs(c.y - (280.f + half)) > 0.01f) flat = false;
        CHECK(flat, "screenshot case: slides flat on the block (mini %d)", mini);
        // After the block: diagonal down again at full slope.
        int k = stepAtX(r, 150.f + half + 5.f);
        CHECK(k > 0 && std::abs(r.path[k].y - r.path[k - 1].y + dy) < 0.01f, "screenshot case: diagonal after block");
        checkMirror("screenshot case", p, l, false, kForever, 150);
    }

    // Corner sweep: the wave's bottom-right corner passes the block's
    // top-left corner `off` units above (> 0) or below (< 0), at different
    // phases (where within a step it crosses x = the block's side). Model
    // rule (sim.hpp, block loop): it lands on the top if, on the first step
    // its outer box touches the block, its bottom was at or above the top
    // the step before, also at a corner it only reached in x this step.
    // So: off > 0 always slides; off below one step's vertical move always
    // crashes (side hit, then the inner box runs into the block); in between
    // the outcome depends on the phase, as the stated rule says (GD's own
    // rule is assumed to be this "last y" one too; see report).
    for (bool mini : { false, true }) {
        int wrong = 0;
        float half = mini ? 3.f : 5.f;
        for (float off = -3.f; off <= 3.01f; off += 0.2f) {
            if (std::abs(off) < 0.05f) continue;
            for (float phase = 0.f; phase < 1.f; phase += 0.125f) {
                Level l = { block(60.f, 250.f, 90.f, 30.f) };
                auto p = waveTo(60.f - half, 280.f + off + half, false, 20.f + phase, mini);
                float dy = dyOf(p);
                auto r = hold(p, l, 100);
                // The stated rule, stepped here on its own.
                bool lands = false;
                float x = p.center.x, y = p.center.y;
                for (int i = 0; i < 100; ++i) {
                    float nx = x + p.dx, ny = y - dy;
                    Rect outer(nx - half, ny - half, 2 * half, 2 * half);
                    if (outer.intersectsRect(l[0].rect)) { lands = y - half >= 280.f - 0.01f; break; }
                    x = nx; y = ny;
                }
                bool ok = survived(r, 100) == lands;
                if (off > 0.f && !survived(r, 100)) ok = false;
                if (off < -dy - 0.05f && survived(r, 100)) ok = false;
                if (!ok) {
                    ++wrong;
                    std::printf("  corner sweep mini %d off %+.1f phase %.3f (rule: %s): %s\n", mini, off, phase,
                        lands ? "lands" : "side hit", describe(r).c_str());
                }
                checkInvariants("corner sweep", p, l, r);
            }
        }
        CHECK(wrong == 0, "corner sweep (mini %d): %d cases disagree with the stated landing rule", mini, wrong);
    }
    // Same for a wave rising onto the bottom-left corner of a block.
    {
        int wrong = 0;
        for (float off = -3.f; off <= 3.01f; off += 0.2f) {
            if (std::abs(off) < 0.05f) continue;
            for (float phase = 0.f; phase < 1.f; phase += 0.125f) {
                Level l = { block(60.f, 300.f, 90.f, 30.f) }; // bottom 300
                auto p = waveTo(55.f, 300.f - off - 5.f, true, 20.f + phase);
                auto r = hold(p, l, 100);
                if (off > 0.f && !survived(r, 100)) ++wrong;
                if (off < -dyOf(p) - 0.05f && survived(r, 100)) ++wrong;
                checkInvariants("ceiling corner sweep", p, l, r);
            }
        }
        CHECK(wrong == 0, "ceiling corner sweep: %d cases wrong", wrong);
    }

    // =====================================================================
    // Walls
    // =====================================================================
    for (bool mini : { false, true }) {
        for (bool up : { false, true }) {
            Level l = { block(60.f, 100.f, 30.f, 400.f) };
            auto p = wave(0.f, 300.f, up, mini);
            auto r = hold(p, l, 100);
            CHECK(!survived(r, 100) && std::strcmp(r.result.cause, "a block") == 0, "wall (up %d mini %d): %s", up,
                mini, describe(r).c_str());
            // Crashes when the inner box reaches x = 60.
            float innerW = p.halfW * kInnerScale;
            float crashX = 60.f - innerW;
            int expect = static_cast<int>(std::ceil((crashX - 0.f) / p.dx)) - 1;
            CHECK(std::abs(r.result.survived - expect) <= 1, "wall crash step %d, expected ~%d", r.result.survived, expect);
            checkInvariants("wall", p, l, r);
            checkMirror("wall", p, l, heldFor(up, false), kForever, 100);
        }
        // Wave sliding on a block floor into a taller block: crash.
        Level l;
        row(l, 0.f, 150.f, 4);
        l.push_back(block(120.f, 150.f, 30.f, 90.f));
        auto s = slider(5.f, 180.f + (mini ? 3.f : 5.f), mini);
        auto r = holdButton(s, l, false, 150);
        CHECK(!survived(r, 150), "slide into a step up: %s", describe(r).c_str());
        checkInvariants("slide into a step up", s, l, r);
    }
    // Mini crashes no earlier than normal at the same wall (smaller box).
    {
        Level l = { block(60.f, 100.f, 30.f, 400.f) };
        auto a = hold(wave(0.f, 300.f, true), l, 100);
        auto b = hold(wave(0.f, 300.f, true, true), l, 100);
        CHECK(b.result.survived >= a.result.survived, "mini wall %d vs normal %d", b.result.survived, a.result.survived);
    }

    // =====================================================================
    // A 1-block gap in a wall (wave rises through it, sliding on its roof)
    // =====================================================================
    for (bool mini : { false, true }) {
        Level l = { block(90.f, 100.f, 30.f, 130.f), block(90.f, 260.f, 30.f, 200.f) }; // gap 230..260
        // Rising: the box's bottom-right corner enters the gap 2 above its floor.
        float half = mini ? 3.f : 5.f;
        auto p = waveTo(90.f - half, 232.f + half, true, 30.f, mini);
        auto r = hold(p, l, 100);
        CHECK(survived(r, 100), "1-block gap rising (mini %d): %s", mini, describe(r).c_str());
        checkInvariants("1-block gap rising", p, l, r);
        checkMirror("1-block gap rising", p, l, true, kForever, 100);
        // Descending through it.
        auto q = waveTo(90.f - half, 258.f - half, false, 30.f, mini);
        auto rq = hold(q, l, 100);
        CHECK(survived(rq, 100), "1-block gap descending (mini %d): %s", mini, describe(rq).c_str());
        checkInvariants("1-block gap descending", q, l, rq);
        // Aimed at the wall below the gap: crash.
        auto w = waveTo(90.f - half, 200.f, true, 30.f, mini);
        auto rw = hold(w, l, 100);
        CHECK(!survived(rw, 100), "1-block gap missed: %s", describe(rw).c_str());
    }
    // A gap narrower than the outer box but wider than the inner box: the
    // model lets the wave through (only the inner box kills on blocks).
    {
        Level l = { block(90.f, 100.f, 30.f, 140.f), block(90.f, 248.f, 30.f, 200.f) }; // gap 240..248 (8 units)
        // Horizontal entry is impossible for a wave, so come from a slide on
        // a ledge at the gap's floor height.
        Level l2 = l;
        l2.push_back(block(0.f, 100.f, 90.f, 140.f)); // top 240
        auto s = slider(5.f, 240.f + 5.f);
        auto r = holdButton(s, l2, false, 100);
        CHECK(survived(r, 100), "slide through an 8-unit gap (inner box fits): %s", describe(r).c_str());
        checkInvariants("slide through an 8-unit gap", s, l2, r);
    }

    // =====================================================================
    // Staircases of blocks
    // =====================================================================
    for (bool mini : { false, true }) {
        // Descending staircase (each step half a block lower), holding down:
        // slides along each step and drops to the next one.
        Level stairs;
        for (int i = 0; i < 8; ++i) stairs.push_back(block(i * 30.f, kFloorY, 30.f, 240.f - i * 15.f));
        auto p = wave(5.f, 340.f, false, mini);
        auto r = hold(p, stairs, 200);
        CHECK(survived(r, 200), "descending staircase (mini %d): %s", mini, describe(r).c_str());
        checkInvariants("descending staircase", p, stairs, r);
        // Ascending staircase of 1-block steps, rising 45 degrees just clear
        // above the corners: survives (normal); mini is steeper and clears.
        Level up;
        for (int i = 0; i < 8; ++i) up.push_back(block(100.f + i * 30.f, kFloorY, 30.f, 30.f * (i + 1)));
        float half = mini ? 3.f : 5.f;
        // Bottom-right corner passes 3 units above the first step's top-left corner.
        auto q = waveTo(100.f - half, kFloorY + 30.f + 3.f + half, true, 5.f, mini);
        auto rq = hold(q, up, 150);
        CHECK(survived(rq, 150), "ascending staircase just above corners (mini %d): %s", mini, describe(rq).c_str());
        checkInvariants("ascending staircase", q, up, rq);
        // Ascending staircase, holding down: crashes into the first riser.
        auto d = slider(5.f, kFloorY + q.halfH, mini);
        auto rd = holdButton(d, up, false, 150);
        CHECK(!survived(rd, 150), "staircase hit while sliding on floor: %s", describe(rd).c_str());
    }

    // =====================================================================
    // Spikes and saws
    // =====================================================================
    for (bool mini : { false, true }) {
        // Spike on the floor in the slide path: crash.
        Level l = { spike(60.f, kFloorY) };
        auto s = slider(0.f, kFloorY + (mini ? 3.f : 5.f), mini);
        auto r = holdButton(s, l, false, 100);
        CHECK(!survived(r, 100) && std::strcmp(r.result.cause, "a spike") == 0, "floor spike slide: %s", describe(r).c_str());
        // Hanging spike above the ceiling slide path: crash.
        Level c = { block(0.f, 300.f, 300.f, 30.f), spikeDown(90.f, 300.f) };
        auto q = slider(0.f, 300.f - (mini ? 3.f : 5.f), mini);
        auto rc = holdButton(q, c, true, 100);
        CHECK(!survived(rc, 100) && std::strcmp(rc.result.cause, "a spike") == 0, "ceiling spike slide: %s", describe(rc).c_str());
        checkMirror("ceiling spike slide", q, c, true, kForever, 100);
        // A spike corridor (spikes on floor and ceiling, 3 blocks apart):
        // flying through the middle survives.
        Level corr;
        for (int i = 0; i < 8; ++i) {
            corr.push_back(block(i * 30.f, 150.f));
            corr.push_back(spike(i * 30.f, 180.f));
            corr.push_back(block(i * 30.f, 300.f));
            corr.push_back(spikeDown(i * 30.f, 300.f));
        }
        auto w = wave(0.f, 225.f, true, mini);
        auto rw = segments(w, corr, { { true, mini ? 8 : 15 }, { false, mini ? 16 : 30 }, { true, mini ? 16 : 30 },
            { false, mini ? 16 : 30 } });
        int total = mini ? 56 : 105;
        CHECK(survived(rw, total), "spike corridor middle (mini %d): %s", mini, describe(rw).c_str());
        checkInvariants("spike corridor", w, corr, rw);
        // Same corridor holding up: hits a ceiling spike, not a block.
        auto ru = hold(wave(0.f, 225.f, true, mini), corr, 100);
        CHECK(!survived(ru, 100) && std::strcmp(ru.result.cause, "a spike") == 0, "spike corridor holding up: %s",
            describe(ru).c_str());
        // Saw in the path.
        Level sl = { saw(100.f, 200.f + 100.f * (mini ? 2.f : 1.f), 20.f) };
        auto a = wave(0.f, 200.f, true, mini);
        auto ra = hold(a, sl, 150);
        CHECK(!survived(ra, 150) && std::strcmp(ra.result.cause, "a saw") == 0, "saw: %s", describe(ra).c_str());
        // Passing just under the saw's reach.
        auto b = wave(0.f, 200.f, false, mini);
        auto rb = hold(b, { saw(100.f, 230.f, 20.f) }, 60);
        CHECK(survived(rb, 60), "under a saw: %s", describe(rb).c_str());
    }

    // =====================================================================
    // Slopes: single triangles as floors and ceilings
    // =====================================================================
    for (bool mini : { false, true }) {
        float half = mini ? 3.f : 5.f;
        // Rising 45 floor slope from the game floor, going up parallel: the
        // normal wave slides up it; mini is steeper and leaves it.
        {
            Level l = { slope(60.f, kFloorY, 90.f, 90.f, true, true), block(150.f, kFloorY, 60.f, 90.f) };
            auto s = slider(5.f, kFloorY + half, mini);
            // Sliding along the floor (held down) into the rising slope: the
            // model's rule says running into the rising face crashes.
            auto r = holdButton(s, l, false, 160);
            CHECK(!survived(r, 160) && std::strcmp(r.result.cause, "a slope") == 0,
                "floor slide into rising slope while holding down (mini %d): %s", mini, describe(r).c_str());
            checkInvariants("floor into rising slope", s, l, r);
            // Pressing up just as the box reaches the slope's foot (its
            // corner already a little inside): rides up, no jump.
            int foot = static_cast<int>((60.f - half - 5.f) / s.dx) + 1;
            auto rf = run(s, l, false, foot, -1, 120);
            CHECK(survived(rf, 120), "floor slide, up at the slope's foot (mini %d): %s", mini, describe(rf).c_str());
            checkInvariants("floor slide, up at the slope's foot", s, l, rf);
            // Holding up from the start: the normal wave rises parallel with
            // the slope only if it is already on it; from the floor at x=5 it
            // flies above it. Survives either way.
            auto u = slider(5.f, kFloorY + half, mini);
            auto ru = holdButton(u, l, true, 150);
            CHECK(survived(ru, 150), "rising above a rising slope (mini %d): %s", mini, describe(ru).c_str());
            checkInvariants("rising above a rising slope", u, l, ru);
        }
        // Sliding up a rising 45 slope (normal wave, on it): survives, and
        // stays on its surface.
        if (!mini) {
            Level l = { slope(60.f, kFloorY, 90.f, 90.f, true, true) };
            // On the slope: bottom-right corner on the line.
            float cx = 80.f;
            auto p = wave(cx, kFloorY + (cx + half - 60.f) + half, true, mini);
            auto r = hold(p, l, 50);
            CHECK(survived(r, 50), "slide up a 45 slope: %s", describe(r).c_str());
            checkInvariants("slide up a 45 slope", p, l, r);
            checkMirror("slide up a 45 slope (lifted)", [&] { auto q = p; q.center.y += 200.f; return q; }(),
                { slope(60.f, kFloorY + 200.f, 90.f, 90.f, true, true) }, true, kForever, 50);
        }
        // Falling 45 floor slope, going down parallel: slides down it to the
        // floor.
        {
            Level l = { block(0.f, kFloorY, 60.f, 90.f), slope(60.f, kFloorY, 90.f, 90.f, false, true) };
            auto s = slider(5.f, 180.f + half, mini);
            auto r = holdButton(s, l, false, 200);
            CHECK(survived(r, 200), "block -> falling slope -> floor (mini %d): %s", mini, describe(r).c_str());
            CHECK(std::abs(r.path.back().y - (kFloorY + half)) < 0.01f, "ends on the floor: %.2f", r.path.back().y);
            checkInvariants("block -> falling slope -> floor", s, l, r);
            // Lifted copy for the mirror test (clear of the game floor).
            Level lifted = { block(0.f, 200.f, 60.f, 90.f), slope(60.f, 200.f, 90.f, 90.f, false, true),
                block(150.f, 170.f, 100.f, 30.f) };
            auto s2 = slider(5.f, 290.f + half, mini);
            checkMirror("block -> falling slope", s2, lifted, false, kForever, 200);
        }
        // Falling 2:1 floor slope (shallower than the wave): lands and slides
        // at the slope's angle, no jump.
        {
            Level l = { slope(60.f, 120.f, 120.f, 60.f, false, true), block(60.f, kFloorY, 120.f, 30.f) };
            auto p = wave(0.f, 200.f + 60.f * (mini ? 2.f : 1.f), false, mini);
            auto r = hold(p, l, 200);
            CHECK(survived(r, 200), "land on a falling 2:1 slope (mini %d): %s", mini, describe(r).c_str());
            checkInvariants("land on a falling 2:1 slope", p, l, r);
        }
        // Rising 2:1 floor slope met while going down: the model crashes
        // (lands only on flat or falling-away surfaces).
        {
            Level l = { slope(60.f, kFloorY, 120.f, 60.f, true, true) };
            auto p = wave(0.f, 220.f, false, mini);
            auto r = hold(p, l, 200);
            CHECK(!survived(r, 200) && std::strcmp(r.result.cause, "a slope") == 0, "down onto rising 2:1 slope: %s",
                describe(r).c_str());
            checkInvariants("down onto rising 2:1 slope", p, l, r);
        }
        // Rising 2:1 floor slope met while going up: flies over it.
        {
            Level l = { slope(60.f, kFloorY, 120.f, 60.f, true, true) };
            auto p = wave(0.f, kFloorY + half + 1.f, true, mini);
            auto r = hold(p, l, 150);
            CHECK(survived(r, 150), "up over rising 2:1 slope: %s", describe(r).c_str());
            checkInvariants("up over rising 2:1 slope", p, l, r);
        }
        // Ceiling slopes, mirror images of the above.
        {
            Level l = { block(0.f, 400.f, 60.f, 90.f), slope(60.f, 400.f, 90.f, 90.f, true, false) };
            auto s = slider(5.f, 400.f - half, mini);
            auto r = holdButton(s, l, true, 120);
            CHECK(survived(r, 120), "block ceiling -> rising ceiling slope (mini %d): %s", mini, describe(r).c_str());
            checkInvariants("block ceiling -> rising ceiling slope", s, l, r);
        }
        // Hazard slope: touching kills, flying above does not.
        {
            Level l = { slope(60.f, 200.f, 90.f, 90.f, false, true, true) };
            auto p = wave(0.f, 320.f, false, mini);
            auto r = hold(p, l, 150);
            CHECK(!survived(r, 150) && std::strcmp(r.result.cause, "a hazard slope") == 0, "down along a hazard slope: %s",
                describe(r).c_str());
            auto q = wave(0.f, 310.f, true, mini);
            auto rq = hold(q, l, 150);
            CHECK(survived(rq, 150), "above a hazard slope: %s", describe(rq).c_str());
        }
    }

    // Landing on a falling 45 slope from a steeper (mini) descent: slides at
    // 45, no jump.
    {
        Level l = { slope(60.f, 200.f, 90.f, 90.f, false, true), block(60.f, 170.f, 200.f, 30.f) };
        auto p = wave(0.f, 420.f, false, true);
        auto r = hold(p, l, 120);
        CHECK(survived(r, 120), "mini lands on falling 45 slope: %s", describe(r).c_str());
        checkInvariants("mini lands on falling 45 slope", p, l, r);
        checkMirror("mini lands on falling 45 slope", p, l, false, kForever, 120);
    }

    // =====================================================================
    // Transitions
    // =====================================================================
    // Rising slope -> block on top (staircase of slope + block): slide up
    // the slope, keep going up past its top.
    {
        Level l = { slope(60.f, 200.f, 30.f, 30.f, true, true), block(90.f, 200.f, 60.f, 30.f), block(0.f, 170.f, 60.f, 30.f) };
        auto p = slider(5.f, 205.f);
        auto r = holdButton(p, l, false, 120);
        checkInvariants("block -> rising slope -> block", p, l, r);
        // While sliding (down), the model hits the rising slope's face.
        CHECK(!survived(r, 120), "slide into rising 45 slope from a block: %s", describe(r).c_str());
        // Pressing up at the slope's foot: rides up and over the block.
        auto rf = run(p, l, false, 40, -1, 120);
        CHECK(survived(rf, 120), "slide, up at the slope's foot: %s", describe(rf).c_str());
        checkInvariants("block -> rising slope (up at foot) -> block", p, l, rf);
        // On the slope going up: continues over the block.
        auto q = wave(70.f, 200.f + 15.f + 5.f, true);
        auto rq = hold(q, l, 80);
        CHECK(survived(rq, 80), "slope -> block, going up: %s", describe(rq).c_str());
        checkInvariants("slope -> block, going up", q, l, rq);
        // On the slope, turning down after it: lands on the block and slides.
        auto rt = turnAt(q, l, 20, 120);
        CHECK(survived(rt, 120), "slope then down onto the block: %s", describe(rt).c_str());
        checkInvariants("slope then down onto the block", q, l, rt);
        checkMirror("slope -> block, going up", q, l, true, kForever, 80);
    }
    // Falling slope ending on a lower block (sliding down onto a ledge).
    {
        Level l = { block(0.f, 200.f, 60.f, 60.f), slope(60.f, 230.f, 30.f, 30.f, false, true),
            block(60.f, 200.f, 30.f, 30.f), block(90.f, 200.f, 90.f, 30.f) };
        auto p = slider(5.f, 265.f);
        auto r = holdButton(p, l, false, 140);
        CHECK(survived(r, 140), "block -> falling slope -> lower block: %s", describe(r).c_str());
        checkInvariants("block -> falling slope -> lower block", p, l, r);
        checkMirror("block -> falling slope -> lower block", p, l, false, kForever, 140);
    }
    // Falling 45 slope then a falling 2:1 slope (landing on a shallower
    // slope): stays on the surface, no jump.
    for (bool mini : { false, true }) {
        float half = mini ? 3.f : 5.f;
        Level l = { block(0.f, 200.f, 60.f, 90.f), slope(60.f, 260.f, 30.f, 30.f, false, true),
            block(60.f, 200.f, 30.f, 60.f), slope(90.f, 230.f, 60.f, 30.f, false, true), block(90.f, 200.f, 60.f, 30.f),
            block(150.f, 200.f, 90.f, 30.f) };
        auto p = slider(5.f, 290.f + half, mini);
        auto r = holdButton(p, l, false, 150);
        CHECK(survived(r, 150), "45 -> 2:1 -> flat (mini %d): %s", mini, describe(r).c_str());
        CHECK(std::abs(r.path.back().y - (230.f + half)) < 0.01f, "ends on the flat block: %.2f", r.path.back().y);
        checkInvariants("45 -> 2:1 -> flat", p, l, r);
        checkMirror("45 -> 2:1 -> flat", p, l, false, kForever, 150);
    }
    // Two 45 slopes in a row (seam between triangles), sliding up.
    {
        Level l = { slope(60.f, 200.f, 30.f, 30.f, true, true), slope(90.f, 230.f, 30.f, 30.f, true, true),
            block(90.f, 200.f, 30.f, 30.f), slope(120.f, 260.f, 30.f, 30.f, true, true), block(120.f, 200.f, 30.f, 60.f) };
        auto q = wave(64.f, 200.f + 9.f + 5.f, true);
        auto r = hold(q, l, 60);
        CHECK(survived(r, 60), "slide up two seamed 45 slopes: %s", describe(r).c_str());
        checkInvariants("slide up two seamed 45 slopes", q, l, r);
        // Going down the same seamed slopes from the top (mirror image in x
        // is not supported, so build falling ones).
        Level f = { slope(60.f, 260.f, 30.f, 30.f, false, true), block(60.f, 200.f, 30.f, 60.f),
            slope(90.f, 230.f, 30.f, 30.f, false, true), block(90.f, 200.f, 30.f, 30.f),
            slope(120.f, 200.f, 30.f, 30.f, false, true), block(0.f, 200.f, 60.f, 90.f), block(150.f, 170.f, 90.f, 30.f) };
        auto s = slider(5.f, 295.f);
        auto rf = holdButton(s, f, false, 150);
        CHECK(survived(rf, 150), "slide down three seamed 45 slopes: %s", describe(rf).c_str());
        checkInvariants("slide down three seamed 45 slopes", s, f, rf);
    }

    // =====================================================================
    // Zigzag corridors made of 45-degree slope triangles
    // =====================================================================
    for (float gap : { 30.f, 45.f, 60.f }) {
        for (bool mini : { false }) {
            // Corridor: floor line starts at y0 rising for w, falling for w, ...
            float y0 = 200.f, w = 60.f;
            auto l = zigzag(0.f, y0, w, 6, gap);
            // Enter at the middle of the gap at x = 0.
            auto p = wave(-0.f, y0 + gap / 2, true, mini);
            int seg = static_cast<int>(std::lround(w / p.dx));
            int first = static_cast<int>(std::lround((w) / p.dx));
            std::vector<std::pair<bool, int>> segs = { { true, first } };
            for (int i = 1; i < 6; ++i) segs.push_back({ i % 2 == 0, seg });
            auto r = segments(p, l, segs);
            int total = first + 5 * seg;
            char name[64];
            std::snprintf(name, sizeof name, "zigzag corridor gap %.0f", gap);
            CHECK(survived(r, total), "%s: %s", name, describe(r).c_str());
            checkInvariants(name, p, l, r);
            // Holding up the whole way: hits the first falling ceiling slope.
            auto ru = hold(p, l, 300);
            CHECK(!survived(ru, 300), "%s holding up: %s", name, describe(ru).c_str());
            CHECK(ru.result.survived >= first - 3, "%s holding up crashes too early (%d < %d)", name, ru.result.survived, first);
            checkInvariants("zigzag holding up", p, l, ru);
            checkMirror(name, p, l, true, first, 2 * seg);
        }
    }
    // Zigzag, sliding along the floor slopes (touching): down the falling
    // floor parallel is a slide, then running into the rising floor at the
    // valley is a crash (model rule: rising face in the direction of
    // travel).
    {
        auto l = zigzag(0.f, 200.f, 60.f, 4, 45.f);
        // On the falling floor section (x 60..120), bottom-left corner on the line.
        float cx = 70.f, half = 5.f;
        float line = 260.f - (cx - half - 60.f);
        auto p = wave(cx, line + half, false);
        auto r = hold(p, l, 80);
        CHECK(!survived(r, 80) && std::strcmp(r.result.cause, "a slope") == 0, "zigzag valley while sliding down: %s",
            describe(r).c_str());
        CHECK(r.result.survived >= static_cast<int>((120.f - cx) / p.dx) - 4, "zigzag: slid until the valley (%d)",
            r.result.survived);
        checkInvariants("zigzag slide down", p, l, r);
        // Same but turn up at the valley: slide up the rising floor.
        int valley = static_cast<int>((120.f - cx) / p.dx);
        auto rt = turnAt(p, l, valley, 60);
        CHECK(survived(rt, 60), "zigzag slide down, up at the valley: %s", describe(rt).c_str());
        checkInvariants("zigzag slide down, up at valley", p, l, rt);
    }
    // Zigzag without fill blocks (just the triangles), wide gap: same result.
    {
        auto l = zigzag(0.f, 200.f, 60.f, 4, 60.f, false);
        auto p = wave(0.f, 230.f, true);
        int seg = static_cast<int>(std::lround(60.f / p.dx));
        auto r = segments(p, l, { { true, seg }, { false, seg }, { true, seg }, { false, seg } });
        CHECK(survived(r, 4 * seg), "unfilled zigzag: %s", describe(r).c_str());
        checkInvariants("unfilled zigzag", p, l, r);
    }
    // Mini wave in a 2:1 zigzag (slopes at the mini wave's angle).
    {
        Level l;
        float y0 = 200.f, w = 30.f, h = 60.f, gap = 50.f;
        for (int i = 0; i < 6; ++i) {
            bool rising = i % 2 == 0;
            l.push_back(slope(i * w, y0, w, h, rising, true));
            l.push_back(slope(i * w, y0 + gap, w, h, rising, false));
        }
        auto p = wave(0.f, y0 + gap / 2, true, true);
        int seg = static_cast<int>(std::lround(w / p.dx));
        auto r = segments(p, l, { { true, seg }, { false, seg }, { true, seg }, { false, seg }, { true, seg } });
        CHECK(survived(r, 5 * seg), "mini steep zigzag: %s", describe(r).c_str());
        checkInvariants("mini steep zigzag", p, l, r);
        // Sliding along the steep floor (parallel for mini).
        auto s = wave(30.f + 2.f, y0 + h - 2.f * (2.f + 3.f - 0.f) + 3.f + 0.5f, false, true);
        auto rs = hold(s, l, 10);
        checkInvariants("mini slide on steep slope", s, l, rs);
    }

    // =====================================================================
    // Diamonds of 4 slope triangles
    // =====================================================================
    for (bool mini : { false, true }) {
        float half = mini ? 3.f : 5.f;
        float cx = 150.f, cy = 300.f, s = 30.f;
        auto d = diamond(cx, cy, s);
        // Passing well above / below: survive.
        auto a = wave(0.f, 300.f, true, mini);
        auto ra = hold(a, d, 150);
        CHECK(survived(ra, 150), "above a diamond: %s", describe(ra).c_str());
        auto b = wave(0.f, 300.f + 2.f, false, mini);
        // Going down along the upper-right face after flying over the top.
        auto rb = hold(b, d, 150);
        checkInvariants("diamond descending", b, d, rb);
        // Head-on at the left tip (center at the tip's height): crash.
        auto h = wave(cx - s - 40.f, cy - 40.f * (mini ? 2.f : 1.f), true, mini);
        auto rh = hold(h, d, 150);
        CHECK(!survived(rh, 150), "into a diamond's left face (mini %d): %s", mini, describe(rh).c_str());
        checkInvariants("into a diamond's left face", h, d, rh);
        // Sliding over the top: rise parallel along the upper-left face
        // (normal) then go down along the upper-right face.
        if (!mini) {
            float x = cx - s + 5.f;
            auto t = wave(x, cy + (x + half - (cx - s)) + half, true);
            int peak = static_cast<int>((cx - half - x) / t.dx) + 1;
            auto rt = turnAt(t, d, peak, 60);
            CHECK(survived(rt, 60), "over a diamond's top: %s", describe(rt).c_str());
            checkInvariants("over a diamond's top", t, d, rt);
            checkMirror("over a diamond's top", t, d, true, peak, 60);
            // Below: slide along the lower-left face going down, then up the
            // lower-right face.
            auto u = wave(x, cy - (x + half - (cx - s)) - half, false);
            auto ru = turnAt(u, d, peak, 60);
            CHECK(survived(ru, 60), "under a diamond's bottom: %s", describe(ru).c_str());
            checkInvariants("under a diamond's bottom", u, d, ru);
        }
        // Tip sweep: rising at 45 past the left tip with the box's bottom
        // edge from 3 below the tip height to 3 above: no vertical jump
        // larger than one step, and nothing ends up inside the diamond.
        for (float off = -3.f; off <= 3.01f; off += 0.25f) {
            auto t = waveTo(cx - s - half, cy + off + half, true, 20.f, mini);
            auto rt = hold(t, d, 80);
            char name[64];
            std::snprintf(name, sizeof name, "diamond tip rising off %.2f mini %d", off, mini);
            checkInvariants(name, t, d, rt);
        }
        // Same descending past the tip.
        for (float off = -3.f; off <= 3.01f; off += 0.25f) {
            auto t = waveTo(cx - s - half, cy + off - half, false, 20.f, mini);
            auto rt = hold(t, d, 80);
            char name[64];
            std::snprintf(name, sizeof name, "diamond tip descending off %.2f mini %d", off, mini);
            checkInvariants(name, t, d, rt);
        }
        // Hazard diamond: any touch kills.
        auto dd = diamond(cx, cy, s, true);
        auto rk = hold(waveTo(cx - s, cy - 10.f, true, 20.f, mini), dd, 150);
        CHECK(!survived(rk, 150) && std::strcmp(rk.result.cause, "a hazard slope") == 0, "hazard diamond: %s",
            describe(rk).c_str());
    }

    // =====================================================================
    // Slope next to a block (block's side meets slope face)
    // =====================================================================
    {
        // Falling floor slope right after a taller block's top-right corner:
        // sliding off the block onto the slope (same height) is smooth.
        Level l = { block(0.f, 200.f, 90.f, 30.f), slope(90.f, 200.f, 30.f, 30.f, false, true), block(120.f, 170.f, 90.f, 30.f) };
        auto p = slider(5.f, 235.f);
        auto r = holdButton(p, l, false, 120);
        CHECK(survived(r, 120), "block -> falling slope same height: %s", describe(r).c_str());
        checkInvariants("block -> falling slope same height", p, l, r);
        // Rising slope ending against a wall: going up the slope into the wall crashes.
        Level w = { slope(60.f, 200.f, 30.f, 30.f, true, true), block(90.f, 200.f, 30.f, 200.f) };
        auto q = wave(64.f, 200.f + 9.f + 5.f, true);
        auto rq = hold(q, w, 60);
        CHECK(!survived(rq, 60) && std::strcmp(rq.result.cause, "a block") == 0, "slope into wall: %s", describe(rq).c_str());
        checkInvariants("slope into wall", q, w, rq);
    }
    // Falling floor slope's vertical back face (on its left) approached from
    // the left: a wall like a block's side. Rising into it 10 below its top
    // crashes; rising past its top corner by 3 survives.
    for (bool mini : { false, true }) {
        float half = mini ? 3.f : 5.f;
        Level l = { slope(90.f, 200.f, 30.f, 30.f, false, true), block(90.f, 170.f, 30.f, 30.f) };
        // Box top 10 below the face's top when it reaches the face.
        auto p = waveTo(90.f - half, 220.f - half, true, 20.f, mini);
        auto r = hold(p, l, 100);
        CHECK(!survived(r, 100), "falling slope back face (mini %d): %s", mini, describe(r).c_str());
        checkInvariants("falling slope back face", p, l, r);
        // Block-face equivalent crashes at about the same step.
        Level b = { block(90.f, 200.f, 30.f, 30.f), block(90.f, 170.f, 30.f, 30.f) };
        auto rb = hold(p, b, 100);
        CHECK(std::abs(rb.result.survived - r.result.survived) <= 2, "slope back face crash step %d vs block %d (mini %d)",
            r.result.survived, rb.result.survived, mini);
        // Corner graze at the face's top while rising: compare with a block.
        for (float off = -2.5f; off <= 2.51f; off += 0.5f) {
            auto t = waveTo(90.f - half, 230.f + off + half, true, 20.f, mini);
            auto rs = hold(t, l, 60);
            char name[64];
            std::snprintf(name, sizeof name, "falling slope top corner rising off %.1f mini %d", off, mini);
            checkInvariants(name, t, l, rs);
        }
    }
    // Descending diagonally (parallel) onto the top-left corner of a falling
    // 45 floor slope: the slope's corner should behave like the block's corner
    // above (slide if passing above, else side hit).
    {
        int jumps = 0;
        float worst = 0.f;
        for (float off = -2.5f; off <= 2.51f; off += 0.25f) {
            for (float phase = 0.f; phase < 1.f; phase += 0.25f) {
                Level l = { slope(60.f, 200.f, 60.f, 60.f, false, true), block(60.f, 170.f, 120.f, 30.f) };
                auto p = waveTo(55.f, 260.f + off + 5.f, false, 20.f + phase);
                auto r = hold(p, l, 80);
                // A path point that goes UP while the wave is going down.
                for (size_t i = 1; i < r.path.size(); ++i) {
                    float rise = r.path[i].y - r.path[i - 1].y;
                    if (rise > 0.01f) { ++jumps; worst = std::max(worst, rise); }
                }
                checkInvariants("falling slope top-left corner", p, l, r);
            }
        }
        CHECK(jumps == 0, "falling slope corner: wave holding down moved UP %d times (max %.2f): pulled up onto the slope "
            "from beside its corner", jumps, worst);
    }

    // Floating slope triangles: passing just under a floor triangle's flat
    // base (or over a ceiling triangle's base) grazes it with the outer box
    // only, like a block's bottom: the inner box never enters the solid, so
    // by the model's rule ("crashes once the inner box is in") it survives.
    for (bool mini : { false, true }) {
        float half = mini ? 3.f : 5.f;
        Level l = { slope(60.f, 300.f, 30.f, 30.f, true, true) }; // solid below line, base y = 300
        auto s = slider(5.f, 300.f - half + 1.f, mini);              // box top 1 above the base
        Level withRoof = l;
        withRoof.push_back(block(0.f, 200.f, 200.f, 300.f - half + 1.f - half - 200.f)); // floor to slide on
        auto r = holdButton(s, withRoof, false, 100);
        CHECK(survived(r, 100), "slide under a floating floor triangle's base (mini %d): %s", mini, describe(r).c_str());
        checkInvariants("slide under a floating triangle", s, withRoof, r);
        // Mirror: over a floating ceiling triangle's base.
        checkMirror("slide under a floating triangle", s, withRoof, false, kForever, 100);
    }

    // =====================================================================
    // Mirror symmetry of a mixed level
    // =====================================================================
    {
        Level l = zigzag(0.f, 200.f, 60.f, 4, 45.f);
        for (auto o : diamond(330.f, 260.f, 20.f)) l.push_back(o);
        l.push_back(spike(280.f, 200.f));
        l.push_back(saw(420.f, 300.f, 15.f));
        for (int at = 0; at < 80; at += 7) {
            auto p = wave(0.f, 222.f, true);
            checkMirror("mixed level", p, l, true, at, 200);
            checkInvariants("mixed level", p, l, turnAt(p, l, at, 200));
        }
    }

    // =====================================================================
    // choose() for the wave
    // =====================================================================
    // A press 1-6 steps early would turn the descending wave up into a
    // hanging spike; waiting passes under it. Delay > 0 and it survives.
    for (bool mini : { false, true }) {
        for (int early = 1; early <= 6; ++early) {
            auto p = wave(0.f, 300.f, false, mini);
            float dy = dyOf(p), half = p.halfH;
            // Up path after delay d: y = 300 + (x*slope) - 2*d*dy. The spike
            // hitbox spans x in [sx+12, sx+18], y in [ty-12, ty]. The box
            // misses it when its top at x = sx+18+half is below ty-12.
            float sx = 30.f;
            float xr = sx + 18.f + half;
            float yAt = 300.f + xr * p.waveSlope; // center y at xr for d = 0
            float ty = yAt + half + 12.f - 2.f * dy * (early - 0.5f);
            Level l = { spikeDown(sx, ty) };
            auto c = choose({ p }, l, true, 10, 0, 48);
            auto check = run(p, l, false, c.delay, -1, 48 + c.delay);
            CHECK(c.delay > 0 && survived(check, 48 + c.delay), "early press %d (mini %d): delay %d, now %d (%s), best %d",
                early, mini, c.delay, c.survivedNow, c.causeNow, c.survivedBest);
            CHECK(std::abs(c.delay - early) <= 1, "early press %d (mini %d): delay %d", early, mini, c.delay);
            // Upside down, mirrored level: same delay.
            auto m = choose({ mirror(p, 600.f) }, mirror(l, 600.f), true, 10, 0, 48);
            CHECK(m.delay == c.delay, "early press %d mirrored: delay %d vs %d", early, m.delay, c.delay);
        }
    }
    // Release early in a narrow 45 zigzag corridor: the wave going up turns
    // down before the peak and runs into the rising floor. A delay fixes it.
    // (Slopes kill only once the inner box crosses the line at the center,
    // so in a 1-block corridor the release has to be about 6+ steps early
    // to crash at all.)
    {
        float y0 = 200.f, w = 60.f, gap = 30.f;
        auto l = zigzag(0.f, y0, w, 4, gap);
        for (int early = 1; early <= 10; ++early) {
            auto p = wave(w - early * kSpeed1x, 0.f, true);
            p.center.y = y0 + p.center.x + gap / 2;
            auto c = choose({ p }, l, false, 12, 0, 30);
            auto now = run(p, l, true, 0, -1, 30);
            auto check = run(p, l, true, c.delay, -1, 30 + c.delay);
            if (early >= 7) {
                CHECK(!survived(now, 30), "zigzag early release %d: releasing now should crash: %s", early,
                    describe(now).c_str());
            }
            if (!survived(now, 30)) {
                CHECK(c.delay > 0 && survived(check, 30 + c.delay), "zigzag early release %d: delay %d, now %d (%s)", early,
                    c.delay, c.survivedNow, c.causeNow);
            }
            else CHECK(c.delay == 0, "zigzag early release %d survives now but delay %d", early, c.delay);
        }
    }
    // On-time press in open air: delay 0.
    {
        auto p = wave(0.f, 300.f, false);
        auto c = choose({ p }, {}, true, 10, 0, 48);
        CHECK(c.delay == 0 && c.survivedNow == 48, "on-time press: delay %d now %d", c.delay, c.survivedNow);
        // With the spike far above the path: still 0.
        auto c2 = choose({ p }, { spikeDown(30.f, 500.f) }, true, 10, 0, 48);
        CHECK(c2.delay == 0, "on-time press with harmless spike: delay %d", c2.delay);
        // On-time release in the zigzag at the peak: delay 0.
        auto l = zigzag(0.f, 200.f, 60.f, 4, 30.f);
        auto q = wave(60.f, 275.f, true);
        auto c3 = choose({ q }, l, false, 10, 0, 30);
        CHECK(c3.delay == 0 && c3.survivedNow == 30, "on-time release at zigzag peak: delay %d now %d (%s)", c3.delay,
            c3.survivedNow, c3.causeNow);
    }
    // Press into an unavoidable wall: delay 0.
    for (bool mini : { false, true }) {
        Level l = { block(40.f, kFloorY, 30.f, 900.f) };
        auto p = wave(0.f, 300.f, false, mini);
        auto c = choose({ p }, l, true, 10, 0, 48);
        CHECK(c.delay == 0 && c.survivedNow < 48, "unavoidable wall: delay %d now %d", c.delay, c.survivedNow);
    }
    // A delay whose wait itself crashes is not chosen: descending toward a
    // floor spike, a late press dies on it, pressing now is fine.
    {
        Level l = { spike(30.f, 270.f), block(30.f, 240.f) };
        auto p = wave(0.f, 300.f, false);
        auto c = choose({ p }, l, true, 10, 0, 48);
        CHECK(c.delay == 0, "press now avoids a floor spike: delay %d now %d (%s)", c.delay, c.survivedNow, c.causeNow);
    }

    return finish("test_wave");
}
