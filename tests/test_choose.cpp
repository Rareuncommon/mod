// choose(): when to fire a held input, the per-step re-evaluation main.cpp
// does while an input is held, dual players, robustness and speed. Also
// Rect, Obstacle::surfaceAt and Estimate.
#include "harness.hpp"

#include <chrono>
#include <cstdint>
#include <limits>

using namespace test;

namespace {

constexpr int kNever = 1 << 30;

// Steps survived after the input when it fires at `delay` (min over the
// players, capped at the lookahead), or -1 if waiting that long crashes.
int afterAt(std::vector<Player> const& ps, std::vector<Obstacle> const& level, bool push, int delay, int gap, int lookahead) {
    int steps = lookahead + delay;
    int survived = steps;
    for (auto const& p : ps) {
        auto r = simulate(p, level, !push, delay, gap > 0 ? delay + gap : -1, steps);
        survived = std::min(survived, r.survived);
    }
    if (survived < delay) return -1;
    return std::min(survived - delay, lookahead);
}

// The documented rule, written out independently: earliest delay with the
// longest survival, only if it beats firing now by kMinGain and firing now
// does not already survive the lookahead.
int referenceDelay(std::vector<Player> const& ps, std::vector<Obstacle> const& level, bool push, int maxDelay, int gap, int lookahead) {
    int now = afterAt(ps, level, push, 0, gap, lookahead);
    if (now >= lookahead) return 0;
    int best = now, bestDelay = 0;
    for (int d = 1; d <= maxDelay; ++d) {
        int a = afterAt(ps, level, push, d, gap, lookahead);
        if (a < 0) break;
        if (a > best) { best = a; bestDelay = d; }
    }
    return best >= now + kMinGain ? bestDelay : 0;
}

// One step of the game with the button not yet fired (the model stands in
// for the game). Returns false if the player crashed on that step.
bool advance(Player& p, std::vector<Obstacle> const& level, bool held) {
    std::vector<Vec> path;
    auto r = simulate(p, level, held, kNever, -1, 1, &path);
    if (r.survived < 1 || path.size() < 2) return false;
    p.lastDy = path[1].y - path[0].y;
    p.center = path[1];
    if (isGroundMode(p.mode)) p.grounded = std::abs(p.lastDy) < 1e-4f;
    return true;
}

struct Hold {
    Choice first;
    int fireAt = 0;          // physics steps after the input arrived
    bool crashedWaiting = false;
    std::vector<int> delays; // decision of every re-evaluation
};

// main.cpp: onWaveButton decides with the full window; if it holds,
// stepWave re-runs choose() every step with remaining = maxDelay - waited
// and fires as soon as it says 0.
//   betweenSteps = false: every re-evaluation sees the players one step
//     further on (waited == physics steps elapsed).
//   betweenSteps = true: as main.cpp counts when the click arrives between
//     processCommands calls: the first stepWave runs (waited = 1) before the
//     game has moved the players, so waited == elapsed + 1.
Hold holdLoop(std::vector<Player> ps, std::vector<Obstacle> const& level, bool push, int maxDelay, int gap, int lookahead,
              bool betweenSteps = false) {
    Hold h;
    h.first = choose(ps, level, push, maxDelay, gap, lookahead);
    h.delays.push_back(h.first.delay);
    if (h.first.delay == 0) return h;
    // PATCHED main.cpp loop: remaining counts physics steps elapsed since
    // the decision, a re-evaluation with nothing moved is skipped, and a held
    // input keeps waiting for any longer survival (minGain 1).
    for (int waited = 1; waited <= maxDelay + 2; ++waited) {
        int elapsed = betweenSteps ? waited - 1 : waited;
        if (elapsed <= 0) continue;
        for (auto& p : ps) {
            if (!advance(p, level, !push)) h.crashedWaiting = true;
        }
        int remaining = std::max(0, maxDelay - elapsed);
        auto c = choose(ps, level, push, remaining, gap, lookahead, 1);
        h.delays.push_back(c.delay);
        if (c.delay == 0) {
            h.fireAt = elapsed;
            return h;
        }
    }
    h.fireAt = -1; // never fired
    return h;
}

std::string fmtAfters(std::vector<Player> const& ps, std::vector<Obstacle> const& level, bool push, int maxDelay, int gap, int lookahead) {
    std::string s = "after[d]:";
    for (int d = 0; d <= maxDelay; ++d) s += " " + std::to_string(afterAt(ps, level, push, d, gap, lookahead));
    return s;
}

std::string fmtDelays(Hold const& h) {
    std::string s;
    for (int d : h.delays) s += std::to_string(d) + " ";
    return s;
}

Obstacle hazardRect(float x, float y, float w, float h) {
    Obstacle o;
    o.hazard = true;
    o.rect = Rect(x, y, w, h);
    return o;
}

// A wave sliding on the floor (moving right, button released).
Player floorWave(Physics const* ph) {
    auto p = player(Mode::Wave, 0.f, kFloorY + 5.f, ph);
    p.lastDy = 0.f;
    return p;
}

// A wave going down at y, button released.
Player fallingWave(Physics const* ph, float y) {
    auto p = player(Mode::Wave, 0.f, y, ph);
    p.lastDy = -kSpeed1x;
    return p;
}

// Ceiling of hazards spanning x in [x0, x1] at height y (bottom edge).
Obstacle ceiling(float x0, float x1, float y) { return hazardRect(x0, y, x1 - x0, 40.f); }

struct Lcg {
    uint64_t s;
    explicit Lcg(uint64_t seed) : s(seed) {}
    uint32_t next() { s = s * 6364136223846793005ull + 1442695040888963407ull; return uint32_t(s >> 33); }
    float uni(float a, float b) { return a + (b - a) * float(next() % 1000000) / 1000000.f; }
    int range(int a, int b) { return a + int(next() % uint32_t(b - a + 1)); }
};

bool finiteChoice(Choice const& c, int maxDelay, int lookahead) {
    return c.delay >= 0 && c.delay <= std::max(maxDelay, 0) && c.survivedNow >= 0 && c.survivedNow <= lookahead
        && c.survivedBest >= 0 && c.survivedBest <= lookahead;
}

} // namespace

int main() {
    auto wave = physicsFor(Mode::Wave);
    auto ship = physicsFor(Mode::Ship);
    auto cube = physicsFor(Mode::Cube);
    constexpr int L = 48;

    // ================= choose() semantics =================

    // Firing now survives the lookahead: delay 0.
    {
        std::vector<Player> ps { fallingWave(&wave, 300.f) };
        auto c = choose(ps, {}, true, 8, 0, L);
        CHECK(c.delay == 0 && c.survivedNow == L && c.survivedBest == L && c.lookahead == L,
            "delay %d now %d best %d", c.delay, c.survivedNow, c.survivedBest);
        auto s = player(Mode::Ship, 0.f, 250.f, &ship);
        c = choose({ s }, {}, true, 8, 0, L);
        CHECK(c.delay == 0 && c.survivedNow == L, "ship open air: delay %d now %d", c.delay, c.survivedNow);
        c = choose({ s }, {}, false, 8, 5, L);
        CHECK(c.delay == 0 && c.survivedNow == L, "ship open air tap: delay %d now %d", c.delay, c.survivedNow);
    }

    // Nothing better: a wall in front kills every path at the same absolute
    // step, so later delays survive less after the input. Delay 0.
    {
        std::vector<Obstacle> level { block(40.f, kFloorY, 30.f, 600.f) };
        std::vector<Player> ps { fallingWave(&wave, 300.f) };
        auto c = choose(ps, level, true, 8, 0, L);
        CHECK(c.delay == 0 && c.survivedNow < L, "wall: delay %d now %d (%s)", c.delay, c.survivedNow, c.causeNow);
        CHECK(std::string(c.causeNow) == "a block", "cause %s", c.causeNow);
        auto s = player(Mode::Ship, 0.f, 250.f, &ship);
        c = choose({ s }, level, true, 8, 0, L);
        CHECK(c.delay == 0 && c.survivedNow < L, "ship wall: delay %d now %d", c.delay, c.survivedNow);
    }

    // All delays crash after the same number of steps (ceiling hazard over a
    // floor-sliding wave: rising always takes the same time): delay 0.
    {
        std::vector<Obstacle> level { ceiling(-30.f, 600.f, kFloorY + 40.f) };
        std::vector<Player> ps { floorWave(&wave) };
        int a0 = afterAt(ps, level, true, 0, 0, L), a5 = afterAt(ps, level, true, 5, 0, L);
        CHECK(a0 < L && a0 == a5, "setup: %s", fmtAfters(ps, level, true, 8, 0, L).c_str());
        auto c = choose(ps, level, true, 8, 0, L);
        CHECK(c.delay == 0 && c.survivedBest == c.survivedNow, "equal crashes: delay %d now %d best %d",
            c.delay, c.survivedNow, c.survivedBest);
    }

    // Earliest best among equals (not full survival, so no early break):
    // floor-sliding wave, a low hazard right ahead kills an early rise, a
    // hazard ceiling kills every later rise after the same number of steps.
    {
        std::vector<Obstacle> level { hazardRect(6.f, kFloorY + 15.f, 1.f, 10.f), ceiling(-30.f, 600.f, kFloorY + 50.f) };
        std::vector<Player> ps { floorWave(&wave) };
        int ref = referenceDelay(ps, level, true, 8, 0, L);
        auto c = choose(ps, level, true, 8, 0, L);
        int aD = afterAt(ps, level, true, c.delay, 0, L), aNext = afterAt(ps, level, true, c.delay + 1, 0, L);
        CHECK(c.delay > 0 && c.delay < 8 && aD == aNext && aD < L, "setup/plateau: delay %d %s", c.delay,
            fmtAfters(ps, level, true, 8, 0, L).c_str());
        CHECK(c.delay == ref, "earliest best: got %d want %d (%s)", c.delay, ref, fmtAfters(ps, level, true, 8, 0, L).c_str());
    }

    // Earliest delay that survives the whole lookahead (early break), wave
    // going down, a hazard just above-ahead; and the same with a ship.
    {
        std::vector<Obstacle> level { hazardRect(8.f, 310.f, 1.f, 80.f) };
        std::vector<Player> ps { fallingWave(&wave, 300.f) };
        auto c = choose(ps, level, true, 8, 0, L);
        int ref = referenceDelay(ps, level, true, 8, 0, L);
        CHECK(c.survivedNow < L && c.delay > 0 && c.survivedBest == L && afterAt(ps, level, true, c.delay - 1, 0, L) < L,
            "delay %d now %d best %d %s", c.delay, c.survivedNow, c.survivedBest, fmtAfters(ps, level, true, 8, 0, L).c_str());
        CHECK(c.delay == ref, "got %d want %d", c.delay, ref);
    }
    {
        // Ship rising with the button held, a hazard below-ahead: releasing
        // now arcs down into it, releasing later arcs over it. The hazard
        // height is searched so that some delay is worth it.
        auto s = player(Mode::Ship, 0.f, 200.f, &ship);
        s.lastDy = 0.8f;
        bool found = false;
        for (float top = 170.f; top < 230.f && !found; top += 0.5f) {
            std::vector<Obstacle> level { hazardRect(40.f, kFloorY, 30.f, top - kFloorY) };
            std::vector<Player> ps { s };
            int ref = referenceDelay(ps, level, false, 8, 0, L);
            if (ref == 0) continue;
            found = true;
            auto c = choose(ps, level, false, 8, 0, L);
            CHECK(c.delay == ref, "ship: got %d want %d %s", c.delay, ref, fmtAfters(ps, level, false, 8, 0, L).c_str());
            CHECK(afterAt(ps, level, false, c.delay, 0, L) >= c.survivedNow + kMinGain, "ship gain");
        }
        CHECK(found, "ship setup where waiting is worth it");
    }

    // Never a delay where waiting crashes first: wave going down onto a
    // spike field; the press is needed soon. The chosen delay's waiting path
    // must be clean, and delays past the waiting crash are never considered.
    {
        std::vector<Obstacle> level;
        for (float x = 0.f; x < 300.f; x += 30.f) level.push_back(spike(x, kFloorY));
        // Firing now hits a hazard overhead; firing late hits the spikes.
        level.push_back(hazardRect(6.f, 150.f, 2.f, 200.f));
        std::vector<Player> ps { fallingWave(&wave, 120.f) };
        auto c = choose(ps, level, true, 30, 0, L);
        int waitCrash = simulate(ps[0], level, false, kNever, -1, 60).survived;
        CHECK(c.delay <= waitCrash, "delay %d but waiting crashes at %d", c.delay, waitCrash);
        int ref = referenceDelay(ps, level, true, 30, 0, L);
        CHECK(c.delay == ref, "got %d want %d %s", c.delay, ref, fmtAfters(ps, level, true, 12, 0, L).c_str());
        // Waiting crashes on step 0 (spike right under): nothing but 0.
        auto p = ps[0];
        p.center.y = kFloorY + 8.f;
        p.center.x = 15.f;
        auto c2 = choose({ p }, level, true, 8, 0, L);
        CHECK(c2.delay == 0, "waiting crashes at once: delay %d", c2.delay);
    }

    // kMinGain edges: falling wave under a hazard ceiling, every step of
    // delay is one more step after the press (after[d] = a0 + d). A window
    // of kMinGain - 1 must not wait, a window of kMinGain must.
    {
        std::vector<Player> ps { fallingWave(&wave, 250.f) };
        std::vector<Obstacle> level { ceiling(-30.f, 900.f, 270.f) };
        int a0 = afterAt(ps, level, true, 0, 0, L);
        bool linear = a0 + 8 < L;
        for (int d = 1; d <= 8; ++d) linear = linear && afterAt(ps, level, true, d, 0, L) == a0 + d;
        CHECK(linear, "setup linear: %s", fmtAfters(ps, level, true, 8, 0, L).c_str());
        auto below = choose(ps, level, true, kMinGain - 1, 0, L);
        auto at = choose(ps, level, true, kMinGain, 0, L);
        CHECK(below.delay == 0 && below.survivedBest == a0 + kMinGain - 1, "gain kMinGain-1: delay %d", below.delay);
        CHECK(at.delay == kMinGain && at.survivedBest == a0 + kMinGain, "gain kMinGain: delay %d", at.delay);
        // Ship: same idea, search a ceiling height giving gain exactly
        // kMinGain - 1 and kMinGain over the window.
        // Ship rising with the button held over a hazard floor: releasing
        // later lands later.
        auto sh = player(Mode::Ship, 0.f, 160.f, &ship);
        sh.lastDy = 0.3f;
        bool sb = false, sa = false;
        for (float top = 100.f; top < 150.f; top += 0.25f) {
            std::vector<Obstacle> sl { hazardRect(-30.f, kFloorY, 900.f, top - kFloorY) };
            for (int w = 1; w <= 8; ++w) {
                int n0 = afterAt({ sh }, sl, false, 0, 0, L), best = n0;
                for (int d = 1; d <= w; ++d) best = std::max(best, afterAt({ sh }, sl, false, d, 0, L));
                if (n0 >= L) continue;
                auto c = choose({ sh }, sl, false, w, 0, L);
                if (best - n0 == kMinGain - 1 && !sb) { sb = true; CHECK(c.delay == 0, "ship gain 2: delay %d", c.delay); }
                if (best - n0 == kMinGain && !sa) { sa = true; CHECK(c.delay > 0, "ship gain 3: delay %d", c.delay); }
            }
        }
        CHECK(sb && sa, "ship kMinGain setups found %d %d", sb, sa);
    }

    // maxDelay 0: only firing now is looked at.
    {
        std::vector<Obstacle> level { hazardRect(8.f, 310.f, 1.f, 80.f) };
        std::vector<Player> ps { fallingWave(&wave, 300.f) };
        auto c = choose(ps, level, true, 0, 0, L);
        CHECK(c.delay == 0 && c.survivedNow < L && c.survivedBest == c.survivedNow, "maxDelay 0: delay %d now %d best %d",
            c.delay, c.survivedNow, c.survivedBest);
        // Negative window (not passed by main.cpp, which clamps): still 0.
        auto c2 = choose(ps, level, true, -3, 0, L);
        CHECK(c2.delay == 0, "maxDelay -3: delay %d", c2.delay);
        // lookahead 0: nothing to look at, fire now.
        auto c3 = choose(ps, level, true, 8, 0, 0);
        CHECK(c3.delay == 0 && c3.survivedNow == 0, "lookahead 0: delay %d now %d", c3.delay, c3.survivedNow);
    }

    // followGap: the release follows the press at the same gap, whatever the
    // delay. A tap (press, release 10 later) in a corridor: wave falling,
    // the up-leg of the tap is what matters.
    {
        std::vector<Obstacle> level { hazardRect(8.f, 310.f, 1.f, 80.f) };
        std::vector<Player> ps { fallingWave(&wave, 300.f) };
        // The simulated path at delay d with gap g turns at d and back at d+g.
        for (int d : { 0, 3 }) {
            std::vector<Vec> path;
            simulate(ps[0], {}, false, d, d + 10, 30, &path);
            int turns = 0;
            for (size_t i = 2; i < path.size(); ++i) {
                float a = path[i - 1].y - path[i - 2].y, b = path[i].y - path[i - 1].y;
                if ((a > 0) != (b > 0)) ++turns;
            }
            float upLeg = path[d + 10].y - path[d].y;
            CHECK(turns == (d > 0 ? 2 : 1) && std::abs(upLeg - 10 * kSpeed1x) < 0.01f, "tap at %d: %d turns, up leg %.2f", d, turns, upLeg);
        }
        auto c = choose(ps, level, true, 8, 10, L);
        int ref = referenceDelay(ps, level, true, 8, 10, L);
        CHECK(c.delay == ref, "tap: got %d want %d %s", c.delay, ref, fmtAfters(ps, level, true, 8, 10, L).c_str());
        // A short tap (gap 2) never reaches the hazard: no need to wait.
        auto c2 = choose(ps, level, true, 8, 2, L);
        CHECK(c2.delay == 0 && c2.survivedNow == L, "short tap: delay %d now %d", c2.delay, c2.survivedNow);
        // Ship tap.
        auto s = player(Mode::Ship, 0.f, 200.f, &ship);
        s.lastDy = -0.6f;
        std::vector<Obstacle> sl { hazardRect(30.f, 214.f, 40.f, 60.f), block(-30.f, kFloorY, 900.f, 1.f) };
        auto c3 = choose({ s }, sl, true, 8, 12, L);
        int ref3 = referenceDelay({ s }, sl, true, 8, 12, L);
        CHECK(finiteChoice(c3, 8, L), "ship tap range");
        CHECK(c3.delay == ref3, "ship tap: got %d want %d", c3.delay, ref3);
    }

    // Lookahead boundary: wave flying straight up with spikes it reaches
    // exactly at step L (just beyond) or at L - 1 (just inside).
    {
        auto p = player(Mode::Wave, 0.f, 200.f, &wave);
        p.lastDy = kSpeed1x; // going up, released: press keeps it... released = going down after the press?
        // Push = false (release) with the button held: going up until the
        // release turns it down. Firing now: down. A floor of spikes below.
        int crashDown = -1;
        for (float y = 100.f; y < 200.f; y += 0.25f) {
            std::vector<Obstacle> level { hazardRect(-30.f, kFloorY, 900.f, y - kFloorY) };
            int s = simulate(p, level, true, 0, -1, 200).survived;
            if (s == L) { crashDown = 1; // crash exactly on step L: just beyond the lookahead
                auto c = choose({ p }, level, false, 8, 0, L);
                CHECK(c.delay == 0 && c.survivedNow == L, "obstacle just beyond: delay %d now %d", c.delay, c.survivedNow);
                break;
            }
        }
        CHECK(crashDown == 1, "found a level crashing exactly at step L");
        for (float y = 100.f; y < 200.f; y += 0.25f) {
            std::vector<Obstacle> level { hazardRect(-30.f, kFloorY, 900.f, y - kFloorY) };
            int s = simulate(p, level, true, 0, -1, 200).survived;
            if (s == L - 1) {
                auto c = choose({ p }, level, false, 8, 0, L);
                CHECK(c.survivedNow == L - 1, "just inside: now %d", c.survivedNow);
                // Delaying the release keeps it going up: every delay is
                // full, so a 1-step delay suffices... but gain is 1 < kMinGain.
                CHECK(c.delay == 0, "gain 1 < kMinGain: delay %d", c.delay);
                break;
            }
        }
    }

    // Cube on the floor: spike ahead; jumping now lands on a second spike,
    // jumping later clears both (reference rule decides).
    {
        auto p = player(Mode::Cube, 0.f, kFloorY + 15.f, &cube);
        p.grounded = true;
        std::vector<Obstacle> level { spike(60.f, kFloorY), spike(200.f, kFloorY) };
        for (int la : { 48, 120 }) {
            auto c = choose({ p }, level, true, 8, 0, la);
            int ref = referenceDelay({ p }, level, true, 8, 0, la);
            CHECK(c.delay == ref, "cube la %d: got %d want %d %s", la, c.delay, ref, fmtAfters({ p }, level, true, 8, 0, la).c_str());
            CHECK(finiteChoice(c, 8, la), "cube choice in range");
        }
    }

    // ================= re-evaluation while held =================

    // Step case: a hazard ceiling that ends at x1 over a floor-sliding
    // wave. A rise before step 6 hits it after a few steps, a rise from step
    // 6 passes its end. First decision: hold 6. The loop must keep holding
    // and fire on step 6, not earlier.
    {
        std::vector<Player> ps { floorWave(&wave) };
        std::vector<Obstacle> level { hazardRect(-30.f, kFloorY + 16.f, 38.5f, 20.f) };
        int full = -1;
        for (int d = 0; d <= 8; ++d) if (full < 0 && afterAt(ps, level, true, d, 0, L) == L) full = d;
        CHECK(full == 6, "setup: first full-survival delay %d (%s)", full, fmtAfters(ps, level, true, 8, 0, L).c_str());
        auto h = holdLoop(ps, level, true, 6, 0, L);
        CHECK(h.first.delay == 6 && h.fireAt == 6 && !h.crashedWaiting, "step case: first %d fired at %d (decisions %s)",
            h.first.delay, h.fireAt, fmtDelays(h).c_str());
        // Same, counted the way main.cpp counts when the click arrives
        // between steps: the first re-evaluation sees the unmoved player
        // with a window one shorter, cannot see delay 6 and fires at once.
        auto hm = holdLoop(ps, level, true, 6, 0, L, true);
        int afterFired = afterAt(ps, level, true, hm.fireAt, 0, L);
        CHECK(hm.fireAt == hm.first.delay && afterFired >= hm.first.survivedBest,
            "main.cpp counting: first decision hold %d (survives %d), fired at %d, survives %d (decisions %s)",
            hm.first.delay, hm.first.survivedBest, hm.fireAt, afterFired, fmtDelays(hm).c_str());
    }

    // Linear case: a falling wave under a hazard ceiling. Each step of delay
    // gives one more step after the press (a0 + d). First decision: wait the
    // whole window. The loop compares each new "now" with kMinGain again and
    // fires kMinGain - 1 steps early, at a worse moment than it planned.
    {
        std::vector<Player> ps { fallingWave(&wave, 250.f) };
        std::vector<Obstacle> level { ceiling(-30.f, 900.f, 270.f) };
        auto h = holdLoop(ps, level, true, 8, 0, L);
        int planned = afterAt(ps, level, true, h.first.delay, 0, L);
        int got = afterAt(ps, level, true, h.fireAt, 0, L);
        CHECK(h.first.delay == 8, "setup: first decision %d (%s)", h.first.delay, fmtAfters(ps, level, true, 8, 0, L).c_str());
        CHECK(h.fireAt <= 8, "fired at %d > maxDelay", h.fireAt);
        CHECK(got >= planned, "linear: planned delay %d (survives %d), fired at %d (survives %d); decisions %s",
            h.first.delay, planned, h.fireAt, got, fmtDelays(h).c_str());
        // Ship version: ship falling under a hazard ceiling.
        auto s = player(Mode::Ship, 0.f, 230.f, &ship);
        s.lastDy = -0.8f;
        std::vector<Obstacle> sl { ceiling(-30.f, 900.f, 262.f), block(-30.f, kFloorY, 900.f, 1.f) };
        auto hs = holdLoop({ s }, sl, true, 8, 0, L);
        int sp = afterAt({ s }, sl, true, hs.first.delay, 0, L), sg = afterAt({ s }, sl, true, hs.fireAt, 0, L);
        CHECK(hs.fireAt <= 8 && sg >= sp, "ship linear: planned %d (survives %d), fired at %d (survives %d); %s; decisions %s",
            hs.first.delay, sp, hs.fireAt, sg, fmtAfters({ s }, sl, true, 8, 0, L).c_str(), fmtDelays(hs).c_str());
    }

    // Randomised: many small levels, wave / ship / cube, presses and
    // releases, taps. choose() must match the reference rule, never pick a
    // delay where waiting crashes, and the hold loop must fire within the
    // window, never crash while holding, never undo a hold (decisions shrink
    // monotonically toward 0), and not fire at a worse moment than planned.
    {
        Lcg rng(12345);
        int cases = 0, mismatches = 0, waitCrash = 0, late = 0, crashedHolding = 0, worse = 0, holds = 0, nonMonotone = 0;
        std::string firstMismatch, firstWorse;
        for (int it = 0; it < 1500; ++it) {
            Mode modes[] = { Mode::Wave, Mode::Wave, Mode::Ship, Mode::Ship, Mode::Cube };
            Mode mode = modes[rng.range(0, 4)];
            Physics const* ph = mode == Mode::Wave ? &wave : mode == Mode::Ship ? &ship : &cube;
            auto p = player(mode, 0.f, mode == Mode::Cube ? kFloorY + 15.f : rng.uni(130.f, 330.f), ph, rng.range(0, 3) == 0);
            if (mode == Mode::Wave) p.lastDy = rng.range(0, 1) ? kSpeed1x * p.waveSlope : -kSpeed1x * p.waveSlope;
            if (mode == Mode::Ship) p.lastDy = rng.uni(-1.f, 1.f);
            if (mode == Mode::Cube) p.grounded = true;
            std::vector<Obstacle> level;
            int n = rng.range(3, 25);
            for (int k = 0; k < n; ++k) {
                float x = rng.uni(10.f, 140.f), y = rng.uni(kFloorY, 360.f);
                switch (rng.range(0, 3)) {
                    case 0: level.push_back(spike(x, y)); break;
                    case 1: level.push_back(spikeDown(x, y)); break;
                    case 2: level.push_back(saw(x, y, rng.uni(4.f, 20.f))); break;
                    default: level.push_back(block(x, y, rng.uni(5.f, 40.f), rng.uni(5.f, 40.f))); break;
                }
            }
            bool push = mode == Mode::Cube ? true : rng.range(0, 1) == 1;
            int maxDelay = rng.range(0, 8), gap = rng.range(0, 2) == 0 ? rng.range(1, 12) : 0;
            int la = mode == Mode::Cube ? 120 : rng.range(20, 60);
            std::vector<Player> ps { p };
            ++cases;
            auto c = choose(ps, level, push, maxDelay, gap, la);
            int ref = referenceDelay(ps, level, push, maxDelay, gap, la);
            if (c.delay != ref) {
                if (!mismatches++) firstMismatch = "got " + std::to_string(c.delay) + " want " + std::to_string(ref);
            }
            if (c.delay > 0 && afterAt(ps, level, push, c.delay, gap, la) < 0) ++waitCrash;
            auto h = holdLoop(ps, level, push, maxDelay, gap, la);
            if (h.first.delay > 0) {
                ++holds;
                if (h.fireAt < 0 || h.fireAt > maxDelay) ++late;
                if (h.crashedWaiting) ++crashedHolding;
                // Each re-evaluation's target (now + its delay) never moves later
                // than the first plan.
                for (size_t k = 1; k < h.delays.size(); ++k) {
                    if (int(k) + h.delays[k] > h.first.delay) { ++nonMonotone; break; }
                }
                int planned = afterAt(ps, level, push, h.first.delay, gap, la);
                int got = h.fireAt >= 0 ? afterAt(ps, level, push, h.fireAt, gap, la) : -1;
                if (got < planned) {
                    if (!worse++) firstWorse = "mode " + std::to_string(int(mode)) + " push " + std::to_string(push)
                        + " gap " + std::to_string(gap) + ": planned " + std::to_string(h.first.delay) + " ("
                        + std::to_string(planned) + "), fired " + std::to_string(h.fireAt) + " (" + std::to_string(got) + ")";
                }
            }
        }
        std::printf("random: %d cases, %d holds, %d fired worse than planned\n", cases, holds, worse);
        CHECK(mismatches == 0, "%d/%d differ from the reference rule; first: %s", mismatches, cases, firstMismatch.c_str());
        CHECK(waitCrash == 0, "%d chose a delay where waiting crashes", waitCrash);
        CHECK(late == 0, "%d holds fired after maxDelay or never", late);
        CHECK(crashedHolding == 0, "%d holds crashed while holding", crashedHolding);
        CHECK(nonMonotone == 0, "%d holds moved their target later than planned (oscillation)", nonMonotone);
        CHECK(worse == 0, "%d/%d holds fired at a worse moment than first planned; first: %s", worse, holds, firstWorse.c_str());
    }

    // ================= dual players =================
    {
        // A: wave far up in open air (safe any time). B: mirrored wave (upside
        // down, going down in its frame) with the hazard that needs a delay.
        auto a = fallingWave(&wave, 520.f);
        auto b = fallingWave(&wave, 300.f);
        std::vector<Obstacle> level { hazardRect(8.f, 310.f, 1.f, 80.f) };
        auto cA = choose({ a }, level, true, 8, 0, L);
        auto cB = choose({ b }, level, true, 8, 0, L);
        auto cAB = choose({ a, b }, level, true, 8, 0, L);
        auto cBA = choose({ b, a }, level, true, 8, 0, L);
        CHECK(cA.delay == 0 && cB.delay > 0, "setup: A %d B %d", cA.delay, cB.delay);
        CHECK(cAB.delay == cB.delay && cBA.delay == cB.delay, "dual follows the endangered player: AB %d BA %d B %d",
            cAB.delay, cBA.delay, cB.delay);
        CHECK(cAB.survivedNow == cB.survivedNow, "dual survivedNow %d vs %d", cAB.survivedNow, cB.survivedNow);
        // A must also survive the waiting: put a hazard on A's waiting path
        // after 2 steps. Then no delay past 2 may be picked.
        std::vector<Obstacle> level2 = level;
        level2.push_back(hazardRect(a.center.x + 2.f * kSpeed1x + 5.f, a.center.y - 40.f, 4.f, 30.f));
        int aWait = simulate(a, level2, false, kNever, -1, 60).survived;
        auto cAB2 = choose({ a, b }, level2, true, 8, 0, L);
        CHECK(aWait < cB.delay && cAB2.delay <= aWait, "A's waiting crash at %d, dual delay %d", aWait, cAB2.delay);
        // Upside-down partner (dual portals put P2 upside down): same input
        // makes it go the other way. A safe one must not change the result.
        auto d = fallingWave(&wave, 500.f);
        d.upsideDown = true;
        d.lastDy = kSpeed1x;
        auto cBD = choose({ b, d }, level, true, 8, 0, L);
        CHECK(cBD.delay == cB.delay, "upside-down partner: %d vs %d", cBD.delay, cB.delay);
        // Dual ship: one safe, one needs the wait.
        auto s1 = player(Mode::Ship, 0.f, 200.f, &ship);
        s1.lastDy = -0.6f;
        auto s2 = player(Mode::Ship, 0.f, 400.f, &ship);
        std::vector<Obstacle> sl { hazardRect(30.f, 214.f, 40.f, 60.f), block(-30.f, kFloorY, 900.f, 1.f) };
        auto cs1 = choose({ s1 }, sl, true, 8, 0, L);
        auto cs12 = choose({ s2, s1 }, sl, true, 8, 0, L);
        CHECK(cs12.delay == cs1.delay, "dual ship %d vs %d", cs12.delay, cs1.delay);
        // Both in danger in incompatible ways: B needs a wait, C dies if it
        // waits at all past 1 step. Never a delay past C's waiting crash.
        auto cc = fallingWave(&wave, 600.f);
        std::vector<Obstacle> level3 = level;
        level3.push_back(hazardRect(cc.center.x + 1.f, cc.center.y - 10.f, 4.f, 6.f));
        int cWait = simulate(cc, level3, false, kNever, -1, 60).survived;
        auto cBC = choose({ b, cc }, level3, true, 8, 0, L);
        CHECK(cBC.delay <= cWait, "C waiting crash %d, delay %d", cWait, cBC.delay);
    }

    // ================= robustness =================
    {
        std::vector<Obstacle> level { hazardRect(8.f, 310.f, 1.f, 80.f), block(60.f, 200.f), spike(90.f, kFloorY),
            saw(120.f, 250.f, 15.f), slope(150.f, kFloorY, 60.f, 30.f, true, true) };
        float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();

        // dx = 0: the wave does not move at all; nothing changes with delay.
        {
            auto p = fallingWave(&wave, 300.f);
            p.dx = 0.f;
            auto c = choose({ p }, level, true, 8, 0, L);
            CHECK(c.delay == 0 && finiteChoice(c, 8, L), "dx 0 wave: delay %d now %d", c.delay, c.survivedNow);
            auto s = player(Mode::Ship, 0.f, 300.f, &ship);
            s.dx = 0.f;
            auto cs = choose({ s }, level, true, 8, 0, L);
            CHECK(finiteChoice(cs, 8, L), "dx 0 ship");
        }
        // Negative dx: the mirrored level gives the same decision.
        {
            std::vector<Obstacle> mirrored;
            for (auto o : std::vector<Obstacle> { hazardRect(8.f, 310.f, 1.f, 80.f), spike(90.f, kFloorY), saw(120.f, 250.f, 15.f) }) {
                o.rect.origin.x = -o.rect.origin.x - o.rect.size.width;
                o.center.x = -o.center.x;
                mirrored.push_back(o);
            }
            std::vector<Obstacle> orig { hazardRect(8.f, 310.f, 1.f, 80.f), spike(90.f, kFloorY), saw(120.f, 250.f, 15.f) };
            auto p = fallingWave(&wave, 300.f);
            auto q = p;
            q.dx = -p.dx;
            auto c1 = choose({ p }, orig, true, 8, 0, L);
            auto c2 = choose({ q }, mirrored, true, 8, 0, L);
            CHECK(c1.delay == c2.delay && c1.survivedNow == c2.survivedNow && c1.delay > 0,
                "mirror: %d/%d vs %d/%d", c1.delay, c1.survivedNow, c2.delay, c2.survivedNow);
            // Mirrored slope: a floor slope rising to the left, wave going
            // left along it.
            auto sl = slope(-150.f, kFloorY, 60.f, 60.f, false, true);
            auto r1 = simulate(q, { sl }, false, kNever, -1, 200);
            auto r2 = simulate(p, { slope(90.f, kFloorY, 60.f, 60.f, true, true) }, false, kNever, -1, 200);
            CHECK(r1.survived == r2.survived, "mirrored slope: %d vs %d", r1.survived, r2.survived);
        }
        // Zero-size hitbox.
        {
            auto p = fallingWave(&wave, 300.f);
            p.halfW = p.halfH = 0.f;
            auto c = choose({ p }, level, true, 8, 0, L);
            CHECK(finiteChoice(c, 8, L), "zero hitbox");
            auto r = simulate(p, { block(5.f, 290.f, 10.f, 30.f) }, false, kNever, -1, 20);
            CHECK(r.survived < 20, "zero hitbox still hits a block: %d", r.survived);
        }
        // Huge speeds.
        for (float dx : { 1e6f, 1e30f, 3e38f, -3e38f }) {
            auto p = fallingWave(&wave, 300.f);
            p.dx = dx;
            auto c = choose({ p }, level, true, 8, 0, L);
            CHECK(finiteChoice(c, 8, L), "dx %g", dx);
            auto s = player(Mode::Ship, 0.f, 300.f, &ship);
            s.lastDy = dx;
            auto cs = choose({ s }, level, true, 8, 4, L);
            CHECK(finiteChoice(cs, 8, L), "ship lastDy %g", dx);
        }
        // NaN / inf in physics or position: no crash, sane output.
        {
            int bad = 0;
            for (float v : { nan, inf, -inf }) {
                Physics ph = ship;
                ph.accelHeld.value = v;
                auto s = player(Mode::Ship, 0.f, 300.f, &ph);
                bad += !finiteChoice(choose({ s }, level, true, 8, 0, L), 8, L);
                bad += !finiteChoice(choose({ s }, level, false, 8, 3, L), 8, L);
                ph = ship;
                ph.accelFree.value = v;
                ph.maxUp = v; ph.maxDown = v;
                bad += !finiteChoice(choose({ s }, level, true, 8, 0, L), 8, L);
                auto p = fallingWave(&wave, 300.f);
                p.center.x = v;
                bad += !finiteChoice(choose({ p }, level, true, 8, 0, L), 8, L);
                p = fallingWave(&wave, 300.f);
                p.center.y = v;
                bad += !finiteChoice(choose({ p }, level, true, 8, 0, L), 8, L);
                p = fallingWave(&wave, 300.f);
                p.dx = v; p.lastDy = v; p.waveSlope = v; p.halfW = v;
                bad += !finiteChoice(choose({ p }, level, true, 8, 0, L), 8, L);
                Physics pc = cube;
                pc.impulse.value = v;
                auto cb = player(Mode::Cube, 0.f, kFloorY + 15.f, &pc);
                cb.grounded = true;
                bad += !finiteChoice(choose({ cb }, level, true, 8, 0, 120), 8, 120);
                // NaN in an obstacle.
                auto lv = level;
                lv.push_back(hazardRect(v, 300.f, 10.f, 10.f));
                lv.push_back(saw(v, v, 10.f));
                Obstacle so = slope(40.f, kFloorY, 0.f, 30.f, true, true); // zero-width slope
                so.y0 = v;
                lv.push_back(so);
                bad += !finiteChoice(choose({ fallingWave(&wave, 300.f) }, lv, true, 8, 0, L), 8, L);
            }
            CHECK(bad == 0, "%d NaN/inf choices out of range", bad);
            // A NaN position must not make the mod hold the click.
            auto p = fallingWave(&wave, 300.f);
            p.center.y = nan;
            auto c = choose({ p }, level, true, 8, 0, L);
            CHECK(c.delay == 0, "NaN position held the click %d", c.delay);
        }
        // Empty obstacle list and empty player list.
        {
            auto c = choose({ fallingWave(&wave, 300.f) }, {}, false, 8, 0, L);
            CHECK(c.delay == 0 && c.survivedNow == L, "no obstacles: %d %d", c.delay, c.survivedNow);
            auto c2 = choose({}, level, true, 8, 0, L);
            CHECK(c2.delay == 0 && c2.survivedNow == L, "no players: %d %d", c2.delay, c2.survivedNow);
        }
    }

    // ================= performance =================
    {
        Lcg rng(7);
        std::vector<Obstacle> many;
        // ~2000 objects over x in [-50, 400] (a path of 128 steps at 1x
        // covers ~170 units of it).
        for (int k = 0; k < 2000; ++k) {
            float x = rng.uni(-50.f, 400.f), y = rng.uni(kFloorY, 600.f);
            switch (k % 4) {
                case 0: many.push_back(spike(x, y)); break;
                case 1: many.push_back(saw(x, y, 6.f)); break;
                case 2: many.push_back(block(x, y, 4.f, 4.f)); break;
                default: many.push_back(slope(x, y, 30.f, 30.f, k % 8 < 4, true)); break;
            }
        }
        // Worst case for choose(): open corridor so every delay simulates the
        // full lookahead + delay (and then breaks early on full survival,
        // so measure a case where nothing survives: a far wall).
        // Over the objects in x but above them in y, so every path is long
        // and every object is looked at every step; a wall at ~100 steps
        // so no delay survives the whole lookahead (no early break).
        auto p = player(Mode::Ship, 0.f, 1000.f, &ship);
        std::vector<Obstacle> farWall = many;
        farWall.push_back(block(kSpeed1x * 100.f, 0.f, 10.f, 3000.f));
        auto t0 = std::chrono::steady_clock::now();
        int reps = 20;
        int sink = 0;
        for (int r = 0; r < reps; ++r) sink += choose({ p }, farWall, true, 8, 0, 120).survivedNow;
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / reps;
        // The live overlay: 2 simulations of 120 steps per player per step.
        auto t2 = std::chrono::steady_clock::now();
        for (int r = 0; r < reps; ++r) {
            sink += simulate(p, many, false, 121, -1, 120).survived;
            sink += simulate(p, many, false, 0, -1, 120).survived;
        }
        auto t3 = std::chrono::steady_clock::now();
        double liveMs = std::chrono::duration<double, std::milli>(t3 - t2).count() / reps;
        std::printf("perf (this build): choose() 2001 obstacles, lookahead 120, maxDelay 8: %.3f ms/call; "
                    "live overlay 2x120 steps: %.3f ms/step = %.0f ms per second at 240 steps/s (sink %d)\n",
            ms, liveMs, liveMs * 240.0, sink);
        CHECK(ms < 50.0, "choose() too slow: %.2f ms", ms);
    }

    // ================= Rect, surfaceAt, Estimate =================
    {
        Rect a(0.f, 0.f, 10.f, 10.f);
        CHECK(a.intersectsRect(Rect(10.f, 0.f, 5.f, 5.f)), "touching right edge counts");
        CHECK(a.intersectsRect(Rect(-5.f, -5.f, 5.f, 5.f)), "touching corner counts");
        CHECK(a.intersectsRect(Rect(0.f, 10.f, 10.f, 1.f)), "touching top edge counts");
        CHECK(!a.intersectsRect(Rect(10.01f, 0.f, 5.f, 5.f)), "gap right");
        CHECK(!a.intersectsRect(Rect(0.f, -1.01f, 5.f, 1.f)), "gap below");
        CHECK(a.intersectsRect(Rect(2.f, 2.f, 1.f, 1.f)) && Rect(2.f, 2.f, 1.f, 1.f).intersectsRect(a), "contained, symmetric");
        CHECK(a.intersectsRect(Rect(5.f, 5.f, 0.f, 0.f)), "zero-size point inside");
        CHECK(a.intersectsRect(Rect(10.f, 10.f, 0.f, 0.f)), "zero-size point on corner");
        CHECK(!a.intersectsRect(Rect(11.f, 5.f, 0.f, 0.f)), "zero-size point outside");
        CHECK(a.getMidX() == 5.f && a.getMidY() == 5.f && a.getMaxX() == 10.f, "mids");
    }
    {
        auto s = slope(100.f, 0.f, 50.f, 30.f, true, true);
        CHECK(s.surfaceAt(50.f) == 0.f && s.surfaceAt(100.f) == 0.f, "left clamp");
        CHECK(s.surfaceAt(150.f) == 30.f && s.surfaceAt(1e9f) == 30.f, "right clamp");
        CHECK(std::abs(s.surfaceAt(125.f) - 15.f) < 1e-4f, "middle %.3f", s.surfaceAt(125.f));
        auto f = slope(100.f, 0.f, 50.f, 30.f, false, true);
        CHECK(f.surfaceAt(0.f) == 30.f && f.surfaceAt(200.f) == 0.f, "falling clamps");
        auto z = s;
        z.rect.size.width = 0.f;
        CHECK(z.surfaceAt(100.f) == z.y0 && z.surfaceAt(-1e9f) == z.y0, "zero width -> y0");
        z.rect.size.width = -5.f;
        CHECK(z.surfaceAt(120.f) == z.y0, "negative width -> y0");
    }
    {
        Estimate e;
        e.add(1.f);
        CHECK(e.value == 1.f && e.count == 1, "first sets");
        e.add(2.f);
        CHECK(std::abs(e.value - 1.2f) < 1e-5f && e.count == 2, "ema 0.2: %.4f", e.value);
        // Before 8 measurements, outliers are taken.
        Estimate f;
        for (int i = 0; i < 7; ++i) f.add(1.f);
        f.add(10.f);
        CHECK(f.count == 8 && f.value > 2.f, "outlier taken before 8: %.3f", f.value);
        // From 8 on, outliers are ignored.
        Estimate g;
        for (int i = 0; i < 8; ++i) g.add(1.f);
        g.add(1.6f);
        CHECK(g.count == 8 && g.value == 1.f && g.rejectedInRow == 1, "outlier rejected: %.3f %d", g.value, g.count);
        g.add(1.5f); // exactly 0.5 * |value| away: not rejected (strict >)
        CHECK(g.count == 9 && g.rejectedInRow == 0, "edge 0.5 accepted: count %d", g.count);
        // Small values use the 0.05 floor.
        Estimate h;
        for (int i = 0; i < 8; ++i) h.add(0.01f);
        h.add(0.05f);
        CHECK(h.count == 9, "within 0.05 floor accepted");
        h.add(0.07f);
        CHECK(h.count == 9 && h.rejectedInRow == 1, "beyond 0.05 floor rejected");
        // 5 rejections, then a good one: counter resets.
        Estimate k;
        for (int i = 0; i < 8; ++i) k.add(-0.025f);
        for (int i = 0; i < 5; ++i) k.add(-1.f);
        CHECK(k.rejectedInRow == 5 && k.value == -0.025f, "5 rejected");
        k.add(-0.026f);
        CHECK(k.rejectedInRow == 0 && k.count == 9, "good one resets");
        for (int i = 0; i < 5; ++i) k.add(-1.f);
        CHECK(std::abs(k.value + 0.0252f) < 1e-4f && k.rejectedInRow == 5, "still old after 5 more: %.4f", k.value);
        // The 6th in a row restarts from it.
        k.add(-1.f);
        CHECK(k.value == -1.f && k.count == 1 && k.rejectedInRow == 0, "restart: value %.3f count %d", k.value, k.count);
        // And then it is not ready (ship needs 5) until it has re-learned.
        Physics ph = ship;
        ph.accelFree = k;
        CHECK(!ph.ready(Mode::Ship), "restarted estimate not ready");
        // NaN / inf measurements must not poison a trusted estimate.
        Estimate n;
        for (int i = 0; i < 20; ++i) n.add(-0.025f);
        n.add(std::numeric_limits<float>::quiet_NaN());
        n.add(-0.025f);
        CHECK(std::isfinite(n.value), "NaN poisons the estimate: value %f count %d", n.value, n.count);
        Estimate m;
        m.add(std::numeric_limits<float>::infinity());
        m.add(1.f);
        CHECK(std::isfinite(m.value), "inf poisons a new estimate: value %f", m.value);
    }

    return finish("test_choose");
}
