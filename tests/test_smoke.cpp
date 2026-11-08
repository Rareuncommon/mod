// Basic sanity checks that the harness and prediction work at all.
#include "harness.hpp"

using namespace test;

int main() {
    auto wave = physicsFor(Mode::Wave);
    // Open air, wave going down then turning up: no crash.
    {
        auto p = player(Mode::Wave, 0.f, 300.f, &wave);
        p.lastDy = -kSpeed1x;
        auto r = run(p, {}, false, 0, -1, 48);
        CHECK(r.result.survived == 48, "%s", describe(r).c_str());
    }
    // A wall straight ahead kills.
    {
        auto p = player(Mode::Wave, 0.f, 300.f, &wave);
        p.lastDy = kSpeed1x;
        auto r = run(p, { block(30.f, 200.f, 30.f, 300.f) }, true, 1000, -1, 48);
        CHECK(r.result.survived < 48, "%s", describe(r).c_str());
    }
    return finish("test_smoke");
}
