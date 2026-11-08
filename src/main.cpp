// Input Buffer
//
// Two modes, chosen by the "mode" setting:
//
//   Buffer             Holds a PlayerButton press for up to `buffer-frames`
//                      physics steps while the player cannot act on it yet,
//                      then fires it on the first step the player can.
//   Snap to best run   Only touches inputs in the short window before a
//                      reference input. A press or release that comes up to
//                      `snap-window` steps before a reference input of the
//                      same kind (button, press/release, player) is held and
//                      fired on the reference step, which puts the player
//                      back on the recorded path. Any other input, early by
//                      more than the window, late, or with no reference input
//                      ahead, reaches the game unchanged. Nothing is fired
//                      that the player did not press.
//
// Best run (the reference): every attempt's inputs, as the game received
// them, are recorded automatically. When an attempt gets further than the
// saved best run for this level and start position (completions beat any
// death), it replaces it. A died attempt drops its last kDeathTrimSteps steps
// of inputs, since those likely caused the death.
//
// The best run also stores both players' positions at every step. Snap only
// moves an input while the player is within `path-tolerance` units of where
// the best run was on that step. Off that path (easy to do in ship or wave,
// where one off click changes the height), the best run's timing no longer
// fits and inputs pass through untouched.
//
// Buffer and Snap only act in practice mode or when the level was started
// from a start position (GJBaseGameLayer::m_isPracticeMode, m_isTestMode,
// m_startPosObject); in a normal run inputs pass through untouched. Recording
// the best run works everywhere since it does not change any input. A label
// in the corner shows whenever Buffer or Snap is active.
//
// Hotkeys (Geode keybind settings, remappable): toggle the mod, and raise or
// lower the help steps (buffer-frames in Buffer mode, snap-window in Snap
// mode). They only respond inside a practice or start position run. The
// pause menu gets a button that opens the mod's settings.
//
// Click info: with `show-click-info` on, the label's second line says what
// the mod did with the last press and why; every decision is also logged.
//
// Bindings: geode-sdk/bindings, bindings/2.2081/GeometryDash.bro (Geode SDK 5.11.2).
// "Inline" = the function body is in bindings/2.2081/inline/*.cpp.
//
// Game mode flags (PlayerObject members). Cube has no flag of its own.
//   cube    no flag set                                     isInNormalMode() (inline)
//   ship    m_isShip
//   ball    m_isBall
//   ufo     m_isBird
//   wave    m_isDart
//   robot   m_isRobot
//   spider  m_isSpider
//   swing   m_isSwing
//   isFlying() (inline) == m_isShip || m_isBird || m_isDart || m_isSwing
//
// "Can act" state used by Buffer mode for the discrete modes:
//   m_isOnGround      inline code clears it in pushDown() and spawnFromPlayer(),
//                     and hitGroundNoJump() saves/restores it around hitGround().
//                     In play it stays set in the air, so the mod only counts
//                     it while the player also did not move vertically last
//                     step (or is on a slope).
//   m_touchingRings   CCArray*; addToTouchedRings() (inline) adds the ring being
//                     touched. When the game empties it is not visible in the
//                     bindings.
//   m_isDead          PlayLayer::isGameplayActive() (inline) reads it.
//
// Buffer mode (wave: see "Wave lookahead" below):
// | mode   | input type | buffered when                 | fires when            |
// |--------|------------|-------------------------------|-----------------------|
// | cube   | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | ball   | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | robot  | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | spider | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | ship   | lookahead  | path ahead crashes            | best step             |
// | ufo    | lookahead  | path ahead crashes            | best step             |
// | wave   | lookahead  | path ahead crashes            | best step             |
// | swing  | lookahead  | path ahead crashes            | best step             |
//
// Continuous modes are the isFlying() group. Their input does not require
// ground contact, so the player can always act on a press and there is no
// game state to wait for. They are handled by the lookahead below (not in
// platformer or sideways). Snap mode covers every mode,
// because it times inputs against the reference instead of the player state.
//
// Lookahead (Buffer mode: wave, ship, UFO, swing, and cube, ball and robot
// on the ground): a click there acts the instant it happens, so there is no
// "can act" moment to wait for, and a click that is early by a frame jumps
// or turns too soon. So instead of waiting for
// one, the mod simulates ahead. On a press or release it predicts, for every
// delay from 0 to `buffer-frames` steps, how many steps the player survives
// after the input fires (up to `wave-lookahead`). The button is assumed to
// stay that way after the input, so in a tight part every choice crashes
// eventually; that crash moves along with the delay and is not a reason to
// wait. The input is only held when some delay survives clearly longer
// (kMinGain steps) than firing now, and it is then fired at the best delay,
// re-checked every step. Otherwise it reaches the game unchanged. Late inputs
// are never moved, and a held input is always fired, never dropped. UFO and
// swing only act on presses, so their releases pass through.
//
// Ship, UFO, swing, cube, ball and robot physics are learned while playing
// (FlightPhysics, per mode and size, kept for the session): every clean
// airborne step measures how the vertical speed changed, split by whether
// the button was held (ship, robot), the speed right after a click (UFO and
// cube/robot jump, ball flip), and the pull toward gravity, plus the fastest
// speeds seen. Until a mode has enough measurements, its inputs pass
// through. Ground modes look at least kGroundLookahead steps ahead so a
// whole jump is seen, and holding jumps again on every landing. Clicks on an
// orb and spider clicks are not simulated.
//
// The prediction is the mod's own model, not the game's physics (src/sim.hpp,
// tested in tests/):
//   - x moves by the last step's measured x movement (speed portals, speed
//     changes and teleports inside the lookahead are not seen).
//   - Wave: y moves by that x movement times the wave slope. The direction
//     before the input is the one the wave moved in last step (from the
//     button and gravity while sliding); the input turns it. The slope is
//     measured from the last clean diagonal steps, or 1 (normal) / 2 (mini)
//     until measured.
//   - Ship, UFO, swing: the vertical speed starts at the last step's and
//     changes by the learned physics; a block, slope or the floor that
//     pushes the player out stops it vertically.
//   - The hitbox is the player's own getObjectRect(), moved along the path.
//   - Hazards (Hazard, AnimatedHazard) kill on overlap; round ones (with an
//     object radius, like saws) use the same circle test as the game's
//     playerIntersectsCircle. Solids (Solid, Breakable) the wave was already
//     level with in x are slid along (top or bottom); after that, any solid
//     the inner box (sim::kInnerScale of the hitbox) overlaps kills, which is
//     a side or head-on hit.
//     The level floor (y = sim::kFloorY) is slid along too.
//   - Slopes (Slope) are the line from slopeYPos() at the left and right of
//     their rect, solid below it (slopeFloorTop()) or above it. The wave
//     slides along one it moves parallel to, or lands on one that is flat or
//     falls away in its direction (like a floor or ceiling), but only when it
//     reached the surface this step (within one step's vertical move); deeper
//     means it ran into the slope's face. Running into a face kills once the
//     inner box is past the line, and a slope with m_slopeIsHazard kills on
//     touch.
//   - Objects are taken where they are now: moving objects, triggers that
//     toggle groups, orbs and portals inside the lookahead are not
//     simulated, and neither is a ceiling.
//   - Objects with no-touch, passable, disabled or group-disabled set are
//     skipped.
// A release that arrives while its press is held keeps its gap to the press
// (the tap length), and the path is simulated with that release.
//
// Live prediction (`live-prediction`): a display only. Every step it draws
// the path if the player clicked (or released) now and the path if they
// don't, with the same simulation. The level's objects are re-scanned every
// kLiveRescanSteps steps, over an area kLiveScanMargin steps bigger than the
// path, instead of every step.
//
// Snap releases: a release that arrives while its press is still held back
// is shifted by the same number of steps as the press (so the hold length is
// kept), then snapped to a reference release if one is within the window.
//
// Left/Right (platformer movement) pass through in Buffer mode and are
// snapped like any other button in Snap mode.
//
// The press logic itself (PlayerObject::pushButton / releaseButton /
// updateJump) has addresses but no bodies in the bindings, so the
// discrete/continuous split above comes from the field and helper names,
// not from the game's own code.
//
// Step counting: one step per GJBaseGameLayer::processCommands call. An input
// that arrives through handleButton between calls is stamped with the next
// step; one that arrives during a call (if the game routes queued input back
// through handleButton) is stamped with the current step. Practice
// checkpoints keep a copy of the step counter (PlayLayer::createCheckpoint /
// loadFromCheckpoint), so a reference can be recorded and used across
// checkpoint respawns.
//
// Reference files: <mod save dir>/references/<level>[-sp<x>].txt, one per
// level and start position (keyed by the start position's x coordinate),
// since step counting starts over at the start position. First line
// "# length <steps> completed <0/1>", then one event per line:
//   <step> <button 1-3> <push 0/1> <player1 0/1>
// and one position line per step from step 0, in order:
//   p <player1 x> <player1 y> <player2 x> <player2 y>

#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>
#include <Geode/ui/GeodeUI.hpp>

#include "sim.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace geode::prelude;

namespace {
    // PlayerButton is Jump = 1, Left = 2, Right = 3 (Geode/Enums.hpp).
    constexpr int kButtonCount = 3;

    enum class Mode { Buffer, Snap };

    // Inputs this close to the death of a died attempt are not saved.
    constexpr int kDeathTrimSteps = 60;

    struct BufferSlot {
        bool pending = false;        // a press is being held back
        bool releasePending = false; // the button was released while held back
        bool releaseNextStep = false;// replay the release on the next step
        int framesLeft = 0;
        int waited = 0;              // steps the press has been held back
    };

    struct InputEvent {
        int step = 0;
        int button = 0;
        bool push = false;
        bool isPlayer1 = true;
    };

    struct RefEvent {
        InputEvent input;
        bool matched = false;   // already used to snap a player input
    };

    // Lookahead (see the header comment). The prediction itself is in
    // sim.hpp; these are the game-side pieces.
    constexpr float kWaveMargin = 60.f;      // extra area searched for objects

    using FlyMode = sim::Mode;
    using sim::isFlyingMode;
    using sim::isGroundMode;
    using FlightPhysics = sim::Physics;
    using WaveObstacle = sim::Obstacle;

    sim::Vec toSim(CCPoint p) { return { p.x, p.y }; }
    sim::Rect toSim(CCRect const& r) { return sim::Rect(r.origin.x, r.origin.y, r.size.width, r.size.height); }
    CCPoint toCC(sim::Vec v) { return ccp(v.x, v.y); }

    struct WaveTrack {
        bool hasPosition = false;
        bool hasDelta = false;
        CCPoint last;            // position at the start of the last step
        CCPoint delta;           // movement during the last step
        float slope = 0.f;       // measured |dy| / |dx|, 0 = not measured
        float size = 0.f;        // m_vehicleSize the slope was measured at
        int mode = -1;           // FlyMode during the last step
        float stepSize = 0.f;    // m_vehicleSize during the last step
        bool upsideDown = false; // gravity during the last step
        bool onGround = false;   // touching ground at the start of the last step
    };

    struct WavePending {
        bool active = false;
        bool push = false;       // the held input is a press (else a release)
        int arrivedStep = 0;     // step the input arrived on
        int waited = 0;          // steps it has been held
        bool hasFollow = false;  // the opposite input arrived while held
        int followGap = 0;       // steps between the two inputs
    };

    struct WaveFollow {
        bool active = false;
        bool push = false;
        int step = 0;            // step to fire on
    };

    // The mode the lookahead simulates for a player.

    FlyMode flyMode(PlayerObject* player) {
        if (!player || player->m_isDead || player->m_isPlatformer || player->m_isSideways) return FlyMode::None;
        if (player->m_isDart) return FlyMode::Wave;
        if (player->m_isShip) return FlyMode::Ship;
        if (player->m_isBird) return FlyMode::Ufo;
        if (player->m_isSwing) return FlyMode::Swing;
        if (player->m_isBall) return FlyMode::Ball;
        if (player->m_isRobot) return FlyMode::Robot;
        if (player->m_isSpider) return FlyMode::None; // teleports, not simulated
        return FlyMode::Cube;
    }

    // A ground-mode jump lasts longer than the lookahead setting usually
    // covers, so they look at least this far ahead to see where it lands.
    constexpr int kGroundLookahead = 120;

    // [mode: ship, UFO, swing, cube, ball, robot][mini]. Kept for the whole
    // session, since the physics are the same in every level.
    FlightPhysics g_physics[6][2];

    FlightPhysics* physicsFor(PlayerObject* player) {
        int index;
        switch (flyMode(player)) {
            case FlyMode::Ship: index = 0; break;
            case FlyMode::Ufo: index = 1; break;
            case FlyMode::Swing: index = 2; break;
            case FlyMode::Cube: index = 3; break;
            case FlyMode::Ball: index = 4; break;
            case FlyMode::Robot: index = 5; break;
            default: return nullptr;
        }
        return &g_physics[index][player->m_vehicleSize < 1.f ? 1 : 0];
    }

    // The learned physics are saved, so each mode is only learned once.
    // Format: "physics1" then, per mode and size, value and count of
    // accelHeld, accelFree and impulse, then maxUp and maxDown.
    void savePhysics() {
        std::ostringstream out;
        out << "physics1";
        for (auto& mode : g_physics) {
            for (auto& p : mode) {
                for (auto* e : { &p.accelHeld, &p.accelFree, &p.impulse }) out << ' ' << e->value << ' ' << e->count;
                out << ' ' << p.maxUp << ' ' << p.maxDown;
            }
        }
        Mod::get()->setSavedValue<std::string>("physics", out.str());
    }

    void loadPhysics() {
        auto text = Mod::get()->getSavedValue<std::string>("physics", "");
        std::istringstream in(text);
        std::string tag;
        if (!(in >> tag) || tag != "physics1") return;
        FlightPhysics loaded[6][2];
        for (auto& mode : loaded) {
            for (auto& p : mode) {
                for (auto* e : { &p.accelHeld, &p.accelFree, &p.impulse }) {
                    if (!(in >> e->value >> e->count)) return;
                    // Keep adapting: a saved estimate counts as a few measurements.
                    e->count = std::min(e->count, 20);
                }
                if (!(in >> p.maxUp >> p.maxDown)) return;
            }
        }
        std::copy(&loaded[0][0], &loaded[0][0] + 12, &g_physics[0][0]);
    }

    struct SnapHold {
        bool pressPending = false; // a snapped press has not fired yet
        int pressStep = 0;         // step the snapped press fires on
        int shift = 0;             // steps the press was moved by
    };

    Mode currentMode() {
        auto mode = Mod::get()->getSettingValue<std::string>("mode");
        if (mode == "Snap to best run") return Mode::Snap;
        return Mode::Buffer;
    }

    bool modEnabled() {
        return Mod::get()->getSettingValue<bool>("enabled");
    }

    bool showWavePrediction() {
        return Mod::get()->getSettingValue<bool>("show-wave-prediction");
    }

    bool showLivePrediction() {
        return Mod::get()->getSettingValue<bool>("live-prediction");
    }

    // Live prediction re-scans the level's objects this often (in steps),
    // not every step, since a scan goes through every object in the level.
    constexpr int kLiveRescanSteps = 15;
    // Extra steps of path the scanned area covers, so it stays valid until
    // the next scan.
    constexpr int kLiveScanMargin = 30;

    int waveLookahead() {
        return static_cast<int>(Mod::get()->getSettingValue<int64_t>("wave-lookahead"));
    }

    int bufferFrames() {
        return static_cast<int>(Mod::get()->getSettingValue<int64_t>("buffer-frames"));
    }

    bool showClickInfo() {
        return Mod::get()->getSettingValue<bool>("show-click-info");
    }

    char const* gamemodeName(PlayerObject* player) {
        if (!player) return "?";
        if (player->m_isShip) return "ship";
        if (player->m_isBird) return "UFO";
        if (player->m_isDart) return "wave";
        if (player->m_isSwing) return "swing";
        if (player->m_isBall) return "ball";
        if (player->m_isRobot) return "robot";
        if (player->m_isSpider) return "spider";
        return "cube";
    }

    float pathTolerance() {
        return static_cast<float>(Mod::get()->getSettingValue<int64_t>("path-tolerance"));
    }

    int snapWindow() {
        return static_cast<int>(Mod::get()->getSettingValue<int64_t>("snap-window"));
    }

    char const* buttonName(int button, bool push) {
        switch (button) {
            case 2: return push ? "left press" : "left release";
            case 3: return push ? "right press" : "right release";
            default: return push ? "press" : "release";
        }
    }

    bool isContinuousMode(PlayerObject* player) {
        return player->isFlying();
    }

    bool touchingRing(PlayerObject* player) {
        return player->m_touchingRings && player->m_touchingRings->count() > 0;
    }

    std::filesystem::path referencePath(GJGameLevel* level, StartPosObject* startPos) {
        int id = level->m_levelID.value();
        auto name = id > 0 ? fmt::format("{}", id) : fmt::format("local-{}", level->m_M_ID);
        if (startPos) name += fmt::format("-sp{}", static_cast<int>(std::round(startPos->getPositionX())));
        return Mod::get()->getSaveDir() / "references" / (name + ".txt");
    }

    struct PathPoint {
        float x1 = 0.f, y1 = 0.f, x2 = 0.f, y2 = 0.f;
    };

    struct Reference {
        std::vector<RefEvent> events;
        std::vector<PathPoint> path; // index = step
        int length = 0;          // steps the run lasted
        bool completed = false;  // the run reached the end of the level
    };

    // How good a run is: a completion beats any death, then further is better.
    bool isBetterRun(int length, bool completed, Reference const& than) {
        if (completed != than.completed) return completed;
        return length > than.length;
    }

    Reference loadReference(std::filesystem::path const& path) {
        Reference ref;
        std::ifstream file(path);
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream in(line);
            if (line.starts_with("p ")) {
                std::string tag;
                PathPoint point;
                if (in >> tag >> point.x1 >> point.y1 >> point.x2 >> point.y2) ref.path.push_back(point);
                continue;
            }
            if (line.starts_with("#")) {
                std::string hash, lengthKey, completedKey;
                int completed = 0;
                in >> hash >> lengthKey >> ref.length >> completedKey >> completed;
                ref.completed = completed != 0;
                continue;
            }
            InputEvent e;
            int push, player1;
            if (!(in >> e.step >> e.button >> push >> player1)) continue;
            if (e.button < 1 || e.button > kButtonCount) continue;
            e.push = push != 0;
            e.isPlayer1 = player1 != 0;
            ref.events.push_back({ e });
        }
        std::stable_sort(ref.events.begin(), ref.events.end(), [](auto const& a, auto const& b) {
            return a.input.step < b.input.step;
        });
        return ref;
    }

    bool saveReference(
        std::filesystem::path const& path, std::vector<InputEvent> const& events,
        std::vector<PathPoint> const& points, int length, bool completed
    ) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        std::ofstream file(path, std::ios::trunc);
        if (!file) return false;
        file << "# length " << length << " completed " << (completed ? 1 : 0) << '\n';
        for (auto const& e : events) {
            file << e.step << ' ' << e.button << ' ' << (e.push ? 1 : 0) << ' ' << (e.isPlayer1 ? 1 : 0) << '\n';
        }
        for (auto const& point : points) {
            file << "p " << point.x1 << ' ' << point.y1 << ' ' << point.x2 << ' ' << point.y2 << '\n';
        }
        return static_cast<bool>(file);
    }

    void notify(std::string const& text, NotificationIcon icon) {
        Notification::create(text, icon)->show();
    }
}

class $modify(BufferPlayLayer, PlayLayer) {
    struct Fields {
        // [0] = player 2 input, [1] = player 1 input (indexed by isPlayer1)
        BufferSlot slots[2][kButtonCount];
        bool firing = false;

        // Physics steps processed this attempt.
        int step = 0;
        std::unordered_map<CheckpointObject*, int> checkpointSteps;

        // Inputs the game received this attempt, for the best run.
        std::vector<InputEvent> recorded;
        std::vector<PathPoint> path; // player positions this attempt, index = step
        bool attemptSaved = false;

        // Best run for the current level and start position.
        bool referenceLoaded = false;
        std::filesystem::path referenceFile;
        std::vector<RefEvent> reference;
        std::vector<PathPoint> referencePath;
        int referenceLength = 0;
        bool referenceCompleted = false;
        std::vector<InputEvent> scheduled; // snapped inputs waiting for their step
        SnapHold holds[2][kButtonCount];   // indexed like slots

        bool inStep = false;     // inside GJBaseGameLayer::processCommands

        // Wave lookahead, indexed like slots (by input channel / player).
        WaveTrack waveTracks[2];   // [0] = m_player2, [1] = m_player1
        WavePending wave[2];
        WaveFollow waveFollow[2];
        bool jumpHeld[2] = {};     // jump state the game has seen per input
        bool pressedSince[2] = {}; // a press reached the game since the snapshot
        bool heldDuring[2] = {};   // jump state during the step just simulated
        bool pressedDuring[2] = {};// a press happened in that step
        CCDrawNode* waveDraw = nullptr; // prediction overlay, in level space
        CCDrawNode* liveDraw = nullptr; // live prediction, redrawn every step
        std::vector<WaveObstacle> liveObstacles;
        int liveScanStep = -1000;       // step of the last object scan

        CCLabelBMFont* indicator = nullptr;
        std::string indicatorText;
        std::string lastClick;   // what happened to the last press
    };

    bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
        if (!PlayLayer::init(level, useReplay, dontCreateObjects)) return false;

        auto winSize = CCDirector::get()->getWinSize();
        auto label = CCLabelBMFont::create("", "bigFont.fnt");
        label->setScale(0.35f);
        label->setOpacity(160);
        label->setAnchorPoint({ 1.f, 0.f });
        label->setPosition({ winSize.width - 5.f, 5.f });
        label->setVisible(false);
        label->setID("input-buffer-indicator"_spr);
        this->addChild(label, 1000);
        m_fields->indicator = label;

        if (m_objectLayer) {
            auto draw = CCDrawNode::create();
            draw->setID("wave-prediction"_spr);
            m_objectLayer->addChild(draw, 9999);
            m_fields->waveDraw = draw;

            auto live = CCDrawNode::create();
            live->setID("live-prediction"_spr);
            m_objectLayer->addChild(live, 9998);
            m_fields->liveDraw = live;
        }

        this->updateIndicator();
        return true;
    }

    // ---- shared ----

    // Buffer and Snap only run where a completion cannot count as a normal run.
    bool assistAllowed() {
        return m_isPracticeMode || m_isTestMode || m_startPosObject;
    }

    void updateIndicator() {
        auto fields = m_fields.self();
        if (!fields->indicator) return;

        std::string text;
        if (modEnabled()) switch (currentMode()) {
            case Mode::Buffer:
                if (this->assistAllowed() && bufferFrames() > 0) {
                    text = fmt::format("Input Buffer: {} steps", bufferFrames());
                    if (flyMode(m_player1) != FlyMode::None) {
                        text += fmt::format(", lookahead {}", waveLookahead());
                    }
                }
                break;
            case Mode::Snap:
                if (this->assistAllowed()) {
                    text = fields->reference.empty()
                        ? fmt::format("Input Buffer: snap {} steps (no best run yet)", snapWindow())
                        : fmt::format("Input Buffer: snap {} steps", snapWindow());
                }
                break;
        }
        if (!text.empty() && showClickInfo() && !fields->lastClick.empty()) {
            text += "\n" + fields->lastClick;
        }
        if (text == fields->indicatorText) return;
        fields->indicatorText = text;
        fields->indicator->setString(text.c_str());
        fields->indicator->setVisible(!text.empty());
    }

    void report(std::string text) {
        auto fields = m_fields.self();
        log::info("Step {}: {}", fields->step, text);
        fields->lastClick = std::move(text);
        this->updateIndicator();
    }

    void fire(bool down, int button, bool isPlayer1) {
        auto fields = m_fields.self();
        fields->firing = true;
        this->handleButton(down, button, isPlayer1);
        fields->firing = false;
    }

    // Returns true if the event was consumed by the mod.
    bool onButton(bool down, int button, bool isPlayer1) {
        if (button < 1 || button > kButtonCount) return false;
        if (m_fields->firing) return false;
        if (!modEnabled()) return false;
        if (!m_started || m_hasCompletedLevel) return false;

        switch (currentMode()) {
            case Mode::Snap:
                if (!this->assistAllowed()) return false;
                return this->onSnapButton(down, button, isPlayer1);
            case Mode::Buffer:
                if (!this->assistAllowed()) return false;
                return this->onBufferButton(down, button, isPlayer1);
        }
        return false;
    }

    // Every input that reaches the game, including ones the mod fired.
    void recordInput(bool down, int button, bool isPlayer1) {
        if (button < 1 || button > kButtonCount) return;
        if (!m_started || m_hasCompletedLevel) return;
        m_fields->recorded.push_back({ this->inputStep(), button, down, isPlayer1 });
    }

    // Saves this attempt as the best run if it got further than the saved one.
    void considerSaving(bool completed) {
        auto fields = m_fields.self();
        if (fields->attemptSaved) return;
        fields->attemptSaved = true;

        this->ensureReference();
        int length = fields->step;
        // A saved run without positions (from v1.5.0) can't be snapped to,
        // so any attempt replaces it.
        Reference current;
        if (!fields->referencePath.empty()) {
            current.length = fields->referenceLength;
            current.completed = fields->referenceCompleted;
        }
        if (!isBetterRun(length, completed, current)) return;

        std::vector<InputEvent> events = fields->recorded;
        std::vector<PathPoint> points = fields->path;
        if (!completed) {
            int cutoff = length - kDeathTrimSteps;
            std::erase_if(events, [cutoff](auto const& e) { return e.step > cutoff; });
            if (cutoff < 0) points.clear();
            else if (points.size() > static_cast<size_t>(cutoff) + 1) points.resize(cutoff + 1);
        }
        if (events.empty() && !completed) return;

        if (saveReference(fields->referenceFile, events, points, length, completed)) {
            log::info("Saved best run: {} steps, {} inputs, completed {}", length, events.size(), completed);
            // Loaded again on the next reset, so the current attempt is unaffected.
            fields->referenceLoaded = false;
        }
        else {
            log::warn("Failed to save best run to {}", fields->referenceFile);
        }
    }

    // The step an input arriving now is processed on.
    int inputStep() {
        return m_fields->inStep ? m_fields->step : m_fields->step + 1;
    }

    void setInStep(bool inStep) {
        m_fields->inStep = inStep;
    }

    // Called once per physics step, before the game processes queued input.
    void onStep() {
        auto fields = m_fields.self();
        fields->step++;
        this->recordPathPoint();
        this->updateWaveTracks();
        this->stepBuffer();
        this->stepWave();
        if (modEnabled() && currentMode() == Mode::Snap && this->assistAllowed()) {
            if (!fields->referenceLoaded) this->ensureReference();
            this->stepSnap();
        }
        // After every input the mod fires this step.
        this->snapshotInputs();
        this->updateLivePrediction();
        this->updateIndicator();
    }

    // Live prediction: every step, draws the path if the player clicked (or
    // released) now, green or red if it crashes, and the path if they
    // don't, in white. Only a display; it changes no input.
    void updateLivePrediction() {
        auto fields = m_fields.self();
        auto draw = fields->liveDraw;
        if (!draw) return;
        draw->clear();
        if (!showLivePrediction() || !modEnabled() || currentMode() != Mode::Buffer || !this->assistAllowed()) return;
        if (!m_started || m_hasCompletedLevel) return;

        bool twoPlayer = m_levelSettings && m_levelSettings->m_twoPlayerMode;
        std::vector<bool> channels { true };
        if (twoPlayer) channels.push_back(false);

        int lookahead = waveLookahead();
        bool rescan = fields->step < fields->liveScanStep || fields->step - fields->liveScanStep >= kLiveRescanSteps;

        for (bool isPlayer1 : channels) {
            if (!this->lookaheadApplies(isPlayer1)) continue;
            if (!this->waveTracksReady(isPlayer1) || !this->physicsReady(isPlayer1)) continue;
            auto players = this->inputPlayers(isPlayer1);
            bool ground = std::any_of(players.begin(), players.end(), [](PlayerObject* player) {
                return isGroundMode(flyMode(player));
            });
            int steps = ground ? std::max(lookahead, kGroundLookahead) : lookahead;

            if (rescan) {
                // Both channels' players, so one scan serves both.
                auto all = this->inputPlayers(true);
                if (twoPlayer) {
                    auto p2 = this->inputPlayers(false);
                    all.insert(all.end(), p2.begin(), p2.end());
                }
                fields->liveObstacles = this->gatherObstacles(all, std::max(lookahead, kGroundLookahead) + kLiveScanMargin);
                fields->liveScanStep = fields->step;
                rescan = false;
            }

            bool held = fields->jumpHeld[isPlayer1 ? 1 : 0];
            for (auto player : players) {
                auto mode = flyMode(player);
                // A release only acts in wave, ship and robot.
                bool actionMatters = !held || mode == FlyMode::Wave || mode == FlyMode::Ship || mode == FlyMode::Robot;

                std::vector<CCPoint> path;
                this->simulateWave(player, fields->liveObstacles, held, steps + 1, -1, steps, &path);
                for (size_t i = 1; i < path.size(); ++i) {
                    draw->drawSegment(path[i - 1], path[i], .5f, ccColor4F { 1.f, 1.f, 1.f, .45f });
                }

                if (!actionMatters) continue;
                path.clear();
                auto sim = this->simulateWave(player, fields->liveObstacles, held, 0, -1, steps, &path);
                ccColor4F color = sim.survived >= steps
                    ? ccColor4F { .2f, 1.f, .2f, .8f }
                    : ccColor4F { 1.f, .2f, .2f, .8f };
                for (size_t i = 1; i < path.size(); ++i) draw->drawSegment(path[i - 1], path[i], .6f, color);
                if (!path.empty() && sim.survived < steps) draw->drawDot(path.back(), 2.5f, color);
            }
        }
    }

    PathPoint currentPoint() {
        PathPoint point;
        if (m_player1) {
            point.x1 = m_player1->getPositionX();
            point.y1 = m_player1->getPositionY();
        }
        if (m_player2) {
            point.x2 = m_player2->getPositionX();
            point.y2 = m_player2->getPositionY();
        }
        return point;
    }

    // Position at the start of this step, before it is processed.
    void recordPathPoint() {
        auto fields = m_fields.self();
        auto point = this->currentPoint();
        size_t index = static_cast<size_t>(fields->step);
        if (fields->path.size() > index) fields->path.resize(index);
        // Steps skipped (none expected) are filled with the current point.
        while (fields->path.size() <= index) fields->path.push_back(point);
    }

    void clearBuffer() {
        auto fields = m_fields.self();
        for (auto& player : fields->slots) {
            for (auto& slot : player) slot = {};
        }
        for (auto& w : fields->wave) w = {};
        for (auto& f : fields->waveFollow) f = {};
    }

    // Positions jump on a reset or checkpoint respawn, so the next speed
    // reading has to start over.
    void resetWaveTracks() {
        for (auto& t : m_fields->waveTracks) t = {};
    }

    // Puts every per-attempt state back to fields->step, which is 0 after a
    // full reset or the checkpoint's step after a checkpoint respawn.
    void rewindToStep() {
        auto fields = m_fields.self();
        int step = fields->step;
        std::erase_if(fields->recorded, [step](auto const& e) { return e.step > step; });
        if (fields->path.size() > static_cast<size_t>(step) + 1) fields->path.resize(step + 1);

        fields->scheduled.clear();
        for (auto& player : fields->holds) {
            for (auto& hold : player) hold = {};
        }
        for (auto& ref : fields->reference) {
            ref.matched = ref.input.step <= step;
        }
    }

    void resetLevel() {
        auto fields = m_fields.self();
        this->clearBuffer();
        fields->step = 0;
        PlayLayer::resetLevel();
        // A checkpoint respawn sets fields->step in loadFromCheckpoint.
        this->clearBuffer();
        fields->attemptSaved = false;
        this->resetWaveTracks();
        if (fields->waveDraw) fields->waveDraw->clear();
        if (fields->liveDraw) fields->liveDraw->clear();
        fields->liveScanStep = -1000;
        // The start position may have changed, which selects another best run.
        this->ensureReference();
        this->rewindToStep();
        this->updateIndicator();
    }

    void destroyPlayer(PlayerObject* player, GameObject* object) {
        PlayLayer::destroyPlayer(player, object);
        if (player && player->m_isDead) {
            this->clearBuffer();
            this->considerSaving(false);
        }
    }

    CheckpointObject* createCheckpoint() {
        auto checkpoint = PlayLayer::createCheckpoint();
        if (checkpoint) m_fields->checkpointSteps[checkpoint] = m_fields->step;
        return checkpoint;
    }

    void loadFromCheckpoint(CheckpointObject* checkpoint) {
        PlayLayer::loadFromCheckpoint(checkpoint);
        auto fields = m_fields.self();
        auto it = fields->checkpointSteps.find(checkpoint);
        if (it != fields->checkpointSteps.end()) fields->step = it->second;
        this->clearBuffer();
        this->resetWaveTracks();
    }

    void removeAllCheckpoints() {
        PlayLayer::removeAllCheckpoints();
        m_fields->checkpointSteps.clear();
    }

    void onQuit() {
        savePhysics();
        PlayLayer::onQuit();
    }

    void levelComplete() {
        this->considerSaving(true);
        PlayLayer::levelComplete();
    }

    // ---- Buffer mode ----

    BufferSlot& slot(int button, bool isPlayer1) {
        return m_fields->slots[isPlayer1 ? 1 : 0][button - 1];
    }

    // The players a button event acts on. Without two-player mode a single
    // input drives both players in dual mode.
    bool inputCanAct(bool isPlayer1) {
        bool twoPlayer = m_levelSettings && m_levelSettings->m_twoPlayerMode;
        if (twoPlayer) return this->canAct(isPlayer1 ? m_player1 : m_player2);
        if (this->canAct(m_player1)) return true;
        return m_gameState.m_isDualMode && this->canAct(m_player2);
    }

    // m_isOnGround is set on landing but is not reliably cleared when the
    // player leaves the ground, so it only counts while the player also did
    // not move vertically last step (on a slope it moves, so that counts).
    bool onGround(PlayerObject* player) {
        if (!player->m_isOnGround) return false;
        if (player->m_isOnSlope || player->m_wasOnSlope) return true;
        auto const& t = this->waveTrack(player);
        if (!t.hasDelta) return true;
        return std::abs(t.delta.y) < 0.01f;
    }

    bool canAct(PlayerObject* player) {
        if (!player || player->m_isDead) return false;
        if (isContinuousMode(player)) return true;
        return this->onGround(player) || touchingRing(player);
    }

    // Why the player can act, for the click info.
    char const* canActReason(bool isPlayer1) {
        bool twoPlayer = m_levelSettings && m_levelSettings->m_twoPlayerMode;
        auto player = twoPlayer && !isPlayer1 ? m_player2 : m_player1;
        if (player && touchingRing(player)) return "touching an orb";
        return "on the ground";
    }

    void logGroundState(PlayerObject* player) {
        if (!player) return;
        auto const& t = this->waveTrack(player);
        log::info(
            "Ground: m_isOnGround {} m_isOnGround2 {} m_isOnGround3 {} m_isOnGround4 {} onSlope {}/{} "
            "rings {} last dy {:.3f}",
            player->m_isOnGround, player->m_isOnGround2, player->m_isOnGround3, player->m_isOnGround4,
            player->m_isOnSlope, player->m_wasOnSlope,
            player->m_touchingRings ? player->m_touchingRings->count() : 0, t.hasDelta ? t.delta.y : 0.f
        );
    }

    bool inputIsContinuous(bool isPlayer1) {
        bool twoPlayer = m_levelSettings && m_levelSettings->m_twoPlayerMode;
        if (twoPlayer) {
            auto player = isPlayer1 ? m_player1 : m_player2;
            return player && isContinuousMode(player);
        }
        return m_player1 && isContinuousMode(m_player1);
    }

    bool onBufferButton(bool down, int button, bool isPlayer1) {
        // A press held from another mode is finished by the normal buffer.
        if (button == static_cast<int>(PlayerButton::Jump) && !slot(button, isPlayer1).pending
            && (m_fields->wave[isPlayer1 ? 1 : 0].active || this->lookaheadApplies(isPlayer1))) {
            return this->onWaveButton(down, isPlayer1);
        }
        auto& s = slot(button, isPlayer1);
        if (!down) {
            if (s.pending) {
                s.releasePending = true;
                return true;
            }
            return false;
        }

        int frames = bufferFrames();
        if (frames <= 0) return false;
        if (button != static_cast<int>(PlayerButton::Jump)) return false;
        auto player = isPlayer1 || !m_player2 ? m_player1 : m_player2;
        if (inputIsContinuous(isPlayer1)) {
            this->report(fmt::format("{}: Buffer can't simulate this (platformer or sideways), use Snap", gamemodeName(player)));
            return false;
        }
        this->logGroundState(player);
        if (inputCanAct(isPlayer1)) {
            this->report(fmt::format("{}: could already jump ({}), passed through", gamemodeName(player), this->canActReason(isPlayer1)));
            return false;
        }

        if (s.releaseNextStep) this->fire(false, button, isPlayer1);
        s.pending = true;
        s.releasePending = false;
        s.releaseNextStep = false;
        s.framesLeft = frames;
        s.waited = 0;
        this->report(fmt::format("{}: in the air, holding click (up to {} steps)", gamemodeName(player), frames));
        return true;
    }

    void stepBuffer() {
        for (int p = 0; p < 2; ++p) {
            bool isPlayer1 = p == 1;
            for (int b = 1; b <= kButtonCount; ++b) {
                auto& s = slot(b, isPlayer1);

                if (s.releaseNextStep) {
                    s.releaseNextStep = false;
                    this->fire(false, b, isPlayer1);
                    continue;
                }
                if (!s.pending) continue;
                s.waited++;

                if (inputCanAct(isPlayer1)) {
                    bool released = s.releasePending;
                    int waited = s.waited;
                    s = {};
                    this->fire(true, b, isPlayer1);
                    this->report(fmt::format("landed, click fired {} steps late", waited));
                    // Release one step later so the press is seen for a step.
                    if (released) s.releaseNextStep = true;
                    continue;
                }

                if (--s.framesLeft > 0) continue;

                // Expired. A still-held button is handed to the game so its
                // own hold behaviour applies; a finished tap is dropped.
                bool released = s.releasePending;
                s = {};
                if (!released) this->fire(true, b, isPlayer1);
                this->report(released
                    ? "buffer ran out before landing, tap dropped"
                    : "buffer ran out before landing, held click passed on");
            }
        }
    }

    // ---- Buffer mode: wave lookahead ----

    // Every jump input that reaches the game, so the mod knows whether the
    // game currently sees the button held.
    void noteInput(bool down, int button, bool isPlayer1) {
        if (button != static_cast<int>(PlayerButton::Jump)) return;
        int ch = isPlayer1 ? 1 : 0;
        if (down && !m_fields->jumpHeld[ch]) m_fields->pressedSince[ch] = true;
        m_fields->jumpHeld[ch] = down;
    }

    // The button state the game uses for the step about to run, for
    // learning the physics on the next step.
    void snapshotInputs() {
        auto fields = m_fields.self();
        for (int ch = 0; ch < 2; ++ch) {
            fields->heldDuring[ch] = fields->jumpHeld[ch];
            fields->pressedDuring[ch] = fields->pressedSince[ch];
            fields->pressedSince[ch] = false;
        }
    }

    // The input channel that drives a player.
    int channelOf(PlayerObject* player) {
        bool twoPlayer = m_levelSettings && m_levelSettings->m_twoPlayerMode;
        return twoPlayer && player == m_player2 ? 0 : 1;
    }

    // The players a jump input on this channel moves.
    std::vector<PlayerObject*> inputPlayers(bool isPlayer1) {
        std::vector<PlayerObject*> players;
        bool twoPlayer = m_levelSettings && m_levelSettings->m_twoPlayerMode;
        if (twoPlayer) {
            auto player = isPlayer1 ? m_player1 : m_player2;
            if (player) players.push_back(player);
            return players;
        }
        if (m_player1) players.push_back(m_player1);
        if (m_gameState.m_isDualMode && m_player2) players.push_back(m_player2);
        return players;
    }

    static bool isSimulatable(PlayerObject* player) {
        return isFlyingMode(flyMode(player));
    }

    // Every player this input drives is in wave, ship, UFO or swing.
    bool inputIsWave(bool isPlayer1) {
        auto players = this->inputPlayers(isPlayer1);
        if (players.empty()) return false;
        return std::all_of(players.begin(), players.end(), isSimulatable);
    }

    // Every player this input drives is a cube, ball or robot standing on
    // the ground (not on an orb, which isn't simulated).
    bool inputIsGroundJump(bool isPlayer1) {
        auto players = this->inputPlayers(isPlayer1);
        if (players.empty()) return false;
        return std::all_of(players.begin(), players.end(), [&](PlayerObject* player) {
            return isGroundMode(flyMode(player)) && this->onGround(player) && !touchingRing(player);
        });
    }

    bool lookaheadApplies(bool isPlayer1) {
        return this->inputIsWave(isPlayer1) || this->inputIsGroundJump(isPlayer1);
    }

    WaveTrack& waveTrack(PlayerObject* player) {
        return m_fields->waveTracks[player == m_player2 ? 0 : 1];
    }

    // Measures each player's movement over the step that just ended.
    void updateWaveTracks() {
        for (auto player : { m_player1, m_player2 }) {
            if (!player) continue;
            auto& t = this->waveTrack(player);
            auto pos = player->getPosition();
            // Mini and normal wave have different slopes.
            if (t.size != player->m_vehicleSize) {
                t.slope = 0.f;
                t.size = player->m_vehicleSize;
            }
            if (t.hasPosition) {
                auto delta = pos - t.last;
                // Slope from a clean diagonal step: moving, and going the
                // same way up or down as the step before (not a turn).
                if (t.hasDelta && std::abs(delta.x) > 0.01f && std::abs(delta.y) > 0.01f
                    && (delta.y > 0.f) == (t.delta.y > 0.f)) {
                    float slope = std::abs(delta.y) / std::abs(delta.x);
                    if (slope > 0.3f && slope < 3.f) t.slope = slope;
                }
                this->learnPhysics(player, t, delta);
                t.delta = delta;
                t.hasDelta = true;
            }
            t.last = pos;
            t.hasPosition = true;
            t.mode = static_cast<int>(flyMode(player));
            t.stepSize = player->m_vehicleSize;
            t.upsideDown = player->m_isUpsideDown;
            t.onGround = player->m_isOnGround
                && (player->m_isOnSlope || player->m_wasOnSlope || !t.hasDelta || std::abs(t.delta.y) < 0.01f);
        }
    }

    // Learns ship, UFO and swing physics from the step that just ran.
    void learnPhysics(PlayerObject* player, WaveTrack const& t, CCPoint delta) {
        auto physics = physicsFor(player);
        if (!physics || !t.hasDelta) return;
        auto mode = flyMode(player);
        // Not the step of a mode or size portal: its movement is no
        // measurement of either mode's physics.
        if (t.mode != static_cast<int>(mode) || t.stepSize != player->m_vehicleSize) return;
        if (std::abs(delta.x) < 0.01f || std::abs(delta.y) > 40.f || std::abs(t.delta.y) > 40.f) return;

        float g = player->m_isUpsideDown ? -1.f : 1.f;
        float speed = g * delta.y, before = g * t.delta.y;
        float accel = speed - before;
        int ch = this->channelOf(player);
        bool held = m_fields->heldDuring[ch];
        bool pressed = m_fields->pressedDuring[ch];

        // Ground modes: the step that left the ground on a held button is
        // the jump (a ball flips gravity on it).
        if (isGroundMode(mode) && t.onGround && (held || pressed) && speed > 0.f) {
            bool flipped = t.upsideDown != player->m_isUpsideDown;
            if (mode == FlyMode::Ball ? flipped : !flipped) physics->impulse.add(speed);
            return;
        }

        // Otherwise only clean airborne steps: same gravity as the step
        // before, not on the ground at either end, moving, no teleport.
        if (t.upsideDown != player->m_isUpsideDown) return;
        bool groundNow = player->m_isOnGround && std::abs(delta.y) < 0.01f;
        if (t.onGround || groundNow) return;

        switch (mode) {
            case FlyMode::Ship:
                if (speed == 0.f || before == 0.f) return; // sliding along something
                if (held) physics->accelHeld.add(accel);
                else physics->accelFree.add(accel);
                break;
            case FlyMode::Ufo:
                if (pressed) physics->impulse.add(speed);
                else if (speed != 0.f && before != 0.f) physics->accelFree.add(accel);
                break;
            case FlyMode::Swing:
                if (pressed) return; // gravity flipped on this step
                if (speed == 0.f || before == 0.f) return;
                physics->accelFree.add(accel);
                break;
            case FlyMode::Cube:
            case FlyMode::Ball:
            case FlyMode::Robot:
                if (speed == 0.f || before == 0.f) return;
                // The robot's jump goes higher while held.
                if (mode == FlyMode::Robot && held) physics->accelHeld.add(accel);
                else physics->accelFree.add(accel);
                break;
            default: return;
        }
        if (speed != 0.f) {
            physics->maxUp = std::max(physics->maxUp, speed);
            physics->maxDown = std::max(physics->maxDown, -speed);
        }
    }

    float waveSlope(PlayerObject* player) {
        auto const& t = this->waveTrack(player);
        if (t.slope > 0.f) return t.slope;
        return player->m_vehicleSize < 1.f ? 2.f : 1.f;
    }

    // Solids and hazards around the players that the next `steps` steps
    // could reach.
    std::vector<WaveObstacle> gatherObstacles(std::vector<PlayerObject*> const& players, int steps) {
        std::vector<WaveObstacle> obstacles;
        if (!m_objects || players.empty()) return obstacles;

        float minX = FLT_MAX, maxX = -FLT_MAX, minY = FLT_MAX, maxY = -FLT_MAX;
        for (auto player : players) {
            auto const& t = this->waveTrack(player);
            auto pos = player->getPosition();
            float reachX = std::abs(t.delta.x) * steps;
            float reachY = std::max(reachX * std::max(this->waveSlope(player), 2.5f), 90.f);
            minX = std::min(minX, pos.x - reachX);
            maxX = std::max(maxX, pos.x + reachX);
            minY = std::min(minY, pos.y - reachY);
            maxY = std::max(maxY, pos.y + reachY);
        }
        sim::Rect area(minX - kWaveMargin, minY - kWaveMargin,
            maxX - minX + 2 * kWaveMargin, maxY - minY + 2 * kWaveMargin);

        for (auto object : CCArrayExt<GameObject*>(m_objects)) {
            if (!object) continue;
            auto type = object->m_objectType;
            bool hazard = type == GameObjectType::Hazard || type == GameObjectType::AnimatedHazard;
            bool solid = type == GameObjectType::Solid || type == GameObjectType::Breakable;
            bool slope = type == GameObjectType::Slope;
            if (!hazard && !solid && !slope) continue;
            if (object->m_isNoTouch || object->m_isPassable) continue;
            if (object->m_isDisabled || object->m_isGroupDisabled) continue;

            WaveObstacle o;
            o.hazard = hazard;
            o.rect = toSim(object->getObjectRect());
            if (hazard && object->m_objectRadius > 0.f) {
                o.radius = object->getObjectRadius();
                o.center = toSim(object->getPosition());
                o.rect = sim::Rect(o.center.x - o.radius, o.center.y - o.radius, o.radius * 2, o.radius * 2);
            }
            if (o.rect.size.width <= 0.f || o.rect.size.height <= 0.f) continue;
            if (!o.rect.intersectsRect(area)) continue;
            if (slope) {
                o.slope = true;
                o.floorTop = object->slopeFloorTop();
                o.deadly = object->m_slopeIsHazard;
                o.y0 = static_cast<float>(object->slopeYPos(o.rect.getMinX()));
                o.y1 = static_cast<float>(object->slopeYPos(o.rect.getMaxX()));
            }
            obstacles.push_back(o);
        }
        return obstacles;
    }

    using WaveSim = sim::Result;
    using WaveChoice = sim::Choice;

    bool waveGoingUp(PlayerObject* player, bool held) {
        return sim::waveGoingUp(this->simPlayer(player), held);
    }

    // The prediction's view of a player right now.
    sim::Player simPlayer(PlayerObject* player) {
        auto const& t = this->waveTrack(player);
        sim::Player p;
        p.mode = flyMode(player);
        auto pos = player->getPosition();
        auto box = player->getObjectRect();
        CCPoint offset = ccp(box.getMidX() - pos.x, box.getMidY() - pos.y);
        // A stale rect would put the hitbox somewhere else entirely.
        if (std::abs(offset.x) > 15.f || std::abs(offset.y) > 15.f) offset = CCPointZero;
        p.center = toSim(pos + offset);
        p.halfW = box.size.width / 2;
        p.halfH = box.size.height / 2;
        p.upsideDown = player->m_isUpsideDown;
        p.dx = t.delta.x;
        p.lastDy = t.delta.y;
        p.waveSlope = this->waveSlope(player);
        p.grounded = isGroundMode(p.mode) && this->onGround(player);
        p.physics = physicsFor(player);
        return p;
    }

    WaveSim simulateWave(
        PlayerObject* player, std::vector<WaveObstacle> const& obstacles,
        bool heldBefore, int switchAt, int switchBackAt, int steps,
        std::vector<CCPoint>* path = nullptr
    ) {
        std::vector<sim::Vec> simPath;
        auto result = sim::simulate(
            this->simPlayer(player), obstacles, heldBefore, switchAt, switchBackAt, steps,
            path ? &simPath : nullptr
        );
        if (path) for (auto v : simPath) path->push_back(toCC(v));
        return result;
    }

    WaveChoice chooseWaveDelay(bool isPlayer1, bool push, int maxDelay, int followGap) {
        auto players = this->inputPlayers(isPlayer1);
        int lookahead = waveLookahead();
        bool ground = std::any_of(players.begin(), players.end(), [](PlayerObject* player) {
            return isGroundMode(flyMode(player));
        });
        if (ground) lookahead = std::max(lookahead, kGroundLookahead);
        auto obstacles = this->gatherObstacles(players, lookahead + maxDelay);
        bool held = !push;

        std::vector<sim::Player> simPlayers;
        for (auto player : players) simPlayers.push_back(this->simPlayer(player));
        auto choice = sim::choose(simPlayers, obstacles, push, maxDelay, followGap, lookahead);

        this->logWaveChoice(players, obstacles, held, maxDelay, followGap, choice);
        if (showWavePrediction()) this->drawWaveChoice(players, obstacles, held, followGap, choice);
        return choice;
    }

    void logWaveChoice(
        std::vector<PlayerObject*> const& players, std::vector<WaveObstacle> const& obstacles,
        bool held, int maxDelay, int followGap, WaveChoice const& choice
    ) {
        int hazards = 0;
        for (auto const& o : obstacles) hazards += o.hazard ? 1 : 0;
        for (auto player : players) {
            auto const& t = this->waveTrack(player);
            auto pos = player->getPosition();
            auto box = player->getObjectRect();
            log::info(
                "Wave: pos ({:.1f}, {:.1f}) rect ({:.1f}, {:.1f}, {:.1f}x{:.1f}) delta ({:.2f}, {:.2f}) "
                "slope {:.2f} size {:.2f} upsideDown {} held {} goingUp {}",
                pos.x, pos.y, box.origin.x, box.origin.y, box.size.width, box.size.height,
                t.delta.x, t.delta.y, this->waveSlope(player), player->m_vehicleSize,
                player->m_isUpsideDown, held, this->waveGoingUp(player, held)
            );
        }
        for (auto player : players) {
            if (auto physics = physicsFor(player)) {
                log::info(
                    "{} physics: held {:.4f} ({}), free {:.4f} ({}), impulse {:.3f} ({}), max up {:.2f}, down {:.2f}",
                    gamemodeName(player), physics->accelHeld.value, physics->accelHeld.count,
                    physics->accelFree.value, physics->accelFree.count, physics->impulse.value,
                    physics->impulse.count, physics->maxUp, physics->maxDown
                );
            }
        }
        log::info(
            "Wave: {} hazards, {} solids nearby; max delay {}, tap gap {}; now survives {} ({}), "
            "best {} at delay {}",
            hazards, obstacles.size() - hazards, maxDelay, followGap, choice.survivedNow,
            choice.causeNow, choice.survivedBest, choice.delay
        );
    }

    // Draws what the prediction saw: the hitbox, the path if the input
    // fires now (red if it crashes, green if not), the path at the chosen
    // delay (yellow), and the obstacles it checked (hazards orange, blocks
    // and slope surfaces cyan).
    void drawWaveChoice(
        std::vector<PlayerObject*> const& players, std::vector<WaveObstacle> const& obstacles,
        bool held, int followGap, WaveChoice const& choice
    ) {
        auto draw = m_fields->waveDraw;
        if (!draw) return;
        draw->clear();

        ccColor4F none { 0.f, 0.f, 0.f, 0.f };
        ccColor4F hazardColor { 1.f, .55f, 0.f, 1.f };   // orange
        ccColor4F solidColor { 0.f, 1.f, 1.f, 1.f };     // cyan
        for (auto const& o : obstacles) {
            ccColor4F color = o.hazard || o.deadly ? hazardColor : solidColor;
            if (o.slope) {
                // The slope's surface line, which is what the wave collides with.
                draw->drawSegment(ccp(o.rect.getMinX(), o.y0), ccp(o.rect.getMaxX(), o.y1), 1.25f, color);
            }
            else if (o.radius > 0.f) {
                draw->drawCircle(toCC(o.center), o.radius, none, 1.f, color, 24);
            }
            else {
                draw->drawRect(ccp(o.rect.getMinX(), o.rect.getMinY()), ccp(o.rect.getMaxX(), o.rect.getMaxY()), none, 1.f, color);
            }
        }

        int lookahead = choice.lookahead;
        auto drawPath = [&](PlayerObject* player, int delay, ccColor4F color) {
            std::vector<CCPoint> path;
            int switchBack = followGap > 0 ? delay + followGap : -1;
            this->simulateWave(player, obstacles, held, delay, switchBack, lookahead + delay, &path);
            for (size_t i = 1; i < path.size(); ++i) draw->drawSegment(path[i - 1], path[i], .75f, color);
            if (!path.empty()) draw->drawDot(path.back(), 2.f, color);
        };

        bool crashesNow = choice.survivedNow < lookahead;
        for (auto player : players) {
            auto box = player->getObjectRect();
            draw->drawRect(box.origin, ccp(box.getMaxX(), box.getMaxY()), none, 1.f, ccColor4F { 1.f, 1.f, 1.f, 1.f });
            drawPath(player, 0, crashesNow ? ccColor4F { 1.f, .2f, .2f, 1.f } : ccColor4F { .2f, 1.f, .2f, 1.f });
            if (choice.delay > 0) drawPath(player, choice.delay, ccColor4F { 1.f, 1.f, .2f, 1.f });
        }
    }

    bool waveTracksReady(bool isPlayer1) {
        auto players = this->inputPlayers(isPlayer1);
        return std::all_of(players.begin(), players.end(), [&](PlayerObject* player) {
            auto const& t = this->waveTrack(player);
            return t.hasDelta && std::abs(t.delta.x) > 0.01f;
        });
    }

    // Ship, UFO and swing physics are learned while playing.
    bool physicsReady(bool isPlayer1) {
        auto players = this->inputPlayers(isPlayer1);
        return std::all_of(players.begin(), players.end(), [&](PlayerObject* player) {
            auto physics = physicsFor(player);
            return !physics || physics->ready(flyMode(player));
        });
    }

    char const* modeName(bool isPlayer1) {
        auto players = this->inputPlayers(isPlayer1);
        return players.empty() ? "?" : gamemodeName(players.front());
    }

    // Fires a held wave input now, and schedules its release if one came in.
    void fireWave(bool isPlayer1) {
        auto fields = m_fields.self();
        int ch = isPlayer1 ? 1 : 0;
        auto w = fields->wave[ch];
        fields->wave[ch] = {};
        if (!w.active) return;
        this->fire(w.push, static_cast<int>(PlayerButton::Jump), isPlayer1);
        if (w.hasFollow) {
            fields->waveFollow[ch] = { true, !w.push, fields->step + w.followGap };
        }
    }

    void fireWaveFollow(bool isPlayer1) {
        auto fields = m_fields.self();
        auto& f = fields->waveFollow[isPlayer1 ? 1 : 0];
        if (!f.active) return;
        bool push = f.push;
        f = {};
        this->fire(push, static_cast<int>(PlayerButton::Jump), isPlayer1);
    }

    bool onWaveButton(bool down, bool isPlayer1) {
        auto fields = m_fields.self();
        int ch = isPlayer1 ? 1 : 0;
        auto& w = fields->wave[ch];
        char const* what = down ? "click" : "release";
        auto mode = this->modeName(isPlayer1);

        if (w.active) {
            // The opposite input while one is held: keep the gap between them.
            if (down != w.push && !w.hasFollow) {
                w.hasFollow = true;
                w.followGap = std::max(1, this->inputStep() - w.arrivedStep);
                return true;
            }
            // Anything more: fire what is held and pass this one through.
            this->fireWave(isPlayer1);
            this->fireWaveFollow(isPlayer1);
            this->report(fmt::format("{}: another input came in, held input fired", mode));
            return false;
        }
        // A scheduled release must reach the game before a new input.
        this->fireWaveFollow(isPlayer1);

        // Not a change of direction (e.g. a repeat): nothing to time.
        if (fields->jumpHeld[ch] == down) return false;
        // UFO, swing, cube and ball only act on presses.
        if (!down) {
            auto players = this->inputPlayers(isPlayer1);
            bool releaseMatters = std::any_of(players.begin(), players.end(), [](PlayerObject* player) {
                auto m = flyMode(player);
                return m == FlyMode::Wave || m == FlyMode::Ship || m == FlyMode::Robot;
            });
            if (!releaseMatters) return false;
        }

        int frames = bufferFrames();
        if (frames <= 0) return false;
        if (!this->waveTracksReady(isPlayer1)) {
            this->report(fmt::format("{}: no speed reading yet, {} passed through", mode, what));
            return false;
        }
        if (!this->physicsReady(isPlayer1)) {
            auto players = this->inputPlayers(isPlayer1);
            bool ground = std::any_of(players.begin(), players.end(), [](PlayerObject* player) {
                return isGroundMode(flyMode(player));
            });
            bool mini = !players.empty() && players.front()->m_vehicleSize < 1.f;
            this->report(fmt::format(
                "{}{}: still learning its physics ({}), {} passed through",
                mini ? "mini " : "", mode, ground ? "jump a few times" : "fly a few seconds", what
            ));
            return false;
        }

        auto choice = this->chooseWaveDelay(isPlayer1, down, frames, 0);
        if (choice.delay == 0) {
            if (choice.survivedNow >= choice.lookahead) {
                this->report(fmt::format("{}: path clear, {} passed through", mode, what));
            }
            else {
                this->report(fmt::format(
                    "{}: {} hits {} in {} steps, best wait survives {}, passed through",
                    mode, what, choice.causeNow, choice.survivedNow, choice.survivedBest
                ));
            }
            return false;
        }

        w = {};
        w.active = true;
        w.push = down;
        w.arrivedStep = this->inputStep();
        this->report(fmt::format(
            "{}: {} would hit {} in {} steps, holding {} steps",
            mode, what, choice.causeNow, choice.survivedNow, choice.delay
        ));
        return true;
    }

    // Each step: fire scheduled releases and held inputs whose path is clear.
    void stepWave() {
        auto fields = m_fields.self();
        bool active = modEnabled() && currentMode() == Mode::Buffer && this->assistAllowed();
        for (int ch = 0; ch < 2; ++ch) {
            bool isPlayer1 = ch == 1;
            auto& f = fields->waveFollow[ch];
            if (f.active && f.step <= fields->step) this->fireWaveFollow(isPlayer1);

            auto& w = fields->wave[ch];
            if (!w.active) continue;
            w.waited++;
            int waited = w.waited;
            char const* what = w.push ? "click" : "release";
            auto mode = this->modeName(isPlayer1);

            if (!active || !this->lookaheadApplies(isPlayer1)) {
                this->fireWave(isPlayer1);
                this->report(fmt::format("{}: mode changed or mod off, {} fired {} steps late", mode, what, waited));
                continue;
            }
            int remaining = std::max(0, bufferFrames() - waited);
            int followGap = w.hasFollow ? w.followGap : 0;
            auto choice = this->chooseWaveDelay(isPlayer1, w.push, remaining, followGap);
            if (choice.delay == 0) {
                this->fireWave(isPlayer1);
                this->report(choice.survivedNow >= choice.lookahead
                    ? fmt::format("{}: path clear, {} fired {} steps late", mode, what, waited)
                    : fmt::format("{}: best moment, {} fired {} steps late", mode, what, waited));
            }
        }
    }

    // ---- Snap mode / best run ----

    bool ensureReference() {
        auto fields = m_fields.self();
        auto path = referencePath(m_level, m_startPosObject);
        if (!fields->referenceLoaded || fields->referenceFile != path) {
            fields->referenceLoaded = true;
            fields->referenceFile = path;
            auto ref = loadReference(path);
            fields->reference = std::move(ref.events);
            fields->referencePath = std::move(ref.path);
            fields->referenceLength = ref.length;
            fields->referenceCompleted = ref.completed;
            // Only the snap state is rewound here; the attempt's recording
            // must survive a reload.
            int step = fields->step;
            for (auto& r : fields->reference) r.matched = r.input.step <= step;
        }
        return !fields->reference.empty();
    }

    // How far the players are from where the best run had them on this
    // step, or -1 if the best run doesn't reach this step.
    float distanceFromBestRun(int step) {
        auto fields = m_fields.self();
        auto const& path = fields->referencePath;
        if (step < 0 || static_cast<size_t>(step) >= path.size()) return -1.f;

        auto const& ref = path[step];
        auto now = this->currentPoint();
        float distance = std::hypot(now.x1 - ref.x1, now.y1 - ref.y1);
        if (m_gameState.m_isDualMode) distance = std::max(distance, std::hypot(now.x2 - ref.x2, now.y2 - ref.y2));
        return distance;
    }

    // Describes why a press got no best-run click inside the window.
    std::string describeNoMatch(int button, bool isPlayer1, int inputStep, int window) {
        constexpr int kSearch = 120;
        RefEvent const* nearest = nullptr;
        for (auto const& ref : m_fields->reference) {
            if (ref.input.step > inputStep + kSearch) break;
            if (ref.input.step < inputStep - kSearch) continue;
            if (ref.input.button != button || !ref.input.push || ref.input.isPlayer1 != isPlayer1) continue;
            if (!nearest || std::abs(ref.input.step - inputStep) < std::abs(nearest->input.step - inputStep)) {
                nearest = &ref;
            }
        }
        if (!nearest) return "no best-run click near here, passed through";
        int offset = nearest->input.step - inputStep;
        if (offset > 0) {
            return fmt::format("best run clicked {} steps later (window {}), passed through", offset, window);
        }
        if (offset == 0) return "best-run click here was already used, passed through";
        return fmt::format("{} steps after the best run's click, too late to move", -offset);
    }

    SnapHold& hold(int button, bool isPlayer1) {
        return m_fields->holds[isPlayer1 ? 1 : 0][button - 1];
    }

    // First unused reference input of this kind on a step in [from, to].
    RefEvent* findReference(int button, bool push, bool isPlayer1, int from, int to) {
        auto& refs = m_fields->reference;
        auto it = std::lower_bound(refs.begin(), refs.end(), from, [](RefEvent const& ref, int step) {
            return ref.input.step < step;
        });
        for (; it != refs.end() && it->input.step <= to; ++it) {
            if (it->matched) continue;
            if (it->input.button != button || it->input.push != push || it->input.isPlayer1 != isPlayer1) continue;
            return &*it;
        }
        return nullptr;
    }

    void schedule(int step, int button, bool push, bool isPlayer1) {
        auto& scheduled = m_fields->scheduled;
        InputEvent e { step, button, push, isPlayer1 };
        auto it = std::upper_bound(scheduled.begin(), scheduled.end(), step, [](int step, InputEvent const& e) {
            return step < e.step;
        });
        scheduled.insert(it, e);
    }

    bool onSnapButton(bool down, int button, bool isPlayer1) {
        // Without a reference, inputs pass through untouched.
        if (!this->ensureReference()) {
            if (down) this->report("no best run here yet, play this part once first");
            return false;
        }

        int window = snapWindow();
        int inputStep = this->inputStep();
        auto& h = hold(button, isPlayer1);

        // Released before the snapped press fired: keep the hold length,
        // never release before the press, and snap to a reference release
        // if one is in the window.
        if (!down && h.pressPending) {
            int target = std::max(inputStep + h.shift, h.pressStep + 1);
            if (auto ref = this->findReference(button, false, isPlayer1, target, target + window)) {
                ref->matched = true;
                target = ref->input.step;
            }
            h.pressPending = false;
            this->schedule(target, button, false, isPlayer1);
            log::debug("Snap: {} moved to step {}", buttonName(button, down), target);
            return true;
        }

        auto mode = gamemodeName(m_player1);
        float distance = this->distanceFromBestRun(inputStep);
        if (distance < 0.f) {
            if (down) this->report(fmt::format("{}: past the end of your best run, passed through", mode));
            return false;
        }
        if (distance > pathTolerance()) {
            if (down) {
                this->report(fmt::format(
                    "{}: {:.0f} units off your best run's path (tolerance {:.0f}), passed through",
                    mode, distance, pathTolerance()
                ));
            }
            return false;
        }

        auto ref = this->findReference(button, down, isPlayer1, inputStep, inputStep + window);
        if (!ref) {
            if (down) this->report(fmt::format("{}: {}", mode, this->describeNoMatch(button, isPlayer1, inputStep, window)));
            return false;
        }

        ref->matched = true;
        int target = ref->input.step;
        if (target == inputStep) {
            if (down) this->report(fmt::format("{}: on time", mode));
            return false;
        }

        this->schedule(target, button, down, isPlayer1);
        if (down) {
            h.pressPending = true;
            h.pressStep = target;
            h.shift = target - inputStep;
        }
        if (down) this->report(fmt::format("{}: {} steps early, moved onto the best run's click", mode, target - inputStep));
        else log::info("Step {}: release {} steps early, moved", m_fields->step, target - inputStep);
        return true;
    }

    void stepSnap() {
        auto fields = m_fields.self();
        auto& scheduled = fields->scheduled;
        size_t count = 0;
        while (count < scheduled.size() && scheduled[count].step <= fields->step) ++count;
        if (count == 0) return;

        std::vector<InputEvent> due(scheduled.begin(), scheduled.begin() + count);
        scheduled.erase(scheduled.begin(), scheduled.begin() + count);
        for (auto const& e : due) {
            if (e.push) {
                auto& h = hold(e.button, e.isPlayer1);
                if (h.pressPending && h.pressStep == e.step) h.pressPending = false;
            }
            this->fire(e.push, e.button, e.isPlayer1);
        }
    }
};

class $modify(BufferGameLayer, GJBaseGameLayer) {
    BufferPlayLayer* bufferLayer() {
        auto pl = PlayLayer::get();
        if (!pl || static_cast<GJBaseGameLayer*>(pl) != this) return nullptr;
        return static_cast<BufferPlayLayer*>(pl);
    }

    void handleButton(bool down, int button, bool isPlayer1) {
        auto pl = this->bufferLayer();
        if (pl) {
            if (pl->onButton(down, button, isPlayer1)) return;
            pl->noteInput(down, button, isPlayer1);
            pl->recordInput(down, button, isPlayer1);
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }

    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        auto pl = this->bufferLayer();
        if (pl) {
            // Inputs fired from onStep are processed on this step.
            pl->setInStep(true);
            pl->onStep();
        }
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
        if (pl) pl->setInStep(false);
    }
};

class $modify(BufferPauseLayer, PauseLayer) {
    void customSetup() {
        PauseLayer::customSetup();

        CCNode* sprite = CCSprite::createWithSpriteFrameName("GJ_optionsBtn02_001.png");
        if (!sprite) sprite = CCSprite::createWithSpriteFrameName("GJ_optionsBtn_001.png");
        if (!sprite) sprite = CCLabelBMFont::create("IB", "bigFont.fnt");
        sprite->setScale(0.6f);
        auto button = CCMenuItemSpriteExtra::create(sprite, this, menu_selector(BufferPauseLayer::onInputBufferSettings));
        button->setID("settings-button"_spr);

        auto menu = CCMenu::create();
        menu->setID("menu"_spr);
        menu->addChild(button);
        menu->setPosition({ 30.f, 30.f });
        this->addChild(menu, 10);
    }

    void onInputBufferSettings(CCObject*) {
        openSettingsPopup(Mod::get());
    }
};

namespace {
    // Hotkeys only act inside a practice or start position run.
    BufferPlayLayer* hotkeyLayer() {
        auto pl = static_cast<BufferPlayLayer*>(PlayLayer::get());
        if (!pl) return nullptr;
        if (!pl->assistAllowed()) {
            notify("Input Buffer only works in practice mode or from a start position", NotificationIcon::Warning);
            return nullptr;
        }
        return pl;
    }

    void adjustHelp(int delta) {
        auto pl = hotkeyLayer();
        if (!pl) return;

        char const* key = nullptr;
        char const* label = nullptr;
        switch (currentMode()) {
            case Mode::Buffer: key = "buffer-frames"; label = "Buffer"; break;
            case Mode::Snap: key = "snap-window"; label = "Snap window"; break;
        }
        auto value = Mod::get()->getSettingValue<int64_t>(key);
        value = std::clamp<int64_t>(value + delta, 0, 30);
        Mod::get()->setSettingValue<int64_t>(key, value);
        notify(fmt::format("{}: {} steps", label, value), NotificationIcon::Info);
        pl->updateIndicator();
    }
}

$on_mod(Loaded) {
    loadPhysics();
    listenForKeybindSettingPresses("toggle-key", [](Keybind const&, bool down, bool repeat, double) {
        if (!down || repeat) return false;
        auto pl = hotkeyLayer();
        if (!pl) return false;
        bool enabled = !modEnabled();
        Mod::get()->setSettingValue<bool>("enabled", enabled);
        notify(enabled ? "Input Buffer: on" : "Input Buffer: off", NotificationIcon::Info);
        pl->updateIndicator();
        return true;
    });
    listenForKeybindSettingPresses("more-help-key", [](Keybind const&, bool down, bool, double) {
        if (!down) return false;
        adjustHelp(1);
        return true;
    });
    listenForKeybindSettingPresses("less-help-key", [](Keybind const&, bool down, bool, double) {
        if (!down) return false;
        adjustHelp(-1);
        return true;
    });
}
