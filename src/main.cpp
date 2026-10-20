// Input Buffer
//
// Three modes, chosen by the "mode" setting:
//
//   Buffer             Holds a PlayerButton press for up to `buffer-frames`
//                      physics steps while the player cannot act on it yet,
//                      then fires it on the first step the player can.
//   Record reference   Records every PlayerButton event with the physics step
//                      it is processed on. A completed run is saved as the
//                      level's reference.
//   Snap to reference  Only touches inputs in the short window before a
//                      reference input. A press or release that comes up to
//                      `snap-window` steps before a reference input of the
//                      same kind (button, press/release, player) is held and
//                      fired on the reference step, which puts the player
//                      back on the recorded path. Any other input, early by
//                      more than the window, late, or with no reference input
//                      ahead, reaches the game unchanged. Nothing is fired
//                      that the player did not press.
//
// Buffer and Snap only act in practice mode or when the level was started
// from a start position (GJBaseGameLayer::m_isPracticeMode, m_isTestMode,
// m_startPosObject); in a normal run inputs pass through untouched. Record
// works everywhere since it does not change any input. A label in the corner
// shows whenever a mode is active.
//
// Hotkeys (Geode keybind settings, remappable): toggle the mod, and raise or
// lower the help steps (buffer-frames in Buffer mode, snap-window in Snap
// mode). They only respond inside a practice or start position run. The
// pause menu gets a button that opens the mod's settings.
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
//   m_touchingRings   CCArray*; addToTouchedRings() (inline) adds the ring being
//                     touched. When the game empties it is not visible in the
//                     bindings.
//   m_isDead          PlayLayer::isGameplayActive() (inline) reads it.
//
// Buffer mode:
// | mode   | input type | buffered when                 | fires when            |
// |--------|------------|-------------------------------|-----------------------|
// | cube   | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | ball   | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | robot  | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | spider | discrete   | !m_isOnGround && no ring      | m_isOnGround or ring  |
// | ship   | continuous | never (pass-through)          | -                     |
// | ufo    | continuous | never (pass-through)          | -                     |
// | wave   | continuous | never (pass-through)          | -                     |
// | swing  | continuous | never (pass-through)          | -                     |
//
// Continuous modes are the isFlying() group. Their input does not require
// ground contact, so the player can always act on a press and Buffer mode
// has no state to wait for. Snap mode covers them, and every other mode,
// because it times inputs against the reference instead of the player state.
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
// since step counting starts over at the start position. One event per line:
//   <step> <button 1-3> <push 0/1> <player1 0/1>

#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>
#include <Geode/ui/GeodeUI.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include <vector>

using namespace geode::prelude;

namespace {
    // PlayerButton is Jump = 1, Left = 2, Right = 3 (Geode/Enums.hpp).
    constexpr int kButtonCount = 3;

    enum class Mode { Buffer, Snap, Record };

    struct BufferSlot {
        bool pending = false;        // a press is being held back
        bool releasePending = false; // the button was released while held back
        bool releaseNextStep = false;// replay the release on the next step
        int framesLeft = 0;
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

    struct SnapHold {
        bool pressPending = false; // a snapped press has not fired yet
        int pressStep = 0;         // step the snapped press fires on
        int shift = 0;             // steps the press was moved by
    };

    Mode currentMode() {
        auto mode = Mod::get()->getSettingValue<std::string>("mode");
        if (mode == "Snap to reference") return Mode::Snap;
        if (mode == "Record reference") return Mode::Record;
        return Mode::Buffer;
    }

    bool modEnabled() {
        return Mod::get()->getSettingValue<bool>("enabled");
    }

    int bufferFrames() {
        return static_cast<int>(Mod::get()->getSettingValue<int64_t>("buffer-frames"));
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

    bool canAct(PlayerObject* player) {
        if (!player || player->m_isDead) return false;
        if (isContinuousMode(player)) return true;
        if (player->m_isOnGround) return true;
        return player->m_touchingRings && player->m_touchingRings->count() > 0;
    }

    std::filesystem::path referencePath(GJGameLevel* level, StartPosObject* startPos) {
        int id = level->m_levelID.value();
        auto name = id > 0 ? fmt::format("{}", id) : fmt::format("local-{}", level->m_M_ID);
        if (startPos) name += fmt::format("-sp{}", static_cast<int>(std::round(startPos->getPositionX())));
        return Mod::get()->getSaveDir() / "references" / (name + ".txt");
    }

    std::vector<RefEvent> loadReference(std::filesystem::path const& path) {
        std::vector<RefEvent> events;
        std::ifstream file(path);
        InputEvent e;
        int push, player1;
        while (file >> e.step >> e.button >> push >> player1) {
            if (e.button < 1 || e.button > kButtonCount) continue;
            e.push = push != 0;
            e.isPlayer1 = player1 != 0;
            events.push_back({ e });
        }
        std::stable_sort(events.begin(), events.end(), [](auto const& a, auto const& b) {
            return a.input.step < b.input.step;
        });
        return events;
    }

    bool saveReference(std::filesystem::path const& path, std::vector<InputEvent> const& events) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        std::ofstream file(path, std::ios::trunc);
        if (!file) return false;
        for (auto const& e : events) {
            file << e.step << ' ' << e.button << ' ' << (e.push ? 1 : 0) << ' ' << (e.isPlayer1 ? 1 : 0) << '\n';
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

        // Record mode
        std::vector<InputEvent> recorded;

        // Snap mode
        bool referenceLoaded = false;
        std::filesystem::path referenceFile;
        std::vector<RefEvent> reference;
        std::vector<InputEvent> scheduled; // snapped inputs waiting for their step
        SnapHold holds[2][kButtonCount];   // indexed like slots

        bool inStep = false;     // inside GJBaseGameLayer::processCommands

        CCLabelBMFont* indicator = nullptr;
        std::string indicatorText;
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
            case Mode::Record:
                text = "Input Buffer: recording";
                break;
            case Mode::Buffer:
                if (this->assistAllowed() && bufferFrames() > 0) {
                    text = fmt::format("Input Buffer: {} steps", bufferFrames());
                }
                break;
            case Mode::Snap:
                if (this->assistAllowed() && !fields->reference.empty()) {
                    text = fmt::format("Input Buffer: snap {} steps", snapWindow());
                }
                break;
        }
        if (text == fields->indicatorText) return;
        fields->indicatorText = text;
        fields->indicator->setString(text.c_str());
        fields->indicator->setVisible(!text.empty());
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
            case Mode::Record:
                m_fields->recorded.push_back({ this->inputStep(), button, down, isPlayer1 });
                return false;
            case Mode::Snap:
                if (!this->assistAllowed()) return false;
                return this->onSnapButton(down, button, isPlayer1);
            case Mode::Buffer:
                if (!this->assistAllowed()) return false;
                return this->onBufferButton(down, button, isPlayer1);
        }
        return false;
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
        this->stepBuffer();
        if (modEnabled() && currentMode() == Mode::Snap && this->assistAllowed()) {
            if (!fields->referenceLoaded) this->ensureReference();
            this->stepSnap();
        }
        this->updateIndicator();
    }

    void clearBuffer() {
        auto fields = m_fields.self();
        for (auto& player : fields->slots) {
            for (auto& slot : player) slot = {};
        }
    }

    // Puts every per-attempt state back to fields->step, which is 0 after a
    // full reset or the checkpoint's step after a checkpoint respawn.
    void rewindToStep() {
        auto fields = m_fields.self();
        int step = fields->step;
        std::erase_if(fields->recorded, [step](auto const& e) { return e.step > step; });

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
        // The start position may have changed, which selects another reference.
        if (modEnabled() && currentMode() == Mode::Snap && this->assistAllowed()) this->ensureReference();
        this->rewindToStep();
        this->updateIndicator();
    }

    void destroyPlayer(PlayerObject* player, GameObject* object) {
        PlayLayer::destroyPlayer(player, object);
        if (player && player->m_isDead) this->clearBuffer();
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
    }

    void removeAllCheckpoints() {
        PlayLayer::removeAllCheckpoints();
        m_fields->checkpointSteps.clear();
    }

    void levelComplete() {
        if (currentMode() == Mode::Record) {
            auto path = referencePath(m_level, m_startPosObject);
            if (saveReference(path, m_fields->recorded)) {
                notify(fmt::format(
                    "Reference saved{} ({} inputs)",
                    m_startPosObject ? " for this start position" : "",
                    m_fields->recorded.size()
                ), NotificationIcon::Success);
                // Pick it up if the player switches to Snap mode on this level.
                m_fields->referenceLoaded = false;
            }
            else {
                notify("Failed to save reference", NotificationIcon::Error);
            }
        }
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
        if (twoPlayer) return canAct(isPlayer1 ? m_player1 : m_player2);
        if (canAct(m_player1)) return true;
        return m_gameState.m_isDualMode && canAct(m_player2);
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
        if (inputIsContinuous(isPlayer1)) return false;
        if (inputCanAct(isPlayer1)) return false;

        if (s.releaseNextStep) this->fire(false, button, isPlayer1);
        s.pending = true;
        s.releasePending = false;
        s.releaseNextStep = false;
        s.framesLeft = frames;
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

                if (inputCanAct(isPlayer1)) {
                    bool released = s.releasePending;
                    s = {};
                    this->fire(true, b, isPlayer1);
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
            }
        }
    }

    // ---- Snap mode ----

    bool ensureReference() {
        auto fields = m_fields.self();
        auto path = referencePath(m_level, m_startPosObject);
        if (!fields->referenceLoaded || fields->referenceFile != path) {
            fields->referenceLoaded = true;
            fields->referenceFile = path;
            fields->reference = loadReference(path);
            this->rewindToStep();
            if (fields->reference.empty()) {
                notify(m_startPosObject
                    ? "No reference for this start position - record one first"
                    : "No reference for this level - record one first",
                    NotificationIcon::Warning);
            }
        }
        return !fields->reference.empty();
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
        if (!this->ensureReference()) return false;

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

        auto ref = this->findReference(button, down, isPlayer1, inputStep, inputStep + window);
        if (!ref) return false;

        ref->matched = true;
        int target = ref->input.step;
        if (target == inputStep) return false;

        this->schedule(target, button, down, isPlayer1);
        if (down) {
            h.pressPending = true;
            h.pressStep = target;
            h.shift = target - inputStep;
        }
        log::debug("Snap: {} {} steps early, moved to step {}", buttonName(button, down), target - inputStep, target);
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
        if (auto pl = this->bufferLayer()) {
            if (pl->onButton(down, button, isPlayer1)) return;
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }

    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        auto pl = this->bufferLayer();
        if (pl) {
            pl->onStep();
            pl->setInStep(true);
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
            case Mode::Record:
                notify("Recording - switch to Buffer or Snap to change help steps", NotificationIcon::Info);
                return;
        }
        auto value = Mod::get()->getSettingValue<int64_t>(key);
        value = std::clamp<int64_t>(value + delta, 0, 30);
        Mod::get()->setSettingValue<int64_t>(key, value);
        notify(fmt::format("{}: {} steps", label, value), NotificationIcon::Info);
        pl->updateIndicator();
    }
}

$on_mod(Loaded) {
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
