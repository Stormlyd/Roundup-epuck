# ZooidManager 多机器人同步测试模式 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 ZooidManager 中增加一个由界面按钮启动的单轮同步运动测试，让启动时在线且激活的全部 e-puck2 机器人经现有 USB 接收器按固定速度序列运动，并在任何结束路径安全归零。

**Architecture:** 纯 C++14 的 `ZooidTestMode` 只负责单调时间驱动的动作阶段；`ZooidSpeedCodec` 负责 `+2007` 速度编码和限幅；`ZooidTestTargets` 负责不可扩张的目标快照。`ZooidManager` 组合三者并独占串口输出，Qt `HomePage` 只发出开始/停止请求和显示状态。

**Tech Stack:** C++14、Qt 5.9.7 Widgets、Qt SerialPort、MinGW 5.3、qmake、现有 ZooidManager USB 帧协议。

## Global Constraints

- 保留现有 Voronoi、Follow、Charge、Triangle 等模式源码，但测试界面不提供这些模式的操作入口。
- 第一阶段不增加 ROS 依赖，也不运行 `epuck-pursuit-webots` ROS 节点。
- 轮速输入必须钳制到 `[-1000, +1000]`，协议字段必须编码为 `speed + 2000 + 7`。
- 外层串口帧继续使用 `0x7E | TYPE_ROBOT_POSITION | localRobotId | payloadLength | payload | 0x21`。
- 测试动作固定为：前进 2 秒、停 1 秒、左转 2 秒、停 1 秒、右转 2 秒、停 1 秒、后退 2 秒、最终连续 3 个发送周期零速。
- 测试启动时只快照 `connected && activated` 的机器人；本轮中途上线的机器人不加入。
- 手动停止、自然完成、全部目标丢失、接收器写失败和程序退出都必须进入相同的零速停止路径。
- 测试运行时不得调用 `assignRobots()`、`runSimulation()`、`sendRobotsOrders()` 或其他旧模式动作输出。
- 计时使用 `std::chrono::steady_clock`，不能使用 `clock()` 作为测试阶段时钟。
- 当前 `ZooidManager` 目录不是 Git 仓库；不得擅自执行 `git init`。各任务末尾保存可验证检查点，只有项目所有者建立仓库后才执行列出的提交命令。

---

## File Structure

- Create `manager/ZooidTestMode.h`: 定义轮速、阶段输出和纯时间状态机接口。
- Create `manager/ZooidTestMode.cpp`: 实现固定动作时间线和单次完成语义。
- Create `manager/ZooidSpeedCodec.h`: 定义纯协议字段编码结果和编码函数。
- Create `manager/ZooidSpeedCodec.cpp`: 实现限幅及 `+2007` 偏置。
- Create `manager/ZooidTestTargets.h`: 定义本轮目标 ID 快照和只缩减的有效集合。
- Create `manager/ZooidTestTargets.cpp`: 实现启动快照、掉线剔除和丢失 ID 记录。
- Create `tests/zooid_core_tests.cpp`: 不依赖 Qt 的状态机、编码和目标集合测试程序。
- Modify `ZooidManager.pro`: 把三个纯 C++ 组件加入正式应用构建。
- Modify `manager/ZooidReceiver.h`: 增加异步串口写失败的原子可观测状态。
- Modify `manager/ZooidReceiver.cpp`: 在写失败处记录错误，并提供一次性消费接口。
- Modify `manager/ZooidManager.h`: 定义公开测试状态/API和线程安全的测试会话字段。
- Modify `manager/ZooidManager.cpp`: 实现目标快照、独占发送、三周期零速、异常停止与退出等待。
- Modify `homePage.h`: 增加测试按钮、状态标签和槽函数。
- Modify `homePage.cpp`: 只显示测试/停止控制入口并轮询管理器状态。
- Create `docs/test-mode-operation.md`: 写实机安全操作与验收说明；当前项目没有 `README.md`。

---

### Task 1: Pure fixed-sequence state machine

**Files:**
- Create: `manager/ZooidTestMode.h`
- Create: `manager/ZooidTestMode.cpp`
- Create: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes: 调用方提供的 `uint64_t nowMs` 单调毫秒时间。
- Produces: `bool ZooidTestMode::start(uint64_t nowMs)`、`void ZooidTestMode::stop()`、`bool ZooidTestMode::isRunning() const`、`TestModeOutput ZooidTestMode::update(uint64_t nowMs)`。

- [ ] **Step 1: Write the failing state-machine boundary tests**

Create the initial test harness with explicit comparisons:

```cpp
#include "../manager/ZooidTestMode.h"
#include <cstdlib>
#include <iostream>

static int failures = 0;

static void expectCommand(const char* name,
                          const TestModeOutput& output,
                          int16_t left,
                          int16_t right,
                          bool running,
                          bool completed)
{
    if (output.wheels.left != left || output.wheels.right != right ||
        output.running != running || output.completed != completed) {
        std::cerr << name << " failed\n";
        ++failures;
    }
}

static void testFixedSequence()
{
    ZooidTestMode mode;
    if (!mode.start(1000)) ++failures;
    expectCommand("start", mode.update(1000), 200, 200, true, false);
    expectCommand("forward-last", mode.update(2999), 200, 200, true, false);
    expectCommand("pause-1", mode.update(3000), 0, 0, true, false);
    expectCommand("left", mode.update(4000), -150, 150, true, false);
    expectCommand("pause-2", mode.update(6000), 0, 0, true, false);
    expectCommand("right", mode.update(7000), 150, -150, true, false);
    expectCommand("pause-3", mode.update(9000), 0, 0, true, false);
    expectCommand("reverse", mode.update(10000), -200, -200, true, false);
    expectCommand("complete", mode.update(12000), 0, 0, false, true);
    expectCommand("complete-once", mode.update(12001), 0, 0, false, false);
}

static void testRestartAndManualStop()
{
    ZooidTestMode mode;
    if (!mode.start(0)) ++failures;
    if (mode.start(1)) ++failures;
    mode.stop();
    expectCommand("manual-stop", mode.update(2), 0, 0, false, false);
    if (!mode.start(10)) ++failures;
    expectCommand("restart-after-stop", mode.update(10), 200, 200, true, false);
}

int main()
{
    testFixedSequence();
    testRestartAndManualStop();
    if (failures != 0) return EXIT_FAILURE;
    std::cout << "zooid core tests passed\n";
    return EXIT_SUCCESS;
}
```

- [ ] **Step 2: Compile to verify the state-machine test fails**

Run from the ZooidManager root:

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp -o tests\zooid_core_tests.exe
```

Expected: compilation fails because `ZooidTestMode.h/.cpp` do not exist.

- [ ] **Step 3: Implement the minimal deterministic state machine**

Use these exact public types in `manager/ZooidTestMode.h`:

```cpp
#ifndef ZOOIDTESTMODE_H
#define ZOOIDTESTMODE_H

#include <cstdint>

struct WheelCommand {
    int16_t left = 0;
    int16_t right = 0;
};

struct TestModeOutput {
    WheelCommand wheels;
    bool running = false;
    bool completed = false;
};

class ZooidTestMode {
public:
    bool start(uint64_t nowMs);
    void stop();
    bool isRunning() const;
    TestModeOutput update(uint64_t nowMs);

private:
    bool running_ = false;
    bool completionPending_ = false;
    uint64_t startedAtMs_ = 0;
};

#endif
```

In `manager/ZooidTestMode.cpp`, compute `elapsed = nowMs >= startedAtMs_ ? nowMs - startedAtMs_ : 0` and use these half-open intervals:

```cpp
0 <= elapsed < 2000       -> { 200,  200}
2000 <= elapsed < 3000    -> {   0,    0}
3000 <= elapsed < 5000    -> {-150,  150}
5000 <= elapsed < 6000    -> {   0,    0}
6000 <= elapsed < 8000    -> { 150, -150}
8000 <= elapsed < 9000    -> {   0,    0}
9000 <= elapsed < 11000   -> {-200, -200}
elapsed >= 11000          -> running=false, completed=true once, {0,0}
```

`stop()` sets both flags false. `start()` returns false while already running; otherwise records `nowMs`, clears the completion latch, and returns true.

- [ ] **Step 4: Compile and run the state-machine tests**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp -o tests\zooid_core_tests.exe
& '.\tests\zooid_core_tests.exe'
```

Expected: exit code 0 and `zooid core tests passed`.

- [ ] **Step 5: Save the task checkpoint**

Record the passing command and output in the implementation log. If the owner has established Git by execution time, use:

```powershell
git add manager/ZooidTestMode.h manager/ZooidTestMode.cpp tests/zooid_core_tests.cpp
git commit -m "feat: add deterministic robot test sequence"
```

---

### Task 2: Wheel-speed protocol codec

**Files:**
- Create: `manager/ZooidSpeedCodec.h`
- Create: `manager/ZooidSpeedCodec.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes: signed wheel speeds in ZooidManager units.
- Produces: `EncodedWheelSpeeds encodeWheelSpeeds(int left, int right)` with clamped raw values and encoded `uint16_t positionX/positionY`.

- [ ] **Step 1: Add failing codec tests**

Add the include and test function:

```cpp
#include "../manager/ZooidSpeedCodec.h"

static void testSpeedCodec()
{
    EncodedWheelSpeeds forward = encodeWheelSpeeds(200, 200);
    if (forward.positionX != 2207 || forward.positionY != 2207) ++failures;

    EncodedWheelSpeeds turn = encodeWheelSpeeds(-150, 150);
    if (turn.positionX != 1857 || turn.positionY != 2157) ++failures;

    EncodedWheelSpeeds stop = encodeWheelSpeeds(0, 0);
    if (stop.positionX != 2007 || stop.positionY != 2007) ++failures;

    EncodedWheelSpeeds clamped = encodeWheelSpeeds(1200, -1300);
    if (clamped.left != 1000 || clamped.right != -1000 ||
        clamped.positionX != 3007 || clamped.positionY != 1007) ++failures;
}
```

Call `testSpeedCodec()` from `main()`.

- [ ] **Step 2: Compile to verify the codec test fails**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp manager\ZooidSpeedCodec.cpp -o tests\zooid_core_tests.exe
```

Expected: compilation fails because `ZooidSpeedCodec` is undefined.

- [ ] **Step 3: Implement the codec**

Use this header contract:

```cpp
#ifndef ZOOIDSPEEDCODEC_H
#define ZOOIDSPEEDCODEC_H

#include <cstdint>

struct EncodedWheelSpeeds {
    int16_t left;
    int16_t right;
    uint16_t positionX;
    uint16_t positionY;
};

EncodedWheelSpeeds encodeWheelSpeeds(int left, int right);

#endif
```

Implement a file-local clamp to `[-1000, 1000]`, then encode both outputs with `clamped + 2007`. Do not expose configurable offsets in this phase.

- [ ] **Step 4: Compile and run all core tests**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp manager\ZooidSpeedCodec.cpp -o tests\zooid_core_tests.exe
& '.\tests\zooid_core_tests.exe'
```

Expected: exit code 0 and `zooid core tests passed`.

- [ ] **Step 5: Save the task checkpoint**

If Git exists:

```powershell
git add manager/ZooidSpeedCodec.h manager/ZooidSpeedCodec.cpp tests/zooid_core_tests.cpp
git commit -m "feat: encode bounded wheel speed commands"
```

---

### Task 3: Immutable-start target set

**Files:**
- Create: `manager/ZooidTestTargets.h`
- Create: `manager/ZooidTestTargets.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes: the sorted or unsorted list of active global IDs at start, followed by current active global IDs each update.
- Produces: `startSnapshot(...)`, `retainActive(...)`, `activeIds()`, `lostIds()`, `empty()`, and `clear()`; no method can add an ID after `startSnapshot`.

- [ ] **Step 1: Add failing snapshot and drop-out tests**

```cpp
#include "../manager/ZooidTestTargets.h"
#include <vector>

static void expectIds(const std::vector<unsigned int>& actual,
                      const std::vector<unsigned int>& expected)
{
    if (actual != expected) ++failures;
}

static void testTargetSnapshot()
{
    ZooidTestTargets targets;
    targets.startSnapshot({12, 1, 12, 5});
    expectIds(targets.activeIds(), {1, 5, 12});

    targets.retainActive({1, 5, 9, 12, 20});
    expectIds(targets.activeIds(), {1, 5, 12});
    expectIds(targets.lostIds(), {});

    targets.retainActive({1, 9, 20});
    expectIds(targets.activeIds(), {1});
    expectIds(targets.lostIds(), {5, 12});

    targets.retainActive({1, 5, 12});
    expectIds(targets.activeIds(), {1});
    targets.clear();
    if (!targets.empty()) ++failures;
}
```

Call `testTargetSnapshot()` from `main()`.

- [ ] **Step 2: Compile to verify the target test fails**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp manager\ZooidSpeedCodec.cpp manager\ZooidTestTargets.cpp -o tests\zooid_core_tests.exe
```

Expected: compilation fails because `ZooidTestTargets` is undefined.

- [ ] **Step 3: Implement a sorted, duplicate-free, shrink-only set**

Use this public contract:

```cpp
class ZooidTestTargets {
public:
    void startSnapshot(const std::vector<unsigned int>& ids);
    void retainActive(const std::vector<unsigned int>& currentlyActiveIds);
    const std::vector<unsigned int>& activeIds() const;
    const std::vector<unsigned int>& lostIds() const;
    bool empty() const;
    void clear();

private:
    std::vector<unsigned int> activeIds_;
    std::vector<unsigned int> lostIds_;
};
```

`startSnapshot` sorts and removes duplicates. `retainActive` uses set intersection for the retained list and set difference for newly lost IDs; it never unions current IDs into the snapshot. `lostIds_` accumulates unique IDs in ascending order until `clear()`.

- [ ] **Step 4: Compile and run all core tests**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp manager\ZooidSpeedCodec.cpp manager\ZooidTestTargets.cpp -o tests\zooid_core_tests.exe
& '.\tests\zooid_core_tests.exe'
```

Expected: exit code 0 and `zooid core tests passed`.

- [ ] **Step 5: Save the task checkpoint**

If Git exists:

```powershell
git add manager/ZooidTestTargets.h manager/ZooidTestTargets.cpp tests/zooid_core_tests.cpp
git commit -m "feat: track immutable robot test targets"
```

---

### Task 4: Receiver write-error observability

**Files:**
- Modify: `manager/ZooidReceiver.h:1-220`
- Modify: `manager/ZooidReceiver.cpp:1-40, 200-245`

**Interfaces:**
- Consumes: the existing boolean result of `ZooidSerialPort::writeBytes`.
- Produces: `bool ZooidReceiver::consumeWriteFailure()`; it returns true once per observed failure and clears the latch.

- [ ] **Step 1: Add a compile-time interface check to the core test build**

Do not pull Qt SerialPort into the pure test executable. Instead, add a source-level verification command before implementation:

```powershell
rg -n "consumeWriteFailure|writeFailure" manager\ZooidReceiver.h manager\ZooidReceiver.cpp
```

Expected before implementation: no matches and exit code 1.

- [ ] **Step 2: Add an atomic failure latch**

In `ZooidReceiver.h`, include `<atomic>`, add:

```cpp
public:
    bool consumeWriteFailure();

private:
    std::atomic<bool> writeFailure{false};
```

In each existing `writeBytes(...)` failure branch inside `usbSendingRoutine()`, set `writeFailure.store(true)`. Implement consumption using `return writeFailure.exchange(false);`. Preserve the existing buffer retry/retention behavior; this task only exposes the failure.

- [ ] **Step 3: Verify all write sites and the public consumer**

```powershell
rg -n "writeBytes|writeFailure|consumeWriteFailure" manager\ZooidReceiver.h manager\ZooidReceiver.cpp
```

Expected: both variable-length and 63-byte write paths latch failure, and exactly one public consumer is declared and defined.

- [ ] **Step 4: Regenerate and compile the application**

```powershell
& 'D:\Qt5.9.7\5.9.7\mingw53_32\bin\qmake.exe' -o Makefile ZooidManager.pro
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\mingw32-make.exe' -f Makefile.Debug -j2
```

Expected: build succeeds with no new compiler errors.

- [ ] **Step 5: Save the task checkpoint**

If Git exists:

```powershell
git add manager/ZooidReceiver.h manager/ZooidReceiver.cpp
git commit -m "feat: expose receiver write failures"
```

---

### Task 5: ZooidManager test-session orchestration

**Files:**
- Modify: `ZooidManager.pro:31-87`
- Modify: `manager/ZooidManager.h:1-30, 46-60, 265-290, 440-470, 475-610`
- Modify: `manager/ZooidManager.cpp:1-40, 303-365, 1712-1720, 2744-2763, 2829-2910`

**Interfaces:**
- Consumes: `ZooidTestMode`, `ZooidSpeedCodec`, `ZooidTestTargets`, existing `Zooid` feedback state, receiver routing and `sendUSB` queue.
- Produces: `TestModeStatus ZooidManager::getTestModeStatus() const`, `std::vector<unsigned int> ZooidManager::getTestModeLostRobotIds() const`, `bool ZooidManager::startTestMode()`, and `void ZooidManager::stopTestMode()`.

- [ ] **Step 1: Add the new sources to qmake and verify the application fails before manager integration**

Add these exact entries:

```qmake
SOURCES += \
    manager/ZooidTestMode.cpp \
    manager/ZooidSpeedCodec.cpp \
    manager/ZooidTestTargets.cpp

HEADERS += \
    manager/ZooidTestMode.h \
    manager/ZooidSpeedCodec.h \
    manager/ZooidTestTargets.h
```

Regenerate/build. Expected at this checkpoint: pure files compile; no manager behavior exists yet.

- [ ] **Step 2: Define the thread-safe public status contract**

Add before `ZooidManager`:

```cpp
enum class TestModeStatus {
    Idle,
    StartPending,
    Running,
    Completed,
    Stopped,
    NoActiveRobots,
    AllTargetsLost,
    ReceiverError
};
```

Add public methods:

```cpp
bool startTestMode();
void stopTestMode();
TestModeStatus getTestModeStatus() const;
std::vector<unsigned int> getTestModeLostRobotIds() const;
```

Add private helpers with these signatures:

```cpp
uint64_t steadyNowMs() const;
std::vector<unsigned int> snapshotActiveZooidIds();
void serviceTestMode(uint64_t nowMs);
void sendTestCommand(const std::vector<unsigned int>& ids, WheelCommand command);
void flushReceiversForIds(const std::vector<unsigned int>& ids);
void beginSafeTestStop(TestModeStatus finalStatus);
void serviceZeroStopBurst();
bool anyReceiverWriteFailed();
```

Add `mutable std::mutex testModeMutex`, request flags, status, `ZooidTestMode`, `ZooidTestTargets`, final-status latch, and integer `zeroStopCyclesRemaining`. Initialize every field explicitly in the constructor.

- [ ] **Step 3: Implement request handling without touching robot collections from the UI thread**

`startTestMode()` locks `testModeMutex`, rejects `StartPending` and `Running`, sets `StartPending`, clears prior lost IDs/status, and returns true. It does not inspect `myZooids`.

`stopTestMode()` locks `testModeMutex` and sets a stop-request flag. `getTestModeStatus()` and `getTestModeLostRobotIds()` copy state under the same mutex.

The manager thread consumes `StartPending`, obtains the active-ID snapshot from `myZooids`, and either starts `ZooidTestMode` or changes status to `NoActiveRobots`. This keeps target collection and command dispatch on the manager side.

- [ ] **Step 4: Implement the exclusive manager-loop branch**

Replace `clock()` scheduling in `managerThreadRun()` with `steady_clock` milliseconds. Each loop must follow this order:

```cpp
processReceiversData();
const uint64_t nowMs = steadyNowMs();
serviceTestMode(nowMs);

if (test status is StartPending, Running, or a zero-stop burst is active) {
    // serviceTestMode owns all outgoing motion commands.
} else {
    // Phase-one test UI is idle: do not run or send legacy control orders.
}

Sleep(1);
```

Move `onlineZooidUpdate()` into the manager-thread update cadence so feedback registration and target snapshot happen in a consistent order. Remove its call from `managerTimerRun()`; retain `zooidPosUpdate()` for display, but skip `chargeUpdate()` and `updateVoronoi()` in the phase-one test runtime.

- [ ] **Step 5: Implement target retention and synchronized sending**

At start, call `startSnapshot(snapshotActiveZooidIds())`. On each update, call `retainActive(snapshotActiveZooidIds())`. If the set becomes empty, call `beginSafeTestStop(TestModeStatus::AllTargetsLost)`.

For a running state output, call `sendTestCommand(activeIds, output.wheels)` once per `SYSTEM_UPDATE_PERIOD`, then `flushReceiversForIds(activeIds)`. Deduplicate receiver IDs before calling `setReadyToSend()` so one USB receiver is awakened once per batch.

When a target is newly lost, queue a zero command for that ID only if `retrieveReceiver(id)` still returns an initialized receiver, then flush it. Preserve the ID in `lostIds()` for UI reporting.

- [ ] **Step 6: Make speed-message construction deterministic**

Change `controlRobotSpeed` to:

```cpp
const EncodedWheelSpeeds encoded = encodeWheelSpeeds(motor1, motor2);
PositionControlMessage msg{};
msg.positionX = encoded.positionX;
msg.positionY = encoded.positionY;
msg.colorRed = static_cast<uint8_t>(color.red());
msg.colorGreen = static_cast<uint8_t>(color.green());
msg.colorBlue = static_cast<uint8_t>(color.blue());
msg.preferredSpeed = static_cast<uint8_t>(ZOOID_RUN_SPEED);
msg.orientation = static_cast<int16_t>(52000);
msg.isFinalGoal = true;
msg.empty = planningMode == ChargePlanning ? 0xfe : 0xff;
msg.controlMode = 1;
```

`manager/Zooid.h` defines `PositionControl=0` and `SpeedControl=1`, so the on-wire `controlMode` byte is explicitly `1`. Add a source assertion or unit assertion for this value; do not serialize the compiler-sized enum object and do not leave the byte implicit.

- [ ] **Step 7: Implement one shared safe-stop path**

`beginSafeTestStop(finalStatus)` must stop the state machine, preserve the last target IDs, set `zeroStopCyclesRemaining = 3`, and block any new nonzero output. `serviceZeroStopBurst()` sends `{0,0}` to all still-routable snapshot IDs, flushes their receivers, decrements the counter, and only after the third batch clears targets and publishes `finalStatus`.

Natural completion maps to `Completed`, user request to `Stopped`, all targets gone to `AllTargetsLost`, and `anyReceiverWriteFailed()` to `ReceiverError`.

Change `stopAllZooids()` to use the same bounded encoding and explicitly flush all involved receivers.

- [ ] **Step 8: Make destruction stop-before-close**

At the start of `~ZooidManager()`, request a safe stop while `managerThread` and receivers are still alive. Wait on a dedicated condition variable for at most 250 ms for the three-cycle zero burst to finish. On timeout, synchronously queue one final zero batch to every still-routable Zooid and flush it. Only then set `updating = false`, join the manager thread, and delete receivers.

Guard `managerThread.join()` with `joinable()` so construction/init failure cannot terminate the process during cleanup.

- [ ] **Step 9: Run core tests and compile the full application**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp manager\ZooidSpeedCodec.cpp manager\ZooidTestTargets.cpp -o tests\zooid_core_tests.exe
& '.\tests\zooid_core_tests.exe'
& 'D:\Qt5.9.7\5.9.7\mingw53_32\bin\qmake.exe' -o Makefile ZooidManager.pro
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\mingw32-make.exe' -f Makefile.Debug -j2
```

Expected: core test exit code 0; full Debug application build succeeds.

- [ ] **Step 10: Audit exclusivity and all stop paths**

```powershell
rg -n "assignRobots\(|runSimulation\(|sendRobotsOrders\(|serviceTestMode|beginSafeTestStop|serviceZeroStopBurst|setReadyToSend" manager\ZooidManager.cpp
```

Expected: legacy calls are absent from the active manager-loop path; all test termination causes call `beginSafeTestStop`; every command batch has a receiver flush.

- [ ] **Step 11: Save the task checkpoint**

If Git exists:

```powershell
git add ZooidManager.pro manager/ZooidManager.h manager/ZooidManager.cpp manager/ZooidTestMode.h manager/ZooidTestMode.cpp manager/ZooidSpeedCodec.h manager/ZooidSpeedCodec.cpp manager/ZooidTestTargets.h manager/ZooidTestTargets.cpp tests/zooid_core_tests.cpp
git commit -m "feat: integrate synchronized hardware test mode"
```

---

### Task 6: Two-button HomePage

**Files:**
- Modify: `homePage.h:80-160, 185-205`
- Modify: `homePage.cpp:3-25, 59-245, 485-515`

**Interfaces:**
- Consumes: `ZooidManager::startTestMode()`, `stopTestMode()`, `getTestModeStatus()`, and `getTestModeLostRobotIds()`.
- Produces: `HomePage::startHardwareTest()`, simplified `stopAllModes()`, and `updateTestModeUi()`.

- [ ] **Step 1: Add the test-mode UI declarations**

Add:

```cpp
private slots:
    void startHardwareTest();
    void stopAllModes();

private:
    void updateTestModeUi();
    QPushButton* testModeBtn = nullptr;
    QPushButton* stopBtn = nullptr;
    QLabel* testModeStatusLabel = nullptr;
```

Initialize every retained legacy widget pointer to `nullptr` in the header. This makes deletion safe if old buttons are no longer constructed.

- [ ] **Step 2: Replace the visible mode controls with test and stop**

In `initGUI()`, retain the simulator panel and admin access, but populate `selectPlanBtnsLayout` only with:

```cpp
testModeBtn = new QPushButton(QStringLiteral("开始测试"), goalPlanPanel);
testModeBtn->setFixedHeight(120);
testModeBtn->setStyleSheet(mainbuttonstyle);

stopBtn = new QPushButton(QStringLiteral("停止"), goalPlanPanel);
stopBtn->setFixedHeight(120);
stopBtn->setStyleSheet(
    "QPushButton{background-color:#cc3333;border-radius:5px;color:white;"
    "font-size:24px;font-weight:bold;}"
    "QPushButton:hover{background-color:#ee4444;}");

testModeStatusLabel = new QLabel(QStringLiteral("等待机器人上线"), goalPlanPanel);
testModeStatusLabel->setWordWrap(true);
testModeStatusLabel->setStyleSheet(m_12 + transparent_bg + font_size_sm + text_white);

selectPlanBtnsLayout->addWidget(testModeBtn);
selectPlanBtnsLayout->addWidget(stopBtn);
selectPlanBtnsLayout->addWidget(testModeStatusLabel);
selectPlanBtnsLayout->addStretch();
```

Do not construct or connect graphical, draw, follow, charge, or Voronoi mode buttons. Keep their handler source methods compiled but unreachable from this page.

- [ ] **Step 3: Connect the two controls**

```cpp
connect(testModeBtn, &QPushButton::clicked,
        this, &HomePage::startHardwareTest);
connect(stopBtn, &QPushButton::clicked,
        this, &HomePage::stopAllModes);
```

`startHardwareTest()` calls `startTestMode()` once. If it returns false, display `测试已在启动或运行中`.

Simplify `stopAllModes()` to call `zooidManager.stopTestMode()` and immediately show `正在安全停止…`. Do not call `stopVoronoiMode()` because its controller/window may be irrelevant to the test-only interface; old source remains available.

- [ ] **Step 4: Map manager state to the status label**

Call `updateTestModeUi()` from the existing 100 ms `dataUpdate()` timer. Use this exact mapping:

```text
Idle            -> 等待开始测试
StartPending    -> 正在检查在线机器人…
Running         -> 测试运行中
Completed       -> 测试完成，机器人已停止
Stopped         -> 已手动停止
NoActiveRobots  -> 没有在线且已激活的机器人
AllTargetsLost  -> 所有测试机器人均已掉线，请现场确认
ReceiverError   -> 接收器发送失败，已执行安全停止
```

Append `掉线 ID: 5, 12` when `getTestModeLostRobotIds()` is nonempty. Disable `testModeBtn` only for `StartPending` and `Running`; keep the stop button enabled at all times.

- [ ] **Step 5: Remove unsafe manual child deletion**

Qt parent ownership deletes child buttons, labels, and `admin`（它由 `new AdminPage(this)` 创建）. In `~HomePage()`, stop and delete only the parentless `updateTimer`. Remove manual deletion of individual mode buttons and `admin` to avoid duplicate ownership cleanup.

- [ ] **Step 6: Compile the application**

```powershell
& 'D:\Qt5.9.7\5.9.7\mingw53_32\bin\qmake.exe' -o Makefile ZooidManager.pro
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\mingw32-make.exe' -f Makefile.Debug -j2
```

Expected: build succeeds; no unresolved old-button references remain. If retained legacy handlers still reference old members, leave the initialized null members declared until a separate cleanup change removes those handlers together.

- [ ] **Step 7: Static UI entry-point audit**

```powershell
rg -n "addWidget\(|connect\(.*clicked|new QPushButton" homePage.cpp
```

Expected in the main control layout: only `开始测试` and `停止` are connected as robot-control actions; admin navigation is allowed.

- [ ] **Step 8: Save the task checkpoint**

If Git exists:

```powershell
git add homePage.h homePage.cpp
git commit -m "feat: expose test-only robot controls"
```

---

### Task 7: Safety documentation and final verification

**Files:**
- Create: `docs/test-mode-operation.md`
- Verify: all files changed in Tasks 1-6

**Interfaces:**
- Consumes: the completed test-mode UI and serial-control behavior.
- Produces: a repeatable operator checklist and verification evidence.

- [ ] **Step 1: Write the operator safety guide**

Document these exact points:

```markdown
# 多机器人测试模式操作

1. 第一次测试时架空所有机器人轮子。
2. 确认 USB 接收器已连接，界面能看到机器人反馈。
3. 点击“开始测试”只执行一轮：前进、左转、右转、后退。
4. 任意时刻可点击“停止”；出现接收器错误或掉线提示时现场确认机器人已经停止。
5. 单机确认左右轮方向后，再依次测试两台和全部机器人。
6. 全部落地测试必须留出足够空间，并保持物理断电手段可达。
```

Include the command table and explain that speed zero is encoded as `2007`, not literal zero in `positionX/positionY`.

- [ ] **Step 2: Run the pure regression suite**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidTestMode.cpp manager\ZooidSpeedCodec.cpp manager\ZooidTestTargets.cpp -o tests\zooid_core_tests.exe
& '.\tests\zooid_core_tests.exe'
```

Expected: exit code 0 and `zooid core tests passed`.

- [ ] **Step 3: Perform a clean qmake Debug rebuild**

Use a dedicated build directory so generated files do not hide stale-object problems:

```powershell
New-Item -ItemType Directory -Force -Path '.build-test-mode' | Out-Null
Push-Location '.build-test-mode'
& 'D:\Qt5.9.7\5.9.7\mingw53_32\bin\qmake.exe' '..\ZooidManager.pro'
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\mingw32-make.exe' -j2 debug
Pop-Location
```

Expected: a successful Debug executable build with no compile or link errors.

- [ ] **Step 4: Perform protocol and exclusivity source audits**

```powershell
rg -n "PositionControlMessage msg\{\}|controlMode|encodeWheelSpeeds|2007|setReadyToSend" manager\ZooidManager.cpp manager\ZooidSpeedCodec.cpp
rg -n "serviceTestMode|assignRobots\(|runSimulation\(|sendRobotsOrders\(" manager\ZooidManager.cpp
rg -n "开始测试|停止|Voronoi|Follow|Graphical|Draw" homePage.cpp
```

Expected: zero-initialized message and explicit control mode are present; test branch is exclusive; legacy mode names do not appear in visible/connected control creation.

- [ ] **Step 5: Run staged hardware acceptance**

Execute and record each result:

```text
[ ] One robot with wheels lifted: command directions match the table.
[ ] One robot on the floor: Stop works during each nonzero phase.
[ ] Two robots: both move synchronously and retain separate IDs.
[ ] All robots: all startup-active IDs run one cycle; late joiners stay still.
[ ] Disconnect one target: remaining targets continue; lost ID is displayed.
[ ] Disconnect/fail a receiver: test stops and ReceiverError is displayed.
[ ] Close the program during motion: routable robots receive zero before serial close.
```

Do not mark hardware acceptance complete from compilation alone.

- [ ] **Step 6: Save the final checkpoint**

If Git exists:

```powershell
git add docs/test-mode-operation.md
git commit -m "docs: add robot test mode safety procedure"
git status --short
```

Expected: all intended source and documentation changes are committed and no generated `.build-test-mode` or `tests/zooid_core_tests.exe` artifact is staged. Without Git, list changed source files and retain the full command output as the final implementation record.
