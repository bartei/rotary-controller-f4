/*
 * Lathe Emulator - Main entry point.
 *
 * Initializes the firmware data structures, starts the physics model,
 * transport layer, and ISR thread, then runs the dashboard.
 */

#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cmath>

#include "config.h"
#include "physics.h"
#include "transport.h"
#include "dashboard.h"

extern "C" {
#include "emulator_state.h"
#include "stm32f4xx_hal.h"
#include "Ramps.h"

/* Firmware externs */
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim9, htim11;
extern UART_HandleTypeDef huart1;
extern void emu_update_timer_counters(void);
}

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
    (void)sig;
    g_running.store(false);
}

/*
 * ISR thread: ticks the physics model and calls the firmware's
 * SynchroRefreshTimerIsr at the configured rate.
 */
static void isrThreadFunc(LathePhysics *physics, rampsHandler_t *rampsData, int rate_hz) {
    auto interval = std::chrono::microseconds(1000000 / rate_hz);
    auto next_tick = std::chrono::steady_clock::now();

    /* Simulate the DWT cycle counter. At "100 MHz", each ISR tick at 10kHz = 10000 cycles. */
    uint32_t cycles_per_tick = 100000000U / (uint32_t)rate_hz;

    /* Track previous step pin state for edge detection */
    int prev_step_pin = 0;

    while (g_running.load()) {
        next_tick += interval;

        /* Advance DWT cycle counter to simulate the inter-ISR interval */
        emu_hw.dwt_cyccnt += cycles_per_tick;
        emu_dwt.CYCCNT = emu_hw.dwt_cyccnt;

        /* Advance physics */
        double dt = 1.0 / (double)rate_hz;
        physics->tick(dt, &rampsData->shared);

        /* Call firmware ISR.
         * The ISR reads DWT->CYCCNT at start and end to measure execution
         * time, but since emu_dwt.CYCCNT is a plain variable both reads
         * return the same value → executionCycles=0. We measure the real
         * wall-clock execution time and patch the result after. */
        SynchroRefreshTimerIsr(rampsData);

        /* Overwrite executionCycles with a realistic estimate.
         * The real ISR takes ~5-10µs on the MCU = 500-1000 cycles at 100MHz.
         * Use a fixed reasonable value since we can't measure sub-µs on host. */
        rampsData->shared.executionCycles = 600;

        /* Detect STEP rising edge and feed into physics */
        if (emu_hw.step_pin && !prev_step_pin) {
            int dir = emu_hw.dir_pin ? 1 : -1;
            physics->onStepPulse(dir);
        }
        prev_step_pin = emu_hw.step_pin;

        /* Increment HAL tick (~1ms per tick, approximate) */
        static int tick_divider = 0;
        tick_divider++;
        if (tick_divider >= rate_hz / 1000) {
            tick_divider = 0;
            HAL_IncTick();
        }

        /* Sleep until next tick */
        std::this_thread::sleep_until(next_tick);
    }
}

int main(int argc, char *argv[]) {
    printf("=== Lathe Emulator ===\n");

    /* Load configuration */
    EmuConfig cfg;
    std::string config_path = "config/lathe.toml";
    if (argc > 1) config_path = argv[1];
    loadConfig(config_path, cfg);

    /* Initialize emulator hardware state */
    memset(&emu_hw, 0, sizeof(emu_hw));
    pthread_mutex_init(&emu_hw.mutex, nullptr);

    /* Set TC flag so Modbus sendTxBuffer busy-wait completes immediately */
    extern USART_TypeDef emu_usart1;
    emu_usart1.SR = USART_SR_TC;

    /* Initialize physics model */
    LathePhysics physics(cfg);

    /* Initialize the firmware's rampsHandler_t struct */
    static rampsHandler_t rampsData;
    memset(&rampsData, 0, sizeof(rampsData));

    /* Wire timer handles to scales (same mapping as firmware's main.c) */
    extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
    rampsData.scaleTimers[0] = &htim1;
    rampsData.scaleTimers[1] = &htim2;
    rampsData.scaleTimers[2] = &htim3;
    rampsData.scaleTimers[3] = &htim4;

    /* Set synchro timer and UART handles */
    rampsData.synchroRefreshTimer = &htim9;
    rampsData.modbusUart = &huart1;

    /* Adjust htim9 prescaler so the firmware's clock_freq calculation yields
     * the emulator's actual ISR rate.  On real hardware the timer runs at
     * 100 kHz (100 MHz / (99+1) / (9+1)); the firmware derives the maximum
     * step-pulse rate from that via servoCycles = clock_freq / maxSpeed.
     * In the emulator the ISR fires at cfg.isr_rate_hz, so we must match:
     *   100 MHz / (Prescaler+1) / (Period+1) == isr_rate_hz              */
    htim9.Init.Prescaler = 100000000U / ((uint32_t)cfg.isr_rate_hz * (htim9.Init.Period + 1)) - 1;

    /* Apply config defaults to servo */
    rampsData.shared.servo.maxSpeed = (float)cfg.servo_max_speed;
    rampsData.shared.servo.acceleration = (float)cfg.servo_acceleration;

    /* Default sync ratios */
    for (int i = 0; i < SCALES_COUNT; i++) {
        rampsData.shared.scales[i].syncRatioNum = 1;
        rampsData.shared.scales[i].syncRatioDen = 100;
    }

    /* Initialize transport */
    Transport transport(cfg);
    transport.start();

    /* Initialize the firmware (Modbus, tasks, etc.) */
    printf("Starting firmware initialization...\n");
    RampsStart(&rampsData);
    printf("Firmware initialized.\n");

    /* Initialize servoCycles to avoid division by zero on first ISR tick.
     * On real hardware updateSpeedTask() sets this within 50ms of boot,
     * but the emulator's ISR thread starts immediately.
     * Formula from updateSpeedTask: clock_freq / maxSpeed
     * clock_freq = 100MHz / (Prescaler+1) / (Period+1) = 100000
     */
    {
        extern uint16_t servoCycles;
        float clock_freq = 100000000.0f /
            ((float)rampsData.synchroRefreshTimer->Init.Prescaler + 1) /
            (float)(rampsData.synchroRefreshTimer->Init.Period + 1);
        float period = (cfg.servo_max_speed > 0) ? floorf(clock_freq / (float)cfg.servo_max_speed) : 138.0f;
        if (period < 1.0f) period = 1.0f;
        if (period > 65535.0f) period = 65535.0f;
        servoCycles = (uint16_t)period;
    }

    /* Install signal handler */
    signal(SIGINT, signalHandler);

    /* Start ISR thread */
    std::thread isrThread(isrThreadFunc, &physics, &rampsData, cfg.isr_rate_hz);

    /* Run dashboard on main thread */
    Dashboard dashboard(cfg, physics, transport, rampsData.shared);

    emu_log_event("Emulator started");
    emu_log_event("PTY: %s", transport.getPtyPath().c_str());
    if (cfg.tcp_enabled) emu_log_event("TCP: port %d", cfg.tcp_port);

    dashboard.run();

    /* Shutdown */
    g_running.store(false);
    if (isrThread.joinable()) isrThread.join();
    transport.stop();

    printf("Emulator stopped.\n");
    return 0;
}
