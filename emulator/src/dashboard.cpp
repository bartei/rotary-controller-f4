/*
 * Dashboard implementation: two-pane ANSI terminal with sparklines.
 */

#include "dashboard.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

extern "C" {
#include "emulator_state.h"
}

/* ANSI escape helpers */
#define ESC "\033"
#define CSI ESC "["
#define CLEAR_SCREEN CSI "2J"
#define CURSOR_HOME  CSI "H"
#define HIDE_CURSOR  CSI "?25l"
#define SHOW_CURSOR  CSI "?25h"
#define WRAP_OFF     CSI "?7l"
#define WRAP_ON      CSI "?7h"
#define BOLD         CSI "1m"
#define DIM          CSI "2m"
#define RESET_ATTR   CSI "0m"
#define FG_GREEN     CSI "32m"
#define FG_YELLOW    CSI "33m"
#define FG_CYAN      CSI "36m"
#define FG_RED       CSI "31m"
#define FG_WHITE     CSI "37m"

static void moveTo(int row, int col) {
    printf(CSI "%d;%dH", row, col);
}

/* Clear from cursor to end of line (preserves content left of cursor) */
static void clearToEol() {
    printf(CSI "0K");
}

/* Unicode block characters for sparklines (8 levels) */
static const char *spark_chars[] = {
    "\u2581", "\u2582", "\u2583", "\u2584",
    "\u2585", "\u2586", "\u2587", "\u2588"
};

std::string Dashboard::SparklineBuffer::render() const {
    if (samples.empty()) return std::string(width, ' ');

    /* Take the last `width` samples (or fewer if not enough) */
    int n = std::min((int)samples.size(), width);
    int start = (int)samples.size() - n;

    /* Find min/max for auto-scaling */
    double mn = samples[start], mx = samples[start];
    for (int i = start; i < (int)samples.size(); i++) {
        mn = std::min(mn, samples[i]);
        mx = std::max(mx, samples[i]);
    }
    if (mx - mn < 0.001) mx = mn + 1.0;

    std::string result;
    /* Pad with spaces if fewer samples than width */
    for (int i = 0; i < width - n; i++) result += " ";

    for (int i = start; i < (int)samples.size(); i++) {
        int level = (int)((samples[i] - mn) / (mx - mn) * 7.0);
        level = std::max(0, std::min(7, level));
        result += spark_chars[level];
    }
    return result;
}

Dashboard::Dashboard(const EmuConfig &cfg, LathePhysics &physics, Transport &transport,
                     rampsSharedData_t &shared)
    : cfg(cfg), physics(physics), transport(transport), shared(shared),
      running(false), manual_move(false), manual_move_timer(0.0), manual_move_used(false),
      spark_rpm(cfg.sparkline_seconds, cfg.refresh_hz, 30),
      spark_zpos(cfg.sparkline_seconds, cfg.refresh_hz, 30),
      spark_zerr(cfg.sparkline_seconds, cfg.refresh_hz, 30)
{
}

double Dashboard::toDisplay(double mm) const {
    return cfg.imperial ? mm / 25.4 : mm;
}

const char* Dashboard::unitSuffix() const {
    return cfg.imperial ? "in" : "mm";
}

int Dashboard::unitPrecision() const {
    return cfg.imperial ? 4 : 3;
}

void Dashboard::run() {
    running.store(true);

    /* Put terminal in raw mode */
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    printf(HIDE_CURSOR);
    printf(WRAP_OFF);
    printf(CLEAR_SCREEN);

    int interval_us = 1000000 / cfg.refresh_hz;

    while (running.load()) {
        /* Update sparklines */
        spark_rpm.push(std::abs(physics.getSpindleRPM()));
        spark_zpos.push(physics.getCarriageMM());
        double err = (double)(int32_t)(shared.servo.desiredSteps - shared.servo.currentSteps);
        spark_zerr.push(std::abs(err));

        draw();
        handleInput();
        usleep(interval_us);
    }

    printf(WRAP_ON);
    printf(SHOW_CURSOR);
    printf(CLEAR_SCREEN);
    printf(CURSOR_HOME);

    /* Restore terminal */
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

void Dashboard::draw() {
    /* Get terminal size */
    struct winsize ws;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
    int term_w = ws.ws_col > 0 ? ws.ws_col : 100;
    int term_h = ws.ws_row > 0 ? ws.ws_row : 30;

    int left_w = 40;
    int right_w = term_w - left_w - 3;
    if (right_w < 20) right_w = 20;

    printf(CURSOR_HOME);

    drawStatePane(1, 1, left_w);
    drawLogPane(1, left_w + 2, right_w, term_h - 2);
    drawStatusBar(term_h, term_w);

    fflush(stdout);
}

void Dashboard::drawStatePane(int startRow, int startCol, int width) {
    int row = startRow;
    /* Helper macro for printing a line at the current row.
     * Can't use variadic lambda with va_start, so use a macro. */
    #define LINE(...) do { moveTo(row++, startCol); clearToEol(); printf(__VA_ARGS__); } while(0)

    double rpm = physics.getSpindleRPM();
    double target = physics.getTargetRPM();
    const char *dir = physics.getSpindleCW() ? "CW" : "CCW";
    const char *arrow = std::abs(rpm) > 0.1 ? "\xe2\x96\xb6" : " ";

    LINE(BOLD " SPINDLE  " RESET_ATTR FG_GREEN "%.0f RPM" RESET_ATTR " %s %s  [target: %.0f]",
         std::abs(rpm), arrow, dir, std::abs(target));

    LINE(" RPM  %s", spark_rpm.render().c_str());

    /* Use the firmware's accumulated encoder position (what Modbus clients see).
     * Note: the "phase" display aliases at low dashboard refresh rates — at
     * 600 RPM / 4000 CPR / 10 Hz refresh, each frame spans exactly one rev
     * so the phase appears frozen. This is real sampling aliasing, not a bug. */
    int32_t enc = shared.scales[0].position;
    int phase = 0;
    if (cfg.spindle_counts_per_rev > 0) {
        phase = (int)(enc % cfg.spindle_counts_per_rev);
        if (phase < 0) phase += cfg.spindle_counts_per_rev;
    }
    LINE(" encoder: %d  phase: %d/%d", enc, phase, cfg.spindle_counts_per_rev);

    LINE("%s", "");

    double z = toDisplay(physics.getCarriageMM());
    LINE(BOLD " Z-AXIS" RESET_ATTR "   %.*f %s   steps: %u",
         unitPrecision(), z, unitSuffix(), shared.servo.currentSteps);
    LINE(" pos  %s", spark_zpos.render().c_str());
    LINE(" " FG_CYAN "\xce\x94" "err" RESET_ATTR " %s", spark_zerr.render().c_str());

    const char *hn_str = "DISENGAGED";
    if (physics.getHalfNutState() == LathePhysics::ENGAGED)
        hn_str = "ENGAGED";
    else if (physics.getHalfNutState() == LathePhysics::ENGAGING)
        hn_str = FG_YELLOW "ENGAGING..." RESET_ATTR;

    LINE(" half-nut: %s   backlash: %.*f", hn_str, unitPrecision(), toDisplay(0.0));

    const char *mode_str = "OFF";
    if (shared.fastData.servoMode == 1) mode_str = "SYNC";
    else if (shared.fastData.servoMode == 2) mode_str = "JOG";

    LINE(" mode: %s  speed: %.0f stp/s", mode_str, shared.fastData.servoSpeed);

    const char *dir_str = (emu_hw.dir_pin) ? "FWD" : "REV";
    const char *ena_str = (emu_hw.ena_pin == 0) ? "ON" : "OFF";
    LINE(" dir: %s  enable: %s  stepsToGo: %d", dir_str, ena_str, shared.servo.stepsToGo);

    LINE("%s", "");

    double xpos = toDisplay(physics.getCrossSlideMM());
    LINE(BOLD " CROSS-SLIDE" RESET_ATTR "  %.*f %s", unitPrecision(), xpos, unitSuffix());

    LINE("%s", "");

    /* Scales */
    LINE(BOLD " SCALES" RESET_ATTR);
    const char *scale_names[] = { "spindle", "z-axis ", "x-slide", "spare  " };
    for (int i = 0; i < 4; i++) {
        const char *en = shared.scales[i].syncEnable ? FG_GREEN "ON " RESET_ATTR : DIM "off" RESET_ATTR;
        LINE(" [%d] %s  %d/%d %s  pos: %d",
             i, scale_names[i],
             shared.scales[i].syncRatioNum, shared.scales[i].syncRatioDen,
             en, shared.scales[i].position);
    }

    LINE("%s", "");

    LINE(" MODBUS  rx: %u  tx: %u  err: %u",
         (unsigned)shared.fastData.cycles,
         0u, 0u);
    LINE(" %.1f" "\xc2\xb5" "s avg  tick: %uk",
         (double)shared.executionCycles / 100.0,
         (unsigned)(emu_hw.dwt_cyccnt / 1000));

    std::string pty_str = transport.getPtyPath().empty() ? "none" : transport.getPtyPath();
    LINE(" PTY: %s  TCP:%d: %d client",
         pty_str.c_str(), cfg.tcp_port, transport.getTcpClientCount());

    #undef LINE
}

void Dashboard::drawLogPane(int startRow, int startCol, int width, int height) {
    /* Header */
    moveTo(startRow, startCol);
    printf(BOLD DIM "EVENT LOG" RESET_ATTR);

    int maxLines = height - 2;
    int count = emu_hw.event_log_count;
    int start = 0;
    if (count > maxLines) start = count - maxLines;

    for (int i = 0; i < maxLines; i++) {
        moveTo(startRow + 1 + i, startCol);
        clearToEol();
        int idx = start + i;
        if (idx < count) {
            int ring_idx = (emu_hw.event_log_head - count + idx) % 256;
            if (ring_idx < 0) ring_idx += 256;
            printf(DIM "%.*s" RESET_ATTR, width, emu_hw.event_log[ring_idx]);
        }
    }
}

void Dashboard::drawStatusBar(int row, int width) {
    moveTo(row, 1);
    printf(CSI "2K");  /* Clear entire line — status bar owns the full row */

    /* Auto-disable manual move after timeout (if configured) */
    if (manual_move && manual_move_used) {
        manual_move_timer += 1.0 / cfg.refresh_hz;
        double timeout = cfg.manual_move_timeout_s;
        if (timeout > 0.0 && manual_move_timer > timeout
            && !physics.isZJogging() && !physics.isXJogging()
            && !physics.isZMoveTargetActive() && !physics.isXMoveTargetActive()) {
            manual_move = false;
            manual_move_used = false;
            emu_log_event("manual move disabled (timeout)");
        }
    }

    const char *move_indicator = "";
    if (manual_move && (physics.isZJogging() || physics.isXJogging()))
        move_indicator = FG_CYAN "[MANUAL active]" RESET_ATTR " ";
    else if (manual_move)
        move_indicator = FG_YELLOW "[MANUAL]" RESET_ATTR " ";

    const char *start_stop = (std::abs(physics.getTargetRPM()) > 0.1) ? "s[T]op" : "s[T]art";

    /* Show Z/X position entry keys only when manual move is enabled */
    const char *zx_keys = "";
    if (manual_move) {
        bool z_allowed = (physics.getHalfNutState() != LathePhysics::ENGAGED);
        zx_keys = z_allowed ? "  [Z]pos [X]pos  arrows" : "  [X]pos  arrows";
    }

    printf("%s" DIM "[S]pindle RPM  [D]ir  [H]alf-nut  [M]anual%s  [E]-stop  %s  [Q]uit" RESET_ATTR,
           move_indicator, zx_keys, start_stop);
}

void Dashboard::handleInput() {
    /* Drain all available input to avoid backlog from key repeat.
     * Deduplicate: for toggle actions (H, T, D) only process the first
     * occurrence in the buffer to avoid key-repeat undoing the action. */
    char buf[64];
    ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return;

    bool had_h = false, had_t = false, had_d = false, had_e = false, had_m = false;
    int last_arrow_z = 0;
    int last_arrow_x = 0;

    for (ssize_t i = 0; i < n; i++) {
        char c = buf[i];

        if (c == 27 && i + 2 < n && buf[i+1] == '[') {
            char arrow = buf[i+2];
            i += 2;

            if (manual_move) {
                /* Left/Right = Z-axis, Up/Down = X-axis */
                int x_sign = cfg.x_up_is_negative ? -1 : 1;
                if (arrow == 'C') last_arrow_z = +1;
                else if (arrow == 'D') last_arrow_z = -1;
                else if (arrow == 'A') last_arrow_x = x_sign;   /* Up */
                else if (arrow == 'B') last_arrow_x = -x_sign;  /* Down */
                manual_move_timer = 0.0;
                manual_move_used = true;
            }
            continue;
        }

        switch (c) {
            case 'q': case 'Q':
                running.store(false);
                return;

            case 's': case 'S':
                promptSpindleRPM();
                return;

            case 'd': case 'D':
                if (!had_d) { had_d = true; physics.toggleDirection(); emu_log_event("spindle direction toggled"); }
                break;

            case 't': case 'T':
                if (!had_t) {
                    had_t = true;
                    if (std::abs(physics.getTargetRPM()) < 0.1) {
                        double rpm = physics.getSpindleCW() ? 500.0 : -500.0;
                        physics.setTargetRPM(rpm);
                        emu_log_event("spindle target -> %.0f RPM", rpm);
                    } else {
                        physics.setTargetRPM(0.0);
                        emu_log_event("spindle target -> 0 RPM");
                    }
                }
                break;

            case 'h': case 'H':
                if (!had_h) { had_h = true; physics.requestHalfNutToggle(); }
                break;

            case 'e': case 'E':
                if (!had_e) { had_e = true; physics.emergencyStop(); }
                break;

            case 'm': case 'M':
                if (!had_m) {
                    had_m = true;
                    manual_move = !manual_move;
                    manual_move_timer = 0.0;
                    manual_move_used = false;
                    emu_log_event("manual move %s", manual_move ? "enabled" : "disabled");
                }
                break;

            case 'z': case 'Z':
                if (manual_move && physics.getHalfNutState() != LathePhysics::ENGAGED) {
                    promptZPosition();
                    return;
                }
                break;

            case 'x': case 'X':
                if (manual_move) {
                    promptXPosition();
                    return;
                }
                break;

            default:
                break;
        }
    }

    /* Apply jog: one call per axis with the last direction seen */
    if (last_arrow_z != 0) physics.jogCarriage(last_arrow_z);
    if (last_arrow_x != 0) physics.jogCrossSlide(last_arrow_x);
}

void Dashboard::promptSpindleRPM() {
    /* Temporarily restore terminal for line input */
    printf(SHOW_CURSOR);
    struct winsize ws;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
    int row = ws.ws_row > 0 ? ws.ws_row : 30;
    moveTo(row, 1);
    clearToEol();
    printf("Enter spindle RPM: ");
    fflush(stdout);

    /* Switch to line mode briefly */
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    char buf[32];
    if (fgets(buf, sizeof(buf), stdin)) {
        double rpm = atof(buf);
        physics.setTargetRPM(rpm);
        emu_log_event("spindle target -> %.0f RPM", rpm);
    }

    /* Back to raw mode */
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    printf(HIDE_CURSOR);

    /* Force immediate full redraw to clear the prompt.
     * CLEAR_SCREEN needed because Enter may have scrolled the terminal. */
    printf(CLEAR_SCREEN);
    draw();
}

void Dashboard::promptZPosition() {
    printf(SHOW_CURSOR);
    struct winsize ws;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
    int row = ws.ws_row > 0 ? ws.ws_row : 30;
    moveTo(row, 1);
    clearToEol();
    printf("Enter Z position (%s), current %.3f: ",
           unitSuffix(), toDisplay(physics.getCarriageMM()));
    fflush(stdout);

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    char buf[32];
    if (fgets(buf, sizeof(buf), stdin) && buf[0] != '\n') {
        double pos = atof(buf);
        /* Convert from display units to mm */
        double pos_mm = cfg.imperial ? pos * 25.4 : pos;
        physics.moveCarriageTo(pos_mm);
        emu_log_event("Z move to %.3f %s", pos, unitSuffix());
        manual_move_timer = 0.0;
        manual_move_used = true;
    }

    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    printf(HIDE_CURSOR);
    /* Clear the prompt area (Enter may have scrolled the terminal) */
    printf(CLEAR_SCREEN);
    draw();
}

void Dashboard::promptXPosition() {
    printf(SHOW_CURSOR);
    struct winsize ws;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
    int row = ws.ws_row > 0 ? ws.ws_row : 30;
    moveTo(row, 1);
    clearToEol();
    printf("Enter X position (%s), current %.3f: ",
           unitSuffix(), toDisplay(physics.getCrossSlideMM()));
    fflush(stdout);

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    char buf[32];
    if (fgets(buf, sizeof(buf), stdin) && buf[0] != '\n') {
        double pos = atof(buf);
        double pos_mm = cfg.imperial ? pos * 25.4 : pos;
        physics.moveCrossSlideTo(pos_mm);
        emu_log_event("X move to %.3f %s", pos, unitSuffix());
        manual_move_timer = 0.0;
        manual_move_used = true;
    }

    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    printf(HIDE_CURSOR);
    printf(CLEAR_SCREEN);
    draw();
}
