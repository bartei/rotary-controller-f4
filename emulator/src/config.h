/*
 * Emulator configuration loaded from TOML file.
 */
#ifndef EMU_CONFIG_H
#define EMU_CONFIG_H

#include <string>

struct EmuConfig {
    /* [display] */
    bool imperial;  /* false=metric, true=imperial */

    /* [spindle] */
    int    spindle_counts_per_rev;
    double spindle_inertia;
    double spindle_max_torque;
    double spindle_friction;
    double spindle_initial_rpm;

    /* [leadscrew] */
    double leadscrew_tpi;
    double leadscrew_mm_per_step;

    /* [z_axis] */
    double z_encoder_counts_per_mm;
    double z_backlash_mm;
    double z_max_mm;
    double z_min_mm;
    double z_initial_mm;
    bool   z_half_nut_engaged;

    /* [cross_slide] */
    double x_encoder_counts_per_mm;
    double x_max_mm;
    double x_min_mm;
    double x_initial_mm;
    double x_manual_step_mm;
    double jog_max_velocity_mm_s;  /* max manual jog speed (mm/s) */
    double jog_acceleration_mm_s2; /* manual jog acceleration (mm/s^2) */
    bool   x_up_is_negative;      /* true: Up arrow = -X (away from operator) */
    double manual_move_timeout_s;  /* seconds of inactivity before auto-disabling manual move; 0 = no timeout (pure toggle) */

    /* [servo] */
    double servo_max_speed;
    double servo_acceleration;

    /* [modbus] */
    int    modbus_address;
    int    modbus_baud;

    /* [transport] */
    bool   pty_enabled;
    int    tcp_port;
    bool   tcp_enabled;

    /* [dashboard] */
    int    refresh_hz;
    int    sparkline_seconds;
    int    log_max_lines;

    /* [simulation] */
    int    isr_rate_hz;
    bool   realtime;

    /* Set defaults */
    EmuConfig();
};

/* Load config from file. Returns true on success. */
bool loadConfig(const std::string &path, EmuConfig &cfg);

#endif /* EMU_CONFIG_H */
