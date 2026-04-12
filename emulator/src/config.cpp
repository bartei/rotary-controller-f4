/*
 * Simple TOML config parser.
 * Only handles flat [section] with key = value lines.
 * Values: strings ("..."), integers, floats, booleans (true/false).
 */

#include "config.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <algorithm>
#include <cctype>

EmuConfig::EmuConfig() {
    imperial = false;

    spindle_counts_per_rev = 4000;
    spindle_inertia = 0.05;
    spindle_max_torque = 2.0;
    spindle_friction = 0.1;
    spindle_initial_rpm = 0.0;

    leadscrew_tpi = 8.0;
    leadscrew_mm_per_step = 0.0025;

    z_encoder_counts_per_mm = 400.0;
    z_backlash_mm = 0.02;
    z_max_mm = 300.0;
    z_min_mm = -5.0;
    z_initial_mm = 0.0;
    z_half_nut_engaged = false;

    x_encoder_counts_per_mm = 400.0;
    x_max_mm = 100.0;
    x_min_mm = -5.0;
    x_initial_mm = 50.0;
    x_manual_step_mm = 0.01;
    jog_max_velocity_mm_s = 10.0;
    jog_acceleration_mm_s2 = 50.0;
    x_up_is_negative = true;
    manual_move_timeout_s = 2.0;

    servo_max_speed = 720;
    servo_acceleration = 120;

    modbus_address = 17;
    modbus_baud = 115200;

    pty_enabled = true;
    tcp_port = 5020;
    tcp_enabled = true;

    refresh_hz = 10;
    sparkline_seconds = 30;
    log_max_lines = 200;

    isr_rate_hz = 10000;
    realtime = true;
}

static std::string trim(const std::string &s) {
    auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

bool loadConfig(const std::string &path, EmuConfig &cfg) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Config: cannot open " << path << ", using defaults\n";
        return false;
    }

    std::string section;
    std::string line;
    /* section.key → value string */
    std::map<std::string, std::string> kv;

    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        if (line[0] == '[') {
            auto end = line.find(']');
            if (end != std::string::npos)
                section = line.substr(1, end - 1);
            continue;
        }

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));

        /* Strip inline comments */
        auto hashPos = val.find('#');
        if (hashPos != std::string::npos && (val[0] != '"')) {
            val = trim(val.substr(0, hashPos));
        }

        /* Strip quotes from strings */
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
        }

        kv[section + "." + key] = val;
    }

    auto getBool = [&](const std::string &k, bool def) -> bool {
        auto it = kv.find(k);
        if (it == kv.end()) return def;
        return it->second == "true" || it->second == "1";
    };
    auto getInt = [&](const std::string &k, int def) -> int {
        auto it = kv.find(k);
        if (it == kv.end()) return def;
        try { return std::stoi(it->second); } catch (...) { return def; }
    };
    auto getDouble = [&](const std::string &k, double def) -> double {
        auto it = kv.find(k);
        if (it == kv.end()) return def;
        try { return std::stod(it->second); } catch (...) { return def; }
    };
    auto getString = [&](const std::string &k, const std::string &def) -> std::string {
        auto it = kv.find(k);
        if (it == kv.end()) return def;
        return it->second;
    };

    cfg.imperial = (getString("display.units", "metric") == "imperial");

    cfg.spindle_counts_per_rev = getInt("spindle.counts_per_rev", cfg.spindle_counts_per_rev);
    cfg.spindle_inertia = getDouble("spindle.inertia_kg_m2", cfg.spindle_inertia);
    cfg.spindle_max_torque = getDouble("spindle.max_torque_nm", cfg.spindle_max_torque);
    cfg.spindle_friction = getDouble("spindle.friction_nm", cfg.spindle_friction);
    cfg.spindle_initial_rpm = getDouble("spindle.initial_rpm", cfg.spindle_initial_rpm);

    cfg.leadscrew_tpi = getDouble("leadscrew.tpi", cfg.leadscrew_tpi);
    cfg.leadscrew_mm_per_step = getDouble("leadscrew.mm_per_step", cfg.leadscrew_mm_per_step);

    cfg.z_encoder_counts_per_mm = getDouble("z_axis.encoder_counts_per_mm", cfg.z_encoder_counts_per_mm);
    cfg.z_backlash_mm = getDouble("z_axis.backlash_mm", cfg.z_backlash_mm);
    cfg.z_max_mm = getDouble("z_axis.max_position_mm", cfg.z_max_mm);
    cfg.z_min_mm = getDouble("z_axis.min_position_mm", cfg.z_min_mm);
    cfg.z_initial_mm = getDouble("z_axis.initial_position_mm", cfg.z_initial_mm);
    cfg.z_half_nut_engaged = getBool("z_axis.half_nut_engaged", cfg.z_half_nut_engaged);

    cfg.x_encoder_counts_per_mm = getDouble("cross_slide.encoder_counts_per_mm", cfg.x_encoder_counts_per_mm);
    cfg.x_max_mm = getDouble("cross_slide.max_position_mm", cfg.x_max_mm);
    cfg.x_min_mm = getDouble("cross_slide.min_position_mm", cfg.x_min_mm);
    cfg.x_initial_mm = getDouble("cross_slide.initial_position_mm", cfg.x_initial_mm);
    cfg.x_manual_step_mm = getDouble("cross_slide.manual_step_mm", cfg.x_manual_step_mm);
    cfg.jog_max_velocity_mm_s = getDouble("cross_slide.jog_max_velocity_mm_s", cfg.jog_max_velocity_mm_s);
    cfg.jog_acceleration_mm_s2 = getDouble("cross_slide.jog_acceleration_mm_s2", cfg.jog_acceleration_mm_s2);
    cfg.x_up_is_negative = getBool("cross_slide.x_up_is_negative", cfg.x_up_is_negative);
    cfg.manual_move_timeout_s = getDouble("cross_slide.manual_move_timeout_s", cfg.manual_move_timeout_s);

    cfg.servo_max_speed = getDouble("servo.max_speed", cfg.servo_max_speed);
    cfg.servo_acceleration = getDouble("servo.acceleration", cfg.servo_acceleration);

    cfg.modbus_address = getInt("modbus.address", cfg.modbus_address);
    cfg.modbus_baud = getInt("modbus.baud", cfg.modbus_baud);

    cfg.pty_enabled = getBool("transport.pty_enabled", cfg.pty_enabled);
    cfg.tcp_port = getInt("transport.tcp_port", cfg.tcp_port);
    cfg.tcp_enabled = getBool("transport.tcp_enabled", cfg.tcp_enabled);

    cfg.refresh_hz = getInt("dashboard.refresh_hz", cfg.refresh_hz);
    cfg.sparkline_seconds = getInt("dashboard.sparkline_seconds", cfg.sparkline_seconds);
    cfg.log_max_lines = getInt("dashboard.log_max_lines", cfg.log_max_lines);

    cfg.isr_rate_hz = getInt("simulation.isr_rate_hz", cfg.isr_rate_hz);
    cfg.realtime = getBool("simulation.realtime", cfg.realtime);

    std::cout << "Config loaded from " << path << "\n";
    return true;
}
