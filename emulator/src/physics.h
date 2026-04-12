/*
 * Lathe physics model.
 *
 * Simulates spindle with inertia, leadscrew, carriage with half-nut, and cross-slide.
 */
#ifndef EMU_PHYSICS_H
#define EMU_PHYSICS_H

#include "config.h"
#include <cstdint>
#include <atomic>
#include <cmath>

class LathePhysics {
public:
    explicit LathePhysics(const EmuConfig &cfg);

    /* Called each ISR tick BEFORE calling into firmware.
     * Updates spindle and feeds encoder counter values into emu_hw. */
    /* shared must point to the firmware's rampsSharedData_t for reading servo state */
    void tick(double dt_seconds, const void *shared_data);

    /* Called from GPIO shim when a STEP rising edge is detected. */
    void onStepPulse(int direction);

    /* --- Spindle control (from dashboard keyboard) --- */
    void setTargetRPM(double rpm);
    void toggleDirection();
    void emergencyStop();

    double getSpindleRPM() const { return spindle_omega * 60.0 / (2.0 * M_PI); }
    double getTargetRPM() const { return spindle_target_rpm; }
    bool   getSpindleCW() const { return spindle_target_rpm >= 0; }
    int64_t getSpindleEncoderCounts() const;

    /* --- Half-nut --- */
    enum HalfNutState { DISENGAGED, ENGAGING, ENGAGED };

    void requestHalfNutToggle();
    HalfNutState getHalfNutState() const { return half_nut_state; }

    /* --- Carriage (Z-axis) manual move --- */
    double getCarriageMM() const { return carriage_mm; }
    void   jogCarriage(int direction);       /* Arrow key jog: direction +1/-1 */
    void   moveCarriageTo(double target_mm); /* Move to specific position */
    bool   isZMoveTargetActive() const { return z_move_active; }
    int64_t getCarriageEncoderCounts() const;

    /* --- Cross-slide (X-axis) manual move --- */
    double getCrossSlideMM() const { return cross_slide_mm; }
    void   jogCrossSlide(int direction);
    void   moveCrossSlideTo(double target_mm);
    bool   isXMoveTargetActive() const { return x_move_active; }
    int64_t getCrossSlideEncoderCounts() const;

    /* --- Jog status --- */
    bool isZJogging() const { return std::abs(z_jog_velocity) > 0.01; }
    bool isXJogging() const { return std::abs(x_jog_velocity) > 0.01; }

    /* --- Leadscrew --- */
    double getLeadscrewPositionMM() const { return leadscrew_position_mm; }
    double getLeadscrewGridSpacingMM() const { return leadscrew_grid_spacing_mm; }

private:
    /* Config */
    double spindle_inertia;
    double spindle_max_torque;
    double spindle_friction;
    int    spindle_counts_per_rev;
    double leadscrew_mm_per_step;
    double leadscrew_tpi;
    double leadscrew_grid_spacing_mm;  /* 25.4 / tpi */
    double z_counts_per_mm;
    double z_backlash_mm;
    double z_max_mm, z_min_mm;
    double x_counts_per_mm;
    double x_max_mm, x_min_mm;
    double x_manual_step_mm;

    /* Spindle state */
    double spindle_theta;         /* cumulative angle in radians */
    double spindle_omega;         /* angular velocity in rad/s */
    double spindle_target_rpm;

    /* Leadscrew state (always tracks stepper steps) */
    double leadscrew_position_mm; /* cumulative from step pulses */
    int64_t leadscrew_total_steps;

    /* Carriage state */
    double carriage_mm;
    HalfNutState half_nut_state;
    bool   half_nut_request_pending;
    double backlash_remaining;
    int    last_carriage_dir;  /* +1 or -1, for backlash */

    /* Cross-slide state */
    double cross_slide_mm;

    /* Manual jog state (velocity-based with acceleration) */
    double z_jog_velocity;       /* current mm/s */
    double z_jog_target_dir;     /* -1, 0, or +1 (arrow key jog) */
    double x_jog_velocity;       /* current mm/s */
    double x_jog_target_dir;     /* -1, 0, or +1 (arrow key jog) */

    /* Move-to-position state */
    bool   z_move_active;
    double z_move_target;        /* target position in mm */
    bool   x_move_active;
    double x_move_target;
    double jog_max_velocity;     /* mm/s from config */
    double jog_acceleration;     /* mm/s^2 from config */
    double jog_timeout;          /* seconds since last key event */
    double z_jog_idle_timer;     /* time since last Z key */
    double x_jog_idle_timer;     /* time since last X key */
    static constexpr double JOG_KEY_TIMEOUT = 0.15; /* seconds of no key → stop */

    /* Half-nut engagement helpers */
    double getLeadscrewPhase() const;
    double getCarriageGridPhase() const;
    void   snapCarriageToGrid();
    bool   checkPhaseAlignment() const;
};

#endif /* EMU_PHYSICS_H */
