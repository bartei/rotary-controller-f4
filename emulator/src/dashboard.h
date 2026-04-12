/*
 * Two-pane ANSI terminal dashboard with sparklines.
 */
#ifndef EMU_DASHBOARD_H
#define EMU_DASHBOARD_H

#include "config.h"
#include "physics.h"
#include "transport.h"
#include <vector>
#include <deque>
#include <atomic>

extern "C" {
#include "Ramps.h"
}

class Dashboard {
public:
    Dashboard(const EmuConfig &cfg, LathePhysics &physics, Transport &transport,
              rampsSharedData_t &shared);

    /* Run the dashboard loop (blocking, runs on main thread).
     * Returns when user presses Q. */
    void run();

    /* Signal the dashboard to stop. */
    void requestStop() { running.store(false); }

private:
    const EmuConfig &cfg;
    LathePhysics &physics;
    Transport &transport;
    rampsSharedData_t &shared;

    std::atomic<bool> running;
    bool manual_move;          /* true when manual move is active */
    double manual_move_timer;  /* seconds since last arrow input */
    bool manual_move_used;     /* true once user has made at least one move */

    /* Sparkline history ring buffers */
    struct SparklineBuffer {
        std::deque<double> samples;
        int max_samples;
        int width;  /* character width for rendering */

        SparklineBuffer(int seconds, int hz, int w)
            : max_samples(seconds * hz), width(w) {}

        void push(double val) {
            samples.push_back(val);
            while ((int)samples.size() > max_samples)
                samples.pop_front();
        }

        std::string render() const;
    };

    SparklineBuffer spark_rpm;
    SparklineBuffer spark_zpos;
    SparklineBuffer spark_zerr;

    /* Unit conversion */
    double toDisplay(double mm) const;
    const char* unitSuffix() const;
    int unitPrecision() const;

    /* Rendering */
    void draw();
    void drawStatePane(int startRow, int startCol, int width);
    void drawLogPane(int startRow, int startCol, int width, int height);
    void drawStatusBar(int row, int width);

    /* Input */
    void handleInput();
    void promptSpindleRPM();
    void promptZPosition();
    void promptXPosition();
};

#endif /* EMU_DASHBOARD_H */
