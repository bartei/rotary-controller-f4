/*
 * Modbus transport layer.
 * PTY pair + TCP socket, feeding bytes into the firmware's UART shim.
 */
#ifndef EMU_TRANSPORT_H
#define EMU_TRANSPORT_H

#include "config.h"
#include <string>
#include <atomic>
#include <thread>

class Transport {
public:
    explicit Transport(const EmuConfig &cfg);
    ~Transport();

    /* Start transport threads. */
    void start();

    /* Stop transport threads. */
    void stop();

    /* Send bytes from firmware TX to all connected clients. */
    void sendToClients(const uint8_t *data, uint16_t size);

    /* Get the PTY slave device path (for user to connect client). */
    std::string getPtyPath() const { return pty_slave_path; }

    /* Connection status */
    bool isPtyConnected() const { return pty_fd >= 0; }
    int  getTcpClientCount() const { return tcp_client_fd >= 0 ? 1 : 0; }

private:
    /* Config */
    bool pty_enabled;
    bool tcp_enabled;
    int  tcp_port;

    /* PTY */
    int pty_master_fd;
    int pty_slave_fd;
    std::string pty_slave_path;
    int pty_fd;  /* alias for master */

    /* TCP */
    int tcp_listen_fd;
    int tcp_client_fd;

    /* Threads */
    std::atomic<bool> running;
    std::thread poll_thread;

    void pollLoop();
    void injectByte(uint8_t byte);

    /* PTY setup */
    bool setupPty();

    /* TCP setup */
    bool setupTcp();
};

#endif /* EMU_TRANSPORT_H */
