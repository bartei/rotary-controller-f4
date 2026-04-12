/*
 * Transport implementation: PTY pair + TCP socket.
 */

#include "transport.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pty.h>
#include <termios.h>
#include <errno.h>

extern "C" {
#include "emulator_state.h"
#include "stm32f4xx_hal_uart.h"
extern UART_HandleTypeDef huart1;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
}

Transport::Transport(const EmuConfig &cfg)
    : pty_enabled(cfg.pty_enabled), tcp_enabled(cfg.tcp_enabled), tcp_port(cfg.tcp_port),
      pty_master_fd(-1), pty_slave_fd(-1), pty_fd(-1),
      tcp_listen_fd(-1), tcp_client_fd(-1), running(false)
{
}

Transport::~Transport() {
    stop();
}

bool Transport::setupPty() {
    if (openpty(&pty_master_fd, &pty_slave_fd, nullptr, nullptr, nullptr) < 0) {
        perror("openpty");
        return false;
    }

    /* Set master to raw mode, non-blocking */
    struct termios tio;
    tcgetattr(pty_master_fd, &tio);
    cfmakeraw(&tio);
    tcsetattr(pty_master_fd, TCSANOW, &tio);
    fcntl(pty_master_fd, F_SETFL, O_NONBLOCK);

    /* Also set slave to raw mode */
    tcgetattr(pty_slave_fd, &tio);
    cfmakeraw(&tio);
    tcsetattr(pty_slave_fd, TCSANOW, &tio);

    pty_slave_path = ttyname(pty_slave_fd);
    pty_fd = pty_master_fd;

    printf("Modbus serial: %s\n", pty_slave_path.c_str());
    return true;
}

bool Transport::setupTcp() {
    tcp_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_listen_fd < 0) {
        perror("socket");
        return false;
    }

    int opt = 1;
    setsockopt(tcp_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(tcp_port);

    if (bind(tcp_listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(tcp_listen_fd);
        tcp_listen_fd = -1;
        return false;
    }

    if (listen(tcp_listen_fd, 1) < 0) {
        perror("listen");
        close(tcp_listen_fd);
        tcp_listen_fd = -1;
        return false;
    }

    fcntl(tcp_listen_fd, F_SETFL, O_NONBLOCK);
    printf("Modbus TCP: listening on port %d\n", tcp_port);
    return true;
}

void Transport::start() {
    if (pty_enabled) setupPty();
    if (tcp_enabled) setupTcp();

    /* Register TX callback */
    emu_hw.on_uart_tx = [](const uint8_t *data, uint16_t size) {
        /* This is called from the firmware's Modbus TX path.
         * We need to get a pointer to the Transport instance.
         * Use the user_data field. */
        auto *self = (Transport*)emu_hw.user_data;
        if (self) self->sendToClients(data, size);
    };
    emu_hw.user_data = this;

    running.store(true);
    poll_thread = std::thread([this]() { pollLoop(); });
}

void Transport::stop() {
    running.store(false);
    if (poll_thread.joinable()) poll_thread.join();
    if (pty_master_fd >= 0) { close(pty_master_fd); pty_master_fd = -1; }
    if (pty_slave_fd >= 0) { close(pty_slave_fd); pty_slave_fd = -1; }
    if (tcp_client_fd >= 0) { close(tcp_client_fd); tcp_client_fd = -1; }
    if (tcp_listen_fd >= 0) { close(tcp_listen_fd); tcp_listen_fd = -1; }
}

void Transport::sendToClients(const uint8_t *data, uint16_t size) {
    if (pty_master_fd >= 0) {
        ssize_t n = write(pty_master_fd, data, size);
        (void)n;
    }
    if (tcp_client_fd >= 0) {
        ssize_t n = send(tcp_client_fd, data, size, MSG_NOSIGNAL);
        if (n < 0) {
            close(tcp_client_fd);
            tcp_client_fd = -1;
            emu_log_event("TCP client disconnected");
        }
    }
}

void Transport::injectByte(uint8_t byte) {
    /* Feed byte into firmware's Modbus UART RX path */
    if (emu_hw.uart_rx_armed && emu_hw.uart_rx_buf) {
        *emu_hw.uart_rx_buf = byte;
        emu_hw.uart_rx_armed = 0;
        HAL_UART_RxCpltCallback(&huart1);
    }
}

void Transport::pollLoop() {
    while (running.load()) {
        struct pollfd fds[3];
        int nfds = 0;

        if (pty_master_fd >= 0) {
            fds[nfds].fd = pty_master_fd;
            fds[nfds].events = POLLIN;
            nfds++;
        }

        if (tcp_listen_fd >= 0) {
            fds[nfds].fd = tcp_listen_fd;
            fds[nfds].events = POLLIN;
            nfds++;
        }

        if (tcp_client_fd >= 0) {
            fds[nfds].fd = tcp_client_fd;
            fds[nfds].events = POLLIN;
            nfds++;
        }

        if (nfds == 0) {
            usleep(10000);
            continue;
        }

        int ret = poll(fds, nfds, 10 /* 10ms timeout */);
        if (ret <= 0) continue;

        for (int i = 0; i < nfds; i++) {
            if (!(fds[i].revents & POLLIN)) continue;

            if (fds[i].fd == pty_master_fd) {
                /* Data from PTY client */
                uint8_t buf[256];
                ssize_t n = read(pty_master_fd, buf, sizeof(buf));
                if (n > 0) {
                    for (ssize_t j = 0; j < n; j++) {
                        injectByte(buf[j]);
                    }
                }
            } else if (fds[i].fd == tcp_listen_fd) {
                /* New TCP connection */
                struct sockaddr_in client_addr;
                socklen_t addr_len = sizeof(client_addr);
                int new_fd = accept(tcp_listen_fd, (struct sockaddr*)&client_addr, &addr_len);
                if (new_fd >= 0) {
                    if (tcp_client_fd >= 0) close(tcp_client_fd);
                    tcp_client_fd = new_fd;
                    fcntl(tcp_client_fd, F_SETFL, O_NONBLOCK);
                    emu_log_event("TCP client connected from %s", inet_ntoa(client_addr.sin_addr));
                }
            } else if (fds[i].fd == tcp_client_fd) {
                /* Data from TCP client */
                uint8_t buf[256];
                ssize_t n = recv(tcp_client_fd, buf, sizeof(buf), 0);
                if (n <= 0) {
                    close(tcp_client_fd);
                    tcp_client_fd = -1;
                    emu_log_event("TCP client disconnected");
                } else {
                    for (ssize_t j = 0; j < n; j++) {
                        injectByte(buf[j]);
                    }
                }
            }
        }
    }
}
