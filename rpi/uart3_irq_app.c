// uart_test_app.c — poll stdin + serial; print complete lines from serial
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <errno.h>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>

/* ---- config ---- */
#define DEVICE      "/dev/uart3_raw"   // change if needed (e.g., "/dev/serial0")
#define BUFSIZE     256
#define TIMEOUT_MS  2000               // write wait max
#define SLICE_MS    50                 // poll period
#define LINEBUF_SZ  2048               // receive line assembly buffer

/* ----- optional: put serial into raw mode (safe even if already raw) ----- */
static int make_serial_raw(int fd, speed_t baud /*e.g., B115200*/)
{
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) return -1;

    cfmakeraw(&tio);
    // Keep 8N1
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;      // 1 stop
    tio.c_cflag &= ~PARENB;      // no parity
    tio.c_cflag &= ~CRTSCTS;     // no HW flow control

    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);

    // Non-blocking read returns immediately; timing handled by poll()
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    return tcsetattr(fd, TCSANOW, &tio);
}

/* ----- nonblocking safe write (handles partial/EAGAIN/EINTR) ----- */
static int write_all_nonblock(int fd, const char *buf, size_t len, int timeout_ms)
{
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);
        if (n > 0) { off += (size_t)n; continue; }
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                struct pollfd wp = { .fd = fd, .events = POLLOUT };
                int pr = poll(&wp, 1, timeout_ms);
                if (pr == 0)   return -ETIMEDOUT;
                if (pr < 0)    return -errno;
                if (wp.revents & (POLLERR | POLLHUP | POLLNVAL)) return -EIO;
                continue; // ready to write again
            }
            return -errno;
        }
    }
    return 0;
}

/* ---- receive line assembler ---- */
static char  linebuf[LINEBUF_SZ];
static size_t linelen = 0;

/* read as much as available; print complete lines only */
static int drain_serial_and_print_lines(int fd)
{
    char tmp[256];

    for (;;) {
        ssize_t n = read(fd, tmp, sizeof(tmp));
        if (n > 0) {
            size_t space = (LINEBUF_SZ - 1) - linelen;
            size_t to_copy = (size_t)n <= space ? (size_t)n : space;
            memcpy(linebuf + linelen, tmp, to_copy);
            linelen += to_copy;
            linebuf[linelen] = '\0';

            // extract lines by '\n'
            char *start = linebuf;
            for (;;) {
                char *nl = memchr(start, '\n', linebuf + linelen - start);
                if (!nl) break;

                size_t len = (size_t)(nl - start);
                if (len && start[len - 1] == '\r') len--; // trim CR of CRLF
                printf("Recv : %.*s\n", (int)len, start);

                start = nl + 1;
            }

            // shift the remainder to the front
            size_t remain = (size_t)(linebuf + linelen - start);
            memmove(linebuf, start, remain);
            linelen = remain;
            linebuf[linelen] = '\0';
            continue;
        }
        if (n == 0) return 0; // peer closed or nothing more (non-tty)
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // no more now
        perror("read");
        return -1;
    }
}

int main(void)
{
    // line-buffer stdout so prompt/echo show nicely
    setvbuf(stdout, NULL, _IOLBF, 0);

    int fd = open(DEVICE, O_RDWR | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }

    // best effort raw serial (ignore error if custom driver already raw)
    (void)make_serial_raw(fd, B115200);

    char send[BUFSIZE];

    printf("Type text to send. Quit with q or Q.\n");

    for (;;) {
        struct pollfd pfds[2] = {
            { .fd = STDIN_FILENO, .events = POLLIN },
            { .fd = fd,           .events = POLLIN | POLLERR | POLLHUP }
        };

        int pr = poll(pfds, 2, SLICE_MS);
        if (pr < 0) { if (errno == EINTR) continue; perror("poll"); break; }

        // 1) always drain serial first (prints only full lines)
        if (pfds[1].revents & POLLIN) {
            if (drain_serial_and_print_lines(fd) < 0) break;
        }
        if (pfds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, "serial error/hup\n");
            break;
        }

        // 2) on user input, send it (append '\n')
        if (pfds[0].revents & POLLIN) {
            if (!fgets(send, sizeof(send), stdin)) break;
            send[strcspn(send, "\n")] = '\0';
            if (!strcmp(send, "q") || !strcmp(send, "Q")) break;

            size_t slen = strlen(send);
            if (slen + 1 < sizeof(send)) {
                send[slen] = '\n';
                send[slen + 1] = '\0';
                slen += 1;
            }

            int wr = write_all_nonblock(fd, send, slen, TIMEOUT_MS);
            if (wr < 0) {
                fprintf(stderr, (wr == -ETIMEDOUT) ? "[Timeout] write POLLOUT\n"
                                                    : "write error: %s\n", strerror(-wr));
                break;
            }
            printf("Sent : %s", send); // send already includes '\n'
        }
    }

    close(fd);
    return 0;
}
