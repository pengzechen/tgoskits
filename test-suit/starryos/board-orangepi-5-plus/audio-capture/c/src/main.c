// StarryOS /dev/audio0 board smoke test.
//
// Validates the RK3588 I2S/TDM PIO capture pipeline end to end from user space:
// open /dev/audio0, poll for POLLIN (exercising the IRQ-driven poll wake), and
// read signed-16-bit little-endian mono PCM until a one-second budget of samples
// is collected or a wall-clock deadline elapses. Captured samples are written to
// a WAV file (best effort, for later gate-5 retrieval) and a signal summary is
// printed so a near-silent noise floor is still recognised as a live stream.
//
// Success is judged only by the unique STARRY_AUDIO_CAPTURE_OK marker: it means
// bytes flowed FIFO -> IRQ -> ring -> read. Distinct failure markers keep a hang
// from being the only failure signal.
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define AUDIO_PATH "/dev/audio0"
#define WAV_PATH "/root/audio0-capture.wav"
#define SAMPLE_RATE 48000U
#define CHANNELS 1U
#define BITS_PER_SAMPLE 16U
#define BYTES_PER_SAMPLE (BITS_PER_SAMPLE / 8U)
// One second of mono S16 at 48 kHz.
#define TARGET_BYTES (SAMPLE_RATE * BYTES_PER_SAMPLE * CHANNELS)
// A working pipeline delivers at least this much within the deadline.
#define MIN_BYTES 4096U
#define MAX_WAIT_MS 15000
#define POLL_SLICE_MS 1000
#define READ_CHUNK 4096U

static long elapsed_ms(const struct timespec *start) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L +
           (now.tv_nsec - start->tv_nsec) / 1000000L;
}

// Little-endian scalar writers keep the WAV header byte layout explicit.
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

// Best effort: a WAV write failure does not fail the pipeline check.
static void write_wav(const uint8_t *pcm, size_t pcm_bytes) {
    uint8_t hdr[44];
    uint32_t byte_rate = SAMPLE_RATE * CHANNELS * BYTES_PER_SAMPLE;
    memcpy(hdr, "RIFF", 4);
    put_u32(hdr + 4, 36U + (uint32_t)pcm_bytes);
    memcpy(hdr + 8, "WAVE", 4);
    memcpy(hdr + 12, "fmt ", 4);
    put_u32(hdr + 16, 16U);
    put_u16(hdr + 20, 1U); // PCM
    put_u16(hdr + 22, (uint16_t)CHANNELS);
    put_u32(hdr + 24, SAMPLE_RATE);
    put_u32(hdr + 28, byte_rate);
    put_u16(hdr + 32, (uint16_t)(CHANNELS * BYTES_PER_SAMPLE));
    put_u16(hdr + 34, (uint16_t)BITS_PER_SAMPLE);
    memcpy(hdr + 36, "data", 4);
    put_u32(hdr + 40, (uint32_t)pcm_bytes);

    int fd = open(WAV_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        printf("STARRY_AUDIO_CAPTURE_WARN wav_open errno=%d\n", errno);
        return;
    }
    if (write(fd, hdr, sizeof(hdr)) != (ssize_t)sizeof(hdr) ||
        write(fd, pcm, pcm_bytes) != (ssize_t)pcm_bytes) {
        printf("STARRY_AUDIO_CAPTURE_WARN wav_write errno=%d\n", errno);
    }
    close(fd);
}

int main(void) {
    uint8_t *pcm = malloc(TARGET_BYTES);
    if (pcm == NULL) {
        printf("STARRY_AUDIO_CAPTURE_FAILED: malloc\n");
        return 1;
    }

    int fd = open(AUDIO_PATH, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        printf("STARRY_AUDIO_CAPTURE_FAILED: open errno=%d\n", errno);
        free(pcm);
        return 1;
    }
    printf("STARRY_AUDIO_CAPTURE_BEGIN path=%s target=%u\n", AUDIO_PATH,
           TARGET_BYTES);
    fflush(stdout);

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    size_t total = 0;
    int io_error = 0;
    while (total < TARGET_BYTES && elapsed_ms(&start) < MAX_WAIT_MS) {
        struct pollfd pfd = {.fd = fd, .events = POLLIN, .revents = 0};
        int pr = poll(&pfd, 1, POLL_SLICE_MS);
        if (pr < 0) {
            if (errno == EINTR) {
                continue;
            }
            printf("STARRY_AUDIO_CAPTURE_FAILED: poll errno=%d\n", errno);
            io_error = 1;
            break;
        }
        if (pr == 0) {
            continue; // slice timeout; outer loop re-checks the deadline
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            printf("STARRY_AUDIO_CAPTURE_FAILED: poll revents=%d\n", pfd.revents);
            io_error = 1;
            break;
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }
        size_t want = TARGET_BYTES - total;
        if (want > READ_CHUNK) {
            want = READ_CHUNK;
        }
        ssize_t n = read(fd, pcm + total, want);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            printf("STARRY_AUDIO_CAPTURE_FAILED: read errno=%d\n", errno);
            io_error = 1;
            break;
        }
        total += (size_t)n;
    }
    close(fd);

    size_t samples = total / BYTES_PER_SAMPLE;
    size_t nonzero = 0;
    int32_t absmax = 0;
    uint64_t abssum = 0;
    for (size_t i = 0; i < samples; i++) {
        int16_t s;
        memcpy(&s, pcm + i * BYTES_PER_SAMPLE, sizeof(s));
        int32_t a = s < 0 ? -(int32_t)s : (int32_t)s;
        if (a != 0) {
            nonzero++;
        }
        if (a > absmax) {
            absmax = a;
        }
        abssum += (uint64_t)a;
    }
    unsigned meanabs = samples ? (unsigned)(abssum / samples) : 0U;

    if (io_error || total < MIN_BYTES) {
        printf("STARRY_AUDIO_CAPTURE_NODATA bytes=%zu samples=%zu waited_ms=%ld\n",
               total, samples, elapsed_ms(&start));
        free(pcm);
        return 1;
    }

    write_wav(pcm, total);
    printf("STARRY_AUDIO_CAPTURE_OK bytes=%zu samples=%zu nonzero=%zu absmax=%d "
           "meanabs=%u wav=%s\n",
           total, samples, nonzero, absmax, meanabs, WAV_PATH);
    fflush(stdout);
    free(pcm);
    return 0;
}
