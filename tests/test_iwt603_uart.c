#define _DEFAULT_SOURCE
#define _XOPEN_SOURCE 600
#include "imu.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

struct pseudo_terminal {
    int master;
    char slave[128];
};

struct concurrent_read_context {
    struct imu_dev *dev;
    int master;
    pthread_barrier_t start;
};

struct concurrent_reader_args {
    struct concurrent_read_context *context;
    unsigned int successful_reads;
};

static struct pseudo_terminal open_terminal(void)
{
    struct pseudo_terminal terminal = {.master = -1};
    char *name;

    terminal.master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(terminal.master >= 0);
    assert(grantpt(terminal.master) == 0);
    assert(unlockpt(terminal.master) == 0);
    name = ptsname(terminal.master);
    assert(name && strlen(name) < sizeof(terminal.slave));
    strcpy(terminal.slave, name);
    return terminal;
}

static void pause_ms(unsigned int milliseconds)
{
    struct timespec delay;
    delay.tv_sec = milliseconds / 1000U;
    delay.tv_nsec = (long)(milliseconds % 1000U) * 1000000L;
    nanosleep(&delay, NULL);
}

static void pause_us(unsigned int microseconds)
{
    struct timespec delay;
    delay.tv_sec = microseconds / 1000000U;
    delay.tv_nsec = (long)(microseconds % 1000000U) * 1000L;
    nanosleep(&delay, NULL);
}

static void send_frame_values(int master, uint8_t type, int16_t x, int16_t y, int16_t z, int16_t extra)
{
    uint8_t frame[11] = {0x55, type, (uint8_t)x, (uint8_t)((uint16_t)x >> 8U), (uint8_t)y, (uint8_t)((uint16_t)y >> 8U),
        (uint8_t)z, (uint8_t)((uint16_t)z >> 8U), (uint8_t)extra, (uint8_t)((uint16_t)extra >> 8U), 0x00};
    size_t i;
    uint8_t checksum = 0;

    for (i = 0; i < 10; ++i)
        checksum = (uint8_t)(checksum + frame[i]);
    frame[10] = checksum;
    assert(write(master, frame, sizeof(frame)) == (ssize_t)sizeof(frame));
}

static void send_frame(int master, uint8_t type)
{
    send_frame_values(master, type, 0x10, 0x20, 0x30, 0x40);
}

static void send_flat_acceleration(int master)
{
    /* IWT603 acceleration scale is +/-16 g: raw 2048 is +1 g. */
    send_frame_values(master, 0x51, 0, 0, 2048, 0);
}

static int wait_for_sample(struct imu_dev *dev, struct imu_data *data, int master, unsigned int timeout_ms)
{
    unsigned int elapsed;
    for (elapsed = 0; elapsed < timeout_ms; elapsed += 10U) {
        send_flat_acceleration(master);
        send_frame(master, 0x52);
        if (imu_read(dev, data) == 0)
            return 0;
        pause_ms(10);
    }
    return -1;
}

static void *send_concurrent_frames(void *opaque)
{
    struct concurrent_read_context *context = opaque;
    unsigned int iteration;
    int barrier_result;

    barrier_result = pthread_barrier_wait(&context->start);
    assert(barrier_result == 0 || barrier_result == PTHREAD_BARRIER_SERIAL_THREAD);
    for (iteration = 0; iteration < 4000U; ++iteration) {
        send_flat_acceleration(context->master);
        send_frame(context->master, 0x52);
        /* Keep the onboard quaternion fresh during concurrent reads. */
        send_frame(context->master, 0x59);
        pause_us(100U);
    }
    return NULL;
}

static void *read_concurrently(void *opaque)
{
    struct concurrent_reader_args *args = opaque;
    unsigned int iteration;
    int barrier_result;

    barrier_result = pthread_barrier_wait(&args->context->start);
    assert(barrier_result == 0 || barrier_result == PTHREAD_BARRIER_SERIAL_THREAD);
    for (iteration = 0; iteration < 8000U; ++iteration) {
        struct imu_data data;
        if (imu_read(args->context->dev, &data) == 0) {
            float quaternion_norm = sqrtf(data.quat[0] * data.quat[0] + data.quat[1] * data.quat[1] +
                data.quat[2] * data.quat[2] + data.quat[3] * data.quat[3]);
            assert(data.timestamp_us > 0);
            assert(isfinite(data.gyro[0]));
            assert(fabsf(quaternion_norm - 1.0f) < 1.0e-4f);
            ++args->successful_reads;
        }
        pause_us(50U);
    }
    return NULL;
}

static void test_concurrent_reads(struct imu_dev *dev, int master)
{
    struct concurrent_read_context context = {.dev = dev, .master = master};
    struct concurrent_reader_args readers[3] = {{.context = &context}, {.context = &context}, {.context = &context}};
    pthread_t writer;
    pthread_t reader_threads[3];
    unsigned int index;

    assert(pthread_barrier_init(&context.start, NULL, 4U) == 0);
    assert(pthread_create(&writer, NULL, send_concurrent_frames, &context) == 0);
    for (index = 0; index < 3U; ++index) {
        assert(pthread_create(&reader_threads[index], NULL, read_concurrently, &readers[index]) == 0);
    }
    assert(pthread_join(writer, NULL) == 0);
    for (index = 0; index < 3U; ++index) {
        assert(pthread_join(reader_threads[index], NULL) == 0);
        assert(readers[index].successful_reads > 0U);
    }
    assert(pthread_barrier_destroy(&context.start) == 0);
}

struct callback_context {
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    unsigned int count;
    struct imu_data latest;
    pthread_t receiver;
    int block;
    int entered;
    int unregister_done;
    struct imu_dev *dev;
};

static void receive_sample(struct imu_dev *dev, const struct imu_data *data, void *opaque)
{
    struct callback_context *context = opaque;
    assert(dev == context->dev);
    pthread_mutex_lock(&context->mutex);
    ++context->count;
    context->latest = *data;
    context->receiver = pthread_self();
    context->entered = 1;
    pthread_cond_broadcast(&context->changed);
    while (context->block)
        pthread_cond_wait(&context->changed, &context->mutex);
    pthread_mutex_unlock(&context->mutex);
}

static int wait_callback(struct callback_context *context, unsigned int count)
{
    struct timespec deadline;
    int result = 0;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += 2;
    pthread_mutex_lock(&context->mutex);
    while (context->count < count && result == 0)
        result = pthread_cond_timedwait(&context->changed, &context->mutex, &deadline);
    const int reached = context->count >= count;
    pthread_mutex_unlock(&context->mutex);
    return reached;
}

static void *unregister_callback(void *opaque)
{
    struct callback_context *context = opaque;
    imu_set_callback(context->dev, NULL, NULL);
    pthread_mutex_lock(&context->mutex);
    context->unregister_done = 1;
    pthread_cond_broadcast(&context->changed);
    pthread_mutex_unlock(&context->mutex);
    return NULL;
}

static void test_callbacks(void)
{
    char directory[] = "/tmp/iwt603-callback-XXXXXX", path[256];
    struct pseudo_terminal terminal = open_terminal();
    struct callback_context context = {.mutex = PTHREAD_MUTEX_INITIALIZER, .changed = PTHREAD_COND_INITIALIZER};
    struct imu_data unused;
    struct imu_config config = {0};
    unsigned int count;
    pthread_t stopper;
    assert(mkdtemp(directory));
    snprintf(path, sizeof(path), "%s/imu", directory);
    assert(symlink(terminal.slave, path) == 0);
    context.dev = imu_alloc_uart("drv_uart_iwt603", path, 921600, NULL);
    assert(context.dev);
    imu_set_callback(context.dev, receive_sample, &context);
    config.mounting_matrix[0] = config.mounting_matrix[4] = config.mounting_matrix[8] = 1;
    config.gyro_offset[0] = .1f;
    assert(imu_init(context.dev, &config) == 0);
    /* No imu_read drives this test. Incomplete/invalid data cannot call back. */
    send_flat_acceleration(terminal.master);
    const uint8_t bad_frame[11] = {0x55, 0x52};
    assert(write(terminal.master, bad_frame, sizeof(bad_frame)) == sizeof(bad_frame));
    pause_ms(30);
    pthread_mutex_lock(&context.mutex);
    assert(context.count == 0);
    pthread_mutex_unlock(&context.mutex);
    send_frame(terminal.master, 0x52);
    assert(wait_callback(&context, 1));
    pthread_mutex_lock(&context.mutex);
    assert(!pthread_equal(context.receiver, pthread_self()));
    assert(context.latest.timestamp_us > 0);
    assert(fabsf(context.latest.gyro[0] - (16.f * 2000.f * 3.14159265358979323846f / 180.f / 32768.f - .1f)) < 1e-5f);
    count = context.count;
    pthread_mutex_unlock(&context.mutex);
    /* A queued burst must reach the application estimator sample by sample. */
    uint8_t burst[5 * 22] = {0};
    for (unsigned int pair = 0; pair < 5; ++pair) {
        uint8_t *acc = burst + pair * 22;
        uint8_t *gyro = acc + 11;
        acc[0] = gyro[0] = 0x55;
        acc[1] = 0x51;
        acc[7] = 8; /* +1 g */
        gyro[1] = 0x52;
        gyro[2] = 16;
        for (unsigned int byte = 0; byte < 10; ++byte) {
            acc[10] = (uint8_t)(acc[10] + acc[byte]);
            gyro[10] = (uint8_t)(gyro[10] + gyro[byte]);
        }
    }
    assert(write(terminal.master, burst, sizeof(burst)) == (ssize_t)sizeof(burst));
    assert(wait_callback(&context, count + 5));
    pthread_mutex_lock(&context.mutex);
    assert(context.count == count + 5);
    count = context.count;
    pthread_mutex_unlock(&context.mutex);

    /* Acceleration-only updates cannot re-publish an old attitude sample. */
    send_flat_acceleration(terminal.master);
    assert(imu_read(context.dev, &unused) < 0);
    assert(imu_init(context.dev, NULL) < 0);
    assert(imu_calibrate_gyro_bias(context.dev, 10) < 0);
    pause_ms(220);
    pthread_mutex_lock(&context.mutex);
    assert(context.count == count); /* No cached-data callbacks during silence. */
    pthread_mutex_unlock(&context.mutex);

    close(terminal.master);
    assert(unlink(path) == 0);
    terminal = open_terminal();
    assert(symlink(terminal.slave, path) == 0);
    /* Event mode faults on disconnect; only synchronous reads auto-reconnect. */
    pause_ms(30);
    assert(imu_read(context.dev, &unused) == -EBUSY);
    imu_set_callback(context.dev, NULL, NULL);
    assert(imu_init(context.dev, &config) == 0);
    imu_set_callback(context.dev, receive_sample, &context);
    for (unsigned int attempt = 0; attempt < 100; ++attempt) {
        send_flat_acceleration(terminal.master);
        send_frame(terminal.master, 0x52);
        pause_ms(10);
        pthread_mutex_lock(&context.mutex);
        const int received = context.count > count;
        pthread_mutex_unlock(&context.mutex);
        if (received)
            break;
    }
    assert(wait_callback(&context, count + 1));

    /* Unregister must wait for an in-flight callback before ctx can be freed. */
    pthread_mutex_lock(&context.mutex);
    context.block = 1;
    context.entered = 0;
    count = context.count;
    pthread_mutex_unlock(&context.mutex);
    send_flat_acceleration(terminal.master);
    send_frame(terminal.master, 0x52);
    assert(wait_callback(&context, count + 1));
    assert(pthread_create(&stopper, NULL, unregister_callback, &context) == 0);
    pause_ms(30);
    pthread_mutex_lock(&context.mutex);
    assert(context.entered && !context.unregister_done);
    context.block = 0;
    pthread_cond_broadcast(&context.changed);
    pthread_mutex_unlock(&context.mutex);
    assert(pthread_join(stopper, NULL) == 0);
    count = context.count;
    send_flat_acceleration(terminal.master);
    send_frame(terminal.master, 0x52);
    assert(wait_for_sample(context.dev, &unused, terminal.master, 1000) == 0);
    pause_ms(30);
    assert(context.count == count); /* Read mode does not invoke callback. */
    imu_set_callback(context.dev, receive_sample, &context);
    send_flat_acceleration(terminal.master);
    send_frame(terminal.master, 0x52);
    assert(wait_callback(&context, count + 1));
    imu_free(context.dev); /* Also joins an active receiver. */
    count = context.count;
    pause_ms(30);
    assert(context.count == count);
    pthread_cond_destroy(&context.changed);
    pthread_mutex_destroy(&context.mutex);
    close(terminal.master);
    unlink(path);
    rmdir(directory);
}

int main(void)
{
    char directory[] = "/tmp/iwt603-test-XXXXXX";
    char stable_path[256];
    struct pseudo_terminal first;
    struct pseudo_terminal second;
    struct imu_dev *dev;
    struct imu_data initial;
    struct imu_data reconnected;

    assert(mkdtemp(directory));
    assert(snprintf(stable_path, sizeof(stable_path), "%s/iwt603", directory) > 0);
    first = open_terminal();
    assert(symlink(first.slave, stable_path) == 0);
    dev = imu_alloc_uart("drv_uart_iwt603", stable_path, 921600, NULL);
    assert(dev);
    assert(imu_init(dev, NULL) == 0);
    assert(wait_for_sample(dev, &initial, first.master, 1000) == 0);
    assert(initial.timestamp_us > 0);
    assert(isfinite(initial.gyro[0]));
    assert(isfinite(initial.quat[0]));
    {
        struct imu_data after_onboard_quaternion;
        send_frame(first.master, 0x59);
        for (unsigned int attempt = 0; attempt < 50; ++attempt) {
            pause_ms(1);
            assert(imu_read(dev, &after_onboard_quaternion) == 0);
            if (after_onboard_quaternion.timestamp_us > initial.timestamp_us)
                break;
        }
        assert(after_onboard_quaternion.timestamp_us > initial.timestamp_us);
        /* 0x59 updates the onboard orientation independently of gyro data. */
        for (unsigned int axis = 0; axis < 4; ++axis)
            assert(fabsf(after_onboard_quaternion.quat[axis] - (axis + 1) / sqrtf(30.f)) < 1e-5f);
    }
    test_concurrent_reads(dev, first.master);

    close(first.master);
    assert(unlink(stable_path) == 0);
    second = open_terminal();
    assert(symlink(second.slave, stable_path) == 0);
    pause_ms(120);
    assert(wait_for_sample(dev, &reconnected, second.master, 3000) == 0);
    assert(reconnected.timestamp_us > initial.timestamp_us);

    imu_free(dev);
    close(second.master);
    assert(unlink(stable_path) == 0);
    assert(rmdir(directory) == 0);
    test_callbacks();
    puts("IWT603 UART read, callback, reconnect and shutdown tests passed");
    return 0;
}
