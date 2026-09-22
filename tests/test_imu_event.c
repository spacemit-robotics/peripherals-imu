#define _DEFAULT_SOURCE
#include "imu_core.h"
#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <stdatomic.h>
#include <string.h>
#include <unistd.h>

struct fake {
    int pipefd[2];
    atomic_int reads;
    atomic_int starts;
    atomic_int stops;
};
struct context {
    atomic_int calls;
    int self_stop;
};

static int init(struct imu_dev *dev)
{
    (void)dev;
    return 0;
}
static int sample(struct imu_dev *dev, struct imu_data *data)
{
    (void)dev;
    memset(data, 0, sizeof(*data));
    data->acc[0] = 3;
    data->timestamp_us = 123;
    return 0;
}
static int failed_init(struct imu_dev *dev)
{
    (void)dev;
    return -EIO;
}

static int diagnostics_result;
static int diagnostics(struct imu_dev *dev, struct imu_diagnostics *data)
{
    (void)dev;
    data->valid_frames = 7;
    return diagnostics_result;
}

static void test_legacy_calls(void)
{
    const struct imu_ops legacy_ops = {
        .init = failed_init, .read = sample, .get_diagnostics = diagnostics};
    struct imu_dev *dev = imu_dev_alloc("legacy", 0);
    assert(dev);
    dev->ops = &legacy_ops;
    struct imu_data data;
    assert(imu_read(dev, &data) == 0);
    assert(imu_init(dev, NULL) == -EIO);
    assert(imu_read(dev, &data) == 0);
    assert(imu_calibrate_gyro_bias(dev, 1) == 0);
    struct imu_diagnostics result;
    diagnostics_result = 1;
    assert(imu_get_diagnostics(dev, &result) == 0);
    assert(result.valid_frames == 7 && result.receive_timestamp_us > 0);
    diagnostics_result = -EIO;
    assert(imu_get_diagnostics(dev, &result) == -EIO);
    assert(result.receive_timestamp_us == 0);
    imu_free(dev);
}
static int start(struct imu_dev *dev, struct imu_event_source *source)
{
    struct fake *f = dev->priv_data;
    atomic_fetch_add(&f->starts, 1);
    source->fd = f->pipefd[0];
    source->events = POLLIN;
    return 0;
}
static int event_read(struct imu_dev *dev, struct imu_data *data)
{
    struct fake *f = dev->priv_data;
    char command;
    assert(read(f->pipefd[0], &command, 1) == 1);
    atomic_fetch_add(&f->reads, 1);
    if (command == 'e')
        return -EAGAIN;
    if (command == 'f')
        return -EIO;
    return sample(dev, data);
}
static void stop(struct imu_dev *dev)
{
    struct fake *f = dev->priv_data;
    atomic_fetch_add(&f->stops, 1);
}
static const struct imu_ops ops = {
    .init = init, .read = sample, .event_start = start, .event_read = event_read, .event_stop = stop};

static void callback(struct imu_dev *dev, const struct imu_data *data, void *ctx)
{
    struct context *c = ctx;
    struct imu_data unused;
    struct imu_diagnostics diagnostics;
    assert(imu_get_diagnostics(dev, &diagnostics) == 0);
    assert(diagnostics.receive_timestamp_us > 0);
    assert(data->timestamp_us == 123);
    assert(data->acc[0] == 2);
    assert(imu_read(dev, &unused) == -EBUSY);
    assert(imu_init(dev, NULL) == -EBUSY);
    assert(imu_calibrate_gyro_bias(dev, 0) == -EBUSY);
    imu_set_callback(dev, callback, ctx); /* Rejected by the void registration API. */
    if (c->self_stop)
        imu_set_callback(dev, NULL, NULL);
    atomic_fetch_add(&c->calls, 1);
}
static void wait_count(atomic_int *count, int wanted)
{
    for (int i = 0; i < 2000 && atomic_load(count) != wanted; ++i)
        usleep(1000);
    assert(atomic_load(count) == wanted);
}
static void wait_mode(struct imu_dev *dev, enum imu_mode wanted)
{
    for (int i = 0; i < 2000; ++i) {
        pthread_mutex_lock(&dev->state_lock);
        int match = dev->mode == wanted;
        pthread_mutex_unlock(&dev->state_lock);
        if (match)
            return;
        usleep(1000);
    }
    assert(!"worker did not reach expected mode");
}
static void send_event(struct fake *f, char command)
{
    assert(write(f->pipefd[1], &command, 1) == 1);
}

int main(void)
{
    test_legacy_calls();
    struct imu_dev *dev = imu_dev_alloc("fake", sizeof(struct fake));
    assert(dev);
    dev->ops = &ops;
    struct fake *f = dev->priv_data;
    assert(pipe(f->pipefd) == 0);
    struct context first = {0}, second = {0};
    struct imu_config config = {0};
    config.acc_offset[0] = 1;
    struct imu_data data;
    assert(imu_init(dev, &config) == 0);
    assert(!dev->worker_valid && atomic_load(&f->starts) == 0);
    assert(imu_read(dev, &data) == 0 && data.acc[0] == 2);
    /* Exercise pending registration on a separate, uninitialized device. */
    struct imu_dev *pending = imu_dev_alloc("pending", 0);
    const struct imu_ops sync_ops = {.init = init, .read = sample};
    pending->ops = &sync_ops;
    imu_set_callback(pending, callback, &first);
    assert(imu_init(pending, &config) == 0);
    assert(pending->cb == callback && pending->cb_ctx == &first);
    assert(!pending->worker_valid);
    assert(imu_read(pending, &data) == 0);
    imu_set_callback(pending, callback, &second);
    assert(pending->cb_ctx == &second && !pending->worker_valid);
    assert(imu_calibrate_gyro_bias(pending, 1) == 0);
    imu_set_callback(pending, NULL, NULL);
    assert(!pending->cb && !pending->cb_ctx);
    imu_free(pending);

    imu_set_callback(dev, callback, &first);
    usleep(20000);
    assert(atomic_load(&f->reads) == 0 && atomic_load(&first.calls) == 0);
    send_event(f, 'e');
    wait_count(&f->reads, 1);
    assert(atomic_load(&first.calls) == 0);
    send_event(f, 's');
    wait_count(&first.calls, 1);
    imu_set_callback(dev, callback, &second);
    send_event(f, 's');
    wait_count(&second.calls, 1);
    assert(atomic_load(&first.calls) == 1);
    imu_set_callback(dev, NULL, NULL);
    second.self_stop = 1;
    imu_set_callback(dev, callback, &second);
    send_event(f, 's');
    wait_count(&second.calls, 2);
    wait_mode(dev, IMU_MODE_POLL);
    assert(imu_read(dev, &data) == 0);
    assert(imu_init(dev, &config) == 0);
    assert(!dev->worker_valid);
    imu_set_callback(dev, callback, &first);
    send_event(f, 'f');
    wait_mode(dev, IMU_MODE_FAULT);
    assert(imu_read(dev, &data) == -EBUSY);
    assert(imu_init(dev, &config) == -EBUSY);
    assert(imu_calibrate_gyro_bias(dev, 0) == -EBUSY);
    imu_set_callback(dev, callback, &first); /* Rejected by the void registration API. */
    imu_set_callback(dev, NULL, NULL);
    assert(imu_read(dev, &data) == -EINVAL);
    assert(imu_init(dev, &config) == 0);
    for (int i = 0; i < 100; ++i) {
        imu_set_callback(dev, callback, &first);
        imu_set_callback(dev, NULL, NULL);
    }
    assert(atomic_load(&f->starts) == atomic_load(&f->stops));
    imu_set_callback(dev, callback, &first);
    int read_fd = f->pipefd[0], write_fd = f->pipefd[1];
    imu_free(dev);
    close(read_fd);
    close(write_fd);
    return 0;
}
