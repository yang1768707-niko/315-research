#ifndef DOWNLOAD_RESET_H
#define DOWNLOAD_RESET_H

#include <stdint.h>

/* One dedicated CPU1 page, outside executable code and the Zigbee EEPROM. */
#define DOWNLOAD_RESET_ADDRESS UINT32_C(0x0806f000)
#define DOWNLOAD_RESET_PAGE_BYTES UINT32_C(4096)
#define DOWNLOAD_RESET_ARMED UINT64_C(0x41524d4544585431)
#define DOWNLOAD_RESET_DONE UINT64_C(0x444f4e4545585431)

typedef struct {
    void *context;
    uint64_t (*marker)(void *);
    int (*discard_records)(void *);
    int (*commit)(void *);
} DownloadResetIO;

/* 0: ordinary reboot, 1: records invalidated and committed, -1: stop startup.
 * Only DONE permits normal startup. An erased/torn marker retries cleanup.
 * Payload is retained; the selected slot is erased on its next acquisition.
 * Commit is last, so interrupted invalidation cannot expose old batches. */
static inline int download_reset_run(const DownloadResetIO *io)
{
    if (io->marker(io->context) == DOWNLOAD_RESET_DONE) return 0;
    if (!io->discard_records(io->context)) return -1;
    if (!io->commit(io->context) ||
        io->marker(io->context) != DOWNLOAD_RESET_DONE) return -1;
    return 1;
}
#endif
