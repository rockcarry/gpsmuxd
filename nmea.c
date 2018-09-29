#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <pthread.h>
#include <fcntl.h>
#include "nmea.h"

typedef struct {
    char       dev[PATH_MAX];
    GPS_STATUS gps_status;

    #define TS_EXIT   (1 << 0)
    uint32_t          thread_status;
    pthread_t         read_thread;
    PFN_NMEA_CALLBACK callback;
    void             *cbparams;
} CONTEXT;

static void do_nmea_parse(CONTEXT *ctxt, char *buf, int n)
{
}

static void* read_thread_proc(void *param)
{
    CONTEXT *ctxt = (CONTEXT*)param;
    int fd = -1;

    while (!(ctxt->thread_status & TS_EXIT)) {
        struct timeval tv;
        fd_set fds;
        int    ret;

        if (fd == -1) {
            fd = open(ctxt->dev, O_WRONLY);
        }
        if (fd <= 0) {
            usleep(100*1000);
            continue;
        }

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* timeout */
        tv.tv_sec  = 1;
        tv.tv_usec = 0;

        ret = select(fd + 1, &fds, NULL, NULL, &tv);
        switch (ret) {
        case  0: printf("select again !\n"); continue;
        case -1: printf("select error !\n"); goto end;
        }

        if (FD_ISSET(fd, &fds)) {
            char buf[32];
            int  n = read(fd, buf, sizeof(buf));
            do_nmea_parse(ctxt, buf, n);
        }
    }

end:
    if (fd != -1) {
        close(fd);
    }
}

void* nmea_init(char *dev, PFN_NMEA_CALLBACK cb, void *params)
{
    CONTEXT *ctxt = calloc(1, sizeof(CONTEXT));
    if (!ctxt) return NULL;

    strcpy(ctxt->dev, dev);
    pthread_create(&ctxt->read_thread, NULL, read_thread_proc, (void*)ctxt);
}

void nmea_exit(void *ctx)
{
    CONTEXT *ctxt = (CONTEXT*)ctx;
    ctxt->thread_status |= TS_EXIT;
    pthread_join(ctxt->read_thread, NULL);
    free(ctxt);
}

void nmea_gps_status(void *ctx, GPS_STATUS *pgs)
{
    CONTEXT *ctxt = (CONTEXT*)ctx;
    memcpy(pgs, &ctxt->gps_status, sizeof(GPS_STATUS));
}

void nmea_print(GPS_STATUS *pgs)
{
}

int main(int argc, char *argv[])
{
    return 0;
}
