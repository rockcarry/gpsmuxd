#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include "gpsnmea.h"

typedef struct {
    char       dev[PATH_MAX];
    GPS_STATUS gps_status;

    #define TS_EXIT   (1 << 0)
    uint32_t          thread_status;
    pthread_t         read_thread;
    PFN_NMEA_CALLBACK callback;
    void             *cbparams;

    #define NMEA_MAX_SIZE 128  // !!important, this size must be power of 2
    char              nmeabuf[NMEA_MAX_SIZE];
    int               nmeahead;
    int               nmeatail;
    int               nmeanum ;
} CONTEXT;

static void do_nmea_parse(CONTEXT *ctxt, char *sentence)
{
    char *tokens[10] = { sentence };
    int   need_callback = 1;
    int   i, j;

    printf("%s", sentence);
    //++ split sentence to tokens
    for (i=0,j=1; sentence[i]&&j<10; i++) {
        if (sentence[i] == ',') {
            sentence[i] = '\0';
            tokens[j++] = sentence + i + 1;
        }
    }
    //-- split sentence to tokens

    if (memcmp(sentence, "$GPRMC", 6) == 0) {
        printf("==ck== %s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
            tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7], tokens[8], tokens[9]);
    } else if (memcmp(sentence, "$GPGGA", 6) == 0) {
    } else if (memcmp(sentence, "$GPGSA", 6) == 0) {
    } else if (memcmp(sentence, "$GPGSV", 6) == 0) {
    } else {
//      printf("unparsed sentence !\n");
        need_callback = 0;
    }
    if (need_callback && ctxt->callback) {
        ctxt->callback(ctxt->cbparams, &ctxt->gps_status);
    }
}

static void add_nmea_buf(CONTEXT *ctxt, char *buf, int n)
{
    int i;
    for (i=0; i<n; i++) {
        //++ enqueue a char to nmeabuf
        ctxt->nmeabuf[ctxt->nmeatail++] = buf[i];
        ctxt->nmeatail &= (NMEA_MAX_SIZE - 1);
        if (ctxt->nmeanum < NMEA_MAX_SIZE) {
            ctxt->nmeanum++;
        } else {
            ctxt->nmeahead++;
            ctxt->nmeahead &= (NMEA_MAX_SIZE - 1);
        }
        //-- enqueue a char to nmeabuf

        //++ dequeue a nmea sentence
        if (buf[i] == '\n') {
            char sentence[NMEA_MAX_SIZE+1];
            int  j = 0;
            while (ctxt->nmeanum > 0) {
                sentence[j++] = ctxt->nmeabuf[ctxt->nmeahead++];
                ctxt->nmeahead &= (NMEA_MAX_SIZE - 1);
                ctxt->nmeanum--;
            }
            sentence[j] = '\0';
            do_nmea_parse(ctxt, sentence);
        }
        //-- dequeue a nmea sentence
    }
}

static int open_serial_port(char *name)
{
    struct termios termattr;
    int    ret;
    int    fd ;

    fd = open(name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("failed to open serial port: %s !\n", name);
        return fd;
    }

    ret = tcgetattr(fd, &termattr);
    if (ret < 0) {
        printf("tcgetattr failed !\n");
    }

    cfmakeraw(&termattr);
    cfsetospeed(&termattr, B9600);
    cfsetispeed(&termattr, B9600);

    termattr.c_cflag &= ~CRTSCTS; // flow ctrl
    termattr.c_cflag &= ~CSIZE;   // data size
    termattr.c_cflag |=  CS8;
    termattr.c_cflag &= ~PARENB;  // no parity
    termattr.c_iflag &= ~INPCK;
    termattr.c_cflag &= ~CSTOPB;  // stopbits 1

    tcsetattr(fd, TCSANOW, &termattr);
    tcflush  (fd, TCIOFLUSH);
    return fd;
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
//          fd = open(ctxt->dev, O_WRONLY);
            fd = open_serial_port(ctxt->dev);
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
            add_nmea_buf(ctxt, buf, n);
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
    ctxt->callback = cb;
    ctxt->cbparams = params;
    pthread_create(&ctxt->read_thread, NULL, read_thread_proc, (void*)ctxt);
    return ctxt;
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
    char  dev_name[PATH_MAX] = "/dev/ttyS1";
    void *nmea = NULL;

    if (argc < 2) {
        printf("\n");
        printf("usage: gpsnmea dev\n");
        printf("  dev - the serial dev name, example /dev/ttyS1.\n");
    }

    if (argc >= 2) strcpy(dev_name, argv[1]);
    printf("serial dev: %s\n", dev_name);
    printf("\n");

    nmea = nmea_init(dev_name, NULL, NULL);
    getchar();
    nmea_exit(nmea);
    return 0;
}
