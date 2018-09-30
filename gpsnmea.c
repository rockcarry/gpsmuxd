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

    int               gpssatidx;
    GPS_SATELLITE     gpssatlist[16];
} CONTEXT;

static int str2int(char *str, int len)
{
    char tmp[12];
    len = len < 11 ? len : 11;
    memcpy(tmp, str, len);
    tmp[len] = '\0';
    return atoi(tmp);
}

static double dm2ddd(double dm)
{
    int    degrees = (int)(dm / 100);
    double minutes = dm - degrees * 100;
    return degrees + minutes / 60.0;
}

static void do_nmea_parse(CONTEXT *ctxt, char *sentence)
{
    #define MAX_TOKEN_NUM  22
    char *tokens[MAX_TOKEN_NUM] = { sentence };
    int   gpsinuse[12];
    int   i, j;

//  printf("%s", sentence);
    //++ split sentence to tokens
    for (i=0,j=1; sentence[i]&&j<MAX_TOKEN_NUM; i++) {
        if (sentence[i] == ',' || sentence[i] == '*') {
            sentence[i] = '\0';
            tokens[j++] = sentence + i + 1;
        }
    }
    for (; j<MAX_TOKEN_NUM; j++) tokens[j] = "";
    //-- split sentence to tokens

    if (memcmp(sentence, "$GPRMC", 6) == 0) {
        ctxt->gps_status.latitude  = (tokens[4][0] == 'S' ? -1 : 1) * dm2ddd(strtod(tokens[3], NULL));
        ctxt->gps_status.longitude = (tokens[6][0] == 'W' ? -1 : 1) * dm2ddd(strtod(tokens[5], NULL));
        ctxt->gps_status.speed     = (float)(strtod(tokens[7], NULL) * 1.852 / 3.6);
        ctxt->gps_status.course    = (float)(strtod(tokens[8], NULL));
        ctxt->gps_status.datetime.tm_year = str2int(tokens[9] + 4, 2) + 2000 - 1900;
        ctxt->gps_status.datetime.tm_mon  = str2int(tokens[9] + 2, 2) - 1;
        ctxt->gps_status.datetime.tm_mday = str2int(tokens[9] + 0, 2);
        ctxt->gps_status.datetime.tm_hour = str2int(tokens[1] + 0, 2);
        ctxt->gps_status.datetime.tm_min  = str2int(tokens[1] + 2, 2);
        ctxt->gps_status.datetime.tm_sec  = str2int(tokens[1] + 4, 2);
        if (tokens[2][0] == 'V') {
            ctxt->gps_status.fixstatus = 0;
        } else if (ctxt->gps_status.fixstatus == 0) {
            ctxt->gps_status.fixstatus = GPS_FIXED;
        }
        memcpy(ctxt->gps_status.satellites, ctxt->gpssatlist, sizeof(GPS_SATELLITE) * 16);
        ctxt->callback(ctxt->cbparams, &ctxt->gps_status);
    } else if (memcmp(sentence, "$GPGGA", 6) == 0) {
        ctxt->gps_status.datetime.tm_hour = str2int(tokens[1] + 0, 2);
        ctxt->gps_status.datetime.tm_min  = str2int(tokens[1] + 2, 2);
        ctxt->gps_status.datetime.tm_sec  = str2int(tokens[1] + 4, 2);
        ctxt->gps_status.latitude  = (tokens[3][0] == 'S' ? -1 : 1) * dm2ddd(strtod(tokens[2], NULL));
        ctxt->gps_status.longitude = (tokens[5][0] == 'W' ? -1 : 1) * dm2ddd(strtod(tokens[4], NULL));
        ctxt->gps_status.hdop      = (float)strtod(tokens[8], NULL);
        ctxt->gps_status.altitude  = (float)strtod(tokens[9], NULL);
        ctxt->gps_status.inuse     = atoi(tokens[7]);
    } else if (memcmp(sentence, "$GPGSA", 6) == 0) {
        ctxt->gps_status.fixstatus = tokens[2][0] > '1' ? tokens[2][0] - '0' : 0;
        ctxt->gps_status.pdop      = (float)strtod(tokens[15], NULL);
        ctxt->gps_status.hdop      = (float)strtod(tokens[16], NULL);
        ctxt->gps_status.vdop      = (float)strtod(tokens[17], NULL);
        for (i=0; i<12; i++) gpsinuse[i] = atoi(tokens[3+i]);
    } else if (memcmp(sentence, "$GPGSV", 6) == 0) {
        if (atoi(tokens[2]) == 1) {
            memset(&ctxt->gpssatlist, 0, sizeof(GPS_SATELLITE) * 16);
            ctxt->gpssatidx = 0;
        }
        ctxt->gps_status.inview = atoi(tokens[3]);
        for (i=0; i<4; i++) {
            if (ctxt->gpssatidx >= 16) break;
            ctxt->gpssatlist[ctxt->gpssatidx].prn       = tokens[i*4+4][0] ? atoi(tokens[i*4+4]) : -1;
            ctxt->gpssatlist[ctxt->gpssatidx].elevation = tokens[i*4+5][0] ? atoi(tokens[i*4+5]) : -1;
            ctxt->gpssatlist[ctxt->gpssatidx].azimuth   = tokens[i*4+6][0] ? atoi(tokens[i*4+6]) : -1;
            ctxt->gpssatlist[ctxt->gpssatidx].snr       = tokens[i*4+7][0] ? atoi(tokens[i*4+7]) :  0;
            for (j=0; j<12; j++) {
                if (gpsinuse[j] == ctxt->gpssatlist[ctxt->gpssatidx].prn) {
                    ctxt->gpssatlist[ctxt->gpssatidx].inuse = 1;
                }
            }
            ctxt->gpssatidx++;
        }
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
//          fd = open(ctxt->dev, O_RDONLY);
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
    return NULL;
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

void* nmea_gps_status(void *ctx)
{
    CONTEXT *ctxt = (CONTEXT*)ctx;
    return &ctxt->gps_status;
}

void nmea_print(GPS_STATUS *pgs)
{
    struct tm loc_tm   = {0};
    time_t    utc_time =  0;
    char      strtime[20];
    int       i, j;

#if 0
    utc_time = timegm(&pgs->datetime);
    localtime_r(&utc_time, &loc_tm);
#else
    loc_tm = pgs->datetime;
    loc_tm.tm_year += 1900;
    loc_tm.tm_mon  += 1;
#endif
    sprintf(strtime, "%04d-%02d-%02d/%02d:%02d:%02d", loc_tm.tm_year, loc_tm.tm_mon, loc_tm.tm_mday, loc_tm.tm_hour, loc_tm.tm_min, loc_tm.tm_sec);
    printf("+----------------------------------------------------------+\n");
    printf("  fix: %d  satenum: %d/%d  time: %s\n",
            pgs->fixstatus, pgs->inuse, pgs->inview, strtime);
    printf("  (%lf, %lf, %.1f)  speed: %-3.1f  course: %-3.1f \n",
            pgs->latitude, pgs->longitude, pgs->altitude, pgs->speed, pgs->course);
    printf("  pdop: %-2.1f   hdop: %-2.1f   vdop: %-2.1f\n",
            pgs->pdop, pgs->hdop, pgs->vdop);
    printf("+----------------------------------------------------------+\n");
    for (i=0; i<pgs->inview; i++) {
        printf("%3d %3d %3d %3d %c ",
            pgs->satellites[i].prn,
            pgs->satellites[i].snr,
            pgs->satellites[i].elevation,
            pgs->satellites[i].azimuth,
            pgs->satellites[i].inuse ? '*' : ' ');
        for (j=0; j<pgs->satellites[i].snr; j++) {
            printf("|");
        }
        printf("\n");
    }
    printf("\n");
}

#if TEST
static void gps_nmea_callback(void *params, GPS_STATUS *pgs)
{
    nmea_print(pgs);
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

    nmea = nmea_init(dev_name, gps_nmea_callback, NULL);
    getchar();
    nmea_exit(nmea);
    return 0;
}
#endif
