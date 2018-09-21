#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <limits.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>

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

volatile sig_atomic_t g_exit = 0;
static void sig_handler(int sig)
{
    switch (sig) {
    case SIGINT:
    case SIGKILL:
        printf("\nexit gpsmuxd now ...\n");
        g_exit = 1;
        break;
    }
}

int main(int argc, char *argv[])
{
    char dev_name [PATH_MAX] = "/dev/ttyS1";
    char fifo_name[PATH_MAX] = "";
    int  fd_serial = -1;
    int *fd_fifos  = NULL;
    int  num_fifos =  5;
    int  i;

    if (argc < 3) {
        printf("\n");
        printf("usage: gpsmuxd dev num\n");
        printf("  dev - the serial dev name, example /dev/ttyS1.\n");
        printf("  num - the number of gps virtual files, example 5.\n\n");
    }

    if (argc >= 2) strcpy(dev_name, argv[1]);
    if (argc >= 3) num_fifos = atoi(argv[2]);

    printf("serial dev: %s\n", dev_name );
    printf("gpsmux num: %d\n", num_fifos);
    printf("\n");

    fd_serial = open_serial_port(dev_name);
    if (fd_serial < 0) {
        return 0;
    }

    fd_fifos = calloc(1, num_fifos * sizeof(int));
    if (!fd_fifos) {
        printf("failed to allocate memory for fifos !\n");
        goto end;
    }

    for (i=0; i<num_fifos; i++) {
        sprintf(fifo_name, "/tmp/gpsmux%d", i);
        unlink(fifo_name);
        mkfifo(fifo_name, 0666);
    }

    // register sigint
    signal(SIGINT , sig_handler);
    signal(SIGKILL, sig_handler);
    signal(SIGPIPE, sig_handler);

    while (!g_exit) {
        struct timeval tv;
        fd_set fds;
        int    ret;

        FD_ZERO(&fds);
        FD_SET(fd_serial, &fds);

        /* timeout */
        tv.tv_sec  = 1;
        tv.tv_usec = 0;

        ret = select(fd_serial + 1, &fds, NULL, NULL, &tv);
        switch (ret) {
        case  0: printf("select again !\n"); continue;
        case -1: printf("select error !\n"); goto end;
        }

        if (FD_ISSET(fd_serial, &fds)) {
            char buf[32];
            int  n = read(fd_serial, buf, sizeof(buf));
//          fwrite(buf, n, 1, stdout);
//          printf("\n");
            for (i=0; i<num_fifos; i++) {
                if (fd_fifos[i] <= 0) {
                    sprintf(fifo_name, "/tmp/gpsmux%d", i);
                    fd_fifos[i] = open(fifo_name, O_WRONLY|O_NONBLOCK);
                }
//              printf("%d ", fd_fifos[i]);
                write(fd_fifos[i], buf, n);
            }
//          printf("\n");
        }
    }

end:
    for (i=0; i<num_fifos; i++) {
        if (fd_fifos[i] > 0) close(fd_fifos[i]);
        sprintf(fifo_name, "/tmp/gpsmux%d", i);
        unlink (fifo_name);
    }
    if (fd_serial > 0) close(fd_serial);
    if (fd_fifos) free(fd_fifos);

    printf("gpsmuxd exit !\n");
    return 0;
}



