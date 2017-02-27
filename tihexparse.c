#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#define MAX_MEM_SEGS (20)
#define MAX_SEG_SIZE (32*1024)

int split_size;

typedef struct segments {
    int seg_type;
    char payload[MAX_SEG_SIZE];
} segments_t;

segments_t segments[MAX_MEM_SEGS];
unsigned int current_segments = 0;
unsigned int nr_segments = 0;

void createpayload(char *f)
{
    int fd = open(f,"r");
    if (fd < 0)
        perror("Failed to open %s\n", f);

       

}


int main(int argc, chat **argv)
{
    if (argc != 2)
        perror("Error in arguments");
    else
        createpayload(argv[1]);
    return 0;
}
