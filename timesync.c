#define _GNU_SOURCE
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
#include "fw.h"
struct timespec sleepval = { .tv_nsec = 50000000 };
#define SPI_DEVNAME "/dev/spidev0.0"
#define SPI_SPEED (5000000) /*500Kbit*/
 
#define BCM2708_PERI_BASE        (0x3F000000)
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
 
//#define GD0_RX_OVERFLOW_DETECT

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
 
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
#define GPIO_CHIP_SEL (22)
#define GPIO_RDY (25)
#define GPIO_SCK (11)
#define GPIO_MOSI (10)
#define GPIO_GD0 (24)
#define GPIO_GD2 (23)


//{
/****************** Radio Regs **********************/
/* ------------------------------------------------------------------
*        CONFIGURATION RADIO REGISTERS
*   -----------------------------------------------------------
*/

#define WBSL_AP_ADDRESS                             (0xCA)
#define WBSL_SETTING_FIFOTHR     0x07    /* FIFOTHR  - RX FIFO and TX FIFO thresholds */
/* Set the SYNC words to be used */
#define WBSL_SETTING_SYNC1       0xD3   /* Modem configuration. */
#define WBSL_SETTING_SYNC0       0x91   /* Modem configuration. */
#define WBSL_SETTING_PKTLEN      0xFE   /* Packet length. */
#define WBSL_SETTING_PKTCTRL1    0x04   /* Packet automation control. */
#define WBSL_SETTING_PKTCTRL0    0x45   /* Packet automation control. */
#define WBSL_SETTING_ADDR        WBSL_AP_ADDRESS  /* Device address. */
#define WBSL_SETTING_FSCTRL1    0x0C   /* (IF) Frequency synthesizer control. */
#define WBSL_SETTING_FSCTRL0    0x00   /* Frequency synthesizer control. */
#define WBSL_SETTING_MDMCFG4    0x2D    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG3    0x3B    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG2    0x13    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG1    0x22    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG0    0xF8    /* Modem configuration. */
#define WBSL_SETTING_DEVIATN     0x62    /* Modem deviation setting (when GSK modulation is enabled). */
#define WBSL_SETTING_MCSM2       0x07
#define WBSL_SETTING_MCSM1       0x3C
#define WBSL_SETTING_MCSM0       0x18   /* Main Radio Control State Machine configuration. */
#define WBSL_SETTING_FOCCFG      0x1D   /* Frequency Offset Compensation Configuration. */
#define WBSL_SETTING_BSCFG       0x1C   /*  Bit synchronization Configuration. */
#define WBSL_SETTING_AGCCTRL2    0xC7   /* AGC control. */
#define WBSL_SETTING_AGCCTRL1    0x00   /*  AGC control. */
#define WBSL_SETTING_AGCCTRL0    0xB0   /* AGC control. */
#define WBSL_SETTING_WOREVT1     0x87
#define WBSL_SETTING_WOREVT0     0x6B
#define WBSL_SETTING_WORCTRL     0xF8
#define WBSL_SETTING_FREND1      0xB6   /* Front end RX configuration. */
#define WBSL_SETTING_FREND0      0x10   /* Front end TX configuration. */
#define WBSL_SETTING_FSCAL3      0xEA   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSCAL2      0x2A   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSCAL1      0x00   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSCAL0      0x1F   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSTEST      0x59
#define WBSL_SETTING_PTEST       0x7F
#define WBSL_SETTING_AGCTEST     0x3F
#define WBSL_SETTING_TEST2       0x88   /* Various test settings. */
#define WBSL_SETTING_TEST1       0x31   /* Various test settings. */
#define WBSL_SETTING_TEST0       0x09   /* Various test settings. */

/***************************End Radio Regs_Values*****************************/
//}
/* ------------------------------------------------------------------------------------------------
 *          SPI ADDRESS SPACE
 * ------------------------------------------------------------------------------------------------
 */


/* status registers */
#define PARTNUM     0x30      /*  PARTNUM    - Chip ID */
#define VERSION     0x31      /*  VERSION    - Chip ID */
#define FREQEST     0x32      /*  FREQEST    – Frequency Offset Estimate from demodulator */
#define LQI         0x33      /*  LQI        – Demodulator estimate for Link Quality */
#define RSSI        0x34      /*  RSSI       – Received signal strength indication */
#define MARCSTATE   0x35      /*  MARCSTATE  – Main Radio Control State Machine state */
#define WORTIME1    0x36      /*  WORTIME1   – High byte of WOR time */
#define WORTIME0    0x37      /*  WORTIME0   – Low byte of WOR time */
#define PKTSTATUS   0x38      /*  PKTSTATUS  – Current GDOx status and packet status */
#define VCO_VC_DAC  0x39      /*  VCO_VC_DAC – Current setting from PLL calibration module */
#define TXBYTES     0x3A      /*  TXBYTES    – Underflow and number of bytes */
#define RXBYTES     0x3B      /*  RXBYTES    – Overflow and number of bytes */

/* burst write registers */
#define PA_TABLE0   0x3E      /*  PA_TABLE0 - PA control settings table */
#define TXFIFO      0x3F      /*  TXFIFO  - Transmit FIFO */
#define RXFIFO      0x3F      /*  RXFIFO  - Receive FIFO */

/* command strobe registers */
#define SRES        0x30      /*  SRES    - Reset chip. */
#define SFSTXON     0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define SXOFF       0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define SCAL        0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define SRX         0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define STX         0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define SIDLE       0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define SRSVD       0x37      /*  SRVSD   - Reserved.  Do not use. */
#define SWOR        0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define SPWD        0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define SFRX        0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define SFTX        0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define SWORRST     0x3C      /*  SWORRST - Reset real time clock. */
#define SNOP        0x3D      /*  SNOP    - No operation. Returns status byte. */

/************************Radio Register addresses ******************/
//{
/* RF registers */
#define IOCFG2      0x00      /*  IOCFG2   - GDO2 output pin configuration  */
#define IOCFG1      0x01      /*  IOCFG1   - GDO1 output pin configuration  */
#define IOCFG0      0x02      /*  IOCFG1   - GDO0 output pin configuration  */
#define FIFOTHR     0x03      /*  FIFOTHR  - RX FIFO and TX FIFO thresholds */
#define SYNC1       0x04      /*  SYNC1    - Sync word, high byte */
#define SYNC0       0x05      /*  SYNC0    - Sync word, low byte */
#define PKTLEN      0x06      /*  PKTLEN   - Packet length */
#define PKTCTRL1    0x07      /*  PKTCTRL1 - Packet automation control */
#define PKTCTRL0    0x08      /*  PKTCTRL0 - Packet automation control */
#define ADDR        0x09      /*  ADDR     - Device address */
#define CHANNR      0x0A      /*  CHANNR   - Channel number */
#define FSCTRL1     0x0B      /*  FSCTRL1  - Frequency synthesizer control */
#define FSCTRL0     0x0C      /*  FSCTRL0  - Frequency synthesizer control */
#define FREQ2       0x0D      /*  FREQ2    - Frequency control word, high byte */
#define FREQ1       0x0E      /*  FREQ1    - Frequency control word, middle byte */
#define FREQ0       0x0F      /*  FREQ0    - Frequency control word, low byte */
#define MDMCFG4     0x10      /*  MDMCFG4  - Modem configuration */
#define MDMCFG3     0x11      /*  MDMCFG3  - Modem configuration */
#define MDMCFG2     0x12      /*  MDMCFG2  - Modem configuration */
#define MDMCFG1     0x13      /*  MDMCFG1  - Modem configuration */
#define MDMCFG0     0x14      /*  MDMCFG0  - Modem configuration */
#define DEVIATN     0x15      /*  DEVIATN  - Modem deviation setting */
#define MCSM2       0x16      /*  MCSM2    - Main Radio Control State Machine configuration */
#define MCSM1       0x17      /*  MCSM1    - Main Radio Control State Machine configuration */
#define MCSM0       0x18      /*  MCSM0    - Main Radio Control State Machine configuration */
#define FOCCFG      0x19      /*  FOCCFG   - Frequency Offset Compensation configuration */
#define BSCFG       0x1A      /*  BSCFG    - Bit Synchronization configuration */
#define AGCCTRL2    0x1B      /*  AGCCTRL2 - AGC control */
#define AGCCTRL1    0x1C      /*  AGCCTRL1 - AGC control */
#define AGCCTRL0    0x1D      /*  AGCCTRL0 - AGC control */
#define WOREVT1     0x1E      /*  WOREVT1  - High byte Event0 timeout */
#define WOREVT0     0x1F      /*  WOREVT0  - Low byte Event0 timeout */
#define WORCTRL     0x20      /*  WORCTRL  - Wake On Radio control */
#define FREND1      0x21      /*  FREND1   - Front end RX configuration */
#define FREND0      0x22      /*  FREDN0   - Front end TX configuration */
#define FSCAL3      0x23      /*  FSCAL3   - Frequency synthesizer calibration */
#define FSCAL2      0x24      /*  FSCAL2   - Frequency synthesizer calibration */
#define FSCAL1      0x25      /*  FSCAL1   - Frequency synthesizer calibration */
#define FSCAL0      0x26      /*  FSCAL0   - Frequency synthesizer calibration */
#define RCCTRL1     0x27      /*  RCCTRL1  - RC oscillator configuration */
#define RCCTRL0     0x28      /*  RCCTRL0  - RC oscillator configuration */
#define FSTEST      0x29      /*  FSTEST   - Frequency synthesizer calibration control */
#define PTEST       0x2A      /*  PTEST    - Production test */
#define AGCTEST     0x2B      /*  AGCTEST  - AGC test */
#define TEST2       0x2C      /*  TEST2    - Various test settings */
#define TEST1       0x2D      /*  TEST1    - Various test settings */
#define TEST0       0x2E      /*  TEST0    - Various test settings */
//}
/************************END Radio Register addresses ******************/

#define WBSL_GDO_SYNC     6
#define WBSL_GDO_PA_PD    27  /* low when transmit is active, low during sleep */

/***************************Wbsl radio config**************************/
static const uint8_t wbslRadioCfg[][2] =
{
    /* internal radio configuration */
    {  MCSM2,     WBSL_SETTING_MCSM2     },
    {  MCSM1,     WBSL_SETTING_MCSM1     },
    {  MCSM0,     WBSL_SETTING_MCSM0     },
    {  SYNC1,     WBSL_SETTING_SYNC1     },
    {  SYNC0,     WBSL_SETTING_SYNC0     },
    {  PKTLEN,    WBSL_SETTING_PKTLEN    },
    {  PKTCTRL1,  WBSL_SETTING_PKTCTRL1  },
    {  PKTCTRL0,  WBSL_SETTING_PKTCTRL0  },
    {  ADDR,      WBSL_SETTING_ADDR      },
    {  FIFOTHR,   WBSL_SETTING_FIFOTHR   },
    {  WOREVT1,   WBSL_SETTING_WOREVT1   },
    {  WOREVT0,   WBSL_SETTING_WOREVT0   },
    {  WORCTRL,   WBSL_SETTING_WORCTRL   },
    /* imported SmartRF radio configuration */
    {  FSCTRL1,   WBSL_SETTING_FSCTRL1   },
    {  FSCTRL0,   WBSL_SETTING_FSCTRL0   },
    {  MDMCFG4,   WBSL_SETTING_MDMCFG4   },
    {  MDMCFG3,   WBSL_SETTING_MDMCFG3   },
    {  MDMCFG2,   WBSL_SETTING_MDMCFG2   },
    {  MDMCFG1,   WBSL_SETTING_MDMCFG1   },
    {  MDMCFG0,   WBSL_SETTING_MDMCFG0   },
    {  DEVIATN,   WBSL_SETTING_DEVIATN   },
    {  FOCCFG,    WBSL_SETTING_FOCCFG    },
    {  BSCFG,     WBSL_SETTING_BSCFG     },
    {  AGCCTRL2,  WBSL_SETTING_AGCCTRL2  },
    {  AGCCTRL1,  WBSL_SETTING_AGCCTRL1  },
    {  AGCCTRL0,  WBSL_SETTING_AGCCTRL0  },
    {  FREND1,    WBSL_SETTING_FREND1    },
    {  FREND0,    WBSL_SETTING_FREND0    },
    {  FSCAL3,    WBSL_SETTING_FSCAL3    },
    {  FSCAL2,    WBSL_SETTING_FSCAL2    },
    {  FSCAL1,    WBSL_SETTING_FSCAL1    },
    {  FSCAL0,    WBSL_SETTING_FSCAL0    },
    {  AGCTEST,   WBSL_SETTING_AGCTEST   },
    {  PTEST,     WBSL_SETTING_PTEST     },
    {  FSTEST,    WBSL_SETTING_FSTEST    },
    {  TEST2,     WBSL_SETTING_TEST2     },
    {  TEST1,     WBSL_SETTING_TEST1     },
    {  TEST0,     WBSL_SETTING_TEST0     },
};

typedef enum freq_settings { ISM_EU =0, ISM_US, ISM_LF, ISM_MAX} fset_t;

uint8_t band_regs[] = {
    FREQ2,    
    FREQ1,    
    FREQ0,    
    CHANNR,   
    PA_TABLE0
};

uint8_t freq_config [ISM_MAX][5] =
{
    //EU
    {
        0x21,      /*  Frequency control word, high byte */
        0x71,      /*  Frequency control word, middle byte */
        0x7A,      /*  Frequency control word, low byte */
        0,      /* Channel number. */
        0xc0    /* PA output power setting. Due to RF regulations (+1.1dBm) */
    },
    //US
    {
        0x22,      /*  Frequency control word, high byte */
        0xB1,      /*  Frequency control word, middle byte */
        0x3B,      /*  Frequency control word, low byte */
        20,      /* Channel number. */
        0xc0    /* PA output power setting. Due to RF regulations (+1.1dBm) */
    },
    //LF
    {
        0x10,      /*  Frequency control word, high byte */
        0xB0,      /*  Frequency control word, middle byte */
        0x71,      /*  Frequency control word, low byte */
        0,      /* Channel number. */
        0x61    /* PA output power setting. Due to RF regulations (+1.1dBm) */
    }
};

fset_t default_freq = ISM_US;




/************************End of WBSL Radio Config */
int  mem_fd;
void *gpio_map;
 
// I/O access
volatile unsigned *gpio;
 
 
// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
 
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
 
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock
 
void setup_io();
 
 
 
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }
 
   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );
 
   close(mem_fd); //No need to keep mem_fd open after mmap
 
   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }
 
   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;
 
 
} // setup_io

const char *spi_dev_name = SPI_DEVNAME;

int open_spi(const char *devname)
{
    int fd = open(devname, O_RDWR);
    if (fd < 0)
    {
        perror("Open Failed\n");
        exit(-1);
    }
    return fd;
}

int close_spi(int fd)
{
    if (close(fd))
    {
        perror("Close Failed\n");
        exit(-1);
    }
    return 0;
}

void default_spi_config(int fd)
{
    int x; 
    x = SPI_MODE_0;
    if(ioctl(fd, SPI_IOC_WR_MODE, &x))
    {
        perror("Failed to set mode to SPI_MODE_0");
    }
    x = SPI_SPEED;
    if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &x))
    {
        perror("Failed to set speed");
    }
    x = 8;  
    if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &x))
    {
        perror("Failed to set bpw");
    }
    else if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &x))
    {
        perror("Failed to get bwp");
    }
    else
    {
        printf("BPW: %d\n", x);
    }
}

#define READ_CMD_BIT (1<<7)
#define BURST_CMD_BIT (1<<6)


void write_reg(int fd, uint8_t addr, uint8_t value, uint8_t *response)
{
    int ret;
    struct spi_ioc_transfer tr;
    #define N_RESP_BYTES (2) 
    uint8_t *wr, *rd;
    wr = malloc(N_RESP_BYTES);
    rd = malloc(N_RESP_BYTES);
    memset(&tr, 0, sizeof(tr));
    memset(wr, 0, 10);
    memset(rd, 0, 10);

    wr[0] = (addr);
    wr[1] = value;

    GPIO_CLR = 1<<(GPIO_CHIP_SEL);
    while(GET_GPIO(GPIO_RDY))
    {
        int i = 0;
        printf("Waiting for Chiprdy ....%d\n", i++);
    }
#if 0
    printf("Writing %02x val %02x\n", addr, value);
#endif
    tr.tx_buf = (unsigned long) wr;
    tr.rx_buf = (unsigned long) rd;
    tr.len = N_RESP_BYTES;
    tr.delay_usecs = 90;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message");

#if 0
    for (ret = 0; ret < tr.len; ret++) {
        printf("..%02X.. ", rd[ret]);
    }
#endif
    if (response)
        response[0] = rd[1];
    GPIO_SET = 1 << (GPIO_CHIP_SEL);
    free(wr);
    free(rd);
}

void read_reg_burst(int fd, uint8_t addr, uint8_t len, uint8_t *dest)
{
    int ret;
    uint32_t i;
    struct spi_ioc_transfer tr;
    #define N_RESP_BYTES (2) 
    uint8_t *wr, *rd;
    wr = malloc(len + 1);
    rd = malloc(len + 1);
    memset(&tr, 0, sizeof(tr));
    memset(wr, 0, 10);
    memset(rd, 0, 10);
    memset(wr, 0xde, sizeof(wr[0])*len);
    wr[0] = (addr) | BURST_CMD_BIT | READ_CMD_BIT;

    GPIO_CLR = 1<<(GPIO_CHIP_SEL);

    while(GET_GPIO(GPIO_RDY))
    {
        printf("Waiting for Chiprdy ....%d\n", i++);
    }
#if 0
    printf("Burst Reading %02x\n", addr);
#endif
    tr.tx_buf = (unsigned long) wr;
    tr.rx_buf = (unsigned long) rd;
    tr.len = len + 1;
    tr.delay_usecs = 90;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message");
#if 0
    for (i = 0; i < tr.len; i++) {
        printf("..%02X.. ", rd[ret]);
    }
#endif
    if (dest)
    {
        int i;
        for (i = 0; i < len; i++)
            dest[i] = rd[i + 1];
    }
    GPIO_SET = 1 << (GPIO_CHIP_SEL);
    free(wr);
    free(rd);
    return;
}

uint8_t write_reg_burst(int fd, uint8_t addr, uint8_t *value, uint8_t len)
{
    int ret;
    uint8_t *ptr;
    struct spi_ioc_transfer tr;
    #define N_RESP_BYTES (2) 
    uint8_t *wr, *rd;
    wr = malloc(len + 1);
    rd = malloc(len + 1);
    memset(&tr, 0, sizeof(tr));
    memset(wr, 0, 10);
    memset(rd, 0, 10);

    wr[0] = (addr) | BURST_CMD_BIT;
    ptr = &wr[1];
    for (int i = 0; i < len; i++)
        *ptr++ = *value++;
    GPIO_CLR = 1<<(GPIO_CHIP_SEL);
    while(GET_GPIO(GPIO_RDY))
    {
        int i = 0;
        printf("Waiting for Chiprdy ....%d\n", i++);
    }
    printf("Writing %02x \n", addr);
    tr.tx_buf = (unsigned long) wr;
    tr.rx_buf = (unsigned long) rd;
    tr.len = len + 1;
    tr.delay_usecs = 90;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message");

#if 0
    for (ret = 0; ret < tr.len; ret++) {
        printf("..%02X.. ", rd[ret]);
    }
#endif
    GPIO_SET = 1 << (GPIO_CHIP_SEL);
    free(wr);
    free(rd);
    return rd[1];
}

uint8_t read_reg(int fd, uint8_t addr, uint8_t *status)
{
    int ret;
    struct spi_ioc_transfer tr;
    #define N_RESP_BYTES (2) 
    uint8_t *wr, *rd;
    wr = calloc(N_RESP_BYTES , sizeof(uint8_t));
    rd = calloc(N_RESP_BYTES , sizeof(uint8_t));
    memset(&tr, 0, sizeof(tr));

    wr[0] = (addr & 0x3f) | READ_CMD_BIT;
    wr[1] = 0x3e;
    if (addr >= 0x30)
    {
        wr[0] |= BURST_CMD_BIT;
    }

    GPIO_CLR = 1<<(GPIO_CHIP_SEL);
    while(GET_GPIO(GPIO_RDY))
    {
        int i = 0;
        printf("Waiting for Chiprdy ....%d\n", i++);
    }

    tr.tx_buf = (unsigned long)wr;
    tr.rx_buf = (unsigned long)rd;
    tr.len = N_RESP_BYTES;
    tr.delay_usecs = 90;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message");
#if 0
    for (ret = 0; ret < tr.len; ret++) {
        printf("%02X ", rd[ret]);
    }
#endif
    GPIO_SET = 1 << (GPIO_CHIP_SEL);
    if (status)
        status[0] = rd[0];
    return rd[1];
}

void strobe_cmd(int fd, uint8_t cmd, uint8_t *response)
{
    assert(cmd <= 0x3d && cmd >= 0x30);        
    write_reg(fd, cmd, 0, response); 
     
}


void cfg_gpio()
{
    INP_GPIO(GPIO_CHIP_SEL); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(GPIO_CHIP_SEL);
    GPIO_SET = 1<<GPIO_CHIP_SEL;

    INP_GPIO(GPIO_RDY); // must use INP_GPIO before we can use OUT_GPIO

    INP_GPIO(GPIO_GD0);
    INP_GPIO(GPIO_GD2);
}

void delay_loop(int count)
{
    volatile int x = count;
    while(x--)
    ;

}

void cc1101_reset()
{
    /*SCK HIGH*/
    INP_GPIO(GPIO_SCK); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(GPIO_SCK);
    GPIO_SET = 1<<GPIO_SCK;

    /*MOSI LOW */
    INP_GPIO(GPIO_MOSI); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(GPIO_MOSI);
    GPIO_CLR = 1<<GPIO_MOSI;

    GPIO_CLR = 1<<(GPIO_CHIP_SEL);
    delay_loop(100);
    GPIO_SET = 1<<(GPIO_CHIP_SEL);
    delay_loop(8000);

    GPIO_CLR = 1<<(GPIO_CHIP_SEL);

    while(GET_GPIO(GPIO_RDY))
        printf("Waiting for GPIO_RDY");

    INP_GPIO(GPIO_SCK); // must use INP_GPIO before we can use OUT_GPIO
    SET_GPIO_ALT(GPIO_SCK,0);

    INP_GPIO(GPIO_MOSI); // must use INP_GPIO before we can use OUT_GPIO
    SET_GPIO_ALT(GPIO_MOSI, 0);
}

void cc1101_cfg_band(int fd, fset_t fset)
{
    uint8_t i;
    assert(fset >= ISM_EU && fset < ISM_MAX);
    for ( i = 0; i < sizeof(band_regs)/sizeof(band_regs[0]); i++)
    {
        write_reg(fd, band_regs[i], freq_config[fset][i], NULL);
    }
    
}

void cc1101_cfg_regs(int fd)
{
    uint32_t i;
    for (i = 0; i < sizeof(wbslRadioCfg)/sizeof(wbslRadioCfg[0]); i++)
    {
        uint8_t reg = wbslRadioCfg[i][0];
        uint8_t val = wbslRadioCfg[i][1];
        write_reg(fd, reg, val, NULL); 
    }
    write_reg(fd, IOCFG0, WBSL_GDO_SYNC, NULL);
    write_reg(fd, IOCFG2, 7, NULL);
}


void cc1101_initialize(int fd, fset_t fset)
{

    cc1101_reset();
    uint8_t response;
    strobe_cmd(fd, SRES, &response);
    while (response & 0x70)
    {
        printf("Waiting for Radio to idle....%02X\n", response);
        strobe_cmd(fd, SIDLE, &response);
    }

    cc1101_cfg_regs(fd);
    cc1101_cfg_band(fd, fset);
}

void rxon(int fd)
{
    strobe_cmd(fd, SRX, NULL); 
}


void rxoff(int fd)
{
    uint8_t response;
    strobe_cmd(fd, SFRX, &response);
    strobe_cmd(fd, SIDLE, &response);
    while (response & 0x70)
    {
        printf("Waiting for Radio to idle....%02X\n", response);
        strobe_cmd(fd, SIDLE, &response);
    }

}

void txstart(int fd)
{
    strobe_cmd(fd, STX, NULL); 
}

void txoff(int fd)
{
    uint8_t response;
    strobe_cmd(fd, SFTX, &response);
    strobe_cmd(fd, SIDLE, &response);
    while (response & 0x70)
    {
        printf("Waiting for Radio to idle....%02X\n", response);
        strobe_cmd(fd, SIDLE, &response);
    }

}


uint8_t CheckRx(int fd)
{
    uint8_t bytes = 0;
    uint8_t marc_state = read_reg(fd, MARCSTATE, NULL) & 0x1f;
    if (marc_state == 0x11)
    {
        printf("RX overflow\n");
        rxoff(fd);
        rxon(fd);
    }
    if (!GET_GPIO(GPIO_GD2))
    {
        return 0;
    }
    else 
    {
        int8_t val = bytes = read_reg(fd,RXBYTES, NULL);
        if (val)
        {
            int8_t tmpval = 0;
            while (val != (tmpval = read_reg(fd, RXBYTES, NULL)))
                val = tmpval;
            val = val & 0x7f;
            return bytes = val;
        }
        else
        {
            /* False Packet Detected ? */
            rxoff(fd); 
            rxon(fd);
            val = 0;
            bytes = 0;
            printf("Radio Reset due to false detection !!!\n");
        }

      }

    return bytes;
}

void ReceivePkt(int fd, uint8_t *pktbuf, uint8_t bytes)
{
    read_reg_burst(fd, RXFIFO, bytes, pktbuf); 
}

void TransmitPkt(int fd, uint8_t *payload, uint8_t len)
{
    /*convert to State machine*/
    rxoff(fd);
    assert(payload[0] == len - 1);
    write_reg_burst(fd, TXFIFO, payload, len);
    txstart(fd);
    while(!GET_GPIO(GPIO_GD0));
    while(GET_GPIO(GPIO_GD0));
}

uint8_t RxPktBuf[256];
uint8_t WaitRx(int fd)
{
    uint8_t len = 0;
    while (!(len = CheckRx(fd)))
        ;
    return len;
}

uint8_t GetPacket(int fd, uint8_t *pktbuf)
{
    uint8_t len;
    rxon(fd);
    if((len = WaitRx(fd)))
    {
        uint32_t j = 0;
        uint8_t clqi = 0, rssi = 0;
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        printf("Time %ld.%09ld ", now.tv_sec, now.tv_nsec);
        printf("Len:%x ", len); 
        ReceivePkt(fd, pktbuf, len);
        clqi = pktbuf[pktbuf[0] + 2];
        rssi =  pktbuf[pktbuf[0] + 1];
        printf("RSSI:%02X LQI : %02X ", rssi, (clqi & 0x7f));
        if (clqi & 0x80)
            printf("CRC:OK\n");
        else
        {
            printf("CRC:Fail\n");
            len = 0;
        }
        for (j = 0; j < len; j++)
        {
            if ( (4*j + 1) % 80 == false)
                printf("\n");
           printf(" %02X ", pktbuf[j]);
        }
        printf("\n");
        fflush(stdout);
        //nanosleep(&sleepval, NULL);
    }
    return len;
}

typedef struct wbsl_header {
    uint8_t da;
    uint8_t ta;
} __attribute__((packed)) wbsl_header_t;

typedef struct wbsl_disc_pkt{
    uint8_t len;
    wbsl_header_t hdr;
    uint8_t vbatt;
    uint8_t payload[4];
}__attribute__((packed)) wbsl_disc_pkt_t;

#define WBSL_ACK_STATUS_SUCCESS (1)
#define WBSL_ACK_STATUS_FAIL    (0)

typedef struct wbsl_disc_ack {
    uint8_t len;
    wbsl_header_t hdr;
    uint8_t status;
}__attribute__((packed)) wbsl_disc_ack_t; 

typedef struct wbsl_data_ack {
    uint8_t len;
    wbsl_header_t hdr;
    uint8_t status;
    uint8_t packethi;
    uint8_t packetlo;
}__attribute__((packed)) wbsl_data_ack_t;

uint8_t linked, epaddr;

void wait_for_discovery(int fd, int force)
{
    uint8_t discoveryPayload[]= {0xBA,0x5E,0xBA,0x11};
    uint8_t len = 0;
    wbsl_disc_pkt_t *disc = (wbsl_disc_pkt_t *)RxPktBuf;
    while (!linked || force)
    {
        len = GetPacket(fd, RxPktBuf);
        if (len < 7)
            continue;
        if (!disc->hdr.da && !memcmp(discoveryPayload, disc->payload, sizeof(discoveryPayload)))
        {
            printf("Discovered %02X\n", disc->hdr.ta);
            epaddr = disc->hdr.ta;
            linked = true;
        }
        else
        {
            printf("Not Disc\n");
            continue;
        }
    }
}


void send_disc_ack(int fd)
{
    wbsl_disc_ack_t dack = {
        .len = sizeof(dack) - sizeof(dack.len), 
        .hdr = { .da = epaddr, 
            .ta = WBSL_AP_ADDRESS}, 
        .status = WBSL_ACK_STATUS_SUCCESS };
    TransmitPkt(fd, (uint8_t *)&dack, sizeof(dack));
}

uint8_t init_packet[] = { 5, 0xad, 0xca, 0, 0 ,0}; 
int start_update(int fd)
{
    int i;
    int j;
    uint8_t retx = 0;
    uint8_t len = 0;
    struct timespec ack_timeout = { .tv_nsec = 20000};
    for ( i = 0; i <  sizeof(npkts)/sizeof(npkts[0]); i++)
    {  
        wait_for_discovery(fd, false);
        rxoff(fd);
        nanosleep(&ack_timeout, NULL);
        send_disc_ack(fd);
        init_packet[3] = npkts[i] & 0xff;
        init_packet[4] = (npkts[i] >> 8 ) & 0xff;
        while(1) {
            retx++;
            init_packet[5] = 1;
            TransmitPkt(fd, init_packet, sizeof(init_packet));
            nanosleep(&ack_timeout, NULL);
            if((len = GetPacket(fd, RxPktBuf)))
            {
               printf("Success Init Packet\n");
               retx = 0;
               break;
            }
        }

        nanosleep(&ack_timeout, NULL);

        for(j = 0; j < npkts[i]; j++)
        {
            TransmitPkt(fd, packets[i][j], packets[i][j][0]+1);
            if((len = GetPacket(fd, RxPktBuf)))
            {
                wbsl_data_ack_t *da = RxPktBuf;
                printf("Packet %x", (da->packethi << 8) + da->packetlo);
                if (da->status == WBSL_ACK_STATUS_SUCCESS)
                {
                    printf("*Succeeded\n");
                    continue;
                }
                else
                {
                    printf("* Failed\n");
                    uint16_t nack_nr  = (da->packethi << 8) + da->packetlo;
                    j = nack_nr;
                }
            }
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    int fd;
    fset_t fset = ISM_US;
    uint8_t status;

    if (argc == 2)
    {
        if (!strcmp(argv[1], "US"))
            fset = ISM_US;
        else if (!strcmp(argv[1], "EU"))
            fset = ISM_EU;
        else if (!strcmp(argv[1], "LF"))
            fset = ISM_LF;
        else 
        {
            printf("Unspecified Band, defaulting to 902MHz/US band\n");
            fset = ISM_US;
        }
    }

    setup_io();
    cfg_gpio();
    fd = open_spi(SPI_DEVNAME);
    default_spi_config(fd);
    cc1101_initialize(fd, fset);
    for( int i = 0; i < 0x3e; i++)
    printf("Reg %02x, Val %02x Status %02x\n", i, read_reg(fd, i, &status), status);
    start_update(fd);
#if 0
    rxon(fd);
    while(1)
    {
        uint32_t len = 0;
        if((len = CheckRx(fd)))
        {
            uint32_t j = 0;
            uint8_t clqi = 0, rssi = 0;
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);
            printf("Time %ld.%09ld ", now.tv_sec, now.tv_nsec);
            printf("Len:%x ", len); 
            ReceivePkt(fd, pktbuf, len);
            clqi = pktbuf[pktbuf[0] + 2];
            rssi =  pktbuf[pktbuf[0] + 1];
            printf("RSSI:%02X LQI : %02X ", rssi, (clqi & 0x7f));
            if (clqi & 0x80)
                printf("CRC:OK\n");
            else
            {
                printf("CRC:Fail\n");
                continue;
            }
            for (j = 0; j < len; j++)
            {
                if ( (4*j + 1) % 80 == false)
                    printf("\n");
                printf(" %02X ", pktbuf[j]);
            }
            printf("\n");
            fflush(stdout);
            //nanosleep(&sleepval, NULL);
        }
    }
#endif
    close_spi(fd);

    return 0;
}


