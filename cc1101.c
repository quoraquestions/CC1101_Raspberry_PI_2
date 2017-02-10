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

#define SPI_DEVNAME "/dev/spidev0.0"
#define SPI_SPEED (500000) /*500Kbit*/
 
#define BCM2708_PERI_BASE        (0x3F000000)
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
 
 
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


//{
/****************** Radio Regs **********************/
/* ------------------------------------------------------------------
*        CONFIGURATION RADIO REGISTERS
*   -----------------------------------------------------------
*/

#define ISM_US
/*Setup radio (SmartRF Studio) */
#define WBSL_AP_ADDRESS                             (0xCA)

#define WBSL_SETTING_FIFOTHR     0x07    /* FIFOTHR  - RX FIFO and TX FIFO thresholds */
/* Set the SYNC words to be used */
#define WBSL_SETTING_SYNC1       0xD3   /* Modem configuration. */
#define WBSL_SETTING_SYNC0       0x91   /* Modem configuration. */
#define WBSL_SETTING_PKTLEN      0xFE   /* Packet length. */
#define WBSL_SETTING_PKTCTRL1    0x06   /* Packet automation control. */
#define WBSL_SETTING_PKTCTRL0    0x45   /* Packet automation control. */
#define WBSL_SETTING_ADDR        WBSL_AP_ADDRESS  /* Device address. */

#ifdef ISM_EU
/* 869.525MHz */
    #define WBSL_SETTING_FREQ2    0x21      /*  Frequency control word, high byte */
    #define WBSL_SETTING_FREQ1    0x71      /*  Frequency control word, middle byte */
    #define WBSL_SETTING_FREQ0    0x7A      /*  Frequency control word, low byte */
    #define WBSL_SETTING_CHANNR     0       /* Channel number. */
    #define WBSL_SETTING_PA_TABLE0  0x67    /* PA output power setting. Due to RF regulations (+1.1dBm) */
#else
    #ifdef ISM_US
/* 902MHz (CHANNR=20 --> 906MHz) */
       #define WBSL_SETTING_FREQ2    0x22     /*  Frequency control word, high byte */
       #define WBSL_SETTING_FREQ1    0xB1     /*  Frequency control word, middle byte */
       #define WBSL_SETTING_FREQ0    0x3B    /*  Frequency control word, low byte */
       #define WBSL_SETTING_CHANNR     20      /* Channel number. */
       #define WBSL_SETTING_PA_TABLE0  0x51  /* PA output power setting. Due to RF regulations (+1.3dBm) */
    #else
        #ifdef ISM_LF
/* 433.30MHz */
           #define WBSL_SETTING_FREQ2    0x10      /*  Frequency control word, high byte */
           #define WBSL_SETTING_FREQ1    0xB0      /*  Frequency control word, middle byte */
           #define WBSL_SETTING_FREQ0    0x71      /*  Frequency control word, low byte */
           #define WBSL_SETTING_CHANNR     0       /* Channel number. */
           #define WBSL_SETTING_PA_TABLE0  0x61    /* PA output power setting. Due to RF regulations (+1.4dBm) */
        #else
            #error "Wrong ISM band specified (valid are ISM_LF, ISM_EU and ISM_US)"
        #endif /* ISM_LF */
   #endif     /* ISM_US */
#endif         /* ISM_EU */


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

/* To leave the CC1101 in a workable state after the Wireless Update has completed,
 * Some RF registers configured for WBSL need to be reset to their default values.
 */

#define RESET_VALUE_MCSM2        0x07
#define RESET_VALUE_PKTCTRL1     0x04
#define RESET_VALUE_ADDR         0x00
#define RESET_VALUE_CHANNR       0x00

#define RESET_VALUE_FSCTRL1      0x0F
#define RESET_VALUE_FSCTRL0      0x00
#define RESET_VALUE_MDMCFG4      0x8C
#define RESET_VALUE_MDMCFG3      0x22
#define RESET_VALUE_MDMCFG2      0x02
#define RESET_VALUE_MDMCFG1      0x22
#define RESET_VALUE_MDMCFG0      0xF8
#define RESET_VALUE_DEVIATN      0x47
#define RESET_VALUE_MCSM1        0x30
#define RESET_VALUE_MCSM0        0x04
#define RESET_VALUE_FOCCFG       0x36
#define RESET_VALUE_BSCFG        0x6C
#define RESET_VALUE_AGCCTRL2     0x03
#define RESET_VALUE_AGCCTRL1     0x40
#define RESET_VALUE_AGCCTRL0     0x91
#define RESET_VALUE_FREND1       0x56
#define RESET_VALUE_FSCAL3       0xA9
#define RESET_VALUE_FSCAL2       0x0A
#define RESET_VALUE_FSCAL1       0x20
#define RESET_VALUE_FSCAL0       0x0D
#define RESET_VALUE_TEST0        0x0B
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
    {  CHANNR,    WBSL_SETTING_CHANNR    },
    {  FSCTRL1,   WBSL_SETTING_FSCTRL1   },
    {  FSCTRL0,   WBSL_SETTING_FSCTRL0   },
    {  FREQ2,     WBSL_SETTING_FREQ2     },
    {  FREQ1,     WBSL_SETTING_FREQ1     },
    {  FREQ0,     WBSL_SETTING_FREQ0     },
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

int default_spi_config(int fd)
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
    return 0;
}

static void transfer(int fd, char strobe)
{
    int ret;
    struct spi_ioc_transfer tr;
    uint8_t tx[2] = {0,0};
    uint8_t rx[2] = {0,0};
    tx[0] = strobe;
    memset(&tr, 0, sizeof(tr));
    GPIO_CLR = 1<<(GPIO_CHIP_SEL);
    while(GET_GPIO(GPIO_RDY))
    {
        int i = 0;
        printf("Waiting for Chiprdy ....%d\n", i++);
    }

    tr.tx_buf = &tx;
    tr.rx_buf = &rx;
    tr.len = sizeof(tx);
    tr.delay_usecs = 30;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message");

    for (ret = 0; ret < tr.len; ret++) {
        printf("%02X ", rx[ret]);
    }
    GPIO_SET = 1 << (GPIO_CHIP_SEL);

    puts("");
}

#define READ_CMD_BIT (1<<7)
#define BURST_CMD_BIT (1<<6)

static void transfer_burst_read_all(int fd)
{
    int ret;
    int count = 48;
    struct spi_ioc_transfer tr;
    uint8_t tx[count];
    uint8_t rx[count];
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    tx[0] = 0 | READ_CMD_BIT | BURST_CMD_BIT;
    memset(&tr, 0, sizeof(tr));
    GPIO_CLR = 1<<(GPIO_CHIP_SEL);
    while(GET_GPIO(GPIO_RDY))
    {
        int i = 0;
        printf("Waiting for Chiprdy ....%d\n", i++);
    }

    tr.tx_buf = &tx;
    tr.rx_buf = &rx;
    tr.len = sizeof(tx);
    tr.delay_usecs = 90;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message");

    for (ret = 0; ret < tr.len; ret++) {
        printf("%02X ", rx[ret]);
    }
    GPIO_SET = 1 << (GPIO_CHIP_SEL);

    puts("");
}



void cfg_gpio()
{
    INP_GPIO(GPIO_CHIP_SEL); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(GPIO_CHIP_SEL);
    GPIO_SET = 1<<GPIO_CHIP_SEL;

    INP_GPIO(GPIO_RDY); // must use INP_GPIO before we can use OUT_GPIO
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

int main()
{
    int fd;
    char x;
    setup_io();
    cfg_gpio();
    fd = open_spi(SPI_DEVNAME);
    cc1101_reset();
    default_spi_config(fd);
    transfer(fd,0x30);
    transfer_burst_read_all(fd);

    transfer(fd,0x70);
    transfer(fd,0xf0);
    transfer(fd,0xf1);
    close_spi(fd);

    return 0;
}
