#if 0
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
