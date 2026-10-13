#include <stdint.h>
#include <string.h>

#include "ch32v00x.h"
//#include "ch32v00x_rcc.h"
//#include "ch32v00x_gpio.h"
//#include "ch32v00x_usart.h"
//#include "ch32v00x_adc.h"
//#include "ch32v00x_tim.h"

#define SYSCLK_HZ   (HSI_VALUE)
#define BAUDRATE    115200

#define TICK_US(n)  ((n) * SYSCLK_HZ / 1000000)
#define TICK_MS(n)  (TICK_US(n) * 1000)

#define PLOT_SIZE   31
#define RXQ_SIZE    8
#define T_HODL      TICK_MS(100)

#define CTRL(c)     ((c) - '@')
#define REDRAW      CTRL('R')
#define TMODE       CTRL('T')
#define SAMPLING    CTRL('S')
#define RUN_STOP    ' '

#define PLOTROW     7
#define PLOTCOL     3   // (in double-width characters)

#define CTRL_MASK   (1 << REDRAW | 1 << TMODE | 1 << SAMPLING)


static const char init_term[] =
    "\e<"       // exit from vt52 emulation mode (because who knows?)
    "\ec"       // reset terminal
    "\e[3J"     // clear scrollback (software emulators)
    "\e[?25l";  // hide cursor (VT-220 and later models)


static uint8_t  rxq[RXQ_SIZE];
static volatile uint8_t rxq_w, rxq_r, rxirq;


static uint8_t  run, sampling, tmode, hpos, tlevel;
static uint8_t  plot[PLOT_SIZE], plot_upd, n_acq, hodl;
static uint32_t t_hodl;

// external low-level acquire function (acquire.s)
int _acquire(void *buf, int buf_sz, int lvl, int tail, volatile uint8_t *stop);

// -------------------------------
// hw peripherals helper functions
// -------------------------------

// gpio init
static void gpio_pin_mode(GPIO_TypeDef *gp, int pin, int mode)
{
    uint32_t    pinmask = 1 << pin;
    uint32_t    shift = pin << 2;
    uint32_t    mask = 0x0f << shift;

    gp->CFGLR = gp->CFGLR & ~mask | mode << shift & mask;

    if ((mode & GPIO_Mode_IPD) == GPIO_Mode_IPD) gp->BCR = pinmask; // PD
    if ((mode & GPIO_Mode_IPU) == GPIO_Mode_IPU) gp->BSHR = pinmask;// PU
}

static void set_sampling(int s)
{
    const uint32_t m = 1 << 13;
    RCC->CFGR0 = RCC->CFGR0 & ~m | (s ? m : 0);
}

// -----------------------------------
// hardware peripherals initialization
// -----------------------------------
static void init_hw()
{
    const int ADC_Continous_Enable = 2;
    const int ADC_CTLR2_ADON_Set = 1;
    const int ADC_CTLR2_EXTTRIG_SWSTART_Set = 0x500000;
    const int ADC_CTLR2_RSTCAL_Set = 8;
    const int ADC_NbrOfChannels = 1;
    const int ADC_CTLR2_CAL_Set = 4;
    const int ADC_Channel = 2;

    const int USART1_CTLR1_UE_Set = 0x2000;    // USART Enable Mask

    const int STK_STE   =   1 << 0; // turn on system counter
    const int STK_STCLK =   1 << 2; // HCLK as aclock source
    const int STK_STRE  =   1 << 3; // auto-reload count-enable bit

    // ---------------------------------------------------------------
    // initialize SysTick so it will run at CPU frequency continuously
    // ---------------------------------------------------------------
    SysTick->CMP = 0xffffffff;
    SysTick->CTLR |= STK_STE | STK_STCLK | STK_STRE;

    // enable peripheral clocks
    RCC->APB2PCENR |= (
        RCC_APB2Periph_GPIOC    |
        RCC_APB2Periph_GPIOD    |
        RCC_APB2Periph_ADC1     |
        RCC_APB2Periph_USART1   |
        RCC_APB2Periph_AFIO
    );

    // USART1  partial mapping USART1_RM=10 (TX: PD6, RX: PD5)
    AFIO->PCFR1 |= GPIO_PartialRemap2_USART1;

    //  gpio pin modes:
    //  PC2     GPIO    virtual ground
    //  PC4     ADC1    CH2
    //  PD6     USART1  TX
    //  PD5     USART1  RX
    GPIOC->BCR = GPIO_Pin_2;
    gpio_pin_mode(GPIOC, 2, GPIO_Mode_Out_PP | GPIO_Speed_2MHz);
    gpio_pin_mode(GPIOC, 4, GPIO_Mode_AIN);
    gpio_pin_mode(GPIOD, 5, GPIO_Mode_IPU);
    gpio_pin_mode(GPIOD, 6, GPIO_Mode_AF_PP | GPIO_Speed_10MHz); // uart tx

    // USART1 initialization
    // ---------------------
    rxq_r = rxq_w = 0;

    USART1->CTLR1 |= USART_Mode_Rx | USART_Mode_Tx; // 8N1 is there after reset
    USART1->BRR = (SYSCLK_HZ + BAUDRATE/2) / BAUDRATE;
    USART1->CTLR1 |= 1 << (USART_IT_RXNE & 0x1f);   // rx empty interrupt
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CTLR1 |= USART1_CTLR1_UE_Set;

    // ADC initialization
    // ------------------

    // reset sequence
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

    // adc configuration
    ADC1->CTLR1 |= ADC_Mode_Independent;
    ADC1->CTLR2 |= ADC_ExternalTrigConv_None | ADC_Continous_Enable;
    ADC1->CTLR2 |= ADC_DataAlign_Left;
    ADC1->RSQR1 |= ADC_NbrOfChannels-1 << 20;

    // channel configuration
    ADC1->SAMPTR2 |= ADC_SampleTime_9Cycles << 3*ADC_Channel;
    ADC1->RSQR3 |= ADC_Channel;

    // enable ADC before calibration
    ADC1->CTLR2 |= ADC_CTLR2_ADON_Set;

    // reset calibration
    ADC1->CTLR2 |= ADC_CTLR2_RSTCAL_Set;
    while (ADC1->CTLR2 & ADC_CTLR2_RSTCAL_Set);

    // calibrate
    ADC1->CTLR2 |= ADC_CTLR2_CAL_Set;
    while (ADC1->CTLR2 & ADC_CTLR2_CAL_Set);

    // start continuous conversion
    ADC1->CTLR2 |= ADC_CTLR2_EXTTRIG_SWSTART_Set;
}

__attribute__ ((interrupt("WCH-Interrupt-fast")))
void USART1_IRQHandler()
{
    static uint8_t esc;
    int c = USART1->DATAR;

    if (c == '\e') {
        esc = '\e';
        c = 0;
    } else if (esc == '\e' && c == '[') {
        esc = '[';
        c = 0;
    } else if (esc == '[') {
        esc = 0;
        if (c < 'A' || c > 'D') c = 0;
    } else if (esc) {
        esc = 0;
        c = 0;
    } else if (c < 32) {
        if (1<<c & ~CTRL_MASK) c = 0;
    } else if (c != RUN_STOP) {
        c = 0;
    }

    if (c) {
        int w = rxq_w, s = w - rxq_r & RXQ_SIZE-1;
        if (s < RXQ_SIZE) {
            rxq[w++] = c;
            rxq_w = w & RXQ_SIZE-1;
            rxirq = 1;
        }
    }
}

// //////////////////////

static int rxc(void)
{
    int c = 0, r = rxq_r, s = rxq_w - r & RXQ_SIZE-1;
    if (s) {
        c = rxq[r++];
        rxq_r = r & RXQ_SIZE-1;
    }
    return c;
}

static void txc(int c)
{
    if (c == '\n') txc('\r');
    while (!(USART1->STATR & USART_FLAG_TXE));
    USART1->DATAR = c;
}

static uint32_t tick()
{
    return SysTick->CNT;
}

static uint32_t tick_elapsed(uint32_t t)
{
    return tick() - t;
}

// lvl != 0, rising edge at "lvl"
// lvl == 0, just sample and we're done
__attribute__ ((noinline))
static int acquire(int lvl, int tail)
{
    uint8_t buf[PLOT_SIZE];
    int p, j;

    rxirq = 0;
    p = _acquire(buf, sizeof buf, lvl, tail, &rxirq);
    if (rxirq) return -1;

    // copy results only if rly captured
    for (j = 0; j < PLOT_SIZE; ++j) {
        if (p == PLOT_SIZE) p = 0;
        plot[j] = plot[j] << 4 | buf[p++] >> 4;
    }
    return 0;
}

static void pf(const char *s)
{
    int c;
    while (c = *s++)
        if (c == '`')
            switch (c = *s++) {
            case '*':   pf("\e[1m"); break;    // bold
            case '~':   pf("\e[7m"); break;    // inverse
            case '_':   pf("\e[2m"); break;    // low intensity
            default:    pf("\e[m"); --s;       // none
            }
        else
            txc(c);
}

static void pnum(int n)
{
    int u = n / 10;
    if (u) txc(u + '0');
    txc('0' + n - u * 10);
}

static void rowcol(int row, int col)
{
    pf("\e[");
    pnum(row);
    txc(';');
    pnum(col);
    txc('H');
}

static void pfrc(int row, int col, const char *s)
{
    rowcol(row, col);
    pf(s);
}

static void pfn(const char *s, int n)
{
    while (n) n -= !*s++;
    pf(s);
}

static void pfrcn(int row, int col, const char *s, int n)
{
    rowcol(row, col);
    pfn(s, n);
}

static char cgrid(int y, int x)
{
    return  x % 6 == 0 && y % 5 == 0    ?   '+' :
            x % 6 == 0                  ?   '|' :
            y % 5 == 0                  ?   '-' :
                                            ' ' ;
}

static void init_grid()
{
    int x, y;
    for (y = 0; y < 16; ++y) {
        pfrc(y + PLOTROW, PLOTCOL, "\e#6`_");
        for (x = 0; x < PLOT_SIZE; ++x)
            txc(cgrid(y, x));

        if (!(y % 5)) {
            pf("  `");
            pfn("3.3V\0""2.2V\0""1.1V\0""0.0V", y / 5);
        }
    }
    pf("\n\e#6`");
}

static void plot_poll(void)
{
    if (!plot_upd) return;

    if (plot_upd > PLOT_SIZE) {
        // update
        pf("`*");

        while (rxq_w == rxq_r && plot_upd > PLOT_SIZE) {
            int x = PLOT_SIZE*2 - plot_upd--,
                y = plot[x] & 15,
                y0 = n_acq < 2 ? ~y : plot[x] >> 4;

            if (y0 != y) pfrc(y + PLOTROW, x + PLOTCOL, "*");
        }
    } else if (n_acq > 1) {
        // cleanup
        pf("`_");

        while (rxq_w == rxq_r && plot_upd) {
            int x = PLOT_SIZE - plot_upd--,
                y = plot[x] & 15, y0 = plot[x] >> 4;

            if (y0 != y) {
                rowcol(y0 + PLOTROW, x + PLOTCOL);
                txc(cgrid(y0, x));
            }
        }
    } else {
        plot_upd = 0;
    }
    pf("`");
}

// scope operation
static void scope_poll(int c)
{
    switch (c) {
    case REDRAW:
        pf(init_term);
        pf( "     `*Ctrl+S` :  Sampling rate:\n"
            "     `*Ctrl+T` :  Trigger type:\n"
            "     `*Space ` :  Run/stop:\n\n"
            "     `_Ctrl+R :  Redraw all`\n"

        );
        init_grid();

        hodl = plot_upd = n_acq = 0;

        --hpos;         scope_poll('C');
        --tlevel;       scope_poll('B');
        tmode ^= 1;     scope_poll(TMODE);
        run ^= 1;       scope_poll(RUN_STOP);
        sampling ^= 1;  scope_poll(SAMPLING);
        break;

    // cursor up
    case 'A':
        if (tlevel < 2) break;
        tlevel -= 2;
    // cursor dn
    case 'B':
        if (tlevel > 13) break;
        ++tlevel;
        pfrc(PLOTROW + tlevel, PLOTCOL + PLOT_SIZE, "`*-`\e");
        txc(c == 'A' ? 'D' : 'M');
        txc(127);
        break;
    // cursor rt
    case 'C':
        if (hpos > PLOT_SIZE-3) break;
        hpos += 2;
    // cursor lt
    case 'D':
        if (hpos < 2) break;
        hpos -= 2;
        pfrc(PLOTROW + 16, PLOTCOL + hpos, " `*|` ");
        ++hpos;
        break;
    case SAMPLING:
        sampling = !sampling;
        set_sampling(sampling);
        pfrcn(1, 32, "600 ksps\0""150 ksps", sampling);
        pfrcn(24, 32, "M: 10 us\0M: 40 us", sampling);
        break;
    case TMODE:
        tmode = !tmode;
        pfrcn(2, 32, "freerun    \0rising edge", tmode);
        break;
    case RUN_STOP:
        run = !run;
        pfrcn(3, 32, "stop\0`*RUN ", run);
        break;
    }

    // frame rate (therefore data rate) limiting
    if (hodl && tick_elapsed(t_hodl) > T_HODL)
        hodl = 0;

    // data acquisition
    if (run && !plot_upd && !hodl) {
        int lvl = tmode ? (tlevel ^ 15) << 4 : 0;
        int tail = PLOT_SIZE - hpos;
        if (!acquire(lvl, tail)) {
            ++n_acq;

            plot_upd = PLOT_SIZE * 2;
            hodl = 1;
            t_hodl = tick();
        }
    }

    plot_poll();
}

static void init_scope()
{
    sampling    =   3;
    hpos        =   5;
    tlevel      =   2;
    run         =   0;
    tmode       =   0;

    scope_poll(REDRAW);
}

void main()
{
    init_hw();
    init_scope();
    for(;;) scope_poll(rxc());
}
