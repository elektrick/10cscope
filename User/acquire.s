    .section  .text, "ax", @progbits
	.globl  _acquire
	.align  2

    .equ    ADC_BASE,       0x40012400
    .equ    ADC_STATR,      0x00
    .equ    ADC_RDATAR,     0x4C
    .equ    ADC_EOC,        1 << 1

/*  int capture(
        uint8_t             *buf,           // a0
        int                 buf_size,       // a1
        int                 lvl,            // a2
        int                 tail,           // a3
        volatile uint8_t    *stop           // a4
    );

    returns index of the last written buffer position. return value MUST
    be ignored when rxirq is non-zero after _acquire was called 
*/
_acquire:
    /* SETUP */
    add     t1, a0, a1          /* t1 = buf + buf_size (wrap position)  */
    mv      t0, a0              /* t0 = pbuf (write pointer)            */
    li      t2, ADC_BASE
    /* CLEAN-UP AND RESYNC */
    jal     a5, _sample
    jal     a5, _sample
    addi    t0, t0, -2          /* discard those two samples */

    /* HISTORY BEFORE TRIGGER */
1:  jal     a5, _sample
    bne     t0, t1, 1b          /* capture entire buffer */
    beqz    a2, 0f              /* lvl == 0 means no trigger so we're done */
    /* TRIGGER */
1:  jal     a5, _sample         /* wait until sample value is below lvl */
    bgt     a0, a2, 1b
1:  jal     a5, _sample         /* wait until sample value is above/equal lvl */
    ble     a0, a2, 1b
    /* TAIL */
1:  beqz    a3, 0f
    jal     a5, _sample
    addi    a3, a3, -1
    j       1b
    /* RETVAL */
0:  sub     a0, t1, t0          /* pwrap - pbuf */
    sub     a0, a1, a0          /* index = bufsz - (pwrap - pbuf)... */
    addi    a0, a0, -1          /* ... -1 */
    ret

_sample:
    bne     t0, t1, 1f              /* write pointer will be    */
    sub     t0, t0, a1              /* wrapped (if it needs to) */
1:  addi    t0, t0, 1               /* and then incremented     */

    lb      a0, (a4)                /* stop flag?               */
    bnez    a0, 0b                  /* exit immediately         */

1:  lw      a0, ADC_STATR(t2)       /* adc status register      */
    andi    a0, a0, ADC_EOC         /* check if data ready      */
    beqz    a0, 1b                  /* wait until ready         */

    lbu     a0, 1 + ADC_RDATAR(t2)  /* read 8-bit ADC value     */
    sb      a0, -1(t0)              /* save it into the buffer  */
    jr      a5
