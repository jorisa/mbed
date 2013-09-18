/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "can_api.h"

#include "cmsis.h"
#include "pinmap.h"
#include "error.h"

#include <math.h>
#include <string.h>

#define MSG_OBJ_MAX			0x0020
#define DLC_MAX				8

#define RX_MSG_ID			0x100
#define RX_EXT_MSG_ID		0x00100000

#define TX_MSG_ID			0x200
#define TX_EXT_MSG_ID		0x00200000

/* BRP+1 = Fpclk/(CANBitRate * QUANTAValue)
   QUANTAValue = 1 + (Tseg1+1) + (Tseg2+1)
   QUANTA value varies based on the Fpclk and sample point
   e.g. (1) sample point is 87.5%, Fpclk is 48Mhz
   the QUANTA should be 16
        (2) sample point is 90%, Fpclk is 12.5Mhz
   the QUANTA should be 10 
   Fpclk = Fclk /APBDIV
   or
   BitRate = Fcclk/(APBDIV * (BRP+1) * ((Tseg1+1)+(Tseg2+1)+1))
*/ 	
/*
 * Bit Timing Values for 24MHz clk frequency
 */
/*    LPC_CAN->BT = 0x0101;	*/	/* 1Mbps with 8M sysclk */
/*    LPC_CAN->BT = 0x4501;	*/	/* 1Mbps with 24M sysclk */
/*    LPC_CAN->BT = 0x5801;	*/	/* 500kbps with 16M sysclk */
/*    LPC_CAN->BT = 0x5803;	*/	/* 250kbps with 16M sysclk */

#define BITRATE100K8MHZ           0x00000113
#define BITRATE125K8MHZ           0x0000010F
#define BITRATE250K8MHZ           0x00000107
#define BITRATE500K8MHZ           0x00000103
#define BITRATE1000K8MHZ          0x00000101

#define BITRATE100K16MHZ          0x00005809
#define BITRATE125K16MHZ          0x00005807
#define BITRATE250K16MHZ          0x00005803
#define BITRATE500K16MHZ          0x00005801

#define BITRATE100K24MHZ          0x00007E09
#define BITRATE125K24MHZ          0x0000450F
#define BITRATE250K24MHZ          0x00004507
#define BITRATE500K24MHZ          0x00004503
#define BITRATE1000K24MHZ         0x00004501

#define CAN_STATUS_INTERRUPT      0x8000    /* 0x0001-0x0020 are the # of the message object */
                                            /* 0x8000 is the status interrupt */
/* CAN Message interface register definitions */
/* bit field of IF command request n register */
#define IFCREQ_BUSY               0x8000    /* 1 is writing is progress, cleared when RD/WR done */

/* CAN CTRL register */
#define CTRL_INIT		(1 << 0)
#define CTRL_IE			(1 << 1) 
#define CTRL_SIE		(1 << 2)
#define CTRL_EIE		(1 << 3) 
#define CTRL_DAR		(1 << 5)
#define CTRL_CCE		(1 << 6) 
#define CTRL_TEST		(1 << 7)
	
/* CAN Status register */
#define STAT_LEC		(0x7 << 0)
#define STAT_TXOK		(1 << 3) 
#define STAT_RXOK		(1 << 4)
#define STAT_EPASS		(1 << 5) 
#define STAT_EWARN		(1 << 6)
#define STAT_BOFF		(1 << 7)

/* bit field of IF command mask register */
#define	DATAB	(1 << 0)   /* 1 is transfer data byte 4-7 to message object, 0 is not */ 
#define	DATAA	(1 << 1)   /* 1 is transfer data byte 0-3 to message object, 0 is not */ 
#define	TREQ	(1 << 2)   /* 1 is set the TxRqst bit, 0 is not */
#define	INTPND	(1 << 3)       
#define	CTRL	(1 << 4)   /* 1 is transfer the CTRL bit to the message object, 0 is not */
#define	ARB		(1 << 5)   /* 1 is transfer the ARB bits to the message object, 0 is not */
#define	MASK	(1 << 6)   /* 1 is transfer the MASK bit to the message object, 0 is not */
#define	WR		(1 << 7)   /* 0 is READ, 1 is WRITE */
#define RD      0x0000

/* bit field of IF mask 2 register */
#define	MASK_MXTD	(1 << 15)     /* 1 extended identifier bit is used in the RX filter unit, 0 is not */ 
#define	MASK_MDIR	(1 << 14)     /* 1 direction bit is used in the RX filter unit, 0 is not */

/* bit field of IF identifier 2 register */
#define	ID_MVAL		(1 << 15)     /* Message valid bit, 1 is valid in the MO handler, 0 is ignored */ 
#define	ID_MTD		(1 << 14)     /* 1 extended identifier bit is used in the RX filter unit, 0 is not */ 
#define	ID_DIR		(1 << 13)     /* 1 direction bit is used in the RX filter unit, 0 is not */

/* bit field of IF message control register */
#define	NEWD		(1 << 15)     /* 1 indicates new data is in the message buffer.  */ 
#define	MLST		(1 << 14)     /* 1 indicates a message loss. */ 
#define	INTP		(1 << 13)     /* 1 indicates message object is an interrupt source */
#define UMSK    	(1 << 12)     /* 1 is to use the mask for the receive filter mask. */
#define	TXIE		(1 << 11)     /* 1 is TX interrupt enabled */ 
#define	RXIE		(1 << 10)     /* 1 is RX interrupt enabled */
#define	ROEN		(1 << 9)      /* 1 is remote frame enabled */
#define TXRQ    	(1 << 8)      /* 1 is TxRqst enabled */
#define	EOB			(1 << 7)      /* End of buffer, always write to 1 */ 
#define	DLC			0x000F        /* bit mask for DLC */ 

#define ID_STD_MASK		0x07FF		
#define ID_EXT_MASK		0x1FFFFFFF
#define DLC_MASK		0x0F



// Type definition to hold a CAN message
struct CANMsg {
    unsigned int  reserved1 : 16;
    unsigned int  dlc       :  4; // Bits 16..19: DLC - Data Length Counter
    unsigned int  reserved0 : 10;
    unsigned int  rtr       :  1; // Bit 30: Set if this is a RTR message
    unsigned int  type      :  1; // Bit 31: Set if this is a 29-bit ID message
    unsigned int  id;             // CAN Message ID (11-bit or 29-bit)
    unsigned char data[8];        // CAN Message Data Bytes 0-7
};
typedef struct CANMsg CANMsg;

//static uint32_t can_irq_id = 0;
//static can_irq_handler irq_handler;

static uint32_t can_disable(can_t *obj) {
    uint32_t sm = LPC_CAN->CNTL;
    LPC_CAN->CNTL |= 1;
    return sm;
}

static inline void can_enable(can_t *obj) {
    if (LPC_CAN->CNTL & 1) {
        LPC_CAN->CNTL &= ~(1);
    }
}

int can_mode(can_t *obj, CanMode mode)
{
    return 0; // not implemented
}

#if 0
static inline void can_irq(uint32_t icr, uint32_t index) {
    uint32_t i;
    
    for(i = 0; i < 8; i++)
    {
        if((can_irq_id != 0) && (icr & (1 << i)))
        {
            switch (i) {
                case 0: irq_handler(can_irq_id, IRQ_RX);      break;
                case 1: irq_handler(can_irq_id, IRQ_TX);      break;
                case 2: irq_handler(can_irq_id, IRQ_ERROR);   break;
                case 3: irq_handler(can_irq_id, IRQ_OVERRUN); break;
                case 4: irq_handler(can_irq_id, IRQ_WAKEUP);  break;
                case 5: irq_handler(can_irq_id, IRQ_PASSIVE); break;
                case 6: irq_handler(can_irq_id, IRQ_ARB);     break;
                case 7: irq_handler(can_irq_id, IRQ_BUS);     break;
                case 8: irq_handler(can_irq_id, IRQ_READY);   break;
            }
        }
    }
}
#endif

// Have to check that the CAN block is active before reading the Interrupt
// Control Register, or the mbed hangs
void can_irq_n() {
    //uint32_t icr;

    /*if(LPC_SYSCON->PCONP & (1 << 13)) {
        //icr = LPC_CAN1->ICR & 0x1FF;
        can_irq(icr, 0);
    }*/
}

// Register CAN object's irq handler
void can_irq_init(can_t *obj, can_irq_handler handler, uint32_t id) {
#if 0
  	irq_handler = handler;
    can_irq_id = id;
#endif	
}

// Unregister CAN object's irq handler
void can_irq_free(can_t *obj) {
#if 0
		LPC_CAN->CNTL &= ~(1 << 1); // Disable Interrupts :)

    can_irq_id = 0;
    NVIC_DisableIRQ(CAN_IRQn);
#endif
}

// Clear or set a irq
void can_irq_set(can_t *obj, CanIrqType type, uint32_t enable) {
#if 0
	uint32_t ier;
    
    switch (type) {
        case IRQ_RX:      ier = (1 << 0); break;
        case IRQ_TX:      ier = (1 << 1); break;
        case IRQ_ERROR:   ier = (1 << 2); break;
        case IRQ_OVERRUN: ier = (1 << 3); break;
        case IRQ_WAKEUP:  ier = (1 << 4); break;
        case IRQ_PASSIVE: ier = (1 << 5); break;
        case IRQ_ARB:     ier = (1 << 6); break;
        case IRQ_BUS:     ier = (1 << 7); break;
        case IRQ_READY:   ier = (1 << 8); break;
        default: return;
    }
    
    // Put CAN in Reset Mode.
    LPC_CAN->CNTL |= 1;
    if(enable == 0) {
        LPC_CAN->CNTL &= ~(1 << ier);
    }
    else {
        LPC_CAN->CNTL &= ~(1 << ier);
    }
    // Take it out of reset...
    LPC_CAN->CNTL &= ~(1);
    
    // Enable NVIC if at least 1 interrupt is active
    NVIC_SetVector(CAN_IRQn, (uint32_t) &can_irq_n);
    NVIC_EnableIRQ(CAN_IRQn);
#endif
}

// This table has the sampling points as close to 75% as possible. The first
// value is TSEG1, the second TSEG2.
static const int timing_pts[23][2] = {
    {0x0, 0x0},      // 2,  50%
    {0x1, 0x0},      // 3,  67%
    {0x2, 0x0},      // 4,  75%
    {0x3, 0x0},      // 5,  80%
    {0x3, 0x1},      // 6,  67%
    {0x4, 0x1},      // 7,  71%
    {0x5, 0x1},      // 8,  75%
    {0x6, 0x1},      // 9,  78%
    {0x6, 0x2},      // 10, 70%
    {0x7, 0x2},      // 11, 73%
    {0x8, 0x2},      // 12, 75%
    {0x9, 0x2},      // 13, 77%
    {0x9, 0x3},      // 14, 71%
    {0xA, 0x3},      // 15, 73%
    {0xB, 0x3},      // 16, 75%
    {0xC, 0x3},      // 17, 76%
    {0xD, 0x3},      // 18, 78%
    {0xD, 0x4},      // 19, 74%
    {0xE, 0x4},      // 20, 75%
    {0xF, 0x4},      // 21, 76%
    {0xF, 0x5},      // 22, 73%
    {0xF, 0x6},      // 23, 70%
    {0xF, 0x7},      // 24, 67%
};

static unsigned int can_speed(unsigned int sclk, unsigned int cclk, unsigned char psjw) {
#if 0
	#warning (matthewelse) This is the bit that I'm going to have no clue about :P
    uint32_t    btr;
    uint16_t    brp = 0;
    uint32_t    calcbit;
    uint32_t    bitwidth;
    int         hit = 0;
    int         bits;
    
    #warning the calculation of bitwidth may be wrong...
    bitwidth = sclk / cclk;
    
    brp = bitwidth / 0x18;
    while ((!hit) && (brp < bitwidth / 4)) {
        brp++;
        for (bits = 22; bits > 0; bits--) {
            calcbit = (bits + 3) * (brp + 1);
            if (calcbit == bitwidth) {
                hit = 1;
                break;
            }
        }
    }
    
    if (hit) {
        btr = ((timing_pts[bits][1] << 20) & 0x00700000)
            | ((timing_pts[bits][0] << 16) & 0x000F0000)
            | ((psjw                << 14) & 0x0000C000)
            | ((brp                 <<  0) & 0x000003FF);
    } else {
        btr = 0xFFFFFFFF;
    }
    
    return btr;
#endif
		return 0;
}

void can_init(can_t *obj, PinName rd, PinName td) {
    // Enable power and clock
	  LPC_SYSCON->PRESETCTRL |= 1 << 3;
    LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 17;
    
		// Disable operation
    if (!(LPC_CAN->CNTL & 1)) {
        LPC_CAN->CNTL |= 1;
    }

		// Divide clock by 6
    LPC_CAN->CLKDIV = 0x05;

    // Set the bit clock
    // Enable changes to the clock etc.
    LPC_CAN->CNTL |= (1 << 6);
    LPC_CAN->BT = 0x2301;
    LPC_CAN->BRPE = 0x0000;
    LPC_CAN->CNTL &= ~(1 << 6);
		
		// Resume operation
    LPC_CAN->CNTL &= ~1;
    while ( LPC_CAN->CNTL & 1 );
}

void can_free(can_t *obj) {
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 17);
    LPC_SYSCON->PRESETCTRL &= 1 << 3;
}

int can_frequency(can_t *obj, int f) {
    int btr = can_speed(SystemCoreClock, (unsigned int)f, 1);
#if 0
    if (btr > 0) {
        uint32_t modmask = can_disable(obj);
        obj->dev->BTR = btr;
        obj->dev->MOD = modmask;
        return 1;
    } else {
        return 0;
    }
#endif
    return 1;
}

int can_write(can_t *obj, CAN_Message msg, int cc) {
	unsigned int CANStatus;
    CANMsg m;

    can_enable(obj);

    m.id   = msg.id ;
    m.dlc  = msg.len & 0xF;
    m.rtr  = msg.type;
    m.type = msg.format;
    memcpy(m.data, msg.data, msg.len);
    const unsigned int *buf = (const unsigned int *)&m;

	  /* MsgVal: 1, Mtd: 0, Dir: 1, ID = 0x200 */
		LPC_CAN->IF1_ARB2 = ID_MVAL | ID_DIR | (msg.id << 2);
		LPC_CAN->IF1_ARB1 = 0x0000;
	
		/* Mxtd: 0, Mdir: 1, Mask is 0x7FF */
		LPC_CAN->IF1_MSK2 = MASK_MDIR | (ID_STD_MASK << 2);
		LPC_CAN->IF1_MSK1 = 0x0000;
	
	  LPC_CAN->IF1_MCTRL = UMSK|TXRQ|EOB|(m.dlc);

		LPC_CAN->IF1_DA1 = (m.data[1] << 8) | m.data[0];
		LPC_CAN->IF1_DA2 = (m.data[3] << 8) | m.data[2];
		LPC_CAN->IF1_DB1 = (m.data[5] << 8) | m.data[4];
		LPC_CAN->IF1_DB2 = (m.data[7] << 8) | m.data[6];

		LPC_CAN->IF1_CMDMSK = WR|MASK|ARB|CTRL|TREQ|DATAA|DATAB;
		LPC_CAN->IF1_CMDREQ = 1;
		
		while( LPC_CAN->IF1_CMDREQ & IFCREQ_BUSY );   /* Could spin here forever */		
		
		while( !(LPC_CAN->STAT & (1 << 3)) );   		/* Wait for TXOK to be set. */
	  LPC_CAN->STAT &= ~(1 << 3);					/* Manually clear is required. */
		
    return 0;
}

int can_read(can_t *obj, CAN_Message *msg) {
    can_enable(obj);

    if (LPC_CAN->STAT & (1 << 4)) {
        unsigned int id = 0xff & LPC_CAN->IF1_ARB1;

        if (LPC_CAN->IF1_ARB2 & 1 << 14) {
            // It's an extended ID.
            id |= (LPC_CAN->IF1_ARB2 & 0x1fff) << 16;
        }

        msg->id         = id;
        msg->len        = (LPC_CAN->IF1_MCTRL & 0x7) < 8 ? (LPC_CAN->IF1_MCTRL & 0x7) : 8;
        msg->format     = (LPC_CAN->IF1_ARB2 & 1 << 14)? CANExtended : CANStandard;
        msg->type       = CANData;
        msg->data[0]    = LPC_CAN->IF1_DA1 & 0xf;
        msg->data[1]    = (LPC_CAN->IF1_DA1 & 0xf0) >> 8;
        msg->data[2]    = LPC_CAN->IF1_DA2 & 0xf;
        msg->data[3]    = (LPC_CAN->IF1_DA2 & 0xf0) >> 8;
        msg->data[4]    = LPC_CAN->IF1_DB1 & 0xf;
        msg->data[5]    = (LPC_CAN->IF1_DB1 & 0xf0) >> 8;
        msg->data[6]    = LPC_CAN->IF1_DB2 & 0xf;
        msg->data[7]    = (LPC_CAN->IF1_DB2 & 0xf0) >> 8;
        return 1;
    }

    return 0;
}

void can_reset(can_t *obj) {
    LPC_SYSCON->PRESETCTRL &= ~(1 << 3);
    LPC_CAN->STAT = 0;
}

unsigned char can_rderror(can_t *obj) {
    return (LPC_CAN->EC >> 8) & 0x7f;
}

unsigned char can_tderror(can_t *obj) {
    return (LPC_CAN->EC & 0xff);
}

void can_monitor(can_t *obj, int silent) {
    if (silent) {
        LPC_CAN->CNTL |= 1 << 7;
        LPC_CAN->TEST |= 1 << 3;
    } else {
        LPC_CAN->CNTL &= ~(1 << 7);
        LPC_CAN->TEST &= ~(1 << 3);
    }

    if (!(LPC_CAN->CNTL & 1)) {
        LPC_CAN->CNTL |= 1;
    }
}
