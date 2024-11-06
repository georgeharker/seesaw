/*******************************************************************************
 * Copyright (C) Dean Miller
 * All rights reserved.
 *
 * This program is open source software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <assert.h>

#include "qpcpp.h"
#include "qp_extras.h"

#include "hsm_id.h"
#include "event.h"
#include "SeesawConfig.h"
#include "AOEncoder.h"
#include "bsp_timer.h"
#include "bsp_gpio.h"

Q_DEFINE_THIS_FILE

using namespace FW;

static_assert(sizeof(encoderEvent) == 4, "encoderEvent must be 4 bytes");

#if CONFIG_ENCODER

Fifo *AOEncoder::m_fifo;

#define BIT_IS_SET(x,b) (((x)&(1UL << b)) != 0)
#define BIT_IS_CLEAR(x,b) (((x)&(1UL << b)) == 0)

#define INPUT_MASK_PORTA(pin_en, porta, pin) (((uint64_t)(pin_en) & (uint32_t)(porta)) << pin)
#define INPUT_MASK_PORTB(pin_en, porta, pin) (((uint64_t)(pin_en) & ~(uint32_t)(porta)) << pin)

#define ENCODER_INPUT_MASK_PORTA \
    (INPUT_MASK_PORTA(CONFIG_ENCODER0, CONFIG_ENCODER0_AB_PORTA, CONFIG_ENCODER0_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER1, CONFIG_ENCODER1_AB_PORTA, CONFIG_ENCODER1_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER2, CONFIG_ENCODER2_AB_PORTA, CONFIG_ENCODER2_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER3, CONFIG_ENCODER3_AB_PORTA, CONFIG_ENCODER3_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER4, CONFIG_ENCODER4_AB_PORTA, CONFIG_ENCODER4_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER5, CONFIG_ENCODER5_AB_PORTA, CONFIG_ENCODER5_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER6, CONFIG_ENCODER6_AB_PORTA, CONFIG_ENCODER6_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER7, CONFIG_ENCODER7_AB_PORTA, CONFIG_ENCODER7_A_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER0, CONFIG_ENCODER0_AB_PORTA, CONFIG_ENCODER0_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER1, CONFIG_ENCODER1_AB_PORTA, CONFIG_ENCODER1_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER2, CONFIG_ENCODER2_AB_PORTA, CONFIG_ENCODER2_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER3, CONFIG_ENCODER3_AB_PORTA, CONFIG_ENCODER3_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER4, CONFIG_ENCODER4_AB_PORTA, CONFIG_ENCODER4_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER5, CONFIG_ENCODER5_AB_PORTA, CONFIG_ENCODER5_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER6, CONFIG_ENCODER6_AB_PORTA, CONFIG_ENCODER6_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER7, CONFIG_ENCODER7_AB_PORTA, CONFIG_ENCODER7_B_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER0, CONFIG_ENCODER0_SW_PORTA, CONFIG_ENCODER0_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER1, CONFIG_ENCODER1_SW_PORTA, CONFIG_ENCODER1_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER2, CONFIG_ENCODER2_SW_PORTA, CONFIG_ENCODER2_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER3, CONFIG_ENCODER3_SW_PORTA, CONFIG_ENCODER3_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER4, CONFIG_ENCODER4_SW_PORTA, CONFIG_ENCODER4_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER5, CONFIG_ENCODER5_SW_PORTA, CONFIG_ENCODER5_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER6, CONFIG_ENCODER6_SW_PORTA, CONFIG_ENCODER6_SW_PIN) | \
     INPUT_MASK_PORTA(CONFIG_ENCODER7, CONFIG_ENCODER7_SW_PORTA, CONFIG_ENCODER7_SW_PIN))

#define ENCODER_INPUT_MASK_PORTB \
    (INPUT_MASK_PORTB(CONFIG_ENCODER0, CONFIG_ENCODER0_AB_PORTA, CONFIG_ENCODER0_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER1, CONFIG_ENCODER1_AB_PORTA, CONFIG_ENCODER1_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER2, CONFIG_ENCODER2_AB_PORTA, CONFIG_ENCODER2_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER3, CONFIG_ENCODER3_AB_PORTA, CONFIG_ENCODER3_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER4, CONFIG_ENCODER4_AB_PORTA, CONFIG_ENCODER4_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER5, CONFIG_ENCODER5_AB_PORTA, CONFIG_ENCODER5_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER6, CONFIG_ENCODER6_AB_PORTA, CONFIG_ENCODER6_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER7, CONFIG_ENCODER7_AB_PORTA, CONFIG_ENCODER7_A_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER0, CONFIG_ENCODER0_AB_PORTA, CONFIG_ENCODER0_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER1, CONFIG_ENCODER1_AB_PORTA, CONFIG_ENCODER1_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER2, CONFIG_ENCODER2_AB_PORTA, CONFIG_ENCODER2_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER3, CONFIG_ENCODER3_AB_PORTA, CONFIG_ENCODER3_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER4, CONFIG_ENCODER4_AB_PORTA, CONFIG_ENCODER4_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER5, CONFIG_ENCODER5_AB_PORTA, CONFIG_ENCODER5_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER6, CONFIG_ENCODER6_AB_PORTA, CONFIG_ENCODER6_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER7, CONFIG_ENCODER7_AB_PORTA, CONFIG_ENCODER7_B_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER0, CONFIG_ENCODER0_SW_PORTA, CONFIG_ENCODER0_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER1, CONFIG_ENCODER1_SW_PORTA, CONFIG_ENCODER1_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER2, CONFIG_ENCODER2_SW_PORTA, CONFIG_ENCODER2_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER3, CONFIG_ENCODER3_SW_PORTA, CONFIG_ENCODER3_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER4, CONFIG_ENCODER4_SW_PORTA, CONFIG_ENCODER4_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER5, CONFIG_ENCODER5_SW_PORTA, CONFIG_ENCODER5_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER6, CONFIG_ENCODER6_SW_PORTA, CONFIG_ENCODER6_SW_PIN) | \
     INPUT_MASK_PORTB(CONFIG_ENCODER7, CONFIG_ENCODER7_SW_PORTA, CONFIG_ENCODER7_SW_PIN))

uint32_t encoder_a_pins[] = {
        CONFIG_ENCODER0_A_PIN,
        CONFIG_ENCODER1_A_PIN,
        CONFIG_ENCODER2_A_PIN,
        CONFIG_ENCODER3_A_PIN,
        CONFIG_ENCODER4_A_PIN,
        CONFIG_ENCODER5_A_PIN,
        CONFIG_ENCODER6_A_PIN,
        CONFIG_ENCODER7_A_PIN
     };
uint32_t encoder_b_pins[] = {
        CONFIG_ENCODER0_B_PIN,
        CONFIG_ENCODER1_B_PIN,
        CONFIG_ENCODER2_B_PIN,
        CONFIG_ENCODER3_B_PIN,
        CONFIG_ENCODER4_B_PIN,
        CONFIG_ENCODER5_B_PIN,
        CONFIG_ENCODER6_B_PIN,
        CONFIG_ENCODER7_B_PIN
     };
uint32_t encoder_sw_pins[] = {
        CONFIG_ENCODER0_SW_PIN,
        CONFIG_ENCODER1_SW_PIN,
        CONFIG_ENCODER2_SW_PIN,
        CONFIG_ENCODER3_SW_PIN,
        CONFIG_ENCODER4_SW_PIN,
        CONFIG_ENCODER5_SW_PIN,
        CONFIG_ENCODER6_SW_PIN,
        CONFIG_ENCODER7_SW_PIN
     };
uint32_t encoder_ab_porta[] = {
        CONFIG_ENCODER0_AB_PORTA,
        CONFIG_ENCODER1_AB_PORTA,
        CONFIG_ENCODER2_AB_PORTA,
        CONFIG_ENCODER3_AB_PORTA,
        CONFIG_ENCODER4_AB_PORTA,
        CONFIG_ENCODER5_AB_PORTA,
        CONFIG_ENCODER6_AB_PORTA,
        CONFIG_ENCODER7_AB_PORTA
     };
uint32_t encoder_sw_porta[] = {
        CONFIG_ENCODER0_SW_PORTA,
        CONFIG_ENCODER1_SW_PORTA,
        CONFIG_ENCODER2_SW_PORTA,
        CONFIG_ENCODER3_SW_PORTA,
        CONFIG_ENCODER4_SW_PORTA,
        CONFIG_ENCODER5_SW_PORTA,
        CONFIG_ENCODER6_SW_PORTA,
        CONFIG_ENCODER7_SW_PORTA
     };

volatile int32_t AOEncoder::m_value[CONFIG_NUM_ENCODERS];
volatile int32_t AOEncoder::m_delta[CONFIG_NUM_ENCODERS];
volatile uint8_t AOEncoder::m_enc_prev_state[CONFIG_NUM_ENCODERS];
volatile uint8_t AOEncoder::m_enc_flags[CONFIG_NUM_ENCODERS];
volatile AOEncoder::status AOEncoder::m_status[CONFIG_NUM_ENCODERS];
AOEncoder::inten AOEncoder::m_inten[CONFIG_NUM_ENCODERS];

AOEncoder::AOEncoder() :
    QActive((QStateHandler)&AOEncoder::InitialPseudoState), 
    m_id(AO_ENCODER), m_name("Encoder") {}

QState AOEncoder::InitialPseudoState(AOEncoder * const me, QEvt const * const e) {
    (void)e;

    me->subscribe(ENCODER_START_REQ);
    me->subscribe(ENCODER_STOP_REQ);
    me->subscribe(ENCODER_READ_REG_REQ);
    me->subscribe(ENCODER_WRITE_REG_REQ);
      
    return Q_TRAN(&AOEncoder::Root);
}

QState AOEncoder::Root(AOEncoder * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            status = Q_TRAN(&AOEncoder::Stopped);
            break;
        }
        case ENCODER_STOP_REQ: {
            LOG_EVENT(e);
            status = Q_TRAN(&AOEncoder::Stopped);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}

QState AOEncoder::Stopped(AOEncoder * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case ENCODER_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new EncoderStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_HANDLED();
            break;
        }
        case ENCODER_START_REQ: {
            LOG_EVENT(e);

            EncoderStartReq const &r = static_cast<EncoderStartReq const &>(*e);
			me->m_fifo = r.getFifo();
            me->m_fifo->Reset();

            for (uint8_t encodernum=0; encodernum<CONFIG_NUM_ENCODERS; encodernum++) {
              AOEncoder::m_value[encodernum] = 0;
              AOEncoder::m_delta[encodernum] = 0;
              AOEncoder::m_enc_prev_state[encodernum] = 0;
              AOEncoder::m_enc_flags[encodernum] = 0;
              me->m_status[encodernum].reg = 0;
              me->m_inten[encodernum].reg = 0;
            }

            uint32_t mask = ENCODER_INPUT_MASK_PORTA;
            gpio_dirclr_bulk(PORTA, mask);
            gpio_pullenset_bulk(mask, PORTA);
            gpio_outset_bulk(PORTA, mask);
            
            mask = ENCODER_INPUT_MASK_PORTB;
            gpio_dirclr_bulk(PORTB, mask);
            gpio_pullenset_bulk(mask, PORTB);
            gpio_outset_bulk(PORTB, mask);

            initTimer(CONFIG_ENCODER_TC, CONFIG_ENCODER_FREQ);
            enableTimer(CONFIG_ENCODER_TC);
            
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new EncoderStartCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            
            status = Q_TRAN(&AOEncoder::Started);
            break;
        }
        default: {
            status = Q_SUPER(&AOEncoder::Root);
            break;
        }
    }
    return status;
}

QState AOEncoder::Started(AOEncoder * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case ENCODER_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new EncoderStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_TRAN(AOEncoder::Stopped);
            break;
        }
        case ENCODER_READ_REG_REQ: {
            LOG_EVENT(e);
            EncoderReadRegReq const &req = static_cast<EncoderReadRegReq const &>(*e);
            Fifo *dest = req.getDest();
            uint8_t reg = req.getReg() & 0xF0;
            uint8_t encodernum = req.getReg() & 0x0F;

            Evt *evt;

            if (reg == SEESAW_ENCODER_FIFO) {
                bool clear = false;
                for (uint8_t encodernum = 0; encodernum < CONFIG_NUM_ENCODERS; encodernum++) {
                    if (AOEncoder::m_status[encodernum].bit.DATA_RDY){
                        if(AOEncoder::m_inten[encodernum].bit.DATA_RDY){
                            clear = true;
                        }
                        AOEncoder::m_status[encodernum].bit.DATA_RDY = 0;
                    }
                }
                if (clear) {
                    // post an interrupt event
                    evt = new InterruptClearReq( SEESAW_INTERRUPT_ENCODER_DATA_RDY );
                    QF::PUBLISH(evt, me);
                }

                //give the requester our pipe
                evt = new DelegateDataReady(req.getRequesterId(), me->m_fifo);
            } else if (reg == SEESAW_ENCODER_COUNT) {
                encoderEvent encevent = {0};
                encevent.bit.TYPE = ENCODER_TYPE_COUNT;
                encevent.bit.count.__PACK = 0x99;       // FIXME 0
                // NOTE: is this potentially approximate
                encevent.bit.count.COUNT = me->m_fifo->GetUsedCount() / sizeof(encoderEvent);
                    
                //return the read register in the default fifo
                dest->Write(encevent.reg, sizeof(encoderEvent));  // NOTE: order should probably be first
                evt = new DelegateDataReady(req.getRequesterId());
            } else {
                encoderEvent encevent = {0};
                encevent.bit.TYPE = ENCODER_TYPE_INVALID;
                if (encodernum < CONFIG_NUM_ENCODERS) {
                    switch(reg) {
                        case SEESAW_ENCODER_STATUS:
                        {
                            encevent.bit.TYPE = ENCODER_TYPE_STATUS;
                            encevent.bit.value.VALUE = static_cast<uint32_t>(AOEncoder::m_status[encodernum].reg);
                            encevent.bit.value.ENCODER = static_cast<uint8_t>(encodernum);
                            break;
                        }

                        case SEESAW_ENCODER_POSITION:
                        {
                            encevent.bit.TYPE = ENCODER_TYPE_VALUE;
                            encevent.bit.value.VALUE = static_cast<uint32_t>(AOEncoder::m_value[encodernum]);
                            encevent.bit.value.ENCODER = static_cast<uint8_t>(encodernum);
                            break;
                        }


                        case SEESAW_ENCODER_DELTA:
                        {
                            encevent.bit.TYPE = ENCODER_TYPE_DELTA;
                            encevent.bit.value.VALUE = static_cast<uint32_t>(AOEncoder::m_delta[encodernum]);
                            encevent.bit.value.ENCODER = static_cast<uint8_t>(encodernum);
                            break;
                        }
                        
                        default: {
                            encevent.bit.TYPE = ENCODER_TYPE_INVALID;
                            break;
                        }
                    }
                }

                //return the read register in the default fifo
                dest->Write(encevent.reg, sizeof(encoderEvent));  // NOTE: order should probably be first
                evt = new DelegateDataReady(req.getRequesterId());
            }

            QF::PUBLISH(evt, me);

            status = Q_HANDLED();
            break;
        }
        case ENCODER_WRITE_REG_REQ: 
        {
            // FIXME: use the keyEvent structure
            LOG_EVENT(e);
            EncoderWriteRegReq const &req = static_cast<EncoderWriteRegReq const &>(*e);
            uint8_t reg = req.getReg() & 0xF0;
            uint8_t encodernum = req.getReg() & 0x0F;
            int32_t value = req.getValue();
            
            if (encodernum < CONFIG_NUM_ENCODERS) {
                switch (reg) {
                    case SEESAW_ENCODER_EVENT:
                    {
                        //turn an event on or off
                        volatile AOEncoder::status *es;
                        es = &me->m_status[encodernum];

                        // Value is:
                        // edge 0-8
                        // enable 16

                        if(value & (0x01 << 16)) //activate the selected edges
                            es->bit.ACTIVE |= (value & 0xFF);
                        else //deactivate the selected edges
                            es->bit.ACTIVE &= (value & 0xFF);

                        break;
                    }

                    case SEESAW_ENCODER_POSITION:
                        AOEncoder::m_value[encodernum] = value;
                        break;
                    case SEESAW_ENCODER_DELTA:
                        AOEncoder::m_delta[encodernum] = value;
                        break;
                    case SEESAW_ENCODER_INTENSET:
                        me->m_inten[encodernum].reg |= value;
                        break;
                    case SEESAW_ENCODER_INTENCLR:
                        me->m_inten[encodernum].reg &= ~value;
                        break;
                    default:
                        break;
                }
                status = Q_HANDLED();
                break;
            }
        }
        default: {
            status = Q_SUPER(&AOEncoder::Root);
            break;
        }
    }
    return status;
}

extern "C" {
void CONFIG_ENCODER_HANDLER( void ) {
    QXK_ISR_ENTRY();

    uint32_t mask = ENCODER_INPUT_MASK_PORTA;
    uint32_t in_a = gpio_read_bulk(PORTA) & mask;
    mask = ENCODER_INPUT_MASK_PORTB;
    uint32_t in_b = gpio_read_bulk(PORTB) & mask;
    bool interrupt[CONFIG_NUM_ENCODERS];

    for (uint8_t encodernum = 0; encodernum < CONFIG_NUM_ENCODERS; encodernum++) {
        int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
        interrupt[encodernum] = false;
  
        uint8_t enc_cur_state = 0;
        // read in the encoder state first
        if (encoder_ab_porta[encodernum]) {
            enc_cur_state |= (((BIT_IS_CLEAR(in_a, encoder_a_pins[encodernum])) << 0) | 
                              ((BIT_IS_CLEAR(in_a, encoder_b_pins[encodernum])) << 1));
        } else {
            enc_cur_state |= (((BIT_IS_CLEAR(in_b, encoder_a_pins[encodernum])) << 0) |
                              ((BIT_IS_CLEAR(in_b, encoder_b_pins[encodernum])) << 1));
        }
        if (encoder_sw_porta[encodernum]) {
            enc_cur_state |= ((BIT_IS_CLEAR(in_a, encoder_sw_pins[encodernum])) << 2);
        } else {
            enc_cur_state |= ((BIT_IS_CLEAR(in_b, encoder_sw_pins[encodernum])) << 2);
        }

        // if any rotation at all
        uint8_t enc_cur_pos = enc_cur_state & 0x03;
        uint8_t enc_prev_pos = AOEncoder::m_enc_prev_state[encodernum] & 0x03;

        if (enc_cur_pos != enc_prev_pos) {
            if (enc_prev_pos == 0x00) {
                // this is the first edge
                if (enc_cur_pos == 0x01) {
                    AOEncoder::m_enc_flags[encodernum] |= (1 << 0);
                }
                else if (enc_cur_pos == 0x02) {
                    AOEncoder::m_enc_flags[encodernum] |= (1 << 1);
                }
            }
          
            if (enc_cur_pos == 0x03) {
                // this is when the encoder is in the middle of a "step"
                AOEncoder::m_enc_flags[encodernum] |= (1 << 4);
            }
            else if (enc_cur_pos == 0x00) {
                // this is the final edge
                if (enc_prev_pos == 0x02) {
                    AOEncoder::m_enc_flags[encodernum] |= (1 << 2);
                }
                else if (enc_prev_pos == 0x01) {
                    AOEncoder::m_enc_flags[encodernum] |= (1 << 3);
                }
              
                // check the first and last edge
                // or maybe one edge is missing, if missing then require the middle state
                // this will reject bounces and false movements
                if (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 0) && (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 2) || BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 4))) {
                    enc_action = 1;
                 }
                 else if (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 2) && (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 0) || BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 4))) {
                    enc_action = 1;
                 }
                 else if (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 1) && (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 3) || BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 4))) {
                    enc_action = -1;
                 }
                 else if (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 3) && (BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 1) || BIT_IS_SET(AOEncoder::m_enc_flags[encodernum], 4))) {
                    enc_action = -1;
                 }
              
                 AOEncoder::m_enc_flags[encodernum] = 0; // reset for next time
            }
        }

        encoderEvent encevent = {0};
        if (enc_action != 0) {
            AOEncoder::m_value[encodernum] += enc_action;
            AOEncoder::m_delta[encodernum] += enc_action;
        
            if (AOEncoder::m_status[encodernum].bit.ACTIVE & (1 << ENCODER_VALUE_CHANGE)) {
                encevent.bit.TYPE = ENCODER_TYPE_VALUE;
                encevent.bit.value.ENCODER = encodernum;
                encevent.bit.value.VALUE = AOEncoder::m_value[encodernum];
            
                AOEncoder::m_fifo->Write(encevent.reg, sizeof(encoderEvent));
            }
            if (AOEncoder::m_status[encodernum].bit.ACTIVE & (1 << ENCODER_DELTA)) {
                encevent.bit.TYPE = ENCODER_TYPE_DELTA;
                encevent.bit.delta.ENCODER = encodernum;
                encevent.bit.delta.DELTA = enc_action;
            
                AOEncoder::m_fifo->Write(encevent.reg, sizeof(encoderEvent));
            }

        }

        uint8_t enc_cur_sw = enc_cur_state & 0x04;
        uint8_t enc_prev_sw = AOEncoder::m_enc_prev_state[encodernum] & 0x04;

        bool press_event = false;
        if (enc_cur_sw && (AOEncoder::m_status[encodernum].bit.ACTIVE & (1 << ENCODER_EDGE_HIGH))) {
            encevent.bit.TYPE = ENCODER_TYPE_PRESS;
            encevent.bit.press.ENCODER = encodernum;
            encevent.bit.press.EDGE = ENCODER_EDGE_HIGH;
            press_event = true;
        
            AOEncoder::m_fifo->Write(encevent.reg, sizeof(encoderEvent));
        }
        if (!enc_cur_sw && (AOEncoder::m_status[encodernum].bit.ACTIVE & (1 << ENCODER_EDGE_LOW))) {
            encevent.bit.TYPE = ENCODER_TYPE_PRESS;
            encevent.bit.press.ENCODER = encodernum;
            encevent.bit.press.EDGE = ENCODER_EDGE_LOW;
            press_event = true;
        
            AOEncoder::m_fifo->Write(encevent.reg, sizeof(encoderEvent));
        }
        if (enc_cur_sw != enc_prev_sw) {
            if (enc_cur_sw && (AOEncoder::m_status[encodernum].bit.ACTIVE & (1 << ENCODER_EDGE_RISING))) {
                encevent.bit.TYPE = ENCODER_TYPE_PRESS;
                encevent.bit.press.ENCODER = encodernum;
                encevent.bit.press.EDGE = ENCODER_EDGE_RISING;
                press_event = true;
            
                AOEncoder::m_fifo->Write(encevent.reg, sizeof(encoderEvent));
            }
            if (!enc_cur_sw && (AOEncoder::m_status[encodernum].bit.ACTIVE & (1 << ENCODER_EDGE_FALLING))) {
                encevent.bit.TYPE = ENCODER_TYPE_PRESS;
                encevent.bit.press.ENCODER = encodernum;
                encevent.bit.press.EDGE = ENCODER_EDGE_FALLING;
                press_event = true;
            
                AOEncoder::m_fifo->Write(encevent.reg, sizeof(encoderEvent));
            }

        }
    
        interrupt[encodernum] = (enc_action != 0 || press_event);
        
        // Record the previous state
        AOEncoder::m_enc_prev_state[encodernum] = enc_cur_state;
    }

    // Re enable ISRs before an call to QF::PUBLISH
    QXK_ISR_EXIT();

    for (uint8_t encodernum = 0; encodernum < CONFIG_NUM_ENCODERS; encodernum++) {
        if (interrupt[encodernum]) {
            // if interrupts are enabled fire an interrupt
            AOEncoder::m_status[encodernum].bit.DATA_RDY = 1;
            if (AOEncoder::m_status[encodernum].bit.DATA_RDY) {
                if (AOEncoder::m_inten[encodernum].bit.DATA_RDY){
                   Evt *evt = new InterruptSetReq( SEESAW_INTERRUPT_ENCODER_DATA_RDY );
                   QF::PUBLISH(evt, 0);
                }
            }
        }
    }

    //clear the interrupt
    CONFIG_ENCODER_TC->COUNT16.INTFLAG.bit.MC0 = 1;
}
}

#endif
