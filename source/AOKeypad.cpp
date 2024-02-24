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

#include "qpcpp.h"
#include "qp_extras.h"

#include "hsm_id.h"
#include "AOKeypad.h"
#include "event.h"
#include "SeesawConfig.h"

#include "bsp_gpio.h"
#include "bsp_sercom.h"
#include "bsp_adc.h"

Q_DEFINE_THIS_FILE

using namespace FW;

#define KEYPAD_MAX_ROWS 8
#define KEYPAD_MAX_COLS 8

#define INPUT_MASK_PORTA(pin_en, porta, pin) (((uint64_t)(pin_en) & (uint32_t)(porta)) << pin)
#define INPUT_MASK_PORTB(pin_en, porta, pin) (((uint64_t)(pin_en) & ~(uint32_t)(porta)) << pin)

#define OUTPUT_MASK_PORTA(pin_en, porta, pin) (((uint64_t)(pin_en) & (uint32_t)(porta)) << pin)
#define OUTPUT_MASK_PORTB(pin_en, porta, pin) (((uint64_t)(pin_en) & ~(uint32_t)(porta)) << pin)

#if KEYPAD_SCAN_ROWS

#define KEYPAD_INPUT_MASK_PORTA (INPUT_MASK_PORTA(CONFIG_KEYPAD_COL0, CONFIG_KEYPAD_COL0_PORTA, CONFIG_KEYPAD_COL0_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL1, CONFIG_KEYPAD_COL1_PORTA, CONFIG_KEYPAD_COL1_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL2, CONFIG_KEYPAD_COL2_PORTA, CONFIG_KEYPAD_COL2_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL3, CONFIG_KEYPAD_COL3_PORTA, CONFIG_KEYPAD_COL3_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL4, CONFIG_KEYPAD_COL4_PORTA, CONFIG_KEYPAD_COL4_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL5, CONFIG_KEYPAD_COL5_PORTA, CONFIG_KEYPAD_COL5_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL6, CONFIG_KEYPAD_COL6_PORTA, CONFIG_KEYPAD_COL6_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_COL7, CONFIG_KEYPAD_COL7_PORTA, CONFIG_KEYPAD_COL7_PIN))

#define KEYPAD_INPUT_MASK_PORTB (INPUT_MASK_PORTB(CONFIG_KEYPAD_COL0, CONFIG_KEYPAD_COL0_PORTA, CONFIG_KEYPAD_COL0_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL1, CONFIG_KEYPAD_COL1_PORTA, CONFIG_KEYPAD_COL1_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL2, CONFIG_KEYPAD_COL2_PORTA, CONFIG_KEYPAD_COL2_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL3, CONFIG_KEYPAD_COL3_PORTA, CONFIG_KEYPAD_COL3_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL4, CONFIG_KEYPAD_COL4_PORTA, CONFIG_KEYPAD_COL4_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL5, CONFIG_KEYPAD_COL5_PORTA, CONFIG_KEYPAD_COL5_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL6, CONFIG_KEYPAD_COL6_PORTA, CONFIG_KEYPAD_COL6_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_COL7, CONFIG_KEYPAD_COL7_PORTA, CONFIG_KEYPAD_COL7_PIN))

#define KEYPAD_OUTPUT_MASK_PORTA (OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW0, CONFIG_KEYPAD_ROW0_PORTA, CONFIG_KEYPAD_ROW0_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW1, CONFIG_KEYPAD_ROW1_PORTA, CONFIG_KEYPAD_ROW1_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW2, CONFIG_KEYPAD_ROW2_PORTA, CONFIG_KEYPAD_ROW2_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW3, CONFIG_KEYPAD_ROW3_PORTA, CONFIG_KEYPAD_ROW3_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW4, CONFIG_KEYPAD_ROW4_PORTA, CONFIG_KEYPAD_ROW4_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW5, CONFIG_KEYPAD_ROW5_PORTA, CONFIG_KEYPAD_ROW5_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW6, CONFIG_KEYPAD_ROW6_PORTA, CONFIG_KEYPAD_ROW6_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_ROW7, CONFIG_KEYPAD_ROW7_PORTA, CONFIG_KEYPAD_ROW7_PIN))

#define KEYPAD_OUTPUT_MASK_PORTB (OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW0, CONFIG_KEYPAD_ROW0_PORTA, CONFIG_KEYPAD_ROW0_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW1, CONFIG_KEYPAD_ROW1_PORTA, CONFIG_KEYPAD_ROW1_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW2, CONFIG_KEYPAD_ROW2_PORTA, CONFIG_KEYPAD_ROW2_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW3, CONFIG_KEYPAD_ROW3_PORTA, CONFIG_KEYPAD_ROW3_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW4, CONFIG_KEYPAD_ROW4_PORTA, CONFIG_KEYPAD_ROW4_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW5, CONFIG_KEYPAD_ROW5_PORTA, CONFIG_KEYPAD_ROW5_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW6, CONFIG_KEYPAD_ROW6_PORTA, CONFIG_KEYPAD_ROW6_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_ROW7, CONFIG_KEYPAD_ROW7_PORTA, CONFIG_KEYPAD_ROW7_PIN))

#else

#define KEYPAD_INPUT_MASK_PORTA (INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW0, CONFIG_KEYPAD_ROW0_PORTA, CONFIG_KEYPAD_ROW0_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW1, CONFIG_KEYPAD_ROW1_PORTA, CONFIG_KEYPAD_ROW1_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW2, CONFIG_KEYPAD_ROW2_PORTA, CONFIG_KEYPAD_ROW2_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW3, CONFIG_KEYPAD_ROW3_PORTA, CONFIG_KEYPAD_ROW3_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW4, CONFIG_KEYPAD_ROW4_PORTA, CONFIG_KEYPAD_ROW4_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW5, CONFIG_KEYPAD_ROW5_PORTA, CONFIG_KEYPAD_ROW5_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW6, CONFIG_KEYPAD_ROW6_PORTA, CONFIG_KEYPAD_ROW6_PIN) \
                               | INPUT_MASK_PORTA(CONFIG_KEYPAD_ROW7, CONFIG_KEYPAD_ROW7_PORTA, CONFIG_KEYPAD_ROW7_PIN))

#define KEYPAD_INPUT_MASK_PORTB (INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW0, CONFIG_KEYPAD_ROW0_PORTA, CONFIG_KEYPAD_ROW0_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW1, CONFIG_KEYPAD_ROW1_PORTA, CONFIG_KEYPAD_ROW1_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW2, CONFIG_KEYPAD_ROW2_PORTA, CONFIG_KEYPAD_ROW2_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW3, CONFIG_KEYPAD_ROW3_PORTA, CONFIG_KEYPAD_ROW3_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW4, CONFIG_KEYPAD_ROW4_PORTA, CONFIG_KEYPAD_ROW4_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW5, CONFIG_KEYPAD_ROW5_PORTA, CONFIG_KEYPAD_ROW5_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW6, CONFIG_KEYPAD_ROW6_PORTA, CONFIG_KEYPAD_ROW6_PIN) \
                               | INPUT_MASK_PORTB(CONFIG_KEYPAD_ROW7, CONFIG_KEYPAD_ROW7_PORTA, CONFIG_KEYPAD_ROW7_PIN))

#define KEYPAD_OUTPUT_MASK_PORTA (OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL0, CONFIG_KEYPAD_COL0_PORTA, CONFIG_KEYPAD_COL0_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL1, CONFIG_KEYPAD_COL1_PORTA, CONFIG_KEYPAD_COL1_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL2, CONFIG_KEYPAD_COL2_PORTA, CONFIG_KEYPAD_COL2_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL3, CONFIG_KEYPAD_COL3_PORTA, CONFIG_KEYPAD_COL3_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL4, CONFIG_KEYPAD_COL4_PORTA, CONFIG_KEYPAD_COL4_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL5, CONFIG_KEYPAD_COL5_PORTA, CONFIG_KEYPAD_COL5_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL6, CONFIG_KEYPAD_COL6_PORTA, CONFIG_KEYPAD_COL6_PIN) \
                                | OUTPUT_MASK_PORTA(CONFIG_KEYPAD_COL7, CONFIG_KEYPAD_COL7_PORTA, CONFIG_KEYPAD_COL7_PIN))

#define KEYPAD_OUTPUT_MASK_PORTB (OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL0, CONFIG_KEYPAD_COL0_PORTA, CONFIG_KEYPAD_COL0_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL1, CONFIG_KEYPAD_COL1_PORTA, CONFIG_KEYPAD_COL1_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL2, CONFIG_KEYPAD_COL2_PORTA, CONFIG_KEYPAD_COL2_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL3, CONFIG_KEYPAD_COL3_PORTA, CONFIG_KEYPAD_COL3_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL4, CONFIG_KEYPAD_COL4_PORTA, CONFIG_KEYPAD_COL4_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL5, CONFIG_KEYPAD_COL5_PORTA, CONFIG_KEYPAD_COL5_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL6, CONFIG_KEYPAD_COL6_PORTA, CONFIG_KEYPAD_COL6_PIN) \
                                | OUTPUT_MASK_PORTB(CONFIG_KEYPAD_COL7, CONFIG_KEYPAD_COL7_PORTA, CONFIG_KEYPAD_COL7_PIN))

#endif

#define KEYPAD_ACTIVE_ROWS (  (CONFIG_KEYPAD_ROW7 << 7) | (CONFIG_KEYPAD_ROW6 << 6) \
                            | (CONFIG_KEYPAD_ROW5 << 5) | (CONFIG_KEYPAD_ROW4 << 4) \
                            | (CONFIG_KEYPAD_ROW3 << 3) | (CONFIG_KEYPAD_ROW2 << 2) \
                            | (CONFIG_KEYPAD_ROW1 << 1) | (CONFIG_KEYPAD_ROW0) )
                    
#define KEYPAD_ACTIVE_COLS (  (CONFIG_KEYPAD_COL7 << 7) | (CONFIG_KEYPAD_COL6 << 6) \
                            | (CONFIG_KEYPAD_COL3 << 5) | (CONFIG_KEYPAD_COL2 << 4) \
                            | (CONFIG_KEYPAD_COL3 << 3) | (CONFIG_KEYPAD_COL2 << 2) \
                            | (CONFIG_KEYPAD_COL1 << 1) | (CONFIG_KEYPAD_COL0) )

#define KEYPAD_EVENT_TO_ROW(e) ( (e)/8 )
#define KEYPAD_EVENT_TO_COL(e) ( (e)%8 )
#define KEYPAD_EVENT(r, c) ( (r)*8 + (c) )

#define KEYPAD_HIGH (1 << KEYPAD_EDGE_HIGH)
#define KEYPAD_LOW (1 << KEYPAD_EDGE_LOW)
#define KEYPAD_RISING (1 << KEYPAD_EDGE_RISING)
#define KEYPAD_FALLING (1 << KEYPAD_EDGE_FALLING)

#if KEYPAD_SCAN_ROWS
static const uint32_t keypad_input_masks_a[] = {
    (((uint32_t)CONFIG_KEYPAD_COL0 & (uint32_t)CONFIG_KEYPAD_COL0_PORTA) << CONFIG_KEYPAD_COL0_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL1 & (uint32_t)CONFIG_KEYPAD_COL1_PORTA) << CONFIG_KEYPAD_COL1_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL2 & (uint32_t)CONFIG_KEYPAD_COL2_PORTA) << CONFIG_KEYPAD_COL2_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL3 & (uint32_t)CONFIG_KEYPAD_COL3_PORTA) << CONFIG_KEYPAD_COL3_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL4 & (uint32_t)CONFIG_KEYPAD_COL4_PORTA) << CONFIG_KEYPAD_COL4_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL5 & (uint32_t)CONFIG_KEYPAD_COL5_PORTA) << CONFIG_KEYPAD_COL5_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL6 & (uint32_t)CONFIG_KEYPAD_COL6_PORTA) << CONFIG_KEYPAD_COL6_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL7 & (uint32_t)CONFIG_KEYPAD_COL7_PORTA) << CONFIG_KEYPAD_COL7_PIN)
};

static const uint32_t keypad_input_masks_b[] = {
    (((uint32_t)CONFIG_KEYPAD_COL0 & ~(uint32_t)CONFIG_KEYPAD_COL0_PORTA) << CONFIG_KEYPAD_COL0_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL1 & ~(uint32_t)CONFIG_KEYPAD_COL1_PORTA) << CONFIG_KEYPAD_COL1_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL2 & ~(uint32_t)CONFIG_KEYPAD_COL2_PORTA) << CONFIG_KEYPAD_COL2_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL3 & ~(uint32_t)CONFIG_KEYPAD_COL3_PORTA) << CONFIG_KEYPAD_COL3_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL4 & ~(uint32_t)CONFIG_KEYPAD_COL4_PORTA) << CONFIG_KEYPAD_COL4_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL5 & ~(uint32_t)CONFIG_KEYPAD_COL5_PORTA) << CONFIG_KEYPAD_COL5_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL6 & ~(uint32_t)CONFIG_KEYPAD_COL6_PORTA) << CONFIG_KEYPAD_COL6_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL7 & ~(uint32_t)CONFIG_KEYPAD_COL7_PORTA) << CONFIG_KEYPAD_COL7_PIN)
};

static const uint32_t keypad_output_masks_a[] = {
    (((uint32_t)CONFIG_KEYPAD_ROW0 & (uint32_t)CONFIG_KEYPAD_ROW0_PORTA) << CONFIG_KEYPAD_ROW0_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW1 & (uint32_t)CONFIG_KEYPAD_ROW1_PORTA) << CONFIG_KEYPAD_ROW1_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW2 & (uint32_t)CONFIG_KEYPAD_ROW2_PORTA) << CONFIG_KEYPAD_ROW2_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW3 & (uint32_t)CONFIG_KEYPAD_ROW3_PORTA) << CONFIG_KEYPAD_ROW3_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW4 & (uint32_t)CONFIG_KEYPAD_ROW4_PORTA) << CONFIG_KEYPAD_ROW4_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW5 & (uint32_t)CONFIG_KEYPAD_ROW5_PORTA) << CONFIG_KEYPAD_ROW5_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW6 & (uint32_t)CONFIG_KEYPAD_ROW6_PORTA) << CONFIG_KEYPAD_ROW6_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW7 & (uint32_t)CONFIG_KEYPAD_ROW7_PORTA) << CONFIG_KEYPAD_ROW7_PIN)
};

static const uint32_t keypad_output_masks_b[] = {
    (((uint32_t)CONFIG_KEYPAD_ROW0 & ~(uint32_t)CONFIG_KEYPAD_ROW0_PORTA) << CONFIG_KEYPAD_ROW0_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW1 & ~(uint32_t)CONFIG_KEYPAD_ROW1_PORTA) << CONFIG_KEYPAD_ROW1_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW2 & ~(uint32_t)CONFIG_KEYPAD_ROW2_PORTA) << CONFIG_KEYPAD_ROW2_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW3 & ~(uint32_t)CONFIG_KEYPAD_ROW3_PORTA) << CONFIG_KEYPAD_ROW3_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW4 & ~(uint32_t)CONFIG_KEYPAD_ROW4_PORTA) << CONFIG_KEYPAD_ROW4_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW5 & ~(uint32_t)CONFIG_KEYPAD_ROW5_PORTA) << CONFIG_KEYPAD_ROW5_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW6 & ~(uint32_t)CONFIG_KEYPAD_ROW6_PORTA) << CONFIG_KEYPAD_ROW6_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW7 & ~(uint32_t)CONFIG_KEYPAD_ROW7_PORTA) << CONFIG_KEYPAD_ROW7_PIN)
};

#else
static const uint32_t keypad_input_masks_a[] = {
    (((uint32_t)CONFIG_KEYPAD_ROW0 & (uint32_t)CONFIG_KEYPAD_ROW0_PORTA) << CONFIG_KEYPAD_ROW0_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW1 & (uint32_t)CONFIG_KEYPAD_ROW1_PORTA) << CONFIG_KEYPAD_ROW1_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW2 & (uint32_t)CONFIG_KEYPAD_ROW2_PORTA) << CONFIG_KEYPAD_ROW2_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW3 & (uint32_t)CONFIG_KEYPAD_ROW3_PORTA) << CONFIG_KEYPAD_ROW3_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW4 & (uint32_t)CONFIG_KEYPAD_ROW4_PORTA) << CONFIG_KEYPAD_ROW4_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW5 & (uint32_t)CONFIG_KEYPAD_ROW5_PORTA) << CONFIG_KEYPAD_ROW5_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW6 & (uint32_t)CONFIG_KEYPAD_ROW6_PORTA) << CONFIG_KEYPAD_ROW6_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW7 & (uint32_t)CONFIG_KEYPAD_ROW7_PORTA) << CONFIG_KEYPAD_ROW7_PIN)
};

static const uint32_t keypad_input_masks_b[] = {
    (((uint32_t)CONFIG_KEYPAD_ROW0 & ~(uint32_t)CONFIG_KEYPAD_ROW0_PORTA) << CONFIG_KEYPAD_ROW0_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW1 & ~(uint32_t)CONFIG_KEYPAD_ROW1_PORTA) << CONFIG_KEYPAD_ROW1_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW2 & ~(uint32_t)CONFIG_KEYPAD_ROW2_PORTA) << CONFIG_KEYPAD_ROW2_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW3 & ~(uint32_t)CONFIG_KEYPAD_ROW3_PORTA) << CONFIG_KEYPAD_ROW3_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW4 & ~(uint32_t)CONFIG_KEYPAD_ROW4_PORTA) << CONFIG_KEYPAD_ROW4_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW5 & ~(uint32_t)CONFIG_KEYPAD_ROW5_PORTA) << CONFIG_KEYPAD_ROW5_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW6 & ~(uint32_t)CONFIG_KEYPAD_ROW6_PORTA) << CONFIG_KEYPAD_ROW6_PIN),
    (((uint32_t)CONFIG_KEYPAD_ROW7 & ~(uint32_t)CONFIG_KEYPAD_ROW7_PORTA) << CONFIG_KEYPAD_ROW7_PIN)
};

static const uint32_t keypad_output_masks_a[] = {
    (((uint32_t)CONFIG_KEYPAD_COL0 & (uint32_t)CONFIG_KEYPAD_COL0_PORTA) << CONFIG_KEYPAD_COL0_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL1 & (uint32_t)CONFIG_KEYPAD_COL1_PORTA) << CONFIG_KEYPAD_COL1_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL2 & (uint32_t)CONFIG_KEYPAD_COL2_PORTA) << CONFIG_KEYPAD_COL2_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL3 & (uint32_t)CONFIG_KEYPAD_COL3_PORTA) << CONFIG_KEYPAD_COL3_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL4 & (uint32_t)CONFIG_KEYPAD_COL4_PORTA) << CONFIG_KEYPAD_COL4_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL5 & (uint32_t)CONFIG_KEYPAD_COL5_PORTA) << CONFIG_KEYPAD_COL5_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL6 & (uint32_t)CONFIG_KEYPAD_COL6_PORTA) << CONFIG_KEYPAD_COL6_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL7 & (uint32_t)CONFIG_KEYPAD_COL7_PORTA) << CONFIG_KEYPAD_COL7_PIN)
};

static const uint32_t keypad_output_masks_b[] = {
    (((uint32_t)CONFIG_KEYPAD_COL0 & ~(uint32_t)CONFIG_KEYPAD_COL0_PORTA) << CONFIG_KEYPAD_COL0_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL1 & ~(uint32_t)CONFIG_KEYPAD_COL1_PORTA) << CONFIG_KEYPAD_COL1_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL2 & ~(uint32_t)CONFIG_KEYPAD_COL2_PORTA) << CONFIG_KEYPAD_COL2_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL3 & ~(uint32_t)CONFIG_KEYPAD_COL3_PORTA) << CONFIG_KEYPAD_COL3_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL4 & ~(uint32_t)CONFIG_KEYPAD_COL4_PORTA) << CONFIG_KEYPAD_COL4_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL5 & ~(uint32_t)CONFIG_KEYPAD_COL5_PORTA) << CONFIG_KEYPAD_COL5_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL6 & ~(uint32_t)CONFIG_KEYPAD_COL6_PORTA) << CONFIG_KEYPAD_COL6_PIN),
    (((uint32_t)CONFIG_KEYPAD_COL7 & ~(uint32_t)CONFIG_KEYPAD_COL7_PORTA) << CONFIG_KEYPAD_COL7_PIN)
};
#endif

AOKeypad::AOKeypad() :
    QActive((QStateHandler)&AOKeypad::InitialPseudoState), 
    m_id(AO_KEYPAD), m_name("Keypad"),m_syncTimer(this, KEYPAD_SYNC) {}

QState AOKeypad::InitialPseudoState(AOKeypad * const me, QEvt const * const e) {
    (void)e;

    me->subscribe(KEYPAD_START_REQ);
    me->subscribe(KEYPAD_STOP_REQ);
    me->subscribe(KEYPAD_SYNC);
    me->subscribe(KEYPAD_WRITE_REG_REQ);
    me->subscribe(KEYPAD_READ_REG_REQ);
      
    return Q_TRAN(&AOKeypad::Root);
}

QState AOKeypad::Root(AOKeypad * const me, QEvt const * const e) {
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
            status = Q_TRAN(&AOKeypad::Stopped);
            break;
        }
		case KEYPAD_STOP_REQ: {
			LOG_EVENT(e);
			status = Q_TRAN(&AOKeypad::Stopped);
			break;
		}
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}

QState AOKeypad::Stopped(AOKeypad * const me, QEvt const * const e) {
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
        case KEYPAD_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new KeypadStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_HANDLED();
            break;
        }
        case KEYPAD_START_REQ: {
            LOG_EVENT(e);
			
            //set inputs
            uint32_t mask = (uint32_t)KEYPAD_INPUT_MASK_PORTA;
			gpio_dirclr_bulk(PORTA, mask);
			gpio_pullenset_bulk(mask, PORTA);
			gpio_outclr_bulk(PORTA, mask);

            mask = (uint32_t)KEYPAD_INPUT_MASK_PORTB;
			gpio_dirclr_bulk(PORTB, mask);
			gpio_pullenset_bulk(mask, PORTB);
			gpio_outclr_bulk(PORTB, mask);

            //set outputs
            mask = (uint32_t)KEYPAD_OUTPUT_MASK_PORTA;
			gpio_dirset_bulk(PORTA, mask);
			gpio_outclr_bulk(PORTA, mask);

            mask = (uint32_t)KEYPAD_OUTPUT_MASK_PORTB;
			gpio_dirset_bulk(PORTB, mask);
			gpio_outclr_bulk(PORTB, mask);


            memset(me->m_state, 0, 64); //initialize state array to 0
            me->m_status.reg = 0;
            me->m_inten.reg = 0;

            KeypadStartReq const &r = static_cast<KeypadStartReq const &>(*e);
			me->m_fifo = r.getFifo();
            me->m_fifo->Reset();


			Evt const &req = EVT_CAST(*e);
			Evt *evt = new KeypadStartCfm(req.GetSeq(), ERROR_SUCCESS);
			QF::PUBLISH(evt, me);
			
			status = Q_TRAN(&AOKeypad::Started);
            break;
        }
        default: {
            status = Q_SUPER(&AOKeypad::Root);
            break;
        }
    }
    return status;
}

QState AOKeypad::Started(AOKeypad * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
			me->m_syncTimer.armX(CONFIG_KEYPAD_SYNC_INTERVAL, CONFIG_KEYPAD_SYNC_INTERVAL);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            me->m_syncTimer.disarm();
            status = Q_HANDLED();
            break;
        }
		case KEYPAD_STOP_REQ: {
			LOG_EVENT(e);
			Evt const &req = EVT_CAST(*e);
			Evt *evt = new KeypadStopCfm(req.GetSeq(), ERROR_SUCCESS);
			QF::PUBLISH(evt, me);
			status = Q_TRAN(AOKeypad::Stopped);
			break;
		}
        case KEYPAD_SYNC: {
            //LOG_EVENT(e);

            keyState *ks;
            uint32_t ina, inb;
            bool val;
            keyEvent keyevent;
            #if KEYPAD_SCAN_ROWS
            for(int i=0; i<KEYPAD_MAX_ROWS; i++){
                if((1 << i) & KEYPAD_ACTIVE_ROWS){
            #else
            for(int i=0; i<KEYPAD_MAX_COLS; i++){
                if((1 << i) & KEYPAD_ACTIVE_COLS){
            #endif
                    bool ouput_is_port_a = true;
                    //set the row high
                    if (keypad_output_masks_a[i]) {
                        gpio_outset_bulk(PORTA, keypad_output_masks_a[i]);
                    } else {
                        Q_ASSERT(keypad_output_masks_b[i]);
                        gpio_outset_bulk(PORTB, keypad_output_masks_b[i]);
                        ouput_is_port_a = false;
                    }

                    //short delay
                    for(int __tmr = 0; __tmr<100; __tmr++) __asm__ volatile ("NOP;");
                    //read everything at once
                    ina = gpio_read_bulk(PORTA) & KEYPAD_INPUT_MASK_PORTA;
                    inb = gpio_read_bulk(PORTB) & KEYPAD_INPUT_MASK_PORTB;

                    #if KEYPAD_SCAN_ROWS
                    for(int j=0; j<KEYPAD_MAX_COLS; j++){
                        if((1 << j) & KEYPAD_ACTIVE_COLS){
                    #else
                    for(int j=0; j<KEYPAD_MAX_ROWS; j++){
                        if((1 << j) & KEYPAD_ACTIVE_ROWS){
                    #endif
                            val = (ina & keypad_input_masks_a[j]) > 0 ||
                                  (inb & keypad_input_masks_b[j]) > 0;

                            keyevent.bit.TYPE = KEYPAD_TYPE_KEY;
                            keyevent.bit.key.EDGE = 0;
                            #if KEYPAD_SCAN_ROWS
                            keyevent.bit.key.NUM = KEYPAD_EVENT(i, j);
                            #else
                            keyevent.bit.key.NUM = KEYPAD_EVENT(j, i);
                            #endif
                            ks = &me->m_state[keyevent.bit.key.NUM];

                            if(ks->bit.ACTIVE & KEYPAD_HIGH && val){
                                keyevent.bit.key.EDGE = KEYPAD_EDGE_HIGH;
                                me->m_fifo->Write(keyevent.reg, 2);
                            }
                            if(ks->bit.ACTIVE & KEYPAD_LOW && !val){
                                keyevent.bit.key.EDGE = KEYPAD_EDGE_LOW;
                                me->m_fifo->Write(keyevent.reg, 2);
                            }
                            if(ks->bit.ACTIVE & KEYPAD_RISING && !ks->bit.STATE && val){
                                keyevent.bit.key.EDGE = KEYPAD_EDGE_RISING;
                                me->m_fifo->Write(keyevent.reg, 2);
                            }
                            if(ks->bit.ACTIVE & KEYPAD_FALLING && ks->bit.STATE && !val){
                                keyevent.bit.key.EDGE = KEYPAD_EDGE_FALLING;
                                me->m_fifo->Write(keyevent.reg, 2);
                            }

                            ks->bit.STATE = val;
                        }
                    }
                    //set the row back low
                    if (ouput_is_port_a) {
                        gpio_outclr_bulk(PORTA, keypad_output_masks_a[i]);
                    } else {
                        gpio_outclr_bulk(PORTB, keypad_output_masks_b[i]);
                    }
                }
            }

            //create an interrupt if there are events in the FIFO
            if(me->m_fifo->GetUsedCount() > 0){
                me->m_status.bit.DATA_RDY = 1;
                if(me->m_inten.bit.DATA_RDY){
                    //post an interrupt event
                    Evt *evt = new InterruptSetReq( SEESAW_INTERRUPT_KEYPAD_DATA_RDY );
                    QF::PUBLISH(evt, me);
                }
            }
            
            status = Q_HANDLED();
            break;
        }
        case KEYPAD_READ_REG_REQ: {
            LOG_EVENT(e);
            KeypadReadRegReq const &req = static_cast<KeypadReadRegReq const &>(*e);
            Fifo *dest = req.getDest();
			uint8_t reg = req.getReg();

            Evt *evt;

            if(reg == SEESAW_KEYPAD_FIFO){
                me->m_status.bit.DATA_RDY = 0;
                if(me->m_inten.bit.DATA_RDY){
                    //post an interrupt event
                    evt = new InterruptClearReq( SEESAW_INTERRUPT_KEYPAD_DATA_RDY );
                    QF::PUBLISH(evt, me);
                }

                //give the requester our pipe
                evt = new DelegateDataReady(req.getRequesterId(), me->m_fifo);
            }
            else{
                keyEvent keyevent;

                switch (reg){
                    case SEESAW_KEYPAD_STATUS:
                        keyevent.bit.TYPE = KEYPAD_TYPE_STATUS;
                        keyevent.bit.status.STATUS = me->m_status.reg;
                        break;
                    case SEESAW_KEYPAD_COUNT:
                        keyevent.bit.TYPE = KEYPAD_TYPE_COUNT;
                        // NOTE: is this potentially approximate
                        keyevent.bit.count.COUNT = me->m_fifo->GetUsedCount() / 2;
                        break;
                    default:
                        keyevent.bit.TYPE = KEYPAD_TYPE_INVALID;
                        break;
                }

                //return the read register in the default fifo
                evt = new DelegateDataReady(req.getRequesterId());
                dest->Write(keyevent.reg, 2);
            }

            QF::PUBLISH(evt, me);

            status = Q_HANDLED();
            break;
        }
        case KEYPAD_WRITE_REG_REQ: {
            // FIXME: use the keyEvent structure
            LOG_EVENT(e);
            KeypadWriteRegReq const &req = static_cast<KeypadWriteRegReq const &>(*e);
            uint32_t c = req.getValue();
            switch (req.getReg()){
                case SEESAW_KEYPAD_INTENSET:
                    me->m_inten.reg |= c >> 8;
                    break;
                case SEESAW_KEYPAD_INTENCLR:
                    me->m_inten.reg &= ~(c >> 8);
                    break;
                case SEESAW_KEYPAD_EVENT:
                    //turn an event on or off
                    keyState *ks;
                    ks = &me->m_state[(c >> 8) & 0xFF];

                    if(c & 0x01) //activate the selected edges
                        ks->bit.ACTIVE |= (c >> 1);
                    else //deactivate the selected edges
                        ks->bit.ACTIVE &= ~(c >> 1);

                    break;
                default:
                    break;
            }
            status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&AOKeypad::Root);
            break;
        }
    }
    return status;
}
