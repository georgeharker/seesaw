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


#ifndef AO_ENCODER_H
#define AO_ENCODER_H

#include "qpcpp.h"
#include "qp_extras.h"

#include "hsm_id.h"

using namespace QP;
using namespace FW;

enum {
    ENCODER_TYPE_VALUE = 0,
    ENCODER_TYPE_DELTA = 1,
    ENCODER_TYPE_PRESS = 2,
    ENCODER_TYPE_COUNT = 3,
    ENCODER_TYPE_STATUS = 4,
    ENCODER_TYPE_INVALID = 0xFF
};

enum {
    ENCODER_EDGE_HIGH = 0,
    ENCODER_EDGE_LOW,
    ENCODER_EDGE_FALLING,
    ENCODER_EDGE_RISING,
    ENCODER_VALUE_CHANGE,
    ENCODER_DELTA
};

union encoderEvent {
    struct __attribute__((packed)) {
        uint8_t TYPE;
        union __attribute__((packed)) {
            struct __attribute__((packed)) {
                uint8_t ENCODER;
                int16_t DELTA;
            } delta;
            struct __attribute__((packed)) {
                uint8_t ENCODER;
                int16_t VALUE;
            } value;
            struct __attribute__((packed)) {
                uint8_t ENCODER;
                int16_t EDGE;
            } press;
            struct __attribute__((packed)) {
                uint8_t __PACK;
                uint16_t COUNT;
            } count;
            struct __attribute__((packed)) {
                uint8_t ENCODER;
                uint16_t STATUS;
            } status;
        };
   } bit;
    uint8_t reg[4];
};

// TYPE = COUNT,rem etc, count down or preamble n bytes, or END
class AOEncoder : public QActive {
public:
    AOEncoder();
    ~AOEncoder() {}
    void Start(uint8_t prio) {
        QActive::start(prio, m_evtQueueStor, ARRAY_COUNT(m_evtQueueStor), NULL, 0);
    }

    static volatile int32_t m_value[CONFIG_NUM_ENCODERS];
    static volatile int32_t m_delta[CONFIG_NUM_ENCODERS];
    static volatile uint8_t m_enc_prev_state[CONFIG_NUM_ENCODERS];
    static volatile uint8_t m_enc_flags[CONFIG_NUM_ENCODERS];

    // The status register
    union status {
        
        struct {
            /* 1: send event on press edge
             * 0: do not
             * bits are 1<<edge
             * */
            uint8_t ACTIVE;

            /* 0: no error
            *  1: error has occurred
            */ 
            uint8_t ERROR: 1;

            /* 0: the value has not changed since last read
            *  1: the value has changed since last read
            */ 
            uint8_t DATA_RDY: 1;
        } bit;
        uint32_t reg;
    };
    static volatile status m_status[CONFIG_NUM_ENCODERS];

    union inten {
        struct {
            /* fire an interrupt when the value has changed */
		    uint8_t DATA_RDY: 1;
        } bit;
        uint8_t reg;
	};
	static inten m_inten[CONFIG_NUM_ENCODERS];

    static Fifo *m_fifo;

protected:
    static QState InitialPseudoState(AOEncoder * const me, QEvt const * const e);
    static QState Root(AOEncoder * const me, QEvt const * const e);
    static QState Stopped(AOEncoder * const me, QEvt const * const e);
    static QState Started(AOEncoder * const me, QEvt const * const e);

    enum {
        EVT_QUEUE_COUNT = 8,
    };
    QEvt const *m_evtQueueStor[EVT_QUEUE_COUNT];
    uint8_t m_id;
	uint16_t m_nextSequence;
    char const * m_name;
};


#endif // AO_ENCODER_H


