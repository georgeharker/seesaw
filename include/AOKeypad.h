
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


#ifndef AO_KEYPAD_H
#define AO_KEYPAD_H

#include "qpcpp.h"
#include "qp_extras.h"

#include "hsm_id.h"

using namespace QP;
using namespace FW;

enum {
    KEYPAD_TYPE_KEY = 0,
    KEYPAD_TYPE_COUNT = 1,
    KEYPAD_TYPE_STATUS = 2,
    KEYPAD_TYPE_INVALID = 0xFF
};

enum {
    KEYPAD_EDGE_HIGH = 0,
    KEYPAD_EDGE_LOW,
    KEYPAD_EDGE_FALLING,
    KEYPAD_EDGE_RISING
};

union keyState {
    struct {
        //the current state of the key
        uint8_t STATE: 1;

        //the registered events for that key
        uint8_t ACTIVE: 4;
    } bit;
    uint8_t reg;
};

union keyEvent {
    struct {
        uint8_t TYPE: 8;
        union {
            struct {
                uint8_t EDGE: 2;
                uint8_t NUM: 6; //64 events max
            } key;
            struct {
                uint8_t COUNT: 8;
            } count;
            struct {
                uint8_t STATUS: 8;
            } status;
        };
    } bit;
    uint8_t reg[2];
};
// TYPE = COUNT,rem etc, count down or preamble n bytes, or END

class AOKeypad : public QActive {
public:
    AOKeypad();
    ~AOKeypad() {}
    void Start(uint8_t prio) {
        QActive::start(prio, m_evtQueueStor, ARRAY_COUNT(m_evtQueueStor), NULL, 0);
    }

protected:
    static QState InitialPseudoState(AOKeypad * const me, QEvt const * const e);
    static QState Root(AOKeypad * const me, QEvt const * const e);
    static QState Stopped(AOKeypad * const me, QEvt const * const e);
    static QState Started(AOKeypad * const me, QEvt const * const e);

    enum {
        EVT_QUEUE_COUNT = 8,
    };
    QEvt const *m_evtQueueStor[EVT_QUEUE_COUNT];
    uint8_t m_id;
	uint16_t m_nextSequence;
    char const * m_name;

    QTimeEvt m_syncTimer;

    Fifo *m_fifo;

    keyState m_state[64];

    // The status register
    union status {
        
        struct {
            /* 0: no error
            *  1: error has occurred
            */ 
            uint8_t ERROR: 1;

            /* 0: no data is ready in the FIFO
            *  1: data is ready in the FIFO
            */ 
            uint8_t DATA_RDY: 1;
        } bit;
        uint8_t reg;
    };
    status m_status;
	
	union inten {
        struct {
            /* fire an interrupt when the FIFO is not empty */
		    uint8_t DATA_RDY: 1;
        } bit;
        uint8_t reg;
	};
	inten m_inten;
};


#endif // AO_KEYPAD_H
