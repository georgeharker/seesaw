//============================================================================
// Product: Simple Blinky example
// Last updated for version 6.6.0
// Last updated on  2019-10-14
//
//                    Q u a n t u m  L e a P s
//                    ------------------------
//                    Modern Embedded Software
//
// Copyright (C) 2005-2019 Quantum Leaps. All rights reserved.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <www.gnu.org/licenses>.
//
// Contact information:
// <www.state-machine.com>
// <info@state-machine.com>
//============================================================================
#ifndef BLINKY_HPP
#define BLINKY_HPP

#include "qp_extras.h"

#include "hsm_id.h"

using namespace QP;

class AOBlinky : public QActive {
private:
    char const * m_name;
    uint8_t m_id;
    QTimeEvt m_timeEvt;

    enum {
        EVT_QUEUE_COUNT = 16,
    };
    QEvt const *m_evtQueueStor[EVT_QUEUE_COUNT];

public:
    AOBlinky();
    void Start(uint8_t prio) {
        QActive::start(prio, m_evtQueueStor, ARRAY_COUNT(m_evtQueueStor), NULL, 0);
    }

protected:
    static QState InitialPseudoState(AOBlinky * const me, QEvt const * const e);
    static QState Root(AOBlinky * const me, QEvt const * const e);
    static QState Stopped(AOBlinky * const me, QEvt const * const e);
    static QState Off(AOBlinky * const me, QEvt const * const e);
    static QState On(AOBlinky * const me, QEvt const * const e);
};

#endif // BLINKY_HPP
