//============================================================================
// Product: Blinky AO
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
#include "qpcpp.h"
#include "qp_extras.h"

#include "hsm_id.h"
#include "event.h"

#include "bsp.h"
#include "bsp_gpio.h"

#include "SeesawConfig.h"
#include "PinMap.h"

#include "AOBlinky.h"

Q_DEFINE_THIS_FILE

#if CONFIG_BLINK

#define BLINK_OUTPUT_MASK (1UL << PIN_ACTIVITY_LED)

//............................................................................
AOBlinky::AOBlinky()
  : QActive((QStateHandler)&AOBlinky::InitialPseudoState),
    m_name("blinky"),
    m_id(AO_BLINKY),
    m_timeEvt(this, BLINK_TIMER)
{
    // empty
}

// HSM definition ............................................................
QState AOBlinky::InitialPseudoState(AOBlinky * const me, QEvt const * const e) {
    (void)e; // unused parameter

    me->subscribe(BLINK_START_REQ);
    me->subscribe(BLINK_STOP_REQ);
    me->subscribe(BLINK_TIMER);

    return Q_TRAN(&AOBlinky::Root);
}

QState AOBlinky::Root(AOBlinky * const me, QEvt const * const e) {
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
            status = Q_TRAN(&AOBlinky::Stopped);
            break;
        }
		case BLINK_STOP_REQ: {
			LOG_EVENT(e);
			status = Q_TRAN(&AOBlinky::Stopped);
			break;
		}
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}

QState AOBlinky::Stopped(AOBlinky * const me, QEvt const * const e) {
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
        case BLINK_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new BlinkStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_HANDLED();
            break;
        }
        case BLINK_START_REQ: {
            LOG_EVENT(e);

            uint32_t mask = BLINK_OUTPUT_MASK;
            gpio_dirclr_bulk(PORTA, mask);
			gpio_outset_bulk(PORTA, mask);
            gpio_init(PORTA, PIN_ACTIVITY_LED, 1); //set as output

			Evt const &req = EVT_CAST(*e);
			Evt *evt = new BlinkStartCfm(req.GetSeq(), ERROR_SUCCESS);
			QF::PUBLISH(evt, me);
			
			status = Q_TRAN(&AOBlinky::On);
            break;
        }
        default: {
            status = Q_SUPER(&AOBlinky::Root);
            break;
        }
    }
    return status;
}



//............................................................................
QState AOBlinky::Off(AOBlinky * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
			
            // arm the time event to expire in half a second and every half second
            me->m_timeEvt.armX(BSP_TICKS_PER_SEC/2U, BSP_TICKS_PER_SEC/2U);

            gpio_write(PORTA, PIN_ACTIVITY_LED, 0); //write low
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            me->m_timeEvt.disarm();
            status = Q_HANDLED();
            break;
        }
		case BLINK_STOP_REQ: {
			LOG_EVENT(e);
			Evt const &req = EVT_CAST(*e);
			Evt *evt = new BlinkStopCfm(req.GetSeq(), ERROR_SUCCESS);
			QF::PUBLISH(evt, me);
			status = Q_TRAN(AOBlinky::Stopped);
			break;
		}
        case BLINK_TIMER: {
            status = Q_TRAN(&AOBlinky::On);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}
//............................................................................
QState AOBlinky::On(AOBlinky * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
			
            // arm the time event to expire in half a second and every half second
            me->m_timeEvt.armX(BSP_TICKS_PER_SEC/2U, BSP_TICKS_PER_SEC/2U);

            gpio_write(PORTA, PIN_ACTIVITY_LED, 1); //write hi
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            me->m_timeEvt.disarm();
            status = Q_HANDLED();
            break;
        }
		case BLINK_STOP_REQ: {
			LOG_EVENT(e);
			Evt const &req = EVT_CAST(*e);
			Evt *evt = new BlinkStopCfm(req.GetSeq(), ERROR_SUCCESS);
			QF::PUBLISH(evt, me);
			status = Q_TRAN(AOBlinky::Stopped);
			break;
		}

        case BLINK_TIMER: {
            status = Q_TRAN(&AOBlinky::Off);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}

#endif
