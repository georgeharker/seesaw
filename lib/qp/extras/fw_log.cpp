/*******************************************************************************
 * Copyright (C) Lawrence Lo (https://github.com/galliumstudio). 
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

#include <stdarg.h>
#include <stdio.h>
#include "bsp.h"
#include "qpcpp.h"
#include "fw_pipe.h"
#include "fw_log.h"
#include "event.h"
#include "SeesawConfig.h"
#include "bsp_sercom.h"

Q_DEFINE_THIS_FILE

using namespace QP;

namespace FW {

char const Log::m_truncatedError[] = "<##TRUN##>";

Fifo * Log::m_fifo = NULL;
QSignal Log::m_sig = 0;
bool Log::m_delay_logging = false;

void Log::AddInterface(Fifo *fifo, QSignal sig) {
    FW_LOG_ASSERT(fifo && sig);
    QF_CRIT_STAT_TYPE crit;
    QF_CRIT_ENTRY(crit);
    m_fifo = fifo;
    m_sig = sig;
    QF_CRIT_EXIT(crit);
}

void Log::DeleteInterface() {
    QF_CRIT_STAT_TYPE crit;
    QF_CRIT_ENTRY(crit);
    m_fifo = NULL;
    m_sig = 0;
    QF_CRIT_EXIT(crit);
}

void Log::DelayLogging() {
    m_delay_logging = true;
}
void Log::UndelayLogging() {
    m_delay_logging = false;
}

void Log::Write(char const *buf) {    
    if (m_fifo && m_delay_logging) {
        bool status1 = false;
        bool status2 = false;
        if (m_fifo->IsTruncated()) {
            m_fifo->WriteNoCrit(reinterpret_cast<uint8_t const *>(m_truncatedError), CONST_STRING_LEN(m_truncatedError), &status1);
        }
        if (!m_fifo->IsTruncated()) {
            m_fifo->WriteNoCrit(reinterpret_cast<uint8_t const *>(buf), strlen(buf), &status2);
        }
        // Post MUST be outside critical section.
        if (status1 || status2) {
            Q_ASSERT(m_sig);
            Evt *evt = new Evt(m_sig);
            QF::PUBLISH(evt, NULL);
        }
    } else {
        writeDataUART(CONFIG_LOG_SERCOM, buf);
    }
}


void Log::Print(char const *format, ...) {
#ifdef ENABLE_LOGGING
    Q_ASSERT(format);
    va_list argp;
    va_start(argp, format);

    char __fmt[80];
    char __buf[80];
    snprintf(__fmt,  sizeof(__fmt), "[%li] %s\n", GetSystemMs(), format);
    vsnprintf(__buf,  sizeof(__buf), __fmt, argp);
    Write(__buf);
#endif
}

void Log::Event(char const *name, char const *func, const char *evtName, int sig) {
#ifdef ENABLE_LOGGING
    Q_ASSERT(name && func && sig && evtName);

    // Don't log logging events
    if (sig == m_sig) return;

    char __buf[80];
    snprintf(__buf, sizeof(__buf), "[%li] %s (%s): %s\n", GetSystemMs(),
             name, func, evtName);
    Write(__buf);
#endif
}

void Log::Debug(char const *name, char const *func, char const *format, ...) {
#ifdef ENABLE_LOGGING
    Q_ASSERT(name && func && format);
    va_list argp;
    va_start(argp, format);

    char __fmt[80];
    char __buf[80];
    snprintf(__buf, sizeof(__buf), "[%li] %s, (%s): %s\n", GetSystemMs(),
             name, func, format);
    vsnprintf(__buf,  sizeof(__buf), __fmt, argp);
    Write(__buf);
#endif
}

} // namespace FW
