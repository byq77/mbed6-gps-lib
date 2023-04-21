#include "gps.h"

const char GPS::GGA_SENTENCE_FORMAT[] = "%*c%*cGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%*c,%f,%*c,%f,%d*%lx"; ///< GGA sentence format string for sscanf

GPS::GPS(PinName tx, PinName rx, int baud)
    : SerialBase(tx, rx, baud)
{
    enable_rx_irq();
}

GPS::~GPS(){};

bool GPS::update()
{
    _mutex.lock();

    readSentence();

    switch (_state)
    {
        case InputState::LOOK_FOR_DOLLA:
        case InputState::READING_MESSAGE:
            _mutex.unlock();
            return false;
        case InputState::MESSAGE_OVERFLOW:
            // reset in case of overflow
            _data_read = 0;
            _state = InputState::LOOK_FOR_DOLLA;
            _mutex.unlock();
            return false;
        default:
            break;
    }

    float time_raw;
    int fix_raw;

    // parse GPGGA message
    if (sscanf(_sentence_buffer, GGA_SENTENCE_FORMAT, &time_raw,
               &latitude, &ns, &longitude, &ew, &fix_raw, &nsats, &hdop,
               &alt, &geoid, &age_of_diff, &diff_ref_station, &checksum) >= 1)
    {
        _state = InputState::LOOK_FOR_DOLLA;
        fix = static_cast<FixType>(fix_raw);
        if(fix == FixType::FIX_NOT_AVAILABLE)
        {
            longitude = 0.0;
            latitude = 0.0;
            nsats = 0;
            hdop = 0.0;
            alt = 0.0;
            geoid = 0.0;
            age_of_diff = 0.0;
            diff_ref_station = 0;

            _mutex.unlock();
            return false;           
        }
        else
        {
            // GPGGA format according http://aprs.gids.nl/nmea/#gga
            // time (float), lat (f), (N/S) (c), long (f), (E/W) (c), fix (d), sats (d),
            // hdop (float), altitude (float), M, geoid (float), M, , ,
            // GPGGA,092010.000,5210.9546,N,00008.8913,E,1,07,1.3,9.7,M,47.0,M,,0000*5D            

            // format utc time to beijing time,add 8 time zone
            time_raw += 80000.00f;
            hour = int(time_raw) / 10000;
            minutes = (int(time_raw) % 10000) / 100;
            seconds = int(time_raw) % 100;

            _mutex.unlock();
            return true;            
        }
    }
    _mutex.unlock();
    return false;
}

void GPS::readSentence()
{
    char *ptr = static_cast<char *>(_sentence_buffer);
    char ch;

    if(_data_read == MAX_SENTENCE_SIZE)
    {
        _state = InputState::MESSAGE_OVERFLOW;
        return;
    }
    else
    {
        ptr+=_data_read;
    }
    
    while (!_rxbuf.empty()) 
    {
        _rxbuf.pop(ch);
        switch (_state)
        {
            case InputState::LOOK_FOR_DOLLA:
                if (ch == '$')
                    _state = InputState::READING_MESSAGE;
                break;
            case InputState::READING_MESSAGE:
                if(ch == '\r')
                {
                    *ptr = 0;
                    _state = InputState::MESSAGE_READ;
                    return;
                }
                else
                {
                    *ptr++ = ch;
                    _data_read++;
                }
                break;
            default:
                break;
        }
    }    

    core_util_critical_section_enter();
    if (_rx_enabled && !_rx_irq_enabled) {
        // only read from hardware in one place
        GPS::rx_irq();
        if (!_rxbuf.full()) {
            enable_rx_irq();
        }
    }
    core_util_critical_section_exit();
}

/* These are all called from critical section
 * Attatch IRQ routines to the serial device.
 */
void GPS::enable_rx_irq()
{
    SerialBase::attach(callback(this, &GPS::rx_irq), RxIrq);
    _rx_irq_enabled = true;
}

void GPS::disable_rx_irq()
{
    SerialBase::attach(NULL, RxIrq);
    _rx_irq_enabled = false;
}

void GPS::enable_tx_irq()
{
    SerialBase::attach(callback(this, &GPS::tx_irq), TxIrq);
    _tx_irq_enabled = true;
}

void GPS::disable_tx_irq()
{
    SerialBase::attach(NULL, TxIrq);
    _tx_irq_enabled = false;
}

void GPS::rx_irq(void)
{
    // Fill in the receive buffer if the peripheral is readable
    // and receive buffer is not full.
    while (!_rxbuf.full() && SerialBase::readable()) {
        char data = SerialBase::_base_getc();
        _rxbuf.push(data);
    }

    if (_rx_irq_enabled && _rxbuf.full()) {
        disable_rx_irq();
    }
}

// Also called from write to start transfer
void GPS::tx_irq(void)
{
    char data;

    // Write to the peripheral if there is something to write
    // and if the peripheral is available to write.
    while (SerialBase::writeable() && _txbuf.pop(data)) {
        SerialBase::_base_putc(data);
    }

    if (_tx_irq_enabled && _txbuf.empty()) {
        disable_tx_irq();
    }
}