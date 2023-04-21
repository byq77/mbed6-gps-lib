/**
 * @file gps.h
 * @author byq77
 * @brief GPS library
 * @version 0.1
 * @sa http://lefebure.com/articles/nmea-gga/
 * @sa https://www.waveshare.com/w/upload/5/57/NMEA0183.pdf
 * @date 2023-04-20
 */
#pragma once

#include "mbed.h"

#define MAX_SENTENCE_SIZE 84

/**
 * @brief A GPS interface for reading from a serial GPS module like NEO-6M
 */
class GPS : private SerialBase, private NonCopyable<GPS>
{
public:
    static const char GGA_SENTENCE_FORMAT[]; ///< GGA sentence format string for sscanf

    /**
     * @brief GPS Fix Indicator
     */
    enum class FixType : int
    {
        FIX_NOT_AVAILABLE = 0,
        GPS_FIX = 1,
        DGPS_FIX = 2,
        PPS_FIX = 3,
        RTK_FIX = 4,
        RTK_FLOAT = 5
    };

    enum class InputState 
    {
        LOOK_FOR_DOLLA,
        READING_MESSAGE,
        MESSAGE_READ,
        MESSAGE_OVERFLOW
    };

    /**
     * @name UTC+8 time
     * Beijig time format(shi,fen,miao).
     */
    ///@{
    int hour{};
    int minutes{};
    int seconds{};
    ///@}
    float latitude{};
    char ns{}; ///< North or South marker
    float longitude{};
    char ew{};                     ///< East or West marker
    FixType fix{FixType::FIX_NOT_AVAILABLE}; ///< GPS Fix Indicator
    int nsats{};                   ///< Number of satellites in use
    float hdop{};                  ///< Horizontal Dilution of precision
    float alt{};                   ///< Elevation or Altitude in Meters above mean sea level.
    float geoid{};                 ///< Height of the Geoid (mean sea level) above the Ellipsoid, in Meters. 

    /**
     * @brief Age of correction
     *
     * Age of correction data for DGPS and RTK solutions, in Seconds.
     * Some receivers output this rounded to the nearest second, some will also include tenths of a second.
     */
    float age_of_diff{};

    /**
     * @brief Correction station ID number, 4-digit
     */
    int diff_ref_station{};

    uint32_t checksum{};

    /**
     * @brief Construct a new GPS object
     *
     * Create the GPS interface, connected to the specified serial port and speed.
     * For example, GlobalSat EM406-A (e.g. on SparkFun GPS Shield) is 4800 Baud,
     * Adafruit Ultimate GPSv3 (connected to serial) is 9600 Baud
     *
     * @param tx TX serial
     * @param rx RX serial
     * @param baud baudrate, default is 9600
     */
    GPS(PinName tx, PinName rx, int baud = 9600);

    ~GPS() override;

    /** 
     * @brief Update the object.
     *
     * Parses the incoming serial stream and updates the GPS object with a new sample data.
     * @return true if there is a valid GPS sentence in the stream.
     */
    bool update();

private:
    char _sentence_buffer[256];
    InputState _state{InputState::LOOK_FOR_DOLLA};
    size_t _data_read{0};
    bool _tx_irq_enabled = false;
    bool _rx_irq_enabled = false;
    bool _tx_enabled = true;
    bool _rx_enabled = true;
    CircularBuffer<char, MBED_CONF_DRIVERS_UART_SERIAL_RXBUF_SIZE> _rxbuf;
    CircularBuffer<char, MBED_CONF_DRIVERS_UART_SERIAL_TXBUF_SIZE> _txbuf;
    PlatformMutex _mutex;

private:
    void readSentence();

    /** Enable processing of byte reception IRQs and register a callback to
     * process them.
     */
    void enable_rx_irq();

    /** Disable processing of byte reception IRQs and de-register callback to
     * process them.
     */
    void disable_rx_irq();

    /** Enable processing of byte transmission IRQs and register a callback to
     * process them.
     */
    void enable_tx_irq();

    /** Disable processing of byte transmission IRQs and de-register callback to
     * process them.
     */
    void disable_tx_irq();

    /** ISRs for serial
     *  Routines to handle interrupts on serial pins.
     *  Copies data into Circular Buffer.
     */
    void tx_irq(void);
    void rx_irq(void);
};