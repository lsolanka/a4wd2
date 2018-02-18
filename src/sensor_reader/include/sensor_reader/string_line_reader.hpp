#pragma once

#include <string>
#include <boost/asio.hpp>

namespace a4wd2::sensor_reader
{

/** This reader opens a serial port, and provides a boost::asio::streambuf
 * interface to read the data.
 */
class serial_string_line_reader : public boost::asio::streambuf
{
  public:
    /** Initialise the serial port and the istream object.
     * @param port string containing the port name
     * @param baud_rate baud rate of the serial port
     * @throws std::runtime_error If the port could not be opened.
     */
    serial_string_line_reader(const std::string& port, int baud_rate);

    /** Get a reference to the stream attached to this streambuf object. The
     * istream can be used to read the data. Buferring is on the line level,
     * i.e. at least one line must be received.
     */
    std::istream& get_stream();

    /** Close the serial port. This should shut down anyone who attempts to
     * read from the port.
     *
     * @todo Check that closing the port is actually a safe operation.
     */
    void shutdown() { m_port.close(); }

  protected:
    int_type underflow() override;

  private:
    boost::asio::io_service m_io_service;
    boost::asio::serial_port m_port;
    std::istream m_istream;
};

} // namespace a4wd2::sensor_reader
