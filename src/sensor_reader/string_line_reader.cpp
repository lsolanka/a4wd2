#include <istream>
#include <sensor_reader/string_line_reader.hpp>

namespace asio = boost::asio;

namespace a4wd2::sensor_reader
{

serial_string_line_reader::int_type serial_string_line_reader::underflow()
{
    boost::asio::read_until(m_port, *this, '\n');
    return boost::asio::streambuf::underflow();
}

serial_string_line_reader::serial_string_line_reader(
        const std::string& port, int baud_rate) :
    m_io_service(),
    m_port(m_io_service),
    m_istream(this)
{

    boost::system::error_code ec = m_port.open(port, ec);
    m_port.set_option(asio::serial_port_base::baud_rate(baud_rate));

    if (!m_port.is_open()) {
        throw std::runtime_error(
            std::string("Could not open serial port: ") + port + ": "
                  + ec.message());
    }
}

std::istream& serial_string_line_reader::get_stream()
{
    return m_istream;
}

} // namespace a4wd2::sensor_reader
