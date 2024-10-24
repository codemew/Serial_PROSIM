#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace boost::asio;

class Serial {
 private:
  /* data */
 public:
  Serial(/* args */);
  ~Serial();
};

Serial::Serial(/* args */) {}

Serial::~Serial() {}

void sendDataToSerialPort(const std::string& portName,
                          const std::string& data) {
  try {
    io_service io;
    serial_port serial(io, portName);

    // Set serial port parameters
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(
        serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::flow_control(
        serial_port_base::flow_control::hardware));

    // Write data to serial port
    write(serial, buffer(data));

    // Flush the output buffer to ensure data is sent
    // serial.flush();

    std::cout << "Data sent: " << data /* << std::endl */;

    // Wait for a brief moment to give the device time to respond
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read response
    char response[128];
    size_t n = serial.read_some(buffer(response, 128));
    std::string status = std::string(response, n);
    status = (status[0] == '*') ? "Success" : status;
    std::cout << "Response: " << status << std::endl << std::endl;

  } catch (boost::system::system_error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

int main() {
  std::string portName =
      "/dev/ttyUSB0";  // Change this to your serial port name
  std::string data = "IDENT\n";
  std::string remoteCmd = "REMOTE\n";

  sendDataToSerialPort(portName, data);
  sendDataToSerialPort(portName, remoteCmd);

  std::cout << "Ready to send commands..." << std::endl;

  std::string inputCmd;
  while (true) {
    std::cout << "$> ";
    std::cin >> inputCmd;
    inputCmd += "\n";
    std::transform(inputCmd.begin(), inputCmd.end(), inputCmd.begin(),
                   ::toupper);

    sendDataToSerialPort(portName, inputCmd);
  }

  return 0;
}
