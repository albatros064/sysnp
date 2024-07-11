#include "serial.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>

namespace sysnp {

namespace nbus {


Serial::~Serial() {
    ttyRunning = false;
    if (ttyWriteThread.joinable()) {
        ttyWriteThread.join();
    }
    if (ttyReadThread.joinable()) {
        ttyReadThread.join();
    }

    close(ttyHandle);
}

void Serial::init(const libconfig::Setting &setting) {
    int intAssignment;

    setting.lookupValue("tty", ttyFile);

    setting.lookupValue("ioAddress", ioAddress);
    setting.lookupValue("interrupt", intAssignment);

    if (intAssignment == 0) {
        interrupt = NBusSignal::Interrupt0;
    }
    else if (intAssignment == 1) {
        interrupt = NBusSignal::Interrupt1;
    }
    else if (intAssignment == 2) {
        interrupt = NBusSignal::Interrupt2;
    }
    else { //if (intAssignment == 3) {
        interrupt = NBusSignal::Interrupt3;
    }

    lastOutData = false;
    hasOutData = false;
}

void Serial::postInit() {
    try {
        ttyHandle = open(ttyFile.c_str(), O_RDWR);
        if (!ttyHandle) {
            machine->debug(5, "Serial failure: couldn't open serial.");
            throw 0;
        }

        struct termios tty;
        if (tcgetattr(ttyHandle, &tty) != 0) {
            machine->debug(5, "Serial failure: couldn't read serial details.");
            std::cout << "Couldn't read serial details." << std::endl;
            throw 1;
        }

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        // Save tty settings, also checking for error
        if (tcsetattr(ttyHandle, TCSANOW, &tty) != 0) {
            machine->debug(5, "Serial failure: error in tcsetattr()");
            throw 2;
        }

        ttyRunning = true;
        ttyReadThread  = std::thread(ttyExecute, std::ref(*this), true);
        ttyWriteThread = std::thread(ttyExecute, std::ref(*this), false);
    }
    catch (int e) {
        ttyRunning = false;
        machine->debug(2, "Serial failure: not initialized.");
    }
}

void Serial::clockUp() {
    machine->debug("Serial::clockUp()");

    if (status != SerialStatus::SerialReady && status != SerialStatus::SerialCleanup) {
        if (status == SerialStatus::SerialReadLatency) {
            machine->debug(6, "Serial::clockUp() SerialReadLatency");

            uint16_t data = 0;

            inDataMutex.lock();

            if (hasInData) {
                machine->debug(6, "Serial::clockUp() - found data");
                data = inData;
                data |= 0x100;
            }
            if (hasOutData) {
                data |= 0x200;
            }

            interface->assertSignal(NBusSignal::Data, data);
            interface->deassertSignal(interrupt);
            hasInData = false;

            inDataMutex.unlock();
        }
        else if (status == SerialStatus::SerialWriteLatency) {
            machine->debug(6, "Serial::clockUp() SerialWriteLatency");
            if (writeLatch & 1) {
                outDataMutex.lock();

                outData = (uint8_t) dataLatch & 0xff;
                hasOutData = true;

                outDataMutex.unlock();
            }
        }
        status = SerialStatus::SerialCleanup;
    }
    else if (status == SerialStatus::SerialCleanup) {
        machine->debug(6, "Serial::clockUp() SerialCleanup");
        interface->deassertSignal(NBusSignal::Data);
        status = SerialStatus::SerialReady;
    }

    if (hasInData || (lastOutData != hasOutData)) {
        machine->debug(6, "Serial::clockUp() Interrupting");
        interface->assertSignal(interrupt, 1);
    }
    lastOutData = hasOutData;
}

void Serial::clockDown() {
    machine->debug("Serial::clockDown()");

    uint32_t read  = interface->senseSignal(NBusSignal::ReadEnable);
    uint32_t write = interface->senseSignal(NBusSignal::WriteEnable);

    if (status == SerialStatus::SerialReady) {
        machine->debug("Latching bus lines");
        if (read || write) {
            addressLatch = interface->senseSignal(NBusSignal::Address) & 0xfffffffe;
            dataLatch    = interface->senseSignal(NBusSignal::Data   );
            readLatch  = read;
            writeLatch = write;

            if (addressLatch == ioAddress && readLatch) {
                if (writeLatch) {
                    machine->debug("Serial - writing");
                    status = SerialStatus::SerialWriteLatency;
                }
                else {
                    machine->debug("Serial - reading");
                    status = SerialStatus::SerialReadLatency;
                }
            }
        }
    }
}

std::string Serial::command(std::stringstream &input) {
    std::stringstream output;
    output << "Serial" << std::endl;
    output << "hasInData  : " << (hasInData   ? "y" : "n") << std::endl;
    output << "lastOutData: " << (lastOutData ? "y" : "n") << std::endl;
    output << "hasOutData : " << (hasOutData  ? "y" : "n") << std::endl;
    output << "inData     : " << std::setw(2) << std::setfill('0') << (int) inData << std::endl;
    output << "Ok.";
    return output.str();
}

void ttyExecute(Serial& serial, bool reading) {
    while (serial.ttyRunning) {
        if (reading) {
            serial.ttyRead();
        }
        else {
            serial.ttyWrite();
        }
    }
}

void Serial::ttyRead() {
    char buffer;
    int bytesRead = read(ttyHandle, &buffer, 1);

    if (!bytesRead) {
        return;
    }

    machine->debug(6, "Serial byte read");

    inDataMutex.lock();
    inData = buffer;
    hasInData = true;
    inDataMutex.unlock();
}

void Serial::ttyWrite() {
    if (hasOutData) {
        machine->debug(6, "Serial byte written");
        write(ttyHandle, &outData, 1);
        hasOutData = false;
    }
}

}; // namespace nbus
}; // namespace sysnp
