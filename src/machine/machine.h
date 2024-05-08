#ifndef SYSNP_MACHINE_H
#define SYSNP_MACHINE_H

#include <string>
#include <memory>
#include <map>

#include <mutex>
#include <thread>
#include <chrono>

#include "device.h"

namespace sysnp {

class Device;

enum RunMode {
    SteppingMode,
    FreeRunMode
};

class Machine : public std::enable_shared_from_this<Machine> {
  public:
	Machine() { debugLevel = 0; }
	virtual ~Machine() {}

    bool load(std::string);

    std::shared_ptr<Device> getDevice(std::string);

    bool readFile(std::string,uint8_t*,uint32_t);

	void run();

    void debug(int, std::string);
    void debug(std::string);

  private:
    std::map<std::string,std::shared_ptr<Device>> devices;
    int debugLevel;

    bool clockRunning;
    uint64_t runCycles;
    std::chrono::time_point<std::chrono::steady_clock> runStart;
    std::chrono::time_point<std::chrono::steady_clock> runEnd;
    
    std::thread runThread;
    std::mutex  runThreadMutex;

    void startRunning(int);
    void stopRunning();

    std::shared_ptr<Device> createDevice(std::string);

    friend void machineRun(Machine&, int);
};

void machineRun(Machine&, int);

}; // namepsace

#endif

