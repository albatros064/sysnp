#ifndef SYSNP_MAINBOARD_H
#define SYSNP_MAINBOARD_H

namespace SysNP {

using std::vector;

class Mainboard;

class Device {
  public:
    Device(Mainboard &);
    ~Device() {}
  protected:
    Mainboard &mainboard;
};
class Mainboard {
  public:
    Mainboard() {}
    ~Mainboard() {}
  protected:
    vector<Device> devices;
};

class NBus : public Mainboard {
  public:
};

}; // namespace SysNP

#endif

