#ifndef Packet_h
#define Packet_h

#include <inttypes.h>

#define PACKET_SIZE 5

typedef union {
  uint16_t i;
  char b[sizeof(uint16_t)];
} ui2bytes_t;

class Packet {
private:
  uint8_t i[PACKET_SIZE];
  uint8_t size = 0;
  uint8_t id = 0;
  uint16_t val = 0;
  uint16_t control = 0;
  uint16_t getControl();
public:
  Packet();
  bool avaible();
  void init(uint8_t id, uint16_t val);
  void write(uint8_t byte);
  void unpack();
  void fill();
  bool isValid();
  void send();
  void clear();
  uint8_t getId();
  uint16_t getVal();
};

#endif
