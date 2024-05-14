#include "Packet.h"
#include <Arduino.h>
uint16_t Packet::getControl(){
  return val > 255 ? val / 4 + 255 : val * val + 255;
}
Packet::Packet() {
  clear();
}

bool Packet::avaible() {
  if (!Serial.available()) {
    return false;
  }
  clear();
  while (Serial.available()) {
    write(Serial.read());
  }
  unpack();
  return true;
}
void Packet::init(uint8_t i, uint16_t v) {
  clear();
  id = i;
  val = v;
  control = getControl();
  fill();
}

void Packet::write(uint8_t byte) {
  if (size < PACKET_SIZE) {
    i[size] = byte;
    size++;
  }
}

void Packet::unpack() {
  id = i[0];
  ui2bytes_t ui2b;
  for (uint8_t x = 0; x < sizeof(uint16_t); x++) {
    ui2b.b[x] = i[x + 1];
  }
  val = ui2b.i;
  for (uint8_t x = 0; x < sizeof(uint16_t); x++) {
    ui2b.b[x] = i[x + 3];
  }
  control = ui2b.i;
}

void Packet::fill() {
  ui2bytes_t ui2b;
  ui2b.i = val;
  i[0] = id;
  for (uint8_t x = 0; x < sizeof(uint16_t); x++) {
    i[x + 1] = ui2b.b[x];
  }
  ui2b.i = control;
  for (uint8_t x = 0; x < sizeof(uint16_t); x++) {
    i[x + 3] = ui2b.b[x];
  }
}
bool Packet::isValid() {
  uint16_t c = getControl();
  return control == c;
}

void Packet::send() { Serial.write(i, PACKET_SIZE); }

uint8_t Packet::getId() { return id; }
uint16_t Packet::getVal() { return val; }

void Packet::clear() {
  size = 0;
  for (uint8_t x = 0; x < PACKET_SIZE; x++) {
    i[x] = 0;
  }
}