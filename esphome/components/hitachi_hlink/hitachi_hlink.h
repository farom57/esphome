#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

namespace esphome {
namespace hitachi_hlink {

static const uint8_t SEPARATOR = 0x0D;
static const uint8_t MAX_LENGTH = 64;
static const uint8_t N_PARAMS = 24;

/*
 * H-link Protocol description
 * This component communicates with an Hitachi A/C through the H-link protocol (also called Hi-Kumo). It uses
 * UART communication with a baud rate of 9600, 8 data_bits, ODD parity and 1 stop bit. The ESP send commands
 * to read or modify internal parameters of the A/C. The A/C send back a message containing the value
 * requested or an acknoledgment that the value has been modified.
 * The messages are written using only ascii characters and always ends with a carriage return (0x0D)
 *
 * Commands from the ESP to the A/C:
 *  - Read the parameter at the [address]: "MT P=[address] C=[checksum]\r"
 *      [address] is 4 hexadecimal characters
 *      [checksum] is 4 hexadecimal characters: FFFF minus the sum of all pairs of characters in [address]
 *  - Set the parameter at the [address] with the value [data]: "ST P=[address],[data] C=[checksum]\r"
 *      [address] is 4 hexadecimal characters
 *      [data] is 2, 4, or 20 hexadecimal characters depending on the address
 *      [checksum] is 4 hexadecimal characters: FFFF minus the sum of all pairs of characters in [address]
 *         and [data]
 *
 * Answer from the  A/C:
 *  - Error: "NG TODO
 *  - Read successful: "OK P=[data] C=[checksum]\r"
 *    [data] is 2, 4, or 20 hexadecimal characters depending on the address
 *    [checksum] is 4 hexadecimal characters: FFFF minus the sum of all pairs of characters in [data]
 *  - Acknowledge modified parameter: "H~ TODO
 *
 * The list of the valid address is described hereunder with their signification when it is known.
 */

struct ParamDescriptor {
  const uint16_t addr;   // address
  const uint8_t length;  // number of hexadecimal characters
  const bool readable;
  const bool writable;
  const char description[65];
};

const ParamDescriptor descr[N_PARAMS] = {
    {0x0000, 2, true, true, "Power state 0=OFF 1=ON"},  // 0000-RW: Power state 0=OFF 1=ON
    {0x0001, 4, true, true, "Operation mode 0010=hot 0020=dry 0040=cool 0050=fan 8000=auto"},
    // 0001-RW: Operation mode 0x0010=hot 0x0020=dry 0x0040=cool 0x0050=fan 0x8000=auto
    {0x0002, 2, true, true,
     "Fan speed 0=auto 1=high 2=medium 3=low 4=Silent"},   // 0002-RW: Fan speed 0=auto 1=high 2=medium 3=low 4=Silent
    {0x0003, 4, true, true, "Target termperature in °C"},  // 0003-RW: Target termperature in °C
    {0x0005, 2, true, false, "unknown, always 0x7E=0b01111110"},  // 0005-RO: unknown, always 0x7E=0b01111110
    {0x0006, 2, true, false,
     "Control via remote permission, 00 = allow 01 = no"},  // 0006-RO: Control via remote control permission, 00 = all
                                                            // settings allowed 01 = all settings prohibited (not tested
                                                            // !)
    {0x0007, 2, true, true,
     "unknown, always 00, maybe for swing mode"},                   // 0007-RW: unknown, always 00, maybe for swing mode
    {0x0008, 2, true, false, "unknown, always 00"},                 // 0008-RO: unknown, always 00
    {0x0009, 2, true, false, "unknown, always 00"},                 // 0009-RO: unknown, always 00
    {0x000A, 2, true, false, "unknown, always 00"},                 // 000A-RO: unknown, always 00
    {0x0011, 2, true, false, "unknown, always FF"},                 // 0011-RO: unknown, always FF
    {0x0012, 2, true, false, "unknown, always FF"},                 // 0012-RO: unknown, always FF
    {0x0013, 2, true, false, "unknown, always 03"},                 // 0013-RO: unknown, always 03
    {0x0014, 2, true, false, "unknown, always 00"},                 // 0014-RO: unknown, always 00
    {0x0100, 2, true, false, "Current indoor temperature in °C"},   // 0100-RO: Current indoor temperature in °C
    {0x0101, 2, true, false, "unknown, always 0x7E=0b01111110"},    // 0101-RO: unknown, always 0x7E=0b01111110
    {0x0102, 2, true, false, "Current outdoor temperature in °C"},  // 0102-RO: Current outdoor temperature in °C
    {0x0201, 4, true, false, "unknown, always 0000"},               // 0201-RO: unknown, always 0000
    {0x0300, 4, false, true,
     "unknown, write only, 0000=normal 0040=absence"},  // 0300-WO: unknown, write only, 0000=normal 0040=absence
    {0x0301, 4, true, false,
     "Active status 0000=Stand-by FFFF=Active"},                  // 0301-RO: Active status 0000=Stand-by FFFF=Active
    {0x0302, 2, true, false, "Filter status 0=OK 1=BAD"},         // 0302-RO: Filter status 0=OK 1=BAD
    {0x0304, 8, true, false, "Absence 0=disabled 80=activated"},  // 0304-RO: Absence 0=disabled 80=activated
    {0x0800, 2, false, true,
     "Beep (TBC), write only, 00=no beep 07=beep"},  // 0800-WO: Beep (TBC), write only, 00=no beep 07=beep
    {0x0900, 20, true, false, "Model"},  // 0900-RO: model, always 5241442D323551504220 for me --> RAD-25QPB in ascii
};

enum ParamLabels {
  POWER,
  MODE,
  SPEED,
  TARGET,
  U0005,
  REMOTE,
  U0007,
  U0008,
  U0009,
  U000A,
  U0011,
  U0012,
  U0013,
  U0014,
  INDOOR,
  U0101,
  OUTDOOR,
  U0201,
  ABSENCE_SET,
  ACTIVE,
  FILTER,
  ABSENCE,
  BEEP,
  MODEL
};

// to store the state of the AC
union ParamData {
  char *text;
  uint8_t u8;
  uint16_t u16;
  uint32_t u32;
};

class HitachiClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  HitachiClimate();

  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void set_outdoor_temperature_sensor(sensor::Sensor *sens) { outside_temp_sensor = sens; }
  void set_filter_sensor(binary_sensor::BinarySensor *sens) {
    filter_sensor = sens;)

  void set_response_timeout(uint32_t ms){this->timeout_ = ms};
    void set_beeper_feedback(bool state){this->beeper_ = state};

   protected:
    sensor::Sensor *outside_temp_sensor;
    binary_sensor::BinarySensor *filter_sensor;
    float outside_temperature;

    ParamData state_[N_PARAMS];
    char model_[11] = {0};
    bool ready_ = false;  // remains false until the state of the AC has been acquired

    // receive buffer
    char buf_[MAX_LENGTH];
    uint8_t buf_index_ = 0;
    uint32_t start_receive_;

    // receiver state
    enum ReceiverOperation {
      OFF,
      SENDING_READ,
      SENDING_SET,
      RECEIVING_READ,
      RECEIVING_SET,
      READ_COMPLETE,
      SET_COMPLETE,
      FAILED_READ,
      FAILED_SET
    } receiver_{OFF};
    uint8_t current_;  // parameter currently read
    uint8_t retry_;    // number of retry

    uint32_t timeout_ = 500;
    bool beeper_ = true;

    // tell the receiver if their is pending read or set operation.
    // Set operation takes the priority over read after the current parameter is completed
    bool pending_read_ = false;
    bool pending_set_ = false;

    // Send the command to read the current_ parameter
    void send_read_cmd();
    // Send the command to set the current_ parameter
    void send_set_cmd();

    /** convert an  hexadecimal char array to array
     *
     * Convert an array of len hexadecimal char to an array of len/2 bytes, return false in case of incorrect formating
     */
    bool hex2byte(char *buf, uint16_t len, uint8_t *out);

    /// compute the checksum of a byte array
    uint16_t checksum_(uint8_t * in, uint8_t len);

    /// print the state_
    void print_state();

    /// convert state_ to Climate members, return false in case of invalid value
    bool climate2state_();

    /// convert Climate members to state_, return false in case of invalid value
    bool state2climate_();
  };
}  // namespace hitachi_hlink
}  // namespace esphome
