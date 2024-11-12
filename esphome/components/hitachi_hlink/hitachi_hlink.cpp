#include "hitachi.h"
#include "esphome/core/log.h"
#include "esphome.h"

namespace esphome {
namespace hitachi_hlink {

static const char *const TAG = "hitachi";

HitachiClimate::HitachiClimate() {}

void HitachiClimate::setup() {
  buf_[0] = 0;
  buf_index_ = 0;

  // initialize write only parameters, other writtable parameters will be initialize at the first read when ready_
  // become true
  state_[ABSENCE_SET].u16 = 0x0000;
  state_[BEEP].u8 = 0x07;

  model_[0] = 0;
  state_[MODEL].text = model_;
}

climate::ClimateTraits HitachiClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL,
                              climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_DRY});
  traits.set_supports_action(true);
  traits.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM,
                                  climate::CLIMATE_FAN_HIGH, climate::CLIMATE_FAN_DIFFUSE});
  return traits;
}

void HitachiClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "hitachi_hlink:");
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_ODD, 8);
}

void HitachiClimate::update() {
  print_state();
  pending_read_ = true;
}

void HitachiClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    this->mode = *call.get_mode();
    ESP_LOGD(TAG, "mode: %s", LOG_STR_ARG(climate_mode_to_string(this->mode)));
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    ESP_LOGD(TAG, "Temperature: %f", target_temperature);
  }
  if (call.get_fan_mode().has_value()) {
    this->fan_mode = *call.get_fan_mode();
    ESP_LOGD(TAG, "Fan mode: %s", LOG_STR_ARG(climate_fan_mode_to_string(this->fan_mode.value())));
  }
  if (climate2state_() && ready_) {
    pending_set_ = true;
  }
  this->mode = mode;
  this->publish_state();
}

// Frequent processing in ESP main loop called at about 60Hz.
// Either send the command, receive data of process incoming data to avoid blocking this function
void HitachiClimate::loop() {
  // At each timestep either send read comm
  switch (receiver_) {
    case OFF: {
      if (pending_set_) {
        current_ = 0;
        receiver_ = SENDING_SET;
      } else if (pending_read_) {
        current_ = 0;
        receiver_ = SENDING_READ;
      }

      break;
    }

    case SENDING_READ:  // send read
    {
      this->send_read_cmd();
      receiver_ = RECEIVING_READ;
      start_receive_ = millis();
      buf_index_ = 0;
      break;
    }

    case SENDING_SET:  // send read
    {
      this->send_set_cmd();
      receiver_ = RECEIVING_SET;
      start_receive_ = millis();
      buf_index_ = 0;
      break;
    }

    case RECEIVING_READ:  // write incoming data in buf_
    {
      // detect timeout
      if (start_receive_ + TIMEOUT < millis()) {
        ESP_LOGW(TAG, "Receive timeout");
        receiver_ = FAILED_READ;
        break;
      }

      while (available() > 0) {
        uint8_t received = read();
        buf_[buf_index_] = received;
        buf_index_++;

        // detect end of message
        if (received == SEPARATOR) {
          buf_[buf_index_ - 1] = 0;  // replace last carriage return by end of string
          receiver_ = READ_COMPLETE;
          break;
        }

        // detect buffer full
        if (buf_index_ >= MAX_LENGTH - 1) {
          ESP_LOGW(TAG, "Internal buffer full");
          receiver_ = FAILED_READ;
          break;
        }
      }
      break;
    }

    case RECEIVING_SET:  // write incoming data in buf_
    {
      // detect timeout
      if (start_receive_ + TIMEOUT < millis()) {
        ESP_LOGW(TAG, "Receive timeout");
        receiver_ = FAILED_SET;
        break;
      }

      while (available() > 0) {
        uint8_t received = read();
        buf_[buf_index_] = received;
        buf_index_++;

        // detect end of message
        if (received == SEPARATOR) {
          buf_[buf_index_ - 1] = 0;  // replace last carriage return by end of string
          receiver_ = SET_COMPLETE;
          break;
        }

        // detect buffer full
        if (buf_index_ >= MAX_LENGTH - 1) {
          ESP_LOGW(TAG, "Internal buffer full");
          receiver_ = FAILED_SET;
          break;
        }
      }
      break;
    }

    case FAILED_READ:  // resend the command in case of error. If more than 3 errors the reading is interupted
    {
      retry_++;
      if (retry_ >= 3) {
        receiver_ = OFF;
        ESP_LOGW(TAG, "Read P=%04X stopped after 3 retry", descr[current_].addr);
      } else {
        if (pending_set_) {  // stop the READ sequence in case a SET is pending
          receiver_ = OFF;
          retry_ = 0;
        } else {
          receiver_ = SENDING_READ;
        }
      }
      break;
    }

    case FAILED_SET:  // resend the command in case of error. If more than 3 errors the reading is interupted
    {
      retry_++;
      if (retry_ >= 3) {
        receiver_ = OFF;
        ESP_LOGW(TAG, "Set P=%04X stopped after 3 retry", descr[current_].addr);
      } else {
        receiver_ = SENDING_SET;
      }
      break;
    }

    case READ_COMPLETE:  // process stored data
    {
      ESP_LOGV(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);

      // The expected answer is in the form "OK P=[data] C=[checksum]"

      if (buf_[0] != 'O' || buf_[1] != 'K') {
        receiver_ = FAILED_READ;
        ESP_LOGW(TAG, "Incorrect answer format");
        ESP_LOGW(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);
        break;
      }

      uint8_t value[MAX_LENGTH] = {0};
      uint8_t chksum_bytes[2] = {0};
      if (!hex2byte(buf_ + 5, descr[current_].length, value)) {
        ESP_LOGW(TAG, "Incorrect value format");
        ESP_LOGW(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);
        receiver_ = FAILED_READ;
        break;
      }

      if (!hex2byte(buf_ + 5 + descr[current_].length + 3, 4, chksum_bytes)) {
        ESP_LOGW(TAG, "Incorrect checksum format");
        ESP_LOGW(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);
        receiver_ = FAILED_READ;
        break;
      }
      uint16_t chksum = (chksum_bytes[0] << 8) + chksum_bytes[1];
      if (chksum != checksum_(value, descr[current_].length / 2)) {
        ESP_LOGW(TAG, "Incorrect checksum, expected: %04X received: %04X", checksum_(value, descr[current_].length / 2),
                 chksum);
        ESP_LOGW(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);
        receiver_ = FAILED_READ;
        break;
      }

      retry_ = 0;

      // Store data depending on the lenght
      switch (descr[current_].length) {
        case 2:
          state_[current_].u8 = value[0];
          break;
        case 4:
          state_[current_].u16 = (value[0] << 8) + value[1];
          break;
        case 8:
          state_[current_].u32 = (value[0] << 24) + (value[1] << 16) + (value[2] << 8) + value[3];
          break;
        case 20:
          memcpy(model_, value, 10);
          break;
      }

      // interrupt the read if a set is pending
      if (pending_set_) {
        receiver_ = OFF;
        break;
      }

      // Go to the next parameter
      current_++;
      while (current_ < N_PARAMS) {
        if (descr[current_].readable) {
          receiver_ = SENDING_READ;
          break;
        }
        current_++;
      }

      // in case it was the last read, send the result
      if (current_ == N_PARAMS) {
        receiver_ = OFF;
        ready_ = true;
        pending_read_ = false;
        if (state2climate_()) {
          publish_state();
          outside_temp_sensor->publish_state(outside_temperature);
        }
      }

      break;
    }

    case SET_COMPLETE:  // process stored data
    {
      ESP_LOGV(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);

      // The expected answer is in the form "OK"

      if (buf_[0] != 'O' || buf_[1] != 'K') {
        receiver_ = FAILED_SET;
        ESP_LOGW(TAG, "Incorrect answer format");
        ESP_LOGW(TAG, "Frame received, buf_index_=%d buf_=%.64s", buf_index_, buf_);
        break;
      }

      retry_ = 0;

      // interrupt the read if a set is pending
      // Go to the next parameter
      current_++;
      while (current_ < N_PARAMS) {
        if (descr[current_].writable) {
          receiver_ = SENDING_SET;
          break;
        }
        current_++;
      }

      // in case it was the last set, send the result
      if (current_ == N_PARAMS) {
        receiver_ = OFF;
        ready_ = true;
        pending_set_ = false;
        pending_read_ = true;  // request immediate update of the state
      }
      break;
    }
  }
}

/** convert an  hexadecimal char array to array
 *
 * Convert an array of len hexadecimal char to an array of len/2 bytes, return false in case of incorrect formating
 */
bool HitachiClimate::hex2byte(char *buf, uint16_t len, uint8_t *out) {
  for (int i = 0; i < len; i++) {
    if (buf[i] >= '0' && buf[i] <= '9') {
      out[i / 2] += (buf[i] - '0') << (4 - 4 * (i % 2));
    } else if (buf[i] >= 'A' && buf[i] <= 'F') {
      out[i / 2] += (buf[i] - 'A' + 10) << (4 - 4 * (i % 2));
    } else {
      return false;
    }
  }
  return true;
}

// Send the command to read a paramater
void HitachiClimate::send_read_cmd() {
  uint16_t address = descr[current_].addr;
  char msg[MAX_LENGTH];
  uint8_t address_bytes[2];
  address_bytes[0] = (address >> 8) & 0xFF;
  address_bytes[1] = address & 0xFF;
  uint16_t chksum = checksum_(address_bytes, 2);
  snprintf(msg, MAX_LENGTH, "MT P=%04X C=%04X\r", address, chksum);
  this->write_str(msg);
  ESP_LOGV(TAG, "Sent MT P=%04X C=%04X\\r", address, chksum);
}

// Send the command to set a paramater
void HitachiClimate::send_set_cmd() {
  uint16_t address = descr[current_].addr;
  char msg[MAX_LENGTH], value_str[21];
  uint8_t data_bytes[MAX_LENGTH / 2];  // append address and value bytes to compute the checksum
  uint8_t data_bytes_len = 0;
  data_bytes[0] = (address >> 8) & 0xFF;
  data_bytes[1] = address & 0xFF;
  switch (descr[current_].length) {
    case 2:
      snprintf(value_str, MAX_LENGTH, "%02X", state_[current_].u8);
      data_bytes[2] = state_[current_].u8;
      data_bytes_len = 3;
      break;
    case 4:
      snprintf(value_str, MAX_LENGTH, "%04X", state_[current_].u16);
      data_bytes[2] = state_[current_].u16 >> 8 & 0x00FF;
      data_bytes[3] = state_[current_].u16 & 0x00FF;
      data_bytes_len = 4;
      break;
    case 8:
      snprintf(value_str, MAX_LENGTH, "%08X", state_[current_].u32);
      data_bytes[2] = state_[current_].u16 >> 24 & 0x00FF;
      data_bytes[3] = state_[current_].u16 >> 16 & 0x00FF;
      data_bytes[4] = state_[current_].u16 >> 8 & 0x00FF;
      data_bytes[5] = state_[current_].u16 & 0x00FF;
      data_bytes_len = 6;
      break;
    case 20:
      snprintf(value_str, MAX_LENGTH, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", state_[current_].text[0],
               state_[current_].text[1], state_[current_].text[2], state_[current_].text[3], state_[current_].text[4],
               state_[current_].text[5], state_[current_].text[6], state_[current_].text[7], state_[current_].text[8],
               state_[current_].text[9]);
      memcpy(data_bytes + 2, state_[current_].text, 10);
      data_bytes_len = 12;
      break;
  }
  uint16_t chksum = checksum_(data_bytes, data_bytes_len);
  snprintf(msg, MAX_LENGTH, "ST P=%04X,%s C=%04X\r", address, value_str, chksum);
  this->write_str(msg);
  ESP_LOGV(TAG, "Sent ST P=%04X,%s C=%04X", address, value_str, chksum);
}

/// compute the checksum of a byte array
uint16_t HitachiClimate::checksum_(uint8_t *in, uint8_t len) {
  int sum = 0xFFFF;
  for (int i = 0; i < len; i++) {
    sum -= in[i];
  }
  return sum;
}

/// print the state_
void HitachiClimate::print_state() {
  if (!ready_) {
    ESP_LOGD(TAG, "Not yet ready");
    return;
  }
  ESP_LOGD(TAG, "State:");
  for (int i = 0; i < N_PARAMS; i++) {
    int len = descr[i].length;
    switch (len) {
      case 2:
        ESP_LOGD(TAG, "%04X : %02X", descr[i].addr, state_[i].u8);
        break;
      case 4:
        ESP_LOGD(TAG, "%04X : %04X", descr[i].addr, state_[i].u16);
        break;
      case 8:
        ESP_LOGD(TAG, "%04X : %08X", descr[i].addr, state_[i].u32);
        break;
      case 20:
        ESP_LOGD(TAG, "%04X : %s", descr[i].addr, state_[i].text);
        break;
    }
  }
}

/// convert state_ to Climate members, return false in case of invalid value
bool HitachiClimate::climate2state_() {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      state_[POWER].u8 = 0x00;
      break;
    case climate::CLIMATE_MODE_HEAT:
      state_[MODE].u16 = 0x0010;
      state_[POWER].u8 = 0x01;
      break;
    case climate::CLIMATE_MODE_DRY:
      state_[MODE].u16 = 0x0020;
      state_[POWER].u8 = 0x01;
      break;
    case climate::CLIMATE_MODE_COOL:
      state_[MODE].u16 = 0x0040;
      state_[POWER].u8 = 0x01;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      state_[MODE].u16 = 0x0050;
      state_[POWER].u8 = 0x01;
      break;
    case climate::CLIMATE_MODE_HEAT_COOL:
      state_[MODE].u16 = 0x8000;
      state_[POWER].u8 = 0x01;
      break;
    default:
      ESP_LOGW(TAG, "Incompatible mode: %d", mode);
      return false;
  }

  if (fan_mode.has_value()) {
    switch (fan_mode.value()) {
      case climate::CLIMATE_FAN_AUTO:
        state_[SPEED].u8 = 0;
        break;
      case climate::CLIMATE_FAN_HIGH:
        state_[SPEED].u8 = 1;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        state_[SPEED].u8 = 2;
        break;
      case climate::CLIMATE_FAN_LOW:
        state_[SPEED].u8 = 3;
        break;
      case climate::CLIMATE_FAN_DIFFUSE:
        state_[SPEED].u8 = 4;
        break;
      default:
        ESP_LOGW(TAG, "Incompatible fan mode: %d", fan_mode.value());
        return false;
    }
  } else {
    ESP_LOGW(TAG, "No value set for fan mode, using AUTO by default");
    state_[SPEED].u8 = 0;
  }

  if (target_temperature <= 32. && target_temperature >= 16.) {
    state_[TARGET].u16 = (uint16_t) target_temperature;
  } else {
    ESP_LOGW(TAG, "Incorrect temperature: %f", target_temperature);
    return false;
  }
  return true;
}

/// convert Climate members to state_, return false in case of invalid value
bool HitachiClimate::state2climate_() {
  // mode
  switch (state_[POWER].u8) {
    case 0x00:
      mode = climate::CLIMATE_MODE_OFF;
      break;
    case 0x01:
      switch (state_[MODE].u16) {
        case 0x0010:
          mode = climate::CLIMATE_MODE_HEAT;
          break;
        case 0x0020:
          mode = climate::CLIMATE_MODE_DRY;
          break;
        case 0x0040:
          mode = climate::CLIMATE_MODE_COOL;
          break;
        case 0x0050:
          mode = climate::CLIMATE_MODE_FAN_ONLY;
          break;
        case 0x8000:
          mode = climate::CLIMATE_MODE_HEAT_COOL;
          break;
        default:
          ESP_LOGW(TAG, "Unknown mode value %04X", state_[MODE].u16);
          return false;
      }
      break;
    default:
      ESP_LOGW(TAG, "Unknown power value %02X", state_[POWER].u8);
      return false;
  }

  // action
  if (state_[POWER].u8 == 0x00) {
    action = climate::CLIMATE_ACTION_OFF;
  } else if (state_[MODE].u16 == 0x0050) {
    action = climate::CLIMATE_ACTION_FAN;
  } else if (state_[ACTIVE].u16 == 0xFFFF) {
    switch (state_[MODE].u16) {
      case 0x0010:
        action = climate::CLIMATE_ACTION_HEATING;
        break;
      case 0x0020:
        action = climate::CLIMATE_ACTION_DRYING;
        break;
      case 0x0040:
        action = climate::CLIMATE_ACTION_COOLING;
        break;
      case 0x8000:
        // in CLIMATE_MODE_HEAT_COOL the actual action is unknown, uses the temperatures to retreive it
        if (state_[TARGET].u16 > state_[INDOOR].u8) {
          action = climate::CLIMATE_ACTION_HEATING;
        } else if (state_[TARGET].u16 < state_[INDOOR].u8 || state_[INDOOR].u8 <= state_[OUTDOOR].u8) {
          action = climate::CLIMATE_ACTION_COOLING;
        } else {
          action = climate::CLIMATE_ACTION_HEATING;
        }
        break;
      default:
        ESP_LOGW(TAG, "Unknown mode value %04X", state_[MODE].u16);
        return false;
    }
  } else {
    action = climate::CLIMATE_ACTION_IDLE;
  }

  // indoor temperature
  if (state_[INDOOR].u8 < 50) {
    current_temperature = state_[INDOOR].u8;
  } else {
    ESP_LOGW(TAG, "Invalid indoor temperature %d (0x%02X)", state_[INDOOR].u8, state_[INDOOR].u8);
    return false;
  }

  // outdoor temperature
  if (state_[OUTDOOR].u8 < 50) {
    outside_temperature = state_[OUTDOOR].u8;
  } else {
    ESP_LOGW(TAG, "Invalid outdoor temperature %d (0x%02X)", state_[OUTDOOR].u8, state_[OUTDOOR].u8);
    return false;
  }

  // target temperature
  if (state_[TARGET].u16 < 50) {
    target_temperature = state_[TARGET].u16;
  } else {
    // invalid target temperature are just ignored since the value is invalid when the AC is OFF
    ESP_LOGV(TAG, "Ignored invalid target temperature %d (0x%02X)", state_[TARGET].u16, state_[TARGET].u16);
  }

  // fan speed
  switch (state_[SPEED].u8) {
    case 0:
      fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case 1:
      fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    case 2:
      fan_mode = climate::CLIMATE_FAN_MEDIUM;
      break;
    case 3:
      fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case 4:
      fan_mode = climate::CLIMATE_FAN_DIFFUSE;
      break;
    default:
      ESP_LOGW(TAG, "Unknown fan speed value %02X", state_[SPEED].u8);
      return false;
  }
  return true;
}

}  // namespace hitachi_hlink
}  // namespace esphome
