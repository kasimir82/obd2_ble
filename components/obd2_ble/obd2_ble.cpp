#include "obd2_ble.h"
#include "esphome/core/log.h"
#include "esp_task_wdt.h"
#include <regex>

#ifdef USE_ESP32

namespace esphome {
namespace obd2_ble {

static const char *TAG = "obd2_ble";

void OBD2BLEClient::setup() {
  ESP_LOGD(TAG, "Setting up OBD2 BLE Client...");
  string_to_uuid(this->service_uuid_str_, &this->service_uuid_);
  string_to_uuid(this->read_char_uuid_str_, &this->read_char_uuid_);
  string_to_uuid(this->write_char_uuid_str_, &this->write_char_uuid_);
  string_to_uuid(this->notify_char_uuid_str_, &this->notify_char_uuid_);
  
  for (auto it = init_commands_.rbegin(); it != init_commands_.rend(); ++it) {
    OBD2Task task;
    task.command = *it;
    task.can_id = "";
    task.mode = "";
    task.pid = "";
    task.ignore_error = false;
    task_queue_.insert(task_queue_.begin(), std::move(task));
  }
  
  dump_config();
}

void OBD2BLEClient::loop() {
  if (!mtu_configured_ || !notifications_ready_ || !descr_write_done_) return;
  
  if (current_task_index_ < task_queue_.size()) {
    OBD2Task &task = task_queue_[current_task_index_];
    process_task(task, command_delay_, command_wait_, response_wait_, publish_delay_, disconnect_delay_);
    if (task.status == DONE || (task.status == ERROR && task.ignore_error)) {
      current_task_index_++;
    }
  } else {
    // ESP_LOGD(TAG, "Task completed. Disconnecting from OBD2 adapter...");
    this->disconnect();
  }
}

void OBD2BLEClient::dump_config() {
  ESP_LOGCONFIG(TAG, "OBD2 BLE Client:");
  ESP_LOGCONFIG(TAG, " Service UUID: %s", service_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, " Write Characteristic UUID: %s", write_char_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, " Notify Characteristic UUID: %s", notify_char_uuid_str_.c_str());
  std::string init_commands_str;
  for (const auto &cmd : init_commands_) {
    init_commands_str += cmd + " ";
  }
  if (!init_commands_str.empty()) init_commands_str.pop_back();
  ESP_LOGCONFIG(TAG, " OBD2 Initialization Commands: %s", init_commands_str.c_str());
  ESP_LOGCONFIG(TAG, " OBD2 Command Delay: %d ms", command_delay_);
  ESP_LOGCONFIG(TAG, " OBD2 Command Wait: %d ms", command_wait_);
  ESP_LOGCONFIG(TAG, " Task queue size: %d", task_queue_.size());
  for (auto &task : task_queue_) {
    std::string line = "  command: " + task.command;
    if (task.can_id != "") {
      line += ", CAN ID: " + task.can_id;
    }
    if (task.mode != "") {
      line += ", Mode: " + task.mode;
    }
    if (task.pid != "") {
      line += ", PID: " + task.pid;
    }
    if (task.sensor != nullptr) {
      line += ", Name: " + task.sensor->get_name();
    } else if (task.text_sensor != nullptr) {
      line += ", Name: " + task.text_sensor->get_name();
    }
    ESP_LOGCONFIG(TAG, "%s", line.c_str());
    for (int i = 0; i < task.codes.size(); ++i) {
      ESP_LOGCONFIG(TAG, "   code: %s, name: %s", task.codes[i].c_str(), task.binary_sensors[i]->get_name().c_str());
    }
  }
}

void OBD2BLEClient::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  ESP_LOGV(TAG, "GATT Event: %d", event);
  switch (event) {
    case ESP_GATTC_REG_EVT:
      ESP_LOGD(TAG, "GATT client is registered.");
      break;
    case ESP_GATTC_CONNECT_EVT:
      ESP_LOGD(TAG, "BLE physical connection is set up.");
      gattc_if_ = gattc_if;
      conn_id_ = param->connect.conn_id;
      ESP_LOGI(TAG, "Connected to device, gattc_if: %d, conn_id: %d", gattc_if_, conn_id_);
      break;
    case ESP_GATTC_OPEN_EVT:
      ESP_LOGD(TAG, "GATT virtual connection is set up.");
      break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
      ESP_LOGD(TAG, "BLE discover service completed.");
      break;
    case ESP_GATTC_SEARCH_RES_EVT:
      ESP_LOGD(TAG, "GATT service discovery result is got.");
      break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGD(TAG, "GATT Search Complete Event");
      auto *write_chr = this->parent_->get_characteristic(this->service_uuid_, this->write_char_uuid_);
      if (write_chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No write characteristic found at device.", this->parent_->address_str().c_str());
        break;
      }
      this->write_char = write_chr;

      auto *notify_chr = this->parent_->get_characteristic(this->service_uuid_, this->notify_char_uuid_);
      if (notify_chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No notify characteristic found at device.", this->parent_->address_str().c_str());
        break;
      }
      this->notify_char = notify_chr;

      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), notify_chr->handle);
      if (status) {
        ESP_LOGD(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }
      break;
    }
    case ESP_GATTC_CFG_MTU_EVT:
      ESP_LOGD(TAG, "Configuration of MTU completes, MTU size: %d.", param->cfg_mtu.mtu);
      mtu_configured_ = true;
      break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      ESP_LOGD(TAG, "Register for notification of a service completes.");
      notifications_ready_ = true;
      break;
    case ESP_GATTC_WRITE_DESCR_EVT:
      ESP_LOGD(TAG, "GATT characteristic descriptor write completes.");
      descr_write_done_ = true;
      break;
    case ESP_GATTC_WRITE_CHAR_EVT:
      on_write();
      break;
    case ESP_GATTC_NOTIFY_EVT:
      this->on_notify(std::vector<uint8_t>(param->notify.value, param->notify.value + param->notify.value_len));
      break;
    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGW(TAG, "Disconnected from OBD2 adapter, reason=%d", param->disconnect.reason);
      on_disconnect();
      break;
    case ESP_GATTC_CLOSE_EVT:
      ESP_LOGD(TAG, "GATT virtual connection is closed.");
      break;
    default:
      ESP_LOGD(TAG, "Unhandled GATT Event: %d", event);
      break;
  }
}

void OBD2BLEClient::add_task_for_sensor(sensor::Sensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid) {
  OBD2Task task;
  task.command = mode + pid;
  task.can_id = can_id;
  task.mode = mode,
  task.pid = pid;
  task.sensor = sensor;
  task_queue_.emplace_back(std::move(task));
}

void OBD2BLEClient::add_task_for_text_sensor(text_sensor::TextSensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid) {
  OBD2Task task;
  if (pid == "") {
    task.command = mode;
  } else {
    task.command = mode + pid;
  }
  task.can_id = can_id;
  task.mode = mode;
  task.pid = pid;
  task.text_sensor = sensor;
  task_queue_.emplace_back(std::move(task));
}

void OBD2BLEClient::add_task_for_binary_sensor(binary_sensor::BinarySensor *sensor, const std::string &mode, const std::string &code) {
  auto it = std::find_if(task_queue_.begin(), task_queue_.end(), [&mode](const OBD2Task& t) { return t.mode == mode; });
  if (it != task_queue_.end()) {
    it->codes.push_back(code);
    it->binary_sensors.push_back(sensor);
  } else {
    OBD2Task task;
    task.command = mode;
    task.mode = mode;
    task.codes.push_back(code);
    task.binary_sensors.push_back(sensor);
    task_queue_.emplace_back(std::move(task));
  }
}

void OBD2BLEClient::string_to_uuid(const std::string &uuid_str, espbt::ESPBTUUID *uuid) {
  ESP_LOGD(TAG, "string_to_uuid: %s", uuid_str.c_str());
  if (uuid_str.length() == 4 || (uuid_str.length() == 6 && uuid_str.substr(0, 2) == "0x")) {
    // 16-bit UUID
    uint16_t uuid16 = std::stoi(uuid_str.substr(uuid_str.length() - 4), nullptr, 16);
    *uuid = espbt::ESPBTUUID::from_uint16(uuid16);
    ESP_LOGD(TAG, "Converted 16-bit UUID: %04X", uuid16);
  } else if (uuid_str.length() == 36) {
    // 128-bit UUID
    *uuid = espbt::ESPBTUUID::from_raw(uuid_str);
    ESP_LOGD(TAG, "Converted 128-bit UUID: %s", uuid_str.c_str());
  } else {
    ESP_LOGE(TAG, "Invalid UUID string length: %s", uuid_str.c_str());
  }
}

void OBD2BLEClient::process_task(OBD2Task &task, const int &command_delay, const int &command_wait, const int &response_wait, const int &publish_delay, const int &disconnect_delay) {
  unsigned long current_time = millis();
  if (current_time - last_command_time_ >= command_delay) {
    switch (task.status) {
      case PENDING:
        ESP_LOGD(TAG, "Sending command: %s", task.command.c_str());
        request_write(task.command);
        //request_read();
        last_command_ = task.command;
        last_command_time_ = current_time;
        task.status = SENDING;
        break;
      case SENDING:
        if (current_time - last_command_time_ >= command_wait) {
          ESP_LOGW(TAG, "Confirmation timeout for command: %s", task.command.c_str());
          task.status = ERROR;
        }
        break;
      case SENT:
        if (current_time - last_command_time_ >= response_wait) {
          ESP_LOGW(TAG, "Response timeout for command: %s", task.command.c_str());
          task.status = ERROR;
        }
        break;
      case RECEIVED:
        ESP_LOGD(TAG, "Processing response for command: %s", task.command.c_str());
        if (parse_data(task)) {
          task.status = PUBLISHING;
        } else {
          task.status = ERROR;
        }
        break;
      case PUBLISHING:
        if (current_time - last_publish_time_ >= publish_delay) {
          ESP_LOGD(TAG, "Publishing response for command: %s", task.command.c_str());
          publish_data(task);
          if (task.published) {
            task.status = DONE;
          } else {
            last_publish_time_ = current_time;
          }
        }
        break;
      case DONE:
        ESP_LOGI(TAG, "DONE task for command: %s", task.command.c_str());
        break;
      case ERROR:
        //ESP_LOGE(TAG, "Task error for command: %s", task.command.c_str());
        if (!task.ignore_error) {
          if (current_time - last_command_time_ >= disconnect_delay) {
            ESP_LOGW(TAG, "Task cancelled. Disconnecting from OBD2 adapter...");
            this->disconnect();
          }
        }
        break;
      default:
        break;
    }
    esp_task_wdt_reset();
  }
}

void OBD2BLEClient::request_write(const std::string &command) {
  if (this->write_char == nullptr) {
    ESP_LOGE(TAG, "Write characteristic is null, cannot send command");
    return;
  }
  std::string command_with_cr = command + "\r";
  //ESP_LOGD(TAG, "Sending AT command: %s", command_with_cr.c_str());
  auto status = esp_ble_gattc_write_char(
      this->parent_->get_gattc_if(),
      this->parent_->get_conn_id(),
      this->write_char->handle,
      command_with_cr.size(),
      (uint8_t *)command_with_cr.c_str(),
      ESP_GATT_WRITE_TYPE_RSP,
      ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", this->parent_->address_str().c_str(), status);
  }
}

void OBD2BLEClient::request_read() {
  if (this->read_char == nullptr) {
    ESP_LOGE(TAG, "Read characteristic is null, cannot request read");
    return;
  }
  ESP_LOGD(TAG, "Request read data for %s", last_command_.c_str());
  auto status = esp_ble_gattc_read_char(
      this->parent()->get_gattc_if(),
      this->parent()->get_conn_id(),
      this->read_char->handle,
      ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_read_char failed, status=%d", this->parent_->address_str().c_str(), status);
  }
}

void OBD2BLEClient::disconnect() {
  if (gattc_if_ != 0 && conn_id_ != std::numeric_limits<uint16_t>::max()) {
    ESP_LOGI(TAG, "Disconnecting from device, gattc_if: %d, conn_id: %d", gattc_if_, conn_id_);
    esp_ble_gattc_close(gattc_if_, conn_id_);
    cleanup();
  }
}

void OBD2BLEClient::on_write() {
  if (current_task_index_ < task_queue_.size()) {
    OBD2Task &task = task_queue_[current_task_index_];
    if (task.status == PENDING || task.status == SENDING) {
      task.status = SENT;
      //ESP_LOGD(TAG, "Sent command: %s", task.command.c_str());
    }
    ESP_LOGD(TAG, "GATT characteristic write operation completes: %s.", task.command.c_str());
  } else {
    ESP_LOGW(TAG, "GATT write event received, but no current task available!");
  }
}

void OBD2BLEClient::on_notify(const std::vector<uint8_t> &data) {
  //ESP_LOGD(TAG, "Received data: %s", format_hex_pretty(data).c_str());
  
  if (current_task_index_ < task_queue_.size()) {
    response_buffer_.insert(response_buffer_.end(), data.begin(), data.end());
    parse_response();
  } else {
    ESP_LOGW(TAG, "GATT notify event received, but no current task available!");
  }
}

void OBD2BLEClient::on_disconnect() {
  gattc_if_ = 0;
  conn_id_ = std::numeric_limits<uint16_t>::max();
  ESP_LOGW(TAG, "Disconnected from OBD2 adapter. gattc_if: %d, conn_id: %d", gattc_if_, conn_id_);
}

void OBD2BLEClient::parse_response() {
  if (!response_buffer_.empty() && response_buffer_.back() == 0x3E) {
    ESP_LOGD(TAG, "Received data in hex: %s", format_hex_pretty(response_buffer_).c_str());
    std::string response(response_buffer_.begin(), response_buffer_.end());
    std::replace(response.begin(), response.end(), '\r', ' ');
    std::replace(response.begin(), response.end(), '\0', ' ');
    ESP_LOGD(TAG, "Received data in str: %s", response.c_str());
    
    std::vector<std::vector<uint8_t>> lines;
    std::vector<uint8_t> line;
    for (auto byte : response_buffer_) {
      if (byte == 0x0D) {
        if (!line.empty()) {
          //ESP_LOGD(TAG, "Line: %s", format_hex_pretty(line).c_str());
          lines.push_back(line);
        }
        line.clear();
      } else {
        line.push_back(byte);
      }
    }
    OBD2Task &task = task_queue_[current_task_index_];
    task.data = std::move(lines);
    task.status = RECEIVED;
    response_buffer_.clear();
  }
}

bool OBD2BLEClient::parse_data(OBD2Task &task) {
  for (const auto &data : task.data) {
    std::string response_str(data.begin(), data.end());
    if (!is_ready(response_str, task.command)) {
      ESP_LOGE(TAG, "Task error for command: %s", task.command.c_str());
      return false;
    }
    if (is_message(task, response_str)) continue;
    
    std::string can_id_str;
    std::uint8_t pci;
    std::uint8_t length;
    std::string response_data_str;
    tie(can_id_str, pci, length, response_data_str) = split_data(response_str);
    if (response_data_str.empty()) response_data_str = response_str;
    ESP_LOGD(TAG, "CAN ID: %s, PCI: %s, Length: %s, Response data: %s", can_id_str.c_str(), format_hex_pretty(pci).c_str(), format_hex_pretty(length).c_str(), response_data_str.c_str());
    
    auto response_data = decode_data(response_data_str);
    if (response_data.empty()) continue;
    
    switch (frame_type(pci)) {
      case FrameType::SINGLE:
        ESP_LOGD(TAG, "Single frame");
        parse_payload(task, can_id_str, length, response_data);
        break;
      case FrameType::FIRST:
        handle_first_frame(task, can_id_str, length, response_data);
        break;
      case FrameType::CONSECUTIVE:
        handle_consecutive_frame(task, can_id_str, response_data);
        break;
      default:
        ESP_LOGW(TAG, "Unknown frame type: %s", format_hex_pretty(pci).c_str());
        break;
    }
  }
  return true;
}

bool OBD2BLEClient::is_ready(const std::string &response_str, const std::string &command) {
  if (response_str.find("UNABLE TO CONNECT") != std::string::npos) {
    ESP_LOGW(TAG, "ELM not available for command: %s", command.c_str());
    return false;
  } else {
    return true;
  }
}

bool OBD2BLEClient::is_message(OBD2Task &task, const std::string &response_str) {
  if (response_str.find("ELM327") != std::string::npos) {
    ESP_LOGD(TAG, "ELM327 available for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("SEARCHING") != std::string::npos) {
    ESP_LOGD(TAG, "Searching for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("OK") != std::string::npos) {
    ESP_LOGD(TAG, "Response OK for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("NO DATA") != std::string::npos) {
    ESP_LOGD(TAG, "No data for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("BUFFER FULL") != std::string::npos) {
    ESP_LOGD(TAG, "No enough buffer for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find(task.command) != std::string::npos) {
    ESP_LOGD(TAG, "Echo received for command: %s", task.command.c_str());
    return true;
  } else if (response_str == "A0") {
    ESP_LOGD(TAG, "Response A0 for command: %s", task.command.c_str());
    return true;
  } else if (response_str == "7") {
    ESP_LOGD(TAG, "Response 7 for command: %s", task.command.c_str());
    return true;
  } else if (response_str == "6") {
    ESP_LOGD(TAG, "Response 6 for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find(">") != std::string::npos) {
    ESP_LOGD(TAG, "Prompt '>' received for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("?") != std::string::npos) {
    ESP_LOGD(TAG, "Error '?' received for command: %s", task.command.c_str());
    return true;
  } else {
    std::regex atrv_regex(R"((\d+\.\d*)V)");
    std::smatch atrv_match;
    if (std::regex_search(response_str, atrv_match, atrv_regex)) {
      if (task.command == "ATRV") {
        task.value_f = std::stof(atrv_match[1]);
      }
      return true;
    } else {
      return false;
    }
  }
}

std::tuple<std::string, std::uint8_t, std::uint8_t, std::string> OBD2BLEClient::split_data(std::string &data_str) {
  std::uint8_t can_id_length = 0;
  std::string can_id_str;
  std::string pci_str;
  std::uint8_t pci = 0;
  std::string length_str;
  std::uint8_t length = 0;
  std::string response_data_str;
  if (data_str.size() > 5 && data_str[0] == '7' && data_str[1] == 'E') {
    can_id_length = 3;
  } else if (data_str.size() > 10 && (data_str[0] == '1' && (data_str[1] == '8' || data_str[1] == '9'))) {
    can_id_length = 8;
  } else {
    ESP_LOGD(TAG, "No valid CAN ID detected: %s", data_str.c_str());
    return std::make_tuple("", 0, 0, "");
  }
  
  can_id_str = data_str.substr(0, can_id_length);
  if (data_str.size() < can_id_length + 2) {
    ESP_LOGD(TAG, "Data string too short for PCI: %s", data_str.c_str());
    return std::make_tuple(can_id_str, 0, 0, "");
  }
  
  pci_str = data_str.substr(can_id_length, 2);
  pci = static_cast<std::uint8_t>(std::stoi(pci_str, nullptr, 16));
  if (pci <= 0x07) {
    length = pci;
    response_data_str = data_str.substr(can_id_length + 2);
  } else if (pci >= 0x10 && pci <= 0x1F) {
    if (data_str.size() < can_id_length + 4) {
      ESP_LOGD(TAG, "Data string too short for First Frame: %s", data_str.c_str());
      return std::make_tuple(can_id_str, pci, 0, "");
    }
    length_str = data_str.substr(can_id_length + 2, 2);
    length = static_cast<std::uint8_t>(std::stoi(length_str, nullptr, 16));
    response_data_str = data_str.substr(can_id_length + 4);
  } else if (pci >= 0x21 && pci <= 0x3F) {
    response_data_str = data_str.substr(can_id_length + 2);
  }
    
  return std::make_tuple(can_id_str, pci, length, response_data_str);
}

std::vector<uint8_t> OBD2BLEClient::decode_data(const std::string &response_data_str) {
  if (response_data_str.size() % 2 != 0) {
    ESP_LOGW(TAG, "Data size is not even, skipping decode: %s", response_data_str.c_str());
    return {};
  }
  if (!std::all_of(response_data_str.begin(), response_data_str.end(), ::isxdigit)) {
    ESP_LOGW(TAG, "Data is not in ASCII hex format: %s", response_data_str.c_str());
    return {};
  }
  return ascii_hex_to_bytes(std::vector<uint8_t>(response_data_str.begin(), response_data_str.end()));
}

std::vector<uint8_t> OBD2BLEClient::ascii_hex_to_bytes(const std::vector<uint8_t> &ascii_data) {
  std::vector<uint8_t> raw_data;
  for (size_t i = 0; i < ascii_data.size(); i += 2) {
    if (i + 1 >= ascii_data.size() || !isxdigit(ascii_data[i]) || !isxdigit(ascii_data[i + 1])) {
      break;
    }
    uint8_t byte = (std::stoi(std::string(1, ascii_data[i]), nullptr, 16) << 4) |
                   std::stoi(std::string(1, ascii_data[i + 1]), nullptr, 16);
    raw_data.push_back(byte);
  }
  return raw_data;
}

OBD2BLEClient::FrameType OBD2BLEClient::frame_type(uint8_t pci) {
  if (pci <= 0x07) return FrameType::SINGLE;
  if (pci >= 0x10 && pci <= 0x1F) return FrameType::FIRST;
  if (pci >= 0x21 && pci <= 0x3F) return FrameType::CONSECUTIVE;
  return FrameType::UNKNOWN;
}

void OBD2BLEClient::handle_first_frame(OBD2Task &task, std::string &can_id_str, uint8_t length, const std::vector<uint8_t> &response_data) {
  if (task.can_id_map.count(can_id_str)) {
    ESP_LOGW(TAG, "Duplicated first frame for CAN ID: %s", can_id_str.c_str());
    return;
  }
  task.can_id_map[can_id_str] = {length, response_data.size(), response_data};
  ESP_LOGD(TAG, "Stored first frame for CAN ID: %s", can_id_str.c_str());
}

void OBD2BLEClient::handle_consecutive_frame(OBD2Task &task, std::string &can_id_str, const std::vector<uint8_t> &response_data) {
  auto it = task.can_id_map.find(can_id_str);
  if (it == task.can_id_map.end()) {
    ESP_LOGW(TAG, "Consecutive frame received for unknown CAN ID: %s", can_id_str.c_str());
    return;
  }

  auto &entry = it->second;
  entry.concat_data.insert(entry.concat_data.end(), response_data.begin(), response_data.end());
  entry.received_count += response_data.size();

  if (entry.received_count >= entry.total_length) {
    ESP_LOGD(TAG, "Completed data for CAN ID: %s", can_id_str.c_str());
    if (entry.concat_data.size() > entry.total_length) {
      entry.concat_data.resize(entry.total_length);
      ESP_LOGD(TAG, "Final data after trimming: %s", format_hex_pretty(entry.concat_data).c_str());
    }
    parse_payload(task, can_id_str, entry.total_length, entry.concat_data);
    task.can_id_map.erase(it);
  }
}

void OBD2BLEClient::parse_payload(OBD2Task &task, std::string &can_id_str, uint8_t &length, std::vector<uint8_t> &response_data) {
  if (length != response_data.size()) {
    ESP_LOGE(TAG, "Unexpected data size, Length: %s, Data size: %s", format_hex_pretty(length).c_str(), format_hex_pretty(response_data.size()).c_str());
    return;
  }
  // Parsing OBD-II Modes
  switch (response_data[0]) {
    case 0x41:  // Mode 01
      if (response_data[1] == 0x00 || response_data[1] == 0x20 || response_data[1] == 0x40 || response_data[1] == 0x60 || response_data[1] == 0x80 || response_data[1] == 0xA0 || response_data[1] == 0xC0) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, false);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        if (task.can_id == can_id_str) {
          handle_mode_01(task, response_data);
        }
      }
      break;
    case 0x45:  // Mode 05
      break;
    case 0x46:  // Mode 06
      if (response_data[1] == 0x00 || response_data[1] == 0x20 || response_data[1] == 0x40 || response_data[1] == 0x60 || response_data[1] == 0x80 || response_data[1] == 0xA0) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, false);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        if (task.can_id == can_id_str) {
          handle_mode_05(task, response_data);
        }
      }
      break;
    case 0x43:  // Mode 03
    case 0x47:  // Mode 07
    case 0x4A:  // Mode 0A
      if (task.can_id == can_id_str) {
        handle_dtc_response(task, response_data);
        ESP_LOGD(TAG, "DTC codes returned: %s", task.value_s.c_str());
      }
      break;
    case 0x49:  // Mode 09
      if (response_data[1] == 0x00) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, false);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        if (task.can_id == can_id_str) {
          handle_mode_09(task, response_data);
        }
      }
      break;
    case 0x62:  // Mode 22
      if (response_data[2] == 0x00 || response_data[2] == 0x20 || response_data[2] == 0x40 || response_data[2] == 0x60 || response_data[2] == 0x80 || response_data[2] == 0xA0 || response_data[2] == 0xC0 || response_data[2] == 0xE0) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, true);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        handle_mode_22(task, response_data);
      }
    default:
      break;
  }
}

std::tuple<std::string, size_t> OBD2BLEClient::list_supported_pids(const std::vector<uint8_t> &response_data, bool is_two_byte_pid) {
  std::string supported_pids_str;
  std::vector<uint8_t> supported_pids_8;
  std::vector<uint16_t> supported_pids_16;
  size_t pids_num = 0;
  
  if (response_data.size() < 6) {
    ESP_LOGW(TAG, "Invalid response size for supported PIDs.");
    return std::make_tuple(supported_pids_str, pids_num);
  }

  uint8_t mode = response_data[0];
  
  if (mode != 0x41 && mode != 0x46 && mode != 0x49 && mode != 0x62) {
    ESP_LOGW(TAG, "Invalid response for supported PIDs.");
    return std::make_tuple(supported_pids_str, pids_num);
  }
  
  uint16_t requested_pid = is_two_byte_pid ? (response_data[1] << 8 | response_data[2]) : response_data[1];
  
  int pid_start = requested_pid + 1;
  size_t data_offset = is_two_byte_pid ? 3 : 2;
  
  for (size_t i = data_offset; i < response_data.size(); i++) {
    uint8_t byte_mask = response_data[i];
    
    for (int bit = 0; bit < 8; bit++) {
      if (byte_mask & (1 << (7 - bit))) {
        if (is_two_byte_pid) {
          uint16_t pid = pid_start + (i - data_offset) * 8 + bit;
          supported_pids_16.push_back(pid);
        } else {
          uint8_t pid = pid_start + (i - data_offset) * 8 + bit;
          supported_pids_8.push_back(pid);
        }
      }
    }
  }
  
  std::string delimiter = " ";
  if (is_two_byte_pid) {
    char buffer[5];
    for (size_t i = 0; i < supported_pids_16.size(); ++i) {
      if (i != 0) {
        supported_pids_str += delimiter;
      }
      snprintf(buffer, sizeof(buffer), "%04X", supported_pids_16[i]);
      supported_pids_str += buffer;
    }
    pids_num = supported_pids_16.size();
  } else {
    char buffer[3];
    for (size_t i = 0; i < supported_pids_8.size(); ++i) {
      if (i != 0) {
        supported_pids_str += delimiter;
      }
      snprintf(buffer, sizeof(buffer), "%02X", supported_pids_8[i]);
      supported_pids_str += buffer;
    }
    pids_num = supported_pids_8.size();
  }
  
  return std::make_tuple(supported_pids_str, pids_num);
}

void OBD2BLEClient::handle_mode_01(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0;
  task.value_s = "";
  uint8_t pid = data[1];
  
  switch (pid) {
    case 0x01:
      task.value_s = get_monitor_status(data);
      break;
    case 0x03:
      task.value_s = get_fuel_system_status(data[2]);
      break;
    case 0x04:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x05:
      task.value_f = data[2] - 40;
      break;
    case 0x06:
      task.value_f = (100.0/128.0) * data[2] - 100.0;
      break;
    case 0x07:
      task.value_f = (100.0/128.0) * data[2] - 100.0;
      break;
    case 0x0B:
      task.value_f = data[2];
      break;
    case 0x0C:
      task.value_f = ((data[2] * 256.0) + data[3]) / 4.0;
      break;
    case  0x0D:
      task.value_f = data[2];
      break;
    case 0x0E:
      task.value_f = (data[2]/2.0) - 64.0;
      break;
    case 0x0F:
      task.value_f = data[2] - 40.0;
      break;
    case 0x10:
      task.value_f = ((data[2] * 256.0) + data[3]) / 100.0;
      break;
    case 0x11:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x13: {
      uint8_t sensor_present = data[2];
      if (sensor_present != 0x00) {
        task.value_s = "Oxygen Sensors: ";
        bool first = true; 
        for (int i = 0; i < 8; ++i) {
          if (sensor_present & (1 << i)) {
            if (!first) {
              task.value_s += ", ";
            }
            first = false;
            if (i < 4) {
              task.value_s += "B1S" + std::to_string(i + 1);
            } else {
              task.value_s += "B2S" + std::to_string(i - 3);
            }
          }
        }
      } else {
        task.value_s = "No Oxygen Sensors Detected";
      }
      }
      break;
    case 0x15:
      task.value_f = (100.0/128.0) * data[3] - 100.0;
      break;
    case 0x1C:
      task.value_s = get_standard_name(data[2]);
      break;
    case 0x1F:
      task.value_f = (data[2] * 256.0) + data[3];
      break;
    case 0x21:
      task.value_f = (data[2] * 256.0) + data[3];
      break;
    case 0x24:
      task.value_f = (2.0/65536.0) * ((256.0 * data[2]) + data[3]);
      break;
    case 0x2C:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x2D:
      task.value_f = (100.0/128.0) * data[2] - 100.0;
      break;
    case 0x2E:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x2F:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x30:
      task.value_f = data[2];
      break;
    case 0x31:
      task.value_f = (256.0 * data[2]) + data[3];
      break;
    case 0x33:
      task.value_f = data[2];
      break;
    case 0x34:
      task.value_f = (2.0/65536.0) * ((256.0 * data[2]) + data[3]);
      break;
    case 0x3C:
      task.value_f = (((256.0 * data[2]) + data[3]) / 10.0) - 40.0;
      break;
    case 0x41:
      break;
    case 0x42:
      task.value_f = ((256.0 * data[2]) + data[3]) / 1000.0;
      break;
    case 0x43:
      task.value_f = (100.0/255.0) * ((256.0 * data[2]) + data[3]);
      break;
    case 0x44:
      task.value_f = (2.0/65536.0) * ((256.0 * data[2]) + data[3]);
      break;
    case 0x45:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x46:
      task.value_f = data[2] - 40.0;
      break;
    case 0x47:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x49:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x4A:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x4C:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x4D:
      task.value_f = (256.0 * data[2]) + data[3];
      break;
    case 0x4E:
      task.value_f = (256.0 * data[2]) + data[3];
      break;
    case 0x51:
      task.value_s = get_fuel_type(data[2]);
      break;
    case 0x53:
      task.value_f = ((256.0 * data[2]) + data[3]) / 200.0;
      break;
    case 0x5A:
      task.value_f = (100.0/255.0) * data[2];
      break;
    case 0x5C:
      task.value_f = data[2] - 40.0;
      break;
    case 0x65:
      break;
    case 0x67: {
      uint8_t sensor_present = data[2];
      if (sensor_present != 0x00) {
        if (is_bit_set(sensor_present, 0)) {
          task.value_f = data[3] - 40.0;
        } else if (is_bit_set(sensor_present, 1)) {
          task.value_f = data[4] - 40.0;
        }
      }
      }
      break;
    case 0x6D:
      break;
    default:
      ESP_LOGD(TAG, "Unsupported PID: %02X", pid);
      break;
  }
  
  if (!std::isnan(task.value_f)) {
    ESP_LOGD(TAG, "Calculated value: %.2f", task.value_f);
  } else if (!task.value_s.empty()) {
    ESP_LOGD(TAG, "Calculated value: %s", task.value_s.c_str());
  }
}

void OBD2BLEClient::handle_mode_05(OBD2Task &task, const std::vector<uint8_t> &data) {
  std::string tid = format_hex(data[1]);
  task.value_f = data[2]; // Example
  // Handle Mode 05 specific logic here
}

void OBD2BLEClient::handle_mode_06(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0;
  task.value_s = "";
  uint8_t pid = data[1];
  
  switch (pid) {
    case 0x01:
      break;
    case 0x02:
      break;
    case 0x21:
      break;
    case 0x31:
      break;
    case 0x35:
      break;
    case 0x36:
      break;
    case 0x3C:
      break;
    case 0x3D:
      break;
    case 0x41:
      break;
    case 0x42:
      break;
    case 0xA1:
      break;
    case 0xA2:
      break;
    case 0xA3:
      break;
    case 0xA4:
      break;
    case 0xA5:
      break;
    default:
      ESP_LOGD(TAG, "Unsupported PID: %02X", pid);
      break;
  }
}

void OBD2BLEClient::handle_mode_09(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0;
  task.value_s = "";
  uint8_t pid = data[1];
  uint8_t indicator = data[2];
  switch (data[1]) {
    case 0x02:
      for (size_t i = 3; i < data.size(); ++i) {
        task.value_s += static_cast<char>(data[i]);
      }
      break;
    case 0x04: {
      std::string calibration_id(data.begin() + 3, data.end());
      task.value_s = calibration_id;
      break;
    }
    case 0x06: {
      for (size_t i = 3; i < data.size(); ++i) {
        task.value_s += format_hex_pretty(data[i]);
      }
      break;
    }
    case 0x08: {
      ESP_LOGI(TAG, "In-use tracking data: %s", format_hex_pretty(data).c_str());
      std::vector<uint8_t> items(data.begin() + 3, data.end());
      bool first = true;
      for (size_t i = 0; i < items.size(); i += 2) {
        uint16_t value = (items[i] << 8) | items[i + 1];
        if (!first) {
          task.value_s += ", ";
        }
        first = false;
        task.value_s += get_ipt_name(i) + "/" + std::to_string(value);
      }
      break;
    }
    case 0x0A: {
      std::string ecu_name(data.begin() + 3, data.end());
      task.value_s = ecu_name;
      break;
    }
    case 0x14: break;
    default: break;
  }
}

void OBD2BLEClient::handle_mode_22(OBD2Task &task, const std::vector<uint8_t> &data) {
  uint16_t pid = (data[1] << 8) | data[2];
  switch (pid) {
    case 0x10A6:
    case 0x10B2:
    case 0x10B4:
    case 0x10B5:
    case 0x10E3:
    case 0x1121:
    case 0x1137:
    case 0x1231:
    case 0x1232:
    case 0x1233:
    case 0x1234:
    case 0x124A:
    case 0x124C:
    default: break;
  }
}

void OBD2BLEClient::handle_dtc_response(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_s = "";
  if (data.size() <= 2) {
    ESP_LOGD(TAG, "No Data");
    task.value_s = "";
  } else {
    for (size_t i = 1; i + 1 < data.size(); i += 2) {
      uint8_t high_byte = data[i];
      uint8_t low_byte = data[i + 1];

      char dtc_type;
      switch (high_byte >> 4) {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3: dtc_type = 'P'; break;
        case 0x4:
        case 0x5: dtc_type = 'C'; break;
        case 0x6:
        case 0x7: dtc_type = 'B'; break;
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB: dtc_type = 'U'; break;
        default: dtc_type = '?'; break;
      }

      char dtc[6];
      snprintf(dtc, sizeof(dtc), "%c%01X%02X", dtc_type, high_byte & 0x0F, low_byte);

      ESP_LOGD(TAG, "Decoded DTC: %s", dtc);
    
      task.value_s += std::string(dtc) + " ";
    }
  }
}

std::string OBD2BLEClient::get_monitor_status(const std::vector<uint8_t> &data) {
  std::string value = "";
  
  if (is_bit_set(data[2], 7)) {
    value += "MIL/on";
  } else {
    value += "MIL/off";
  }
  
  if (is_bit_set(data[3], 0) && !is_bit_set(data[3], 4)) {
    value += ", Misfire/completed";
  } else if (is_bit_set(data[3], 0) && is_bit_set(data[3], 4)) {
    value += ", Misfire/noncomplete";
  }
  
  if (is_bit_set(data[3], 1) && !is_bit_set(data[3], 5)) {
    value += ", FuelSystem/completed";
  } else if (is_bit_set(data[3], 1) && is_bit_set(data[3], 5)) {
    value += ", FuelSystem/noncomplete";
  }
  
  if (is_bit_set(data[3], 2) && !is_bit_set(data[3], 6)) {
    value += ", Components/completed";
  } else if (is_bit_set(data[3], 2) && is_bit_set(data[3], 6)) {
    value += ", Components/noncomplete";
  }
  
  if (is_bit_set(data[4], 0) && !is_bit_set(data[5], 0)) {
    value += ", Catalyst/completed";
  } else if (is_bit_set(data[4], 0) && is_bit_set(data[5], 0)) {
    value += ", Catalyst/noncomplete";
  }
  
  if (is_bit_set(data[4], 1) && !is_bit_set(data[5], 1)) {
    value += ", HeatedCatalyst/completed";
  } else if (is_bit_set(data[4], 1) && is_bit_set(data[5], 1)) {
    value += ", HeatedCatalyst/noncomplete";
  }
  
  if (is_bit_set(data[4], 2) && !is_bit_set(data[5], 2)) {
    value += ", EvaporativeSystem/completed";
  } else if (is_bit_set(data[4], 2) && is_bit_set(data[5], 2)) {
    value += ", EvaporativeSystem/noncomplete";
  }
  
  if (is_bit_set(data[4], 3) && !is_bit_set(data[5], 3)) {
    value += ", SecondaryAirSystem/completed";
  } else if (is_bit_set(data[4], 3) && is_bit_set(data[5], 3)) {
    value += ", SecondaryAirSystem/noncomplete";
  }
  
  if (is_bit_set(data[4], 4) && !is_bit_set(data[5], 4)) {
    value += ", GasolineParticulateFilter/completed";
  } else if (is_bit_set(data[4], 4) && is_bit_set(data[5], 4)) {
    value += ", GasolineParticulateFilter/noncomplete";
  }
  
  if (is_bit_set(data[4], 5) && !is_bit_set(data[5], 5)) {
    value += ", OxygenSensor/completed";
  } else if (is_bit_set(data[4], 5) && is_bit_set(data[5], 5)) {
    value += ", OxygenSensor/noncomplete";
  }
  
  if (is_bit_set(data[4], 6) && !is_bit_set(data[5], 6)) {
    value += ", OxygenSensorHeater/completed";
  } else if (is_bit_set(data[4], 6) && is_bit_set(data[5], 6)) {
    value += ", OxygenSensorHeater/noncomplete";
  }
  
  if (is_bit_set(data[4], 7) && !is_bit_set(data[5], 7)) {
    value += ", EGRVVTSystem/completed";
  } else if (is_bit_set(data[4], 7) && is_bit_set(data[5], 7)) {
    value += ", EGRVVTSystem/noncomplete";
  }
  
  return value;
}

std::string OBD2BLEClient::get_fuel_system_status(const uint8_t &num) {
  switch (num) {
    case 0x00: return "The motor is off";
    case 0x01: return "Open loop due to insufficient engine temperature";
    case 0x02: return "Closed loop, using oxygen sensor feedback to determine fuel mix";
    case 0x04: return "Open loop due to engine load OR fuel cut due to deceleration";
    case 0x08: return "Open loop due to system failure";
    case 0x10: return "Closed loop, using at least one oxygen sensor but there is a fault in the feedback system";
    default: return "Unknown";
  }
}

std::string OBD2BLEClient::get_standard_name(const uint8_t &num) {
  switch (num) {
    case 0x01: return "OBD-II";
    case 0x02: return "OBD";
    case 0x03: return "OBD and OBD-II";
    case 0x04: return "OBD-I";
    case 0x05: return "Not OBD compliant";
    case 0x06: return "EOBD";
    case 0x07: return "EOBD and OBD-II";
    case 0x08: return "EOBD and OBD";
    case 0x09: return "EOBD, OBD and OBD II";
    case 0x0A: return "JOBD";
    case 0x0B: return "JOBD and OBD II";
    case 0x0C: return "JOBD and EOBD";
    case 0x0D: return "JOBD, EOBD, and OBD II";
    case 0x11: return "EMD";
    case 0x12: return "EMD+";
    case 0x13: return "HD OBD-C";
    case 0x14: return "HD OBD";
    case 0x15: return "WWH OBD";
    case 0x17: return "HD EOBD-I";
    case 0x18: return "HD EOBD-I N";
    case 0x19: return "HD EOBD-II";
    case 0x1A: return "HD EOBD-II N";
    case 0x1C: return "OBDBr-1";
    case 0x1D: return "OBDBr-2";
    case 0x1E: return "KOBD";
    case 0x1F: return "IOBD I";
    case 0x20: return "IOBD II";
    case 0x21: return "HD EOBD-IV";
    default:  return "Unknown";
  }
}

std::string OBD2BLEClient::get_fuel_type(const uint8_t &num) {
  switch (num) {
    case 0x01: return "Gasoline";
    case 0x02: return "Methanol";
    case 0x03: return "Ethanol";
    case 0x04: return "Diesel";
    case 0x05: return "LPG";
    case 0x06: return "CNG";
    case 0x07: return "Propane";
    case 0x08: return "Electric";
    case 0x09: return "Bifuel running Gasoline";
    case 0x0A: return "Bifuel running Methanol";
    case 0x0B: return "Bifuel running Ethanol";
    case 0x0C: return "Bifuel running LPG";
    case 0x0D: return "Bifuel running CNG";
    case 0x0E: return "Bifuel running Propane";
    case 0x0F: return "Bifuel running Electricity";
    case 0x10: return "Bifuel running electric and combustion engine";
    case 0x11: return "Hybrid gasoline";
    case 0x12: return "Hybrid Ethanol";
    case 0x13: return "Hybrid Diesel";
    case 0x14: return "Hybrid Electric";
    case 0x15: return "Hybrid running electric and combustion engine";
    case 0x16: return "Hybrid Regenerative";
    case 0x17: return "Bifuel running diesel";
    default:  return "Unknown";
  }
}

bool OBD2BLEClient::is_bit_set(const uint8_t &value, const int &position) {
  uint8_t mask = 1 << position;
  return (value & mask) != 0;
}

std::string OBD2BLEClient::get_ipt_name(const size_t &num) {
  switch (num) {
    case 0: return "OBDCOND";
    case 2: return "IGNCNTR";
    case 4: return "CATCOMP1";
    case 6: return "CATCOND1";
    case 8: return "CATCOMP2";
    case 10: return "CATCOND2";
    case 12: return "O2SCOMP1";
    case 14: return "O2SCOND1";
    case 16: return "O2SCOMP2";
    case 18: return "O2SCOND2";
    case 20: return "EGRCOMP";
    case 22: return "EGRCOND";
    case 24: return "AIRCOMP";
    case 26: return "AIRCOND";
    case 28: return "EVAPCOMP";
    case 30: return "EVAPCOND";
    case 32: return "SO2SCOMP1";
    case 34: return "SO2SCOND1";
    case 36: return "SO2SCOMP2";
    case 38: return "SO2SCOND2";
    default: return "Unknown";
  }
}

void OBD2BLEClient::publish_data(OBD2Task &task) {
  if (!std::isnan(task.value_f) && task.sensor != nullptr) {
    task.sensor->publish_state(task.value_f);
    task.published = true;
  } else if (!task.value_s.empty() && task.text_sensor != nullptr && task.codes.empty()) {
    task.text_sensor->publish_state(task.value_s);
    task.published = true;
  } else if (!task.codes.empty() && task.text_sensor != nullptr) {
    if (current_code_index_ < task.codes.size()) {
      bool code_state = false;
      if (!task.value_s.empty()) {
        code_state = (task.value_s.find(task.codes[current_code_index_]) != std::string::npos);
        if (code_state) {
          ESP_LOGI(TAG, "DTC code %s found in response", task.codes[current_code_index_].c_str());
        }
      }
      std::string sensor_name = task.binary_sensors[current_code_index_]->get_name();
      if (sensor_name.find(task.codes[current_code_index_])) {
        task.binary_sensors[current_code_index_]->publish_state(code_state);
        ESP_LOGD(TAG, "Published Code: %s", task.codes[current_code_index_].c_str());
      } else {
        ESP_LOGW(TAG, "No match, Code: %s, Sensor: %s", task.codes[current_code_index_].c_str(), task.binary_sensors[current_code_index_]->get_name().c_str());
      }
      current_code_index_++;
    } else {
      ESP_LOGD(TAG, "Code publishing completed");
      task.text_sensor->publish_state(task.value_s);
      task.published = true;
    }
  } else {
    task.published = true;
  }
}

void OBD2BLEClient::cleanup() {
  ESP_LOGW(TAG, "Cleaning up...");
  mtu_configured_ = false;
  notifications_ready_ = false;
  descr_write_done_ = false;
  current_task_index_ = 0;
  current_code_index_ = 0;
  for (auto &task : task_queue_) {
    task.status = PENDING;
    task.data.clear();
    task.can_id_map.clear();
    task.published = false;
  }
}

}  // namespace obd2_ble
}  // namespace esphome

#endif
