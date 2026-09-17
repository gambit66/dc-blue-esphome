#pragma once

#include <cstdint>

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "dc_blue_cover.h"

namespace esphome {
namespace dc_blue {

namespace protocol {
constexpr uint8_t CMD_DOOR_STATE = 0x2C;
constexpr uint8_t CMD_LIGHT_LOCK = 0x55;
constexpr uint8_t DOOR_CLOSED = 0x20;
constexpr uint8_t DOOR_MOVING = 0x08;
constexpr uint8_t DOOR_OPEN = 0x02;
constexpr uint8_t FLAG_AC_POWER = 0x04;
constexpr uint8_t LIGHT_ON = 0x13;
constexpr uint8_t LIGHT_OFF = 0x15;
constexpr uint8_t STRIKE_LOCK = 0x0B;
constexpr uint8_t MAGNETIC_LOCK = 0x0D;
}  // namespace protocol

class DcBlueComponent : public Component {
 public:
  SUB_BINARY_SENSOR(light)
  SUB_BINARY_SENSOR(ac_power)

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;

  void set_data_pin(InternalGPIOPin *pin) { this->data_pin_ = pin; }
  void set_trigger_pin(InternalGPIOPin *pin) { this->trigger_pin_ = pin; }
  void set_symbol_period(uint32_t period) { this->symbol_period_ = period; }
  void set_inverted(bool inverted) { this->receiver_.inverted = inverted; }
  void set_trigger_period(uint32_t period) { this->trigger_period_ = period; }
  void set_clear_period(uint32_t period) { this->clear_period_ = period; }
  void set_garage_cover_sensor(DcBlueCover *cover) {
    this->garage_cover_sensor_ = cover;
    cover->set_parent(this);
  }

  // Called by the cover on the ESPHome loop task. Keep at most one pending
  // pulse so repeated commands cannot accumulate delayed door movements.
  void request_trigger();
  void cancel_pending_trigger() { this->trigger_pending_ = false; }
  bool is_trigger_busy() const { return this->trigger_pending_ || this->trigger_state_ != TriggerState::IDLE; }

 protected:
  static constexpr uint8_t QUEUE_SIZE = 4;
  static constexpr uint8_t QUEUE_MASK = QUEUE_SIZE - 1;
  static_assert((QUEUE_SIZE & QUEUE_MASK) == 0, "QUEUE_SIZE must be a power of two");

  // The GPIO ISR, timer ISR and loop all use this lock when accessing the
  // receiver. Volatile alone does not synchronize the ESP32's two cores.
  struct Receiver {
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
    ISRInternalGPIOPin pin;
    gptimer_handle_t timer{nullptr};
    bool active{false};
    bool inverted{false};
    bool sample_next{true};
    bool waiting_for_header{true};
    uint32_t header{UINT32_MAX};
    uint32_t frame{0};
    uint8_t captured_bits{0};
    uint32_t queue[QUEUE_SIZE]{};
    uint8_t write_index{0};
    uint8_t read_index{0};
    uint32_t frames_dropped{0};
  };

  static bool IRAM_ATTR timer_intr_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event, void *arg);
  static void IRAM_ATTR gpio_intr_(Receiver *receiver);
  bool check_timer_(esp_err_t error, const char *operation);
  void process_frame_(uint32_t frame);
  void process_door_state_(float position, cover::CoverOperation next_direction);
  void process_motor_running_();
  void process_trigger_();

  InternalGPIOPin *data_pin_{nullptr};
  InternalGPIOPin *trigger_pin_{nullptr};
  DcBlueCover *garage_cover_sensor_{nullptr};
  Receiver receiver_;
  bool timer_enabled_{false};
  bool timer_started_{false};
  bool interrupt_attached_{false};
  bool ready_{false};
  cover::CoverOperation next_direction_{cover::COVER_OPERATION_OPENING};

  uint32_t symbol_period_{970};  // microseconds
  uint32_t trigger_period_{1000};  // milliseconds
  uint32_t clear_period_{1000};  // milliseconds

  enum class TriggerState : uint8_t { IDLE, PIN_HIGH, PIN_LOW_WAIT };
  TriggerState trigger_state_{TriggerState::IDLE};
  bool trigger_pending_{false};
  uint32_t trigger_state_time_{0};
};

}  // namespace dc_blue
}  // namespace esphome
