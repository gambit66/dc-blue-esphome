#include "dc_blue.h"

#include <cinttypes>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dc_blue {

static const char *const TAG = "dc_blue";

bool IRAM_ATTR DcBlueComponent::timer_intr_(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *arg) {
  auto *receiver = static_cast<Receiver *>(arg);
  portENTER_CRITICAL_ISR(&receiver->lock);
  if (receiver->active) {
    // The timer runs at half the symbol period; sample in the middle of each bit.
    const bool sample = receiver->sample_next;
    receiver->sample_next = !sample;
    if (sample) {
      const bool value = receiver->pin.digital_read() != receiver->inverted;
      if (receiver->waiting_for_header) {
        receiver->header = (receiver->header << 1) | value;
        if (receiver->header == 0x01) {
          receiver->header = UINT32_MAX;
          receiver->waiting_for_header = false;
          receiver->captured_bits = 0;
          receiver->frame = 0;
        }
      } else {
        receiver->frame = (receiver->frame << 1) | value;
        if (++receiver->captured_bits == 24) {
          const uint8_t next = (receiver->write_index + 1) & QUEUE_MASK;
          if (next != receiver->read_index) {
            receiver->queue[receiver->write_index] = receiver->frame;
            receiver->write_index = next;
          } else if (receiver->frames_dropped != UINT32_MAX) {
            ++receiver->frames_dropped;
          }
          receiver->waiting_for_header = true;
        }
      }
    }
  }
  portEXIT_CRITICAL_ISR(&receiver->lock);
  // No FreeRTOS task is woken by this callback.
  return false;
}

void IRAM_ATTR DcBlueComponent::gpio_intr_(Receiver *receiver) {
  portENTER_CRITICAL_ISR(&receiver->lock);
  if (receiver->active && receiver->waiting_for_header) {
    receiver->sample_next = true;
    // Resetting a running counter schedules the next sample half a bit after
    // this edge. No stop/start transition is needed in the interrupt handler.
    gptimer_set_raw_count(receiver->timer, 0);
  }
  portEXIT_CRITICAL_ISR(&receiver->lock);
}

bool DcBlueComponent::check_timer_(esp_err_t error, const char *operation) {
  if (error == ESP_OK) {
    return true;
  }
  ESP_LOGE(TAG, "Timer %s failed: %s", operation, esp_err_to_name(error));
  this->on_shutdown();
  this->mark_failed();
  return false;
}

void DcBlueComponent::setup() {
  if (this->data_pin_ == nullptr || this->trigger_pin_ == nullptr) {
    ESP_LOGE(TAG, "Data and trigger pins are required");
    this->mark_failed();
    return;
  }

  // Set the inactive latch before enabling the output. Respect the pin mode
  // and inversion configured in YAML (including input pull-ups).
  this->trigger_pin_->digital_write(false);
  this->trigger_pin_->setup();
  this->trigger_pin_->digital_write(false);
  this->data_pin_->setup();
  this->receiver_.pin = this->data_pin_->to_isr();

  // Arduino on ESP32 also uses ESP-IDF, so both frameworks can share GPTimer.
  gptimer_config_t timer_config{};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000;
  if (!this->check_timer_(gptimer_new_timer(&timer_config, &this->receiver_.timer), "allocation")) {
    return;
  }

  gptimer_alarm_config_t alarm_config{};
  alarm_config.alarm_count = this->symbol_period_ / 2;
  alarm_config.flags.auto_reload_on_alarm = true;
  if (!this->check_timer_(gptimer_set_alarm_action(this->receiver_.timer, &alarm_config), "alarm configuration")) {
    return;
  }

  gptimer_event_callbacks_t callbacks{};
  callbacks.on_alarm = timer_intr_;
  if (!this->check_timer_(gptimer_register_event_callbacks(this->receiver_.timer, &callbacks, &this->receiver_),
                          "callback registration")) {
    return;
  }
  if (!this->check_timer_(gptimer_enable(this->receiver_.timer), "enable")) {
    return;
  }
  this->timer_enabled_ = true;
  if (!this->check_timer_(gptimer_start(this->receiver_.timer), "start")) {
    return;
  }
  this->timer_started_ = true;

  portENTER_CRITICAL(&this->receiver_.lock);
  this->receiver_.active = true;
  portEXIT_CRITICAL(&this->receiver_.lock);
  // Edges must not try to reset the timer until it is fully configured/running.
  this->data_pin_->attach_interrupt(gpio_intr_, &this->receiver_, gpio::INTERRUPT_ANY_EDGE);
  this->interrupt_attached_ = true;
  this->ready_ = true;

  if (this->garage_cover_sensor_ != nullptr) {
    this->garage_cover_sensor_->initialize();
  }
  // Binary sensors remain unknown until their first valid motor frame.
}

void DcBlueComponent::on_shutdown() {
  this->ready_ = false;
  this->trigger_pending_ = false;
  this->trigger_state_ = TriggerState::IDLE;
  if (this->trigger_pin_ != nullptr) {
    this->trigger_pin_->digital_write(false);
  }

  portENTER_CRITICAL(&this->receiver_.lock);
  this->receiver_.active = false;
  portEXIT_CRITICAL(&this->receiver_.lock);
  if (this->interrupt_attached_) {
    this->data_pin_->detach_interrupt();
    this->interrupt_attached_ = false;
  }
  if (this->timer_started_) {
    gptimer_stop(this->receiver_.timer);
    this->timer_started_ = false;
  }
  if (this->timer_enabled_) {
    gptimer_disable(this->receiver_.timer);
    this->timer_enabled_ = false;
  }
  if (this->receiver_.timer != nullptr) {
    gptimer_del_timer(this->receiver_.timer);
    this->receiver_.timer = nullptr;
  }
}

void DcBlueComponent::loop() {
  if (!this->ready_) {
    return;
  }
  this->process_trigger_();

  uint32_t frames[QUEUE_SIZE - 1];
  uint8_t count = 0;
  // Copy a bounded batch under the lock; publishing and logging stay outside.
  portENTER_CRITICAL(&this->receiver_.lock);
  const uint32_t dropped = this->receiver_.frames_dropped;
  this->receiver_.frames_dropped = 0;
  while (this->receiver_.read_index != this->receiver_.write_index) {
    frames[count++] = this->receiver_.queue[this->receiver_.read_index];
    this->receiver_.read_index = (this->receiver_.read_index + 1) & QUEUE_MASK;
  }
  portEXIT_CRITICAL(&this->receiver_.lock);

  if (dropped != 0) {
    ESP_LOGW(TAG, "Queue overflow: %" PRIu32 " frame(s) dropped", dropped);
  }
  for (uint8_t i = 0; i < count; ++i) {
    this->process_frame_(frames[i]);
  }
}

void DcBlueComponent::process_frame_(uint32_t frame) {
  const uint8_t command = (frame >> 16) & 0xFF;
  const uint8_t state = (frame >> 8) & 0xFF;
  const uint8_t checksum = frame & 0xFF;
  const uint8_t expected_checksum = state | 0x01;

  if (command != protocol::CMD_DOOR_STATE && command != protocol::CMD_LIGHT_LOCK) {
    ESP_LOGW(TAG, "Unknown command: %02X data: %02X checksum: %02X (frame: %08" PRIX32 ")",
             static_cast<unsigned>(command), static_cast<unsigned>(state), static_cast<unsigned>(checksum), frame);
    return;
  }
  if (checksum != expected_checksum) {
    ESP_LOGW(TAG, "Invalid frame checksum: %08" PRIX32 " (expected %02X)", frame,
             static_cast<unsigned>(expected_checksum));
    return;
  }

  if (command == protocol::CMD_DOOR_STATE) {
    const bool ac_power = (state & protocol::FLAG_AC_POWER) != 0;
    const uint8_t door_state = state & ~protocol::FLAG_AC_POWER;
    if (this->ac_power_binary_sensor_ != nullptr) {
      this->ac_power_binary_sensor_->publish_state(ac_power);
    }

    switch (door_state) {
      case protocol::DOOR_CLOSED:
        this->process_door_state_(cover::COVER_CLOSED, cover::COVER_OPERATION_OPENING);
        break;
      case protocol::DOOR_MOVING:
        this->process_motor_running_();
        break;
      case protocol::DOOR_OPEN:
        this->process_door_state_(cover::COVER_OPEN, cover::COVER_OPERATION_CLOSING);
        break;
      default:
        ESP_LOGW(TAG, "Unknown door state: %02X (frame: %08" PRIX32 ")", static_cast<unsigned>(door_state), frame);
        break;
    }
    return;
  }

  switch (state) {
    case protocol::LIGHT_ON:
    case protocol::LIGHT_OFF:
      if (this->light_binary_sensor_ != nullptr) {
        this->light_binary_sensor_->publish_state(state == protocol::LIGHT_ON);
      }
      break;
    case protocol::STRIKE_LOCK:
      ESP_LOGD(TAG, "Strike lock");
      break;
    case protocol::MAGNETIC_LOCK:
      ESP_LOGD(TAG, "Magnetic lock");
      break;
    default:
      ESP_LOGW(TAG, "Unknown light/lock state: %02X (frame: %08" PRIX32 ")", static_cast<unsigned>(state), frame);
      break;
  }
}

void DcBlueComponent::process_door_state_(float position, cover::CoverOperation next_direction) {
  this->next_direction_ = next_direction;
  auto *sensor = this->garage_cover_sensor_;
  if (sensor != nullptr &&
      (sensor->position != position || sensor->current_operation != cover::COVER_OPERATION_IDLE)) {
    sensor->position = position;
    sensor->current_operation = cover::COVER_OPERATION_IDLE;
    sensor->publish_state(false);
  }
}

void DcBlueComponent::process_motor_running_() {
  auto *sensor = this->garage_cover_sensor_;
  if (sensor != nullptr && sensor->current_operation != this->next_direction_) {
    sensor->current_operation = this->next_direction_;
    sensor->publish_state(false);
  }
}

void DcBlueComponent::request_trigger() {
  if (!this->ready_) {
    ESP_LOGW(TAG, "Ignoring trigger: component is not ready");
    return;
  }
  if (this->trigger_pending_) {
    ESP_LOGD(TAG, "Ignoring trigger: a pulse is already pending");
    return;
  }
  this->trigger_pending_ = true;
}

void DcBlueComponent::process_trigger_() {
  const uint32_t now = millis();
  switch (this->trigger_state_) {
    case TriggerState::IDLE:
      if (this->trigger_pending_) {
        this->trigger_pending_ = false;
        this->trigger_pin_->digital_write(true);
        this->trigger_state_ = TriggerState::PIN_HIGH;
        this->trigger_state_time_ = now;
      }
      break;
    case TriggerState::PIN_HIGH:
      if (now - this->trigger_state_time_ >= this->trigger_period_) {
        this->trigger_pin_->digital_write(false);
        this->trigger_state_ = TriggerState::PIN_LOW_WAIT;
        this->trigger_state_time_ = now;
      }
      break;
    case TriggerState::PIN_LOW_WAIT:
      if (now - this->trigger_state_time_ >= this->clear_period_) {
        this->trigger_state_ = TriggerState::IDLE;
      }
      break;
  }
}

void DcBlueComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DC Blue:");
  LOG_PIN("  Trigger Pin: ", this->trigger_pin_);
  LOG_PIN("  Data Pin: ", this->data_pin_);
  ESP_LOGCONFIG(TAG, "  Symbol period: %" PRIu32 " us", this->symbol_period_);
  ESP_LOGCONFIG(TAG, "  Inverted: %s", YESNO(this->receiver_.inverted));
  ESP_LOGCONFIG(TAG, "  Trigger period: %" PRIu32 " ms", this->trigger_period_);
  ESP_LOGCONFIG(TAG, "  Clear period: %" PRIu32 " ms", this->clear_period_);
}

}  // namespace dc_blue
}  // namespace esphome
