#include "dc_blue_cover.h"
#include "dc_blue.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dc_blue {

static const char *const TAG = "dc_blue.cover";

void DcBlueCover::initialize() {
  ESP_LOGD(TAG, "Assuming door closed and idle until motor feedback arrives");
  this->position = cover::COVER_CLOSED;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state(false);
}

cover::CoverTraits DcBlueCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(false);
  traits.set_supports_toggle(true);
  traits.set_supports_tilt(false);
  // The motor reports motion, but not its direction or intermediate stops.
  traits.set_is_assumed_state(true);
  return traits;
}

void DcBlueCover::control(const cover::CoverCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Component is not configured");
    return;
  }

  if (call.get_stop()) {
    this->parent_->cancel_pending_trigger();
    // A pulse while idle starts the motor; it cannot act as a stop command.
    if (this->current_operation != cover::COVER_OPERATION_IDLE) {
      this->parent_->request_trigger();
    }
    return;
  }

  if (call.get_toggle().value_or(false)) {
    this->parent_->request_trigger();
    return;
  }

  if (!call.get_position().has_value()) {
    return;
  }

  const float target = *call.get_position();
  if (target != cover::COVER_OPEN && target != cover::COVER_CLOSED) {
    ESP_LOGD(TAG, "Intermediate positions are not supported");
    return;
  }
  if (this->current_operation != cover::COVER_OPERATION_IDLE || this->parent_->is_trigger_busy()) {
    ESP_LOGD(TAG, "Ignoring position command while moving or processing a trigger");
    return;
  }
  if (this->position != target) {
    this->parent_->request_trigger();
  }
}

}  // namespace dc_blue
}  // namespace esphome
