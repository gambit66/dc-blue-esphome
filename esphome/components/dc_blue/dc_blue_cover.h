#pragma once

#include "esphome/components/cover/cover.h"

namespace esphome {
namespace dc_blue {

class DcBlueComponent;

class DcBlueCover : public cover::Cover {
 public:
  void initialize();
  void set_parent(DcBlueComponent *parent) { this->parent_ = parent; }
  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;
  DcBlueComponent *parent_{nullptr};
};

}  // namespace dc_blue
}  // namespace esphome
