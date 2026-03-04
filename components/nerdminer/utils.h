#pragma once

#include <stddef.h>
#include <stdint.h>

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"

namespace esphome {
namespace nerdminer {

void trim(std::string &s);

uint32_t try_overclock();
void setup_powermanagement();

}  // namespace nerdminer
}  // namespace esphome
