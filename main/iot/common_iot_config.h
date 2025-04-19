#ifndef COMMON_IOT_CONFIG_H
#define COMMON_IOT_CONFIG_H

#include "iot/thing_manager.h"

namespace iot {

// 初始化DogControl，添加小狗控制器到ThingManager
void InitializeDogControl(ThingManager& thing_manager);

} // namespace iot

#endif // COMMON_IOT_CONFIG_H 