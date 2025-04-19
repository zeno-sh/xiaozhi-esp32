#include "common_iot_config.h"
#include <esp_log.h>

#define TAG "CommonIotConfig"

namespace iot {

// 添加DogControl到ThingManager
void InitializeDogControl(ThingManager& thing_manager) {
    // 创建DogControl实例并添加到ThingManager
    Thing* dog_control = CreateThing("DogControl");
    if (dog_control) {
        ESP_LOGI(TAG, "添加小狗控制器到ThingManager");
        thing_manager.AddThing(dog_control);
    } else {
        ESP_LOGE(TAG, "创建小狗控制器失败");
    }
}

} // namespace iot 