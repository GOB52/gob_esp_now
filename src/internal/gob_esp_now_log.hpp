
#ifndef GOBLIB_ESP_NOW_LOG_HPP
#define GOBLIB_ESP_NOW_LOG_HPP

#include <esp_log.h>

#define LIB_LOGE(fmt, ...) ESP_LOGE(goblib::esp_now::LIB_TAG, #fmt, ##__VA_ARGS__)
#define LIB_LOGW(fmt, ...) ESP_LOGW(goblib::esp_now::LIB_TAG, #fmt, ##__VA_ARGS__)
#define LIB_LOGI(fmt, ...) ESP_LOGI(goblib::esp_now::LIB_TAG, #fmt, ##__VA_ARGS__)
#define LIB_LOGD(fmt, ...) ESP_LOGD(goblib::esp_now::LIB_TAG, #fmt, ##__VA_ARGS__)
#define LIB_LOGV(fmt, ...) ESP_LOGV(goblib::esp_now::LIB_TAG, #fmt, ##__VA_ARGS__)

#endif
