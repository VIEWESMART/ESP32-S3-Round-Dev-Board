#include "sdkconfig.h"
#include "board.h"
#include "iot/thing.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_system.h"
#include <driver/sdspi_host.h>
#include <esp_log.h>
#include "config.h"
#include <string>
#include "lvgl.h"
#define TAG "SDCard"

namespace iot
{
    class SDCard : public Thing
    {
    public:
        SDCard() : Thing("SDCard", "SD卡存储管理"), is_sd_card_initialized_(false)
        {
            InitializeSDCard();
            DefineProperties();
            DefineMethods();
        }

    private:
        bool is_sd_card_initialized_;
        sdmmc_card_t *card_ = nullptr;
        const char *mount_point_ = "/sdcard";

        // 初始化 SD 卡
        void InitializeSDCard()
        {
            esp_err_t ret;
            esp_vfs_fat_sdmmc_mount_config_t mount_config = {
                .format_if_mount_failed = false,
                .max_files = 100, // 打开文件的最大数量
                .allocation_unit_size = 4 * 1024};

            ESP_LOGI(TAG, "初始化 SD 卡");
            ESP_LOGI(TAG, "使用 SDMMC 外设");
            sdmmc_host_t host = SDMMC_HOST_DEFAULT();
            host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
            sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
            slot_config.cd = SDCARD_PIN_CD,
            slot_config.width = 1;
            slot_config.clk = SDCARD_PIN_CLK;
            slot_config.cmd = SDCARD_PIN_CMD;
            slot_config.d0 = SDCARD_PIN_D0;
            slot_config.d1 = SDCARD_PIN_D1;
            slot_config.d2 = SDCARD_PIN_D2;
            slot_config.d3 = SDCARD_PIN_D3;
            slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
            ESP_LOGI(TAG, "挂载文件系统");
            ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card_);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "初始化卡失败 (%s). "
                              "确保 SD 卡线具有上拉电阻。",
                         esp_err_to_name(ret));
                return;
            }
            is_sd_card_initialized_ = true;
            ESP_LOGI(TAG, "文件系统挂载");
            sdmmc_card_print_info(stdout, card_);
        }

        void DefineProperties()
        {
            if (!is_sd_card_initialized_)
            {
                ESP_LOGW(TAG, "SD 卡未初始化。跳过属性定义。");
                return;
            }
            properties_.AddNumberProperty("capacity", "总存储空间（单位：MB）", [this]() -> int
                                          {
                                              return GetSDCardCapacity(); // 获取总容量
                                          });

            properties_.AddNumberProperty("used_space", "已用存储空间（单位：MB）", [this]() -> int
                                          {
                                              return GetSDCardUsedSpace(); // 获取已用空间
                                          });

            properties_.AddNumberProperty("free_space", "剩余存储空间（单位：MB）", [this]() -> int
                                          {
                                              return GetSDCardFreeSpace(); // 获取剩余空间
                                          });

            properties_.AddStringProperty("file_list", "文件列表", [this]() -> std::string
                                          {
                                              return ListDirsWithLVGL(); // 列出指定路径下的文件
                                          });
            properties_.AddStringProperty("music_list", "音乐列表", [this]() -> std::string
                                          {
                                              return ListMusicsWithLVGL(); // 列出指定路径下的文件
                                          });
        }

        void DefineMethods()
        {
            if (!is_sd_card_initialized_)
            {
                ESP_LOGW(TAG, "SD 卡未初始化。跳过方法定义。");
                return;
            }

        }

        // 底层实现函数

        int GetSDCardCapacity()
        {
            uint64_t total_bytes = 0, free_bytes = 0;
            esp_err_t err = esp_vfs_fat_info(mount_point_, &total_bytes, &free_bytes);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "无法获取文件系统统计信息: %s", esp_err_to_name(err));
                return -1;
            }
            // 将字节数转换为 MB
            uint64_t total_mb = total_bytes / (1024 * 1024);
            return static_cast<int>(total_mb);
        }

        int GetSDCardUsedSpace()
        {
            uint64_t total_bytes = 0, free_bytes = 0;
            esp_err_t err = esp_vfs_fat_info(mount_point_, &total_bytes, &free_bytes);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "无法获取文件系统统计信息: %s", esp_err_to_name(err));
                return -1;
            }
            uint64_t total_mb = total_bytes / (1024 * 1024);
            uint64_t free_mb = free_bytes / (1024 * 1024);
            uint64_t used_space = total_mb - free_mb;
            return static_cast<int>(used_space);
        }

        int GetSDCardFreeSpace()
        {
            uint64_t total_bytes = 0, free_bytes = 0;
            esp_err_t err = esp_vfs_fat_info(mount_point_, &total_bytes, &free_bytes);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "无法获取文件系统统计信息: %s", esp_err_to_name(err));
                return -1;
            }
            uint64_t free_mb = free_bytes / (1024 * 1024);
            return static_cast<int>(free_mb);
        }

        std::string ListMusicsWithLVGL()
        {
            lv_fs_dir_t dir;
            std::string fullPath = "S:/music/"; // 使用 LVGL 的驱动器字母 'S'
            std::string fileList;

            lv_fs_res_t res = lv_fs_dir_open(&dir, fullPath.c_str());
            if (res != LV_FS_RES_OK)
            {
                ESP_LOGE(TAG, "无法打开目录: %s", fullPath.c_str());
                return "";
            }

            char filename[256]; // 文件名缓冲区
            while (lv_fs_dir_read(&dir, filename, sizeof(filename)) == LV_FS_RES_OK && filename[0] != '\0')
            {
                fileList += filename;
                fileList += "，";
            }

            lv_fs_dir_close(&dir);

            if (!fileList.empty())
            {
                fileList.pop_back(); // 移除最后一个符号
            }

            return fileList;
        }

        std::string ListDirsWithLVGL()
        {
            lv_fs_dir_t dir;
            std::string fullPath = "S:"; // 使用 LVGL 的驱动器字母 'S'
            std::string fileList;

            lv_fs_res_t res = lv_fs_dir_open(&dir, fullPath.c_str());
            if (res != LV_FS_RES_OK)
            {
                ESP_LOGE(TAG, "无法打开目录: %s", fullPath.c_str());
                return "";
            }

            char filename[256]; // 文件名缓冲区
            while (lv_fs_dir_read(&dir, filename, sizeof(filename)) == LV_FS_RES_OK && filename[0] != '\0')
            {
                fileList += filename;
                fileList += "，";
            }

            lv_fs_dir_close(&dir);

            if (!fileList.empty())
            {
                fileList.pop_back(); // 移除最后一个多余的逗号
            }

            return fileList;
        }
    };

} // namespace iot

DECLARE_THING(SDCard);
