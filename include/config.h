// ============================================================
// config.h — 3D打印耗材烘干箱控制系统 配置文件
// 主控芯片: ESP32-C3
// ============================================================
#ifndef CONFIG_H
#define CONFIG_H

// ==================== 引脚定义 ====================
#define HEATER_PIN          4       // 加热片继电器 — IO4（高电平触发）
#define FAN_PIN             5       // 12V风扇继电器 — IO5（高电平触发）
#define HEATER_DS18B20_PIN  13      // 加热片 DS18B20 数据引脚 — IO13
#define BOX_DS18B20_PIN     11      // 箱内环境 DS18B20 数据引脚 — IO11

// ==================== WiFi 配置 ====================
#define WIFI_SSID       "hs10000"
#define WIFI_PASS       "kazikazi@123"

// ==================== 默认控制参数 ====================
#define DEFAULT_TARGET_TEMP     60      // 默认箱内目标温度 (℃)
#define DEFAULT_DRY_TIME        120     // 默认烘干时间 (分钟)
#define HYSTERESIS              3       // 滞回比较回差 (℃)

// ==================== 安全保护参数（默认值） ====================
#define DEFAULT_BOX_STOP_TEMP    85     // 箱内停机温度 (℃)
#define DEFAULT_HEATER_STOP_TEMP 100    // 加热片停机温度 (℃)
#define DEFAULT_HEATER_OFFSET    20     // 加热片超前偏移 (℃)
#define DEFAULT_HEATER_FALLBACK  20     // 加热片停机后恢复的回落差值 (℃)
#define COOLING_TARGET_TEMP      35     // 散热目标温度 (℃)
#define SENSOR_FAULT_THRESHOLD   5      // 传感器连续读取失败次数阈值

// ==================== 单传感器模式 ====================
#define PREF_KEY_SINGLE_SENSOR      "singleSensor"  // 存储键名
#define DEFAULT_SINGLE_SENSOR       false           // 默认关闭
#define SINGLE_SENSOR_COOLING_TIMEOUT 300            // 散热倒计时秒数（5分钟）

// ==================== 时间间隔参数 (ms) ====================
#define SENSOR_READ_INTERVAL    1000
#define WEB_CHECK_INTERVAL      50
#define AGENT_CHECK_INTERVAL    100

// ==================== Preferences 存储键名 ====================
#define PREF_NAMESPACE          "drybox"
#define PREF_KEY_TARGET_TEMP    "target"
#define PREF_KEY_DRY_TIME       "time"
#define PREF_KEY_BOX_STOP       "boxStop"
#define PREF_KEY_HEATER_STOP    "heaterStop"
#define PREF_KEY_HEATER_OFFSET  "heaterOffset"
#define PREF_KEY_HEATER_FALLBACK "heaterFallback"
#define PREF_KEY_AGENT_ENABLED  "agentEnabled"
#define PREF_KEY_AGENT_APIKEY   "agentApiKey"

// ==================== 状态机枚举 ====================
enum SystemState {
    STATE_STANDBY = 0,
    STATE_DRYING = 1,
    STATE_COOLING = 2
};

#endif // CONFIG_H