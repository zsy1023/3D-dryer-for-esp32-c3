/**
 * @file main.cpp
 * @brief 3D打印耗材烘干箱控制系统（ESP32-C3）—— 双传感器 + TCP Agent + 单传感器应急模式
 *
 * 功能：
 * - 双DS18B20温度传感器：加热片(IO13) + 箱内环境(IO11)
 * - 箱内目标温度控制，加热片超前加热（可设偏移量）
 * - 加热片停机保护（最高优先级），箱内过温保护
 * - 新增单传感器模式：仅使用加热片传感器，箱内传感器忽略
 *   - 加热时直接用目标温度控制加热片（无偏移）
 *   - 散热阶段依据加热片温度降至35℃自动停止
 * - Web服务器(80)提供完整人机界面，手机/电脑自适应
 * - TCP Agent服务器(8080)供智能体远程控制，支持API Key认证
 *   - 新增 SINGLESENSOR ON/OFF 命令（需重启生效）
 *   - 新增 RESTART 命令远程重启
 * - 参数可配置，停机阈值等修改后重启生效
 * - 非阻塞温度读取，系统运行流畅
 * - 修复：TCP服务器在WiFi连接成功后才启动，避免崩溃
 * - 修复：API Key输入框防刷新覆盖
 */

#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==================== 传感器对象 ====================
OneWire wireHeater(HEATER_DS18B20_PIN);
OneWire wireBox(BOX_DS18B20_PIN);
DallasTemperature dsHeater(&wireHeater);
DallasTemperature dsBox(&wireBox);

WebServer server(80);
Preferences prefs;

// ==================== 全局变量 ====================
// 温度相关
float boxTemp = 0.0;
float heaterTemp = 0.0;
bool boxSensorOK = false;
bool heaterSensorOK = false;
int sensorFailCount = 0;
bool sensorFault = false;

// 控制参数
int targetTemp = DEFAULT_TARGET_TEMP;
int dryTime = DEFAULT_DRY_TIME;
int remainingSeconds = 0;
unsigned long dryStartMillis = 0;

// 安全阈值（可配置，从NVS加载）
int boxStopTemp = DEFAULT_BOX_STOP_TEMP;
int heaterStopTemp = DEFAULT_HEATER_STOP_TEMP;
int heaterOffset = DEFAULT_HEATER_OFFSET;
int heaterFallback = DEFAULT_HEATER_FALLBACK;

// 状态
SystemState systemState = STATE_STANDBY;
bool overheatActive = false;   // 箱内过热标志
bool heaterForceOff = false;   // 加热片强制关闭（过温保护）
bool heaterRelay = false;
bool fanRelay = false;

// 手动控制
bool manualMode = false;
bool manualHeater = false;
bool manualFan = false;

// 单传感器模式（从NVS加载）
bool singleSensorMode = false;

// 非阻塞温度读取状态
bool tempReqHeater = false;
unsigned long tempReqTimeHeater = 0;
bool tempReqBox = false;
unsigned long tempReqTimeBox = 0;

unsigned long lastSensorRead = 0;

// ==================== 日志系统 ====================
#define LOG_BUF_SIZE 2048
static char logBuffer[LOG_BUF_SIZE] = "";

void logMessage(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char line[256];
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  Serial.print(line);
  if (strlen(logBuffer) + strlen(line) >= LOG_BUF_SIZE - 10) {
    char* p = strchr(logBuffer, '\n');
    if (p) memmove(logBuffer, p + 1, strlen(p + 1) + 1);
    else logBuffer[0] = '\0';
  }
  strcat(logBuffer, line);
}
#define LOG(fmt, ...) logMessage(fmt "\n", ##__VA_ARGS__)

// ==================== Agent TCP 服务器 ====================
WiFiServer tcpServer(8080);
WiFiClient tcpClient;
bool agentEnabled = false;
String agentApiKey = "";
bool agentConnected = false;
String agentInputBuffer = "";

// ==================== 硬件初始化 ====================
void initHardware() {
  pinMode(HEATER_PIN, OUTPUT); pinMode(FAN_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW); digitalWrite(FAN_PIN, LOW);

  dsHeater.begin();
  dsBox.begin();

  int hCount = dsHeater.getDeviceCount();
  int bCount = dsBox.getDeviceCount();
  LOG("[硬件] 加热片传感器: %s", hCount > 0 ? "已检测到" : "未检测到");
  LOG("[硬件] 箱内传感器: %s", bCount > 0 ? "已检测到" : "未检测到");
}

// ==================== 参数加载 ====================
void loadParameters() {
  prefs.begin(PREF_NAMESPACE, false);
  targetTemp = prefs.getInt(PREF_KEY_TARGET_TEMP, DEFAULT_TARGET_TEMP);
  dryTime = prefs.getInt(PREF_KEY_DRY_TIME, DEFAULT_DRY_TIME);
  boxStopTemp = prefs.getInt(PREF_KEY_BOX_STOP, DEFAULT_BOX_STOP_TEMP);
  heaterStopTemp = prefs.getInt(PREF_KEY_HEATER_STOP, DEFAULT_HEATER_STOP_TEMP);
  heaterOffset = prefs.getInt(PREF_KEY_HEATER_OFFSET, DEFAULT_HEATER_OFFSET);
  heaterFallback = prefs.getInt(PREF_KEY_HEATER_FALLBACK, DEFAULT_HEATER_FALLBACK);
  agentEnabled = prefs.getBool(PREF_KEY_AGENT_ENABLED, false);
  agentApiKey = prefs.getString(PREF_KEY_AGENT_APIKEY, "");
  singleSensorMode = prefs.getBool(PREF_KEY_SINGLE_SENSOR, DEFAULT_SINGLE_SENSOR);

  if (targetTemp < 0 || targetTemp > 100) targetTemp = DEFAULT_TARGET_TEMP;
  if (dryTime < 1 || dryTime > 999) dryTime = DEFAULT_DRY_TIME;
  if (boxStopTemp < 0 || boxStopTemp > 85) boxStopTemp = DEFAULT_BOX_STOP_TEMP;
  if (heaterStopTemp < 0 || heaterStopTemp > 120) heaterStopTemp = DEFAULT_HEATER_STOP_TEMP;
  if (heaterOffset < 0 || heaterOffset > 50) heaterOffset = DEFAULT_HEATER_OFFSET;
  if (heaterFallback < 5 || heaterFallback > 50) heaterFallback = DEFAULT_HEATER_FALLBACK;

  prefs.end();

  LOG("[参数] 目标温度: %d°C, 时间: %d分钟", targetTemp, dryTime);
  LOG("[参数] 箱内停机: %d°C, 加热片停机: %d°C, 偏移: %d°C, 回落: %d°C", 
      boxStopTemp, heaterStopTemp, heaterOffset, heaterFallback);
  LOG("[参数] 单传感器模式: %s", singleSensorMode ? "启用" : "禁用");
  LOG("[Agent] 状态: %s, API Key: %s", agentEnabled ? "启用" : "禁用", 
      agentApiKey.length() > 0 ? "已配置" : "未配置");
  // TCP 服务器将在 WiFi 连接成功后再启动
}

// ==================== 非阻塞温度读取 ====================
void readHeaterTemperature() {
  if (!tempReqHeater) {
    dsHeater.requestTemperatures();
    tempReqHeater = true;
    tempReqTimeHeater = millis();
    return;
  }
  if (millis() - tempReqTimeHeater < 750) return;

  float t = dsHeater.getTempCByIndex(0);
  tempReqHeater = false;
  if (t == DEVICE_DISCONNECTED_C) {
    heaterSensorOK = false;
  } else {
    heaterTemp = t;
    heaterSensorOK = true;
  }
}

void readBoxTemperature() {
  // 单传感器模式：忽略箱内传感器，强制标记正常
  if (singleSensorMode) {
    boxSensorOK = true;
    sensorFailCount = 0;
    sensorFault = false;
    boxTemp = -999.0;   // 用于界面显示“忽略”
    return;
  }

  if (!tempReqBox) {
    dsBox.requestTemperatures();
    tempReqBox = true;
    tempReqTimeBox = millis();
    return;
  }
  if (millis() - tempReqTimeBox < 750) return;

  float t = dsBox.getTempCByIndex(0);
  tempReqBox = false;
  if (t == DEVICE_DISCONNECTED_C) {
    boxSensorOK = false;
    sensorFailCount++;
  } else {
    boxTemp = t;
    boxSensorOK = true;
    sensorFailCount = 0;
    if (sensorFault) {
      sensorFault = false;
      LOG("[传感器] 箱内传感器故障恢复");
    }
  }
}

// ==================== 安全检测 ====================
void checkSensorFault() {
  if (singleSensorMode) return;   // 不检测
  if (!sensorFault && sensorFailCount >= SENSOR_FAULT_THRESHOLD) {
    sensorFault = true;
    LOG("[保护] 箱内传感器故障！停止加热");
    if (systemState == STATE_DRYING) {
      systemState = STATE_COOLING;
    }
  }
}

void checkOverheatProtection() {
  // 箱内过热保护（仅双传感器时有效）
  if (!singleSensorMode && boxSensorOK) {
    if (!overheatActive && boxTemp >= boxStopTemp) {
      overheatActive = true;
      LOG("[保护] 箱内过热！%.1f°C >= %d°C，强制散热", boxTemp, boxStopTemp);
      systemState = STATE_COOLING;
    }
    if (overheatActive && boxTemp <= COOLING_TARGET_TEMP) {
      overheatActive = false;
      LOG("[保护] 箱内过热解除，温度降至 %.1f°C", boxTemp);
    }
  }

  // 加热片过热保护（始终有效）
  if (heaterSensorOK) {
    if (!heaterForceOff && heaterTemp >= heaterStopTemp) {
      heaterForceOff = true;
      LOG("[保护] 加热片过热！%.1f°C >= %d°C，强制关闭加热", heaterTemp, heaterStopTemp);
    }
    if (heaterForceOff && heaterTemp <= heaterStopTemp - heaterFallback) {
      heaterForceOff = false;
      LOG("[保护] 加热片温度回落至 %.1f°C，允许加热", heaterTemp);
    }
  } else {
    heaterForceOff = true;
  }
}

// ==================== 继电器控制 ====================
void setRelays(bool h, bool f) {
  if (heaterForceOff && h) {
    h = false;
  }
  digitalWrite(HEATER_PIN, h ? HIGH : LOW);
  digitalWrite(FAN_PIN, f ? HIGH : LOW);
  if (h != heaterRelay || f != fanRelay) {
    LOG("[继电器] 加热: %s, 风扇: %s", h ? "ON" : "OFF", f ? "ON" : "OFF");
  }
  heaterRelay = h;
  fanRelay = f;
}

// ==================== 状态机 ====================
void runStateMachine() {
  unsigned long now = millis();
  switch (systemState) {
    case STATE_STANDBY:
      setRelays(false, false);
      remainingSeconds = 0;
      break;

    case STATE_DRYING:
      {
        int heaterTarget = singleSensorMode ? targetTemp : targetTemp + heaterOffset;
        if (heaterTarget > heaterStopTemp) heaterTarget = heaterStopTemp;

        if (heaterSensorOK && !heaterForceOff) {
          if (heaterTemp < heaterTarget - HYSTERESIS) {
            setRelays(true, true);
          } else if (heaterTemp > heaterTarget + HYSTERESIS) {
            setRelays(false, true);
          }
        } else {
          setRelays(false, true);
        }

        unsigned long elapsed = (now - dryStartMillis) / 1000;
        int total = dryTime * 60;
        if (elapsed >= (unsigned long)total) {
          LOG("[状态机] 烘干完成 -> 散热");
          systemState = STATE_COOLING;
        } else {
          remainingSeconds = total - elapsed;
        }
      }
      break;

    case STATE_COOLING:
      setRelays(false, true);
      if (singleSensorMode) {
        if (heaterTemp <= COOLING_TARGET_TEMP) {
          LOG("[状态机] 散热完成（加热片温度 %.1f°C），返回待机", heaterTemp);
          setRelays(false, false);
          systemState = STATE_STANDBY;
        }
      } else {
        if (boxSensorOK && boxTemp <= COOLING_TARGET_TEMP) {
          LOG("[状态机] 散热完成，返回待机");
          setRelays(false, false);
          systemState = STATE_STANDBY;
        }
      }
      break;
  }
}

// ==================== 控制函数 ====================
void startDrying() {
  if (!singleSensorMode && (sensorFault || !boxSensorOK)) {
    LOG("[控制] 无法启动：传感器故障");
    return;
  }
  if (!heaterSensorOK) {
    LOG("[控制] 无法启动：加热片传感器故障");
    return;
  }
  if (overheatActive || heaterForceOff) {
    LOG("[控制] 无法启动：过温保护中");
    return;
  }
  systemState = STATE_DRYING;
  dryStartMillis = millis();
  remainingSeconds = dryTime * 60;
  if (singleSensorMode) {
    LOG("[控制] 开始烘干（单传感器），加热片目标: %d°C，时间: %d分钟", 
        targetTemp, dryTime);
  } else {
    LOG("[控制] 开始烘干，箱内目标: %d°C，加热片目标: %d°C，时间: %d分钟", 
        targetTemp, targetTemp + heaterOffset, dryTime);
  }
}

void stopDrying() {
  if (systemState != STATE_STANDBY) {
    LOG("[控制] 停止 -> 散热");
    systemState = STATE_COOLING;
  }
}

// ==================== 工具函数 ====================
String getStateString() {
  if (!singleSensorMode && (sensorFault || !boxSensorOK)) return "传感器故障";
  if (overheatActive) return "过热保护中";
  if (heaterForceOff) return "加热片过温";
  switch (systemState) {
    case STATE_STANDBY: return "待机";
    case STATE_DRYING:  return "烘干中";
    case STATE_COOLING: return "散热中";
  }
  return "未知";
}

String formatTime(int sec) {
  int m = sec / 60;
  int s = sec % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", m, s);
  return String(buf);
}

String escapeJson(const String& input) {
  String out = input;
  out.replace("\\", "\\\\");
  out.replace("\"", "\\\"");
  out.replace("\n", "\\n");
  out.replace("\r", "");
  return out;
}

// ==================== Web 处理函数 ====================
void handleLog() {
  String json = "{\"log\":\"" + escapeJson(logBuffer) + "\"}";
  server.send(200, "application/json", json);
}

void handleStatus() {
  String s = "{";
  if (singleSensorMode) {
    s += "\"boxTemp\":-999.0";
  } else {
    s += "\"boxTemp\":" + String(boxTemp, 1);
  }
  s += ",\"heaterTemp\":" + String(heaterTemp, 1) +
       ",\"state\":" + String((int)systemState) +
       ",\"stateStr\":\"" + getStateString() + "\"" +
       ",\"remaining\":" + String(remainingSeconds) +
       ",\"remainingStr\":\"" + formatTime(remainingSeconds) + "\"" +
       ",\"targetTemp\":" + String(targetTemp) +
       ",\"dryTime\":" + String(dryTime) +
       ",\"heater\":" + String(heaterRelay ? 1 : 0) +
       ",\"fan\":" + String(fanRelay ? 1 : 0) +
       ",\"fault\":" + String(sensorFault ? 1 : 0) +
       ",\"overheat\":" + String(overheatActive ? 1 : 0) +
       ",\"heaterForceOff\":" + String(heaterForceOff ? 1 : 0) +
       ",\"manual\":" + String(manualMode ? 1 : 0) +
       ",\"mHeat\":" + String(manualHeater ? 1 : 0) +
       ",\"mFan\":" + String(manualFan ? 1 : 0) +
       ",\"boxStopTemp\":" + String(boxStopTemp) +
       ",\"heaterStopTemp\":" + String(heaterStopTemp) +
       ",\"heaterOffset\":" + String(heaterOffset) +
       ",\"heaterFallback\":" + String(heaterFallback) +
       ",\"agentEnabled\":" + String(agentEnabled ? 1 : 0) +
       ",\"agentApiKeySet\":" + String(agentApiKey.length() > 0 ? 1 : 0) +
       ",\"singleSensor\":" + String(singleSensorMode ? 1 : 0) +
       "}";
  server.send(200, "application/json", s);
}

void handleControl() {
  bool ok = false;
  if (server.hasArg("targetTemp")) {
    targetTemp = server.arg("targetTemp").toInt();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(PREF_KEY_TARGET_TEMP, targetTemp);
    prefs.end();
    LOG("[控制] 目标温度设为 %d°C", targetTemp);
    ok = true;
  }
  if (server.hasArg("dryTime")) {
    dryTime = server.arg("dryTime").toInt();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(PREF_KEY_DRY_TIME, dryTime);
    prefs.end();
    LOG("[控制] 烘干时间设为 %d 分钟", dryTime);
    ok = true;
  }
  if (server.hasArg("boxStopTemp")) {
    boxStopTemp = server.arg("boxStopTemp").toInt();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(PREF_KEY_BOX_STOP, boxStopTemp);
    prefs.end();
    LOG("[控制] 箱内停机温度设为 %d°C (重启生效)", boxStopTemp);
    ok = true;
  }
  if (server.hasArg("heaterStopTemp")) {
    heaterStopTemp = server.arg("heaterStopTemp").toInt();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(PREF_KEY_HEATER_STOP, heaterStopTemp);
    prefs.end();
    LOG("[控制] 加热片停机温度设为 %d°C (重启生效)", heaterStopTemp);
    ok = true;
  }
  if (server.hasArg("heaterOffset")) {
    heaterOffset = server.arg("heaterOffset").toInt();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(PREF_KEY_HEATER_OFFSET, heaterOffset);
    prefs.end();
    LOG("[控制] 加热片偏移设为 %d°C (重启生效)", heaterOffset);
    ok = true;
  }
  if (server.hasArg("heaterFallback")) {
    heaterFallback = server.arg("heaterFallback").toInt();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(PREF_KEY_HEATER_FALLBACK, heaterFallback);
    prefs.end();
    LOG("[控制] 回落差值设为 %d°C (重启生效)", heaterFallback);
    ok = true;
  }
  if (server.hasArg("singleSensor")) {
    bool val = server.arg("singleSensor").toInt() ? true : false;
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putBool(PREF_KEY_SINGLE_SENSOR, val);
    prefs.end();
    LOG("[控制] 单传感器模式设为 %s (重启生效)", val ? "启用" : "禁用");
    ok = true;
  }
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    if (cmd == "start") { startDrying(); ok = true; }
    if (cmd == "stop")  { stopDrying();  ok = true; }
  }
  server.send(200, "application/json", "{\"success\":" + String(ok ? "true" : "false") + "}");
}

void handleManual() {
  if (server.hasArg("mode")) {
    manualMode = server.arg("mode").toInt() ? true : false;
    if (!manualMode) { manualHeater = false; manualFan = false; LOG("[手动] 退出手动控制"); }
    else { LOG("[手动] 开启手动控制"); }
  }
  if (manualMode) {
    if (server.hasArg("heater")) {
      manualHeater = server.arg("heater").toInt() ? true : false;
      LOG("[手动] 加热: %s", manualHeater ? "ON" : "OFF");
    }
    if (server.hasArg("fan")) {
      manualFan = server.arg("fan").toInt() ? true : false;
      LOG("[手动] 风扇: %s", manualFan ? "ON" : "OFF");
    }
  }
  String s = "{\"manual\":" + String(manualMode ? 1 : 0) +
             ",\"mHeat\":" + String(manualHeater ? 1 : 0) +
             ",\"mFan\":" + String(manualFan ? 1 : 0) + "}";
  server.send(200, "application/json", s);
}

void handleAgentToggle() {
  if (server.hasArg("enable")) {
    agentEnabled = server.arg("enable").toInt() ? true : false;
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putBool(PREF_KEY_AGENT_ENABLED, agentEnabled);
    prefs.end();
    if (agentEnabled) {
      if (WiFi.status() == WL_CONNECTED) {
        tcpServer.begin();
        LOG("[Agent] 服务已启用");
      } else {
        LOG("[Agent] WiFi未连接，无法启动Agent服务器");
      }
    } else {
      if (tcpClient) tcpClient.stop();
      tcpServer.stop();
      LOG("[Agent] 服务已禁用");
    }
  }
  server.send(200, "application/json", "{\"agentEnabled\":" + String(agentEnabled ? 1 : 0) + "}");
}

void handleAgentApiKey() {
  if (server.hasArg("apikey")) {
    agentApiKey = server.arg("apikey");
    agentApiKey.trim();
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putString(PREF_KEY_AGENT_APIKEY, agentApiKey);
    prefs.end();
    LOG("[Agent] API Key 已更新");
  }
  server.send(200, "application/json", "{\"success\":true}");
}

void handleRestart() {
  LOG("[系统] 收到软重启请求");
  server.send(200, "text/plain", "OK");
  delay(100);
  ESP.restart();
}

// ==================== Agent TCP 处理 ====================
void processAgentCommand(String cmdLine) {
  cmdLine.trim();
  if (cmdLine.length() == 0) return;

  int spaceIdx = cmdLine.indexOf(' ');
  String cmd = (spaceIdx > 0) ? cmdLine.substring(0, spaceIdx) : cmdLine;
  cmd.toUpperCase();
  String args = (spaceIdx > 0) ? cmdLine.substring(spaceIdx + 1) : "";

  if (cmd == "STATUS") {
    String json = "{";
    if (singleSensorMode) json += "\"boxTemp\":-999.0";
    else json += "\"boxTemp\":" + String(boxTemp, 1);
    json += ",\"heaterTemp\":" + String(heaterTemp, 1) +
            ",\"state\":\"" + getStateString() + "\"" +
            ",\"targetTemp\":" + String(targetTemp) +
            ",\"dryTime\":" + String(dryTime) +
            ",\"remaining\":" + String(remainingSeconds) +
            ",\"heater\":" + String(heaterRelay ? 1 : 0) +
            ",\"fan\":" + String(fanRelay ? 1 : 0) +
            ",\"fault\":" + String(sensorFault ? 1 : 0) +
            ",\"manual\":" + String(manualMode ? 1 : 0) +
            ",\"singleSensor\":" + String(singleSensorMode ? 1 : 0) + "}";
    tcpClient.println(json);
  } else if (cmd == "START") {
    if (!singleSensorMode && (sensorFault || !boxSensorOK)) {
      tcpClient.println("ERR: sensor fault");
    } else if (!heaterSensorOK) {
      tcpClient.println("ERR: heater sensor fault");
    } else if (overheatActive || heaterForceOff) {
      tcpClient.println("ERR: overheat");
    } else {
      startDrying();
      tcpClient.println("OK");
    }
  } else if (cmd == "STOP") {
    stopDrying();
    tcpClient.println("OK");
  } else if (cmd == "SETTEMP") {
    int val = args.toInt();
    if (val >= 0 && val <= 100) {
      targetTemp = val;
      prefs.begin(PREF_NAMESPACE, false);
      prefs.putInt(PREF_KEY_TARGET_TEMP, targetTemp);
      prefs.end();
      LOG("[Agent] 设置目标温度: %d°C", targetTemp);
      tcpClient.println("OK");
    } else {
      tcpClient.println("ERR: invalid value");
    }
  } else if (cmd == "SETTIME") {
    int val = args.toInt();
    if (val >= 1 && val <= 999) {
      dryTime = val;
      prefs.begin(PREF_NAMESPACE, false);
      prefs.putInt(PREF_KEY_DRY_TIME, dryTime);
      prefs.end();
      LOG("[Agent] 设置烘干时间: %d 分钟", dryTime);
      tcpClient.println("OK");
    } else {
      tcpClient.println("ERR: invalid value");
    }
  } else if (cmd == "MANUAL") {
    if (args.equalsIgnoreCase("ON")) {
      manualMode = true;
      LOG("[Agent] 开启手动模式");
    } else if (args.equalsIgnoreCase("OFF")) {
      manualMode = false;
      manualHeater = false;
      manualFan = false;
      LOG("[Agent] 关闭手动模式");
    }
    tcpClient.println("OK");
  } else if (cmd == "HEATER") {
    if (manualMode) {
      if (args.equalsIgnoreCase("ON")) { manualHeater = true; LOG("[Agent] 手动加热 ON"); }
      else if (args.equalsIgnoreCase("OFF")) { manualHeater = false; LOG("[Agent] 手动加热 OFF"); }
      tcpClient.println("OK");
    } else {
      tcpClient.println("ERR: not in manual mode");
    }
  } else if (cmd == "FAN") {
    if (manualMode) {
      if (args.equalsIgnoreCase("ON")) { manualFan = true; LOG("[Agent] 手动风扇 ON"); }
      else if (args.equalsIgnoreCase("OFF")) { manualFan = false; LOG("[Agent] 手动风扇 OFF"); }
      tcpClient.println("OK");
    } else {
      tcpClient.println("ERR: not in manual mode");
    }
  } else if (cmd == "SINGLESENSOR") {
    if (args.equalsIgnoreCase("ON")) {
      prefs.begin(PREF_NAMESPACE, false);
      prefs.putBool(PREF_KEY_SINGLE_SENSOR, true);
      prefs.end();
      LOG("[Agent] 单传感器模式设为 启用 (重启生效)");
      tcpClient.println("OK (reboot required)");
    } else if (args.equalsIgnoreCase("OFF")) {
      prefs.begin(PREF_NAMESPACE, false);
      prefs.putBool(PREF_KEY_SINGLE_SENSOR, false);
      prefs.end();
      LOG("[Agent] 单传感器模式设为 禁用 (重启生效)");
      tcpClient.println("OK (reboot required)");
    } else {
      tcpClient.println("ERR: usage SINGLESENSOR ON/OFF");
    }
  } else if (cmd == "RESTART") {
    tcpClient.println("OK");
    LOG("[Agent] 收到重启命令");
    delay(100);
    ESP.restart();
  } else if (cmd == "AUTH") {
    tcpClient.println("OK");
  } else {
    tcpClient.println("ERR: unknown command");
  }
}

void handleAgentClient() {
  if (!tcpClient.connected()) {
    tcpClient.stop();
    agentConnected = false;
    LOG("[Agent] 客户端断开");
    return;
  }

  while (tcpClient.available()) {
    char c = tcpClient.read();
    if (c == '\n') {
      processAgentCommand(agentInputBuffer);
      agentInputBuffer = "";
    } else if (c != '\r') {
      agentInputBuffer += c;
    }
  }
}

// ==================== 主页面 ====================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0\"><title>烘干箱</title><style>";
  html += "*{box-sizing:border-box;margin:0;padding:0}body{font-family:-apple-system,sans-serif;background:#f0f2f5;padding:16px}.c{max-width:480px;margin:0 auto}.card{background:#fff;border-radius:16px;padding:20px;margin-bottom:12px}";
  html += ".t{font-size:56px;font-weight:bold;text-align:center}.t2{font-size:40px;font-weight:bold;text-align:center;color:#ff6600}.s{text-align:center;font-size:18px;margin:8px 0}.r{text-align:center;font-size:32px;font-weight:bold}.l{font-size:13px;color:#999}";
  html += ".row{display:flex;gap:12px}.col{flex:1}.inp{width:100%;padding:12px;border:2px solid #ddd;border-radius:10px;font-size:16px}";
  html += ".btn{width:100%;padding:14px;border:none;border-radius:12px;font-size:17px;font-weight:600;cursor:pointer;margin:6px 0}";
  html += ".g{background:#4CAF50;color:#fff}.r2{background:#f44336;color:#fff}.b{background:#2196F3;color:#fff}";
  html += ".h{display:none}.al{text-align:center;margin-top:12px}.al a{color:#f44336;text-decoration:none}";
  html += ".sw{position:relative;display:inline-block;width:48px;height:26px;vertical-align:middle;margin-right:8px}";
  html += ".sw input{opacity:0;width:0;height:0}";
  html += ".sl{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#ccc;border-radius:26px;transition:.3s}";
  html += ".sl:before{position:absolute;content:'';height:20px;width:20px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.3s}";
  html += ".sw input:checked+.sl{background:#2196F3}";
  html += ".sw input:checked+.sl:before{transform:translateX(22px)}";
  html += "@media (max-width:768px){.t2{font-size:40px;display:block}.heater-row{display:block}}";
  html += "@media (min-width:769px){.temps-row{display:flex;gap:12px}.temps-row .col{flex:1;text-align:center}}";
  html += "@media (max-width:768px){.log-panel{display:none}}";
  html += "</style></head><body><div class=\"c\">";
  
  html += "<div class=\"card\"><div class=\"temps-row\"><div class=\"col\"><div class=\"t\"><span id=\"bt\">--</span>&deg;C</div><div class=\"l\">箱内温度</div></div><div class=\"col\"><div class=\"t2\" id=\"ht2\"><span id=\"ht\">--</span>&deg;C</div><div class=\"l\">加热片温度</div></div></div>";
  html += "<div class=\"s\"><span id=\"s\">连接中...</span></div>";
  html += "<div class=\"l\" style=\"margin-top:8px\">剩余时间</div><div class=\"r\" id=\"r\">--:--</div>";
  html += "<div style=\"margin-top:8px;font-size:14px;color:#666\">目标 <span id=\"stTarget\">--</span>&deg;C · 设定 <span id=\"stTime\">--</span>分 · 剩余 <span id=\"stRemain\">--</span><br><span id=\"stTask\">待机</span> · <span id=\"h\">加热 OFF</span> · <span id=\"f\">风扇 OFF</span></div></div>";
  
  html += "<div class=\"card\"><div class=\"row\"><div class=\"col\"><div class=\"l\">温度 (&deg;C)</div><input class=\"inp\" type=\"number\" id=\"tt\" min=\"0\" max=\"100\" onchange=\"saveTemp()\"></div>";
  html += "<div class=\"col\"><div class=\"l\">时间 (分钟)</div><input class=\"inp\" type=\"number\" id=\"dt\" min=\"1\" max=\"999\" onchange=\"saveTime()\"></div></div>";
  html += "<button class=\"btn g\" id=\"bs\" onclick=\"c('start')\">开始</button><button class=\"btn r2 h\" id=\"bp\" onclick=\"c('stop')\">停止</button></div><div id=\"b\"></div>";
  
  html += "<div class=\"card\"><div class=\"l\">手动控制</div><div style=\"margin:12px 0\">";
  html += "<label class=\"sw\"><input type=\"checkbox\" id=\"mm\"><span class=\"sl\"></span></label><label style=\"font-size:14px;color:#666\">无视传感器故障</label></div>";
  html += "<div id=\"mc\" class=\"h\"><button class=\"btn g\" id=\"mh\" onclick=\"mh(1)\">加热 ON</button><button class=\"btn r2 h\" id=\"mhf\" onclick=\"mh(0)\">加热 OFF</button>";
  html += "<button class=\"btn b\" id=\"mf\" onclick=\"mf(1)\">风扇 ON</button><button class=\"btn r2 h\" id=\"mff\" onclick=\"mf(0)\">风扇 OFF</button></div></div>";
  
  html += "<div class=\"card\"><div class=\"l\" style=\"font-weight:bold;margin-bottom:8px\">高级设置 (重启生效)</div>";
  html += "<div class=\"row\"><div class=\"col\"><div class=\"l\">箱内停机 (°C)</div><input class=\"inp\" type=\"number\" id=\"bst\" min=\"0\" max=\"85\" onchange=\"saveBoxStop()\"></div>";
  html += "<div class=\"col\"><div class=\"l\">加热片停机 (°C)</div><input class=\"inp\" type=\"number\" id=\"hst\" min=\"0\" max=\"120\" onchange=\"saveHeaterStop()\"></div></div>";
  html += "<div class=\"row\"><div class=\"col\"><div class=\"l\">超前偏移 (°C)</div><input class=\"inp\" type=\"number\" id=\"hoff\" min=\"0\" max=\"50\" onchange=\"saveHeaterOffset()\"></div>";
  html += "<div class=\"col\"><div class=\"l\">回落差值 (°C)</div><input class=\"inp\" type=\"number\" id=\"hfb\" min=\"5\" max=\"50\" onchange=\"saveHeaterFallback()\"></div></div>";
  html += "<div style=\"margin-top:12px\"><label class=\"sw\"><input type=\"checkbox\" id=\"ae\" onchange=\"toggleAgent()\"><span class=\"sl\"></span></label><label style=\"font-size:14px;color:#666\">启用 Agent (端口8080)</label></div>";
  html += "<div style=\"margin-top:12px\"><label class=\"sw\"><input type=\"checkbox\" id=\"ssm\" onchange=\"saveSingleSensor()\"><span class=\"sl\"></span></label><label style=\"font-size:14px;color:#666\">单传感器模式（重启生效）</label></div>";
  html += "<div style=\"margin-top:8px\"><input class=\"inp\" type=\"text\" id=\"apikey\" placeholder=\"API Key (留空不验证)\" onchange=\"saveApiKey()\"></div>";
  html += "</div>";
  
  html += "<div class=\"log-panel\"><div class=\"card\"><div class=\"l\" style=\"margin-bottom:8px\">🔍 系统日志</div>";
  html += "<pre id=\"logBox\" style=\"background:#1e1e1e;color:#d4d4d4;padding:12px;border-radius:8px;font-size:12px;max-height:200px;overflow-y:auto;white-space:pre-wrap;word-break:break-all\"></pre></div></div>";
  html += "<div class=\"al\"><a href=\"#\" onclick=\"restartDevice()\">重启设备</a></div></div>";
  
  html += "<script>";
  html += "var _ignoreTempUpdate=false; var _ignoreTimeUpdate=false;";
  html += "var mm=document.getElementById('mm');";
  html += "mm.onchange=function(){fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'mode='+(mm.checked?1:0)}).then(function(r){return r.json()}).then(function(){p()}).catch(function(){})};";
  html += "function mh(v){fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'mode=1&heater='+v}).then(function(r){return r.json()}).then(function(){p()}).catch(function(){})}";
  html += "function mf(v){fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'mode=1&fan='+v}).then(function(r){return r.json()}).then(function(){p()}).catch(function(){})}";

  html += "function saveTemp(){";
  html += "var v=document.getElementById('tt').value; _ignoreTempUpdate=true;";
  html += "fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'targetTemp='+v})";
  html += ".then(function(r){return r.json()}).then(function(d){_ignoreTempUpdate=false;if(d.success)p();}).catch(function(){_ignoreTempUpdate=false;});";
  html += "}";

  html += "function saveTime(){";
  html += "var v=document.getElementById('dt').value; _ignoreTimeUpdate=true;";
  html += "fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'dryTime='+v})";
  html += ".then(function(r){return r.json()}).then(function(d){_ignoreTimeUpdate=false;if(d.success)p();}).catch(function(){_ignoreTimeUpdate=false;});";
  html += "}";

  html += "function saveBoxStop(){var v=document.getElementById('bst').value;fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'boxStopTemp='+v}).then(function(){alert('已保存，重启生效')}).catch(function(){})}";
  html += "function saveHeaterStop(){var v=document.getElementById('hst').value;fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'heaterStopTemp='+v}).then(function(){alert('已保存，重启生效')}).catch(function(){})}";
  html += "function saveHeaterOffset(){var v=document.getElementById('hoff').value;fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'heaterOffset='+v}).then(function(){alert('已保存，重启生效')}).catch(function(){})}";
  html += "function saveHeaterFallback(){var v=document.getElementById('hfb').value;fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'heaterFallback='+v}).then(function(){alert('已保存，重启生效')}).catch(function(){})}";
  html += "function toggleAgent(){var v=document.getElementById('ae').checked?1:0;fetch('/api/agent/toggle',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'enable='+v}).then(function(){p()}).catch(function(){})}";
  html += "function saveApiKey(){var v=document.getElementById('apikey').value;fetch('/api/agent/apikey',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'apikey='+encodeURIComponent(v)}).then(function(){alert('API Key 已保存')}).catch(function(){})}";

  html += "function saveSingleSensor(){var v=document.getElementById('ssm').checked?1:0;fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'singleSensor='+v}).then(function(){alert('已保存，重启生效')}).catch(function(){alert('保存失败')});}";

  html += "function u(d){try{";
  html += "if(d.singleSensor){document.getElementById('bt').textContent='忽略';}else{document.getElementById('bt').textContent=d.boxTemp.toFixed(1);}";
  html += "document.getElementById('ht').textContent=d.heaterTemp.toFixed(1);";
  html += "document.getElementById('s').textContent=d.stateStr+(d.fault?' 故障':'')+(d.overheat?' 过热':'');";
  html += "document.getElementById('r').textContent=d.remainingStr;";
  html += "document.getElementById('stTarget').textContent=d.targetTemp;";
  html += "document.getElementById('stTime').textContent=d.dryTime;";
  html += "document.getElementById('stRemain').textContent=d.remainingStr;";
  html += "document.getElementById('stTask').textContent=d.stateStr;";
  html += "document.getElementById('h').textContent=d.heater?'加热 ON':'加热 OFF';";
  html += "document.getElementById('f').textContent=d.fan?'风扇 ON':'风扇 OFF';";
  html += "if(!_ignoreTempUpdate && document.activeElement!==document.getElementById('tt')) document.getElementById('tt').value=d.targetTemp;";
  html += "if(!_ignoreTimeUpdate && document.activeElement!==document.getElementById('dt')) document.getElementById('dt').value=d.dryTime;";
  // 高级设置输入框仅在失焦时更新
  html += "if(document.activeElement!==document.getElementById('bst')) document.getElementById('bst').value=d.boxStopTemp;";
  html += "if(document.activeElement!==document.getElementById('hst')) document.getElementById('hst').value=d.heaterStopTemp;";
  html += "if(document.activeElement!==document.getElementById('hoff')) document.getElementById('hoff').value=d.heaterOffset;";
  html += "if(document.activeElement!==document.getElementById('hfb')) document.getElementById('hfb').value=d.heaterFallback;";
  // API Key 仅在失焦时更新掩码
  html += "if(document.activeElement!==document.getElementById('apikey')) document.getElementById('apikey').value=d.agentApiKeySet?'***':'';";
  html += "document.getElementById('ae').checked=d.agentEnabled;";
  html += "document.getElementById('ssm').checked = d.singleSensor ? true : false;";
  html += "if(d.state===0||d.fault||d.overheat){document.getElementById('bs').classList.remove('h');document.getElementById('bp').classList.add('h')}";
  html += "else{document.getElementById('bs').classList.add('h');document.getElementById('bp').classList.remove('h')}";
  html += "var w='';if(d.fault)w='<div style=\"background:#f8d7da;padding:10px;border-radius:8px;margin-bottom:12px\">传感器故障</div>';";
  html += "if(d.overheat)w='<div style=\"background:#f44336;color:#fff;padding:10px;border-radius:8px;margin-bottom:12px;font-weight:bold\">&#x26A0; 过热保护中</div>';";
  html += "document.getElementById('b').innerHTML=w;";
  html += "mm.checked=d.manual?true:false;";
  html += "if(d.manual){document.getElementById('mc').classList.remove('h')}else{document.getElementById('mc').classList.add('h')}";
  html += "var x=document.getElementById('mh'),y=document.getElementById('mhf'),z=document.getElementById('mf'),w=document.getElementById('mff');";
  html += "if(d.manual){if(d.mHeat){x.classList.add('h');y.classList.remove('h')}else{x.classList.remove('h');y.classList.add('h')}";
  html += "if(d.mFan){z.classList.add('h');w.classList.remove('h')}else{z.classList.remove('h');w.classList.add('h')}}";
  html += "}catch(e){}}";

  html += "function p(){fetch('/api/status').then(function(r){return r.json()}).then(function(d){u(d)}).catch(function(){})}";
  html += "function c(m){fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd='+m+'&targetTemp='+document.getElementById('tt').value+'&dryTime='+document.getElementById('dt').value}).then(function(r){return r.json()}).then(function(d){if(d.success)p()}).catch(function(){})}";
  html += "function updateLogs(){fetch('/api/log').then(function(r){return r.json()}).then(function(d){document.getElementById('logBox').textContent=d.log;}).catch(function(){})}";
  html += "function restartDevice(){if(confirm('确认重启设备？')){fetch('/api/restart',{method:'POST'}).then(function(){alert('重启中...');}).catch(function(){alert('重启请求失败');})}}";
  html += "setInterval(p,1000);p();setInterval(updateLogs,1000);updateLogs();</script></body></html>";

  server.send(200, "text/html; charset=utf-8", html);
}

// ==================== 初始化 ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  LOG("--- 系统启动 ---");

  initHardware();
  loadParameters();

  if (singleSensorMode) {
    boxSensorOK = true;
    boxTemp = -999.0;
    sensorFailCount = 0;
    sensorFault = false;
    overheatActive = false;
  }

  LOG("正在连接 WiFi: %s ...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    LOG("\nWiFi 连接成功！");
    LOG("IP 地址: %s", WiFi.localIP().toString().c_str());
    if (agentEnabled) {
      tcpServer.begin();
      LOG("[Agent] TCP服务器已启动 (端口8080)");
    }
  } else {
    LOG("\nWiFi 连接失败！");
  }

  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/control", HTTP_POST, handleControl);
  server.on("/api/manual", HTTP_POST, handleManual);
  server.on("/api/log", handleLog);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/agent/toggle", HTTP_POST, handleAgentToggle);
  server.on("/api/agent/apikey", HTTP_POST, handleAgentApiKey);
  server.begin();

  LOG("[系统] 初始化完成");
}

// ==================== 主循环 ====================
void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastSensorRead >= 1000) {
    if (!tempReqHeater) readHeaterTemperature();
    if (!tempReqBox) readBoxTemperature();
    lastSensorRead = now;
  }
  if (tempReqHeater) readHeaterTemperature();
  if (tempReqBox) readBoxTemperature();

  checkSensorFault();
  checkOverheatProtection();

  if (overheatActive) {
    setRelays(false, true);
  } else if (manualMode) {
    setRelays(manualHeater, manualFan);
  } else if (!sensorFault && boxSensorOK && heaterSensorOK) {
    runStateMachine();
  } else {
    setRelays(false, fanRelay);
  }

  if (agentEnabled) {
    if (!tcpClient || !tcpClient.connected()) {
      tcpClient = tcpServer.accept();
      if (tcpClient) {
        agentConnected = true;
        agentInputBuffer = "";
        LOG("[Agent] 客户端已连接");
      }
    }
    if (agentConnected && tcpClient.connected()) {
      handleAgentClient();
    }
  }
}