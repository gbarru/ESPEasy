
#include <Arduino.h>

#ifdef CONTINUOUS_INTEGRATION
#pragma GCC diagnostic error "-Wall"
#else
#pragma GCC diagnostic warning "-Wall"
#endif


#ifdef PLUGIN_SET_GENERIC_ESP32
  #ifndef ESP32
    #define ESP32
  #endif
#endif
# 86 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy.ino"
#include "ESPEasy-Globals.h"

#include "_Plugin_Helper.h"

#include "_CPlugin_Helper.h"


boolean Blynk_get(const String& command, byte controllerIndex,float *data = NULL );

int firstEnabledBlynkController() {
  for (byte i = 0; i < CONTROLLER_MAX; ++i) {
    byte ProtocolIndex = getProtocolIndex(Settings.Protocol[i]);
    if (Protocol[ProtocolIndex].Number == 12 && Settings.ControllerEnabled[i]) {
      return i;
    }
  }
  return -1;
}



#ifdef CORE_POST_2_5_0



void preinit() {




  ESP8266WiFiClass::preinitWiFiOff();
}
#endif




void sw_watchdog_callback(void *arg)
{
  yield();
  ++sw_watchdog_callback_count;
}




void setup()
{
#ifdef ESP8266_DISABLE_EXTRA4K
  disable_extra4k_at_link_time();
#endif
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_OFF);
  run_compiletime_checks();
  lowestFreeStack = getFreeStackWatermark();
  lowestRAM = FreeMem();
#ifndef ESP32
  ets_isr_attach(8, sw_watchdog_callback, NULL);
#endif

  resetPluginTaskData();
  Plugin_id.resize(PLUGIN_MAX);
  Task_id_to_Plugin_id.resize(TASKS_MAX);

  checkRAM(F("setup"));
  #if defined(ESP32)
    for(byte x = 0; x < 16; x++)
      ledChannelPin[x] = -1;
  #endif

  Serial.begin(115200);


  initLog();

#if defined(ESP32)
  WiFi.onEvent(WiFiEvent);
#else

  stationConnectedHandler = WiFi.onStationModeConnected(onConnected);
 stationDisconnectedHandler = WiFi.onStationModeDisconnected(onDisconnect);
 stationGotIpHandler = WiFi.onStationModeGotIP(onGotIP);
  stationModeDHCPTimeoutHandler = WiFi.onStationModeDHCPTimeout(onDHCPTimeout);
  APModeStationConnectedHandler = WiFi.onSoftAPModeStationConnected(onConnectedAPmode);
  APModeStationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(onDisonnectedAPmode);
#endif

  if (SpiffsSectors() < 32)
  {
    serialPrintln(F("\nNo (or too small) SPIFFS area..\nSystem Halted\nPlease reflash with 128k SPIFFS minimum!"));
    while (true)
      delay(1);
  }

  emergencyReset();

  String log = F("\n\n\rINIT : Booting version: ");
  log += F(BUILD_GIT);
  log += " (";
  log += getSystemLibraryString();
  log += ')';
  addLog(LOG_LEVEL_INFO, log);
  log = F("INIT : Free RAM:");
  log += FreeMem();
  addLog(LOG_LEVEL_INFO, log);



  if (readFromRTC())
  {
    RTC.bootFailedCount++;
    RTC.bootCounter++;
    lastMixedSchedulerId_beforereboot = RTC.lastMixedSchedulerId;
    readUserVarFromRTC();

    if (RTC.deepSleepState == 1)
    {
      log = F("INIT : Rebooted from deepsleep #");
      lastBootCause=BOOT_CAUSE_DEEP_SLEEP;
    }
    else
      log = F("INIT : Warm boot #");

    log += RTC.bootCounter;
    log += F(" Last Task: ");
    log += decodeSchedulerId(lastMixedSchedulerId_beforereboot);
  }

  else
  {
    initRTC();


    if (lastBootCause == BOOT_CAUSE_MANUAL_REBOOT)
      lastBootCause = BOOT_CAUSE_COLD_BOOT;
    log = F("INIT : Cold Boot");
  }
  log += F(" - Restart Reason: ");
  log += getResetReasonString();

  RTC.deepSleepState=0;
  saveToRTC();

  addLog(LOG_LEVEL_INFO, log);

  fileSystemCheck();
  progMemMD5check();
  LoadSettings();
  Settings.UseRTOSMultitasking = false;
  if (RTC.bootFailedCount > 10 && RTC.bootCounter > 10) {
    byte toDisable = RTC.bootFailedCount - 10;
    toDisable = disablePlugin(toDisable);
    if (toDisable != 0) {
      toDisable = disableController(toDisable);
    }
    if (toDisable != 0) {
      toDisable = disableNotification(toDisable);
    }
  }


  checkRuleSets();



  if (Settings.Version != VERSION || Settings.PID != ESP_PROJECT_PID)
  {

    serialPrint(F("\nPID:"));
    serialPrintln(String(Settings.PID));
    serialPrint(F("Version:"));
    serialPrintln(String(Settings.Version));
    serialPrintln(F("INIT : Incorrect PID or version!"));
    delay(1000);
    ResetFactory();
  }

  if (Settings.UseSerial)
  {

    Serial.flush();
    Serial.begin(Settings.BaudRate);

  }

  if (Settings.Build != BUILD)
    BuildFixes();


  log = F("INIT : Free RAM:");
  log += FreeMem();
  addLog(LOG_LEVEL_INFO, log);

  if (Settings.UseSerial && Settings.SerialLogLevel >= LOG_LEVEL_DEBUG_MORE)
    Serial.setDebugOutput(true);

  checkRAM(F("hardwareInit"));
  hardwareInit();

  timermqtt_interval = 250;
  timerAwakeFromDeepSleep = millis();
  if (Settings.UseRules && isDeepSleepEnabled())
  {
    String event = F("System#NoSleep=");
    event += Settings.deepSleep;
    rulesProcessing(event);
  }

  PluginInit();
  CPluginInit();
  NPluginInit();
  log = F("INFO : Plugins: ");
  log += deviceCount + 1;
  log += getPluginDescriptionString();
  log += " (";
  log += getSystemLibraryString();
  log += ')';
  addLog(LOG_LEVEL_INFO, log);

  if (deviceCount + 1 >= PLUGIN_MAX) {
    addLog(LOG_LEVEL_ERROR, F("Programming error! - Increase PLUGIN_MAX"));
  }

  if (Settings.UseRules)
  {
    String event = F("System#Wake");
    rulesProcessing(event);
  }

  if (!selectValidWiFiSettings()) {
    wifiSetup = true;
  }
# 327 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy.ino"
  WiFiConnectRelaxed();

  #ifdef FEATURE_REPORTING
  ReportStatus();
  #endif

  #ifdef FEATURE_ARDUINO_OTA
  ArduinoOTAInit();
  #endif


  if (Settings.UDPPort != 0)
    portUDP.begin(Settings.UDPPort);

  sendSysInfoUDP(3);

  if (systemTimePresent())
    initTime();

#if FEATURE_ADC_VCC
  vcc = ESP.getVcc() / 1000.0;
#endif

  if (Settings.UseRules)
  {
    String event = F("System#Boot");
    rulesProcessing(event);
  }

  writeDefaultCSS();

  UseRTOSMultitasking = Settings.UseRTOSMultitasking;
  #ifdef USE_RTOS_MULTITASKING
    if(UseRTOSMultitasking){
      log = F("RTOS : Launching tasks");
      addLog(LOG_LEVEL_INFO, log);
      xTaskCreatePinnedToCore(RTOS_TaskServers, "RTOS_TaskServers", 16384, NULL, 1, NULL, 1);
      xTaskCreatePinnedToCore(RTOS_TaskSerial, "RTOS_TaskSerial", 8192, NULL, 1, NULL, 1);
      xTaskCreatePinnedToCore(RTOS_Task10ps, "RTOS_Task10ps", 8192, NULL, 1, NULL, 1);
      xTaskCreatePinnedToCore(
                    RTOS_HandleSchedule,
                    "RTOS_HandleSchedule",
                    16384,
                    NULL,
                    1,
                    NULL,
                    1);
    }
  #endif
# 384 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy.ino"
  setIntervalTimerOverride(TIMER_20MSEC, 5);
  setIntervalTimerOverride(TIMER_100MSEC, 66);
  setIntervalTimerOverride(TIMER_1SEC, 777);
  setIntervalTimerOverride(TIMER_30SEC, 1333);
  setIntervalTimerOverride(TIMER_MQTT, 88);
  setIntervalTimerOverride(TIMER_STATISTICS, 2222);
}

#ifdef USE_RTOS_MULTITASKING
void RTOS_TaskServers( void * parameter )
{
 while (true){
  delay(100);
  WebServer.handleClient();
  checkUDP();
 }
}

void RTOS_TaskSerial( void * parameter )
{
 while (true){
    delay(100);
    if (Settings.UseSerial)
    if (Serial.available())
      if (!PluginCall(PLUGIN_SERIAL_IN, 0, dummyString))
        serial();
 }
}

void RTOS_Task10ps( void * parameter )
{
 while (true){
    delay(100);
    run10TimesPerSecond();
 }
}

void RTOS_HandleSchedule( void * parameter )
{
 while (true){
    handle_schedule();
 }
}

#endif

int firstEnabledMQTTController() {
  for (byte i = 0; i < CONTROLLER_MAX; ++i) {
    byte ProtocolIndex = getProtocolIndex(Settings.Protocol[i]);
    if (Protocol[ProtocolIndex].usesMQTT && Settings.ControllerEnabled[i]) {
      return i;
    }
  }
  return -1;
}

bool getControllerProtocolDisplayName(byte ProtocolIndex, byte parameterIdx, String& protoDisplayName) {
  EventStruct tmpEvent;
  tmpEvent.idx=parameterIdx;
  return CPluginCall(ProtocolIndex, CPLUGIN_GET_PROTOCOL_DISPLAY_NAME, &tmpEvent, protoDisplayName);
}

void updateLoopStats() {
  ++loopCounter;
  ++loopCounter_full;
  if (lastLoopStart == 0) {
    lastLoopStart = micros();
    return;
  }
  const long usecSince = usecPassedSince(lastLoopStart);
  miscStats[LOOP_STATS].add(usecSince);

  loop_usec_duration_total += usecSince;
  lastLoopStart = micros();
  if (usecSince <= 0 || usecSince > 10000000)
    return;
  if (shortestLoop > static_cast<unsigned long>(usecSince)) {
    shortestLoop = usecSince;
    loopCounterMax = 30 * 1000000 / usecSince;
  }
  if (longestLoop < static_cast<unsigned long>(usecSince))
    longestLoop = usecSince;
}

void updateLoopStats_30sec(byte loglevel) {
  loopCounterLast = loopCounter;
  loopCounter = 0;
  if (loopCounterLast > loopCounterMax)
    loopCounterMax = loopCounterLast;

  msecTimerHandler.updateIdleTimeStats();

#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(loglevel)) {
    String log = F("LoopStats: shortestLoop: ");
    log += shortestLoop;
    log += F(" longestLoop: ");
    log += longestLoop;
    log += F(" avgLoopDuration: ");
    log += loop_usec_duration_total / loopCounter_full;
    log += F(" loopCounterMax: ");
    log += loopCounterMax;
    log += F(" loopCounterLast: ");
    log += loopCounterLast;
    log += F(" countFindPluginId: ");
    log += countFindPluginId;
    addLog(loglevel, log);
  }
#endif
  countFindPluginId = 0;
  loop_usec_duration_total = 0;
  loopCounter_full = 1;
}

float getCPUload() {
  return 100.0 - msecTimerHandler.getIdleTimePct();
}

int getLoopCountPerSec() {
  return loopCounterLast / 30;
}







void loop()
{





  dummyString = String();

  updateLoopStats();

  if (wifiSetupConnect)
  {

    WiFiConnectRelaxed();
    wifiSetupConnect = false;
  }
  if (WiFi.status() == WL_DISCONNECTED) {
    delay(100);
  }
  if ((wifiStatus != ESPEASY_WIFI_SERVICES_INITIALIZED) || unprocessedWifiEvents()) {


    delay(1);
    if (!processedConnect) {
      addLog(LOG_LEVEL_INFO, F("WIFI : Entering processConnect()"));
      processConnect();
    }
    if (hasIPaddr() || !processedGetIP) {
      addLog(LOG_LEVEL_INFO, F("WIFI : Entering processGotIP()"));
      processGotIP();
    }
    if (!processedDisconnect) {
      addLog(LOG_LEVEL_INFO, F("WIFI : Entering processDisconnect()"));
      processDisconnect();
    }
    if (!processedDHCPTimeout) {
      addLog(LOG_LEVEL_INFO, F("WIFI : DHCP timeout, Calling disconnect()"));
      processedDHCPTimeout = true;
      processDisconnect();
    }
    if ((wifiStatus & ESPEASY_WIFI_GOT_IP) && (wifiStatus & ESPEASY_WIFI_CONNECTED)) {
      wifiStatus = ESPEASY_WIFI_SERVICES_INITIALIZED;
    }
  } else if (!WiFiConnected()) {




    String wifilog = F("WIFI : Wifi status out sync WiFi.status() = ");
    wifilog += String(WiFi.status());

    addLog(LOG_LEVEL_INFO, wifilog);
  }
  if (wifiStatus == ESPEASY_WIFI_DISCONNECTED) {
    String wifilog = F("WIFI : Disconnected: WiFi.status() = ");
    wifilog += String(WiFi.status());

    addLog(LOG_LEVEL_INFO, wifilog);
    delay(100);
  }
  if (!processedConnectAPmode) processConnectAPmode();
  if (!processedDisconnectAPmode) processDisconnectAPmode();
  if (!processedScanDone) processScanDone();

  bool firstLoopConnectionsEstablished = checkConnectionsEstablished() && firstLoop;
  if (firstLoopConnectionsEstablished) {
     addLog(LOG_LEVEL_INFO, F("firstLoopConnectionsEstablished"));
     firstLoop = false;
     timerAwakeFromDeepSleep = millis();

     if (Settings.UseRules && isDeepSleepEnabled())
     {
        String event = F("System#NoSleep=");
        event += Settings.deepSleep;
        rulesProcessing(event);
     }


     RTC.bootFailedCount = 0;
     saveToRTC();
  }


  if ((firstLoopConnectionsEstablished || readyForSleep()) && isDeepSleepEnabled())
  {
      runPeriodicalMQTT();

      run50TimesPerSecond();
      run10TimesPerSecond();
      runEach30Seconds();
      runOncePerSecond();
  }

  else
  {
    if (!UseRTOSMultitasking) {

      handle_schedule();
    }
  }

  backgroundtasks();

  if (readyForSleep()){
    if (Settings.UseRules)
    {
      String event = F("System#Sleep");
      rulesProcessing(event);
    }

    runPeriodicalMQTT();
    flushAndDisconnectAllClients();

    SPIFFS.end();

    deepSleep(Settings.Delay);

  }
}

bool checkConnectionsEstablished() {
  if (wifiStatus != ESPEASY_WIFI_SERVICES_INITIALIZED) return false;
  if (!processedConnect) return false;






  return true;
}

void flushAndDisconnectAllClients() {
  if (anyControllerEnabled()) {
    bool mqttControllerEnabled = firstEnabledMQTTController() >= 0;
    unsigned long timer = millis() + 1000;
    while (!timeOutReached(timer)) {

      CPluginCall(CPLUGIN_FLUSH, 0);
      if (mqttControllerEnabled && MQTTclient.connected()) {
        MQTTclient.loop();
      }
    }
    if (mqttControllerEnabled && MQTTclient.connected()) {
      MQTTclient.disconnect();
      updateMQTTclient_connected();
    }
    saveToRTC();
    delay(100);
  }
}

void runPeriodicalMQTT() {

  if (!WiFiConnected(10)) {
    updateMQTTclient_connected();
    return;
  }

  int enabledMqttController = firstEnabledMQTTController();
  if (enabledMqttController >= 0) {
    if (!MQTTclient.loop()) {
      updateMQTTclient_connected();
      if (MQTTCheck(enabledMqttController)) {
        updateMQTTclient_connected();
      }
    }
  } else {
    if (MQTTclient.connected()) {
      MQTTclient.disconnect();
      updateMQTTclient_connected();
    }
  }
}

void updateMQTTclient_connected() {
  if (MQTTclient_connected != MQTTclient.connected()) {
    MQTTclient_connected = !MQTTclient_connected;
    if (!MQTTclient_connected) {
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String connectionError = F("MQTT : Connection lost, state: ");
        connectionError += getMQTT_state();
        addLog(LOG_LEVEL_ERROR, connectionError);
      }
    } else {
      schedule_all_tasks_using_MQTT_controller();
    }
    if (Settings.UseRules) {
      String event = MQTTclient_connected ? F("MQTT#Connected") : F("MQTT#Disconnected");
      rulesProcessing(event);
    }
  }
  if (!MQTTclient_connected) {

    if (timermqtt_interval < 30000) {
      timermqtt_interval += 5000;
    }
  } else {
    timermqtt_interval = 250;
  }
  setIntervalTimer(TIMER_MQTT);
}





void run50TimesPerSecond() {
  START_TIMER;
  PluginCall(PLUGIN_FIFTY_PER_SECOND, 0, dummyString);
  STOP_TIMER(PLUGIN_CALL_50PS);
}




void run10TimesPerSecond() {
  {
    START_TIMER;
    PluginCall(PLUGIN_TEN_PER_SECOND, 0, dummyString);
    STOP_TIMER(PLUGIN_CALL_10PS);
  }
  {
    START_TIMER;

    PluginCall(PLUGIN_MONITOR, 0, dummyString);
    STOP_TIMER(PLUGIN_CALL_10PSU);
  }
  if (Settings.UseRules && eventBuffer.length() > 0)
  {
    rulesProcessing(eventBuffer);
    eventBuffer = "";
  }
  #ifdef USES_C015
  if (WiFiConnected())
      Blynk_Run_c015();
  #endif
  #ifndef USE_RTOS_MULTITASKING
    WebServer.handleClient();
  #endif
}





void runOncePerSecond()
{
  START_TIMER;
  updateLogLevelCache();
  dailyResetCounter++;
  if (dailyResetCounter > 86400)
  {
    RTC.flashDayCounter=0;
    saveToRTC();
    dailyResetCounter=0;
    addLog(LOG_LEVEL_INFO, F("SYS  : Reset 24h counters"));
  }

  if (Settings.ConnectionFailuresThreshold)
    if (connectionFailures > Settings.ConnectionFailuresThreshold)
      delayedReboot(60);

  if (cmd_within_mainloop != 0)
  {
    switch (cmd_within_mainloop)
    {
      case CMD_WIFI_DISCONNECT:
        {
          WifiDisconnect();
          break;
        }
      case CMD_REBOOT:
        {
          reboot();
          break;
        }
    }
    cmd_within_mainloop = 0;
  }
  WifiCheck();


  if (systemTimePresent())
    checkTime();


  PluginCall(PLUGIN_ONCE_A_SECOND, 0, dummyString);


  if (Settings.UseRules)
    rulesTimers();


  if (SecuritySettings.Password[0] != 0)
  {
    if (WebLoggedIn)
      WebLoggedInTimer++;
    if (WebLoggedInTimer > 300)
      WebLoggedIn = false;
  }


  if (Settings.WDI2CAddress != 0)
  {
    Wire.beginTransmission(Settings.WDI2CAddress);
    Wire.write(0xA5);
    Wire.endTransmission();
  }
# 840 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy.ino"
  checkResetFactoryPin();
  STOP_TIMER(PLUGIN_CALL_1PS);
}

void logTimerStatistics() {
  byte loglevel = LOG_LEVEL_DEBUG;
  updateLoopStats_30sec(loglevel);
#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(loglevel)) {
    String queueLog = F("Scheduler stats: (called/tasks/max_length/idle%) ");
    queueLog += msecTimerHandler.getQueueStats();
    addLog(loglevel, queueLog);
  }
#endif
}




void runEach30Seconds()
{
   extern void checkRAMtoLog();
  checkRAMtoLog();
  wdcounter++;
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    log.reserve(80);
    log = F("WD   : Uptime ");
    log += wdcounter / 2;
    log += F(" ConnectFailures ");
    log += connectionFailures;
    log += F(" FreeMem ");
    log += FreeMem();
    log += F(" WiFiStatus ");
    log += WiFi.status();


    addLog(LOG_LEVEL_INFO, log);
  }
  sendSysInfoUDP(1);
  refreshNodeList();


  CPluginCall(CPLUGIN_INTERVAL, 0);

  #if defined(ESP8266)
  if (Settings.UseSSDP)
    SSDP_update();
  #endif
#if FEATURE_ADC_VCC
  vcc = ESP.getVcc() / 1000.0;
#endif

  #ifdef FEATURE_REPORTING
  ReportStatus();
  #endif

}
# 916 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy.ino"
void SensorSendTask(byte TaskIndex)
{
  checkRAM(F("SensorSendTask"));
  if (Settings.TaskDeviceEnabled[TaskIndex])
  {
    byte varIndex = TaskIndex * VARS_PER_TASK;

    bool success = false;
    byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[TaskIndex]);
    LoadTaskSettings(TaskIndex);

    struct EventStruct TempEvent;
    TempEvent.TaskIndex = TaskIndex;
    TempEvent.BaseVarIndex = varIndex;

    TempEvent.sensorType = Device[DeviceIndex].VType;

    float preValue[VARS_PER_TASK];
    for (byte varNr = 0; varNr < VARS_PER_TASK; varNr++)
      preValue[varNr] = UserVar[varIndex + varNr];

    if(Settings.TaskDeviceDataFeed[TaskIndex] == 0)
      success = PluginCall(PLUGIN_READ, &TempEvent, dummyString);
    else
      success = true;

    if (success)
    {
      START_TIMER;
      for (byte varNr = 0; varNr < VARS_PER_TASK; varNr++)
      {
        if (ExtraTaskSettings.TaskDeviceFormula[varNr][0] != 0)
        {
          String formula = ExtraTaskSettings.TaskDeviceFormula[varNr];
          formula.replace(F("%pvalue%"), String(preValue[varNr]));
          formula.replace(F("%value%"), String(UserVar[varIndex + varNr]));
          float result = 0;
          byte error = Calculate(formula.c_str(), &result);
          if (error == 0)
            UserVar[varIndex + varNr] = result;
        }
      }
      STOP_TIMER(COMPUTE_FORMULA_STATS);
      sendData(&TempEvent);
    }
  }
}





bool runningBackgroundTasks=false;
void backgroundtasks()
{


  delay(0);
# 986 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy.ino"
  if (runningBackgroundTasks)
  {
    return;
  }
  START_TIMER
  const bool wifiConnected = WiFiConnected();
  runningBackgroundTasks=true;

  #if defined(ESP8266)
  if (wifiConnected) {
    tcpCleanup();
  }
  #endif
  process_serialWriteBuffer();
  if(!UseRTOSMultitasking){
    if (Settings.UseSerial && Serial.available()) {
      if (!PluginCall(PLUGIN_SERIAL_IN, 0, dummyString)) {
        serial();
      }
    }
    if (wifiConnected) {
      WebServer.handleClient();
      checkUDP();
    }
  }


  if (dnsServerActive && wifiConnected)
    dnsServer.processNextRequest();

  #ifdef FEATURE_ARDUINO_OTA
  if(Settings.ArduinoOTAEnable && wifiConnected)
    ArduinoOTA.handle();


  while (ArduinoOTAtriggered)
  {
    delay(0);
    if (WiFiConnected()) {
      ArduinoOTA.handle();
    }
  }

  #endif

  #ifdef FEATURE_MDNS

  if (wifiConnected) {
    MDNS.update();
  }
  #endif

  delay(0);

  statusLED(false);

  runningBackgroundTasks=false;
  STOP_TIMER(BACKGROUND_TASKS);
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Command.ino"

#include "Commands/Common.h"
#include "Commands/Blynk.h"
#include "Commands/Blynk_c015.h"
#include "Commands/Diagnostic.h"
#include "Commands/HTTP.h"
#include "Commands/i2c.h"
#include "Commands/MQTT.h"
#include "Commands/Networks.h"
#include "Commands/Notifications.h"
#include "Commands/RTC.h"
#include "Commands/Rules.h"
#include "Commands/SDCARD.h"
#include "Commands/Settings.h"
#include "Commands/System.h"
#include "Commands/Tasks.h"
#include "Commands/Time.h"
#include "Commands/Timer.h"
#include "Commands/UPD.h"
#include "Commands/wd.h"
#include "Commands/WiFi.h"





String doExecuteCommand(const char *cmd, struct EventStruct *event, const char *line)
{
  String cmd_lc;

  cmd_lc = cmd;
  cmd_lc.toLowerCase();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("Command: ");
    log += cmd_lc;
    addLog(LOG_LEVEL_INFO, log);
#ifndef BUILD_NO_DEBUG
    addLog(LOG_LEVEL_DEBUG, line);
#endif
  }


  #define COMMAND_CASE(S, C) \
  if (strcmp_P(cmd_lc.c_str(), PSTR(S)) == 0) { return C(event, line); }

  switch (cmd_lc[0]) {
    case 'a': {
      COMMAND_CASE("accessinfo", Command_AccessInfo_Ls);
      break;
    }
    case 'b': {
      COMMAND_CASE("background", Command_Background);
    #ifdef USES_C012
      COMMAND_CASE("blynkget", Command_Blynk_Get);
    #endif
    #ifdef USES_C015
      COMMAND_CASE("blynkset", Command_Blynk_Set);
    #endif
      COMMAND_CASE("build", Command_Settings_Build);
      break;
    }
    case 'c': {
      COMMAND_CASE("clearaccessblock", Command_AccessInfo_Clear);
      COMMAND_CASE("clearrtcram", Command_RTC_Clear);
      COMMAND_CASE("config", Command_Task_RemoteConfig);
      break;
    }
    case 'd': {
      COMMAND_CASE("debug", Command_Debug);
      COMMAND_CASE("deepsleep", Command_System_deepSleep);
      COMMAND_CASE("delay", Command_Delay);
      COMMAND_CASE("dns", Command_DNS);
      COMMAND_CASE("dst", Command_DST);
      break;
    }
    case 'e': {
      COMMAND_CASE("erase", Command_WiFi_Erase);
      COMMAND_CASE("event", Command_Rules_Events);
      COMMAND_CASE("executerules", Command_Rules_Execute);
      break;
    }
    case 'g': {
      COMMAND_CASE("gateway", Command_Gateway);
      break;
    }
    case 'i': {
      COMMAND_CASE("i2cscanner", Command_i2c_Scanner);
      COMMAND_CASE("ip", Command_IP);
      break;
    }
    case 'j': {
      COMMAND_CASE("jsonportstatus", Command_JSONPortStatus);
    }
    case 'l': {
      COMMAND_CASE("let", Command_Rules_Let);
      COMMAND_CASE("load", Command_Settings_Load);
      COMMAND_CASE("logentry", Command_logentry);
      COMMAND_CASE("logportstatus", Command_logPortStatus);
      COMMAND_CASE("lowmem", Command_Lowmem);
      break;
    }
    case 'm': {
      COMMAND_CASE("malloc", Command_Malloc);
      COMMAND_CASE("meminfo", Command_MemInfo);
      COMMAND_CASE("meminfodetail", Command_MemInfo_detail);
      COMMAND_CASE("messagedelay", Command_MQTT_messageDelay);
      COMMAND_CASE("mqttretainflag", Command_MQTT_Retain);
      break;
    }
    case 'n': {
      COMMAND_CASE("name", Command_Settings_Name);
      COMMAND_CASE("nosleep", Command_System_NoSleep);
      COMMAND_CASE("notify", Command_Notifications_Notify);
      COMMAND_CASE("ntphost", Command_NTPHost);
      break;
    }
    case 'p': {
      COMMAND_CASE("password", Command_Settings_Password);
      COMMAND_CASE("publish", Command_MQTT_Publish);
      break;
    }
    case 'r': {
      COMMAND_CASE("reboot", Command_System_Reboot);
      COMMAND_CASE("reset", Command_Settings_Reset);
      COMMAND_CASE("resetflashwritecounter", Command_RTC_resetFlashWriteCounter);
      COMMAND_CASE("restart", Command_System_Restart);
      COMMAND_CASE("rules", Command_Rules_UseRules);
      break;
    }
    case 's': {
      COMMAND_CASE("save", Command_Settings_Save);
        #ifdef FEATURE_SD
      COMMAND_CASE("sdcard", Command_SD_LS);
      COMMAND_CASE("sdremove", Command_SD_Remove);
        #endif
      COMMAND_CASE("sendto", Command_UPD_SendTo);
      COMMAND_CASE("sendtohttp", Command_HTTP_SendToHTTP);
      COMMAND_CASE("sendtoudp", Command_UDP_SendToUPD);
      COMMAND_CASE("serialfloat", Command_SerialFloat);
      COMMAND_CASE("settings", Command_Settings_Print);
      COMMAND_CASE("subnet", Command_Subnet);
      COMMAND_CASE("sysload", Command_SysLoad);
      break;
    }
    case 't': {
      COMMAND_CASE("taskclear", Command_Task_Clear);
      COMMAND_CASE("taskclearall", Command_Task_ClearAll);
      COMMAND_CASE("taskrun", Command_Task_Run);
      COMMAND_CASE("taskvalueset", Command_Task_ValueSet);
      COMMAND_CASE("taskvaluetoggle", Command_Task_ValueToggle);
      COMMAND_CASE("taskvaluesetandrun", Command_Task_ValueSetAndRun);
      COMMAND_CASE("timerpause", Command_Timer_Pause);
      COMMAND_CASE("timerresume", Command_Timer_Resume);
      COMMAND_CASE("timerset", Command_Timer_Set);
      COMMAND_CASE("timezone", Command_TimeZone);
      break;
    }
    case 'u': {
      COMMAND_CASE("udpport", Command_UDP_Port);
      COMMAND_CASE("udptest", Command_UDP_Test);
      COMMAND_CASE("unit", Command_Settings_Unit);
      COMMAND_CASE("usentp", Command_useNTP);
      break;
    }
    case 'w': {
      COMMAND_CASE("wdconfig", Command_WD_Config);
      COMMAND_CASE("wdread", Command_WD_Read);
      COMMAND_CASE("wifiapmode", Command_Wifi_APMode);
      COMMAND_CASE("wificonnect", Command_Wifi_Connect);
      COMMAND_CASE("wifidisconnect", Command_Wifi_Disconnect);
      COMMAND_CASE("wifikey", Command_Wifi_Key);
      COMMAND_CASE("wifikey2", Command_Wifi_Key2);
      COMMAND_CASE("wifimode", Command_Wifi_Mode);
      COMMAND_CASE("wifiscan", Command_Wifi_Scan);
      COMMAND_CASE("wifissid", Command_Wifi_SSID);
      COMMAND_CASE("wifissid2", Command_Wifi_SSID2);
      COMMAND_CASE("wifistamode", Command_Wifi_STAMode);
      break;
    }
    default:
      break;
  }

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String errorUnknown = F("Command unknown: \"");
    errorUnknown += cmd_lc;
    errorUnknown += '\"';
    addLog(LOG_LEVEL_INFO, errorUnknown);
  }
  return F("\nUnknown command!");

  #undef COMMAND_CASE
}


String return_command_success()
{
  return F("\nOk");
}

String return_command_failed()
{
  return F("\nFailed");
}

String return_not_connected()
{
  return F("Not connected to WiFi");
}

String return_result(struct EventStruct *event, const String& result)
{
  serialPrintln(result);

  if (event->Source == VALUE_SOURCE_SERIAL) {
    return return_command_success();
  }
  return result;
}

String return_see_serial(struct EventStruct *event)
{
  if (event->Source == VALUE_SOURCE_SERIAL) {
    return return_command_success();
  }
  return F("Output sent to serial");
}

void ExecuteCommand(byte source, const char *Line)
{
  checkRAM(F("ExecuteCommand"));
  String cmd;

  if (!GetArgv(Line, cmd, 1)) {
    return;
  }
  struct EventStruct TempEvent;



  TempEvent.Source = source;
  {

    String TmpStr1;

    if (GetArgv(Line, TmpStr1, 2)) { TempEvent.Par1 = CalculateParam(TmpStr1.c_str()); }

    if (GetArgv(Line, TmpStr1, 3)) { TempEvent.Par2 = CalculateParam(TmpStr1.c_str()); }

    if (GetArgv(Line, TmpStr1, 4)) { TempEvent.Par3 = CalculateParam(TmpStr1.c_str()); }

    if (GetArgv(Line, TmpStr1, 5)) { TempEvent.Par4 = CalculateParam(TmpStr1.c_str()); }

    if (GetArgv(Line, TmpStr1, 6)) { TempEvent.Par5 = CalculateParam(TmpStr1.c_str()); }
  }

  String status = doExecuteCommand(cmd.c_str(), &TempEvent, Line);
  delay(0);
  SendStatus(source, status);
  delay(0);







}

#ifdef FEATURE_SD
void printDirectory(File dir, int numTabs)
{
  while (true) {
    File entry = dir.openNextFile();

    if (!entry) {

      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      serialPrint("\t");
    }
    serialPrint(entry.name());

    if (entry.isDirectory()) {
      serialPrintln("/");
      printDirectory(entry, numTabs + 1);
    } else {

      serialPrint("\t\t");
      serialPrintln(String(entry.size(), DEC));
    }
    entry.close();
  }
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Controller.ino"




void sendData(struct EventStruct *event)
{
  START_TIMER;
  checkRAM(F("sendData"));
  LoadTaskSettings(event->TaskIndex);

  if (Settings.UseRules) {
    createRuleEvents(event->TaskIndex);
  }

  if (Settings.UseValueLogger && Settings.InitSPI && (Settings.Pin_sd_cs >= 0)) {
    SendValueLogger(event->TaskIndex);
  }
# 42 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Controller.ino"
  LoadTaskSettings(event->TaskIndex);

  for (byte x = 0; x < CONTROLLER_MAX; x++)
  {
    event->ControllerIndex = x;
    event->idx = Settings.TaskDeviceID[x][event->TaskIndex];

    if (Settings.TaskDeviceSendData[event->ControllerIndex][event->TaskIndex] &&
        Settings.ControllerEnabled[event->ControllerIndex] &&
        Settings.Protocol[event->ControllerIndex])
    {
      event->ProtocolIndex = getProtocolIndex(Settings.Protocol[event->ControllerIndex]);

      if (validUserVar(event)) {
        CPluginCall(event->ProtocolIndex, CPLUGIN_PROTOCOL_SEND, event, dummyString);
      }
#ifndef BUILD_NO_DEBUG
      else {
        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
          String log = F("Invalid value detected for controller ");
          String controllerName;
          CPluginCall(event->ProtocolIndex, CPLUGIN_GET_DEVICENAME, event, controllerName);
          log += controllerName;
          addLog(LOG_LEVEL_DEBUG, log);
        }
      }
#endif
    }
  }


  PluginCall(PLUGIN_EVENT_OUT, event, dummyString);
  lastSend = millis();
  STOP_TIMER(SEND_DATA_STATS);
}

boolean validUserVar(struct EventStruct *event) {
  byte valueCount = getValueCountFromSensorType(event->sensorType);

  for (int i = 0; i < valueCount; ++i) {
    const float f(UserVar[event->BaseVarIndex + i]);

    if (!isValidFloat(f)) { return false; }
  }
  return true;
}






void callback(char *c_topic, byte *b_payload, unsigned int length) {
  statusLED(true);
  int enabledMqttController = firstEnabledMQTTController();

  if (enabledMqttController < 0) {
    addLog(LOG_LEVEL_ERROR, F("MQTT : No enabled MQTT controller"));
    return;
  }

  if (length > 384)
  {
    addLog(LOG_LEVEL_ERROR, F("MQTT : Ignored too big message"));
    return;
  }

  struct EventStruct TempEvent;


  TempEvent.String1 = c_topic;
  TempEvent.String2.reserve(length);

  for (unsigned int i = 0; i < length; ++i) {
    char c = static_cast<char>(*(b_payload + i));
    TempEvent.String2 += c;
  }
# 133 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Controller.ino"
  byte ProtocolIndex = getProtocolIndex(Settings.Protocol[enabledMqttController]);
  schedule_controller_event_timer(ProtocolIndex, CPLUGIN_PROTOCOL_RECV, &TempEvent);
}




void MQTTDisconnect()
{
  if (MQTTclient.connected()) {
    MQTTclient.disconnect();
    addLog(LOG_LEVEL_INFO, F("MQTT : Disconnected from broker"));
    updateMQTTclient_connected();
  }
}




bool MQTTConnect(int controller_idx)
{
  ++mqtt_reconnect_count;
  MakeControllerSettings(ControllerSettings);
  LoadControllerSettings(controller_idx, ControllerSettings);

  if (!ControllerSettings.checkHostReachable(true)) {
    return false;
  }

  if (MQTTclient.connected()) {
    MQTTclient.disconnect();
    updateMQTTclient_connected();
  }
  mqtt = WiFiClient();
  mqtt.setTimeout(ControllerSettings.ClientTimeout);
  MQTTclient.setClient(mqtt);

  if (ControllerSettings.UseDNS) {
    MQTTclient.setServer(ControllerSettings.getHost().c_str(), ControllerSettings.Port);
  } else {
    MQTTclient.setServer(ControllerSettings.getIP(), ControllerSettings.Port);
  }
  MQTTclient.setCallback(callback);


  String clientid;

  if (Settings.MQTTUseUnitNameAsClientId) {
    clientid = Settings.Name;

    if (Settings.Unit != 0) {
      clientid += '_';
      clientid += Settings.Unit;
    }
  }
  else {
    clientid = F("ESPClient_");
    clientid += WiFi.macAddress();
  }
  clientid.replace(' ', '_');

  if ((wifi_reconnects >= 1) && Settings.uniqueMQTTclientIdReconnect()) {





    clientid += '_';
    clientid += wifi_reconnects;
  }

  String LWTTopic = ControllerSettings.MQTTLwtTopic;

  if (LWTTopic.length() == 0)
  {
    LWTTopic = ControllerSettings.Subscribe;
    LWTTopic += F("/LWT");
  }
  LWTTopic.replace(F("/#"), F("/status"));
  parseSystemVariables(LWTTopic, false);

  String LWTMessageConnect = ControllerSettings.LWTMessageConnect;

  if (LWTMessageConnect.length() == 0) {
    LWTMessageConnect = F(DEFAULT_MQTT_LWT_CONNECT_MESSAGE);
  }
  parseSystemVariables(LWTMessageConnect, false);

  String LWTMessageDisconnect = ControllerSettings.LWTMessageDisconnect;

  if (LWTMessageDisconnect.length() == 0) {
    LWTMessageDisconnect = F(DEFAULT_MQTT_LWT_DISCONNECT_MESSAGE);
  }
  parseSystemVariables(LWTMessageDisconnect, false);

  boolean MQTTresult = false;
  uint8_t willQos = 0;
  boolean willRetain = true;
  boolean cleanSession = false;

  if ((SecuritySettings.ControllerUser[controller_idx] != 0) && (SecuritySettings.ControllerPassword[controller_idx] != 0)) {
    MQTTresult =
      MQTTclient.connect(clientid.c_str(),
                         SecuritySettings.ControllerUser[controller_idx],
                         SecuritySettings.ControllerPassword[controller_idx],
                         LWTTopic.c_str(),
                         willQos,
                         willRetain,
                         LWTMessageDisconnect.c_str(),
                         cleanSession);
  } else {
    MQTTresult = MQTTclient.connect(clientid.c_str(),
                                    nullptr,
                                    nullptr,
                                    LWTTopic.c_str(),
                                    willQos,
                                    willRetain,
                                    LWTMessageDisconnect.c_str(),
                                    cleanSession);
  }
  delay(0);

  if (!MQTTresult) {
    addLog(LOG_LEVEL_ERROR, F("MQTT : Failed to connect to broker"));
    MQTTclient.disconnect();
    updateMQTTclient_connected();
    return false;
  }
  String log = F("MQTT : Connected to broker with client ID: ");
  log += clientid;
  addLog(LOG_LEVEL_INFO, log);
  String subscribeTo = ControllerSettings.Subscribe;
  parseSystemVariables(subscribeTo, false);
  MQTTclient.subscribe(subscribeTo.c_str());
  log = F("Subscribed to: ");
  log += subscribeTo;
  addLog(LOG_LEVEL_INFO, log);

  if (MQTTclient.publish(LWTTopic.c_str(), LWTMessageConnect.c_str(), 1)) {
    updateMQTTclient_connected();
    statusLED(true);
    mqtt_reconnect_count = 0;


    if (MQTTclient_should_reconnect) { CPluginCall(CPLUGIN_GOT_CONNECTED, 0); }
    MQTTclient_should_reconnect = false;
    return true;
  }
  return false;
}




bool MQTTCheck(int controller_idx)
{
  if (!WiFiConnected(10)) {
    return false;
  }
  byte ProtocolIndex = getProtocolIndex(Settings.Protocol[controller_idx]);

  if (Protocol[ProtocolIndex].usesMQTT)
  {
    if (MQTTclient_should_reconnect || !MQTTclient.connected())
    {
      if (MQTTclient_should_reconnect) {
        addLog(LOG_LEVEL_ERROR, F("MQTT : Intentional reconnect"));
      } else {
        connectionFailures += 2;
      }
      return MQTTConnect(controller_idx);
    } else if (connectionFailures) {
      connectionFailures--;
    }
  }


  return true;
}




void SendStatusOnlyIfNeeded(int eventSource, bool param1, uint32_t key, const String& param2, int16_t param3) {
  switch (eventSource) {
    case VALUE_SOURCE_HTTP:
    case VALUE_SOURCE_SERIAL:
    case VALUE_SOURCE_MQTT:
    case VALUE_SOURCE_WEB_FRONTEND:
      SendStatus(eventSource, getPinStateJSON(param1, key, param2, param3));
      break;
  }
}

void SendStatus(byte source, const String& status)
{
  switch (source)
  {
    case VALUE_SOURCE_HTTP:
    case VALUE_SOURCE_WEB_FRONTEND:

      if (printToWeb) {
        printWebString += status;
      }
      break;
    case VALUE_SOURCE_MQTT:
      MQTTStatus(status);
      break;
    case VALUE_SOURCE_SERIAL:
      serialPrintln(status);
      break;
  }
}

boolean MQTTpublish(int controller_idx, const char *topic, const char *payload, boolean retained)
{
  const bool success = MQTTDelayHandler.addToQueue(MQTT_queue_element(controller_idx, topic, payload, retained));

  scheduleNextMQTTdelayQueue();
  return success;
}

void scheduleNextMQTTdelayQueue() {
  scheduleNextDelayQueue(TIMER_MQTT_DELAY_QUEUE, MQTTDelayHandler.getNextScheduleTime());
}

void processMQTTdelayQueue() {
  START_TIMER;
  MQTT_queue_element *element(MQTTDelayHandler.getNext());

  if (element == NULL) { return; }

  if (MQTTclient.publish(element->_topic.c_str(), element->_payload.c_str(), element->_retained)) {
    setIntervalTimerOverride(TIMER_MQTT, 10);
    MQTTDelayHandler.markProcessed(true);
  } else {
    MQTTDelayHandler.markProcessed(false);
#ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("MQTT : process MQTT queue not published, ");
      log += MQTTDelayHandler.sendQueue.size();
      log += F(" items left in queue");
      addLog(LOG_LEVEL_DEBUG, log);
    }
#endif
  }
  scheduleNextMQTTdelayQueue();
  STOP_TIMER(MQTT_DELAY_QUEUE);
}




void MQTTStatus(const String& status)
{
  MakeControllerSettings(ControllerSettings);
  int enabledMqttController = firstEnabledMQTTController();

  if (enabledMqttController >= 0) {
    LoadControllerSettings(enabledMqttController, ControllerSettings);
    String pubname = ControllerSettings.Subscribe;
    pubname.replace(F("/#"), F("/status"));
    parseSystemVariables(pubname, false);
    MQTTpublish(enabledMqttController, pubname.c_str(), status.c_str(), Settings.MQTTRetainFlag);
  }
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Convert.ino"




String getBearing(int degrees)
{
  const __FlashStringHelper *bearing[] = {
    F("N"),
    F("NNE"),
    F("NE"),
    F("ENE"),
    F("E"),
    F("ESE"),
    F("SE"),
    F("SSE"),
    F("S"),
    F("SSW"),
    F("SW"),
    F("WSW"),
    F("W"),
    F("WNW"),
    F("NW"),
    F("NNW")
  };
  int nr_directions = (int)(sizeof(bearing) / sizeof(bearing[0]));
  float stepsize = (360.0 / nr_directions);

  if (degrees < 0) { degrees += 360; }
  int bearing_idx = int((degrees + (stepsize / 2.0)) / stepsize) % nr_directions;

  if (bearing_idx < 0) {
    return "";
  }
  else {
    return bearing[bearing_idx];
  }
}

float CelsiusToFahrenheit(float celsius) {
  return celsius * (9.0 / 5.0) + 32;
}

int m_secToBeaufort(float m_per_sec) {
  if (m_per_sec < 0.3) { return 0; }

  if (m_per_sec < 1.6) { return 1; }

  if (m_per_sec < 3.4) { return 2; }

  if (m_per_sec < 5.5) { return 3; }

  if (m_per_sec < 8.0) { return 4; }

  if (m_per_sec < 10.8) { return 5; }

  if (m_per_sec < 13.9) { return 6; }

  if (m_per_sec < 17.2) { return 7; }

  if (m_per_sec < 20.8) { return 8; }

  if (m_per_sec < 24.5) { return 9; }

  if (m_per_sec < 28.5) { return 10; }

  if (m_per_sec < 32.6) { return 11; }
  return 12;
}

String centimeterToImperialLength(float cm) {
  return millimeterToImperialLength(cm * 10.0);
}

String millimeterToImperialLength(float mm) {
  float inches = mm / 25.4;
  int feet = inches / 12;

  inches = inches - (feet * 12);
  String result;
  result.reserve(10);

  if (feet != 0) {
    result += feet;
    result += '\'';
  }
  result += toString(inches, 1);
  result += '"';
  return result;
}

float minutesToDay(int minutes) {
  return minutes / 1440.0;
}

String minutesToDayHour(int minutes) {
  int days = minutes / 1440;
  int hours = (minutes % 1440) / 60;
  char TimeString[6];

  sprintf_P(TimeString, PSTR("%d%c%02d%c"), days, 'd', hours, 'h');
  return TimeString;
}

String minutesToHourMinute(int minutes) {
  int hours = (minutes % 1440) / 60;
  int mins = (minutes % 1440) % 60;
  char TimeString[20];

  sprintf_P(TimeString, PSTR("%d%c%02d%c"), hours, 'h', mins, 'm');
  return TimeString;
}

String minutesToDayHourMinute(int minutes) {
  int days = minutes / 1440;
  int hours = (minutes % 1440) / 60;
  int mins = (minutes % 1440) % 60;
  char TimeString[20];

  sprintf_P(TimeString, PSTR("%d%c%02d%c%02d%c"), days, 'd', hours, 'h', mins, 'm');
  return TimeString;
}

String secondsToDayHourMinuteSecond(int seconds) {
  int sec = seconds % 60;
  int minutes = seconds / 60;
  int days = minutes / 1440;
  int hours = (minutes % 1440) / 60;
  int mins = (minutes % 1440) % 60;
  char TimeString[20];

  sprintf_P(TimeString, PSTR("%d%c%02d%c%02d%c%02d"), days, 'd', hours, ':', mins, ':', sec);
  return TimeString;
}

String format_msec_duration(long duration) {
  String result;

  if (duration < 0) {
    result = "-";
    duration = -1 * duration;
  }

  if (duration < 10000) {
    result += duration;
    result += F(" ms");
    return result;
  }
  duration /= 1000;

  if (duration < 3600) {
    int sec = duration % 60;
    int minutes = duration / 60;

    if (minutes > 0) {
      result += minutes;
      result += F(" m ");
    }
    result += sec;
    result += F(" s");
    return result;
  }
  duration /= 60;

  if (duration < 1440) { return minutesToHourMinute(duration); }
  return minutesToDayHourMinute(duration);
}




unsigned long float2ul(float f)
{
  unsigned long ul;

  memcpy(&ul, &f, 4);
  return ul;
}




float ul2float(unsigned long ul)
{
  float f;

  memcpy(&f, &ul, 4);
  return f;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyRTC.ino"
# 45 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyRTC.ino"
#define CACHE_STORAGE_SPIFFS 0


#define CACHE_STORAGE_OTA_FREE 1


#define CACHE_STORAGE_NO_OTA_FREE 2


#define CACHE_STORAGE_BEHIND_SPIFFS 3





boolean saveToRTC()
{
  #if defined(ESP32)
  return false;
  #else

  if (!system_rtc_mem_write(RTC_BASE_STRUCT, (byte *)&RTC, sizeof(RTC)) || !readFromRTC())
  {
      # ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : Error while writing to RTC"));
      # endif
    return false;
  }
  else
  {
    return true;
  }
  #endif
}




void initRTC()
{
  memset(&RTC, 0, sizeof(RTC));
  RTC.ID1 = 0xAA;
  RTC.ID2 = 0x55;
  saveToRTC();

  memset(&UserVar, 0, sizeof(UserVar));
  saveUserVarToRTC();
}




boolean readFromRTC()
{
  #if defined(ESP32)
  return false;
  #else

  if (!system_rtc_mem_read(RTC_BASE_STRUCT, (byte *)&RTC, sizeof(RTC))) {
    return false;
  }
  return RTC.ID1 == 0xAA && RTC.ID2 == 0x55;
  #endif
}




boolean saveUserVarToRTC()
{
  #if defined(ESP32)
  return false;
  #else


  byte *buffer = (byte *)&UserVar;
  size_t size = sizeof(UserVar);
  uint32_t sum = calc_CRC32(buffer, size);
  boolean ret = system_rtc_mem_write(RTC_BASE_USERVAR, buffer, size);
  ret &= system_rtc_mem_write(RTC_BASE_USERVAR + (size >> 2), (byte *)&sum, 4);
  return ret;
  #endif
}




boolean readUserVarFromRTC()
{
  #if defined(ESP32)
  return false;
  #else


  byte *buffer = (byte *)&UserVar;
  size_t size = sizeof(UserVar);
  boolean ret = system_rtc_mem_read(RTC_BASE_USERVAR, buffer, size);
  uint32_t sumRAM = calc_CRC32(buffer, size);
  uint32_t sumRTC = 0;
  ret &= system_rtc_mem_read(RTC_BASE_USERVAR + (size >> 2), (byte *)&sumRTC, 4);

  if (!ret || (sumRTC != sumRAM))
  {
      # ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : Checksum error on reading RTC user var"));
      # endif
    memset(buffer, 0, size);
  }
  return ret;
  #endif
}




struct RTC_cache_handler_struct
{
  RTC_cache_handler_struct() {
    bool success = loadMetaData() && loadData();

    if (!success) {
      #ifdef RTC_STRUCT_DEBUG
      addLog(LOG_LEVEL_INFO, F("RTC  : Error reading cache data"));
      #endif
      memset(&RTC_cache, 0, sizeof(RTC_cache));
      flush();
    } else {
      #ifdef RTC_STRUCT_DEBUG
      rtc_debug_log(F("Read from RTC cache"), RTC_cache.writePos);
      #endif
    }
  }

  unsigned int getFreeSpace() {
    if (RTC_cache.writePos >= RTC_CACHE_DATA_SIZE) {
      return 0;
    }
    return RTC_CACHE_DATA_SIZE - RTC_cache.writePos;
  }

  void resetpeek() {
    if (fp) {
      fp.close();
    }
    peekfilenr = 0;
    peekreadpos = 0;
  }

  bool peek(uint8_t *data, unsigned int size) {
    int retries = 2;

    while (retries > 0) {
      --retries;

      if (!fp) {
        int tmppos;
        String fname;

        if (peekfilenr == 0) {
          fname = getReadCacheFileName(tmppos);
          peekfilenr = getCacheFileCountFromFilename(fname);
        } else {
          ++peekfilenr;
          fname = createCacheFilename(peekfilenr);
        }

        if (fname.length() == 0) { return false; }
        fp = tryOpenFile(fname.c_str(), "r");
      }

      if (!fp) { return false; }

      if (fp.read(data, size)) {
        return true;
      }
      fp.close();
    }
    return true;
  }


  bool write(uint8_t *data, unsigned int size) {
    #ifdef RTC_STRUCT_DEBUG
    rtc_debug_log(F("write RTC cache data"), size);
    #endif

    if (getFreeSpace() < size) {
      if (!flush()) {
        return false;
      }
    }


    for (unsigned int i = 0; i < size; ++i) {
      RTC_cache_data[RTC_cache.writePos] = data[i];
      ++RTC_cache.writePos;
    }



    int startOffset = RTC_cache.writePos - size;
    startOffset -= startOffset % 4;

    if (startOffset < 0) {
      startOffset = 0;
    }
    int nrBytes = RTC_cache.writePos - startOffset;

    if (nrBytes % 4 != 0) {
      nrBytes -= nrBytes % 4;
      nrBytes += 4;
    }

    if ((nrBytes + startOffset) > RTC_CACHE_DATA_SIZE) {

      nrBytes = RTC_CACHE_DATA_SIZE - startOffset;
    }
    return saveRTCcache(startOffset, nrBytes);
  }


  bool flush() {
    if (prepareFileForWrite()) {
      if (RTC_cache.writePos > 0) {
        size_t filesize = fw.size();
        int bytesWriten = fw.write(&RTC_cache_data[0], RTC_cache.writePos);

        if ((bytesWriten < RTC_cache.writePos) || (fw.size() == filesize)) {
          #ifdef RTC_STRUCT_DEBUG
          String log = F("RTC  : error writing file. Size before: ");
          log += filesize;
          log += F(" after: ");
          log += fw.size();
          log += F(" writen: ");
          log += bytesWriten;
          addLog(LOG_LEVEL_ERROR, log);
          #endif
          fw.close();

          if (!GarbageCollection()) {

            writeerror = true;
          }
          return false;
        }
        delay(0);
        fw.flush();
        #ifdef RTC_STRUCT_DEBUG
        addLog(LOG_LEVEL_INFO, F("RTC  : flush RTC cache"));
        #endif
        initRTCcache_data();
        clearRTCcacheData();
        saveRTCcache();
        return true;
      }
    }
    return false;
  }



  String getReadCacheFileName(int& readPos) {
    initRTCcache_data();

    for (int i = 0; i < 2; ++i) {
      String fname = createCacheFilename(RTC_cache.readFileNr);

      if (SPIFFS.exists(fname)) {
        if (i != 0) {

          RTC_cache.readPos = 0;
        }
        readPos = RTC_cache.readPos;
        return fname;
      }

      if (i == 0) {
        updateRTC_filenameCounters();
      }
    }


    RTC_cache.readPos = 0;
    readPos = RTC_cache.readPos;
    return "";
  }

  String getPeekCacheFileName(bool& islast) {
    int tmppos;
    String fname;

    if (peekfilenr == 0) {
      fname = getReadCacheFileName(tmppos);
      peekfilenr = getCacheFileCountFromFilename(fname);
    } else {
      ++peekfilenr;
      fname = createCacheFilename(peekfilenr);
    }
    islast = peekfilenr > RTC_cache.writeFileNr;

    if (SPIFFS.exists(fname)) {
      return fname;
    }
    return "";
  }

  bool deleteOldestCacheBlock() {
    if (updateRTC_filenameCounters()) {
      if (RTC_cache.readFileNr != RTC_cache.writeFileNr) {

        String fname = createCacheFilename(RTC_cache.readFileNr);

        if (tryDeleteFile(fname)) {
          #ifdef RTC_STRUCT_DEBUG
          String log = F("RTC  : Removed file from SPIFFS: ");
          log += fname;
          addLog(LOG_LEVEL_INFO, String(log));
          #endif
          updateRTC_filenameCounters();
          writeerror = false;
          return true;
        }
      }
    }
    return false;
  }

private:

  bool loadMetaData()
  {
    #if defined(ESP32)
    return false;
    #else

    if (!system_rtc_mem_read(RTC_BASE_CACHE, (byte *)&RTC_cache, sizeof(RTC_cache))) {
      return false;
    }

    return RTC_cache.checksumMetadata == calc_CRC32((byte *)&RTC_cache, sizeof(RTC_cache) - sizeof(uint32_t));
    #endif
  }

  bool loadData()
  {
    #if defined(ESP32)
    return false;
    #else
    initRTCcache_data();

    if (!system_rtc_mem_read(RTC_BASE_CACHE + (sizeof(RTC_cache) / 4), (byte *)&RTC_cache_data[0], RTC_CACHE_DATA_SIZE)) {
      return false;
    }

    if (RTC_cache.checksumData != getDataChecksum()) {
        # ifdef RTC_STRUCT_DEBUG
      addLog(LOG_LEVEL_ERROR, F("RTC  : Checksum error reading RTC cache data"));
        # endif
      return false;
    }
    return RTC_cache.checksumData == getDataChecksum();
    #endif
  }

  bool saveRTCcache() {
    return saveRTCcache(0, RTC_CACHE_DATA_SIZE);
  }

  bool saveRTCcache(unsigned int startOffset, size_t nrBytes)
  {
    #if defined(ESP32)
    return false;
    #else
    RTC_cache.checksumData = getDataChecksum();
    RTC_cache.checksumMetadata = calc_CRC32((byte *)&RTC_cache, sizeof(RTC_cache) - sizeof(uint32_t));

    if (!system_rtc_mem_write(RTC_BASE_CACHE, (byte *)&RTC_cache, sizeof(RTC_cache)) || !loadMetaData())
    {
        # ifdef RTC_STRUCT_DEBUG
      addLog(LOG_LEVEL_ERROR, F("RTC  : Error while writing cache metadata to RTC"));
        # endif
      return false;
    }
    delay(0);

    if (nrBytes > 0) {
      const size_t address = RTC_BASE_CACHE + ((sizeof(RTC_cache) + startOffset) / 4);

      if (!system_rtc_mem_write(address, (byte *)&RTC_cache_data[startOffset], nrBytes))
      {
          # ifdef RTC_STRUCT_DEBUG
        addLog(LOG_LEVEL_ERROR, F("RTC  : Error while writing cache data to RTC"));
          # endif
        return false;
      }
        # ifdef RTC_STRUCT_DEBUG
      rtc_debug_log(F("Write cache data to RTC"), nrBytes);
        # endif
    }
    return true;
    #endif
  }

  uint32_t getDataChecksum() {
    initRTCcache_data();
    size_t dataLength = RTC_cache.writePos;

    if (dataLength > RTC_CACHE_DATA_SIZE) {

      dataLength = RTC_CACHE_DATA_SIZE;
    }


    return calc_CRC32((byte *)&RTC_cache_data[0], RTC_CACHE_DATA_SIZE);
  }

  void initRTCcache_data() {
    if (RTC_cache_data.size() != RTC_CACHE_DATA_SIZE) {
      RTC_cache_data.resize(RTC_CACHE_DATA_SIZE);
    }

    if (RTC_cache.writeFileNr == 0) {

      updateRTC_filenameCounters();
    }
  }

  void clearRTCcacheData() {
    for (size_t i = 0; i < RTC_CACHE_DATA_SIZE; ++i) {
      RTC_cache_data[i] = 0;
    }
    RTC_cache.writePos = 0;
  }


  bool updateRTC_filenameCounters() {
    size_t filesizeHighest;

    if (getCacheFileCounters(RTC_cache.readFileNr, RTC_cache.writeFileNr, filesizeHighest)) {
      if (filesizeHighest >= CACHE_FILE_MAX_SIZE) {

        ++RTC_cache.writeFileNr;
      }
      return true;
    } else {

      RTC_cache.writeFileNr = 1;
    }
    return false;
  }

  bool prepareFileForWrite() {



    if (SpiffsFull()) {
      #ifdef RTC_STRUCT_DEBUG
      addLog(LOG_LEVEL_ERROR, String(F("RTC  : SPIFFS full")));
      #endif
      return false;
    }
    unsigned int retries = 3;

    while (retries > 0) {
      --retries;

      if (fw && (fw.size() >= CACHE_FILE_MAX_SIZE)) {
        fw.close();
        GarbageCollection();
      }

      if (!fw) {

        initRTCcache_data();

        if (updateRTC_filenameCounters()) {
          if (writeerror || (SpiffsFreeSpace() < ((2 * CACHE_FILE_MAX_SIZE) + SpiffsBlocksize()))) {

            deleteOldestCacheBlock();
          }
        }

        String fname = createCacheFilename(RTC_cache.writeFileNr);
        fw = tryOpenFile(fname.c_str(), "a+");

        if (!fw) {
          #ifdef RTC_STRUCT_DEBUG
          addLog(LOG_LEVEL_ERROR, String(F("RTC  : error opening file")));
          #endif
        } else {
          #ifdef RTC_STRUCT_DEBUG

          if (loglevelActiveFor(LOG_LEVEL_INFO)) {
            String log = F("Write to ");
            log += fname;
            log += F(" size");
            rtc_debug_log(log, fw.size());
          }
          #endif
        }
      }
      delay(0);

      if (fw && (fw.size() < CACHE_FILE_MAX_SIZE)) {
        return true;
      }
    }
    #ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, String(F("RTC  : prepareFileForWrite failed")));
    #endif
    return false;
  }

#ifdef RTC_STRUCT_DEBUG
  void rtc_debug_log(const String& description, size_t nrBytes) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log.reserve(18 + description.length());
      log = F("RTC  : ");
      log += description;
      log += ' ';
      log += nrBytes;
      log += F(" bytes");
      addLog(LOG_LEVEL_INFO, log);
    }
  }

#endif

  RTC_cache_struct RTC_cache;
  std::vector<uint8_t>RTC_cache_data;
  File fw;
  File fr;
  File fp;
  size_t peekfilenr = 0;
  size_t peekreadpos = 0;

  byte storageLocation = CACHE_STORAGE_SPIFFS;
  bool writeerror = false;
};
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyRules.ino"
#define RULE_FILE_SEPARAROR '/'
#define RULE_MAX_FILENAME_LENGTH 24

String EventToFileName(String& eventName) {
  int size = eventName.length();
  int index = eventName.indexOf('=');

  if (index > -1) {
    size = index;
  }
#if defined(ESP8266)
  String fileName = F("rules/");
#endif
#if defined(ESP32)
  String fileName = F("/rules/");
#endif
  fileName += eventName.substring(0, size);
  fileName.replace('#', RULE_FILE_SEPARAROR);
  fileName.toLowerCase();
  return fileName;
}

String FileNameToEvent(String& fileName) {
#if defined(ESP8266)
  String eventName = fileName.substring(6);
#endif
#if defined(ESP32)
  String eventName = fileName.substring(7);
#endif
  eventName.replace(RULE_FILE_SEPARAROR, '#');
  return eventName;
}

void checkRuleSets() {
  for (byte x = 0; x < RULESETS_MAX; x++) {
#if defined(ESP8266)
    String fileName = F("rules");
#endif
#if defined(ESP32)
    String fileName = F("/rules");
#endif
    fileName += x + 1;
    fileName += F(".txt");

    if (SPIFFS.exists(fileName)) {
      activeRuleSets[x] = true;
    }
    else {
      activeRuleSets[x] = false;
    }

#ifndef BUILD_NO_DEBUG

    if (Settings.SerialLogLevel == LOG_LEVEL_DEBUG_DEV) {
      serialPrint(fileName);
      serialPrint(" ");
      serialPrintln(String(activeRuleSets[x]));
    }
#endif
  }
}




void rulesProcessing(String& event) {
  if (!Settings.UseRules) {
    return;
  }
  START_TIMER
    checkRAM(F("rulesProcessing"));
#ifndef BUILD_NO_DEBUG
  unsigned long timer = millis();
#endif

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("EVENT: ");
    log += event;
    addLog(LOG_LEVEL_INFO, log);
  }

  if (Settings.OldRulesEngine()) {
    for (byte x = 0; x < RULESETS_MAX; x++) {
#if defined(ESP8266)
      String fileName = F("rules");
#endif
#if defined(ESP32)
      String fileName = F("/rules");
#endif
      fileName += x + 1;
      fileName += F(".txt");

      if (activeRuleSets[x]) {
        rulesProcessingFile(fileName, event);
      }
    }
  } else {
    String fileName = EventToFileName(event);


    if (SPIFFS.exists(fileName)) {
      rulesProcessingFile(fileName, event);
    }
#ifndef BUILD_NO_DEBUG
    else {
      addLog(LOG_LEVEL_DEBUG, String(F("EVENT: ")) + event +
             String(F(" is ingnored. File ")) + fileName +
             String(F(" not found.")));
    }
#endif
  }

#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("EVENT: ");
    log += event;
    log += F(" Processing time:");
    log += timePassedSince(timer);
    log += F(" milliSeconds");
    addLog(LOG_LEVEL_DEBUG, log);
  }
#endif
  STOP_TIMER(RULES_PROCESSING);
  backgroundtasks();
}




String rulesProcessingFile(const String& fileName, String& event) {
  if (!Settings.UseRules || !fileExists(fileName)) {
    return "";
  }
  checkRAM(F("rulesProcessingFile"));
#ifndef BUILD_NO_DEBUG

  if (Settings.SerialLogLevel == LOG_LEVEL_DEBUG_DEV) {
    serialPrint(F("RuleDebug Processing:"));
    serialPrintln(fileName);
    serialPrintln(F("     flags CMI  parse output:"));
  }
#endif

  static byte nestingLevel = 0;
  int data = 0;
  String log = "";

  nestingLevel++;

  if (nestingLevel > RULES_MAX_NESTING_LEVEL) {
    addLog(LOG_LEVEL_ERROR, F("EVENT: Error: Nesting level exceeded!"));
    nestingLevel--;
    return log;
  }

  fs::File f = tryOpenFile(fileName, "r+");
  SPIFFS_CHECK(f, fileName.c_str());

  String line = "";
  bool match = false;
  bool codeBlock = false;
  bool isCommand = false;
  bool condition[RULES_IF_MAX_NESTING_LEVEL];
  bool ifBranche[RULES_IF_MAX_NESTING_LEVEL];
  byte ifBlock = 0;
  byte fakeIfBlock = 0;

  byte *buf = new byte[RULES_BUFFER_SIZE]();
  int len = 0;

  while (f.available()) {
    len = f.read((byte *)buf, RULES_BUFFER_SIZE);

    for (int x = 0; x < len; x++) {
      data = buf[x];

      SPIFFS_CHECK(data >= 0, fileName.c_str());

      if (data != 10) {
        line += char(data);
      } else {
        line.replace("\r", "");

        if ((line.substring(0, 2) != F("//")) && (line.length() > 0)) {
          parseCompleteNonCommentLine(line, event, log, match, codeBlock,
                                      isCommand, condition, ifBranche, ifBlock,
                                      fakeIfBlock);
          backgroundtasks();
        }

        line = "";
      }
    }
  }
  delete[] buf;

  if (f) {
    f.close();
  }

  nestingLevel--;
  checkRAM(F("rulesProcessingFile2"));
  return "";
}

void parseCompleteNonCommentLine(String& line, String& event, String& log,
                                 bool& match, bool& codeBlock, bool& isCommand,
                                 bool condition[], bool ifBranche[],
                                 byte& ifBlock, byte& fakeIfBlock) {
  isCommand = true;


  int comment = line.indexOf(F("//"));

  if (comment > 0) {
    line = line.substring(0, comment);
  }

  if (match || !codeBlock) {




    if (match && !fakeIfBlock) {


      if (event.charAt(0) == '!') {
        line.replace(F("%eventvalue%"), event);


      } else {
        int equalsPos = event.indexOf("=");

        if (equalsPos > 0) {
          String tmpString = event.substring(equalsPos + 1);



          String tmpParam;

          if (GetArgv(tmpString.c_str(), tmpParam, 1)) {
            line.replace(F("%eventvalue%"),
                         tmpParam);
            line.replace(F("%eventvalue1%"),
                         tmpParam);

          }

          if (GetArgv(tmpString.c_str(), tmpParam, 2)) {
            line.replace(F("%eventvalue2%"),
                         tmpParam);
          }


          if (GetArgv(tmpString.c_str(), tmpParam, 3)) {
            line.replace(F("%eventvalue3%"),
                         tmpParam);
          }


          if (GetArgv(tmpString.c_str(), tmpParam, 4)) {
            line.replace(F("%eventvalue4%"),
                         tmpParam);
          }


        }
      }
    }
    line = parseTemplate(line, line.length());
  }
  line.trim();

  String lineOrg = line;
  line.toLowerCase();

  String eventTrigger = "";
  String action = "";

  if (!codeBlock)

  {
    if (line.startsWith(F("on "))) {
      ifBlock = 0;
      fakeIfBlock = 0;
      line = line.substring(3);
      int split = line.indexOf(F(" do"));

      if (split != -1) {
        eventTrigger = line.substring(0, split);
        action = lineOrg.substring(split + 7);
        action.trim();
      }

      if (eventTrigger == "*") {
        match = true;
      }
      else {
        match = ruleMatch(event, eventTrigger);
      }

      if (action.length() > 0)
      {
        isCommand = true;
        codeBlock = false;
      } else {
        isCommand = false;
        codeBlock = true;
      }
    }
  } else {
    action = lineOrg;
  }

  String lcAction = action;
  lcAction.toLowerCase();

  if (lcAction == F("endon"))

  {
    isCommand = false;
    codeBlock = false;
    match = false;
    ifBlock = 0;
    fakeIfBlock = 0;
  }

#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
    String log = F("RuleDebug: ");
    log += codeBlock;
    log += match;
    log += isCommand;
    log += ": ";
    log += line;
    addLog(LOG_LEVEL_DEBUG_DEV, log);
  }
#endif

  if (match)
  {
    processMatchedRule(lcAction, action, event, log, match, codeBlock,
                       isCommand, condition, ifBranche, ifBlock, fakeIfBlock);
  }
}

void processMatchedRule(String& lcAction, String& action, String& event,
                        String& log, bool& match, bool& codeBlock,
                        bool& isCommand, bool condition[], bool ifBranche[],
                        byte& ifBlock, byte& fakeIfBlock) {
  if (fakeIfBlock) {
    isCommand = false;
  }
  else if (ifBlock) {
    if (condition[ifBlock - 1] != ifBranche[ifBlock - 1]) {
      isCommand = false;
    }
  }
  int split =
    lcAction.indexOf(F("elseif "));

  if (split != -1) {
    isCommand = false;

    if (ifBlock && !fakeIfBlock) {
      if (ifBranche[ifBlock - 1]) {
        if (condition[ifBlock - 1]) {
          ifBranche[ifBlock - 1] = false;
        }
        else {
          String check = lcAction.substring(split + 7);
          condition[ifBlock - 1] = conditionMatchExtended(check);
#ifndef BUILD_NO_DEBUG

          if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
            log = F("Lev.");
            log += String(ifBlock);
            log += F(": [elseif ");
            log += check;
            log += F("]=");
            log += toString(condition[ifBlock - 1]);
            addLog(LOG_LEVEL_DEBUG, log);
          }
#endif
        }
      }
    }
  } else {
    split = lcAction.indexOf(F("if "));

    if (split != -1) {
      if (ifBlock < RULES_IF_MAX_NESTING_LEVEL) {
        if (isCommand) {
          ifBlock++;
          String check = lcAction.substring(split + 3);
          condition[ifBlock - 1] = conditionMatchExtended(check);
          ifBranche[ifBlock - 1] = true;
#ifndef BUILD_NO_DEBUG

          if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
            log = F("Lev.");
            log += String(ifBlock);
            log += F(": [if ");
            log += check;
            log += F("]=");
            log += toString(condition[ifBlock - 1]);
            addLog(LOG_LEVEL_DEBUG, log);
          }
#endif
        } else {
          fakeIfBlock++;
        }
      } else {
        fakeIfBlock++;

        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          log = F("Lev.");
          log += String(ifBlock);
          log = F(": Error: IF Nesting level exceeded!");
          addLog(LOG_LEVEL_ERROR, log);
        }
      }
      isCommand = false;
    }
  }

  if ((lcAction == F("else")) && !fakeIfBlock)


  {
    ifBranche[ifBlock - 1] = false;
    isCommand = false;
#ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      log = F("Lev.");
      log += String(ifBlock);
      log += F(": [else]=");
      log += toString(condition[ifBlock - 1] == ifBranche[ifBlock - 1]);
      addLog(LOG_LEVEL_DEBUG, log);
    }
#endif
  }

  if (lcAction == F("endif"))
  {
    if (fakeIfBlock) {
      fakeIfBlock--;
    }
    else if (ifBlock) {
      ifBlock--;
    }
    isCommand = false;
  }



  if (isCommand) {
    if (event.charAt(0) == '!') {
      action.replace(F("%eventvalue%"), event);


    } else {
      int equalsPos = event.indexOf("=");

      if (equalsPos > 0) {
        String tmpString = event.substring(equalsPos + 1);

        String tmpParam;

        if (GetArgv(tmpString.c_str(), tmpParam, 1)) {
          action.replace(F("%eventvalue%"),
                         tmpParam);
          action.replace(F("%eventvalue1%"),
                         tmpParam);

        }

        if (GetArgv(tmpString.c_str(), tmpParam, 2)) {
          action.replace(F("%eventvalue2%"),
                         tmpParam);
        }


        if (GetArgv(tmpString.c_str(), tmpParam, 3)) {
          action.replace(F("%eventvalue3%"),
                         tmpParam);
        }


        if (GetArgv(tmpString.c_str(), tmpParam, 4)) {
          action.replace(F("%eventvalue4%"),
                         tmpParam);
        }


      }
    }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("ACT  : ");
      log += action;
      addLog(LOG_LEVEL_INFO, log);
    }

    struct EventStruct TempEvent;
    parseCommandString(&TempEvent, action);




    delay(0);



    String tmpAction(action);

    if (!PluginCall(PLUGIN_WRITE, &TempEvent, tmpAction)) {
      if (!tmpAction.equals(action)) {
        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          String log = F("PLUGIN_WRITE altered the string: ");
          log += action;
          log += F(" to: ");
          log += tmpAction;
          addLog(LOG_LEVEL_ERROR, log);
        }


      }
      ExecuteCommand(VALUE_SOURCE_SYSTEM, action.c_str());
    }
    delay(0);
  }
}




boolean ruleMatch(String& event, String& rule) {
  checkRAM(F("ruleMatch"));
  boolean match = false;
  String tmpEvent = event;
  String tmpRule = rule;


  tmpRule.replace("[", "");
  tmpRule.replace("]", "");


  if (event.charAt(0) == '!') {
    int pos = rule.indexOf('*');

    if (pos != -1)
    {
      tmpEvent = event.substring(0, pos - 1);
      tmpRule = rule.substring(0, pos - 1);
    } else {
      pos = rule.indexOf('#');

      if (pos ==
          -1)
      {
        tmpEvent = event.substring(0, rule.length());
        tmpRule = rule;
      }
    }

    if (tmpEvent.equalsIgnoreCase(tmpRule)) {
      return true;
    }
    else {
      return false;
    }
  }

  if (event.startsWith(
        F("Clock#Time")))
  {
    int pos1 = event.indexOf("=");
    int pos2 = rule.indexOf("=");

    if ((pos1 > 0) && (pos2 > 0)) {
      tmpEvent = event.substring(0, pos1);
      tmpRule = rule.substring(0, pos2);

      if (tmpRule.equalsIgnoreCase(tmpEvent))
      {
        tmpEvent = event.substring(pos1 + 1);
        tmpRule = rule.substring(pos2 + 1);
        unsigned long clockEvent = string2TimeLong(tmpEvent);
        unsigned long clockSet = string2TimeLong(tmpRule);

        if (matchClockEvent(clockEvent, clockSet)) {
          return true;
        }
        else {
          return false;
        }
      }
    }
  }


  float value = 0;
  int pos = event.indexOf("=");

  if (pos) {
    tmpEvent = event.substring(pos + 1);
    value = tmpEvent.toFloat();
    tmpEvent = event.substring(0, pos);
  }


  int comparePos = 0;
  char compare = ' ';
  comparePos = rule.indexOf(">");

  if (comparePos > 0) {
    compare = '>';
  } else {
    comparePos = rule.indexOf("<");

    if (comparePos > 0) {
      compare = '<';
    } else {
      comparePos = rule.indexOf("=");

      if (comparePos > 0) {
        compare = '=';
      }
    }
  }

  float ruleValue = 0;

  if (comparePos > 0) {
    tmpRule = rule.substring(comparePos + 1);
    ruleValue = tmpRule.toFloat();
    tmpRule = rule.substring(0, comparePos);
  }

  switch (compare) {
    case '>':

      if (tmpRule.equalsIgnoreCase(tmpEvent) && (value > ruleValue)) {
        match = true;
      }
      break;

    case '<':

      if (tmpRule.equalsIgnoreCase(tmpEvent) && (value < ruleValue)) {
        match = true;
      }
      break;

    case '=':

      if (tmpRule.equalsIgnoreCase(tmpEvent) && (value == ruleValue)) {
        match = true;
      }
      break;

    case ' ':

      if (tmpRule.equalsIgnoreCase(tmpEvent)) {
        match = true;
      }
      break;
  }
  checkRAM(F("ruleMatch2"));
  return match;
}




boolean conditionMatchExtended(String& check) {
  int condAnd = -1;
  int condOr = -1;
  boolean rightcond = false;
  boolean leftcond = conditionMatch(check);

  do {
    condAnd = check.indexOf(F(" and "));
    condOr = check.indexOf(F(" or "));

    if ((condAnd > 0) || (condOr > 0)) {
      if ((condAnd > 0) && (((condOr < 0) && (condOr < condAnd)) ||
                            ((condOr > 0) && (condOr > condAnd)))) {
        check = check.substring(condAnd + 5);
        rightcond = conditionMatch(check);
        leftcond = (leftcond && rightcond);
      } else {
        check = check.substring(condOr + 4);
        rightcond = conditionMatch(check);
        leftcond = (leftcond || rightcond);
      }
    }
  } while (condAnd > 0 || condOr > 0);
  return leftcond;
}

boolean conditionMatch(const String& check) {
  boolean match = false;

  char compare = ' ';

  int posStart = check.length();
  int posEnd = posStart;
  int comparePos = 0;

  if (((comparePos = check.indexOf("!=")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '!' + '=';
  }

  if (((comparePos = check.indexOf("<>")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '!' + '=';
  }

  if (((comparePos = check.indexOf(">=")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '>' + '=';
  }

  if (((comparePos = check.indexOf("<=")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '<' + '=';
  }

  if (((comparePos = check.indexOf("<")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 1;
    compare = '<';
  }

  if (((comparePos = check.indexOf(">")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 1;
    compare = '>';
  }

  if (((comparePos = check.indexOf("=")) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd = posStart + 1;
    compare = '=';
  }

  float Value1 = 0;
  float Value2 = 0;

  if (compare > ' ') {
    String tmpCheck1 = check.substring(0, posStart);
    String tmpCheck2 = check.substring(posEnd);

    if (!isFloat(tmpCheck1) || !isFloat(tmpCheck2)) {
      Value1 = timeStringToSeconds(tmpCheck1);
      Value2 = timeStringToSeconds(tmpCheck2);
    } else {
      Value1 = tmpCheck1.toFloat();
      Value2 = tmpCheck2.toFloat();
    }
  } else {
    return false;
  }

  switch (compare) {
    case '>' + '=':

      if (Value1 >= Value2) {
        match = true;
      }
      break;

    case '<' + '=':

      if (Value1 <= Value2) {
        match = true;
      }
      break;

    case '!' + '=':

      if (Value1 != Value2) {
        match = true;
      }
      break;

    case '>':

      if (Value1 > Value2) {
        match = true;
      }
      break;

    case '<':

      if (Value1 < Value2) {
        match = true;
      }
      break;

    case '=':

      if (Value1 == Value2) {
        match = true;
      }
      break;
  }
  return match;
}




void rulesTimers() {
  if (!Settings.UseRules) {
    return;
  }

  for (byte x = 0; x < RULES_TIMER_MAX; x++) {
    if (!RulesTimer[x].paused && (RulesTimer[x].timestamp != 0L))
    {
      if (timeOutReached(RulesTimer[x].timestamp))
      {
        RulesTimer[x].timestamp = 0L;
        String event = F("Rules#Timer=");
        event += x + 1;
        rulesProcessing(event);
      }
    }
  }
}




void createRuleEvents(byte TaskIndex) {
  if (!Settings.UseRules) {
    return;
  }
  LoadTaskSettings(TaskIndex);
  byte BaseVarIndex = TaskIndex * VARS_PER_TASK;
  byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[TaskIndex]);
  byte sensorType = Device[DeviceIndex].VType;

  for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++) {
    String eventString = getTaskDeviceName(TaskIndex);
    eventString += F("#");
    eventString += ExtraTaskSettings.TaskDeviceValueNames[varNr];
    eventString += F("=");

    if (sensorType == SENSOR_TYPE_LONG) {
      eventString += (unsigned long)UserVar[BaseVarIndex] +
                     ((unsigned long)UserVar[BaseVarIndex + 1] << 16);
    }
    else {
      eventString += UserVar[BaseVarIndex + varNr];
    }

    rulesProcessing(eventString);
  }
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyStatistics.ino"
# 45 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyStatistics.ino"
void jsonStatistics(bool clearStats) {
  bool firstPlugin = true;
  bool firstFunction = true;
  int currentPluginId = -1;

  stream_json_start_array(F("plugin"));

  for (auto& x: pluginStats) {
    if (!x.second.isEmpty()) {
      const int pluginId = x.first / 256;

      if (currentPluginId != pluginId) {

        currentPluginId = pluginId;

        if (!firstFunction) {

          stream_json_end_object_element(true);
        }

        if (!firstPlugin) {

          stream_json_end_array_element(true);
          stream_json_end_object_element(false);
        }


        stream_plugin_timing_stats_json(pluginId);
        firstFunction = true;
      } else {
        if (!firstFunction) {

          stream_json_end_object_element(false);
        }
      }

      unsigned long minVal, maxVal;
      unsigned int c = x.second.getMinMax(minVal, maxVal);
      stream_plugin_function_timing_stats_json(getPluginFunctionName(x.first % 256),
                                               c, minVal, maxVal, x.second.getAvg());

      if (clearStats) { x.second.reset(); }
      firstFunction = false;
      firstPlugin = false;
    }
  }
  stream_json_end_object_element(true);
  stream_json_end_array_element(true);
  stream_json_end_object_element(true);
  stream_json_end_array_element(true);
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyStorage.ino"




String FileError(int line, const char *fname)
{
  String err = F("FS   : Error while reading/writing ");

  err += fname;
  err += F(" in ");
  err += line;
  addLog(LOG_LEVEL_ERROR, err);
  return err;
}




void flashCount()
{
  if (RTC.flashDayCounter <= MAX_FLASHWRITES_PER_DAY) {
    RTC.flashDayCounter++;
  }
  RTC.flashCounter++;
  saveToRTC();
}

String flashGuard()
{
  checkRAM(F("flashGuard"));

  if (RTC.flashDayCounter > MAX_FLASHWRITES_PER_DAY)
  {
    String log = F("FS   : Daily flash write rate exceeded! (powercycle to reset this)");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  flashCount();
  return String();
}


#define FLASH_GUARD() { String flashErr = flashGuard(); \
                        if (flashErr.length()) return (flashErr); }


String appendLineToFile(const String& fname, const String& line) {
  return appendToFile(fname, reinterpret_cast<const uint8_t *>(line.c_str()), line.length());
}

String appendToFile(const String& fname, const uint8_t *data, unsigned int size) {
  fs::File f = tryOpenFile(fname, "a+");

  SPIFFS_CHECK(f, fname.c_str());
  SPIFFS_CHECK(f.write(data, size), fname.c_str());
  f.close();
  return "";
}

bool fileExists(const String& fname) {
  return SPIFFS.exists(fname);
}

fs::File tryOpenFile(const String& fname, const String& mode) {
  START_TIMER;
  fs::File f;

  if ((mode == "r") && !fileExists(fname)) {
    return f;
  }
  f = SPIFFS.open(fname, mode.c_str());
  STOP_TIMER(TRY_OPEN_FILE);
  return f;
}

bool tryDeleteFile(const String& fname) {
  if (fname.length() > 0)
  {
    bool res = SPIFFS.remove(fname);



    uint8_t retries = 3;

    while (retries > 0 && GarbageCollection()) {
      --retries;
    }
    return res;
  }
  return false;
}




String BuildFixes()
{
  checkRAM(F("BuildFixes"));
  serialPrintln(F("\nBuild changed!"));

  if (Settings.Build < 145)
  {
    String fname = F(FILE_NOTIFICATION);
    fs::File f = tryOpenFile(fname, "w");
    SPIFFS_CHECK(f, fname.c_str());

    if (f)
    {
      for (int x = 0; x < 4096; x++)
      {

        uint8_t zero_value = 0;
        SPIFFS_CHECK(f.write(&zero_value, 1), fname.c_str());
      }
      f.close();
    }
  }

  if (Settings.Build < 20101)
  {
    serialPrintln(F("Fix reset Pin"));
    Settings.Pin_Reset = -1;
  }

  if (Settings.Build < 20102) {


    serialPrintln(F("Fix settings with uninitalized data or corrupted by switching between versions"));
    Settings.UseRTOSMultitasking = false;
    Settings.Pin_Reset = -1;
    Settings.SyslogFacility = DEFAULT_SYSLOG_FACILITY;
    Settings.MQTTUseUnitNameAsClientId = DEFAULT_MQTT_USE_UNITNAME_AS_CLIENTID;
    Settings.StructSize = sizeof(Settings);
  }

  if (Settings.Build < 20103) {
    Settings.ResetFactoryDefaultPreference = 0;
    Settings.OldRulesEngine(DEFAULT_RULES_OLDENGINE);
  }


  Settings.Build = BUILD;
  return SaveSettings();
}




void fileSystemCheck()
{
  checkRAM(F("fileSystemCheck"));
  addLog(LOG_LEVEL_INFO, F("FS   : Mounting..."));

  if (SPIFFS.begin())
  {
    #if defined(ESP8266)
    fs::FSInfo fs_info;
    SPIFFS.info(fs_info);

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("FS   : Mount successful, used ");
      log = log + fs_info.usedBytes;
      log = log + F(" bytes of ");
      log = log + fs_info.totalBytes;
      addLog(LOG_LEVEL_INFO, log);
    }


    uint8_t retries = 3;

    while (retries > 0 && GarbageCollection()) {
      --retries;
    }
    #endif

    fs::File f = tryOpenFile(FILE_CONFIG, "r");

    if (!f)
    {
      ResetFactory();
    }

    if (f) { f.close(); }
  }
  else
  {
    String log = F("FS   : Mount failed");
    serialPrintln(log);
    addLog(LOG_LEVEL_ERROR, log);
    ResetFactory();
  }
}




bool GarbageCollection() {
  #ifdef CORE_POST_2_6_0


  START_TIMER;

  if (SPIFFS.gc()) {
    addLog(LOG_LEVEL_INFO, F("FS   : Success garbage collection"));
    STOP_TIMER(SPIFFS_GC_SUCCESS);
    return true;
  }
  STOP_TIMER(SPIFFS_GC_FAIL);
  return false;
  #else


  return false;
  #endif
}




String SaveSettings(void)
{
  checkRAM(F("SaveSettings"));
  MD5Builder md5;
  uint8_t tmp_md5[16] = { 0 };
  String err;

  Settings.StructSize = sizeof(struct SettingsStruct);
# 241 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyStorage.ino"
  Settings.validate();
  err = SaveToFile((char *)FILE_CONFIG, 0, (byte *)&Settings, sizeof(Settings));

  if (err.length()) {
    return err;
  }



  if (!SettingsCheck(err)) { return err; }



  SecuritySettings.validate();
  memcpy(SecuritySettings.ProgmemMd5, CRCValues.runTimeMD5, 16);
  md5.begin();
  md5.add((uint8_t *)&SecuritySettings, sizeof(SecuritySettings) - 16);
  md5.calculate();
  md5.getBytes(tmp_md5);

  if (memcmp(tmp_md5, SecuritySettings.md5, 16) != 0) {

    memcpy(SecuritySettings.md5, tmp_md5, 16);
    err = SaveToFile((char *)FILE_SECURITY, 0, (byte *)&SecuritySettings, sizeof(SecuritySettings));

    if (WifiIsAP(WiFi.getMode())) {

      wifiSetupConnect = true;
    }
  }
  afterloadSettings();
  return err;
}

void afterloadSettings() {
  ExtraTaskSettings.clear();
  ResetFactoryDefaultPreference_struct pref(Settings.ResetFactoryDefaultPreference);
  DeviceModel model = pref.getDeviceModel();




  if (modelMatchingFlashSize(model)) {
    ResetFactoryDefaultPreference = Settings.ResetFactoryDefaultPreference;
  }
  msecTimerHandler.setEcoMode(Settings.EcoPowerMode());
}




String LoadSettings()
{
  checkRAM(F("LoadSettings"));
  String err;
  uint8_t calculatedMd5[16];
  MD5Builder md5;

  err = LoadFromFile((char *)FILE_CONFIG, 0, (byte *)&Settings, sizeof(SettingsStruct));

  if (err.length()) {
    return err;
  }
  Settings.validate();
# 325 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyStorage.ino"
  err = LoadFromFile((char *)FILE_SECURITY, 0, (byte *)&SecuritySettings, sizeof(SecurityStruct));
  md5.begin();
  md5.add((uint8_t *)&SecuritySettings, sizeof(SecuritySettings) - 16);
  md5.calculate();
  md5.getBytes(calculatedMd5);

  if (memcmp(calculatedMd5, SecuritySettings.md5, 16) == 0) {
    addLog(LOG_LEVEL_INFO, F("CRC  : SecuritySettings CRC   ...OK "));

    if (memcmp(SecuritySettings.ProgmemMd5, CRCValues.runTimeMD5, 16) != 0) {
      addLog(LOG_LEVEL_INFO, F("CRC  : binary has changed since last save of Settings"));
    }
  }
  else {
    addLog(LOG_LEVEL_ERROR, F("CRC  : SecuritySettings CRC   ...FAIL"));
  }
  setupStaticIPconfig();
  afterloadSettings();
  SecuritySettings.validate();
  return err;
}




byte disablePlugin(byte bootFailedCount) {
  for (byte i = 0; i < TASKS_MAX && bootFailedCount > 0; ++i) {
    if (Settings.TaskDeviceEnabled[i]) {
      --bootFailedCount;

      if (bootFailedCount == 0) {
        Settings.TaskDeviceEnabled[i] = false;
      }
    }
  }
  return bootFailedCount;
}




byte disableController(byte bootFailedCount) {
  for (byte i = 0; i < CONTROLLER_MAX && bootFailedCount > 0; ++i) {
    if (Settings.ControllerEnabled[i]) {
      --bootFailedCount;

      if (bootFailedCount == 0) {
        Settings.ControllerEnabled[i] = false;
      }
    }
  }
  return bootFailedCount;
}




byte disableNotification(byte bootFailedCount) {
  for (byte i = 0; i < NOTIFICATION_MAX && bootFailedCount > 0; ++i) {
    if (Settings.NotificationEnabled[i]) {
      --bootFailedCount;

      if (bootFailedCount == 0) {
        Settings.NotificationEnabled[i] = false;
      }
    }
  }
  return bootFailedCount;
}




bool getSettingsParameters(SettingsType settingsType, int index, int& max_index, int& offset, int& max_size, int& struct_size) {
  struct_size = 0;

  switch (settingsType) {
    case BasicSettings_Type:
      max_index = 1;
      offset = 0;
      max_size = DAT_BASIC_SETTINGS_SIZE;
      struct_size = sizeof(SettingsStruct);
      break;
    case TaskSettings_Type:
      max_index = TASKS_MAX;
      offset = DAT_OFFSET_TASKS + (index * DAT_TASKS_DISTANCE);
      max_size = DAT_TASKS_SIZE;
      struct_size = sizeof(ExtraTaskSettingsStruct);
      break;
    case CustomTaskSettings_Type:
      max_index = TASKS_MAX;
      offset = DAT_OFFSET_TASKS + (index * DAT_TASKS_DISTANCE) + DAT_TASKS_CUSTOM_OFFSET;
      max_size = DAT_TASKS_CUSTOM_SIZE;


      break;
    case ControllerSettings_Type:
      max_index = CONTROLLER_MAX;
      offset = DAT_OFFSET_CONTROLLER + (index * DAT_CONTROLLER_SIZE);
      max_size = DAT_CONTROLLER_SIZE;
      struct_size = sizeof(ControllerSettingsStruct);
      break;
    case CustomControllerSettings_Type:
      max_index = CONTROLLER_MAX;
      offset = DAT_OFFSET_CUSTOM_CONTROLLER + (index * DAT_CUSTOM_CONTROLLER_SIZE);
      max_size = DAT_CUSTOM_CONTROLLER_SIZE;


      break;
    case NotificationSettings_Type:
      max_index = NOTIFICATION_MAX;
      offset = index * DAT_NOTIFICATION_SIZE;
      max_size = DAT_NOTIFICATION_SIZE;
      struct_size = sizeof(NotificationSettingsStruct);
      break;
    default:
      max_index = -1;
      offset = -1;
      return false;
  }
  return true;
}

int getMaxFilePos(SettingsType settingsType) {
  int max_index, offset, max_size;
  int struct_size = 0;

  getSettingsParameters(settingsType, 0, max_index, offset, max_size, struct_size);
  getSettingsParameters(settingsType, max_index - 1, offset, max_size);
  return offset + max_size - 1;
}

int getFileSize(SettingsType settingsType) {
  if (settingsType == NotificationSettings_Type) {
    return getMaxFilePos(settingsType);
  }

  int max_file_pos = 0;

  for (int st = 0; st < SettingsType_MAX; ++st) {
    int filePos = getMaxFilePos(static_cast<SettingsType>(st));

    if (filePos > max_file_pos) {
      max_file_pos = filePos;
    }
  }
  return max_file_pos;
}

bool getAndLogSettingsParameters(bool read, SettingsType settingsType, int index, int& offset, int& max_size) {
#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
    String log = read ? F("Read") : F("Write");
    log += F(" settings: ");
    log += getSettingsTypeString(settingsType);
    log += F(" index: ");
    log += index;
    addLog(LOG_LEVEL_DEBUG_DEV, log);
  }
#endif
  return getSettingsParameters(settingsType, index, offset, max_size);
}

bool getSettingsParameters(SettingsType settingsType, int index, int& offset, int& max_size) {
  int max_index = -1;
  int struct_size;

  if (!getSettingsParameters(settingsType, index, max_index, offset, max_size, struct_size)) {
    return false;
  }

  if ((index >= 0) && (index < max_index)) { return true; }
  offset = -1;
  return false;
}




String SaveTaskSettings(byte TaskIndex)
{
  checkRAM(F("SaveTaskSettings"));

  if (ExtraTaskSettings.TaskIndex != TaskIndex) {
    return F("SaveTaskSettings taskIndex does not match");
  }
  String err = SaveToFile(TaskSettings_Type,
                          TaskIndex,
                          (char *)FILE_CONFIG,
                          (byte *)&ExtraTaskSettings,
                          sizeof(struct ExtraTaskSettingsStruct));

  if (err.length() == 0) {
    err = checkTaskSettings(TaskIndex);
  }
  return err;
}




String LoadTaskSettings(byte TaskIndex)
{
  if ((TaskIndex < 0) || (TaskIndex >= TASKS_MAX)) {
    return String();
  }
  checkRAM(F("LoadTaskSettings"));

  if (ExtraTaskSettings.TaskIndex == TaskIndex) {
    return String();
  }
  START_TIMER
  ExtraTaskSettings.clear();
  String result = "";
  result =
    LoadFromFile(TaskSettings_Type, TaskIndex, (char *)FILE_CONFIG, (byte *)&ExtraTaskSettings, sizeof(struct ExtraTaskSettingsStruct));


  ExtraTaskSettings.TaskIndex = TaskIndex;

  if (ExtraTaskSettings.TaskDeviceValueNames[0][0] == 0) {

    struct EventStruct TempEvent;
    TempEvent.TaskIndex = TaskIndex;
    String dummyString;


    PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, dummyString);
  }
  ExtraTaskSettings.validate();
  STOP_TIMER(LOAD_TASK_SETTINGS);

  return result;
}




String SaveCustomTaskSettings(int TaskIndex, byte *memAddress, int datasize)
{
  checkRAM(F("SaveCustomTaskSettings"));
  return SaveToFile(CustomTaskSettings_Type, TaskIndex, (char *)FILE_CONFIG, memAddress, datasize);
}

String getCustomTaskSettingsError(byte varNr) {
  String error = F("Error: Text too long for line ");

  error += varNr + 1;
  error += '\n';
  return error;
}




String ClearCustomTaskSettings(int TaskIndex)
{

  return ClearInFile(CustomTaskSettings_Type, TaskIndex, (char *)FILE_CONFIG);
}




String LoadCustomTaskSettings(int TaskIndex, byte *memAddress, int datasize)
{
  START_TIMER;
  checkRAM(F("LoadCustomTaskSettings"));
  String result = LoadFromFile(CustomTaskSettings_Type, TaskIndex, (char *)FILE_CONFIG, memAddress, datasize);
  STOP_TIMER(LOAD_CUSTOM_TASK_STATS);
  return result;
}




String SaveControllerSettings(int ControllerIndex, ControllerSettingsStruct& controller_settings)
{
  checkRAM(F("SaveControllerSettings"));
  controller_settings.validate();
  return SaveToFile(ControllerSettings_Type, ControllerIndex,
                    (char *)FILE_CONFIG, (byte *)&controller_settings, sizeof(controller_settings));
}




String LoadControllerSettings(int ControllerIndex, ControllerSettingsStruct& controller_settings) {
  checkRAM(F("LoadControllerSettings"));
  String result =
    LoadFromFile(ControllerSettings_Type, ControllerIndex,
                 (char *)FILE_CONFIG, (byte *)&controller_settings, sizeof(controller_settings));
  controller_settings.validate();
  return result;
}




String ClearCustomControllerSettings(int ControllerIndex)
{
  checkRAM(F("ClearCustomControllerSettings"));


  return ClearInFile(CustomControllerSettings_Type, ControllerIndex, (char *)FILE_CONFIG);
}




String SaveCustomControllerSettings(int ControllerIndex, byte *memAddress, int datasize)
{
  checkRAM(F("SaveCustomControllerSettings"));
  return SaveToFile(CustomControllerSettings_Type, ControllerIndex, (char *)FILE_CONFIG, memAddress, datasize);
}




String LoadCustomControllerSettings(int ControllerIndex, byte *memAddress, int datasize)
{
  checkRAM(F("LoadCustomControllerSettings"));
  return LoadFromFile(CustomControllerSettings_Type, ControllerIndex, (char *)FILE_CONFIG, memAddress, datasize);
}




String SaveNotificationSettings(int NotificationIndex, byte *memAddress, int datasize)
{
  checkRAM(F("SaveNotificationSettings"));
  return SaveToFile(NotificationSettings_Type, NotificationIndex, (char *)FILE_NOTIFICATION, memAddress, datasize);
}




String LoadNotificationSettings(int NotificationIndex, byte *memAddress, int datasize)
{
  checkRAM(F("LoadNotificationSettings"));
  return LoadFromFile(NotificationSettings_Type, NotificationIndex, (char *)FILE_NOTIFICATION, memAddress, datasize);
}




String InitFile(const char *fname, int datasize)
{
  checkRAM(F("InitFile"));
  FLASH_GUARD();

  fs::File f = tryOpenFile(fname, "w");

  if (f) {
    SPIFFS_CHECK(f, fname);

    for (int x = 0; x < datasize; x++)
    {

      uint8_t zero_value = 0;
      SPIFFS_CHECK(f.write(&zero_value, 1), fname);
    }
    f.close();
  }


  return String();
}




String SaveToFile(char *fname, int index, byte *memAddress, int datasize)
{
#ifndef ESP32

  if (allocatedOnStack(memAddress)) {
    String log = F("SaveToFile: ");
    log += fname;
    log += F(" ERROR, Data allocated on stack");
    addLog(LOG_LEVEL_ERROR, log);


  }
#endif

  if (index < 0) {
    String log = F("SaveToFile: ");
    log += fname;
    log += F(" ERROR, invalid position in file");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  START_TIMER;
  checkRAM(F("SaveToFile"));
  FLASH_GUARD();
  {
    String log = F("SaveToFile: free stack: ");
    log += getCurrentFreeStack();
    addLog(LOG_LEVEL_INFO, log);
  }
  delay(1);
  unsigned long timer = millis() + 50;
  fs::File f = tryOpenFile(fname, "r+");

  if (f) {
    SPIFFS_CHECK(f, fname);
    SPIFFS_CHECK(f.seek(index, fs::SeekSet), fname);
    byte *pointerToByteToSave = memAddress;

    for (int x = 0; x < datasize; x++)
    {

      uint8_t byteToSave = *pointerToByteToSave;
      SPIFFS_CHECK(f.write(&byteToSave, 1), fname);
      pointerToByteToSave++;

      if (x % 256 == 0) {

        timer = millis() + 50;
        delay(0);
      }

      if (timeOutReached(timer)) {
        timer += 50;
        delay(0);
      }
    }
    f.close();
    String log = F("FILE : Saved ");
    log = log + fname;
    addLog(LOG_LEVEL_INFO, log);
  } else {
    String log = F("SaveToFile: ");
    log += fname;
    log += F(" ERROR, Cannot save to file");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  STOP_TIMER(SAVEFILE_STATS);
  {
    String log = F("SaveToFile: free stack after: ");
    log += getCurrentFreeStack();
    addLog(LOG_LEVEL_INFO, log);
  }


  return String();
}




String ClearInFile(char *fname, int index, int datasize)
{
  if (index < 0) {
    String log = F("ClearInFile: ");
    log += fname;
    log += F(" ERROR, invalid position in file");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }

  checkRAM(F("ClearInFile"));
  FLASH_GUARD();

  fs::File f = tryOpenFile(fname, "r+");

  if (f) {
    SPIFFS_CHECK(f, fname);

    SPIFFS_CHECK(f.seek(index, fs::SeekSet), fname);

    for (int x = 0; x < datasize; x++)
    {

      uint8_t zero_value = 0;
      SPIFFS_CHECK(f.write(&zero_value, 1), fname);
    }
    f.close();
  } else {
    String log = F("ClearInFile: ");
    log += fname;
    log += F(" ERROR, Cannot save to file");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }


  return String();
}




String LoadFromFile(char *fname, int offset, byte *memAddress, int datasize)
{
  if (offset < 0) {
    String log = F("LoadFromFile: ");
    log += fname;
    log += F(" ERROR, invalid position in file");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  delay(1);
  START_TIMER;

  checkRAM(F("LoadFromFile"));
  fs::File f = tryOpenFile(fname, "r");
  SPIFFS_CHECK(f, fname);
  SPIFFS_CHECK(f.seek(offset, fs::SeekSet), fname);
  SPIFFS_CHECK(f.read(memAddress, datasize), fname);
  f.close();

  STOP_TIMER(LOADFILE_STATS);
  delay(1);

  return String();
}




String getSettingsFileIndexRangeError(bool read, SettingsType settingsType, int index) {
  if (settingsType >= SettingsType_MAX) {
    String error = F("Unknown settingsType: ");
    error += static_cast<int>(settingsType);
    return error;
  }
  String error = read ? F("Load") : F("Save");
  error += getSettingsTypeString(settingsType);
  error += F(" index out of range: ");
  error += index;
  return error;
}

String getSettingsFileDatasizeError(bool read, SettingsType settingsType, int index, int datasize, int max_size) {
  String error = read ? F("Load") : F("Save");

  error += getSettingsTypeString(settingsType);
  error += '(';
  error += index;
  error += F(") datasize(");
  error += datasize;
  error += F(") > max_size(");
  error += max_size;
  error += ')';
  return error;
}

String LoadFromFile(SettingsType settingsType, int index, char *fname, byte *memAddress, int datasize) {
  bool read = true;
  int offset, max_size;

  if (!getAndLogSettingsParameters(read, settingsType, index, offset, max_size)) {
    return getSettingsFileIndexRangeError(read, settingsType, index);
  }

  if (datasize > max_size) {
    return getSettingsFileDatasizeError(read, settingsType, index, datasize, max_size);
  }
  return LoadFromFile(fname, offset, memAddress, datasize);
}

String SaveToFile(SettingsType settingsType, int index, char *fname, byte *memAddress, int datasize) {
  bool read = false;
  int offset, max_size;

  if (!getAndLogSettingsParameters(read, settingsType, index, offset, max_size)) {
    return getSettingsFileIndexRangeError(read, settingsType, index);
  }

  if (datasize > max_size) {
    return getSettingsFileDatasizeError(read, settingsType, index, datasize, max_size);
  }
  return SaveToFile(fname, offset, memAddress, datasize);
}

String ClearInFile(SettingsType settingsType, int index, char *fname) {
  bool read = false;
  int offset, max_size;

  if (!getAndLogSettingsParameters(read, settingsType, index, offset, max_size)) {
    return getSettingsFileIndexRangeError(read, settingsType, index);
  }
  return ClearInFile(fname, offset, max_size);
}




int SpiffsSectors()
{
  checkRAM(F("SpiffsSectors"));
  #if defined(ESP8266)
    # ifdef CORE_POST_2_6_0
  uint32_t _sectorStart = ((uint32_t)&_FS_start - 0x40200000) / SPI_FLASH_SEC_SIZE;
  uint32_t _sectorEnd = ((uint32_t)&_FS_end - 0x40200000) / SPI_FLASH_SEC_SIZE;
    # else
  uint32_t _sectorStart = ((uint32_t)&_SPIFFS_start - 0x40200000) / SPI_FLASH_SEC_SIZE;
  uint32_t _sectorEnd = ((uint32_t)&_SPIFFS_end - 0x40200000) / SPI_FLASH_SEC_SIZE;
    # endif

  return _sectorEnd - _sectorStart;
  #endif
  #if defined(ESP32)
  return 32;
  #endif
}

size_t SpiffsUsedBytes() {
  size_t result = 1;

  #ifdef ESP32
  result = SPIFFS.usedBytes();
  #endif
  #ifdef ESP8266
  fs::FSInfo fs_info;
  SPIFFS.info(fs_info);
  result = fs_info.usedBytes;
  #endif
  return result;
}

size_t SpiffsTotalBytes() {
  size_t result = 1;

  #ifdef ESP32
  result = SPIFFS.totalBytes();
  #endif
  #ifdef ESP8266
  fs::FSInfo fs_info;
  SPIFFS.info(fs_info);
  result = fs_info.totalBytes;
  #endif
  return result;
}

size_t SpiffsBlocksize() {
  size_t result = 8192;

  #ifdef ESP32
  result = 8192;
  #endif
  #ifdef ESP8266
  fs::FSInfo fs_info;
  SPIFFS.info(fs_info);
  result = fs_info.blockSize;
  #endif
  return result;
}

size_t SpiffsPagesize() {
  size_t result = 256;

  #ifdef ESP32
  result = 256;
  #endif
  #ifdef ESP8266
  fs::FSInfo fs_info;
  SPIFFS.info(fs_info);
  result = fs_info.pageSize;
  #endif
  return result;
}

size_t SpiffsFreeSpace() {
  int freeSpace = SpiffsTotalBytes() - SpiffsUsedBytes();

  if (freeSpace < static_cast<int>(2 * SpiffsBlocksize())) {


    return 0;
  }
  return freeSpace - 2 * SpiffsBlocksize();
}

bool SpiffsFull() {
  return SpiffsFreeSpace() == 0;
}




String createCacheFilename(unsigned int count) {
  String fname;

  fname.reserve(16);
  #ifdef ESP32
  fname = '/';
  #endif
  fname += "cache_";
  fname += String(count);
  fname += ".bin";
  return fname;
}


int getCacheFileCountFromFilename(const String& fname) {
  int startpos = fname.indexOf('_');

  if (startpos < 0) { return -1; }
  int endpos = fname.indexOf(F(".bin"));

  if (endpos < 0) { return -1; }
  String digits = fname.substring(startpos + 1, endpos);
  int result;

  if (validIntFromString(fname.substring(startpos + 1, endpos), result)) {
    return result;
  }
  return -1;
}



bool getCacheFileCounters(uint16_t& lowest, uint16_t& highest, size_t& filesizeHighest) {
  lowest = 65535;
  highest = 0;
  filesizeHighest = 0;
#ifdef ESP8266
  Dir dir = SPIFFS.openDir("cache");

  while (dir.next()) {
    String filename = dir.fileName();
    int count = getCacheFileCountFromFilename(filename);

    if (count >= 0) {
      if (lowest > count) {
        lowest = count;
      }

      if (highest < count) {
        highest = count;
        filesizeHighest = dir.fileSize();
      }
    }
  }
#endif
#ifdef ESP32
  File root = SPIFFS.open("/cache");
  File file = root.openNextFile();

  while (file)
  {
    if (!file.isDirectory()) {
      int count = getCacheFileCountFromFilename(file.name());

      if (count >= 0) {
        if (lowest > count) {
          lowest = count;
        }

        if (highest < count) {
          highest = count;
          filesizeHighest = file.size();
        }
      }
    }
    file = root.openNextFile();
  }
#endif

  if (lowest <= highest) {
    return true;
  }
  lowest = 0;
  highest = 0;
  return false;
}




#ifdef ESP32
String getPartitionType(byte pType, byte pSubType) {
  esp_partition_type_t partitionType = static_cast<esp_partition_type_t>(pType);
  esp_partition_subtype_t partitionSubType = static_cast<esp_partition_subtype_t>(pSubType);

  if (partitionType == ESP_PARTITION_TYPE_APP) {
    if ((partitionSubType >= ESP_PARTITION_SUBTYPE_APP_OTA_MIN) &&
        (partitionSubType < ESP_PARTITION_SUBTYPE_APP_OTA_MAX)) {
      String result = F("OTA partition ");
      result += (partitionSubType - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);
      return result;
    }

    switch (partitionSubType) {
      case ESP_PARTITION_SUBTYPE_APP_FACTORY: return F("Factory app");
      case ESP_PARTITION_SUBTYPE_APP_TEST: return F("Test app");
      default: break;
    }
  }

  if (partitionType == ESP_PARTITION_TYPE_DATA) {
    switch (partitionSubType) {
      case ESP_PARTITION_SUBTYPE_DATA_OTA: return F("OTA selection");
      case ESP_PARTITION_SUBTYPE_DATA_PHY: return F("PHY init data");
      case ESP_PARTITION_SUBTYPE_DATA_NVS: return F("NVS");
      case ESP_PARTITION_SUBTYPE_DATA_COREDUMP: return F("COREDUMP");
      case ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD: return F("ESPHTTPD");
      case ESP_PARTITION_SUBTYPE_DATA_FAT: return F("FAT");
      case ESP_PARTITION_SUBTYPE_DATA_SPIFFS: return F("SPIFFS");
      default: break;
    }
  }
  String result = F("Unknown(");
  result += partitionSubType;
  result += ')';
  return result;
}

String getPartitionTableHeader(const String& itemSep, const String& lineEnd) {
  String result;

  result += F("Address");
  result += itemSep;
  result += F("Size");
  result += itemSep;
  result += F("Label");
  result += itemSep;
  result += F("Partition Type");
  result += itemSep;
  result += F("Encrypted");
  result += lineEnd;
  return result;
}

String getPartitionTable(byte pType, const String& itemSep, const String& lineEnd) {
  esp_partition_type_t partitionType = static_cast<esp_partition_type_t>(pType);
  String result;
  const esp_partition_t *_mypart;
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, ESP_PARTITION_SUBTYPE_ANY, NULL);

  if (_mypartiterator) {
    do {
      _mypart = esp_partition_get(_mypartiterator);
      result += formatToHex(_mypart->address);
      result += itemSep;
      result += formatToHex_decimal(_mypart->size, 1024);
      result += itemSep;
      result += _mypart->label;
      result += itemSep;
      result += getPartitionType(_mypart->type, _mypart->subtype);
      result += itemSep;
      result += (_mypart->encrypted ? F("Yes") : F("-"));
      result += lineEnd;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != NULL);
  }
  esp_partition_iterator_release(_mypartiterator);
  return result;
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyWifi.ino"
#define WIFI_RECONNECT_WAIT 20000
#define WIFI_AP_OFF_TIMER_DURATION 60000

#ifdef CORE_POST_2_5_0
# include <AddrList.h>
#endif

bool unprocessedWifiEvents() {
  if (processedConnect && processedGetIP && processedDisconnect && processedDHCPTimeout) { return false; }
  return true;
}





void processConnect() {
  if (processedConnect) { return; }
  delay(100);
  ++wifi_reconnects;


  const long connect_duration = timeDiff(last_wifi_connect_attempt_moment, lastConnectMoment);

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("WIFI : Connected! AP: ");
    log += WiFi.SSID();
    log += " (";
    log += WiFi.BSSIDstr();
    log += F(") Ch: ");
    log += last_channel;

    if ((connect_duration > 0) && (connect_duration < 30000)) {

      log += F(" Duration: ");
      log += connect_duration;
      log += F(" ms");
    }
    addLog(LOG_LEVEL_INFO, log);
  }

  if (Settings.UseRules && bssid_changed) {
    String event = F("WiFi#ChangedAccesspoint");
    rulesProcessing(event);
  }

  if (useStaticIP()) {
    setupStaticIPconfig();
    markGotIP();
  }

  if (!WiFi.getAutoConnect()) {
    WiFi.setAutoConnect(true);
  }
  logConnectionStatus();
  processedConnect = true;
}

void processDisconnect() {
  if (processedDisconnect) { return; }
  delay(100);

  if (Settings.UseRules) {
    String event = F("WiFi#Disconnected");
    rulesProcessing(event);
  }

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("WIFI : Disconnected! Reason: '");
    log += getLastDisconnectReason();
    log += '\'';

    if (lastConnectedDuration > 0) {
      log += F(" Connected for ");
      log += format_msec_duration(lastConnectedDuration);
    }
    addLog(LOG_LEVEL_INFO, log);
  }
  #ifdef ESP8266
  WiFiUDP::stopAll();
  WiFiClient::stopAll();
  #endif
  mqtt_reconnect_count = 0;

  if (Settings.WiFiRestart_connection_lost()) {
    WifiDisconnect();
  }
  logConnectionStatus();
  processedDisconnect = true;
}

void processGotIP() {
  if (processedGetIP) {
    return;
  }
  IPAddress ip = WiFi.localIP();

  if (!useStaticIP()) {
    if ((ip[0] == 0) && (ip[1] == 0) && (ip[2] == 0) && (ip[3] == 0)) {
      return;
    }
  }
  processedGetIP = true;
  const IPAddress gw = WiFi.gatewayIP();
  const IPAddress subnet = WiFi.subnetMask();
  const long dhcp_duration = timeDiff(lastConnectMoment, lastGetIPmoment);

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("WIFI : ");

    if (useStaticIP()) {
      log += F("Static IP: ");
    } else {
      log += F("DHCP IP: ");
    }
    log += formatIP(ip);
    log += " (";
    log += WifiGetHostname();
    log += F(") GW: ");
    log += formatIP(gw);
    log += F(" SN: ");
    log += formatIP(subnet);

    if ((dhcp_duration > 0) && (dhcp_duration < 30000)) {

      log += F("   duration: ");
      log += dhcp_duration;
      log += F(" ms");
    }
    addLog(LOG_LEVEL_INFO, log);
    WiFi.setAutoReconnect(true);
  }



  if ((Settings.IP_Octet != 0) && (Settings.IP_Octet != 255))
  {
    ip[3] = Settings.IP_Octet;

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("IP   : Fixed IP octet:");
      log += formatIP(ip);
      addLog(LOG_LEVEL_INFO, log);
    }
    WiFi.config(ip, gw, subnet);
  }

  #ifdef FEATURE_MDNS
  addLog(LOG_LEVEL_INFO, F("WIFI : Starting mDNS..."));
  bool mdns_started = MDNS.begin(WifiGetHostname().c_str());

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("WIFI : ");

    if (mdns_started) {
      log += F("mDNS started, with name: ");
      log += WifiGetHostname();
      log += F(".local");
    }
    else {
      log += F("mDNS failed");
    }
    addLog(LOG_LEVEL_INFO, log);
  }
  #endif


  if (systemTimePresent()) {
    initTime();
  }
  mqtt_reconnect_count = 0;
  MQTTclient_should_reconnect = true;
  timermqtt_interval = 100;
  setIntervalTimer(TIMER_MQTT);
  sendGratuitousARP_now();

  if (Settings.UseRules)
  {
    String event = F("WiFi#Connected");
    rulesProcessing(event);
  }
  statusLED(true);



  setWebserverRunning(true);
  #ifdef FEATURE_MDNS

  if (mdns_started) {
    MDNS.addService("http", "tcp", 80);
  }
  #endif
  wifi_connect_attempt = 0;

  if (wifiSetup) {

    wifiSetup = false;
    SaveSettings();
  }
  logConnectionStatus();
}

void processConnectAPmode() {
  if (processedConnectAPmode) { return; }
  processedConnectAPmode = true;

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("AP Mode: Client connected: ");
    log += formatMAC(lastMacConnectedAPmode);
    log += F(" Connected devices: ");
    log += WiFi.softAPgetStationNum();
    addLog(LOG_LEVEL_INFO, log);
  }
  timerAPoff = 0;
  setWebserverRunning(true);




  if (!dnsServerActive) {
    dnsServerActive = true;
    dnsServer.start(DNS_PORT, "*", apIP);
  }
}

void processDisconnectAPmode() {
  if (processedDisconnectAPmode) { return; }
  processedDisconnectAPmode = true;
  const int nrStationsConnected = WiFi.softAPgetStationNum();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("AP Mode: Client disconnected: ");
    log += formatMAC(lastMacDisconnectedAPmode);
    log += F(" Connected devices: ");
    log += nrStationsConnected;
    addLog(LOG_LEVEL_INFO, log);
  }

  if (nrStationsConnected == 0) {
    timerAPoff = millis() + WIFI_AP_OFF_TIMER_DURATION;
  }
}

void processDisableAPmode() {
  if (timerAPoff == 0) { return; }

  if (WifiIsAP(WiFi.getMode())) {

    if (timeOutReached(timerAPoff)) {
      setAP(false);
    }
  }
}

void processScanDone() {
  if (processedScanDone) { return; }
  processedScanDone = true;

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("WIFI  : Scan finished, found: ");
    log += scan_done_number;
    addLog(LOG_LEVEL_INFO, log);
  }

  int bestScanID = -1;
  int32_t bestRssi = -1000;
  uint8_t bestWiFiSettings = lastWiFiSettings;

  if (selectValidWiFiSettings()) {
    bool done = false;
    String lastWiFiSettingsSSID = getLastWiFiSettingsSSID();

    for (int settingNr = 0; !done && settingNr < 2; ++settingNr) {
      for (int i = 0; i < scan_done_number; ++i) {
        if (WiFi.SSID(i) == lastWiFiSettingsSSID) {
          int32_t rssi = WiFi.RSSI(i);

          if (bestRssi < rssi) {
            bestRssi = rssi;
            bestScanID = i;
            bestWiFiSettings = lastWiFiSettings;
          }
        }
      }

      if (!selectNextWiFiSettings()) { done = true; }
    }

    if (bestScanID >= 0) {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("WIFI  : Selected: ");
        log += formatScanResult(bestScanID, " ");
        addLog(LOG_LEVEL_INFO, log);
      }
      lastWiFiSettings = bestWiFiSettings;
      uint8_t *scanbssid = WiFi.BSSID(bestScanID);

      if (scanbssid) {
        for (int i = 0; i < 6; ++i) {
          lastBSSID[i] = *(scanbssid + i);
        }
      }
    }
  }
}

void resetWiFi() {
  addLog(LOG_LEVEL_INFO, F("Reset WiFi."));
  lastDisconnectMoment = millis();
  WifiDisconnect();



#ifdef ESP8266


  WiFi.~ESP8266WiFiClass();
  WiFi = ESP8266WiFiClass();
#endif
}

bool hasIPaddr() {
#ifdef CORE_POST_2_5_0
  bool configured = false;

  for (auto addr : addrList) {
    if ((configured = !addr.isLocal() && (addr.ifnumber() == STATION_IF))) {






      break;
    }
  }
  return configured;
#else
  return WiFi.isConnected();
#endif
}

void connectionCheckHandler()
{
  if ((reconnectChecker == false) && !WiFiConnected()) {
    reconnectChecker = true;
    resetWiFi();
    WiFi.reconnect();
  }
  else if (WiFiConnected() && (reconnectChecker == true)) {
    reconnectChecker = false;
  }
}

void WifiScanAsync() {
  addLog(LOG_LEVEL_INFO, F("WIFI  : Start network scan"));
  #ifdef ESP32
  bool async = true;
  bool show_hidden = false;
  bool passive = false;
  uint32_t max_ms_per_chan = 300;
  WiFi.scanNetworks(async, show_hidden, passive, max_ms_per_chan);
  #else



  #endif
}
# 384 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasyWifi.ino"
bool WifiIsAP(WiFiMode_t wifimode)
{
  #if defined(ESP32)
  return (wifimode & WIFI_MODE_AP) != 0;
  #else
  return (wifimode & WIFI_AP) != 0;
  #endif
}

bool WifiIsSTA(WiFiMode_t wifimode)
{
  #if defined(ESP32)
  return (wifimode & WIFI_MODE_STA) != 0;
  #else
  return (wifimode & WIFI_STA) != 0;
  #endif
}




void setSTA(bool enable) {
  switch (WiFi.getMode()) {
    case WIFI_OFF:

      if (enable) { setWifiMode(WIFI_STA); }
      break;
    case WIFI_STA:

      if (!enable) { setWifiMode(WIFI_OFF); }
      break;
    case WIFI_AP:

      if (enable) { setWifiMode(WIFI_AP_STA); }
      break;
    case WIFI_AP_STA:

      if (!enable) { setWifiMode(WIFI_AP); }
      break;
    default:
      break;
  }
}

void setAP(bool enable) {
  WiFiMode_t wifimode = WiFi.getMode();

  if (WifiIsAP(wifimode) == enable) {
    return;
  }

  switch (wifimode) {
    case WIFI_OFF:

      if (enable) { setWifiMode(WIFI_AP); }
      break;
    case WIFI_STA:

      if (enable) { setWifiMode(WIFI_AP_STA); }
      break;
    case WIFI_AP:

      if (!enable) { setWifiMode(WIFI_OFF); }
      break;
    case WIFI_AP_STA:

      if (!enable) { setWifiMode(WIFI_STA); }
      break;
    default:
      addLog(LOG_LEVEL_INFO, F("WIFI : Unknown mode (setAP)"));
      break;
  }
}


void setAPinternal(bool enable)
{
  if (enable) {


    String softAPSSID = WifiGetAPssid();
    String pwd = SecuritySettings.WifiAPKey;
    IPAddress subnet(DEFAULT_AP_SUBNET);

    if (!WiFi.softAPConfig(apIP, apIP, subnet)) {
      addLog(LOG_LEVEL_ERROR, F("WIFI : [AP] softAPConfig failed!"));
    }

    if (WiFi.softAP(softAPSSID.c_str(), pwd.c_str())) {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String event = F("WiFi#APmodeEnabled");
        rulesProcessing(event);
        String log(F("WIFI : AP Mode ssid will be "));
        log += softAPSSID;
        log += F(" with address ");
        log += WiFi.softAPIP().toString();
        addLog(LOG_LEVEL_INFO, log);
      }
    } else {
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log(F("WIFI : Error while starting AP Mode with SSID: "));
        log += softAPSSID;
        log += F(" IP: ");
        log += apIP.toString();
        addLog(LOG_LEVEL_ERROR, log);
      }
    }
    #ifdef ESP32

    #else

    if (wifi_softap_dhcps_status() != DHCP_STARTED) {
      if (!wifi_softap_dhcps_start()) {
        addLog(LOG_LEVEL_ERROR, F("WIFI : [AP] wifi_softap_dhcps_start failed!"));
      }
    }
    #endif
  } else {
    timerAPoff = 0;

    if (dnsServerActive) {
      dnsServerActive = false;
      dnsServer.stop();
    }
  }
}

void setWifiMode(WiFiMode_t wifimode) {
  const WiFiMode_t cur_mode = WiFi.getMode();

  if (cur_mode == wifimode) {
    return;
  }
  String log = F("WIFI : Current mode ");
  log += String(cur_mode);
  addLog(LOG_LEVEL_INFO, log);

  switch (wifimode) {
    case WIFI_OFF:
      addLog(LOG_LEVEL_INFO, F("WIFI : Switch off WiFi"));
      break;
    case WIFI_STA:
      addLog(LOG_LEVEL_INFO, F("WIFI : Set WiFi to STA"));
      break;
    case WIFI_AP:
      addLog(LOG_LEVEL_INFO, F("WIFI : Set WiFi to AP"));
      break;
    case WIFI_AP_STA:
      addLog(LOG_LEVEL_INFO, F("WIFI : Set WiFi to AP+STA"));
      break;
    default:
      addLog(LOG_LEVEL_INFO, F("WIFI : Unknown mode"));
      break;
  }

  if (!WiFi.mode(wifimode)) {
    addLog(LOG_LEVEL_INFO, F("WIFI : Cannot set mode!!!!!"));
  }
  setupStaticIPconfig();
  delay(30);
  bool new_mode_AP_enabled = WifiIsAP(wifimode);

  if (WifiIsAP(cur_mode) && !new_mode_AP_enabled) {
    String event = F("WiFi#APmodeDisabled");
    rulesProcessing(event);
  }

  if (WifiIsAP(cur_mode) != new_mode_AP_enabled) {

    setAPinternal(new_mode_AP_enabled);
  }
}




String WifiGetAPssid()
{
  String ssid(Settings.Name);

  if (Settings.appendUnitToHostname()) {
    ssid += "_";
    ssid += Settings.Unit;
  }
  return ssid;
}




String WifiGetHostname()
{
  String hostname(WifiGetAPssid());

  hostname.replace(" ", "-");
  hostname.replace("_", "-");
  return hostname;
}

bool useStaticIP() {
  return Settings.IP[0] != 0 && Settings.IP[0] != 255;
}

bool WiFiConnected() {
  START_TIMER;

  if (unprocessedWifiEvents()) { return false; }


  if (wifiStatus == ESPEASY_WIFI_SERVICES_INITIALIZED) {
    if ((WiFi.RSSI() < 0) && WiFi.isConnected()) {
      timerAPstart = 0;
      processDisableAPmode();
      STOP_TIMER(WIFI_ISCONNECTED_STATS);
      return true;
    }


    addLog(LOG_LEVEL_INFO, F("WIFI  : WiFiConnected() out of sync"));
    resetWiFi();
  } else {
    if (hasIPaddr()) {
      wifiStatus = ESPEASY_WIFI_SERVICES_INITIALIZED;
    }
  }

  if (timerAPstart == 0) {
    timerAPstart = millis() + WIFI_RECONNECT_WAIT;
  }

  if (timeOutReached(timerAPstart)) {
    setAP(true);
  }
  delay(1);
  STOP_TIMER(WIFI_NOTCONNECTED_STATS);
  return false;
}

void WiFiConnectRelaxed() {
  if (WiFiConnected()) {
    return;
  }

  if (prepareWiFi()) {
    if (selectValidWiFiSettings()) {
      tryConnectWiFi();
      unsigned long timeout = millis() + 5000;

      while (WiFi.status() == WL_DISCONNECTED && !timeOutReached(timeout)) {
        addLog(LOG_LEVEL_INFO, F("Waiting for WiFi connect"));
        process_serialWriteBuffer();
        delay(100);
      }

      return;
    }
  }
  addLog(LOG_LEVEL_ERROR, F("WIFI : Could not connect to AP!"));


  setAP(true);
}




bool prepareWiFi() {
  if (!selectValidWiFiSettings()) {
    addLog(LOG_LEVEL_ERROR, F("WIFI : No valid wifi settings"));
    return false;
  }
  setSTA(true);
  delay(100);
  #ifdef ESP8266
  WiFi.forceSleepWake();
  #endif
  char hostname[40];
  safe_strncpy(hostname, WifiGetHostname().c_str(), sizeof(hostname));
  #if defined(ESP8266)
  wifi_station_set_hostname(hostname);

  if (Settings.WifiNoneSleep()) {


    WiFi.setSleepMode(WIFI_NONE_SLEEP);
  }
  #endif
  #if defined(ESP32)
  WiFi.setHostname(hostname);
  #endif
  return true;
}




const char* getLastWiFiSettingsSSID() {
  return lastWiFiSettings == 0 ? SecuritySettings.WifiSSID : SecuritySettings.WifiSSID2;
}

const char* getLastWiFiSettingsPassphrase() {
  return lastWiFiSettings == 0 ? SecuritySettings.WifiKey : SecuritySettings.WifiKey2;
}

bool selectNextWiFiSettings() {
  uint8_t tmp = lastWiFiSettings;

  lastWiFiSettings = (lastWiFiSettings + 1) % 2;

  if (!wifiSettingsValid(getLastWiFiSettingsSSID(), getLastWiFiSettingsPassphrase())) {

    lastWiFiSettings = tmp;
    return false;
  }
  return true;
}

bool selectValidWiFiSettings() {
  if (wifiSettingsValid(getLastWiFiSettingsSSID(), getLastWiFiSettingsPassphrase())) {
    return true;
  }
  return selectNextWiFiSettings();
}

bool wifiSettingsValid(const char *ssid, const char *pass) {
  if ((ssid[0] == 0) || (strcasecmp(ssid, "ssid") == 0)) {
    return false;
  }


  if (strlen(ssid) > 32) { return false; }

  if (strlen(pass) > 64) { return false; }
  return true;
}

bool wifiConnectTimeoutReached() {
  if (wifi_connect_attempt == 0) { return true; }

  if (timeDiff(last_wifi_connect_attempt_moment, lastDisconnectMoment) > 0) {

    return true;
  }

  if (WifiIsAP(WiFi.getMode())) {

    return timeOutReached(last_wifi_connect_attempt_moment + 20000);
  }



  #if defined(ESP8266)
  const unsigned int randomOffset_in_sec = (wifi_connect_attempt == 1) ? 0 : 1000 * ((ESP.getChipId() & 0xF));
  #endif
  #if defined(ESP32)
  const unsigned int randomOffset_in_sec = (wifi_connect_attempt == 1) ? 0 : 1000 * ((ESP.getEfuseMac() & 0xF));
  #endif
  return timeOutReached(last_wifi_connect_attempt_moment + DEFAULT_WIFI_CONNECTION_TIMEOUT + randomOffset_in_sec);
}

void setConnectionSpeed() {
  #ifdef ESP8266

  if (!Settings.ForceWiFi_bg_mode() || (wifi_connect_attempt > 10)) {
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  } else {
    WiFi.setPhyMode(WIFI_PHY_MODE_11G);
  }
  #endif


  #ifdef ESP32
  uint8_t protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;

  if (!Settings.ForceWiFi_bg_mode() || (wifi_connect_attempt > 10)) {

    protocol |= WIFI_PROTOCOL_11N;
  }

  if (WifiIsSTA(WiFi.getMode())) {
    esp_wifi_set_protocol(WIFI_IF_STA, protocol);
  }

  if (WifiIsAP(WiFi.getMode())) {
    esp_wifi_set_protocol(WIFI_IF_AP, protocol);
  }
  #endif
}

void setupStaticIPconfig() {
  setUseStaticIP(useStaticIP());

  if (!useStaticIP()) {



    return;
  }
  const IPAddress ip = Settings.IP;
  const IPAddress gw = Settings.Gateway;
  const IPAddress subnet = Settings.Subnet;
  const IPAddress dns = Settings.DNS;

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("IP   : Static IP : ");
    log += formatIP(ip);
    log += F(" GW: ");
    log += formatIP(gw);
    log += F(" SN: ");
    log += formatIP(subnet);
    log += F(" DNS: ");
    log += formatIP(dns);
    addLog(LOG_LEVEL_INFO, log);
  }
  WiFi.config(ip, gw, subnet, dns);
}




bool tryConnectWiFi() {
  if (wifiSetupConnect) {
    lastWiFiSettings = 0;
    wifi_connect_attempt = 0;
  }

  if (wifiStatus != ESPEASY_WIFI_DISCONNECTED) {
    if (!WifiIsAP(WiFi.getMode())) {

      return true;
    }
  }

  if (!wifiConnectTimeoutReached()) {
    delay(0);
    return true;
  }

  if (!selectValidWiFiSettings()) {
    addLog(LOG_LEVEL_ERROR, F("WIFI : No valid WiFi settings!"));
    return false;
  }

  if ((wifi_connect_attempt != 0) && ((wifi_connect_attempt % 2) == 0)) {
    selectNextWiFiSettings();
  }

  if (wifi_connect_attempt > 5) {
    setAP(true);
  }
  const char *ssid = getLastWiFiSettingsSSID();
  const char *passphrase = getLastWiFiSettingsPassphrase();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("WIFI : Connecting ");
    log += ssid;
    log += F(" attempt #");
    log += wifi_connect_attempt;
    addLog(LOG_LEVEL_INFO, log);
  }
  setupStaticIPconfig();
  setConnectionSpeed();
  last_wifi_connect_attempt_moment = millis();

  switch (wifi_connect_attempt) {
    case 0:

      if (lastBSSID[0] == 0) {
        WiFi.begin(ssid, passphrase);
      }
      else {
        WiFi.begin(ssid, passphrase, last_channel, &lastBSSID[0]);
      }
      break;
    default:
      WiFi.begin(ssid, passphrase);
  }
  delay(100);
  ++wifi_connect_attempt;
  logConnectionStatus();

  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL: {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("WIFI : No SSID found matching: ");
        log += ssid;
        addLog(LOG_LEVEL_INFO, log);
      }
      break;
    }
    case WL_CONNECT_FAILED: {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("WIFI : Connection failed to: ");
        log += ssid;
        addLog(LOG_LEVEL_INFO, log);
      }
      break;
    }
    case WL_DISCONNECTED: {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("WIFI : WiFi.status() = WL_DISCONNECTED  SSID: ");
        log += ssid;
        addLog(LOG_LEVEL_INFO, log);
      }
      break;
    }
    case WL_IDLE_STATUS: {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("WIFI : Connection in IDLE state: ");
        log += ssid;
        addLog(LOG_LEVEL_INFO, log);
      }
      break;
    }
    case WL_CONNECTED: {
      return true;
    }
    default:
      break;
  }
  return false;
}




void WifiDisconnect()
{
  #if defined(ESP32)
  WiFi.disconnect();
  #else
  ETS_UART_INTR_DISABLE();
  wifi_station_disconnect();
  ETS_UART_INTR_ENABLE();
  #endif
  wifiStatus = ESPEASY_WIFI_DISCONNECTED;


  setSTA(false);
}




void WifiScan()
{

  serialPrintln(F("WIFI : SSID Scan start"));
  int n = WiFi.scanNetworks(false, true);

  if (n == 0) {
    serialPrintln(F("WIFI : No networks found"));
  }
  else
  {
    serialPrint( F("WIFI : "));
    serialPrint(String(n));
    serialPrintln(F(" networks found"));

    for (int i = 0; i < n; ++i)
    {

      serialPrint( F("WIFI : "));
      serialPrint(String(i + 1));
      serialPrint( ": ");
      serialPrintln(formatScanResult(i, " "));
      delay(10);
    }
  }
  serialPrintln("");
}

String formatScanResult(int i, const String& separator) {
  String result = WiFi.SSID(i);

  htmlEscape(result);
  #ifndef ESP32

  if (WiFi.isHidden(i)) {
    result += F("#Hidden#");
  }
  #endif
  result += separator;
  result += WiFi.BSSIDstr(i);
  result += separator;
  result += F("Ch:");
  result += WiFi.channel(i);
  result += " (";
  result += WiFi.RSSI(i);
  result += F("dBm) ");

  switch (WiFi.encryptionType(i)) {
  #ifdef ESP32
    case WIFI_AUTH_OPEN: result += F("open"); break;
    case WIFI_AUTH_WEP: result += F("WEP"); break;
    case WIFI_AUTH_WPA_PSK: result += F("WPA/PSK"); break;
    case WIFI_AUTH_WPA2_PSK: result += F("WPA2/PSK"); break;
    case WIFI_AUTH_WPA_WPA2_PSK: result += F("WPA/WPA2/PSK"); break;
    case WIFI_AUTH_WPA2_ENTERPRISE: result += F("WPA2 Enterprise"); break;
  #else
    case ENC_TYPE_WEP: result += F("WEP"); break;
    case ENC_TYPE_TKIP: result += F("WPA/PSK"); break;
    case ENC_TYPE_CCMP: result += F("WPA2/PSK"); break;
    case ENC_TYPE_NONE: result += F("open"); break;
    case ENC_TYPE_AUTO: result += F("WPA/WPA2/PSK"); break;
  #endif
    default:
      break;
  }
  return result;
}

#ifndef ESP32
String SDKwifiStatusToString(uint8_t sdk_wifistatus) {
  switch (sdk_wifistatus) {
    case STATION_IDLE: return F("STATION_IDLE");
    case STATION_CONNECTING: return F("STATION_CONNECTING");
    case STATION_WRONG_PASSWORD: return F("STATION_WRONG_PASSWORD");
    case STATION_NO_AP_FOUND: return F("STATION_NO_AP_FOUND");
    case STATION_CONNECT_FAIL: return F("STATION_CONNECT_FAIL");
    case STATION_GOT_IP: return F("STATION_GOT_IP");
  }
  return getUnknownString();
}

#endif

String ArduinoWifiStatusToString(uint8_t arduino_corelib_wifistatus) {
  String log;

  switch (arduino_corelib_wifistatus) {
    case WL_IDLE_STATUS: log += F("WL_IDLE_STATUS"); break;
    case WL_NO_SSID_AVAIL: log += F("WL_NO_SSID_AVAIL"); break;
    case WL_SCAN_COMPLETED: log += F("WL_SCAN_COMPLETED"); break;
    case WL_CONNECTED: log += F("WL_CONNECTED"); break;
    case WL_CONNECT_FAILED: log += F("WL_CONNECT_FAILED"); break;
    case WL_CONNECTION_LOST: log += F("WL_CONNECTION_LOST"); break;
    case WL_DISCONNECTED: log += F("WL_DISCONNECTED"); break;
    default: log += arduino_corelib_wifistatus; break;
  }
  return log;
}

String ESPeasyWifiStatusToString() {
  String log;

  switch (wifiStatus) {
    case ESPEASY_WIFI_DISCONNECTED: log += F("ESPEASY_WIFI_DISCONNECTED"); break;
    case ESPEASY_WIFI_CONNECTED: log += F("ESPEASY_WIFI_CONNECTED"); break;
    case ESPEASY_WIFI_GOT_IP: log += F("ESPEASY_WIFI_GOT_IP"); break;
    case ESPEASY_WIFI_SERVICES_INITIALIZED: log += F("ESPEASY_WIFI_SERVICES_INITIALIZED"); break;
    default: log += wifiStatus;
  }
  return log;
}

void logConnectionStatus() {
  String log;

  #ifndef ESP32
  const uint8_t arduino_corelib_wifistatus = WiFi.status();
  const uint8_t sdk_wifistatus = wifi_station_get_connect_status();

  if ((arduino_corelib_wifistatus == WL_CONNECTED) != (sdk_wifistatus == STATION_GOT_IP)) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("WIFI  : SDK station status differs from Arduino status. SDK-status: ");
      log += SDKwifiStatusToString(sdk_wifistatus);
      log += F(" Arduino status: ");
      log += ArduinoWifiStatusToString(arduino_corelib_wifistatus);
      addLog(LOG_LEVEL_ERROR, log);
    }
  }
  #endif
#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
    String log = F("WIFI  : Arduino wifi status: ");
    log += ArduinoWifiStatusToString(WiFi.status());
    log += F(" ESPeasy internal wifi status: ");
    log += ESPeasyWifiStatusToString();
    addLog(LOG_LEVEL_DEBUG_MORE, log);
  }
#endif
}




void WifiCheck()
{
  if (wifiSetup) {
    return;
  }
  delay(0);


  IPAddress ip = WiFi.localIP();

  if (WiFiConnected()) {
    if (!useStaticIP()) {
      if ((ip[0] == 0) && (ip[1] == 0) && (ip[2] == 0) && (ip[3] == 0)) {

        addLog(LOG_LEVEL_ERROR, F("WIFI : DHCP renew probably failed"));
        resetWiFi();
      }
    }
  }

  if ((WiFi.status() == WL_DISCONNECTED) && (timePassedSince(last_wifi_connect_attempt_moment) > 5000)) {
    WifiDisconnect();
  }

  switch (wifiStatus) {
    case ESPEASY_WIFI_SERVICES_INITIALIZED:
      break;
    case ESPEASY_WIFI_DISCONNECTED:

      if ((lastDisconnectMoment == 0) || timeOutReached(lastDisconnectMoment + 5000)) {
        WiFiConnectRelaxed();
      }
      break;
    default:

      if (timeOutReached(last_wifi_connect_attempt_moment + (10000 + wifi_connect_attempt * 200))) {
        WifiDisconnect();
      }
      break;
  }

  if (mqtt_reconnect_count > 10) {
    connectionCheckHandler();
  }
}




bool getSubnetRange(IPAddress& low, IPAddress& high)
{
  if (WifiIsAP(WiFi.getMode())) {

    return false;
  }

  if (wifiStatus < ESPEASY_WIFI_GOT_IP) {
    return false;
  }
  const IPAddress ip = WiFi.localIP();
  const IPAddress subnet = WiFi.subnetMask();
  low = ip;
  high = ip;


  for (byte i = 0; i < 4; ++i) {
    if (subnet[i] != 255) {
      low[i] = low[i] & subnet[i];
      high[i] = high[i] | ~subnet[i];
    }
  }
  return true;
}

String getLastDisconnectReason() {
  String reason = "(";

  reason += lastDisconnectReason;
  reason += F(") ");

  switch (lastDisconnectReason) {
    case WIFI_DISCONNECT_REASON_UNSPECIFIED: reason += F("Unspecified"); break;
    case WIFI_DISCONNECT_REASON_AUTH_EXPIRE: reason += F("Auth expire"); break;
    case WIFI_DISCONNECT_REASON_AUTH_LEAVE: reason += F("Auth leave"); break;
    case WIFI_DISCONNECT_REASON_ASSOC_EXPIRE: reason += F("Assoc expire"); break;
    case WIFI_DISCONNECT_REASON_ASSOC_TOOMANY: reason += F("Assoc toomany"); break;
    case WIFI_DISCONNECT_REASON_NOT_AUTHED: reason += F("Not authed"); break;
    case WIFI_DISCONNECT_REASON_NOT_ASSOCED: reason += F("Not assoced"); break;
    case WIFI_DISCONNECT_REASON_ASSOC_LEAVE: reason += F("Assoc leave"); break;
    case WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED: reason += F("Assoc not authed"); break;
    case WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD: reason += F("Disassoc pwrcap bad"); break;
    case WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD: reason += F("Disassoc supchan bad"); break;
    case WIFI_DISCONNECT_REASON_IE_INVALID: reason += F("IE invalid"); break;
    case WIFI_DISCONNECT_REASON_MIC_FAILURE: reason += F("Mic failure"); break;
    case WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT: reason += F("4way handshake timeout"); break;
    case WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT: reason += F("Group key update timeout"); break;
    case WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS: reason += F("IE in 4way differs"); break;
    case WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID: reason += F("Group cipher invalid"); break;
    case WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID: reason += F("Pairwise cipher invalid"); break;
    case WIFI_DISCONNECT_REASON_AKMP_INVALID: reason += F("AKMP invalid"); break;
    case WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION: reason += F("Unsupp RSN IE version"); break;
    case WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP: reason += F("Invalid RSN IE cap"); break;
    case WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED: reason += F("802 1X auth failed"); break;
    case WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED: reason += F("Cipher suite rejected"); break;
    case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT: reason += F("Beacon timeout"); break;
    case WIFI_DISCONNECT_REASON_NO_AP_FOUND: reason += F("No AP found"); break;
    case WIFI_DISCONNECT_REASON_AUTH_FAIL: reason += F("Auth fail"); break;
    case WIFI_DISCONNECT_REASON_ASSOC_FAIL: reason += F("Assoc fail"); break;
    case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT: reason += F("Handshake timeout"); break;
    default: reason += getUnknownString(); break;
  }
  return reason;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPEasy_checks.ino"
void run_compiletime_checks() {
  check_size<CRCStruct, 168u>();
  check_size<SecurityStruct, 593u>();
  const unsigned int SettingsStructSize = (244 + 82 * TASKS_MAX);
  check_size<SettingsStruct, SettingsStructSize>();
  check_size<ControllerSettingsStruct, 748u>();
  check_size<NotificationSettingsStruct, 996u>();
  check_size<ExtraTaskSettingsStruct, 472u>();
  check_size<EventStruct, 96u>();



  const unsigned int LogStructSize = ((12u + 17 * LOG_STRUCT_MESSAGE_LINES) + 3) & ~3;
  check_size<LogStruct, LogStructSize>();
  check_size<DeviceStruct, 7u>();
  check_size<ProtocolStruct, 10u>();
  check_size<NotificationStruct, 3u>();
  check_size<NodeStruct, 24u>();
  check_size<systemTimerStruct, 28u>();
  check_size<RTCStruct, 20u>();
  check_size<rulesTimerStatus, 12u>();
  check_size<portStatusStruct, 4u>();
  check_size<ResetFactoryDefaultPreference_struct, 4u>();
  check_size<GpioFactorySettingsStruct, 11u>();
}

String ReportOffsetErrorInStruct(const String& structname, size_t offset) {
  String error;

  error.reserve(48 + structname.length());
  error = F("Error: Incorrect offset in struct: ");
  error += structname;
  error += '(';
  error += String(offset);
  error += ')';
  return error;
}





bool SettingsCheck(String& error) {
  error = "";
#ifdef esp8266
  size_t offset = offsetof(SettingsStruct, ResetFactoryDefaultPreference);

  if (offset != 1224) {
    error = ReportOffsetErrorInStruct(F("SettingsStruct"), offset);
  }
#endif

  if (!Settings.networkSettingsEmpty()) {
    if ((Settings.IP[0] == 0) || (Settings.Gateway[0] == 0) || (Settings.Subnet[0] == 0) || (Settings.DNS[0] == 0)) {
      error += F("Error: Either fill all IP settings fields or leave all empty");
    }
  }

  return error.length() == 0;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPeasyControllerCache.ino"

struct ControllerCache_struct {
  ControllerCache_struct() {}

  ~ControllerCache_struct() {
    if (_RTC_cache_handler != nullptr) {
      delete _RTC_cache_handler;
      _RTC_cache_handler = nullptr;
    }
  }


  bool write(uint8_t *data, unsigned int size) {
    if (_RTC_cache_handler == nullptr) {
      return false;
    }
    return _RTC_cache_handler->write(data, size);
  }



  bool read(uint8_t *data, unsigned int size) {
    return true;
  }


  bool flush() {
    if (_RTC_cache_handler == nullptr) {
      return false;
    }
    return _RTC_cache_handler->flush();
  }

  void init() {
    if (_RTC_cache_handler == nullptr) {
      _RTC_cache_handler = new RTC_cache_handler_struct;
    }
  }

  bool isInitialized() {
    return _RTC_cache_handler != nullptr;
  }


  void clearCache() {}

  bool deleteOldestCacheBlock() {
    if (_RTC_cache_handler != nullptr) {
      return _RTC_cache_handler->deleteOldestCacheBlock();
    }
    return false;
  }

  void resetpeek() {
    if (_RTC_cache_handler != nullptr) {
      _RTC_cache_handler->resetpeek();
    }
  }


  bool peek(uint8_t *data, unsigned int size) {
    if (_RTC_cache_handler == nullptr) {
      return false;
    }
    return _RTC_cache_handler->peek(data, size);
  }

  String getPeekCacheFileName(bool& islast) {
    if (_RTC_cache_handler == nullptr) {
      return "";
    }
    return _RTC_cache_handler->getPeekCacheFileName(islast);
  }

  int readFileNr = 0;
  int readPos = 0;

private:

  RTC_cache_handler_struct *_RTC_cache_handler = nullptr;
};
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/ESPeasyGPIO.ino"
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Hardware.ino"



void hardwareInit()
{

  for (byte gpio = 0; gpio < PIN_D_MAX; ++gpio) {
    bool serialPinConflict = (Settings.UseSerial && (gpio == 1 || gpio == 3));

    if (!serialPinConflict && (Settings.PinBootStates[gpio] != 0)) {
      const uint32_t key = createKey(1, gpio);

      switch (Settings.PinBootStates[gpio])
      {
        case 1:
          pinMode(gpio, OUTPUT);
          digitalWrite(gpio, LOW);
          globalMapPortStatus[key].state = LOW;
          globalMapPortStatus[key].mode = PIN_MODE_OUTPUT;
          globalMapPortStatus[key].init = 1;


          break;
        case 2:
          pinMode(gpio, OUTPUT);
          digitalWrite(gpio, HIGH);
          globalMapPortStatus[key].state = HIGH;
          globalMapPortStatus[key].mode = PIN_MODE_OUTPUT;
          globalMapPortStatus[key].init = 1;


          break;
        case 3:
          pinMode(gpio, INPUT_PULLUP);
          globalMapPortStatus[key].state = 0;
          globalMapPortStatus[key].mode = PIN_MODE_INPUT_PULLUP;
          globalMapPortStatus[key].init = 1;


          break;
      }
    }
  }

  if (Settings.Pin_Reset != -1) {
    pinMode(Settings.Pin_Reset, INPUT_PULLUP);
  }


  if (Settings.Pin_i2c_sda != -1)
  {
    String log = F("INIT : I2C");
    addLog(LOG_LEVEL_INFO, log);
    Wire.begin(Settings.Pin_i2c_sda, Settings.Pin_i2c_scl);

    if (Settings.WireClockStretchLimit)
    {
      String log = F("INIT : I2C custom clockstretchlimit:");
      log += Settings.WireClockStretchLimit;
      addLog(LOG_LEVEL_INFO, log);
        #if defined(ESP8266)
      Wire.setClockStretchLimit(Settings.WireClockStretchLimit);
        #endif
    }
  }


  if (Settings.WDI2CAddress != 0)
  {
    delay(500);
    Wire.beginTransmission(Settings.WDI2CAddress);
    Wire.write(0x83);
    Wire.write(17);
    Wire.endTransmission();

    Wire.requestFrom(Settings.WDI2CAddress, (uint8_t)1);

    if (Wire.available())
    {
      byte status = Wire.read();

      if (status & 0x1)
      {
        String log = F("INIT : Reset by WD!");
        addLog(LOG_LEVEL_ERROR, log);
        lastBootCause = BOOT_CAUSE_EXT_WD;
      }
    }
  }


  if (Settings.InitSPI)
  {
    SPI.setHwCs(false);
    SPI.begin();
    String log = F("INIT : SPI Init (without CS)");
    addLog(LOG_LEVEL_INFO, log);
  }
  else
  {
    String log = F("INIT : SPI not enabled");
    addLog(LOG_LEVEL_INFO, log);
  }

#ifdef FEATURE_SD

  if (Settings.Pin_sd_cs >= 0)
  {
    if (SD.begin(Settings.Pin_sd_cs))
    {
      String log = F("SD   : Init OK");
      addLog(LOG_LEVEL_INFO, log);
    }
    else
    {
      String log = F("SD   : Init failed");
      addLog(LOG_LEVEL_ERROR, log);
    }
  }
#endif
}

void checkResetFactoryPin() {
  static byte factoryResetCounter = 0;

  if (Settings.Pin_Reset == -1) {
    return;
  }

  if (digitalRead(Settings.Pin_Reset) == 0) {
    factoryResetCounter++;
  }
  else
  {
    if (factoryResetCounter > 9) {

      ResetFactory();
    }

    if (factoryResetCounter > 3) {

      reboot();
    }
    factoryResetCounter = 0;
  }
}




String getDeviceModelBrandString(DeviceModel model) {
  switch (model) {
    case DeviceModel_Sonoff_Basic:
    case DeviceModel_Sonoff_TH1x:
    case DeviceModel_Sonoff_S2x:
    case DeviceModel_Sonoff_TouchT1:
    case DeviceModel_Sonoff_TouchT2:
    case DeviceModel_Sonoff_TouchT3:
    case DeviceModel_Sonoff_4ch:
    case DeviceModel_Sonoff_POW:
    case DeviceModel_Sonoff_POWr2: return F("Sonoff");
    case DeviceModel_Shelly1: return F("Shelly");


    default: return "";
  }
}

String getDeviceModelString(DeviceModel model) {
  String result;

  result.reserve(16);
  result = getDeviceModelBrandString(model);

  switch (model) {
    case DeviceModel_Sonoff_Basic: result += F(" Basic"); break;
    case DeviceModel_Sonoff_TH1x: result += F(" TH1x"); break;
    case DeviceModel_Sonoff_S2x: result += F(" S2x"); break;
    case DeviceModel_Sonoff_TouchT1: result += F(" TouchT1"); break;
    case DeviceModel_Sonoff_TouchT2: result += F(" TouchT2"); break;
    case DeviceModel_Sonoff_TouchT3: result += F(" TouchT3"); break;
    case DeviceModel_Sonoff_4ch: result += F(" 4ch"); break;
    case DeviceModel_Sonoff_POW: result += F(" POW"); break;
    case DeviceModel_Sonoff_POWr2: result += F(" POW-r2"); break;
    case DeviceModel_Shelly1: result += '1'; break;


    default: result += F("default");
  }
  return result;
}

bool modelMatchingFlashSize(DeviceModel model) {
  uint32_t size_MB = getFlashRealSizeInBytes() >> 20;


  switch (model) {
    case DeviceModel_Sonoff_Basic:
    case DeviceModel_Sonoff_TH1x:
    case DeviceModel_Sonoff_S2x:
    case DeviceModel_Sonoff_TouchT1:
    case DeviceModel_Sonoff_TouchT2:
    case DeviceModel_Sonoff_TouchT3:
    case DeviceModel_Sonoff_4ch: return size_MB == 1;
    case DeviceModel_Sonoff_POW:
    case DeviceModel_Sonoff_POWr2: return size_MB == 4;
    case DeviceModel_Shelly1: return size_MB == 2;


    default: return true;
  }
  return true;
}

void setFactoryDefault(DeviceModel model) {
  ResetFactoryDefaultPreference.setDeviceModel(model);
}




void addSwitchPlugin(byte taskIndex, byte gpio, const String& name, bool activeLow) {
  setTaskDevice_to_TaskIndex(1, taskIndex);
  setBasicTaskValues(
    taskIndex,
    0,
    true,
    name,
    gpio,
    -1,
    -1);
  Settings.TaskDevicePin1PullUp[taskIndex] = true;

  if (activeLow) {
    Settings.TaskDevicePluginConfig[taskIndex][2] = 1;
  }
  Settings.TaskDevicePluginConfig[taskIndex][3] = 1;
}

void addPredefinedPlugins(const GpioFactorySettingsStruct& gpio_settings) {
  byte taskIndex = 0;

  for (int i = 0; i < 4; ++i) {
    if (gpio_settings.button[i] >= 0) {
      String label = F("Button");
      label += (i + 1);
      addSwitchPlugin(taskIndex, gpio_settings.button[i], label, true);
      ++taskIndex;
    }

    if (gpio_settings.relais[i] >= 0) {
      String label = F("Relay");
      label += (i + 1);
      addSwitchPlugin(taskIndex, gpio_settings.relais[i], label, false);
      ++taskIndex;
    }
  }
}

void addButtonRelayRule(byte buttonNumber, byte relay_gpio) {
  Settings.UseRules = true;
  String fileName;
  #if defined(ESP32)
  fileName += '/';
  #endif
  fileName += F("rules1.txt");
  String rule = F("on ButtonBNR#state do\n  if [RelayBNR#state]=0\n    gpio,GNR,1\n  else\n    gpio,GNR,0\n  endif\nendon\n");
  rule.replace(F("BNR"), String(buttonNumber));
  rule.replace(F("GNR"), String(relay_gpio));
  String result = appendLineToFile(fileName, rule);

  if (result.length() > 0) {
    addLog(LOG_LEVEL_ERROR, result);
  }
}

void addPredefinedRules(const GpioFactorySettingsStruct& gpio_settings) {
  for (int i = 0; i < 4; ++i) {
    if ((gpio_settings.button[i] >= 0) && (gpio_settings.relais[i] >= 0)) {
      addButtonRelayRule((i + 1), gpio_settings.relais[i]);
    }
  }
}

#ifdef ESP32




bool getGpioInfo(int gpio, int& pinnr, bool& input, bool& output, bool& warning) {
  pinnr = -1;



  input = gpio <= 39;
  output = gpio <= 33;

  if ((gpio < 0) || (gpio == 20) || (gpio == 24) || ((gpio > 27) && (gpio < 32))) {
    input = false;
    output = false;
  }

  if ((input == false) && (output == false)) {
    return false;
  }


  warning = (gpio == 0 || gpio == 2);

  if (gpio == 12) {





    warning = true;
  }

  if (gpio == 15) {


    warning = true;
  }
  return true;
}

#else


bool getGpioInfo(int gpio, int& pinnr, bool& input, bool& output, bool& warning) {
  pinnr = -1;
  input = true;
  output = true;


  warning = (gpio == 0 || gpio == 2 || gpio == 15);

  switch (gpio) {
    case 0: pinnr = 3; break;
    case 1: pinnr = 10; break;
    case 2: pinnr = 4; break;
    case 3: pinnr = 9; break;
    case 4: pinnr = 2; break;
    case 5: pinnr = 1; break;
    case 6:
    case 7:
    case 8: pinnr = -1; break;
    case 9: pinnr = 11; break;
    case 10: pinnr = 12; break;
    case 11: pinnr = -1; break;
    case 12: pinnr = 6; break;
    case 13: pinnr = 7; break;
    case 14: pinnr = 5; break;


    case 15: pinnr = 8; input = false; break;
    case 16: pinnr = 0; break;
  }
  # ifndef ESP8285

  if ((gpio == 9) || (gpio == 10)) {

    warning = true;
  }
  # endif

  if (pinnr < 0) {
    input = false;
    output = false;
    return false;
  }
  return true;
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/I2C.ino"




bool I2C_read_bytes(uint8_t i2caddr, I2Cdata_bytes& data) {
  const uint8_t size = data.getSize();

  return size == i2cdev.readBytes(i2caddr, data.getRegister(), size, data.get());
}

bool I2C_read_words(uint8_t i2caddr, I2Cdata_words& data) {
  const uint8_t size = data.getSize();

  return size == i2cdev.readWords(i2caddr, data.getRegister(), size, data.get());
}


#ifdef ESP32
  # define END_TRANSMISSION_FLAG true
#else
  # define END_TRANSMISSION_FLAG false
#endif




void I2C_wakeup(uint8_t i2caddr) {
  Wire.beginTransmission(i2caddr);
  Wire.endTransmission();
}




bool I2C_write8(uint8_t i2caddr, byte value) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}




bool I2C_write8_reg(uint8_t i2caddr, byte reg, byte value) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}




bool I2C_write16_reg(uint8_t i2caddr, byte reg, uint16_t value) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}




bool I2C_write16_LE_reg(uint8_t i2caddr, byte reg, uint16_t value) {
  return I2C_write16_reg(i2caddr, reg, (value << 8) | (value >> 8));
}




uint8_t I2C_read8(uint8_t i2caddr, bool *is_ok) {
  uint8_t value;

  byte count = Wire.requestFrom(i2caddr, (byte)1);

  if (is_ok != NULL) {
    *is_ok = (count == 1);
  }

  value = Wire.read();


  return value;
}




uint8_t I2C_read8_reg(uint8_t i2caddr, byte reg, bool *is_ok) {
  uint8_t value;

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);

  if (Wire.endTransmission(END_TRANSMISSION_FLAG) != 0) {
# 105 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/I2C.ino"
    if (is_ok != NULL) {
      *is_ok = false;
    }
  }
  byte count = Wire.requestFrom(i2caddr, (byte)1);

  if (is_ok != NULL) {
    *is_ok = (count == 1);
  }
  value = Wire.read();

  return value;
}




uint16_t I2C_read16_reg(uint8_t i2caddr, byte reg) {
  uint16_t value(0);

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(END_TRANSMISSION_FLAG);
  Wire.requestFrom(i2caddr, (byte)2);
  value = (Wire.read() << 8) | Wire.read();

  return value;
}




int32_t I2C_read24_reg(uint8_t i2caddr, byte reg) {
  int32_t value;

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(END_TRANSMISSION_FLAG);
  Wire.requestFrom(i2caddr, (byte)3);
  value = (((int32_t)Wire.read()) << 16) | (Wire.read() << 8) | Wire.read();

  return value;
}




int32_t I2C_read32_reg(uint8_t i2caddr, byte reg) {
  int32_t value;

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(END_TRANSMISSION_FLAG);
  Wire.requestFrom(i2caddr, (byte)4);
  value = (((int32_t)Wire.read()) << 24) | (((uint32_t)Wire.read()) << 16) | (Wire.read() << 8) | Wire.read();

  return value;
}




uint16_t I2C_read16_LE_reg(uint8_t i2caddr, byte reg) {
  uint16_t temp = I2C_read16_reg(i2caddr, reg);

  return (temp >> 8) | (temp << 8);
}




int16_t I2C_readS16_reg(uint8_t i2caddr, byte reg) {
  return (int16_t)I2C_read16_reg(i2caddr, reg);
}

int16_t I2C_readS16_LE_reg(uint8_t i2caddr, byte reg) {
  return (int16_t)I2C_read16_LE_reg(i2caddr, reg);
}

#undef END_TRANSMISSION_FLAG
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Misc.ino"




String getUnknownString() { return F("Unknown"); }

String getNodeTypeDisplayString(byte nodeType) {
  switch (nodeType)
  {
    case NODE_TYPE_ID_ESP_EASY_STD: return F("ESP Easy");
    case NODE_TYPE_ID_ESP_EASYM_STD: return F("ESP Easy Mega");
    case NODE_TYPE_ID_ESP_EASY32_STD: return F("ESP Easy 32");
    case NODE_TYPE_ID_RPI_EASY_STD: return F("RPI Easy");
    case NODE_TYPE_ID_ARDUINO_EASY_STD: return F("Arduino Easy");
    case NODE_TYPE_ID_NANO_EASY_STD: return F("Nano Easy");
  }
  return "";
}

String getSettingsTypeString(SettingsType settingsType) {
  switch (settingsType) {
    case BasicSettings_Type: return F("Settings");
    case TaskSettings_Type: return F("TaskSettings");
    case CustomTaskSettings_Type: return F("CustomTaskSettings");
    case ControllerSettings_Type: return F("ControllerSettings");
    case CustomControllerSettings_Type: return F("CustomControllerSettings");
    case NotificationSettings_Type: return F("NotificationSettings");
    default:
      break;
  }
  return "";
}

String getMQTT_state() {
  switch (MQTTclient.state()) {
    case MQTT_CONNECTION_TIMEOUT : return F("Connection timeout");
    case MQTT_CONNECTION_LOST : return F("Connection lost");
    case MQTT_CONNECT_FAILED : return F("Connect failed");
    case MQTT_DISCONNECTED : return F("Disconnected");
    case MQTT_CONNECTED : return F("Connected");
    case MQTT_CONNECT_BAD_PROTOCOL : return F("Connect bad protocol");
    case MQTT_CONNECT_BAD_CLIENT_ID : return F("Connect bad client_id");
    case MQTT_CONNECT_UNAVAILABLE : return F("Connect unavailable");
    case MQTT_CONNECT_BAD_CREDENTIALS: return F("Connect bad credentials");
    case MQTT_CONNECT_UNAUTHORIZED : return F("Connect unauthorized");
    default: break;
  }
  return "";
}




String getLastBootCauseString() {
  switch (lastBootCause)
  {
    case BOOT_CAUSE_MANUAL_REBOOT: return F("Manual reboot");
    case BOOT_CAUSE_DEEP_SLEEP:
       return F("Deep sleep");
    case BOOT_CAUSE_COLD_BOOT:
       return F("Cold boot");
    case BOOT_CAUSE_EXT_WD:
       return F("External Watchdog");
  }
  return getUnknownString();
}

#ifdef ESP32

String getResetReasonString(byte icore) {
  bool isDEEPSLEEP_RESET(false);
  switch (rtc_get_reset_reason( (RESET_REASON) icore)) {
    case NO_MEAN : return F("NO_MEAN");
    case POWERON_RESET : return F("Vbat power on reset");
    case SW_RESET : return F("Software reset digital core");
    case OWDT_RESET : return F("Legacy watch dog reset digital core");
    case DEEPSLEEP_RESET : isDEEPSLEEP_RESET = true; break;
    case SDIO_RESET : return F("Reset by SLC module, reset digital core");
    case TG0WDT_SYS_RESET : return F("Timer Group0 Watch dog reset digital core");
    case TG1WDT_SYS_RESET : return F("Timer Group1 Watch dog reset digital core");
    case RTCWDT_SYS_RESET : return F("RTC Watch dog Reset digital core");
    case INTRUSION_RESET : return F("Instrusion tested to reset CPU");
    case TGWDT_CPU_RESET : return F("Time Group reset CPU");
    case SW_CPU_RESET : return F("Software reset CPU");
    case RTCWDT_CPU_RESET : return F("RTC Watch dog Reset CPU");
    case EXT_CPU_RESET : return F("for APP CPU, reseted by PRO CPU");
    case RTCWDT_BROWN_OUT_RESET : return F("Reset when the vdd voltage is not stable");
    case RTCWDT_RTC_RESET : return F("RTC Watch dog reset digital core and rtc module");
    default: break;
  }
  if (isDEEPSLEEP_RESET) {
    String reason = F("Deep Sleep, Wakeup reason (");
    reason += rtc_get_wakeup_cause();
    reason += ')';
    return reason;
  }
  return getUnknownString();
}
#endif

String getResetReasonString() {
  #ifdef ESP32
  String reason = F("CPU0: ");
  reason += getResetReasonString(0);
  reason += F(" CPU1: ");
  reason += getResetReasonString(1);
  return reason;
  #else
  return ESP.getResetReason();
  #endif
}

uint32_t getFlashRealSizeInBytes() {
  #if defined(ESP32)
    return ESP.getFlashChipSize();
  #else
    return ESP.getFlashChipRealSize();
  #endif
}

String getSystemBuildString() {
  String result;
  result += BUILD;
  result += ' ';
  result += F(BUILD_NOTES);
  return result;
}

String getPluginDescriptionString() {
  String result;
  #ifdef PLUGIN_BUILD_NORMAL
    result += F(" [Normal]");
  #endif
  #ifdef PLUGIN_BUILD_TESTING
    result += F(" [Testing]");
  #endif
  #ifdef PLUGIN_BUILD_DEV
    result += F(" [Development]");
  #endif
  #ifdef PLUGIN_DESCR
  result += " [";
  result += F(PLUGIN_DESCR);
  result += ']';
  #endif
  return result;
}

String getSystemLibraryString() {
  String result;
  #if defined(ESP32)
    result += F("ESP32 SDK ");
    result += ESP.getSdkVersion();
  #else
    result += F("ESP82xx Core ");
    result += ESP.getCoreVersion();
    result += F(", NONOS SDK ");
    result += system_get_sdk_version();
    result += F(", LWIP: ");
    result += getLWIPversion();
  #endif
  if (puyaSupport()) {
    result += F(" PUYA support");
  }
  return result;
}

#ifndef ESP32
String getLWIPversion() {
  String result;
  result += LWIP_VERSION_MAJOR;
  result += '.';
  result += LWIP_VERSION_MINOR;
  result += '.';
  result += LWIP_VERSION_REVISION;
  if (LWIP_VERSION_IS_RC) {
    result += F("-RC");
    result += LWIP_VERSION_RC;
  } else if (LWIP_VERSION_IS_DEVELOPMENT) {
    result += F("-dev");
  }
  return result;
}
#endif

bool puyaSupport() {
  bool supported = false;
#ifdef PUYA_SUPPORT

  if (PUYA_SUPPORT) supported = true;
#endif
#ifdef PUYASUPPORT

  supported = true;
#endif
  return supported;
}

uint8_t getFlashChipVendorId() {
#ifdef PUYA_SUPPORT
  return ESP.getFlashChipVendorId();
#else
  #if defined(ESP8266)
    uint32_t flashChipId = ESP.getFlashChipId();
    return (flashChipId & 0x000000ff);
  #else
    return 0xFF;
  #endif
#endif
}

bool flashChipVendorPuya() {
  uint8_t vendorId = getFlashChipVendorId();
  return vendorId == 0x85;
}
# 226 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Misc.ino"
#if defined(ESP8266)
  #include <md5.h>
#endif
#if defined(ESP8266)

struct tcp_pcb;
extern struct tcp_pcb* tcp_tw_pcbs;
extern "C" void tcp_abort (struct tcp_pcb* pcb);

void tcpCleanup()
{

     while(tcp_tw_pcbs!=NULL)
    {
      tcp_abort(tcp_tw_pcbs);
    }

 }
#endif






#ifdef ESP32

uint32_t getCurrentFreeStack() {
  register uint8_t *sp asm("a1");
  return (sp - pxTaskGetStackStart(NULL));
}

uint32_t getFreeStackWatermark() {
  return uxTaskGetStackHighWaterMark(NULL);
}


bool canYield() { return true; }

#else
#ifdef CORE_PRE_2_4_2

extern "C" {
#include <cont.h>
  extern cont_t g_cont;
}

uint32_t getCurrentFreeStack() {
  register uint32_t *sp asm("a1");
  return 4 * (sp - g_cont.stack);
}

uint32_t getFreeStackWatermark() {
  return cont_get_free_stack(&g_cont);
}

bool canYield() {
  return cont_can_yield(&g_cont);
}

bool allocatedOnStack(const void* address) {
  register uint32_t *sp asm("a1");
  if (sp < address) return false;
  return g_cont.stack < address;
}


#else



extern "C" {
#include <cont.h>
  extern cont_t* g_pcont;
}

uint32_t getCurrentFreeStack() {

  register uint32_t *sp asm("a1");
  return 4 * (sp - g_pcont->stack);
}

uint32_t getFreeStackWatermark() {
  return cont_get_free_stack(g_pcont);
}

bool canYield() {
  return cont_can_yield(g_pcont);
}

bool allocatedOnStack(const void* address) {
  register uint32_t *sp asm("a1");
  if (sp < address) return false;
  return g_pcont->stack < address;
}

#endif
#endif

bool isDeepSleepEnabled()
{
  if (!Settings.deepSleep)
    return false;





  pinMode(16,INPUT_PULLUP);
  if (!digitalRead(16))
  {
    return false;
  }
  return true;
}

bool readyForSleep()
{
  if (!isDeepSleepEnabled())
    return false;
  if (!checkConnectionsEstablished()) {

    return timeOutReached(timerAwakeFromDeepSleep + 12000);
  }
  return timeOutReached(timerAwakeFromDeepSleep + 1000 * Settings.deepSleep);
}

void deepSleep(int dsdelay)
{

  checkRAM(F("deepSleep"));
  if (!isDeepSleepEnabled())
  {

    return;
  }


  if (lastBootCause!=BOOT_CAUSE_DEEP_SLEEP)
  {
    addLog(LOG_LEVEL_INFO, F("SLEEP: Entering deep sleep in 30 seconds."));
    if (Settings.UseRules && isDeepSleepEnabled())
      {
        String event = F("System#NoSleep=30");
        rulesProcessing(event);
      }
    delayBackground(30000);

    if (!isDeepSleepEnabled())
    {
      addLog(LOG_LEVEL_INFO, F("SLEEP: Deep sleep cancelled (GPIO16 connected to GND)"));
      return;
    }
  }
  saveUserVarToRTC();
  deepSleepStart(dsdelay);
}

void deepSleepStart(int dsdelay)
{

  String event = F("System#Sleep");
  rulesProcessing(event);

  RTC.deepSleepState = 1;
  saveToRTC();

  addLog(LOG_LEVEL_INFO, F("SLEEP: Powering down to deepsleep..."));
  delay(100);
  #if defined(ESP8266)
    #if defined(CORE_POST_2_5_0)
      uint64_t deepSleep_usec = dsdelay * 1000000ULL;
      if ((deepSleep_usec > ESP.deepSleepMax()) || dsdelay < 0) {
        deepSleep_usec = ESP.deepSleepMax();
      }
      ESP.deepSleepInstant(deepSleep_usec, WAKE_RF_DEFAULT);
    #else
      if (dsdelay > 4294 || dsdelay < 0)
        dsdelay = 4294;
      ESP.deepSleep((uint32_t)dsdelay * 1000000, WAKE_RF_DEFAULT);
    #endif
  #endif
  #if defined(ESP32)
    esp_sleep_enable_timer_wakeup((uint32_t)dsdelay * 1000000);
    esp_deep_sleep_start();
  #endif
}

boolean remoteConfig(struct EventStruct *event, const String& string)
{
  checkRAM(F("remoteConfig"));
  boolean success = false;
  String command = parseString(string, 1);

  if (command == F("config"))
  {
    success = true;
    if (parseString(string, 2) == F("task"))
    {
      String configTaskName = parseStringKeepCase(string, 3);
      String configCommand = parseStringToEndKeepCase(string, 4);
      if (configTaskName.length() == 0 || configCommand.length() == 0)
        return success;

      int8_t index = getTaskIndexByName(configTaskName);
      if (index != -1)
      {
        event->TaskIndex = index;
        success = PluginCall(PLUGIN_SET_CONFIG, event, configCommand);
      }
    }
  }
  return success;
}

int8_t getTaskIndexByName(const String& TaskNameSearch)
{

  for (byte x = 0; x < TASKS_MAX; x++)
  {
    LoadTaskSettings(x);
    String TaskName = getTaskDeviceName(x);
    if ((TaskName.length() != 0 ) && (TaskNameSearch.equalsIgnoreCase(TaskName)))
    {
      return x;
    }
  }
  return -1;
}




String formatGpioDirection(gpio_direction direction) {
  switch (direction) {
    case gpio_input: return F("&larr; ");
    case gpio_output: return F("&rarr; ");
    case gpio_bidirectional: return F("&#8644; ");
  }
  return "";
}

String formatGpioLabel(int gpio, bool includeWarning) {
  int pinnr = -1;
  bool input, output, warning;
  if (getGpioInfo(gpio, pinnr, input, output, warning)) {
    if (!includeWarning) {
      return createGPIO_label(gpio, pinnr, true, true, false);
    }
    return createGPIO_label(gpio, pinnr, input, output, warning);
  }
  return "-";
}

String formatGpioName(const String& label, gpio_direction direction, bool optional) {
  int reserveLength = 5 + 8 + label.length();
  if (optional) {
    reserveLength += 11;
  }
  String result;
  result.reserve(reserveLength);
  result += F("GPIO ");
  result += formatGpioDirection(direction);
  result += label;
  if (optional)
    result += F("(optional)");
  return result;
}

String formatGpioName(const String& label, gpio_direction direction) {
  return formatGpioName(label, direction, false);
}

String formatGpioName_input(const String& label) {
  return formatGpioName(label, gpio_input, false);
}

String formatGpioName_output(const String& label) {
  return formatGpioName(label, gpio_output, false);
}

String formatGpioName_bidirectional(const String& label) {
  return formatGpioName(label, gpio_bidirectional, false);
}

String formatGpioName_input_optional(const String& label) {
  return formatGpioName(label, gpio_input, true);
}

String formatGpioName_output_optional(const String& label) {
  return formatGpioName(label, gpio_output, true);
}



String formatGpioName_TX(bool optional) {
  return formatGpioName("RX", gpio_output, optional);
}

String formatGpioName_RX(bool optional) {
  return formatGpioName("TX", gpio_input, optional);
}

String formatGpioName_TX_HW(bool optional) {
  return formatGpioName("RX (HW)", gpio_output, optional);
}

String formatGpioName_RX_HW(bool optional) {
  return formatGpioName("TX (HW)", gpio_input, optional);
}
# 607 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Misc.ino"
bool getBitFromUL(uint32_t number, byte bitnr) {
  return (number >> bitnr) & 1UL;
}

void setBitToUL(uint32_t& number, byte bitnr, bool value) {
  uint32_t newbit = value ? 1UL : 0UL;
  number ^= (-newbit ^ number) & (1UL << bitnr);
}





String getPinStateJSON(boolean search, uint32_t key, const String& log, int16_t noSearchValue)
{
  checkRAM(F("getPinStateJSON"));
  printToWebJSON = true;
  byte mode = PIN_MODE_INPUT;
  int16_t value = noSearchValue;
  boolean found = false;

  if (search && existPortStatus(key))
  {
    mode = globalMapPortStatus[key].mode;
    value = globalMapPortStatus[key].state;
    found = true;
  }

  if (!search || (search && found))
  {
    String reply;
    reply.reserve(128);
    reply += F("{\n\"log\": \"");
    reply += log.substring(7, 32);
    reply += F("\",\n\"plugin\": ");
    reply += getPluginFromKey(key);
    reply += F(",\n\"pin\": ");
    reply += getPortFromKey(key);
    reply += F(",\n\"mode\": \"");
    reply += getPinModeString(mode);
    reply += F("\",\n\"state\": ");
    reply += value;
    reply += F("\n}\n");
    return reply;
  }
  return "?";
}

String getPinModeString(byte mode) {
  switch (mode)
  {
    case PIN_MODE_UNDEFINED: return F("undefined");
    case PIN_MODE_INPUT: return F("input");
    case PIN_MODE_INPUT_PULLUP: return F("input pullup");
    case PIN_MODE_OFFLINE: return F("offline");
    case PIN_MODE_OUTPUT: return F("output");
    case PIN_MODE_PWM: return F("PWM");
    case PIN_MODE_SERVO: return F("servo");
    default:
      break;
  }
  return F("ERROR: Not Defined");
}





#if defined(ESP32)
  #define PWMRANGE 1024
#endif
#define STATUS_PWM_NORMALVALUE (PWMRANGE>>2)
#define STATUS_PWM_NORMALFADE (PWMRANGE>>8)
#define STATUS_PWM_TRAFFICRISE (PWMRANGE>>1)

void statusLED(boolean traffic)
{
  static int gnStatusValueCurrent = -1;
  static long int gnLastUpdate = millis();

  if (Settings.Pin_status_led == -1)
    return;

  if (gnStatusValueCurrent<0)
    pinMode(Settings.Pin_status_led, OUTPUT);

  int nStatusValue = gnStatusValueCurrent;

  if (traffic)
  {
    nStatusValue += STATUS_PWM_TRAFFICRISE;
  }
  else
  {

    if (WiFiConnected())
    {
      long int delta = timePassedSince(gnLastUpdate);
      if (delta>0 || delta<0 )
      {
        nStatusValue -= STATUS_PWM_NORMALFADE;
        nStatusValue = std::max(nStatusValue, STATUS_PWM_NORMALVALUE);
        gnLastUpdate=millis();
      }
    }

    else if (WifiIsAP(WiFi.getMode()))
    {
      nStatusValue = ((millis()>>1) & PWMRANGE) - (PWMRANGE>>2);
    }

    else
    {
      nStatusValue = (millis()>>1) & (PWMRANGE>>2);
    }
  }

  nStatusValue = constrain(nStatusValue, 0, PWMRANGE);

  if (gnStatusValueCurrent != nStatusValue)
  {
    gnStatusValueCurrent = nStatusValue;

    long pwm = nStatusValue * nStatusValue;
    pwm >>= 10;
    if (Settings.Pin_status_led_Inversed)
      pwm = PWMRANGE-pwm;

    #if defined(ESP8266)
      analogWrite(Settings.Pin_status_led, pwm);
    #endif
  }
}





void delayBackground(unsigned long dsdelay)
{
  unsigned long timer = millis() + dsdelay;
  while (!timeOutReached(timer))
    backgroundtasks();
}





void parseCommandString(struct EventStruct *event, const String& string)
{
  checkRAM(F("parseCommandString"));
  String TmpStr1;
  event->Par1 = 0;
  event->Par2 = 0;
  event->Par3 = 0;
  event->Par4 = 0;
  event->Par5 = 0;

  if (GetArgv(string.c_str(), TmpStr1, 2)) { event->Par1 = CalculateParam(TmpStr1.c_str()); }
  if (GetArgv(string.c_str(), TmpStr1, 3)) { event->Par2 = CalculateParam(TmpStr1.c_str()); }
  if (GetArgv(string.c_str(), TmpStr1, 4)) { event->Par3 = CalculateParam(TmpStr1.c_str()); }
  if (GetArgv(string.c_str(), TmpStr1, 5)) { event->Par4 = CalculateParam(TmpStr1.c_str()); }
  if (GetArgv(string.c_str(), TmpStr1, 6)) { event->Par5 = CalculateParam(TmpStr1.c_str()); }
}




void taskClear(byte taskIndex, boolean save)
{
  checkRAM(F("taskClear"));
  Settings.clearTask(taskIndex);
  ExtraTaskSettings.clear();
  ExtraTaskSettings.TaskIndex = taskIndex;
  if (save) {
    SaveTaskSettings(taskIndex);
    SaveSettings();
  }
}

String checkTaskSettings(byte taskIndex) {
  String err = LoadTaskSettings(taskIndex);
  if (err.length() > 0) return err;
  if (!ExtraTaskSettings.checkUniqueValueNames()) {
    return F("Use unique value names");
  }
  if (!ExtraTaskSettings.checkInvalidCharInNames()) {
    return F("Invalid character in names. Do not use ',#[]' or space.");
  }
  String deviceName = ExtraTaskSettings.TaskDeviceName;
  if (deviceName.length() == 0) {
    if (Settings.TaskDeviceEnabled[taskIndex]) {

      return F("Warning: Task Device Name is empty. It is adviced to give tasks an unique name");
    }
  }
  for (int i = 0; i < TASKS_MAX; ++i) {
    if (i != taskIndex && Settings.TaskDeviceEnabled[i]) {
      LoadTaskSettings(i);
      if (ExtraTaskSettings.TaskDeviceName[0] != 0) {
        if (strcasecmp(ExtraTaskSettings.TaskDeviceName, deviceName.c_str()) == 0) {
          err = F("Task Device Name is not unique, conflicts with task ID #");
          err += (i+1);

        }
      }
    }
  }

  err += LoadTaskSettings(taskIndex);
  return err;
}






byte getDeviceIndex(byte Number)
{
  for (byte x = 0; x <= deviceCount ; x++) {
    if (Device[x].Number == Number) {
      return x;
    }
  }
  return 0;
}




String getPluginNameFromDeviceIndex(byte deviceIndex) {
  String deviceName = "";
  Plugin_ptr[deviceIndex](PLUGIN_GET_DEVICENAME, 0, deviceName);
  return deviceName;
}





byte getProtocolIndex(byte Number)
{
  for (byte x = 0; x <= protocolCount ; x++) {
    if (Protocol[x].Number == Number) {
      return x;
    }
  }
  return 0;
}




byte getNotificationProtocolIndex(byte Number)
{
  for (byte x = 0; x <= notificationCount ; x++) {
    if (Notification[x].Number == Number) {
      return(x);
    }
  }
  return(NPLUGIN_NOT_FOUND);
}





bool HasArgv(const char *string, unsigned int argc) {
  int pos_begin, pos_end;
  return GetArgvBeginEnd(string, argc, pos_begin, pos_end);
}

bool GetArgv(const char *string, String& argvString, unsigned int argc) {
  int pos_begin, pos_end;
  bool hasArgument = GetArgvBeginEnd(string, argc, pos_begin, pos_end);
  argvString = "";
  if (pos_begin >= 0 && pos_end >= 0) {
    argvString.reserve(pos_end - pos_begin);
    for (int i = pos_begin; i < pos_end && i >= 0; ++i) {
      argvString += string[i];
    }
  }
  return hasArgument;
}

bool GetArgvBeginEnd(const char *string, const unsigned int argc, int& pos_begin, int& pos_end) {
  pos_begin = -1;
  pos_end = -1;
  size_t string_len = strlen(string);
  unsigned int string_pos = 0, argc_pos = 0;
  char c, d;
  boolean parenthesis = false;
  char matching_parenthesis = '"';

  while (string_pos < string_len)
  {
    c = string[string_pos];
    d = 0;
    if ((string_pos + 1) < string_len) {
      d = string[string_pos + 1];
    }

    if (!parenthesis && c == ' ' && d == ' ') {}
    else if (!parenthesis && c == ' ' && d == ',') {}
    else if (!parenthesis && c == ',' && d == ' ') {}
    else if (!parenthesis && c == ' ' && d >= 33 && d <= 126) {}
    else if (!parenthesis && c == ',' && d >= 33 && d <= 126) {}
    else if (c == '"' || c == '\'' || c == '[') {
      parenthesis = true;
      matching_parenthesis = c;
      if (c == '[') {
        matching_parenthesis = ']';
      }
    }
    else
    {
      if (pos_begin == -1) {
        pos_begin = string_pos;
        pos_end = string_pos;
      }
      ++pos_end;

      if ((!parenthesis && (d == ' ' || d == ',' || d == 0)) || (parenthesis && (d == matching_parenthesis)))
      {
        if (d == matching_parenthesis) {
          parenthesis = false;
        }
        argc_pos++;

        if (argc_pos == argc)
        {
          return true;
        }
        pos_begin = -1;
        pos_end = -1;
        string_pos++;
      }
    }
    string_pos++;
  }
  return false;
}
# 969 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Misc.ino"
#if defined(ARDUINO_ESP8266_RELEASE_2_3_0)
void dump (uint32_t addr) {
  serialPrint (String(addr, HEX));
  serialPrint(": ");
  for (uint32_t a = addr; a < addr + 16; a++)
  {
    serialPrint ( String(pgm_read_byte(a), HEX));
    serialPrint (" ");
  }
  serialPrintln("");
}
#endif

uint32_t progMemMD5check(){
    checkRAM(F("progMemMD5check"));
    #define BufSize 10
    uint32_t calcBuffer[BufSize];
    CRCValues.numberOfCRCBytes = 0;
    memcpy (calcBuffer,CRCValues.compileTimeMD5,16);
    if( memcmp (calcBuffer, "MD5_MD5_MD5_",12)==0){
        addLog(LOG_LEVEL_INFO, F("CRC  : No program memory checksum found. Check output of crc2.py"));
        return 0;
    }
    MD5Builder md5;
    md5.begin();
    for (int l = 0; l<4; l++){
        uint32_t *ptrStart = (uint32_t *)&CRCValues.compileTimeMD5[16+l*4];
        uint32_t *ptrEnd = (uint32_t *)&CRCValues.compileTimeMD5[16+4*4+l*4];
        if ((*ptrStart) == 0) break;
        for (uint32_t i = *ptrStart; i< (*ptrEnd) ; i=i+sizeof(calcBuffer)){
             for (int buf = 0; buf < BufSize; buf ++){
                calcBuffer[buf] = pgm_read_dword((uint32_t*)i+buf);
                CRCValues.numberOfCRCBytes+=sizeof(calcBuffer[0]);
             }
             md5.add((uint8_t *)&calcBuffer[0],(*ptrEnd-i)<sizeof(calcBuffer) ? (*ptrEnd-i):sizeof(calcBuffer) );
        }
   }
   md5.calculate();
   md5.getBytes(CRCValues.runTimeMD5);
   if ( CRCValues.checkPassed()) {
      addLog(LOG_LEVEL_INFO, F("CRC  : program checksum       ...OK"));
      return CRCValues.numberOfCRCBytes;
   }
   addLog(LOG_LEVEL_INFO, F("CRC  : program checksum       ...FAIL"));
   return 0;
}




String getTaskDeviceName(byte TaskIndex) {
  LoadTaskSettings(TaskIndex);
  return ExtraTaskSettings.TaskDeviceName;
}





void ResetFactory()
{
  const GpioFactorySettingsStruct gpio_settings(ResetFactoryDefaultPreference.getDeviceModel());

  checkRAM(F("ResetFactory"));

  serialPrint(F("RESET: Resetting factory defaults... using "));
  serialPrint(getDeviceModelString(ResetFactoryDefaultPreference.getDeviceModel()));
  serialPrintln(F(" settings"));
  delay(1000);
  if (readFromRTC())
  {
    serialPrint(F("RESET: Warm boot, reset count: "));
    serialPrintln(String(RTC.factoryResetCounter));
    if (RTC.factoryResetCounter >= 3)
    {
      serialPrintln(F("RESET: Too many resets, protecting your flash memory (powercycle to solve this)"));
      return;
    }
  }
  else
  {
    serialPrintln(F("RESET: Cold boot"));
    initRTC();

  }

  RTC.flashCounter=0;
  RTC.factoryResetCounter++;
  saveToRTC();


  SPIFFS.end();
  serialPrintln(F("RESET: formatting..."));
  SPIFFS.format();
  serialPrintln(F("RESET: formatting done..."));
  if (!SPIFFS.begin())
  {
    serialPrintln(F("RESET: FORMAT SPIFFS FAILED!"));
    return;
  }



  String fname;

  fname=FILE_CONFIG;
  InitFile(fname.c_str(), CONFIG_FILE_SIZE);

  fname=FILE_SECURITY;
  InitFile(fname.c_str(), 4096);

  fname=FILE_NOTIFICATION;
  InitFile(fname.c_str(), 4096);

  fname=FILE_RULES;
  InitFile(fname.c_str(), 0);

  Settings.clearMisc();
  if (!ResetFactoryDefaultPreference.keepNTP()) {
    Settings.clearTimeSettings();
    Settings.UseNTP = DEFAULT_USE_NTP;
    strcpy_P(Settings.NTPHost, PSTR(DEFAULT_NTP_HOST));
    Settings.TimeZone = DEFAULT_TIME_ZONE;
    Settings.DST = DEFAULT_USE_DST;
  }

  if (!ResetFactoryDefaultPreference.keepNetwork()) {
    Settings.clearNetworkSettings();

    str2ip(F(DEFAULT_IPRANGE_LOW), SecuritySettings.AllowedIPrangeLow);
    str2ip(F(DEFAULT_IPRANGE_HIGH), SecuritySettings.AllowedIPrangeHigh);
    SecuritySettings.IPblockLevel = DEFAULT_IP_BLOCK_LEVEL;

    #if DEFAULT_USE_STATIC_IP
      str2ip((char*)DEFAULT_IP, Settings.IP);
      str2ip((char*)DEFAULT_DNS, Settings.DNS);
      str2ip((char*)DEFAULT_GW, Settings.Gateway);
      str2ip((char*)DEFAULT_SUBNET, Settings.Subnet);
    #endif
  }

  Settings.clearNotifications();
  Settings.clearControllers();
  Settings.clearTasks();
  if (!ResetFactoryDefaultPreference.keepLogSettings()) {
    Settings.clearLogSettings();
    str2ip((char*)DEFAULT_SYSLOG_IP, Settings.Syslog_IP);

    setLogLevelFor(LOG_TO_SYSLOG, DEFAULT_SYSLOG_LEVEL);
    setLogLevelFor(LOG_TO_SERIAL, DEFAULT_SERIAL_LOG_LEVEL);
    setLogLevelFor(LOG_TO_WEBLOG, DEFAULT_WEB_LOG_LEVEL);
    setLogLevelFor(LOG_TO_SDCARD, DEFAULT_SD_LOG_LEVEL);
    Settings.SyslogFacility = DEFAULT_SYSLOG_FACILITY;
    Settings.UseValueLogger = DEFAULT_USE_SD_LOG;
  }
  if (!ResetFactoryDefaultPreference.keepUnitName()) {
    Settings.clearUnitNameSettings();
    Settings.Unit = UNIT;
    strcpy_P(Settings.Name, PSTR(DEFAULT_NAME));
    Settings.UDPPort = 0;
  }
  if (!ResetFactoryDefaultPreference.keepWiFi()) {
    strcpy_P(SecuritySettings.WifiSSID, PSTR(DEFAULT_SSID));
    strcpy_P(SecuritySettings.WifiKey, PSTR(DEFAULT_KEY));
    strcpy_P(SecuritySettings.WifiAPKey, PSTR(DEFAULT_AP_KEY));
    SecuritySettings.WifiSSID2[0] = 0;
    SecuritySettings.WifiKey2[0] = 0;
  }
  SecuritySettings.Password[0] = 0;

  Settings.ResetFactoryDefaultPreference = ResetFactoryDefaultPreference.getPreference();




  Settings.PID = ESP_PROJECT_PID;
  Settings.Version = VERSION;
  Settings.Build = BUILD;

  Settings.Delay = DEFAULT_DELAY;
  Settings.Pin_i2c_sda = gpio_settings.i2c_sda;
  Settings.Pin_i2c_scl = gpio_settings.i2c_scl;
  Settings.Pin_status_led = gpio_settings.status_led;
  Settings.Pin_status_led_Inversed = DEFAULT_PIN_STATUS_LED_INVERSED;
  Settings.Pin_sd_cs = -1;
  Settings.Pin_Reset = -1;
  Settings.Protocol[0] = DEFAULT_PROTOCOL;
  Settings.deepSleep = false;
  Settings.CustomCSS = false;
  Settings.InitSPI = false;
  for (byte x = 0; x < TASKS_MAX; x++)
  {
    Settings.TaskDevicePin1[x] = -1;
    Settings.TaskDevicePin2[x] = -1;
    Settings.TaskDevicePin3[x] = -1;
    Settings.TaskDevicePin1PullUp[x] = true;
    Settings.TaskDevicePin1Inversed[x] = false;
    for (byte y = 0; y < CONTROLLER_MAX; y++)
      Settings.TaskDeviceSendData[y][x] = true;
    Settings.TaskDeviceTimer[x] = Settings.Delay;
  }


  Settings.UseRules = DEFAULT_USE_RULES;

  Settings.MQTTRetainFlag = DEFAULT_MQTT_RETAIN;
  Settings.MessageDelay = DEFAULT_MQTT_DELAY;
  Settings.MQTTUseUnitNameAsClientId = DEFAULT_MQTT_USE_UNITNAME_AS_CLIENTID;


  Settings.UseSerial = DEFAULT_USE_SERIAL;
  Settings.BaudRate = DEFAULT_SERIAL_BAUD;
# 1192 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Misc.ino"
#ifdef PLUGIN_DESCR
  strcpy_P(Settings.Name, PSTR(PLUGIN_DESCR));
#endif

  addPredefinedPlugins(gpio_settings);
  addPredefinedRules(gpio_settings);

  SaveSettings();

#if DEFAULT_CONTROLLER
  MakeControllerSettings(ControllerSettings);
  strcpy_P(ControllerSettings.Subscribe, PSTR(DEFAULT_SUB));
  strcpy_P(ControllerSettings.Publish, PSTR(DEFAULT_PUB));
  strcpy_P(ControllerSettings.MQTTLwtTopic, PSTR(DEFAULT_MQTT_LWT_TOPIC));
  strcpy_P(ControllerSettings.LWTMessageConnect, PSTR(DEFAULT_MQTT_LWT_CONNECT_MESSAGE));
  strcpy_P(ControllerSettings.LWTMessageDisconnect, PSTR(DEFAULT_MQTT_LWT_DISCONNECT_MESSAGE));
  str2ip((char*)DEFAULT_SERVER, ControllerSettings.IP);
  ControllerSettings.HostName[0]=0;
  ControllerSettings.Port = DEFAULT_PORT;
  SaveControllerSettings(0, ControllerSettings);
#endif
  checkRAM(F("ResetFactory2"));
  serialPrintln(F("RESET: Succesful, rebooting. (you might need to press the reset button if you've justed flashed the firmware)"));

  delay(1000);
  WiFi.persistent(true);
  intent_to_reboot = true;
  WifiDisconnect();
  WiFi.persistent(false);
  reboot();
}






void emergencyReset()
{

  Serial.begin(115200);
  Serial.write(0xAA);
  Serial.write(0x55);
  delay(1);
  if (Serial.available() == 2)
    if (Serial.read() == 0xAA && Serial.read() == 0x55)
    {
      serialPrintln(F("\n\n\rSystem will reset to factory defaults in 10 seconds..."));
      delay(10000);
      ResetFactory();
    }
}





unsigned long FreeMem(void)
{
  #if defined(ESP8266)
    return system_get_free_heap_size();
  #endif
  #if defined(ESP32)
    return ESP.getFreeHeap();
  #endif
}







boolean isFloat(const String& tBuf) {
  return isNumerical(tBuf, false);
}

boolean isValidFloat(float f) {
  if (f == NAN) return false;
  if (f == INFINITY) return false;
  if (-f == INFINITY)return false;
  if (isnan(f)) return false;
  if (isinf(f)) return false;
  return true;
}

boolean isInt(const String& tBuf) {
  return isNumerical(tBuf, true);
}

bool validIntFromString(const String& tBuf, int& result) {
  const String numerical = getNumerical(tBuf, true);
  const bool isvalid = isInt(numerical);
  result = numerical.toInt();
  return isvalid;
}

bool validFloatFromString(const String& tBuf, float& result) {
  const String numerical = getNumerical(tBuf, false);
  const bool isvalid = isFloat(numerical);
  result = numerical.toFloat();
  return isvalid;
}


String getNumerical(const String& tBuf, bool mustBeInteger) {
  String result = "";
  const unsigned int bufLength = tBuf.length();
  if (bufLength == 0) return result;
  boolean decPt = false;
  int firstDec = 0;
  char c = tBuf.charAt(0);
  if(c == '+' || c == '-') {
    result += c;
    firstDec = 1;
  }
  for(unsigned int x=firstDec; x < bufLength; ++x) {
    c = tBuf.charAt(x);
    if(c == '.') {
      if (mustBeInteger) return result;

      if(decPt) return result;
      else decPt = true;
    }
    else if(c < '0' || c > '9') return result;
    result += c;
  }
  return result;
}

boolean isNumerical(const String& tBuf, bool mustBeInteger) {
  const unsigned int bufLength = tBuf.length();
  if (bufLength == 0) return false;
  boolean decPt = false;
  int firstDec = 0;
  char c = tBuf.charAt(0);
  if(c == '+' || c == '-')
    firstDec = 1;
  for(unsigned int x=firstDec; x < bufLength; ++x) {
    c = tBuf.charAt(x);
    if(c == '.') {
      if (mustBeInteger) return false;

      if(decPt) return false;
      else decPt = true;
    }
    else if(c < '0' || c > '9') return false;
  }
  return true;
}


float timeStringToSeconds(String tBuf) {
 float sec = 0;
 int split = tBuf.indexOf(':');
 if (split < 0) {
  sec += tBuf.toFloat() * 60 * 60;
 } else {
  sec += tBuf.substring(0, split).toFloat() * 60 * 60;
  tBuf = tBuf.substring(split +1);
  split = tBuf.indexOf(':');
  if (split < 0) {
   sec += tBuf.toFloat() * 60;
  } else {
   sec += tBuf.substring(0, split).toFloat() * 60;
   sec += tBuf.substring(split +1).toFloat();
  }
 }
 return sec;
}




void initLog()
{

  Settings.UseSerial=true;
  Settings.SyslogFacility=0;
  setLogLevelFor(LOG_TO_SYSLOG, 0);
  setLogLevelFor(LOG_TO_SERIAL, 2);
  setLogLevelFor(LOG_TO_WEBLOG, 2);
  setLogLevelFor(LOG_TO_SDCARD, 0);
}




String getLogLevelDisplayString(int logLevel) {
  switch (logLevel) {
    case LOG_LEVEL_NONE: return F("None");
    case LOG_LEVEL_ERROR: return F("Error");
    case LOG_LEVEL_INFO: return F("Info");
    case LOG_LEVEL_DEBUG: return F("Debug");
    case LOG_LEVEL_DEBUG_MORE: return F("Debug More");
    case LOG_LEVEL_DEBUG_DEV: return F("Debug dev");

    default:
      return "";
  }
}

String getLogLevelDisplayStringFromIndex(byte index, int& logLevel) {
  switch (index) {
    case 0: logLevel = LOG_LEVEL_ERROR; break;
    case 1: logLevel = LOG_LEVEL_INFO; break;
    case 2: logLevel = LOG_LEVEL_DEBUG; break;
    case 3: logLevel = LOG_LEVEL_DEBUG_MORE; break;
    case 4: logLevel = LOG_LEVEL_DEBUG_DEV; break;

    default: logLevel = -1; return "";
  }
  return getLogLevelDisplayString(logLevel);
}

void addToLog(byte loglevel, const String& string)
{
  addToLog(loglevel, string.c_str());
}

void addToLog(byte logLevel, const __FlashStringHelper* flashString)
{
    checkRAM(F("addToLog"));
    String s(flashString);
    addToLog(logLevel, s.c_str());
}

void disableSerialLog() {
  log_to_serial_disabled = true;
  setLogLevelFor(LOG_TO_SERIAL, 0);
}

void setLogLevelFor(byte destination, byte logLevel) {
  switch (destination) {
    case LOG_TO_SERIAL:
      if (!log_to_serial_disabled || logLevel == 0)
        Settings.SerialLogLevel = logLevel; break;
    case LOG_TO_SYSLOG: Settings.SyslogLevel = logLevel; break;
    case LOG_TO_WEBLOG: Settings.WebLogLevel = logLevel; break;
    case LOG_TO_SDCARD: Settings.SDLogLevel = logLevel; break;
    default:
      break;
  }
  updateLogLevelCache();
}

void updateLogLevelCache() {
  byte max_lvl = 0;
  if (log_to_serial_disabled) {
    if (Settings.UseSerial) {
      Serial.setDebugOutput(false);
    }
  } else {
    max_lvl = _max(max_lvl, Settings.SerialLogLevel);
#ifndef BUILD_NO_DEBUG
    if (Settings.UseSerial && Settings.SerialLogLevel >= LOG_LEVEL_DEBUG_MORE) {
      Serial.setDebugOutput(true);
    }
#endif
  }
  max_lvl = _max(max_lvl, Settings.SyslogLevel);
  if (Logging.logActiveRead()) {
    max_lvl = _max(max_lvl, Settings.WebLogLevel);
  }
#ifdef FEATURE_SD
  max_lvl = _max(max_lvl, Settings.SDLogLevel);
#endif
  highest_active_log_level = max_lvl;
}

bool loglevelActiveFor(byte logLevel) {
  return loglevelActive(logLevel, highest_active_log_level);
}

byte getSerialLogLevel() {
  if (log_to_serial_disabled || !Settings.UseSerial) return 0;
  if (wifiStatus != ESPEASY_WIFI_SERVICES_INITIALIZED){
    if (Settings.SerialLogLevel < LOG_LEVEL_INFO) {
      return LOG_LEVEL_INFO;
    }
  }
  return Settings.SerialLogLevel;
}

byte getWebLogLevel() {
  byte logLevelSettings = 0;
  if (Logging.logActiveRead()) {
    logLevelSettings = Settings.WebLogLevel;
  } else {
    if (Settings.WebLogLevel != 0) {
      updateLogLevelCache();
    }
  }
  return logLevelSettings;
}

boolean loglevelActiveFor(byte destination, byte logLevel) {
  byte logLevelSettings = 0;
  switch (destination) {
    case LOG_TO_SERIAL: {
      logLevelSettings = getSerialLogLevel();
      break;
    }
    case LOG_TO_SYSLOG: {
      logLevelSettings = Settings.SyslogLevel;
      break;
    }
    case LOG_TO_WEBLOG: {
      logLevelSettings = getWebLogLevel();
      break;
    }
    case LOG_TO_SDCARD: {
      #ifdef FEATURE_SD
      logLevelSettings = Settings.SDLogLevel;
      #endif
      break;
    }
    default:
      return false;
  }
  return loglevelActive(logLevel, logLevelSettings);
}


boolean loglevelActive(byte logLevel, byte logLevelSettings) {
  return (logLevel <= logLevelSettings);
}

void addToLog(byte logLevel, const char *line)
{
  if (loglevelActiveFor(LOG_TO_SERIAL, logLevel)) {
    addToSerialBuffer(String(millis()).c_str());
    addToSerialBuffer(" : ");
    addToSerialBuffer(line);
    addNewlineToSerialBuffer();
  }
  if (loglevelActiveFor(LOG_TO_SYSLOG, logLevel)) {
    syslog(logLevel, line);
  }
  if (loglevelActiveFor(LOG_TO_WEBLOG, logLevel)) {
    Logging.add(logLevel, line);
  }

#ifdef FEATURE_SD
  if (loglevelActiveFor(LOG_TO_SDCARD, logLevel)) {
    File logFile = SD.open("log.dat", FILE_WRITE);
    if (logFile)
      logFile.println(line);
    logFile.close();
  }
#endif
}





void delayedReboot(int rebootDelay)
{

  while (rebootDelay != 0 )
  {
    serialPrint(F("Delayed Reset "));
    serialPrintln(String(rebootDelay));
    rebootDelay--;
    delay(1000);
  }
  reboot();
}

void reboot() {

  flushAndDisconnectAllClients();
  SPIFFS.end();
  #if defined(ESP32)
    ESP.restart();
  #else
    ESP.reset();
  #endif
}





String parseTemplate(String &tmpString, byte lineSize)
{
  checkRAM(F("parseTemplate"));
  String newString = "";

  newString.reserve(lineSize);

  parseSystemVariables(tmpString, false);


  int leftBracketIndex = tmpString.indexOf('[');
  if (leftBracketIndex == -1)
    newString = tmpString;
  else
  {
    byte count = 0;
    byte currentTaskIndex = ExtraTaskSettings.TaskIndex;

    while (leftBracketIndex >= 0 && count < 10 - 1)
    {
      newString += tmpString.substring(0, leftBracketIndex);
      tmpString = tmpString.substring(leftBracketIndex + 1);
      int rightBracketIndex = tmpString.indexOf(']');
      if (rightBracketIndex >= 0)
      {
        String tmpStringMid = tmpString.substring(0, rightBracketIndex);
        tmpString = tmpString.substring(rightBracketIndex + 1);
        int hashtagIndex = tmpStringMid.indexOf('#');
        if (hashtagIndex >= 0) {
          String deviceName = tmpStringMid.substring(0, hashtagIndex);
          String valueName = tmpStringMid.substring(hashtagIndex + 1);
          String valueFormat = "";
          hashtagIndex = valueName.indexOf('#');
          if (hashtagIndex >= 0)
          {
            valueFormat = valueName.substring(hashtagIndex + 1);
            valueName = valueName.substring(0, hashtagIndex);
          }

          if (deviceName.equalsIgnoreCase(F("Plugin")))
          {
            String tmpString = tmpStringMid.substring(7);
            tmpString.replace('#', ',');
            if (PluginCall(PLUGIN_REQUEST, 0, tmpString))
              newString += tmpString;
          }
          else if (deviceName.equalsIgnoreCase(F("Var"))) {
            String tmpString = tmpStringMid.substring(4);
            if (tmpString.length()>0 && isDigit(tmpString[0])) {
              const int varNum = tmpString.toInt();
              if (varNum > 0 && varNum <= CUSTOM_VARS_MAX)
                newString += String(customFloatVar[varNum-1]);
            }
          }
          else
            for (byte y = 0; y < TASKS_MAX; y++)
            {
              if (Settings.TaskDeviceEnabled[y])
              {
                LoadTaskSettings(y);
                String taskDeviceName = getTaskDeviceName(y);
                if (taskDeviceName.length() != 0)
                {
                  if (deviceName.equalsIgnoreCase(taskDeviceName))
                  {
                    boolean match = false;
                    for (byte z = 0; z < VARS_PER_TASK; z++)
                      if (valueName.equalsIgnoreCase(ExtraTaskSettings.TaskDeviceValueNames[z]))
                      {




                        match = true;
                        bool isvalid;
                        String value = formatUserVar(y, z, isvalid);
                        if (isvalid) {
                          transformValue(newString, lineSize, value, valueFormat, tmpString);
                          break;
                        }
                      }
                    if (!match)
                    {
                      struct EventStruct TempEvent;
                      TempEvent.TaskIndex = y;
                      String tmpName = valueName;
                      if (PluginCall(PLUGIN_GET_CONFIG, &TempEvent, tmpName))
                        newString += tmpName;
                    }
                    break;
                  }
                }
              }
            }
        }
      }
      leftBracketIndex = tmpString.indexOf('[');
      count++;
    }
    checkRAM(F("parseTemplate2"));
    newString += tmpString;

    if (currentTaskIndex != 255)
      LoadTaskSettings(currentTaskIndex);
  }


  parseStandardConversions(newString, false);


  while (newString.length() < lineSize)
    newString += ' ';
  checkRAM(F("parseTemplate3"));
  return newString;
}






void transformValue(
 String& newString,
  byte lineSize,
 String value,
 String& valueFormat,
  const String &tmpString)
{
  checkRAM(F("transformValue"));




  if (valueFormat.length() > 0)
  {
    String valueJust = "";

    int hashtagIndex = valueFormat.indexOf('#');
    if (hashtagIndex >= 0)
    {
      valueJust = valueFormat.substring(hashtagIndex + 1);
      valueFormat = valueFormat.substring(0, hashtagIndex);
    }



    if (valueFormat.length() > 0)
    {
      const int val = value == "0" ? 0 : 1;
      const float valFloat = value.toFloat();

      String tempValueFormat = valueFormat;
      int tempValueFormatLength = tempValueFormat.length();
      const int invertedIndex = tempValueFormat.indexOf('!');
      const bool inverted = invertedIndex >= 0 ? 1 : 0;
      if (inverted)
        tempValueFormat.remove(invertedIndex,1);

      const int rightJustifyIndex = tempValueFormat.indexOf('R');
      const bool rightJustify = rightJustifyIndex >= 0 ? 1 : 0;
      if (rightJustify)
        tempValueFormat.remove(rightJustifyIndex,1);

      tempValueFormatLength = tempValueFormat.length();


      if (tempValueFormatLength > 0)
      {
        switch (tempValueFormat[0])
          {
          case 'V':
            break;
          case 'O':
            value = val == inverted ? F("OFF") : F(" ON");
            break;
          case 'C':
            value = val == inverted ? F("CLOSE") : F(" OPEN");
            break;
          case 'M':
            value = val == inverted ? F("AUTO") : F(" MAN");
            break;
          case 'm':
            value = val == inverted ? F("A") : F("M");
            break;
          case 'H':
            value = val == inverted ? F("COLD") : F(" HOT");
            break;
          case 'U':
            value = val == inverted ? F("DOWN") : F("  UP");
            break;
          case 'u':
            value = val == inverted ? F("D") : F("U");
            break;
          case 'Y':
            value = val == inverted ? F(" NO") : F("YES");
            break;
          case 'y':
            value = val == inverted ? F("N") : F("Y");
            break;
          case 'X':
            value = val == inverted ? F("O") : F("X");
            break;
          case 'I':
            value = val == inverted ? F("OUT") : F(" IN");
            break;
          case 'Z' :
            value = val == inverted ? "0" : "1";
            break;
          case 'D' :
            {
              int x;
              int y;
              x = 0;
              y = 0;

              switch (tempValueFormatLength)
              {
                case 2:
                  if (isDigit(tempValueFormat[1]))
                  {
                    x = (int)tempValueFormat[1]-'0';
                  }
                  break;
                case 3:
                  if (tempValueFormat[1]=='.' && isDigit(tempValueFormat[2]))
                  {
                    y = (int)tempValueFormat[2]-'0';
                  }
                  break;
                case 4:
                  if (isDigit(tempValueFormat[1]) && tempValueFormat[2]=='.' && isDigit(tempValueFormat[3]))
                  {
                    x = (int)tempValueFormat[1]-'0';
                    y = (int)tempValueFormat[3]-'0';
                  }
                  break;
                case 1:
                default:
                  break;
              }
              value = toString(valFloat,y);
              int indexDot;
              indexDot = value.indexOf('.') > 0 ? value.indexOf('.') : value.length();
              for (byte f = 0; f < (x - indexDot); f++)
                value = "0" + value;
              break;
            }
          case 'F' :
            value = (int)floorf(valFloat);
            break;
          case 'E' :
            value = (int)ceilf(valFloat);
            break;
          default:
            value = F("ERR");
            break;
          }


          const int valueJustLength = valueJust.length();
          if (valueJustLength > 0)
          {
            value.trim();
            switch (valueJust[0])
            {
            case 'P' :
              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]))
                {
                  int filler = valueJust[1] - value.length() - '0' ;
                  for (byte f = 0; f < filler; f++)
                    newString += ' ';
                }
              }
              break;
            case 'S' :
              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]))
                {
                  int filler = valueJust[1] - value.length() - '0' ;
                  for (byte f = 0; f < filler; f++)
                    value += ' ';
                }
              }
              break;
            case 'L':
              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]))
                {
                  value = value.substring(0,(int)valueJust[1]-'0');
                }
              }
              break;
            case 'R':
              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]))
                {
                  value = value.substring(std::max(0,(int)value.length()-((int)valueJust[1]-'0')));
                 }
              }
              break;
            case 'U':
              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]) && valueJust[2]=='.' && isDigit(valueJust[3]) && valueJust[1] > '0' && valueJust[3] > '0')
                {
                  value = value.substring(std::min((int)value.length(),(int)valueJust[1]-'0'-1),(int)valueJust[1]-'0'-1+(int)valueJust[3]-'0');
                }
                else
                {
                  newString += F("ERR");
                }
              }
              break;
            default:
              newString += F("ERR");
              break;
          }
        }
      }
      if (rightJustify)
      {
        int filler = lineSize - newString.length() - value.length() - tmpString.length() ;
        for (byte f = 0; f < filler; f++)
          newString += ' ';
      }
      {
#ifndef BUILD_NO_DEBUG
        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
          String logFormatted = F("DEBUG: Formatted String='");
          logFormatted += newString;
          logFormatted += value;
          logFormatted += '\'';
          addLog(LOG_LEVEL_DEBUG, logFormatted);
        }
#endif
      }
    }
  }


  newString += String(value);
  {
#ifndef BUILD_NO_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
      String logParsed = F("DEBUG DEV: Parsed String='");
      logParsed += newString;
      logParsed += '\'';
      addLog(LOG_LEVEL_DEBUG_DEV, logParsed);
    }
#endif
  }
  checkRAM(F("transformValue2"));
}




#define CALCULATE_OK 0
#define CALCULATE_ERROR_STACK_OVERFLOW 1
#define CALCULATE_ERROR_BAD_OPERATOR 2
#define CALCULATE_ERROR_PARENTHESES_MISMATCHED 3
#define CALCULATE_ERROR_UNKNOWN_TOKEN 4
#define STACK_SIZE 10
#define TOKEN_MAX 20

float globalstack[STACK_SIZE];
float *sp = globalstack - 1;
float *sp_max = &globalstack[STACK_SIZE - 1];

#define is_operator(c) (c == '+' || c == '-' || c == '*' || c == '/' || c == '^' || c == '%')
#define is_unary_operator(c) (c == '!')

int push(float value)
{
  if (sp != sp_max)
  {
    *(++sp) = value;
    return 0;
  }
  else
    return CALCULATE_ERROR_STACK_OVERFLOW;
}

float pop()
{
  if (sp != (globalstack - 1))
    return *(sp--);
  else
    return 0.0;
}

float apply_operator(char op, float first, float second)
{
  switch (op)
  {
    case '+':
      return first + second;
    case '-':
      return first - second;
    case '*':
      return first * second;
    case '/':
      return first / second;
    case '%':
      return static_cast<int>(round(first)) % static_cast<int>(round(second));
    case '^':
      return pow(first, second);
    default:
      return 0;
  }
}

float apply_unary_operator(char op, float first)
{
  switch (op)
  {
    case '!':
      return (round(first) == 0) ? 1 : 0;
    default:
      return 0;
  }
}

char *next_token(char *linep)
{
  while (isspace(*(linep++)));
  while (*linep && !isspace(*(linep++)));
  return linep;
}

int RPNCalculate(char* token)
{
  if (token[0] == 0)
    return 0;

  if (is_operator(token[0]) && token[1] == 0)
  {
    float second = pop();
    float first = pop();

    if (push(apply_operator(token[0], first, second)))
      return CALCULATE_ERROR_STACK_OVERFLOW;
  } else if (is_unary_operator(token[0]) && token[1] == 0)
  {
    float first = pop();

    if (push(apply_unary_operator(token[0], first)))
      return CALCULATE_ERROR_STACK_OVERFLOW;

  } else
    if (push(atof(token)))
      return CALCULATE_ERROR_STACK_OVERFLOW;

  return 0;
}






int op_preced(const char c)
{
  switch (c)
  {
    case '!':
      return 4;
    case '^':
      return 3;
    case '*':
    case '/':
    case '%':
      return 2;
    case '+':
    case '-':
      return 1;
  }
  return 0;
}

bool op_left_assoc(const char c)
{
  switch (c)
  {
    case '^':
    case '*':
    case '/':
    case '+':
    case '-':
    case '%':
      return true;
    case '!':
      return false;
  }
  return false;
}

unsigned int op_arg_count(const char c)
{
  switch (c)
  {
    case '^':
    case '*':
    case '/':
    case '+':
    case '-':
    case '%':
      return 2;
    case '!':
      return 1;
  }
  return 0;
}


int Calculate(const char *input, float* result)
{
  checkRAM(F("Calculate"));
  const char *strpos = input, *strend = input + strlen(input);
  char token[25];
  char c, oc, *TokenPos = token;
  char stack[32];
  unsigned int sl = 0;
  char sc;
  int error = 0;


  sp = globalstack - 1;
  oc=c=0;

  if (input[0] == '=') {
    ++strpos;
    c = *strpos;
  }

  while (strpos < strend)
  {

    oc = c;
    c = *strpos;
    if (c != ' ')
    {

      if ((c >= '0' && c <= '9') || c == '.' || (c == '-' && is_operator(oc)))
      {
        *TokenPos = c;
        ++TokenPos;
      }


      else if (is_operator(c) || is_unary_operator(c))
      {
        *(TokenPos) = 0;
        error = RPNCalculate(token);
        TokenPos = token;
        if (error)return error;
        while (sl > 0)
        {
          sc = stack[sl - 1];





          if (is_operator(sc) && ((op_left_assoc(c) && (op_preced(c) <= op_preced(sc))) || (op_preced(c) < op_preced(sc))))
          {

            *TokenPos = sc;
            ++TokenPos;
            *(TokenPos) = 0;
            error = RPNCalculate(token);
            TokenPos = token;
            if (error)return error;
            sl--;
          }
          else
            break;
        }

        stack[sl] = c;
        ++sl;
      }

      else if (c == '(')
      {
        stack[sl] = c;
        ++sl;
      }

      else if (c == ')')
      {
        bool pe = false;


        while (sl > 0)
        {
          *(TokenPos) = 0;
          error = RPNCalculate(token);
          TokenPos = token;
          if (error)return error;
          sc = stack[sl - 1];
          if (sc == '(')
          {
            pe = true;
            break;
          }
          else
          {
            *TokenPos = sc;
            ++TokenPos;
            sl--;
          }
        }

        if (!pe)
          return CALCULATE_ERROR_PARENTHESES_MISMATCHED;


        sl--;


        if (sl > 0)
          sc = stack[sl - 1];

      }
      else
        return CALCULATE_ERROR_UNKNOWN_TOKEN;
    }
    ++strpos;
  }


  while (sl > 0)
  {
    sc = stack[sl - 1];
    if (sc == '(' || sc == ')')
      return CALCULATE_ERROR_PARENTHESES_MISMATCHED;

    *(TokenPos) = 0;
    error = RPNCalculate(token);
    TokenPos = token;
    if (error)return error;
    *TokenPos = sc;
    ++TokenPos;
    --sl;
  }

  *(TokenPos) = 0;
  error = RPNCalculate(token);
  TokenPos = token;
  if (error)
  {
    *result = 0;
    return error;
  }
  *result = *sp;
  checkRAM(F("Calculate2"));
  return CALCULATE_OK;
}

int CalculateParam(const char *TmpStr) {
  int returnValue;



  if (TmpStr[0] != '=') {
    returnValue=str2int(TmpStr);
  } else {
    float param=0;

    int returnCode=Calculate(&TmpStr[1], &param);
    if (returnCode!=CALCULATE_OK) {
      String errorDesc;
      switch (returnCode) {
        case CALCULATE_ERROR_STACK_OVERFLOW:
          errorDesc = F("Stack Overflow");
          break;
        case CALCULATE_ERROR_BAD_OPERATOR:
          errorDesc = F("Bad Operator");
          break;
        case CALCULATE_ERROR_PARENTHESES_MISMATCHED:
          errorDesc = F("Parenthesis mismatch");
          break;
        case CALCULATE_ERROR_UNKNOWN_TOKEN:
          errorDesc = F("Unknown token");
          break;
        default:
          errorDesc = F("Unknown error");
          break;
        }
        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          String log = String(F("CALCULATE PARAM ERROR: ")) + errorDesc;
          addLog(LOG_LEVEL_ERROR, log);
          log = F("CALCULATE PARAM ERROR details: ");
          log += TmpStr;
          log += F(" = ");
          log += round(param);
          addLog(LOG_LEVEL_ERROR, log);
        }
      }
#ifndef BUILD_NO_DEBUG
        else {
      if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
        String log = F("CALCULATE PARAM: ");
        log += TmpStr;
        log += F(" = ");
        log += round(param);
        addLog(LOG_LEVEL_DEBUG, log);
      }
    }
#endif
    returnValue=round(param);
  }
  return returnValue;
}

void SendValueLogger(byte TaskIndex)
{
  bool featureSD = false;
  #ifdef FEATURE_SD
    featureSD = true;
  #endif

  String logger;
  if (featureSD || loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    LoadTaskSettings(TaskIndex);
    byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[TaskIndex]);
    for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++)
    {
      logger += getDateString('-');
      logger += ' ';
      logger += getTimeString(':');
      logger += ',';
      logger += Settings.Unit;
      logger += ',';
      logger += getTaskDeviceName(TaskIndex);
      logger += ',';
      logger += ExtraTaskSettings.TaskDeviceValueNames[varNr];
      logger += ',';
      logger += formatUserVarNoCheck(TaskIndex, varNr);
      logger += "\r\n";
    }

#ifndef BUILD_NO_DEBUG
    addLog(LOG_LEVEL_DEBUG, logger);
#endif
  }

#ifdef FEATURE_SD
  String filename = F("VALUES.CSV");
  File logFile = SD.open(filename, FILE_WRITE);
  if (logFile)
    logFile.print(logger);
  logFile.close();
#endif
}


#define TRACES 3
#define TRACEENTRIES 15

class RamTracker{
  private:
    String traces[TRACES] ;
    unsigned int tracesMemory[TRACES] ;
    unsigned int readPtr, writePtr;
    String nextAction[TRACEENTRIES];
    unsigned int nextActionStartMemory[TRACEENTRIES];

    unsigned int bestCaseTrace (void){
       unsigned int lowestMemoryInTrace = 0;
       unsigned int lowestMemoryInTraceIndex=0;
       for (int i = 0; i<TRACES; i++) {
          if (tracesMemory[i] > lowestMemoryInTrace){
            lowestMemoryInTrace= tracesMemory[i];
            lowestMemoryInTraceIndex=i;
            }
          }

      return lowestMemoryInTraceIndex;
      }

  public:
    RamTracker(void){
        readPtr=0;
        writePtr=0;
        for (int i = 0; i< TRACES; i++) {
          traces[i]="";
          tracesMemory[i]=0xffffffff;
          }
        for (int i = 0; i< TRACEENTRIES; i++) {
          nextAction[i]="startup";
          nextActionStartMemory[i] = ESP.getFreeHeap();
          }
        };

    void registerRamState(String &s){
       nextAction[writePtr]=s;
       nextActionStartMemory[writePtr]=ESP.getFreeHeap();
       int bestCase = bestCaseTrace();
       if ( ESP.getFreeHeap() < tracesMemory[bestCase]){
            traces[bestCase]="";
            readPtr = writePtr+1;
            if (readPtr>=TRACEENTRIES) readPtr=0;
            tracesMemory[bestCase] = ESP.getFreeHeap();

            for (int i = 0; i<TRACEENTRIES; i++) {
              traces[bestCase]+= nextAction[readPtr];
              traces[bestCase]+= "-> ";
              traces[bestCase]+= String(nextActionStartMemory[readPtr]);
              traces[bestCase]+= ' ';
              readPtr++;
              if (readPtr >=TRACEENTRIES) readPtr=0;
            }
       }
       writePtr++;
       if (writePtr >= TRACEENTRIES) writePtr=0;
    };
   void getTraceBuffer(){
#ifndef BUILD_NO_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
        String retval="Memtrace\n";
        for (int i = 0; i< TRACES; i++){
          retval += String(i);
          retval += ": lowest: ";
          retval += String(tracesMemory[i]);
          retval += "  ";
          retval += traces[i];
          addLog(LOG_LEVEL_DEBUG_DEV, retval);
          retval="";
        }
      }
#endif
    }
}myRamTracker;

void checkRAMtoLog(void){
  myRamTracker.getTraceBuffer();
}

void checkRAM(const __FlashStringHelper* flashString, int a ) {
 String s=String(a);
 checkRAM(flashString,s);
}

void checkRAM(const __FlashStringHelper* flashString, String &a ) {
  String s = flashString;
  checkRAM(s,a);
}

void checkRAM(String &flashString, String &a ) {
  String s = flashString;
  s+=" (";
  s+=a;
  s+=")";
  checkRAM(s);
}

void checkRAM( const __FlashStringHelper* flashString)
{
  String s = flashString;
  myRamTracker.registerRamState(s);

  uint32_t freeRAM = FreeMem();
  if (freeRAM < lowestRAM)
  {
    lowestRAM = freeRAM;
    lowestRAMfunction = flashString;
  }
  uint32_t freeStack = getFreeStackWatermark();
  if (freeStack < lowestFreeStack) {
    lowestFreeStack = freeStack;
    lowestFreeStackfunction = flashString;
  }
}

void checkRAM( String &a ) {
  myRamTracker.registerRamState(a);
}
# 2470 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Misc.ino"
void tone_espEasy(uint8_t _pin, unsigned int frequency, unsigned long duration) {
  #ifdef ESP32
    delay(duration);
  #else
    analogWriteFreq(frequency);

    analogWrite(_pin,100);
    delay(duration);
    analogWrite(_pin,0);
  #endif
}




void play_rtttl(uint8_t _pin, const char *p )
{
  checkRAM(F("play_rtttl"));
  #define OCTAVE_OFFSET 0


  const int notes[] = { 0,
    262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
    523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
    1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
    2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951
  };



  byte default_dur = 4;
  byte default_oct = 6;
  int bpm = 63;
  int num;
  long wholenote;
  long duration;
  byte note;
  byte scale;




  while(*p != ':') p++;
  p++;


  if(*p == 'd')
  {
    p++; p++;
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    if(num > 0) default_dur = num;
    p++;
  }


  if(*p == 'o')
  {
    p++; p++;
    num = *p++ - '0';
    if(num >= 3 && num <=7) default_oct = num;
    p++;
  }


  if(*p == 'b')
  {
    p++; p++;
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    bpm = num;
    p++;
  }


  wholenote = (60 * 1000L / bpm) * 4;


  while(*p)
  {

    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }

    if (num) duration = wholenote / num;
    else duration = wholenote / default_dur;


    note = 0;

    switch(*p)
    {
      case 'c':
        note = 1;
        break;
      case 'd':
        note = 3;
        break;
      case 'e':
        note = 5;
        break;
      case 'f':
        note = 6;
        break;
      case 'g':
        note = 8;
        break;
      case 'a':
        note = 10;
        break;
      case 'b':
        note = 12;
        break;
      case 'p':
      default:
        note = 0;
    }
    p++;


    if(*p == '#')
    {
      note++;
      p++;
    }


    if(*p == '.')
    {
      duration += duration/2;
      p++;
    }


    if(isdigit(*p))
    {
      scale = *p - '0';
      p++;
    }
    else
    {
      scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if(*p == ',')
      p++;


    if(note)
    {
      tone_espEasy(_pin, notes[(scale - 4) * 12 + note], duration);
    }
    else
    {
      delay(duration/10);
    }
  }
 checkRAM(F("play_rtttl2"));
}



bool OTA_possible(uint32_t& maxSketchSize, bool& use2step) {
#if defined(ESP8266)
  maxSketchSize = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  const bool otaPossible = maxSketchSize > SMALLEST_OTA_IMAGE;
  use2step = maxSketchSize < ESP.getSketchSize();
  if (use2step) {
    const uint32_t totalSketchSpace = ESP.getFreeSketchSpace() + ESP.getSketchSize();
    maxSketchSize = totalSketchSpace - SMALLEST_OTA_IMAGE;
  }
  if (maxSketchSize > MAX_SKETCH_SIZE) maxSketchSize = MAX_SKETCH_SIZE;
  return otaPossible;
#else
  return false;
#endif
}

#ifdef FEATURE_ARDUINO_OTA




void ArduinoOTAInit()
{
  checkRAM(F("ArduinoOTAInit"));

  ArduinoOTA.setPort(ARDUINO_OTA_PORT);
  ArduinoOTA.setHostname(Settings.Name);
  if (SecuritySettings.Password[0]!=0)
    ArduinoOTA.setPassword(SecuritySettings.Password);

  ArduinoOTA.onStart([]() {
      serialPrintln(F("OTA  : Start upload"));
      SPIFFS.end();
  });

  ArduinoOTA.onEnd([]() {
      serialPrintln(F("\nOTA  : End"));


      serialPrintln(F("\nOTA  : DO NOT RESET OR POWER OFF UNTIL BOOT+FLASH IS COMPLETE."));
      delay(100);
      reboot();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (Settings.UseSerial)
      Serial.printf("OTA  : Progress %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
      serialPrint(F("\nOTA  : Error (will reboot): "));
      if (error == OTA_AUTH_ERROR) serialPrintln(F("Auth Failed"));
      else if (error == OTA_BEGIN_ERROR) serialPrintln(F("Begin Failed"));
      else if (error == OTA_CONNECT_ERROR) serialPrintln(F("Connect Failed"));
      else if (error == OTA_RECEIVE_ERROR) serialPrintln(F("Receive Failed"));
      else if (error == OTA_END_ERROR) serialPrintln(F("End Failed"));

      delay(100);
      reboot();
  });
  ArduinoOTA.begin();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("OTA  : Arduino OTA enabled on port ");
    log += ARDUINO_OTA_PORT;
    addLog(LOG_LEVEL_INFO, log);
  }
}

#endif

int calc_CRC16(const String& text) {
  return calc_CRC16(text.c_str(), text.length());
}

int calc_CRC16(const char *ptr, int count)
{
    int crc;
    char i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (int) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return crc;
}

uint32_t calc_CRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}





float compute_dew_point_temp(float temperature, float humidity_percentage) {
  return pow(humidity_percentage / 100.0, 0.125) *
         (112.0 + 0.9*temperature) + 0.1*temperature - 112.0;
}




float compute_humidity_from_dewpoint(float temperature, float dew_temperature) {
  return 100.0 * pow((112.0 - 0.1 * temperature + dew_temperature) /
                     (112.0 + 0.9 * temperature), 8);
}







void savePortStatus(uint32_t key, struct portStatusStruct &tempStatus) {
  if (tempStatus.task<=0 && tempStatus.monitor<=0 && tempStatus.command<=0)
    globalMapPortStatus.erase(key);
  else
    globalMapPortStatus[key] = tempStatus;
}

bool existPortStatus(uint32_t key) {
  bool retValue = false;

  std::map<uint32_t,portStatusStruct>::iterator it;
  it = globalMapPortStatus.find(key);
  if (it != globalMapPortStatus.end()) {
    retValue = true;
  }
  return retValue;
}

void removeTaskFromPort(uint32_t key) {
  if (existPortStatus(key)) {
    (globalMapPortStatus[key].task > 0) ? globalMapPortStatus[key].task-- : globalMapPortStatus[key].task = 0;
    if (globalMapPortStatus[key].task<=0 && globalMapPortStatus[key].monitor<=0 && globalMapPortStatus[key].command<=0&& globalMapPortStatus[key].init<=0)
      globalMapPortStatus.erase(key);
  }
}

void removeMonitorFromPort(uint32_t key) {
  if (existPortStatus(key)) {
    globalMapPortStatus[key].monitor=0;
    if (globalMapPortStatus[key].task<=0 && globalMapPortStatus[key].monitor<=0 && globalMapPortStatus[key].command<=0&& globalMapPortStatus[key].init<=0)
      globalMapPortStatus.erase(key);
  }
}

void addMonitorToPort(uint32_t key) {
  globalMapPortStatus[key].monitor=1;
}

uint32_t createKey(uint16_t pluginNumber, uint16_t portNumber) {
  return (uint32_t) pluginNumber << 16 | portNumber;
}

uint16_t getPluginFromKey(uint32_t key) {
  return (uint16_t)(key >> 16);
}

uint16_t getPortFromKey(uint32_t key) {
  return (uint16_t)(key);
}







void HSV2RGB(float H, float S, float I, int rgb[3]) {
  int r, g, b;
  H = fmod(H,360);
  H = 3.14159*H/(float)180;
  S = S / 100;
  S = S>0?(S<1?S:1):0;
  I = I / 100;
  I = I>0?(I<1?I:1):0;


  if(H < 2.09439) {
    r = 255*I/3*(1+S*cos(H)/cos(1.047196667-H));
    g = 255*I/3*(1+S*(1-cos(H)/cos(1.047196667-H)));
    b = 255*I/3*(1-S);
  } else if(H < 4.188787) {
    H = H - 2.09439;
    g = 255*I/3*(1+S*cos(H)/cos(1.047196667-H));
    b = 255*I/3*(1+S*(1-cos(H)/cos(1.047196667-H)));
    r = 255*I/3*(1-S);
  } else {
    H = H - 4.188787;
    b = 255*I/3*(1+S*cos(H)/cos(1.047196667-H));
    r = 255*I/3*(1+S*(1-cos(H)/cos(1.047196667-H)));
    g = 255*I/3*(1-S);
  }
  rgb[0]=r;
  rgb[1]=g;
  rgb[2]=b;
}




void HSV2RGBW(float H, float S, float I, int rgbw[4]) {
  int r, g, b, w;
  float cos_h, cos_1047_h;
  H = fmod(H,360);
  H = 3.14159*H/(float)180;
  S = S / 100;
  S = S>0?(S<1?S:1):0;
  I = I / 100;
  I = I>0?(I<1?I:1):0;

  if(H < 2.09439) {
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667-H);
    r = S*255*I/3*(1+cos_h/cos_1047_h);
    g = S*255*I/3*(1+(1-cos_h/cos_1047_h));
    b = 0;
    w = 255*(1-S)*I;
  } else if(H < 4.188787) {
    H = H - 2.09439;
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667-H);
    g = S*255*I/3*(1+cos_h/cos_1047_h);
    b = S*255*I/3*(1+(1-cos_h/cos_1047_h));
    r = 0;
    w = 255*(1-S)*I;
  } else {
    H = H - 4.188787;
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667-H);
    b = S*255*I/3*(1+cos_h/cos_1047_h);
    r = S*255*I/3*(1+(1-cos_h/cos_1047_h));
    g = 0;
    w = 255*(1-S)*I;
  }

  rgbw[0]=r;
  rgbw[1]=g;
  rgbw[2]=b;
  rgbw[3]=w;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Modbus.ino"

#ifndef MODBUS_H
#define MODBUS_H 

enum MODBUS_states_t { MODBUS_IDLE, MODBUS_RECEIVE, MODBUS_RECEIVE_PAYLOAD };
enum MODBUS_registerTypes_t { signed16, unsigned16, signed32, unsigned32, signed64, unsigned64 };

#define MODBUS_FUNCTION_READ 4

class Modbus {
public:

  Modbus(void);
  bool handle();
  bool begin(uint8_t function,
             uint8_t ModbusID,
             uint16_t ModbusRegister,
             MODBUS_registerTypes_t type,
             char *IPaddress);
  double read() {
    if (resultReceived) {
      resultReceived = false;
      return result;
    }
    else {
      return -1;
    }
  }

  bool available() {
    return resultReceived;
  }

  unsigned int getReadErrors() {
    return errcnt;
  }

  void resetReadErrors() {
    errcnt = 0;
  }

  void stop() {
    TXRXstate = MODBUS_IDLE;
    handle();
  }

  bool tryRead(uint8_t ModbusID,
               uint16_t M_register,
               MODBUS_registerTypes_t type,
               char *IPaddress,
               double & result);

private:

  WiFiClient *ModbusClient;
  unsigned int errcnt;
  char sendBuffer[12] = { 0, 1, 0, 0, 0, 6, 0x7e, 4, 0x9d, 7, 0, 1 };
  String LogString = "";
  unsigned long timeout;
  MODBUS_states_t TXRXstate;
  unsigned int RXavailable;
  unsigned int payLoad;
  bool hasTimeout();
  MODBUS_registerTypes_t incomingValue;
  double result;
  bool resultReceived;
  bool isBusy(void) {
    return TXRXstate != MODBUS_IDLE;
  }

  uint16_t currentRegister;
  uint8_t currentFunction;
};
#endif


Modbus::Modbus() : ModbusClient(nullptr), errcnt(0), timeout(0),
  TXRXstate(MODBUS_IDLE), RXavailable(0), payLoad(0) {}

bool Modbus::begin(uint8_t function, uint8_t ModbusID, uint16_t ModbusRegister, MODBUS_registerTypes_t type, char *IPaddress)
{
  currentRegister = ModbusRegister;
  currentFunction = function;
  incomingValue = type;
  resultReceived = false;
  ModbusClient = new WiFiClient();
  ModbusClient->setNoDelay(true);
  ModbusClient->setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
  timeout = millis();
  ModbusClient->flush();

  if (ModbusClient->connected()) {
    LogString += F(" already connected. ");
  } else {
    LogString += F("connect: "); LogString += IPaddress;

    if (ModbusClient->connect(IPaddress, 502) != 1) {
      LogString += F(" fail. ");
      TXRXstate = MODBUS_IDLE;
      errcnt++;

      if (LogString.length() > 1) { addLog(LOG_LEVEL_DEBUG, LogString); }
      return false;
    }
  }
  LogString += F(" OK, sending read request: ");

  sendBuffer[6] = ModbusID;
  sendBuffer[7] = function;
  sendBuffer[8] = (ModbusRegister >> 8);
  sendBuffer[9] = (ModbusRegister & 0x00ff);

  if ((incomingValue == signed64) || (incomingValue == unsigned64)) {
    sendBuffer[11] = 4;
  }

  if ((incomingValue == signed32) || (incomingValue == unsigned32)) {
    sendBuffer[11] = 2;
  }

  if ((incomingValue == signed16) || (incomingValue == unsigned16)) {
    sendBuffer[11] = 1;
  }
  ModbusClient->flush();
  ModbusClient->write(&sendBuffer[0], sizeof(sendBuffer));

  for (unsigned int i = 0; i < sizeof(sendBuffer); i++) {
    LogString += ((unsigned int)(sendBuffer[i]));
    LogString += (" ");
  }
  TXRXstate = MODBUS_RECEIVE;

  if (LogString.length() > 1) { addLog(LOG_LEVEL_DEBUG, LogString); }
  return true;
}

bool Modbus::handle() {
  unsigned int RXavailable = 0;

  LogString = "";
  int64_t rxValue = 0;

  switch (TXRXstate) {
    case MODBUS_IDLE:


      if (ModbusClient) {
        ModbusClient->flush();
        ModbusClient->stop();
        delete (ModbusClient);
        delay(1);
        ModbusClient = nullptr;
      }
      break;

    case MODBUS_RECEIVE:

      if (hasTimeout()) { break; }

      if (ModbusClient->available() < 9) { break; }

      LogString += F("reading bytes: ");

      for (int a = 0; a < 9; a++) {
        payLoad = ModbusClient->read();
        LogString += (payLoad); LogString += ' ';
      }
      LogString += F("> ");

      if (payLoad > 8) {
        LogString += F("Payload too large !? ");
        errcnt++;
        TXRXstate = MODBUS_IDLE;
      }

    case MODBUS_RECEIVE_PAYLOAD:

      if (hasTimeout()) { break; }
      RXavailable = ModbusClient->available();

      if (payLoad != RXavailable) {
        TXRXstate = MODBUS_RECEIVE_PAYLOAD;
        break;
      }

      for (unsigned int i = 0; i < RXavailable; i++) {
        rxValue = rxValue << 8;
        char a = ModbusClient->read();
        rxValue = rxValue | a;
        LogString += ((int)a); LogString += (" ");
      }

      switch (incomingValue) {
        case signed16:
          result = (int16_t)rxValue;
          break;
        case unsigned16:
          result = (uint16_t)rxValue;
          break;
        case signed32:
          result = (int32_t)rxValue;
          break;
        case unsigned32:
          result = (uint32_t)rxValue;
          break;
        case signed64:
          result = (int64_t)rxValue;
          break;
        case unsigned64:
          result = (uint64_t)rxValue;
          break;
      }

      LogString += "value: "; LogString += result;



      TXRXstate = MODBUS_IDLE;

      resultReceived = true;
      break;

    default:
      LogString += F("default. ");
      TXRXstate = MODBUS_IDLE;
      break;
  }

  if (LogString.length() > 1) { addLog(LOG_LEVEL_DEBUG, LogString); }
  return true;
}

bool Modbus::hasTimeout()
{
  if ((millis() - timeout) > 10000) {
    LogString += F("Modbus RX timeout. "); LogString += String(ModbusClient->available());
    errcnt++;
    TXRXstate = MODBUS_IDLE;
    return true;
  }
  return false;
}




bool Modbus::tryRead(uint8_t ModbusID, uint16_t M_register, MODBUS_registerTypes_t type, char *IPaddress, double& result) {
  if (isBusy()) { return false;
  }

  if (available()) {
    if ((currentFunction == MODBUS_FUNCTION_READ) && (currentRegister == M_register)) {
      result = read();
      return true;
    }
  } else {
    begin(MODBUS_FUNCTION_READ, ModbusID, M_register, type, IPaddress);
  }
  return false;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Modbus_RTU.ino"
#include <Arduino.h>
#include <ESPeasySerial.h>

#define MODBUS_RECEIVE_BUFFER 256
#define MODBUS_BROADCAST_ADDRESS 0xFE

#define MODBUS_READ_HOLDING_REGISTERS 0x03
#define MODBUS_READ_INPUT_REGISTERS 0x04
#define MODBUS_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_WRITE_MULTIPLE_REGISTERS 0x10

#define MODBUS_CMD_READ_RAM 0x44
#define MODBUS_CMD_READ_EEPROM 0x46
#define MODBUS_CMD_WRITE_RAM 0x41
#define MODBUS_CMD_WRITE_EEPROM 0x43


#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION 1
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 2
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE 3
#define MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE 4
#define MODBUS_EXCEPTION_ACKNOWLEDGE 5
#define MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY 6
#define MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE 7
#define MODBUS_EXCEPTION_MEMORY_PARITY 8
#define MODBUS_EXCEPTION_NOT_DEFINED 9
#define MODBUS_EXCEPTION_GATEWAY_PATH 10
#define MODBUS_EXCEPTION_GATEWAY_TARGET 11


#define MODBUS_BADCRC (MODBUS_EXCEPTION_GATEWAY_TARGET + 1)
#define MODBUS_BADDATA (MODBUS_EXCEPTION_GATEWAY_TARGET + 2)
#define MODBUS_BADEXC (MODBUS_EXCEPTION_GATEWAY_TARGET + 3)
#define MODBUS_UNKEXC (MODBUS_EXCEPTION_GATEWAY_TARGET + 4)
#define MODBUS_MDATA (MODBUS_EXCEPTION_GATEWAY_TARGET + 5)
#define MODBUS_BADSLAVE (MODBUS_EXCEPTION_GATEWAY_TARGET + 6)


struct ModbusRTU_struct {
  ModbusRTU_struct() : easySerial(nullptr) {}

  ~ModbusRTU_struct() {
    reset();
  }

  void reset() {
    if (easySerial != nullptr) {
      delete easySerial;
      easySerial = nullptr;
    }
    detected_device_description = "";

    for (int i = 0; i < 8; ++i) {
      _sendframe[i] = 0;
    }
    _sendframe_used = 0;

    for (int i = 0; i < MODBUS_RECEIVE_BUFFER; ++i) {
      _recv_buf[i] = 0xff;
    }
    _recv_buf_used = 0;
    _modbus_address = MODBUS_BROADCAST_ADDRESS;
    _reads_pass = 0;
    _reads_crc_failed = 0;
  }

  bool init(const int16_t serial_rx, const int16_t serial_tx, int16_t baudrate, byte address) {
    return init(serial_rx, serial_tx, baudrate, address, -1);
  }

  bool init(const int16_t serial_rx, const int16_t serial_tx, int16_t baudrate, byte address, int8_t dere_pin) {
    if ((serial_rx < 0) || (serial_tx < 0)) {
      return false;
    }
    reset();
    easySerial = new ESPeasySerial(serial_rx, serial_tx);
    easySerial->begin(baudrate);

    if (!isInitialized()) { return false; }
    _modbus_address = address;
    _dere_pin = dere_pin;

    if (_dere_pin != -1) {
      pinMode(_dere_pin, OUTPUT);
    }

    detected_device_description = getDevice_description(_modbus_address);

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log += detected_device_description;
      addLog(LOG_LEVEL_INFO, log);
      modbus_log_MEI(_modbus_address);
    }
    return true;
  }

  bool isInitialized() const {
    return easySerial != nullptr;
  }

  void getStatistics(uint32_t& pass, uint32_t& fail) {
    pass = _reads_pass;
    fail = _reads_crc_failed;
  }

  void setModbusTimeout(uint16_t timeout) {
    _modbus_timeout = timeout;
  }

  uint16_t getModbusTimeout() const {
    return _modbus_timeout;
  }

  String getDevice_description(byte slaveAddress) {
    bool more_follows = true;
    byte next_object_id = 0;
    byte conformity_level = 0;
    unsigned int object_value_int;
    String description;
    String obj_text;

    for (byte object_id = 0; object_id < 0x84; ++object_id) {
      if (object_id == 6) {
        object_id = 0x82;
      }
      int result = modbus_get_MEI(slaveAddress, object_id, obj_text,
                                  object_value_int, next_object_id,
                                  more_follows, conformity_level);
      String label;

      switch (object_id) {
        case 0x01:

          if (result == 0) { label = "Pcode"; }
          break;
        case 0x02:

          if (result == 0) { label = "Rev"; }
          break;
        case 0x82:
        {
          if (result != 0) {
            uint32_t sensorId = readSensorId();
            obj_text = String(sensorId, HEX);
            result = 0;
          }

          if (result == 0) { label = "S/N"; }
          break;
        }
        case 0x83:
        {
          if (result != 0) {
            uint32_t sensorId = readTypeId();
            obj_text = String(sensorId, HEX);
            result = 0;
          }

          if (result == 0) { label = "Type"; }
          break;
        }
        default:
          break;
      }

      if (result == 0) {
        if (label.length() > 0) {

          description += label;
          description += ": ";
        }

        if (obj_text.length() > 0) {
          description += obj_text;
          description += " - ";
        }
      }
    }
    return description;
  }


  void buildRead_RAM_EEPROM(byte slaveAddress, byte functionCode,
                            short startAddress, byte number_bytes) {
    _sendframe[0] = slaveAddress;
    _sendframe[1] = functionCode;
    _sendframe[2] = (byte)(startAddress >> 8);
    _sendframe[3] = (byte)(startAddress & 0xFF);
    _sendframe[4] = number_bytes;
    _sendframe_used = 5;
  }


  void buildWriteCommandRegister(byte slaveAddress, byte value) {
    _sendframe[0] = slaveAddress;
    _sendframe[1] = MODBUS_CMD_WRITE_RAM;
    _sendframe[2] = 0;
    _sendframe[3] = 0x60;
    _sendframe[4] = 1;
    _sendframe[5] = value;
    _sendframe_used = 6;
  }

  void buildWriteMult16bRegister(byte slaveAddress, uint16_t startAddress, uint16_t value) {
    _sendframe[0] = slaveAddress;
    _sendframe[1] = MODBUS_WRITE_MULTIPLE_REGISTERS;
    _sendframe[2] = (byte)(startAddress >> 8);
    _sendframe[3] = (byte)(startAddress & 0xFF);
    _sendframe[4] = 0;
    _sendframe[5] = 1;
    _sendframe[6] = 2;
    _sendframe[7] = (byte)(value >> 8);
    _sendframe[8] = (byte)(value & 0xFF);
    _sendframe_used = 9;
  }

  void buildFrame(byte slaveAddress, byte functionCode,
                  short startAddress, short parameter) {
    _sendframe[0] = slaveAddress;
    _sendframe[1] = functionCode;
    _sendframe[2] = (byte)(startAddress >> 8);
    _sendframe[3] = (byte)(startAddress & 0xFF);
    _sendframe[4] = (byte)(parameter >> 8);
    _sendframe[5] = (byte)(parameter & 0xFF);
    _sendframe_used = 6;
  }

  void build_modbus_MEI_frame(byte slaveAddress, byte device_id,
                              byte object_id) {
    _sendframe[0] = slaveAddress;
    _sendframe[1] = 0x2B;
    _sendframe[2] = 0x0E;






    _sendframe[3] = device_id;
    _sendframe[4] = object_id;
    _sendframe_used = 5;
  }

  String MEI_objectid_to_name(byte object_id) {
    String result;

    switch (object_id) {
      case 0: result = "VendorName"; break;
      case 1: result = "ProductCode"; break;
      case 2: result = "MajorMinorRevision"; break;
      case 3: result = "VendorUrl"; break;
      case 4: result = "ProductName"; break;
      case 5: result = "ModelName"; break;
      case 6: result = "UserApplicationName"; break;
      case 0x80: result = "MemoryMapVersion"; break;
      case 0x81: result = "Firmware Rev."; break;
      case 0x82: result = "Sensor S/N"; break;
      case 0x83: result = "Sensor type"; break;
      default:
        result = "0x";
        result += String(object_id, HEX);
    }
    return result;
  }

  String parse_modbus_MEI_response(unsigned int& object_value_int,
                                   byte & next_object_id,
                                   bool & more_follows,
                                   byte & conformity_level) {
    String result;

    if (_recv_buf_used < 8) {

      addLog(LOG_LEVEL_INFO,
             String("MEI response too small: ") + _recv_buf_used);
      next_object_id = 0xFF;
      more_follows = false;
      return result;
    }
    int pos = 4;


    conformity_level = _recv_buf[pos++];
    more_follows = _recv_buf[pos++] != 0;
    next_object_id = _recv_buf[pos++];
    const byte number_objects = _recv_buf[pos++];
    byte object_id = 0;

    for (int i = 0; i < number_objects; ++i) {
      if ((pos + 3) < _recv_buf_used) {
        object_id = _recv_buf[pos++];
        const byte object_length = _recv_buf[pos++];

        if ((pos + object_length) < _recv_buf_used) {
          String object_value;

          if (object_id < 0x80) {

            object_value.reserve(object_length);
            object_value_int = static_cast<unsigned int>(-1);

            for (int c = 0; c < object_length; ++c) {
              object_value += char(_recv_buf[pos++]);
            }
          } else {
            object_value.reserve(2 * object_length + 2);
            object_value_int = 0;

            for (int c = 0; c < object_length; ++c) {
              object_value_int =
                object_value_int << 8 | _recv_buf[pos++];
            }
            object_value = "0x";
            String hexvalue(object_value_int, HEX);
            hexvalue.toUpperCase();
            object_value += hexvalue;
          }

          if (i != 0) {

            result += String(", ");
          }
          result += object_value;
        }
      }
    }
    return result;
  }

  void logModbusException(byte value) {
    if (value == 0) {
      return;
    }
# 420 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Modbus_RTU.ino"
  }
# 438 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Modbus_RTU.ino"
  byte processCommand() {

    unsigned int crc =
      ModRTU_CRC(_sendframe, _sendframe_used);



    byte checksumHi = (byte)((crc >> 8) & 0xFF);
    byte checksumLo = (byte)(crc & 0xFF);

    _sendframe[_sendframe_used++] = checksumLo;
    _sendframe[_sendframe_used++] = checksumHi;

    int nrRetriesLeft = 2;
    byte return_value = 0;

    while (nrRetriesLeft > 0) {
      return_value = 0;


      startWrite();
      easySerial->write(_sendframe, _sendframe_used);
      startRead();


      _recv_buf_used = 0;
      unsigned long timeout = millis() + _modbus_timeout;
      bool validPacket = false;

      while (!validPacket && _recv_buf_used < MODBUS_RECEIVE_BUFFER && !timeOutReached(timeout)) {
        while (easySerial->available()) {
          _recv_buf[_recv_buf_used++] = easySerial->read();
        }


        crc = ModRTU_CRC(_recv_buf, _recv_buf_used);
        validPacket = crc == 0;
        delay(0);
      }


      if (crc != 0u) {
        ++_reads_crc_failed;
        return_value = MODBUS_BADCRC;
      } else {
        const byte received_functionCode = _recv_buf[1];

        if ((received_functionCode & 0x80) != 0) {
          return_value = _recv_buf[2];
        }
        ++_reads_pass;
      }

      switch (return_value) {
        case MODBUS_EXCEPTION_ACKNOWLEDGE:
        case MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY:
        case MODBUS_BADCRC:


          break;
        default:
          nrRetriesLeft = 0;
          break;
      }
      --nrRetriesLeft;
    }
    return return_value;
  }

  uint32_t read_32b_InputRegister(short address) {
    uint32_t result = 0;
    int idHigh = readInputRegister(address);
    int idLow = readInputRegister(address + 1);

    if ((idHigh >= 0) && (idLow >= 0)) {
      result = idHigh;
      result = result << 16;
      result += idLow;
    }
    return result;
  }

  uint32_t read_32b_HoldingRegister(short address) {
    uint32_t result = 0;

    process_32b_register(_modbus_address, MODBUS_READ_HOLDING_REGISTERS, address, result);
    return result;
  }

  float read_float_HoldingRegister(short address) {
    union {
      uint32_t ival;
      float fval;
    } conversion;

    conversion.ival = read_32b_HoldingRegister(address);
    return conversion.fval;




  }

  int readInputRegister(short address) {

    return process_16b_register(_modbus_address, MODBUS_READ_INPUT_REGISTERS, address, 1);
  }

  int readHoldingRegister(short address) {

    return process_16b_register(
      _modbus_address, MODBUS_READ_HOLDING_REGISTERS, address, 1);
  }


  int writeSingleRegister(short address, short value) {

    return process_16b_register(
      _modbus_address, MODBUS_WRITE_SINGLE_REGISTER, address, value);
  }


  int writeMultipleRegisters(short address, short value) {
    return preset_mult16b_register(
      _modbus_address, address, value);
  }

  byte modbus_get_MEI(byte slaveAddress, byte object_id,
                      String& result, unsigned int& object_value_int,
                      byte& next_object_id, bool& more_follows,
                      byte& conformity_level) {

    build_modbus_MEI_frame(slaveAddress, 4, object_id);
    const byte process_result = processCommand();

    if (process_result == 0) {
      result = parse_modbus_MEI_response(object_value_int,
                                         next_object_id, more_follows,
                                         conformity_level);
    } else {
      more_follows = false;
    }
    return process_result;
  }

  void modbus_log_MEI(byte slaveAddress) {



    bool more_follows = true;
    byte conformity_level = 0;
    byte object_id = 0;
    byte next_object_id = 0;

    while (more_follows) {
      String result;
      unsigned int object_value_int;
      const byte process_result = modbus_get_MEI(
        slaveAddress, object_id, result, object_value_int, next_object_id,
        more_follows, conformity_level);

      if (process_result == 0) {
        if (result.length() > 0) {
          String log = MEI_objectid_to_name(object_id);
          log += ": ";
          log += result;
          addLog(LOG_LEVEL_INFO, log);
        }
      } else {
        switch (process_result) {
          case MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS:


            break;
          default:
            logModbusException(process_result);
            break;
        }
      }



      if (more_follows) {
        object_id = next_object_id;
      } else if (object_id < 0x84) {


        more_follows = true;
        object_id++;

        if (object_id == 7) {

          object_id = 0x80;
        }
      }
    }
  }

  int process_16b_register(byte slaveAddress, byte functionCode,
                           short startAddress, short parameter) {
    buildFrame(slaveAddress, functionCode, startAddress, parameter);
    const byte process_result = processCommand();

    if (process_result == 0) {
      return (_recv_buf[3] << 8) | (_recv_buf[4]);
    }
    logModbusException(process_result);
    return -1;
  }


  int preset_mult16b_register(byte slaveAddress, uint16_t startAddress, uint16_t value) {
    buildWriteMult16bRegister(slaveAddress, startAddress, value);
    const byte process_result = processCommand();

    if (process_result == 0) {
      return (_recv_buf[4] << 8) | (_recv_buf[5]);
    }
    logModbusException(process_result);
    return -1;
  }

  bool process_32b_register(byte slaveAddress, byte functionCode,
                            short startAddress, uint32_t& result) {
    buildFrame(slaveAddress, functionCode, startAddress, 2);
    const byte process_result = processCommand();

    if (process_result == 0) {
      result = 0;

      for (byte i = 0; i < 4; ++i) {
        result = result << 8;
        result += _recv_buf[i + 3];
      }
      return true;
    }
    logModbusException(process_result);
    return false;
  }

  int writeSpecialCommandRegister(byte command) {
    buildWriteCommandRegister(_modbus_address, command);
    const byte process_result = processCommand();

    if (process_result == 0) {
      return 0;
    }
    logModbusException(process_result);
    return -1;
  }

  unsigned int read_RAM_EEPROM(byte command, byte startAddress,
                               byte nrBytes) {
    buildRead_RAM_EEPROM(_modbus_address, command,
                         startAddress, nrBytes);
    const byte process_result = processCommand();

    if (process_result == 0) {
      unsigned int result = 0;

      for (int i = 0; i < _recv_buf[2]; ++i) {

        result = (result << 8) | _recv_buf[i + 3];
      }
      return result;
    }
    logModbusException(process_result);
    return -1;
  }


  unsigned int ModRTU_CRC(byte *buf, int len) {
    unsigned int crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
      crc ^= (unsigned int)buf[pos];

      for (int i = 8; i != 0; i--) {
        if ((crc & 0x0001) != 0) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  uint32_t readTypeId() {
    return read_32b_InputRegister(25);
  }

  uint32_t readSensorId() {
    return read_32b_InputRegister(29);
  }

  String detected_device_description;

private:

  void startWrite() {

    if ((_dere_pin == -1) || !isInitialized()) { return; }
    digitalWrite(_dere_pin, HIGH);
    delay(2);
  }

  void startRead() {
    if (!isInitialized()) { return; }
    easySerial->flush();


    if (_dere_pin != -1) {
      digitalWrite(_dere_pin, LOW);
    }
  }

  byte _sendframe[12] = { 0 };
  byte _sendframe_used = 0;
  byte _recv_buf[MODBUS_RECEIVE_BUFFER] = { 0xff };
  byte _recv_buf_used = 0;
  byte _modbus_address = MODBUS_BROADCAST_ADDRESS;
  int8_t _dere_pin = -1;
  uint32_t _reads_pass = 0;
  uint32_t _reads_crc_failed = 0;
  uint16_t _modbus_timeout = 180;

  ESPeasySerial *easySerial;
};
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Networking.ino"





#define IPADDR2STR(addr) (uint8_t)((uint32_t)addr & 0xFF), (uint8_t)(((uint32_t)addr >> 8) & 0xFF), (uint8_t)(((uint32_t)addr >> 16) & 0xFF), (uint8_t)(((uint32_t)addr >> 24) & 0xFF)


#include <lwip/netif.h>
#ifdef ESP8266
  #if !defined(ARDUINO_ESP8266_RELEASE_2_4_0) && !defined(ARDUINO_ESP8266_RELEASE_2_3_0)
    #define SUPPORT_ARP
  #endif
#endif

#ifdef SUPPORT_ARP
#include <lwip/etharp.h>
#endif




void syslog(byte logLevel, const char *message)
{
  if (Settings.Syslog_IP[0] != 0 && WiFiConnected())
  {
    IPAddress broadcastIP(Settings.Syslog_IP[0], Settings.Syslog_IP[1], Settings.Syslog_IP[2], Settings.Syslog_IP[3]);
    portUDP.beginPacket(broadcastIP, 514);
    char str[256];
    str[0] = 0;
    byte prio = Settings.SyslogFacility * 8;
    if ( logLevel == LOG_LEVEL_ERROR )
      prio += 3;
    else if ( logLevel == LOG_LEVEL_INFO )
      prio += 5;
    else
      prio += 7;




    snprintf_P(str, sizeof(str), PSTR("<%u>%s EspEasy: %s"), prio, Settings.Name, message);



    #if defined(ESP8266)
      portUDP.write(str);
    #endif
    #if defined(ESP32)
      portUDP.write((uint8_t*)str,strlen(str));
    #endif
    portUDP.endPacket();
  }
}





boolean runningUPDCheck = false;
void checkUDP()
{
  if (Settings.UDPPort == 0)
    return;

  if (runningUPDCheck)
    return;

  runningUPDCheck = true;


  int packetSize = portUDP.parsePacket();
  if (packetSize > 0 )
  {
    statusLED(true);

    IPAddress remoteIP = portUDP.remoteIP();
    if (portUDP.remotePort() == 123)
    {

      runningUPDCheck = false;
      return;
    }
    if (packetSize >= 2 && packetSize < UDP_PACKETSIZE_MAX) {

      std::vector<char> packetBuffer;
      packetBuffer.resize(packetSize + 1);
      memset(&packetBuffer[0], 0, packetSize + 1);

      int len = portUDP.read(&packetBuffer[0], packetSize);
      if (len >= 2) {
        if (packetBuffer[0] != 255)
        {
          packetBuffer[len] = 0;
          String request = &packetBuffer[0];
#ifndef BUILD_NO_DEBUG
          addLog(LOG_LEVEL_DEBUG, request);
#endif
          struct EventStruct TempEvent;
          parseCommandString(&TempEvent, request);
          TempEvent.Source = VALUE_SOURCE_SYSTEM;
          if (!PluginCall(PLUGIN_WRITE, &TempEvent, request))
            ExecuteCommand(VALUE_SOURCE_SYSTEM, &packetBuffer[0]);
        }
        else
        {

          switch (packetBuffer[1])
          {

            case 1:
              {
                if (len < 13)
                  break;
                byte unit = packetBuffer[12];
#ifndef BUILD_NO_DEBUG
                byte mac[6];
                byte ip[4];
                for (byte x = 0; x < 6; x++)
                  mac[x] = packetBuffer[x + 2];
                for (byte x = 0; x < 4; x++)
                  ip[x] = packetBuffer[x + 8];
#endif
                Nodes[unit].age = 0;
                NodesMap::iterator it = Nodes.find(unit);
                if (it != Nodes.end()) {
                  for (byte x = 0; x < 4; x++)
                    it->second.ip[x] = packetBuffer[x + 8];
                  it->second.age = 0;
                  if (len >= 41)
                  {
                    it->second.build = packetBuffer[13] + 256*packetBuffer[14];
                    char tmpNodeName[26] = {0};
                    memcpy(&tmpNodeName[0], reinterpret_cast<byte*>(&packetBuffer[15]), 25);
                    tmpNodeName[25] = 0;
                    it->second.nodeName = tmpNodeName;
                    it->second.nodeName.trim();
                    it->second.nodeType = packetBuffer[40];
                  }
                }

#ifndef BUILD_NO_DEBUG
                if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
                  char macaddress[20];
                  formatMAC(mac, macaddress);
                  char log[80];
                  sprintf_P(log, PSTR("UDP  : %s,%s,%u"), macaddress, formatIP(ip).c_str(), unit);
                  addLog(LOG_LEVEL_DEBUG_MORE, log);
                }
#endif
                break;
              }

            default:
              {
                struct EventStruct TempEvent;
                TempEvent.Data = reinterpret_cast<byte*>(&packetBuffer[0]);
                TempEvent.Par1 = remoteIP[3];
                TempEvent.Par2 = len;
                PluginCall(PLUGIN_UDP_IN, &TempEvent, dummyString);
                CPluginCall(CPLUGIN_UDP_IN, &TempEvent);
                break;
              }
          }
        }
      }
    }
  }
  #if defined(ESP32)
    portUDP.flush();
  #endif
  runningUPDCheck = false;
}





void SendUDPCommand(byte destUnit, char* data, byte dataLength)
{
  if (!WiFiConnected(10)) {
    return;
  }
  if (destUnit != 0)
  {
    sendUDP(destUnit, (byte*)data, dataLength);
    delay(10);
  } else {
    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it) {
      if (it->first != Settings.Unit) {
        sendUDP(it->first, (byte*)data, dataLength);
        delay(10);
      }
    }
  }
  delay(50);
}





void sendUDP(byte unit, byte* data, byte size)
{
  if (!WiFiConnected(10)) {
    return;
  }

  IPAddress remoteNodeIP;
  if (unit == 255)
    remoteNodeIP = {255,255,255,255};
  else {
    NodesMap::iterator it = Nodes.find(unit);
    if (it == Nodes.end())
      return;
    if (it->second.ip[0] == 0)
      return;
    remoteNodeIP = it->second.ip;
  }

#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
    String log = F("UDP  : Send UDP message to ");
    log += unit;
    addLog(LOG_LEVEL_DEBUG_MORE, log);
  }
#endif

  statusLED(true);
  portUDP.beginPacket(remoteNodeIP, Settings.UDPPort);
  portUDP.write(data, size);
  portUDP.endPacket();
}





void refreshNodeList()
{
  bool mustSendGratuitousARP = false;
  for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ) {
    bool mustRemove = true;
    if (it->second.ip[0] != 0) {
      if (it->second.age > 8) {

        mustSendGratuitousARP = true;
      }
      if (it->second.age < 10) {
        it->second.age++;
        mustRemove = false;
        ++it;
      }
    }
    if (mustRemove) {
      it = Nodes.erase(it);
    }
  }
  if (mustSendGratuitousARP) {
    sendGratuitousARP_now();
  }
}




void sendSysInfoUDP(byte repeats)
{
  if (Settings.UDPPort == 0 || !WiFiConnected(10))
    return;
# 283 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Networking.ino"
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, F("UDP  : Send Sysinfo message"));
#endif
  for (byte counter = 0; counter < repeats; counter++)
  {
    uint8_t mac[] = {0, 0, 0, 0, 0, 0};
    uint8_t* macread = WiFi.macAddress(mac);
    byte data[80];
    data[0] = 255;
    data[1] = 1;
    for (byte x = 0; x < 6; x++)
      data[x + 2] = macread[x];
    IPAddress ip = WiFi.localIP();
    for (byte x = 0; x < 4; x++)
      data[x + 8] = ip[x];
    data[12] = Settings.Unit;
    data[13] = Settings.Build & 0xff;
    data[14] = Settings.Build >> 8;
    memcpy((byte*)data+15,Settings.Name,25);
    data[40] = NODE_TYPE_ID;
    statusLED(true);

    IPAddress broadcastIP(255, 255, 255, 255);
    portUDP.beginPacket(broadcastIP, Settings.UDPPort);
    portUDP.write(data, 80);
    portUDP.endPacket();
    if (counter < (repeats - 1))
      delay(500);
  }

  Nodes[Settings.Unit].age = 0;

  NodesMap::iterator it = Nodes.find(Settings.Unit);
  if (it != Nodes.end())
  {
    IPAddress ip = WiFi.localIP();
    for (byte x = 0; x < 4; x++)
      it->second.ip[x] = ip[x];
    it->second.age = 0;
    it->second.build = Settings.Build;
    it->second.nodeType = NODE_TYPE_ID;
  }
}

#if defined(ESP8266)



void SSDP_schema(WiFiClient &client) {
  if (!WiFiConnected(10)) {
    return;
  }

  const IPAddress ip = WiFi.localIP();
  const uint32_t chipId = ESP.getChipId();
  char uuid[64];
  sprintf_P(uuid, PSTR("38323636-4558-4dda-9188-cda0e6%02x%02x%02x"),
            (uint16_t) ((chipId >> 16) & 0xff),
            (uint16_t) ((chipId >> 8) & 0xff),
            (uint16_t) chipId & 0xff );

  String ssdp_schema = F(
                         "HTTP/1.1 200 OK\r\n"
                         "Content-Type: text/xml\r\n"
                         "Connection: close\r\n"
                         "Access-Control-Allow-Origin: *\r\n"
                         "\r\n"
                         "<?xml version=\"1.0\"?>"
                         "<root xmlns=\"urn:schemas-upnp-org:device-1-0\">"
                         "<specVersion>"
                         "<major>1</major>"
                         "<minor>0</minor>"
                         "</specVersion>"
                         "<URLBase>http://");
  ssdp_schema += formatIP(ip);
  ssdp_schema += F(":80/</URLBase>"
                   "<device>"
                   "<deviceType>urn:schemas-upnp-org:device:BinaryLight:1</deviceType>"
                   "<friendlyName>");
  ssdp_schema += Settings.Name;
  ssdp_schema += F("</friendlyName>"
                   "<presentationURL>/</presentationURL>"
                   "<serialNumber>");
  ssdp_schema += ESP.getChipId();
  ssdp_schema += F("</serialNumber>"
                   "<modelName>ESP Easy</modelName>"
                   "<modelNumber>");
  ssdp_schema += F(BUILD_GIT);
  ssdp_schema += F("</modelNumber>"
                   "<modelURL>http://www.letscontrolit.com</modelURL>"
                   "<manufacturer>http://www.letscontrolit.com</manufacturer>"
                   "<manufacturerURL>http://www.letscontrolit.com</manufacturerURL>"
                   "<UDN>uuid:");
  ssdp_schema += uuid;
  ssdp_schema += F("</UDN></device>"
                   "</root>\r\n"
                   "\r\n");

  client.printf(ssdp_schema.c_str());
}





typedef enum {
  NONE,
  SEARCH,
  NOTIFY
} ssdp_method_t;

UdpContext* _server;

IPAddress _respondToAddr;
uint16_t _respondToPort;

bool _pending;
unsigned short _delay;
unsigned long _process_time;
unsigned long _notify_time;

#define SSDP_INTERVAL 1200
#define SSDP_PORT 1900
#define SSDP_METHOD_SIZE 10
#define SSDP_URI_SIZE 2
#define SSDP_BUFFER_SIZE 64
#define SSDP_MULTICAST_TTL 2

static const IPAddress SSDP_MULTICAST_ADDR(239, 255, 255, 250);





bool SSDP_begin() {
  _pending = false;

  if (_server) {
    _server->unref();
    _server = 0;
  }

  _server = new UdpContext;
  _server->ref();

  ip_addr_t ifaddr;
  ifaddr.addr = WiFi.localIP();
  ip_addr_t multicast_addr;
  multicast_addr.addr = (uint32_t) SSDP_MULTICAST_ADDR;
  if (igmp_joingroup(&ifaddr, &multicast_addr) != ERR_OK ) {
    return false;
  }

#ifdef CORE_POST_2_5_0

  if (!_server->listen(IP_ADDR_ANY, SSDP_PORT)) {
    return false;
  }

  _server->setMulticastInterface(&ifaddr);
  _server->setMulticastTTL(SSDP_MULTICAST_TTL);
  _server->onRx(&SSDP_update);
  if (!_server->connect(&multicast_addr, SSDP_PORT)) {
    return false;
  }
#else
  if (!_server->listen(*IP_ADDR_ANY, SSDP_PORT)) {
    return false;
  }

  _server->setMulticastInterface(ifaddr);
  _server->setMulticastTTL(SSDP_MULTICAST_TTL);
  _server->onRx(&SSDP_update);
  if (!_server->connect(multicast_addr, SSDP_PORT)) {
    return false;
  }
#endif

  SSDP_update();

  return true;
}





void SSDP_send(byte method) {
  uint32_t ip = WiFi.localIP();


  String _ssdp_response_template = F(
                                     "HTTP/1.1 200 OK\r\n"
                                     "EXT:\r\n"
                                     "ST: upnp:rootdevice\r\n");

  String _ssdp_notify_template = F(
                                   "NOTIFY * HTTP/1.1\r\n"
                                   "HOST: 239.255.255.250:1900\r\n"
                                   "NT: upnp:rootdevice\r\n"
                                   "NTS: ssdp:alive\r\n");

  String _ssdp_packet_template = F(
                                   "%s"
                                   "CACHE-CONTROL: max-age=%u\r\n"
                                   "SERVER: Arduino/1.0 UPNP/1.1 ESPEasy/%u\r\n"
                                   "USN: uuid:%s\r\n"
                                   "LOCATION: http://%u.%u.%u.%u:80/ssdp.xml\r\n"
                                   "\r\n");
  {
    char uuid[64];
    uint32_t chipId = ESP.getChipId();
    sprintf_P(uuid, PSTR("38323636-4558-4dda-9188-cda0e6%02x%02x%02x"),
              (uint16_t) ((chipId >> 16) & 0xff),
              (uint16_t) ((chipId >> 8) & 0xff),
              (uint16_t) chipId & 0xff );

    char *buffer = new char[1460]();
    int len = snprintf(buffer, 1460,
                       _ssdp_packet_template.c_str(),
                       (method == 0) ? _ssdp_response_template.c_str() : _ssdp_notify_template.c_str(),
                       SSDP_INTERVAL,
                       Settings.Build,
                       uuid,
                       IPADDR2STR(&ip)
                      );

    _server->append(buffer, len);
    delete[] buffer;
  }

  ip_addr_t remoteAddr;
  uint16_t remotePort;
  if (method == 0) {
    remoteAddr.addr = _respondToAddr;
    remotePort = _respondToPort;
  } else {
    remoteAddr.addr = SSDP_MULTICAST_ADDR;
    remotePort = SSDP_PORT;
  }
  _server->send(&remoteAddr, remotePort);
  statusLED(true);
}





void SSDP_update() {

  if (!_pending && _server->next()) {
    ssdp_method_t method = NONE;

    _respondToAddr = _server->getRemoteAddress();
    _respondToPort = _server->getRemotePort();

    typedef enum {METHOD, URI, PROTO, KEY, VALUE, ABORT} states;
    states state = METHOD;

    typedef enum {START, MAN, ST, MX} headers;
    headers header = START;

    uint8_t cursor = 0;
    uint8_t cr = 0;

    char buffer[SSDP_BUFFER_SIZE] = {0};

    while (_server->getSize() > 0) {
      char c = _server->read();

      (c == '\r' || c == '\n') ? cr++ : cr = 0;

      switch (state) {
        case METHOD:
          if (c == ' ') {
            if (strcmp_P(buffer, PSTR("M-SEARCH")) == 0) method = SEARCH;
            else if (strcmp_P(buffer, PSTR("NOTIFY")) == 0) method = NOTIFY;

            if (method == NONE) state = ABORT;
            else state = URI;
            cursor = 0;

          } else if (cursor < SSDP_METHOD_SIZE - 1) {
            buffer[cursor++] = c;
            buffer[cursor] = '\0';
          }
          break;
        case URI:
          if (c == ' ') {
            if (strcmp(buffer, "*")) state = ABORT;
            else state = PROTO;
            cursor = 0;
          } else if (cursor < SSDP_URI_SIZE - 1) {
            buffer[cursor++] = c;
            buffer[cursor] = '\0';
          }
          break;
        case PROTO:
          if (cr == 2) {
            state = KEY;
            cursor = 0;
          }
          break;
        case KEY:
          if (cr == 4) {
            _pending = true;
            _process_time = millis();
          }
          else if (c == ' ') {
            cursor = 0;
            state = VALUE;
          }
          else if (c != '\r' && c != '\n' && c != ':' && cursor < SSDP_BUFFER_SIZE - 1) {
            buffer[cursor++] = c;
            buffer[cursor] = '\0';
          }
          break;
        case VALUE:
          if (cr == 2) {
            switch (header) {
              case START:
                break;
              case MAN:
                break;
              case ST:
                if (strcmp_P(buffer, PSTR("ssdp:all"))) {
                  state = ABORT;
                }

                if (strcmp_P(buffer, PSTR("urn:schemas-upnp-org:device:BinaryLight:1")) == 0) {
                  _pending = true;
                  _process_time = millis();
                  state = KEY;
                }
                break;
              case MX:
                _delay = random(0, atoi(buffer)) * 1000L;
                break;
            }

            if (state != ABORT) {
              state = KEY;
              header = START;
              cursor = 0;
            }
          } else if (c != '\r' && c != '\n') {
            if (header == START) {
              if (strncmp(buffer, "MA", 2) == 0) header = MAN;
              else if (strcmp(buffer, "ST") == 0) header = ST;
              else if (strcmp(buffer, "MX") == 0) header = MX;
            }

            if (cursor < SSDP_BUFFER_SIZE - 1) {
              buffer[cursor++] = c;
              buffer[cursor] = '\0';
            }
          }
          break;
        case ABORT:
          _pending = false; _delay = 0;
          break;
      }
    }
  }

  if (_pending && timeOutReached(_process_time + _delay)) {
    _pending = false; _delay = 0;
    SSDP_send(NONE);
  } else if (_notify_time == 0 || timeOutReached(_notify_time + (SSDP_INTERVAL * 1000L))) {
    _notify_time = millis();
    SSDP_send(NOTIFY);
  }

  if (_pending) {
    while (_server->next())
      _server->flush();
  }
}
#endif


bool WiFiConnected(uint32_t timeout_ms) {
  uint32_t timer = millis() + (timeout_ms > 500 ? 500 : timeout_ms);
  uint32_t min_delay = timeout_ms / 20;
  if (min_delay < 10) {
    delay(0);
    min_delay = 10;
  }



  while (!WiFiConnected()) {
    if (timeOutReached(timer)) {
      return false;
    }
    delay(min_delay);
  }
  return true;
}

bool hostReachable(const IPAddress& ip) {
  if (!WiFiConnected()) return false;

  return true;
# 715 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Networking.ino"
}

bool connectClient(WiFiClient& client, const char* hostname, uint16_t port) {
  IPAddress ip;
  if (resolveHostByName(hostname, ip)) {
    return connectClient(client, ip, port);
  }
  return false;
}

bool connectClient(WiFiClient& client, IPAddress ip, uint16_t port)
{
  START_TIMER;
  if (!WiFiConnected()) {
    return false;
  }
  bool connected = (client.connect(ip, port) == 1);
  yield();
  if (!connected) {
    sendGratuitousARP_now();
  }
  STOP_TIMER(CONNECT_CLIENT_STATS);
#if defined(ESP32) || defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ARDUINO_ESP8266_RELEASE_2_4_0)
#else
  if (connected)
    client.keepAlive();
#endif
  return connected;
}

bool resolveHostByName(const char* aHostname, IPAddress& aResult) {
  START_TIMER;
  if (!WiFiConnected()) {
    return false;
  }
#if defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ESP32)
  bool resolvedIP = WiFi.hostByName(aHostname, aResult) == 1;
#else
  bool resolvedIP = WiFi.hostByName(aHostname, aResult, CONTROLLER_CLIENTTIMEOUT_DFLT) == 1;
#endif
  yield();
  if (!resolvedIP) {
    sendGratuitousARP_now();
  }
  STOP_TIMER(HOST_BY_NAME_STATS);
  return resolvedIP;
}

bool hostReachable(const String& hostname) {
  IPAddress remote_addr;
  if (resolveHostByName(hostname.c_str(), remote_addr)) {
    return hostReachable(remote_addr);
  }
  String log = F("Hostname cannot be resolved: ");
  log += hostname;
  addLog(LOG_LEVEL_ERROR, log);
  return false;
}



bool beginWiFiUDP_randomPort(WiFiUDP& udp) {
  if (!WiFiConnected()) {
    return false;
  }
  unsigned int attempts = 3;
  while (attempts > 0) {
    --attempts;
    long port = random(1025, 65535);
    if (udp.begin(port) != 0)
      return true;
  }
  return false;
}

void sendGratuitousARP() {
  if (!WiFiConnected()) {
    return;
  }
#ifdef SUPPORT_ARP


  START_TIMER;
  netif *n = netif_list;
  while (n) {
    etharp_gratuitous(n);
    n = n->next;
  }
  STOP_TIMER(GRAT_ARP_STATS);
#endif
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
#define TIMER_ID_SHIFT 28

#define SYSTEM_EVENT_QUEUE 0
#define CONST_INTERVAL_TIMER 1
#define PLUGIN_TASK_TIMER 2
#define TASK_DEVICE_TIMER 3
#define GPIO_TIMER 4

#include <list>
struct EventStructCommandWrapper {
  EventStructCommandWrapper() : id(0) {}

  EventStructCommandWrapper(unsigned long i, const struct EventStruct& e) : id(i), event(e) {}

  unsigned long id;
  String cmd;
  String line;
  struct EventStruct event;
};
std::list<EventStructCommandWrapper> EventQueue;





void setTimer(unsigned long timerType, unsigned long id, unsigned long msecFromNow) {
  setNewTimerAt(getMixedId(timerType, id), millis() + msecFromNow);
}

void setNewTimerAt(unsigned long id, unsigned long timer) {
  START_TIMER;
  msecTimerHandler.registerAt(id, timer);
  STOP_TIMER(SET_NEW_TIMER);
}


unsigned long getMixedId(unsigned long timerType, unsigned long id) {
  return (timerType << TIMER_ID_SHIFT) + id;
}

unsigned long decodeSchedulerId(unsigned long mixed_id, unsigned long& timerType) {
  timerType = (mixed_id >> TIMER_ID_SHIFT);
  const unsigned long mask = (1 << TIMER_ID_SHIFT) - 1;
  return mixed_id & mask;
}

String decodeSchedulerId(unsigned long mixed_id) {
  if (mixed_id == 0) {
    return F("Background Task");
  }
  unsigned long timerType = 0;
  const unsigned long id = decodeSchedulerId(mixed_id, timerType);
  String result;
  result.reserve(32);

  switch (timerType) {
    case CONST_INTERVAL_TIMER:
      result = F("Const Interval");
      break;
    case PLUGIN_TASK_TIMER:
      result = F("Plugin Task");
      break;
    case TASK_DEVICE_TIMER:
      result = F("Task Device");
      break;
    case GPIO_TIMER:
      result = F("GPIO");
      break;
  }
  result += F(" timer, id: ");
  result += String(id);
  return result;
}




void handle_schedule() {
  START_TIMER
  unsigned long timer;
  unsigned long mixed_id = 0;

  if (timePassedSince(last_system_event_run) < 500) {

    mixed_id = msecTimerHandler.getNextId(timer);
  }

  if (RTC.lastMixedSchedulerId != mixed_id) {
    RTC.lastMixedSchedulerId = mixed_id;
    saveToRTC();
  }

  if (mixed_id == 0) {



    backgroundtasks();
    process_system_event_queue();
    last_system_event_run = millis();
    STOP_TIMER(HANDLE_SCHEDULER_IDLE);
    return;
  }

  unsigned long timerType = 0;
  const unsigned long id = decodeSchedulerId(mixed_id, timerType);

  delay(0);

  switch (timerType) {
    case CONST_INTERVAL_TIMER:
      process_interval_timer(id, timer);
      break;
    case PLUGIN_TASK_TIMER:
      process_plugin_task_timer(id);
      break;
    case TASK_DEVICE_TIMER:
      process_task_device_timer(id, timer);
      break;
    case GPIO_TIMER:
      process_gpio_timer(id);
      break;
  }
  STOP_TIMER(HANDLE_SCHEDULER_TASK);
}






void setIntervalTimer(unsigned long id) {
  setIntervalTimer(id, millis());
}

void setIntervalTimerAt(unsigned long id, unsigned long newtimer) {
  setNewTimerAt(getMixedId(CONST_INTERVAL_TIMER, id), newtimer);
}

void setIntervalTimerOverride(unsigned long id, unsigned long msecFromNow) {
  unsigned long timer = millis();

  setNextTimeInterval(timer, msecFromNow);
  setNewTimerAt(getMixedId(CONST_INTERVAL_TIMER, id), timer);
}

void scheduleNextDelayQueue(unsigned long id, unsigned long nextTime) {
  if (nextTime != 0) {

    setIntervalTimerAt(id, nextTime);
  }
}

void setIntervalTimer(unsigned long id, unsigned long lasttimer) {

  unsigned long interval = 0;

  switch (id) {
    case TIMER_20MSEC: interval = 20; break;
    case TIMER_100MSEC: interval = 100; break;
    case TIMER_1SEC: interval = 1000; break;
    case TIMER_30SEC: interval = 30000; break;
    case TIMER_MQTT: interval = timermqtt_interval; break;
    case TIMER_STATISTICS: interval = 30000; break;
    case TIMER_GRATUITOUS_ARP: interval = timer_gratuitous_arp_interval; break;



    case TIMER_MQTT_DELAY_QUEUE:
    case TIMER_C001_DELAY_QUEUE:
    case TIMER_C003_DELAY_QUEUE:
    case TIMER_C004_DELAY_QUEUE:
    case TIMER_C007_DELAY_QUEUE:
    case TIMER_C008_DELAY_QUEUE:
    case TIMER_C009_DELAY_QUEUE:
    case TIMER_C010_DELAY_QUEUE:
    case TIMER_C011_DELAY_QUEUE:
    case TIMER_C012_DELAY_QUEUE:
    case TIMER_C013_DELAY_QUEUE:
    case TIMER_C014_DELAY_QUEUE:
    case TIMER_C015_DELAY_QUEUE:
    case TIMER_C016_DELAY_QUEUE:
    case TIMER_C017_DELAY_QUEUE:
    case TIMER_C018_DELAY_QUEUE:
    case TIMER_C019_DELAY_QUEUE:
    case TIMER_C020_DELAY_QUEUE:
      interval = 1000; break;
  }
  unsigned long timer = lasttimer;
  setNextTimeInterval(timer, interval);
  setNewTimerAt(getMixedId(CONST_INTERVAL_TIMER, id), timer);
}

void sendGratuitousARP_now() {
  sendGratuitousARP();

  if (Settings.gratuitousARP()) {
    timer_gratuitous_arp_interval = 100;
    setIntervalTimer(TIMER_GRATUITOUS_ARP);
  }
}

void process_interval_timer(unsigned long id, unsigned long lasttimer) {


  setIntervalTimer(id, lasttimer);

  switch (id) {
    case TIMER_20MSEC: run50TimesPerSecond(); break;
    case TIMER_100MSEC:

      if (!UseRTOSMultitasking) {
        run10TimesPerSecond();
      }
      break;
    case TIMER_1SEC: runOncePerSecond(); break;
    case TIMER_30SEC: runEach30Seconds(); break;
    case TIMER_MQTT: runPeriodicalMQTT(); break;
    case TIMER_STATISTICS: logTimerStatistics(); break;
    case TIMER_GRATUITOUS_ARP:


      timer_gratuitous_arp_interval = 2 * timer_gratuitous_arp_interval;

      if (timer_gratuitous_arp_interval > TIMER_GRATUITOUS_ARP_MAX) {
        timer_gratuitous_arp_interval = TIMER_GRATUITOUS_ARP_MAX;
      }

      if (Settings.gratuitousARP()) {
        sendGratuitousARP();
      }
      break;
    case TIMER_MQTT_DELAY_QUEUE: processMQTTdelayQueue(); break;
  #ifdef USES_C001
    case TIMER_C001_DELAY_QUEUE:
      process_c001_delay_queue();
      break;
  #endif
  #ifdef USES_C003
    case TIMER_C003_DELAY_QUEUE:
      process_c003_delay_queue();
      break;
  #endif
  #ifdef USES_C004
    case TIMER_C004_DELAY_QUEUE:
      process_c004_delay_queue();
      break;
  #endif
  #ifdef USES_C007
    case TIMER_C007_DELAY_QUEUE:
      process_c007_delay_queue();
      break;
  #endif
  #ifdef USES_C008
    case TIMER_C008_DELAY_QUEUE:
      process_c008_delay_queue();
      break;
  #endif
  #ifdef USES_C009
    case TIMER_C009_DELAY_QUEUE:
      process_c009_delay_queue();
      break;
  #endif
  #ifdef USES_C010
    case TIMER_C010_DELAY_QUEUE:
      process_c010_delay_queue();
      break;
  #endif
  #ifdef USES_C011
    case TIMER_C011_DELAY_QUEUE:
      process_c011_delay_queue();
      break;
  #endif
  #ifdef USES_C012
    case TIMER_C012_DELAY_QUEUE:
      process_c012_delay_queue();
      break;
  #endif
# 294 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
  #ifdef USES_C015
    case TIMER_C015_DELAY_QUEUE:
      process_c015_delay_queue();
      break;
  #endif
  #ifdef USES_C016
    case TIMER_C016_DELAY_QUEUE:
      process_c016_delay_queue();
      break;
  #endif

  #ifdef USES_C017
    case TIMER_C017_DELAY_QUEUE:
      process_c017_delay_queue();
      break;
  #endif
# 337 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
  }
}




unsigned long createPluginTaskTimerId(byte plugin, int Par1) {
  const unsigned long mask = (1 << TIMER_ID_SHIFT) - 1;
  const unsigned long mixed = (Par1 << 8) + plugin;

  return mixed & mask;
}
# 357 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
void setPluginTaskTimer(unsigned long msecFromNow, byte plugin, short taskIndex, int Par1, int Par2, int Par3, int Par4, int Par5)
{

  const unsigned long systemTimerId = createPluginTaskTimerId(plugin, Par1);
  systemTimerStruct timer_data;

  timer_data.TaskIndex = taskIndex;
  timer_data.Par1 = Par1;
  timer_data.Par2 = Par2;
  timer_data.Par3 = Par3;
  timer_data.Par4 = Par4;
  timer_data.Par5 = Par5;
  systemTimers[systemTimerId] = timer_data;
  setTimer(PLUGIN_TASK_TIMER, systemTimerId, msecFromNow);
}

void process_plugin_task_timer(unsigned long id) {
  START_TIMER;
  const systemTimerStruct timer_data = systemTimers[id];
  struct EventStruct TempEvent;
  TempEvent.TaskIndex = timer_data.TaskIndex;
  TempEvent.Par1 = timer_data.Par1;
  TempEvent.Par2 = timer_data.Par2;
  TempEvent.Par3 = timer_data.Par3;
  TempEvent.Par4 = timer_data.Par4;
  TempEvent.Par5 = timer_data.Par5;


  TempEvent.Source = VALUE_SOURCE_SYSTEM;
  const int y = getPluginId(timer_data.TaskIndex);
# 397 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
  systemTimers.erase(id);

  if (y >= 0) {
    String dummy;
    Plugin_ptr[y](PLUGIN_TIMER_IN, &TempEvent, dummy);
  }
  STOP_TIMER(PROC_SYS_TIMER);
}





unsigned long createGPIOTimerId(byte pinNumber, int Par1) {
  const unsigned long mask = (1 << TIMER_ID_SHIFT) - 1;
  const unsigned long mixed = (Par1 << 8) + pinNumber;

  return mixed & mask;
}

void setGPIOTimer(unsigned long msecFromNow, int Par1, int Par2, int Par3, int Par4, int Par5)
{

  const unsigned long systemTimerId = createGPIOTimerId(Par1, Par2);

  setTimer(GPIO_TIMER, systemTimerId, msecFromNow);
}

void process_gpio_timer(unsigned long id) {

  byte pinNumber = id & 0xFF;
  byte pinStateValue = (id >> 8);

  digitalWrite(pinNumber, pinStateValue);
}
# 440 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
void schedule_task_device_timer_at_init(unsigned long task_index) {
  unsigned long runAt = millis();

  if (!isDeepSleepEnabled()) {



    runAt += (task_index * 37) + Settings.MessageDelay;
  } else {
    runAt += (task_index * 11) + 10;
  }
  schedule_task_device_timer(task_index, runAt);
}


void schedule_all_task_device_timers() {
  for (byte task = 0; task < TASKS_MAX; task++) {
    schedule_task_device_timer_at_init(task);
  }
}

void schedule_all_tasks_using_MQTT_controller() {
  int ControllerIndex = firstEnabledMQTTController();

  if (ControllerIndex < 0) { return; }

  for (byte task = 0; task < TASKS_MAX; task++) {
    if (Settings.TaskDeviceSendData[ControllerIndex][task] &&
        Settings.ControllerEnabled[ControllerIndex] &&
        Settings.Protocol[ControllerIndex])
    {
      schedule_task_device_timer_at_init(task);
    }
  }
}

void schedule_task_device_timer(unsigned long task_index, unsigned long runAt) {
# 488 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Scheduler.ino"
  if (task_index >= TASKS_MAX) { return; }
  byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[task_index]);

  if (!Device[DeviceIndex].TimerOption) { return; }

  if (Device[DeviceIndex].TimerOptional && (Settings.TaskDeviceTimer[task_index] == 0)) {
    return;
  }

  if (Settings.TaskDeviceEnabled[task_index]) {
    setNewTimerAt(getMixedId(TASK_DEVICE_TIMER, task_index), runAt);
  }
}

void process_task_device_timer(unsigned long task_index, unsigned long lasttimer) {
  unsigned long newtimer = Settings.TaskDeviceTimer[task_index];

  if (newtimer != 0) {
    newtimer = lasttimer + (newtimer * 1000);
    schedule_task_device_timer(task_index, newtimer);
  }
  START_TIMER;
  SensorSendTask(task_index);
  STOP_TIMER(SENSOR_SEND_TASK);
}







void schedule_plugin_task_event_timer(byte DeviceIndex, byte Function, struct EventStruct *event) {
  schedule_event_timer(TaskPluginEnum, DeviceIndex, Function, event);
}

void schedule_controller_event_timer(byte ProtocolIndex, byte Function, struct EventStruct *event) {
  schedule_event_timer(ControllerPluginEnum, ProtocolIndex, Function, event);
}

void schedule_notification_event_timer(byte NotificationProtocolIndex, byte Function, struct EventStruct *event) {
  schedule_event_timer(NotificationPluginEnum, NotificationProtocolIndex, Function, event);
}

void schedule_command_timer(const char *cmd, struct EventStruct *event, const char *line) {
  String cmdStr;

  cmdStr += cmd;
  String lineStr;
  lineStr += line;



  const int crc = calc_CRC16(cmdStr) ^ calc_CRC16(lineStr);
  const unsigned long mixedId = createSystemEventMixedId(CommandTimerEnum, static_cast<uint16_t>(crc));
  EventStructCommandWrapper eventWrapper(mixedId, *event);
  eventWrapper.cmd = cmdStr;
  eventWrapper.line = lineStr;
  EventQueue.push_back(eventWrapper);
}

void schedule_event_timer(PluginPtrType ptr_type, byte Index, byte Function, struct EventStruct *event) {
  const unsigned long mixedId = createSystemEventMixedId(ptr_type, Index, Function);



  EventQueue.emplace_back(mixedId, *event);
}

unsigned long createSystemEventMixedId(PluginPtrType ptr_type, uint16_t crc16) {
  unsigned long subId = ptr_type;

  subId = (subId << 16) + crc16;
  return getMixedId(SYSTEM_EVENT_QUEUE, subId);
}

unsigned long createSystemEventMixedId(PluginPtrType ptr_type, byte Index, byte Function) {
  unsigned long subId = ptr_type;

  subId = (subId << 8) + Index;
  subId = (subId << 8) + Function;
  return getMixedId(SYSTEM_EVENT_QUEUE, subId);
}

void process_system_event_queue() {
  if (EventQueue.size() == 0) { return; }
  unsigned long id = EventQueue.front().id;
  byte Function = id & 0xFF;
  byte Index = (id >> 8) & 0xFF;
  PluginPtrType ptr_type = static_cast<PluginPtrType>((id >> 16) & 0xFF);





  String tmpString;

  switch (ptr_type) {
    case TaskPluginEnum:
      LoadTaskSettings(EventQueue.front().event.TaskIndex);
      Plugin_ptr[Index](Function, &EventQueue.front().event, tmpString);
      break;
    case ControllerPluginEnum:
      CPluginCall(Index, Function, &EventQueue.front().event, tmpString);
      break;
    case NotificationPluginEnum:
      NPlugin_ptr[Index](Function, &EventQueue.front().event, tmpString);
      break;
    case CommandTimerEnum:
    {
      String status = doExecuteCommand(
        EventQueue.front().cmd.c_str(),
        &EventQueue.front().event,
        EventQueue.front().line.c_str());
      delay(0);
      SendStatus(EventQueue.front().event.Source, status);
      delay(0);
      break;
    }
  }
  EventQueue.pop_front();
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/Serial.ino"



#define INPUT_BUFFER_SIZE 128

byte SerialInByte;
int SerialInByteCounter = 0;
char InputBuffer_Serial[INPUT_BUFFER_SIZE + 2];

void serial()
{
  while (Serial.available())
  {
    delay(0);
    SerialInByte = Serial.read();

    if (SerialInByte == 255)
    {
      Serial.flush();
      return;
    }

    if (isprint(SerialInByte))
    {
      if (SerialInByteCounter < INPUT_BUFFER_SIZE) {
        InputBuffer_Serial[SerialInByteCounter++] = SerialInByte;
      }
    }

    if ((SerialInByte == '\r') || (SerialInByte == '\n'))
    {
      if (SerialInByteCounter == 0) {
        break;
      }
      InputBuffer_Serial[SerialInByteCounter] = 0;
      Serial.write('>');
      serialPrintln(InputBuffer_Serial);
      String action = InputBuffer_Serial;
      struct EventStruct TempEvent;
      action = parseTemplate(action, action.length());
      parseCommandString(&TempEvent, action);
      TempEvent.Source = VALUE_SOURCE_SERIAL;

      if (!PluginCall(PLUGIN_WRITE, &TempEvent, action)) {
        ExecuteCommand(VALUE_SOURCE_SERIAL, action.c_str());
      }
      SerialInByteCounter = 0;
      InputBuffer_Serial[0] = 0;
    }
  }
}

void addToSerialBuffer(const char *line) {
  process_serialWriteBuffer();
  const size_t line_length = strlen(line);
  int roomLeft = ESP.getFreeHeap();

  if (roomLeft < 500) {
    roomLeft = 0;
  } else if (roomLeft < 3000) {
    roomLeft = 128 - serialWriteBuffer.size();
  } else {
    roomLeft -= 3000;
  }

  if (roomLeft > 0) {
    size_t pos = 0;

    while (pos < line_length && pos < static_cast<size_t>(roomLeft)) {
      serialWriteBuffer.push_back(line[pos]);
      ++pos;
    }
  }
}

void addNewlineToSerialBuffer() {
  serialWriteBuffer.push_back('\r');
  serialWriteBuffer.push_back('\n');
}

void process_serialWriteBuffer() {
  if (serialWriteBuffer.size() == 0) { return; }
  size_t snip = 128;
#if defined(ESP8266)
  snip = Serial.availableForWrite();
#endif

  if (snip > 0) {
    size_t bytes_to_write = serialWriteBuffer.size();

    if (snip < bytes_to_write) { bytes_to_write = snip; }

    for (size_t i = 0; i < bytes_to_write; ++i) {
      const char c = serialWriteBuffer.front();
      Serial.write(c);
      serialWriteBuffer.pop_front();
    }
  }
}



void serialPrint(const String& text) {
  addToSerialBuffer(text.c_str());
  process_serialWriteBuffer();
}

void serialPrintln(const String& text) {
  addToSerialBuffer(text.c_str());
  addNewlineToSerialBuffer();
  process_serialWriteBuffer();
}

void serialPrintln() {
  addNewlineToSerialBuffer();
  process_serialWriteBuffer();
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/StringConverter.ino"





unsigned long str2int(const char *string)
{
  unsigned long temp = atof(string);

  return temp;
}




bool string2float(const String& string, float& floatvalue) {
  if (!isFloat(string)) { return false; }
  floatvalue = atof(string.c_str());
  return true;
}




boolean str2ip(const String& string, byte *IP) {
  return str2ip(string.c_str(), IP);
}

boolean str2ip(const char *string, byte *IP)
{
  IPAddress tmpip;

  if ((*string == 0) || tmpip.fromString(string)) {

    for (byte i = 0; i < 4; ++i) {
      IP[i] = tmpip[i];
    }
    return true;
  }
  return false;
}

String formatIP(const IPAddress& ip) {
#if defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  IPAddress tmp(ip);
  return tmp.toString();
#else
  return ip.toString();
#endif
}

void formatMAC(const uint8_t *mac, char (& strMAC)[20]) {
  sprintf_P(strMAC, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

String formatMAC(const uint8_t *mac) {
  char str[20];

  formatMAC(mac, str);
  return String(str);
}

String formatToHex(unsigned long value, const String& prefix) {
  String result = prefix;
  String hex(value, HEX);

  hex.toUpperCase();
  result += hex;
  return result;
}

String formatToHex(unsigned long value) {
  return formatToHex(value, F("0x"));
}

String formatHumanReadable(unsigned long value, unsigned long factor) {
  String result = formatHumanReadable(value, factor, 2);

  result.replace(F(".00"), "");
  return result;
}

String formatHumanReadable(unsigned long value, unsigned long factor, int NrDecimals) {
  float floatValue(value);
  byte steps = 0;

  while (value >= factor) {
    value /= factor;
    ++steps;
    floatValue /= float(factor);
  }
  String result = toString(floatValue, NrDecimals);

  switch (steps) {
    case 0: return String(value);
    case 1: result += 'k'; break;
    case 2: result += 'M'; break;
    case 3: result += 'G'; break;
    case 4: result += 'T'; break;
    default:
      result += '*';
      result += factor;
      result += '^';
      result += steps;
      break;
  }
  return result;
}

String formatToHex_decimal(unsigned long value) {
  return formatToHex_decimal(value, 1);
}

String formatToHex_decimal(unsigned long value, unsigned long factor) {
  String result = formatToHex(value);

  result += " (";

  if (factor > 1) {
    result += formatHumanReadable(value, factor);
  } else {
    result += value;
  }
  result += ')';
  return result;
}




String toString(float value, byte decimals)
{
  String sValue = String(value, decimals);

  sValue.trim();
  return sValue;
}

String toString(WiFiMode_t mode)
{
  String result = F("Undefined");

  switch (mode)
  {
    case WIFI_OFF:
      result = F("Off");
      break;
    case WIFI_STA:
      result = F("STA");
      break;
    case WIFI_AP:
      result = F("AP");
      break;
    case WIFI_AP_STA:
      result = F("AP+STA");
      break;
    default:
      break;
  }
  return result;
}

String toString(bool value) {
  return value ? F("true") : F("false");
}




void removeExtraNewLine(String& line) {
  while (line.endsWith("\r\n\r\n")) {
    line.remove(line.length() - 2);
  }
}

void addNewLine(String& line) {
  line += "\r\n";
}




String doFormatUserVar(byte TaskIndex, byte rel_index, bool mustCheck, bool& isvalid) {
  isvalid = true;
  const byte BaseVarIndex = TaskIndex * VARS_PER_TASK;
  const byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[TaskIndex]);

  if (Device[DeviceIndex].ValueCount <= rel_index) {
    isvalid = false;

    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("No sensor value for TaskIndex: ");
      log += TaskIndex;
      log += F(" varnumber: ");
      log += rel_index;
      addLog(LOG_LEVEL_ERROR, log);
    }
    return "";
  }

  if (Device[DeviceIndex].VType == SENSOR_TYPE_LONG) {
    return String((unsigned long)UserVar[BaseVarIndex] + ((unsigned long)UserVar[BaseVarIndex + 1] << 16));
  }
  float f(UserVar[BaseVarIndex + rel_index]);

  if (mustCheck && !isValidFloat(f)) {
    isvalid = false;
#ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("Invalid float value for TaskIndex: ");
      log += TaskIndex;
      log += F(" varnumber: ");
      log += rel_index;
      addLog(LOG_LEVEL_DEBUG, log);
    }
#endif
    f = 0;
  }
  return toString(f, ExtraTaskSettings.TaskDeviceValueDecimals[rel_index]);
}

String formatUserVarNoCheck(byte TaskIndex, byte rel_index) {
  bool isvalid;

  return doFormatUserVar(TaskIndex, rel_index, false, isvalid);
}

String formatUserVar(byte TaskIndex, byte rel_index, bool& isvalid) {
  return doFormatUserVar(TaskIndex, rel_index, true, isvalid);
}

String formatUserVarNoCheck(struct EventStruct *event, byte rel_index)
{
  return formatUserVarNoCheck(event->TaskIndex, rel_index);
}

String formatUserVar(struct EventStruct *event, byte rel_index, bool& isvalid)
{
  return formatUserVar(event->TaskIndex, rel_index, isvalid);
}




void wrap_String(const String& string, const String& wrap, String& result) {
  result += wrap;
  result += string;
  result += wrap;
}




String to_json_object_value(const String& object, const String& value) {
  String result;

  result.reserve(object.length() + value.length() + 6);
  wrap_String(object, "\"", result);
  result += F(":");

  if (value.length() == 0) {

    result += F("\"\"");
  } else if (!isFloat(value)) {

    if ((value.indexOf('\n') != -1) || (value.indexOf('\r') != -1) || (value.indexOf('"') != -1)) {

      String tmpValue(value);
      tmpValue.replace('\n', '^');
      tmpValue.replace('\r', '^');
      tmpValue.replace('"', '\'');
      wrap_String(tmpValue, "\"", result);
    } else {
      wrap_String(value, "\"", result);
    }
  } else {

    result += value;
  }
  return result;
}




String stripWrappingChar(const String& text, char wrappingChar) {
  unsigned int length = text.length();

  if ((length >= 2) && stringWrappedWithChar(text, wrappingChar)) {
    return text.substring(1, length - 1);
  }
  return text;
}

bool stringWrappedWithChar(const String& text, char wrappingChar) {
  unsigned int length = text.length();

  if (length < 2) { return false; }

  if (text.charAt(0) != wrappingChar) { return false; }
  return text.charAt(length - 1) == wrappingChar;
}

bool isQuoteChar(char c) {
  return c == '\'' || c == '"';
}

bool isParameterSeparatorChar(char c) {
  return c == ',' || c == ' ';
}

String stripQuotes(const String& text) {
  if (text.length() >= 2) {
    char c = text.charAt(0);

    if (isQuoteChar(c)) {
      return stripWrappingChar(text, c);
    }
  }
  return text;
}

bool safe_strncpy(char *dest, const String& source, size_t max_size) {
  return safe_strncpy(dest, source.c_str(), max_size);
}

bool safe_strncpy(char *dest, const char *source, size_t max_size) {
  if (max_size < 1) { return false; }

  if (dest == NULL) { return false; }

  if (source == NULL) { return false; }
  bool result = true;
  memset(dest, 0, max_size);
  size_t str_length = strlen(source);

  if (str_length >= max_size) {
    str_length = max_size;
    result = false;
  }
  strncpy(dest, source, str_length);
  dest[max_size - 1] = 0;
  return result;
}


String to_internal_string(const String& input) {
  String result = input;

  result.trim();
  result.toLowerCase();
  result.replace(' ', '_');
  return result;
}




String parseString(const String& string, byte indexFind, bool toEndOfString, bool toLowerCase) {
  int startpos = 0;

  if (indexFind > 0) {
    startpos = getParamStartPos(string, indexFind);

    if (startpos < 0) {
      return "";
    }
  }
  const int endpos = getParamStartPos(string, indexFind + 1);
  String result;

  if (toEndOfString || (endpos <= 0)) {
    result = string.substring(startpos);
  } else {
    result = string.substring(startpos, endpos - 1);
  }

  if (toLowerCase) {
    result.toLowerCase();
  }
  return stripQuotes(result);
}

String parseString(const String& string, byte indexFind) {
  return parseString(string, indexFind, false, true);
}

String parseStringKeepCase(const String& string, byte indexFind) {
  return parseString(string, indexFind, false, false);
}

String parseStringToEnd(const String& string, byte indexFind) {
  return parseString(string, indexFind, true, true);
}

String parseStringToEndKeepCase(const String& string, byte indexFind) {
  return parseString(string, indexFind, true, false);
}




int getParamStartPos(const String& string, byte indexFind)
{

  if (indexFind <= 1) { return 0; }
  byte count = 1;
  bool quotedStringActive = false;
  char quoteStartChar = '"';
  unsigned int lastParamStartPos = 0;
  const unsigned int strlength = string.length();

  if (strlength < indexFind) { return -1; }

  for (unsigned int x = 0; x < (strlength - 1); ++x)
  {
    const char c = string.charAt(x);


    if (!quotedStringActive) {
      if (isQuoteChar(c)) {

        if (lastParamStartPos == x) {
          quotedStringActive = true;
          quoteStartChar = c;
        }
      }
    } else {
      if (c == quoteStartChar) {

        quotedStringActive = false;
      }
    }


    if (!quotedStringActive) {
      if (isParameterSeparatorChar(c))
      {
        lastParamStartPos = x + 1;
        ++count;

        if (count == indexFind) {
          return lastParamStartPos;
        }
      }
    }
  }
  return -1;
}


void htmlEscape(String& html)
{
  html.replace("&", F("&amp;"));
  html.replace("\"", F("&quot;"));
  html.replace("'", F("&#039;"));
  html.replace("<", F("&lt;"));
  html.replace(">", F("&gt;"));
  html.replace("/", F("&#047;"));
}

void htmlStrongEscape(String& html)
{
  String escaped;

  escaped.reserve(html.length());

  for (unsigned i = 0; i < html.length(); ++i)
  {
    if (((html[i] >= 'a') && (html[i] <= 'z')) || ((html[i] >= 'A') && (html[i] <= 'Z')) || ((html[i] >= '0') && (html[i] <= '9')))
    {
      escaped += html[i];
    }
    else
    {
      char s[4];
      sprintf(s, "%03d", static_cast<int>(html[i]));
      escaped += "&#";
      escaped += s;
      escaped += ";";
    }
  }
  html = escaped;
}




void parseControllerVariables(String& s, struct EventStruct *event, boolean useURLencode) {
  parseSystemVariables(s, useURLencode);
  parseEventVariables(s, event, useURLencode);
  parseStandardConversions(s, useURLencode);
}

void repl(const String& key, const String& val, String& s, boolean useURLencode)
{
  if (useURLencode) {
    s.replace(key, URLEncode(val.c_str()));
  } else {
    s.replace(key, val);
  }
}

void parseSpecialCharacters(String& s, boolean useURLencode)
{
  bool no_accolades = s.indexOf('{') == -1 || s.indexOf('}') == -1;
  bool no_html_entity = s.indexOf('&') == -1 || s.indexOf(';') == -1;

  if (no_accolades && no_html_entity) {
    return;
  }
  {

    const char degree[3] = { 0xc2, 0xb0, 0 };
    const char degreeC[4] = { 0xe2, 0x84, 0x83, 0 };
    const char degree_C[4] = { 0xc2, 0xb0, 'C', 0 };
    repl(F("{D}"), degree, s, useURLencode);
    repl(F("&deg;"), degree, s, useURLencode);
    repl(degreeC, degree_C, s, useURLencode);
  }
  {

    const char laquo[3] = { 0xc2, 0xab, 0 };
    const char raquo[3] = { 0xc2, 0xbb, 0 };
    repl(F("{<<}"), laquo, s, useURLencode);
    repl(F("&laquo;"), laquo, s, useURLencode);
    repl(F("{>>}"), raquo, s, useURLencode);
    repl(F("&raquo;"), raquo, s, useURLencode);
  }
  {

    const char mu[3] = { 0xc2, 0xb5, 0 };
    repl(F("{u}"), mu, s, useURLencode);
    repl(F("&micro;"), mu, s, useURLencode);
  }
  {

    const char euro[4] = { 0xe2, 0x82, 0xac, 0 };
    const char yen[3] = { 0xc2, 0xa5, 0 };
    const char pound[3] = { 0xc2, 0xa3, 0 };
    const char cent[3] = { 0xc2, 0xa2, 0 };
    repl(F("{E}"), euro, s, useURLencode);
    repl(F("&euro;"), euro, s, useURLencode);
    repl(F("{Y}"), yen, s, useURLencode);
    repl(F("&yen;"), yen, s, useURLencode);
    repl(F("{P}"), pound, s, useURLencode);
    repl(F("&pound;"), pound, s, useURLencode);
    repl(F("{c}"), cent, s, useURLencode);
    repl(F("&cent;"), cent, s, useURLencode);
  }
  {

    const char sup1[3] = { 0xc2, 0xb9, 0 };
    const char sup2[3] = { 0xc2, 0xb2, 0 };
    const char sup3[3] = { 0xc2, 0xb3, 0 };
    const char frac14[3] = { 0xc2, 0xbc, 0 };
    const char frac12[3] = { 0xc2, 0xbd, 0 };
    const char frac34[3] = { 0xc2, 0xbe, 0 };
    const char plusmn[3] = { 0xc2, 0xb1, 0 };
    const char times[3] = { 0xc3, 0x97, 0 };
    const char divide[3] = { 0xc3, 0xb7, 0 };
    repl(F("{^1}"), sup1, s, useURLencode);
    repl(F("&sup1;"), sup1, s, useURLencode);
    repl(F("{^2}"), sup2, s, useURLencode);
    repl(F("&sup2;"), sup2, s, useURLencode);
    repl(F("{^3}"), sup3, s, useURLencode);
    repl(F("&sup3;"), sup3, s, useURLencode);
    repl(F("{1_4}"), frac14, s, useURLencode);
    repl(F("&frac14;"), frac14, s, useURLencode);
    repl(F("{1_2}"), frac12, s, useURLencode);
    repl(F("&frac12;"), frac12, s, useURLencode);
    repl(F("{3_4}"), frac34, s, useURLencode);
    repl(F("&frac34;"), frac34, s, useURLencode);
    repl(F("{+-}"), plusmn, s, useURLencode);
    repl(F("&plusmn;"), plusmn, s, useURLencode);
    repl(F("{x}"), times, s, useURLencode);
    repl(F("&times;"), times, s, useURLencode);
    repl(F("{..}"), divide, s, useURLencode);
    repl(F("&divide;"), divide, s, useURLencode);
  }
}


#define SMART_REPL(T,S) \
  if (s.indexOf(T) != -1) { repl((T), (S), s, useURLencode); }
#define SMART_REPL_T(T,S) \
  if (s.indexOf(T) != -1) { (S((T), s, useURLencode)); }
void parseSystemVariables(String& s, boolean useURLencode)
{
  parseSpecialCharacters(s, useURLencode);

  if (s.indexOf('%') == -1) {
    return;
  }
  #if FEATURE_ADC_VCC
  repl(F("%vcc%"), String(vcc), s, useURLencode);
  #endif
  repl(F("%CR%"), "\r", s, useURLencode);
  repl(F("%LF%"), "\n", s, useURLencode);
  repl(F("%SP%"), " ", s, useURLencode);
  repl(F("%R%"), F("\\r"), s, useURLencode);
  repl(F("%N%"), F("\\n"), s, useURLencode);
  SMART_REPL(F("%ip4%"), WiFi.localIP().toString().substring(WiFi.localIP().toString().lastIndexOf('.') + 1))
  SMART_REPL(F("%ip%"), WiFi.localIP().toString())
  SMART_REPL(F("%rssi%"), String((wifiStatus == ESPEASY_WIFI_DISCONNECTED) ? 0 : WiFi.RSSI()))
  SMART_REPL(F("%ssid%"), (wifiStatus == ESPEASY_WIFI_DISCONNECTED) ? F("--") : WiFi.SSID())
  SMART_REPL(F("%bssid%"), (wifiStatus == ESPEASY_WIFI_DISCONNECTED) ? F("00:00:00:00:00:00") : WiFi.BSSIDstr())
  SMART_REPL(F("%wi_ch%"), String((wifiStatus == ESPEASY_WIFI_DISCONNECTED) ? 0 : WiFi.channel()))
  SMART_REPL(F("%unit%"), String(Settings.Unit))
  SMART_REPL(F("%mac%"), String(WiFi.macAddress()))
  #if defined(ESP8266)
  SMART_REPL(F("%mac_int%"), String(ESP.getChipId()))
  #endif

  if (s.indexOf(F("%sys")) != -1) {
    SMART_REPL(F("%sysload%"), String(getCPUload()))
    SMART_REPL(F("%sysheap%"), String(ESP.getFreeHeap()));
    SMART_REPL(F("%systm_hm%"), getTimeString(':', false))
    SMART_REPL(F("%systm_hm_am%"), getTimeString_ampm(':', false))
    SMART_REPL(F("%systime%"), getTimeString(':'))
    SMART_REPL(F("%systime_am%"), getTimeString_ampm(':'))
    SMART_REPL(F("%sysbuild_date%"), String(CRCValues.compileDate))
    SMART_REPL(F("%sysbuild_time%"), String(CRCValues.compileTime))
    repl(F("%sysname%"), Settings.Name, s, useURLencode);


    char valueString[5];
    #define SMART_REPL_TIME(T, F, V) \
  if (s.indexOf(T) != -1) { sprintf_P(valueString, (F), (V)); repl((T), valueString, s, useURLencode); }
    SMART_REPL_TIME(F("%sysyear%"), PSTR("%d"), year())
    SMART_REPL_TIME(F("%sysmonth%"), PSTR("%d"), month())
    SMART_REPL_TIME(F("%sysday%"), PSTR("%d"), day())
    SMART_REPL_TIME(F("%syshour%"), PSTR("%d"), hour())
    SMART_REPL_TIME(F("%sysmin%"), PSTR("%d"), minute())
    SMART_REPL_TIME(F("%syssec%"), PSTR("%d"), second())
    SMART_REPL_TIME(F("%syssec_d%"), PSTR("%d"), ((hour() * 60) + minute()) * 60 + second());
    SMART_REPL(F("%sysweekday%"), String(weekday()))
    SMART_REPL(F("%sysweekday_s%"), weekday_str())


    SMART_REPL_TIME(F("%sysyears%"), PSTR("%02d"), year() % 100)
    SMART_REPL_TIME(F("%sysyear_0%"), PSTR("%04d"), year())
    SMART_REPL_TIME(F("%syshour_0%"), PSTR("%02d"), hour())
    SMART_REPL_TIME(F("%sysday_0%"), PSTR("%02d"), day())
    SMART_REPL_TIME(F("%sysmin_0%"), PSTR("%02d"), minute())
    SMART_REPL_TIME(F("%syssec_0%"), PSTR("%02d"), second())
    SMART_REPL_TIME(F("%sysmonth_0%"), PSTR("%02d"), month())

    #undef SMART_REPL_TIME
  }
  SMART_REPL(F("%lcltime%"), getDateTimeString('-', ':', ' '))
  SMART_REPL(F("%lcltime_am%"), getDateTimeString_ampm('-', ':', ' '))
  SMART_REPL(F("%uptime%"), String(wdcounter / 2))
  SMART_REPL(F("%unixtime%"), String(getUnixTime()))
  SMART_REPL_T(F("%sunset"), replSunSetTimeString)
  SMART_REPL_T(F("%sunrise"), replSunRiseTimeString)

  if (s.indexOf(F("%is")) != -1) {
    SMART_REPL(F("%ismqtt%"), String(MQTTclient_connected));
    SMART_REPL(F("%iswifi%"), String(wifiStatus));
    SMART_REPL(F("%isntp%"), String(statusNTPInitialized));
    #ifdef USES_P037
    SMART_REPL(F("%ismqttimp%"), String(P037_MQTTImport_connected));
    #endif
  }
  const int v_index = s.indexOf("%v");

  if ((v_index != -1) && isDigit(s[v_index + 2])) {
    for (byte i = 0; i < CUSTOM_VARS_MAX; ++i) {
      SMART_REPL("%v" + toString(i + 1, 0) + '%', String(customFloatVar[i]))
    }
  }
}

String getReplacementString(const String& format, String& s) {
  int startpos = s.indexOf(format);
  int endpos = s.indexOf('%', startpos + 1);
  String R = s.substring(startpos, endpos + 1);

#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("ReplacementString SunTime: ");
    log += R;
    log += F(" offset: ");
    log += getSecOffset(R);
    addLog(LOG_LEVEL_DEBUG, log);
  }
#endif
  return R;
}

void replSunRiseTimeString(const String& format, String& s, boolean useURLencode) {
  String R = getReplacementString(format, s);

  repl(R, getSunriseTimeString(':', getSecOffset(R)), s, useURLencode);
}

void replSunSetTimeString(const String& format, String& s, boolean useURLencode) {
  String R = getReplacementString(format, s);

  repl(R, getSunsetTimeString(':', getSecOffset(R)), s, useURLencode);
}

void parseEventVariables(String& s, struct EventStruct *event, boolean useURLencode)
{

  LoadTaskSettings(event->TaskIndex);
  SMART_REPL(F("%id%"), String(event->idx))

  if (s.indexOf(F("%val")) != -1) {
    if (event->sensorType == SENSOR_TYPE_LONG) {
      SMART_REPL(F("%val1%"), String((unsigned long)UserVar[event->BaseVarIndex] + ((unsigned long)UserVar[event->BaseVarIndex + 1] << 16)))
    } else {
      SMART_REPL(F("%val1%"), formatUserVarNoCheck(event, 0))
      SMART_REPL(F("%val2%"), formatUserVarNoCheck(event, 1))
      SMART_REPL(F("%val3%"), formatUserVarNoCheck(event, 2))
      SMART_REPL(F("%val4%"), formatUserVarNoCheck(event, 3))
    }
  }


  repl(F("%tskname%"), ExtraTaskSettings.TaskDeviceName, s, useURLencode);

  if (s.indexOf(F("%vname")) != -1) {
    repl(F("%vname1%"), ExtraTaskSettings.TaskDeviceValueNames[0], s, useURLencode);
    repl(F("%vname2%"), ExtraTaskSettings.TaskDeviceValueNames[1], s, useURLencode);
    repl(F("%vname3%"), ExtraTaskSettings.TaskDeviceValueNames[2], s, useURLencode);
    repl(F("%vname4%"), ExtraTaskSettings.TaskDeviceValueNames[3], s, useURLencode);
  }
}

#undef SMART_REPL_T
#undef SMART_REPL

bool getConvertArgument(const String& marker, const String& s, float& argument, int& startIndex, int& endIndex) {
  String argumentString;

  if (getConvertArgumentString(marker, s, argumentString, startIndex, endIndex)) {
    if (!isFloat(argumentString)) { return false; }
    argument = argumentString.toFloat();
    return true;
  }
  return false;
}

bool getConvertArgument2(const String& marker, const String& s, float& arg1, float& arg2, int& startIndex, int& endIndex) {
  String argumentString;

  if (getConvertArgumentString(marker, s, argumentString, startIndex, endIndex)) {
    int pos_comma = argumentString.indexOf(',');

    if (pos_comma == -1) { return false; }
    String arg1_s = argumentString.substring(0, pos_comma);

    if (!isFloat(arg1_s)) { return false; }
    String arg2_s = argumentString.substring(pos_comma + 1);

    if (!isFloat(arg2_s)) { return false; }
    arg1 = arg1_s.toFloat();
    arg2 = arg2_s.toFloat();
    return true;
  }
  return false;
}

bool getConvertArgumentString(const String& marker, const String& s, String& argumentString, int& startIndex, int& endIndex) {
  startIndex = s.indexOf(marker);

  if (startIndex == -1) { return false; }

  int startIndexArgument = startIndex + marker.length();

  if (s.charAt(startIndexArgument) != '(') {
    return false;
  }
  ++startIndexArgument;
  endIndex = s.indexOf(')', startIndexArgument);

  if (endIndex == -1) { return false; }

  argumentString = s.substring(startIndexArgument, endIndex);

  if (argumentString.length() == 0) { return false; }
  ++endIndex;
  return true;
}



void parseStandardConversions(String& s, boolean useURLencode) {
  if (s.indexOf(F("%c_")) == -1) {
    return;
  }
  float arg1 = 0.0;
  int startIndex = 0;
  int endIndex = 0;




  #define SMART_CONV(T, FUN) \
  while (getConvertArgument((T), s, arg1, startIndex, endIndex)) { repl(s.substring(startIndex, endIndex), (FUN), s, useURLencode); }
  SMART_CONV(F("%c_w_dir%"), getBearing(arg1))
  SMART_CONV(F("%c_c2f%"), toString(CelsiusToFahrenheit(arg1), 2))
  SMART_CONV(F("%c_ms2Bft%"), String(m_secToBeaufort(arg1)))
  SMART_CONV(F("%c_cm2imp%"), centimeterToImperialLength(arg1))
  SMART_CONV(F("%c_mm2imp%"), millimeterToImperialLength(arg1))
  SMART_CONV(F("%c_m2day%"), toString(minutesToDay(arg1), 2))
  SMART_CONV(F("%c_m2dh%"), minutesToDayHour(arg1))
  SMART_CONV(F("%c_m2dhm%"), minutesToDayHourMinute(arg1))
  SMART_CONV(F("%c_s2dhms%"), secondsToDayHourMinuteSecond(arg1))
  #undef SMART_CONV


  #define SMART_CONV(T, FUN) \
  while (getConvertArgument2((T), s, arg1, arg2, startIndex, endIndex)) { repl(s.substring(startIndex, endIndex), (FUN), s, useURLencode); }
  float arg2 = 0.0;
  SMART_CONV(F("%c_dew_th%"), toString(compute_dew_point_temp(arg1, arg2), 2))
  #undef SMART_CONV
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/StringProvider.ino"
#include "StringProviderTypes.h"

String getInternalLabel(LabelType::Enum label) {
  return to_internal_string(getLabel(label));
}

String getLabel(LabelType::Enum label) {
  switch (label)
  {
    case LabelType::UNIT_NR: return F("Unit Number");
    case LabelType::UNIT_NAME: return F("Unit Name");
    case LabelType::HOST_NAME: return F("Hostname");

    case LabelType::LOCAL_TIME: return F("Local Time");
    case LabelType::UPTIME: return F("Uptime");
    case LabelType::LOAD_PCT: return F("Load");
    case LabelType::LOOP_COUNT: return F("Load LC");
    case LabelType::CPU_ECO_MODE: return F("CPU Eco Mode");

    case LabelType::FREE_MEM: return F("Free RAM");
    case LabelType::FREE_STACK: return F("Free Stack");
#ifdef CORE_POST_2_5_0
    case LabelType::HEAP_MAX_FREE_BLOCK: return F("Heap Max Free Block");
    case LabelType::HEAP_FRAGMENTATION: return F("Heap Fragmentation");
#endif

    case LabelType::BOOT_TYPE: return F("Last Boot Cause");
    case LabelType::BOOT_COUNT: return F("Boot Count");
    case LabelType::RESET_REASON: return F("Reset Reason");
    case LabelType::LAST_TASK_BEFORE_REBOOT: return F("Last Task");
    case LabelType::SW_WD_COUNT: return F("SW WD count");


    case LabelType::WIFI_CONNECTION: return F("WiFi Connection");
    case LabelType::WIFI_RSSI: return F("RSSI");
    case LabelType::IP_CONFIG: return F("IP Config");
    case LabelType::IP_CONFIG_STATIC: return F("Static");
    case LabelType::IP_CONFIG_DYNAMIC: return F("DHCP");
    case LabelType::IP_ADDRESS: return F("IP Address");
    case LabelType::IP_SUBNET: return F("IP Subnet");
    case LabelType::IP_ADDRESS_SUBNET: return F("IP / Subnet");
    case LabelType::GATEWAY: return F("Gateway");
    case LabelType::CLIENT_IP: return F("Client IP");
    case LabelType::DNS: return F("DNS");
    case LabelType::DNS_1: return F("DNS 1");
    case LabelType::DNS_2: return F("DNS 2");
    case LabelType::ALLOWED_IP_RANGE: return F("Allowed IP Range");
    case LabelType::STA_MAC: return F("STA MAC");
    case LabelType::AP_MAC: return F("AP MAC");
    case LabelType::SSID: return F("SSID");
    case LabelType::BSSID: return F("BSSID");
    case LabelType::CHANNEL: return F("Channel");
    case LabelType::CONNECTED: return F("Connected");
    case LabelType::CONNECTED_MSEC: return F("Connected msec");
    case LabelType::LAST_DISCONNECT_REASON: return F("Last Disconnect Reason");
    case LabelType::LAST_DISC_REASON_STR: return F("Last Disconnect Reason str");
    case LabelType::NUMBER_RECONNECTS: return F("Number Reconnects");

    case LabelType::FORCE_WIFI_BG: return F("Force WiFi B/G");
    case LabelType::RESTART_WIFI_LOST_CONN: return F("Restart WiFi Lost Conn");
    case LabelType::FORCE_WIFI_NOSLEEP: return F("Force WiFi No Sleep");
    case LabelType::PERIODICAL_GRAT_ARP: return F("Periodical send Gratuitous ARP");
    case LabelType::CONNECTION_FAIL_THRESH: return F("Connection Failure Threshold");

    case LabelType::BUILD_DESC: return F("Build");
    case LabelType::GIT_BUILD: return F("Git Build");
    case LabelType::SYSTEM_LIBRARIES: return F("System Libraries");
    case LabelType::PLUGINS: return F("Plugins");
    case LabelType::PLUGIN_DESCRIPTION: return F("Plugin description");
    case LabelType::BUILD_TIME: return F("Build Time");
    case LabelType::BINARY_FILENAME: return F("Binary Filename");

    case LabelType::SYSLOG_LOG_LEVEL: return F("Syslog Log Level");
    case LabelType::SERIAL_LOG_LEVEL: return F("Serial Log Level");
    case LabelType::WEB_LOG_LEVEL: return F("Web Log Level");
  #ifdef FEATURE_SD
    case LabelType::SD_LOG_LEVEL: return F("SD Log Level");
  #endif

    case LabelType::ESP_CHIP_ID: return F("ESP Chip ID");
    case LabelType::ESP_CHIP_FREQ: return F("ESP Chip Frequency");
    case LabelType::ESP_BOARD_NAME: return F("ESP Board Name");

    case LabelType::FLASH_CHIP_ID: return F("Flash Chip ID");
    case LabelType::FLASH_CHIP_REAL_SIZE: return F("Flash Chip Real Size");
    case LabelType::FLASH_IDE_SIZE: return F("Flash IDE Size");
    case LabelType::FLASH_IDE_SPEED: return F("Flash IDE Speed");
    case LabelType::FLASH_IDE_MODE: return F("Flash IDE Mode");
    case LabelType::FLASH_WRITE_COUNT: return F("Flash Writes");
    case LabelType::SKETCH_SIZE: return F("Sketch Size");
    case LabelType::SKETCH_FREE: return F("Sketch Free");
    case LabelType::SPIFFS_SIZE: return F("SPIFFS Size");
    case LabelType::SPIFFS_FREE: return F("SPIFFS Free");

  }
  return F("MissingString");
}



String getValue(LabelType::Enum label) {
  switch (label)
  {
    case LabelType::UNIT_NR: return String(Settings.Unit);
    case LabelType::UNIT_NAME: return String(Settings.Name);
    case LabelType::HOST_NAME:
    #ifdef ESP32
      return WiFi.getHostname();
    #else
      return WiFi.hostname();
    #endif

    case LabelType::LOCAL_TIME: return getDateTimeString('-',':',' ');
    case LabelType::UPTIME: return String(wdcounter / 2);
    case LabelType::LOAD_PCT: return String(getCPUload());
    case LabelType::LOOP_COUNT: return String(getLoopCountPerSec());
    case LabelType::CPU_ECO_MODE: return jsonBool(Settings.EcoPowerMode());

    case LabelType::FREE_MEM: return String(ESP.getFreeHeap());
    case LabelType::FREE_STACK: break;
#ifdef CORE_POST_2_5_0
    case LabelType::HEAP_MAX_FREE_BLOCK: return String(ESP.getMaxFreeBlockSize());
    case LabelType::HEAP_FRAGMENTATION: return String(ESP.getHeapFragmentation());
#endif

    case LabelType::BOOT_TYPE: return getLastBootCauseString();
    case LabelType::BOOT_COUNT: break;
    case LabelType::RESET_REASON: return getResetReasonString();
    case LabelType::LAST_TASK_BEFORE_REBOOT: return decodeSchedulerId(lastMixedSchedulerId_beforereboot);
    case LabelType::SW_WD_COUNT: return String(sw_watchdog_callback_count);

    case LabelType::WIFI_CONNECTION: break;
    case LabelType::WIFI_RSSI: return String(WiFi.RSSI());
    case LabelType::IP_CONFIG: return useStaticIP() ? getLabel(LabelType::IP_CONFIG_STATIC) : getLabel(LabelType::IP_CONFIG_DYNAMIC);
    case LabelType::IP_CONFIG_STATIC: break;
    case LabelType::IP_CONFIG_DYNAMIC: break;
    case LabelType::IP_ADDRESS: return WiFi.localIP().toString();
    case LabelType::IP_SUBNET: return WiFi.subnetMask().toString();
    case LabelType::IP_ADDRESS_SUBNET: return String(getValue(LabelType::IP_ADDRESS) + F(" / ") + getValue(LabelType::IP_SUBNET));
    case LabelType::GATEWAY: return WiFi.gatewayIP().toString();
    case LabelType::CLIENT_IP: return formatIP(WebServer.client().remoteIP());
    case LabelType::DNS: return String(getValue(LabelType::DNS_1) + F(" / ") + getValue(LabelType::DNS_2));
    case LabelType::DNS_1: return WiFi.dnsIP(0).toString();
    case LabelType::DNS_2: return WiFi.dnsIP(1).toString();
    case LabelType::ALLOWED_IP_RANGE: return describeAllowedIPrange();
    case LabelType::STA_MAC: return WiFi.macAddress();
    case LabelType::AP_MAC: break;
    case LabelType::SSID: return WiFi.SSID();
    case LabelType::BSSID: return WiFi.BSSIDstr();
    case LabelType::CHANNEL: return String(WiFi.channel());
    case LabelType::CONNECTED: return format_msec_duration(timeDiff(lastConnectMoment, millis()));
    case LabelType::CONNECTED_MSEC: return String(timeDiff(lastConnectMoment, millis()));
    case LabelType::LAST_DISCONNECT_REASON: return String(lastDisconnectReason);
    case LabelType::LAST_DISC_REASON_STR: return getLastDisconnectReason();
    case LabelType::NUMBER_RECONNECTS: return String(wifi_reconnects);

    case LabelType::FORCE_WIFI_BG: return jsonBool(Settings.ForceWiFi_bg_mode());
    case LabelType::RESTART_WIFI_LOST_CONN: return jsonBool(Settings.WiFiRestart_connection_lost());
    case LabelType::FORCE_WIFI_NOSLEEP: return jsonBool(Settings.WifiNoneSleep());
    case LabelType::PERIODICAL_GRAT_ARP: return jsonBool(Settings.gratuitousARP());
    case LabelType::CONNECTION_FAIL_THRESH: return String(Settings.ConnectionFailuresThreshold);

    case LabelType::BUILD_DESC: return String(BUILD);
    case LabelType::GIT_BUILD: return String(F(BUILD_GIT));
    case LabelType::SYSTEM_LIBRARIES: return getSystemLibraryString();
    case LabelType::PLUGINS: return String(deviceCount + 1);
    case LabelType::PLUGIN_DESCRIPTION: return getPluginDescriptionString();
    case LabelType::BUILD_TIME: break;
    case LabelType::BINARY_FILENAME: break;

    case LabelType::SYSLOG_LOG_LEVEL: return getLogLevelDisplayString(Settings.SyslogLevel);
    case LabelType::SERIAL_LOG_LEVEL: return getLogLevelDisplayString(getSerialLogLevel());
    case LabelType::WEB_LOG_LEVEL: return getLogLevelDisplayString(getWebLogLevel());
  #ifdef FEATURE_SD
    case LabelType::SD_LOG_LEVEL: return getLogLevelDisplayString(Settings.SDLogLevel);
  #endif

    case LabelType::ESP_CHIP_ID: break;
    case LabelType::ESP_CHIP_FREQ: break;
    case LabelType::ESP_BOARD_NAME: break;

    case LabelType::FLASH_CHIP_ID: break;
    case LabelType::FLASH_CHIP_REAL_SIZE: break;
    case LabelType::FLASH_IDE_SIZE: break;
    case LabelType::FLASH_IDE_SPEED: break;
    case LabelType::FLASH_IDE_MODE: break;
    case LabelType::FLASH_WRITE_COUNT: break;
    case LabelType::SKETCH_SIZE: break;
    case LabelType::SKETCH_FREE: break;
    case LabelType::SPIFFS_SIZE: break;
    case LabelType::SPIFFS_FREE: break;

  }
  return F("MissingString");
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/TimeESPeasy.ino"





#define SECS_PER_MIN (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY (SECS_PER_HOUR * 24UL)
#define DAYS_PER_WEEK (7UL)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52UL)
#define SECS_YR_2000 (946684800UL)
#define LEAP_YEAR(Y) (((1970 + Y) > 0) && !((1970 + Y) % 4) && (((1970 + Y) % 100) || !((1970 + Y) % 400)))
#include <time.h>

struct tm tm;
uint32_t syncInterval = 3600;
double sysTime = 0.0;
uint32_t prevMillis = 0;
uint32_t nextSyncTime = 0;
double externalTimeSource = -1.0;
struct tm tsRise, tsSet;
struct tm sunRise;
struct tm sunSet;

byte PrevMinutes = 0;

float sunDeclination(int doy) {


  return 0.409526325277017 * sin(0.0169060504029192 * (doy - 80.0856919827619));
}

float diurnalArc(float dec, float lat) {

  float rad = 0.0174532925;
  float height = -50.0 / 60.0 * rad;
  float latRad = lat * rad;

  return 12.0 * acos((sin(height) - sin(latRad) * sin(dec)) / (cos(latRad) * cos(dec))) / 3.1415926536;
}

float equationOfTime(int doy) {


  return -0.170869921174742 * sin(0.0336997028793971 * doy + 0.465419984181394) - 0.129890681040717 * sin(
    0.0178674832556871 * doy - 0.167936777524864);
}

int dayOfYear(int year, int month, int day) {

  int z = 14 - month;

  z /= 12;
  int y = year + 4800 - z;
  int m = month + 12 * z - 3;
  int j = 153 * m + 2;
  j = j / 5 + day + y * 365 + y / 4 - y / 100 + y / 400 - 32045;
  y = year + 4799;
  int k = y * 365 + y / 4 - y / 100 + y / 400 - 31738;
  return j - k + 1;
}

void calcSunRiseAndSet() {
  int doy = dayOfYear(tm.tm_year, tm.tm_mon, tm.tm_mday);
  float eqt = equationOfTime(doy);
  float dec = sunDeclination(doy);
  float da = diurnalArc(dec, Settings.Latitude);
  float rise = 12 - da - eqt;
  float set = 12 + da - eqt;

  tsRise.tm_hour = (int)rise;
  tsRise.tm_min = (rise - (int)rise) * 60.0;
  tsSet.tm_hour = (int)set;
  tsSet.tm_min = (set - (int)set) * 60.0;
  tsRise.tm_mday = tsSet.tm_mday = tm.tm_mday;
  tsRise.tm_mon = tsSet.tm_mon = tm.tm_mon;
  tsRise.tm_year = tsSet.tm_year = tm.tm_year;


  int secOffset_longitude = -1.0 * (Settings.Longitude / 15.0) * 3600;
  tsSet = addSeconds(tsSet, secOffset_longitude, false);
  tsRise = addSeconds(tsRise, secOffset_longitude, false);

  breakTime(toLocal(makeTime(tsRise)), sunRise);
  breakTime(toLocal(makeTime(tsSet)), sunSet);
}

struct tm getSunRise(int secOffset) {
  return addSeconds(tsRise, secOffset, true);
}

struct tm getSunSet(int secOffset) {
  return addSeconds(tsSet, secOffset, true);
}

struct tm addSeconds(const struct tm& ts, int seconds, bool toLocalTime) {
  unsigned long time = makeTime(ts);

  time += seconds;

  if (toLocalTime) {
    time = toLocal(time);
  }
  struct tm result;
  breakTime(time, result);
  return result;
}

void breakTime(unsigned long timeInput, struct tm& tm) {
  uint8_t year;
  uint8_t month, monthLength;
  uint32_t time;
  unsigned long days;
  const uint8_t monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  time = (uint32_t)timeInput;
  tm.tm_sec = time % 60;
  time /= 60;
  tm.tm_min = time % 60;
  time /= 60;
  tm.tm_hour = time % 24;
  time /= 24;
  tm.tm_wday = ((time + 4) % 7) + 1;

  year = 0;
  days = 0;

  while ((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  tm.tm_year = year;

  days -= LEAP_YEAR(year) ? 366 : 365;
  time -= days;

  days = 0;
  month = 0;
  monthLength = 0;

  for (month = 0; month < 12; month++) {
    if (month == 1) {
      if (LEAP_YEAR(year)) {
        monthLength = 29;
      } else {
        monthLength = 28;
      }
    } else {
      monthLength = monthDays[month];
    }

    if (time >= monthLength) {
      time -= monthLength;
    } else {
      break;
    }
  }
  tm.tm_mon = month + 1;
  tm.tm_mday = time + 1;
}

uint32_t getUnixTime()
{
  return static_cast<uint32_t>(sysTime);
}

int getSecOffset(const String& format) {
  int position_minus = format.indexOf('-');
  int position_plus = format.indexOf('+');

  if ((position_minus == -1) && (position_plus == -1)) {
    return 0;
  }
  int sign_position = _max(position_minus, position_plus);
  int position_percent = format.indexOf('%', sign_position);

  if (position_percent == -1) {
    return 0;
  }
  String valueStr = getNumerical(format.substring(sign_position, position_percent), true);

  if (!isInt(valueStr)) { return 0; }
  int value = valueStr.toInt();

  switch (format.charAt(position_percent - 1)) {
    case 'm':
    case 'M':
      return value * 60;
    case 'h':
    case 'H':
      return value * 3600;
  }
  return value;
}

String getSunriseTimeString(char delimiter) {
  return getTimeString(sunRise, delimiter, false, false);
}

String getSunsetTimeString(char delimiter) {
  return getTimeString(sunSet, delimiter, false, false);
}

String getSunriseTimeString(char delimiter, int secOffset) {
  if (secOffset == 0) {
    return getSunriseTimeString(delimiter);
  }
  return getTimeString(getSunRise(secOffset), delimiter, false, false);
}

String getSunsetTimeString(char delimiter, int secOffset) {
  if (secOffset == 0) {
    return getSunsetTimeString(delimiter);
  }
  return getTimeString(getSunSet(secOffset), delimiter, false, false);
}

unsigned long now() {

  bool timeSynced = false;
  const long msec_passed = timePassedSince(prevMillis);

  sysTime += static_cast<double>(msec_passed) / 1000.0;
  prevMillis += msec_passed;

  if (nextSyncTime <= sysTime) {

    double unixTime_d = -1.0;

    if (externalTimeSource > 0.0) {
      unixTime_d = externalTimeSource;
      externalTimeSource = -1.0;
    }

    if ((unixTime_d > 0.0) || getNtpTime(unixTime_d)) {
      prevMillis = millis();
      timeSynced = true;

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        double time_offset = sysTime - unixTime_d;
        String log = F("Time adjusted by ");
        log += String(time_offset * 1000.0);
        log += F(" msec. Wander: ");
        log += String((time_offset * 1000.0) / syncInterval);
        log += F(" msec/second");
        addLog(LOG_LEVEL_INFO, log)
      }
      sysTime = unixTime_d;


      applyTimeZone(unixTime_d);
      nextSyncTime = (uint32_t)unixTime_d + syncInterval;
    }
  }
  uint32_t localSystime = toLocal(sysTime);
  breakTime(localSystime, tm);

  if (timeSynced) {
    calcSunRiseAndSet();

    if (Settings.UseRules) {
      String event = statusNTPInitialized ? F("Time#Set") : F("Time#Initialized");
      rulesProcessing(event);
    }
    statusNTPInitialized = true;
  }
  return (unsigned long)localSystime;
}

int year(unsigned long t) {
  struct tm tmp;

  breakTime(t, tmp);
  return 1970 + tmp.tm_year;
}

int weekday(unsigned long t) {
  struct tm tmp;

  breakTime(t, tmp);
  return tmp.tm_wday;
}

int year()
{
  return 1970 + tm.tm_year;
}

byte month()
{
  return tm.tm_mon;
}

byte day()
{
  return tm.tm_mday;
}

byte hour()
{
  return tm.tm_hour;
}

byte minute()
{
  return tm.tm_min;
}

byte second()
{
  return tm.tm_sec;
}


int weekday()
{
  return tm.tm_wday;
}

String weekday_str()
{
  const int wday(weekday() - 1);
  const String weekDays = F("SunMonTueWedThuFriSat");

  return weekDays.substring(wday * 3, wday * 3 + 3);
}

void initTime()
{
  nextSyncTime = 0;
  now();
}

bool systemTimePresent() {
  return nextSyncTime > 0 || Settings.UseNTP;
}

void checkTime()
{
  now();

  if (tm.tm_min != PrevMinutes)
  {
    PluginCall(PLUGIN_CLOCK_IN, 0, dummyString);
    PrevMinutes = tm.tm_min;

    if (Settings.UseRules)
    {
      String event;
      event.reserve(21);
      event = F("Clock#Time=");
      event += weekday_str();
      event += ",";

      if (hour() < 10) {
        event += '0';
      }
      event += hour();
      event += ":";

      if (minute() < 10) {
        event += '0';
      }
      event += minute();
      rulesProcessing(event);
    }
  }
}

bool getNtpTime(double& unixTime_d)
{
  if (!Settings.UseNTP || !WiFiConnected(10)) {
    return false;
  }
  IPAddress timeServerIP;
  String log = F("NTP  : NTP host ");

  if (Settings.NTPHost[0] != 0) {
    resolveHostByName(Settings.NTPHost, timeServerIP);
    log += Settings.NTPHost;


    nextSyncTime = sysTime + 20;
  } else {

    String ntpServerName = String(random(0, 3));
    ntpServerName += F(".pool.ntp.org");
    resolveHostByName(ntpServerName.c_str(), timeServerIP);
    log += ntpServerName;


    nextSyncTime = sysTime + 5;
  }

  log += " (";
  log += timeServerIP.toString();
  log += ')';

  if (!hostReachable(timeServerIP)) {
    log += F(" unreachable");
    addLog(LOG_LEVEL_INFO, log);
    return false;
  }

  WiFiUDP udp;

  if (!beginWiFiUDP_randomPort(udp)) {
    return 0;
  }

  const int NTP_PACKET_SIZE = 48;
  byte packetBuffer[NTP_PACKET_SIZE];

  log += F(" queried");
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, log);
#endif

  while (udp.parsePacket() > 0) {
  }
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  if (udp.beginPacket(timeServerIP, 123) == 0) {
    udp.stop();
    return 0;
  }
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();


  uint32_t beginWait = millis();

  while (!timeOutReached(beginWait + 1000)) {
    int size = udp.parsePacket();
    int remotePort = udp.remotePort();

    if ((size >= NTP_PACKET_SIZE) && (remotePort == 123)) {
      udp.read(packetBuffer, NTP_PACKET_SIZE);





      unsigned long secsSince1900;



      secsSince1900 = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      uint32_t txTm = secsSince1900 - 2208988800UL;

      unsigned long txTm_f;
      txTm_f = (unsigned long)packetBuffer[44] << 24;
      txTm_f |= (unsigned long)packetBuffer[45] << 16;
      txTm_f |= (unsigned long)packetBuffer[46] << 8;
      txTm_f |= (unsigned long)packetBuffer[47];


      unixTime_d = static_cast<double>(txTm);


      unixTime_d += (static_cast<double>(txTm_f) / 4294967295.0);

      long total_delay = timePassedSince(beginWait);



      double delay_compensation = static_cast<double>(total_delay) / 2000.0;
      unixTime_d += delay_compensation;

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("NTP  : NTP replied: delay ");
        log += total_delay;
        log += F(" mSec");
        log += F(" Accuracy increased by ");
        double fractpart, intpart;
        fractpart = modf(unixTime_d, &intpart);

        if (fractpart < delay_compensation) {

          fractpart += 1.0;
        }
        log += String(fractpart, 3);
        log += F(" seconds");
        addLog(LOG_LEVEL_INFO, log);
      }
      udp.stop();

      return true;
    }
    delay(10);
  }
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, F("NTP  : No reply"));
#endif
  udp.stop();
  return false;
}
# 517 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/TimeESPeasy.ino"
long timeDiff(const unsigned long prev, const unsigned long next)
{
  long signed_diff = 0;


  const unsigned long half_max_unsigned_long = 2147483647u;

  if (next >= prev) {
    const unsigned long diff = next - prev;

    if (diff <= half_max_unsigned_long) {


      signed_diff = static_cast<long>(diff);
    } else {

      signed_diff = static_cast<long>((ULONG_MAX - next) + prev + 1u);
      signed_diff = -1 * signed_diff;
    }
  } else {

    const unsigned long diff = prev - next;

    if (diff <= half_max_unsigned_long) {

      signed_diff = static_cast<long>(diff);
      signed_diff = -1 * signed_diff;
    } else {

      signed_diff = static_cast<long>((ULONG_MAX - prev) + next + 1u);
    }
  }
  return signed_diff;
}



long timePassedSince(unsigned long timestamp) {
  return timeDiff(timestamp, millis());
}

long usecPassedSince(unsigned long timestamp) {
  return timeDiff(timestamp, micros());
}


boolean timeOutReached(unsigned long timer) {
  const long passed = timePassedSince(timer);

  return passed >= 0;
}

boolean usecTimeOutReached(unsigned long timer) {
  const long passed = usecPassedSince(timer);

  return passed >= 0;
}

void setNextTimeInterval(unsigned long& timer, const unsigned long step) {
  timer += step;
  const long passed = timePassedSince(timer);

  if (passed < 0) {

    return;
  }

  if (static_cast<unsigned long>(passed) > step) {

    timer = millis() + step;
    return;
  }


  timer = millis() + (step - passed);
}




String timeLong2String(unsigned long lngTime)
{
  unsigned long x = 0;
  String time = "";

  x = (lngTime >> 16) & 0xf;

  if (x == 0x0f) {
    x = 0;
  }
  String weekDays = F("AllSunMonTueWedThuFriSatWrkWkd");
  time = weekDays.substring(x * 3, x * 3 + 3);
  time += ",";

  x = (lngTime >> 12) & 0xf;

  if (x == 0xf) {
    time += "*";
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  x = (lngTime >> 8) & 0xf;

  if (x == 0xf) {
    time += "*";
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  time += ":";

  x = (lngTime >> 4) & 0xf;

  if (x == 0xf) {
    time += "*";
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  x = (lngTime) & 0xf;

  if (x == 0xf) {
    time += "*";
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  return time;
}



String getDateString(const struct tm& ts, char delimiter) {
  char DateString[20];
  const int year = 1970 + ts.tm_year;

  sprintf_P(DateString, PSTR("%4d%c%02d%c%02d"), year, delimiter, ts.tm_mon, delimiter, ts.tm_mday);
  return DateString;
}

String getDateString(char delimiter)
{
  return getDateString(tm, delimiter);
}

String getDateString(const struct tm& ts)
{
  return getDateString(tm, ':');
}



String getDateString()
{
  return getDateString('\0');
}



String getTimeString(const struct tm& ts, char delimiter, bool am_pm, bool show_seconds)
{
  char TimeString[20];

  if (am_pm) {
    uint8_t hour(ts.tm_hour % 12);

    if (hour == 0) { hour = 12; }
    const char a_or_p = ts.tm_hour < 12 ? 'A' : 'P';

    if (show_seconds) {
      sprintf_P(TimeString, PSTR("%d%c%02d%c%02d %cM"),
                hour, delimiter, ts.tm_min, delimiter, ts.tm_sec, a_or_p);
    } else {
      sprintf_P(TimeString, PSTR("%d%c%02d %cM"),
                hour, delimiter, ts.tm_min, a_or_p);
    }
  } else {
    if (show_seconds) {
      sprintf_P(TimeString, PSTR("%02d%c%02d%c%02d"),
                ts.tm_hour, delimiter, ts.tm_min, delimiter, ts.tm_sec);
    } else {
      sprintf_P(TimeString, PSTR("%d%c%02d"),
                ts.tm_hour, delimiter, ts.tm_min);
    }
  }
  return TimeString;
}

String getTimeString(char delimiter, bool show_seconds )
{
  return getTimeString(tm, delimiter, false, show_seconds);
}

String getTimeString_ampm(char delimiter, bool show_seconds )
{
  return getTimeString(tm, delimiter, true, show_seconds);
}



String getTimeString()
{
  return getTimeString('\0');
}

String getTimeString_ampm()
{
  return getTimeString_ampm('\0');
}




String getDateTimeString(const struct tm& ts, char dateDelimiter, char timeDelimiter, char dateTimeDelimiter, bool am_pm)
{
  String ret = getDateString(ts, dateDelimiter);

  if (dateTimeDelimiter != '\0') {
    ret += dateTimeDelimiter;
  }
  ret += getTimeString(ts, timeDelimiter, am_pm, true);
  return ret;
}

String getDateTimeString(const struct tm& ts)
{
  return getDateTimeString(ts, '-', ':', ' ', false);
}

String getDateTimeString(char dateDelimiter, char timeDelimiter, char dateTimeDelimiter) {
  return getDateTimeString(tm, dateDelimiter, timeDelimiter, dateTimeDelimiter, false);
}

String getDateTimeString_ampm(char dateDelimiter, char timeDelimiter, char dateTimeDelimiter) {
  return getDateTimeString(tm, dateDelimiter, timeDelimiter, dateTimeDelimiter, true);
}




unsigned long string2TimeLong(const String& str)
{



  char command[20];
  int w, x, y;
  unsigned long a;
  {

    String tmpString(str);
    tmpString.toLowerCase();
    tmpString.toCharArray(command, 20);
  }
  unsigned long lngTime = 0;
  String TmpStr1;

  if (GetArgv(command, TmpStr1, 1))
  {
    String day = TmpStr1;
    String weekDays = F("allsunmontuewedthufrisatwrkwkd");
    y = weekDays.indexOf(TmpStr1) / 3;

    if (y == 0) {
      y = 0xf;
    }
    lngTime |= (unsigned long)y << 16;
  }

  if (GetArgv(command, TmpStr1, 2))
  {
    y = 0;

    for (x = TmpStr1.length() - 1; x >= 0; x--)
    {
      w = TmpStr1[x];

      if (((w >= '0') && (w <= '9')) || (w == '*'))
      {
        a = 0xffffffff ^ (0xfUL << y);
        lngTime &= a;

        if (w == '*') {
          lngTime |= (0xFUL << y);
        }
        else {
          lngTime |= (w - '0') << y;
        }
        y += 4;
      }
      else
      if (w == ':') {}
      else
      {
        break;
      }
    }
  }
  #undef TmpStr1Length
  return lngTime;
}




boolean matchClockEvent(unsigned long clockEvent, unsigned long clockSet)
{
  unsigned long Mask;

  for (byte y = 0; y < 8; y++)
  {
    if (((clockSet >> (y * 4)) & 0xf) == 0xf)
    {
      Mask = 0xffffffff ^ (0xFUL << (y * 4));
      clockEvent &= Mask;
      clockEvent |= (0xFUL << (y * 4));
    }
  }

  if (((clockSet >> (16)) & 0xf) == 0x8) {
    if (weekday() >= 2 and weekday() <= 6)
    {
      Mask = 0xffffffff ^ (0xFUL << (16));
      clockEvent &= Mask;
      clockEvent |= (0x8UL << (16));
    }
  }

  if (((clockSet >> (16)) & 0xf) == 0x9) {
    if (weekday() == 1 or weekday() == 7)
    {
      Mask = 0xffffffff ^ (0xFUL << (16));
      clockEvent &= Mask;
      clockEvent |= (0x9UL << (16));
    }
  }

  if (clockEvent == clockSet) {
    return true;
  }
  return false;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/TimeZoneESPeasy.ino"
#include <time.h>







TimeChangeRule m_dst;
TimeChangeRule m_std;
uint32_t m_dstUTC = 0;
uint32_t m_stdUTC = 0;
uint32_t m_dstLoc = 0;
uint32_t m_stdLoc = 0;
# 60 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/TimeZoneESPeasy.ino"
void getDefaultDst_flash_values(uint16_t& start, uint16_t& end) {


  TimeChangeRule CEST(Last, Sun, Mar, 2, Settings.TimeZone);
  TimeChangeRule CET(Last, Sun, Oct, 3, Settings.TimeZone);

  start = CEST.toFlashStoredValue();
  end = CET.toFlashStoredValue();
}

void applyTimeZone(uint32_t curTime) {
  int dst_offset = Settings.DST ? 60 : 0;
  uint16_t tmpStart(Settings.DST_Start);
  uint16_t tmpEnd(Settings.DST_End);

  for (int i = 0; i < 2; ++i) {
    TimeChangeRule start(tmpStart, Settings.TimeZone + dst_offset);
    TimeChangeRule end(tmpEnd, Settings.TimeZone);

    if (start.isValid() && end.isValid()) {
      setTimeZone(start, end, curTime);
      return;
    }
    getDefaultDst_flash_values(tmpStart, tmpEnd);
  }
}

void setTimeZone(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart, uint32_t curTime) {
  m_dst = dstStart;
  m_std = stdStart;

  if (calcTimeChanges(year(curTime))) {
    logTimeZoneInfo();
  }
}

void logTimeZoneInfo() {
  String log = F("Current Time Zone: ");

  if (m_std.offset != m_dst.offset) {

    log += F(" DST time start: ");

    if (m_dstLoc != 0) {
      struct tm tmp;
      breakTime(m_dstLoc, tmp);
      log += getDateTimeString(tmp, '-', ':', ' ', false);
    }
    log += F(" offset: ");
    log += m_dst.offset;
    log += F(" min");
  }


  log += F("STD time start: ");

  if (m_stdLoc != 0) {
    struct tm tmp;
    breakTime(m_stdLoc, tmp);
    log += getDateTimeString(tmp, '-', ':', ' ', false);
  }
  log += F(" offset: ");
  log += m_std.offset;
  log += F(" min");
  addLog(LOG_LEVEL_INFO, log);
}

uint32_t makeTime(const struct tm& tm) {



  const uint8_t monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  int i;
  uint32_t seconds;


  seconds = tm.tm_year * (SECS_PER_DAY * 365);

  for (i = 0; i < tm.tm_year; i++) {
    if (LEAP_YEAR(i)) {
      seconds += SECS_PER_DAY;
    }
  }


  for (i = 1; i < tm.tm_mon; i++) {
    if ((i == 2) && LEAP_YEAR(tm.tm_year)) {
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * monthDays[i - 1];
    }
  }
  seconds += (tm.tm_mday - 1) * SECS_PER_DAY;
  seconds += tm.tm_hour * SECS_PER_HOUR;
  seconds += tm.tm_min * SECS_PER_MIN;
  seconds += tm.tm_sec;
  return (uint32_t)seconds;
}





uint32_t calcTimeChangeForRule(const TimeChangeRule& r, int yr)
{
  uint8_t m = r.month;
  uint8_t w = r.week;

  if (w == 0)
  {
    if (++m > 12)
    {
      m = 1;
      ++yr;
    }
    w = 1;
  }


  struct tm tm;
  tm.tm_hour = r.hour;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  tm.tm_mday = 1;
  tm.tm_mon = m;
  tm.tm_year = yr - 1970;
  uint32_t t = makeTime(tm);


  t += ((r.dow - weekday(t) + 7) % 7 + (w - 1) * 7) * SECS_PER_DAY;


  if (r.week == 0) { t -= 7 * SECS_PER_DAY; }
  return t;
}





bool calcTimeChanges(int yr)
{
  uint32_t dstLoc = calcTimeChangeForRule(m_dst, yr);
  uint32_t stdLoc = calcTimeChangeForRule(m_std, yr);
  bool changed = (m_dstLoc != dstLoc) || (m_stdLoc != stdLoc);

  m_dstLoc = dstLoc;
  m_stdLoc = stdLoc;
  m_dstUTC = m_dstLoc - m_std.offset * SECS_PER_MIN;
  m_stdUTC = m_stdLoc - m_dst.offset * SECS_PER_MIN;
  return changed;
}





uint32_t toLocal(uint32_t utc)
{

  if (year(utc) != year(m_dstUTC)) { calcTimeChanges(year(utc)); }

  if (utcIsDST(utc)) {
    return utc + m_dst.offset * SECS_PER_MIN;
  }
  else {
    return utc + m_std.offset * SECS_PER_MIN;
  }
}





bool utcIsDST(uint32_t utc)
{

  if (year(utc) != year(m_dstUTC)) { calcTimeChanges(year(utc)); }

  if (m_stdUTC == m_dstUTC) {
    return false;
  }
  else if (m_stdUTC > m_dstUTC) {
    return utc >= m_dstUTC && utc < m_stdUTC;
  }
  else {
    return !(utc >= m_stdUTC && utc < m_dstUTC);
  }
}





bool locIsDST(uint32_t local)
{

  if (year(local) != year(m_dstLoc)) { calcTimeChanges(year(local)); }

  if (m_stdUTC == m_dstUTC) {
    return false;
  }
  else if (m_stdLoc > m_dstLoc) {
    return local >= m_dstLoc && local < m_stdLoc;
  }
  else {
    return !(local >= m_stdLoc && local < m_dstLoc);
  }
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"



#define ALL_ALLOWED 0
#define LOCAL_SUBNET_ALLOWED 1
#define ONLY_IP_RANGE_ALLOWED 2
#define _HEAD false
#define _TAIL true
#define CHUNKED_BUFFER_SIZE 400

void sendContentBlocking(String& data);
void sendHeaderBlocking(bool json, const String& origin = "");

class StreamingBuffer {
private:
  bool lowMemorySkip;

public:
  uint32_t initialRam;
  uint32_t beforeTXRam;
  uint32_t duringTXRam;
  uint32_t finalRam;
  uint32_t maxCoreUsage;
  uint32_t maxServerUsage;
  unsigned int sentBytes;
  uint32_t flashStringCalls;
  uint32_t flashStringData;
  String buf;

  StreamingBuffer(void) : lowMemorySkip(false),
    initialRam(0), beforeTXRam(0), duringTXRam(0), finalRam(0), maxCoreUsage(0),
    maxServerUsage(0), sentBytes(0), flashStringCalls(0), flashStringData(0)
  {
    buf.reserve(CHUNKED_BUFFER_SIZE + 50);
    buf = "";
  }
  StreamingBuffer operator= (String& a) { flush(); return addString(a); }
  StreamingBuffer operator= (const String& a) { flush(); return addString(a); }
  StreamingBuffer operator+= (char a) { return addString(String(a)); }
  StreamingBuffer operator+= (long unsigned int a) { return addString(String(a)); }
  StreamingBuffer operator+= (float a) { return addString(String(a)); }
  StreamingBuffer operator+= (int a) { return addString(String(a)); }
  StreamingBuffer operator+= (uint32_t a) { return addString(String(a)); }
  StreamingBuffer operator+= (const String& a) { return addString(a); }

  StreamingBuffer operator+= (PGM_P str) {
    ++flashStringCalls;
    if (!str) return *this;
    if (lowMemorySkip) return *this;
    int flush_step = CHUNKED_BUFFER_SIZE - this->buf.length();
    if (flush_step < 1) flush_step = 0;
    unsigned int pos = 0;
    const unsigned int length = strlen_P((PGM_P)str);
    if (length == 0) return *this;
    flashStringData += length;
    while (pos < length) {
      if (flush_step == 0) {
        sendContentBlocking(this->buf);
        flush_step = CHUNKED_BUFFER_SIZE;
      }
      this->buf += (char)pgm_read_byte(&str[pos]);
      ++pos;
      --flush_step;
    }
    checkFull();
    return *this;
  }


  StreamingBuffer addString(const String& a) {
    if (lowMemorySkip) return *this;
    int flush_step = CHUNKED_BUFFER_SIZE - this->buf.length();
    if (flush_step < 1) flush_step = 0;
    int pos = 0;
    const int length = a.length();
    while (pos < length) {
      if (flush_step == 0) {
        sendContentBlocking(this->buf);
        flush_step = CHUNKED_BUFFER_SIZE;
      }
      this->buf += a[pos];
      ++pos;
      --flush_step;
    }
    checkFull();
    return *this;
  }

  void flush() {
    if (lowMemorySkip) {
      this->buf = "";
    } else {
      sendContentBlocking(this->buf);
    }
  }

  void checkFull(void) {
    if (lowMemorySkip) this->buf = "";
    if (this->buf.length() > CHUNKED_BUFFER_SIZE) {
      trackTotalMem();
      sendContentBlocking(this->buf);
    }
  }

  void startStream() {
    startStream(false, "");
  }

  void startStream(const String& origin) {
    startStream(false, origin);
  }

  void startJsonStream() {
    startStream(true, "*");
  }

private:
  void startStream(bool json, const String& origin) {
    maxCoreUsage = maxServerUsage = 0;
    initialRam = ESP.getFreeHeap();
    beforeTXRam = initialRam;
    sentBytes = 0;
    buf = "";
    if (beforeTXRam < 3000) {
      lowMemorySkip = true;
      WebServer.send(200, "text/plain", "Low memory. Cannot display webpage :-(");
       #if defined(ESP8266)
         tcpCleanup();
       #endif
      return;
    } else
      sendHeaderBlocking(json, origin);
  }

  void trackTotalMem() {
    beforeTXRam = ESP.getFreeHeap();
    if ((initialRam - beforeTXRam) > maxServerUsage)
      maxServerUsage = initialRam - beforeTXRam;
  }

public:

  void trackCoreMem() {
    duringTXRam = ESP.getFreeHeap();
    if ((initialRam - duringTXRam) > maxCoreUsage)
      maxCoreUsage = (initialRam - duringTXRam);
  }

  void endStream(void) {
    if (!lowMemorySkip) {
      if (buf.length() > 0) sendContentBlocking(buf);
      buf = "";
      sendContentBlocking(buf);
      finalRam = ESP.getFreeHeap();
# 164 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
    } else {
      addLog(LOG_LEVEL_ERROR, String("Webpage skipped: low memory: ") + finalRam);
      lowMemorySkip = false;
    }
  }
} TXBuffer;

void sendContentBlocking(String& data) {
  checkRAM(F("sendContentBlocking"));
  uint32_t freeBeforeSend = ESP.getFreeHeap();
  const uint32_t length = data.length();
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_DEV, String("sendcontent free: ") + freeBeforeSend + " chunk size:" + length);
#endif
  freeBeforeSend = ESP.getFreeHeap();
  if (TXBuffer.beforeTXRam > freeBeforeSend)
    TXBuffer.beforeTXRam = freeBeforeSend;
  TXBuffer.duringTXRam = freeBeforeSend;
#if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  String size = formatToHex(length) + "\r\n";

  WebServer.sendContent(size);
  if (length > 0) WebServer.sendContent(data);
  WebServer.sendContent("\r\n");
#else
  unsigned int timeout = 0;
  if (freeBeforeSend < 5000) timeout = 100;
  if (freeBeforeSend < 4000) timeout = 1000;
  const uint32_t beginWait = millis();
  WebServer.sendContent(data);
  while ((ESP.getFreeHeap() < freeBeforeSend) &&
         !timeOutReached(beginWait + timeout)) {
    if (ESP.getFreeHeap() < TXBuffer.duringTXRam)
      TXBuffer.duringTXRam = ESP.getFreeHeap();
    ;
    TXBuffer.trackCoreMem();
    checkRAM(F("duringDataTX"));
    delay(1);
  }
#endif

  TXBuffer.sentBytes += length;
  data = "";
  delay(0);
}

void sendHeaderBlocking(bool json, const String& origin) {
  checkRAM(F("sendHeaderBlocking"));
  WebServer.client().flush();
  String contenttype;
  if (json)
    contenttype = F("application/json");
  else
    contenttype = F("text/html");

#if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  WebServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  WebServer.sendHeader(F("Accept-Ranges"), F("none"));
  WebServer.sendHeader(F("Cache-Control"), F("no-cache"));
  WebServer.sendHeader(F("Transfer-Encoding"), F("chunked"));
  if (json)
    WebServer.sendHeader(F("Access-Control-Allow-Origin"),"*");
  WebServer.send(200, contenttype, "");
#else
  unsigned int timeout = 0;
  uint32_t freeBeforeSend = ESP.getFreeHeap();
  if (freeBeforeSend < 5000) timeout = 100;
  if (freeBeforeSend < 4000) timeout = 1000;
  const uint32_t beginWait = millis();
  WebServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  WebServer.sendHeader(F("Cache-Control"), F("no-cache"));
  if (origin.length() > 0) {
    WebServer.sendHeader(F("Access-Control-Allow-Origin"), origin);
  }
  WebServer.send(200, contenttype, "");

  while ((ESP.getFreeHeap() < freeBeforeSend) &&
         !timeOutReached(beginWait + timeout)) {
    checkRAM(F("duringHeaderTX"));
    delay(1);
  }
#endif
  delay(0);
}

void sendHeadandTail(const String& tmplName, boolean Tail = false, boolean rebooting = false) {
  String pageTemplate = "";
  String fileName = tmplName;
  fileName += F(".htm");
  fs::File f = tryOpenFile(fileName, "r");

  if (f) {
    pageTemplate.reserve(f.size());
    while (f.available()) pageTemplate += (char)f.read();
    f.close();
  } else {

    getWebPageTemplateDefault(tmplName, pageTemplate);
  }
  checkRAM(F("sendWebPage"));

  lastWeb = millis();

  if (Tail) {
    TXBuffer += pageTemplate.substring(
        11 +
        pageTemplate.indexOf(F("{{content}}")));
  } else {
    int indexStart = 0;
    int indexEnd = 0;
    int readPos = 0;
    String varName;
    String meta;
    if(rebooting){
      meta = F("<meta http-equiv='refresh' content='10 url=/'>");
    }
    while ((indexStart = pageTemplate.indexOf(F("{{"), indexStart)) >= 0) {
      TXBuffer += pageTemplate.substring(readPos, indexStart);
      readPos = indexStart;
      if ((indexEnd = pageTemplate.indexOf(F("}}"), indexStart)) > 0) {
        varName = pageTemplate.substring(indexStart + 2, indexEnd);
        indexStart = indexEnd + 2;
        readPos = indexEnd + 2;
        varName.toLowerCase();

        if (varName == F("content")) {
          break;
        } else if (varName == F("error")) {
          getErrorNotifications();
        }
        else if(varName == F("meta")) {
          TXBuffer += meta;
        }
        else {
          getWebPageTemplateVar(varName);
        }
      } else {

        readPos += 2;
        indexStart += 2;
      }
    }
  }
  if (shouldReboot) {

    html_add_script(false);
    TXBuffer += DATA_REBOOT_JS;
    html_add_script_end();
  }
}

void sendHeadandTail_stdtemplate(boolean Tail = false, boolean rebooting = false) {
  sendHeadandTail(F("TmplStd"), Tail, rebooting);
}

boolean ipLessEqual(const IPAddress& ip, const IPAddress& high)
{
  for (byte i = 0; i < 4; ++i) {
    if (ip[i] > high[i]) return false;
  }
  return true;
}

boolean ipInRange(const IPAddress& ip, const IPAddress& low, const IPAddress& high)
{
  return (ipLessEqual(low, ip) && ipLessEqual(ip, high));
}

String describeAllowedIPrange() {
  String reply;
  switch (SecuritySettings.IPblockLevel) {
    case ALL_ALLOWED:
     reply += F("All Allowed");
      break;
    default:
    {
      IPAddress low, high;
      getIPallowedRange(low, high);
      reply += formatIP(low);
      reply += F(" - ");
      reply += formatIP(high);
    }
  }
  return reply;
}

bool getIPallowedRange(IPAddress& low, IPAddress& high)
{
  switch (SecuritySettings.IPblockLevel) {
    case LOCAL_SUBNET_ALLOWED:
      return getSubnetRange(low, high);
    case ONLY_IP_RANGE_ALLOWED:
      low = SecuritySettings.AllowedIPrangeLow;
      high = SecuritySettings.AllowedIPrangeHigh;
      break;
    default:
      low = IPAddress(0,0,0,0);
      high = IPAddress(255,255,255,255);
      return false;
  }
  return true;
}

boolean clientIPallowed()
{

  IPAddress low, high;
  if (!getIPallowedRange(low, high))
  {

    return true;
  }
  WiFiClient client(WebServer.client());
  if (ipInRange(client.remoteIP(), low, high))
    return true;

  if (WifiIsAP(WiFi.getMode())) {

    return true;
  }
  String response = F("IP blocked: ");
  response += formatIP(client.remoteIP());
  WebServer.send(403, F("text/html"), response);
  if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
    response += F(" Allowed: ");
    response += formatIP(low);
    response += F(" - ");
    response += formatIP(high);
    addLog(LOG_LEVEL_ERROR, response);
  }
  return false;
}

void clearAccessBlock()
{
  SecuritySettings.IPblockLevel = ALL_ALLOWED;
}





#define HTML_SYMBOL_WARNING "&#9888;"
#define HTML_SYMBOL_INPUT "&#8656;"
#define HTML_SYMBOL_OUTPUT "&#8658;"
#define HTML_SYMBOL_I_O "&#8660;"


#if defined(ESP8266)
  #define TASKS_PER_PAGE 12
#endif
#if defined(ESP32)
  #define TASKS_PER_PAGE 32
#endif

int getFormItemInt(const String &key, int defaultValue) {
  int value = defaultValue;
  getCheckWebserverArg_int(key, value);
  return value;
}

bool getCheckWebserverArg_int(const String &key, int& value) {
  String valueStr = WebServer.arg(key);
  if (!isInt(valueStr)) return false;
  value = valueStr.toInt();
  return true;
}

#define strncpy_webserver_arg(D,N) safe_strncpy(D, WebServer.arg(N).c_str(), sizeof(D));
#define update_whenset_FormItemInt(K,V) { int tmpVal; if (getCheckWebserverArg_int(K, tmpVal)) V=tmpVal;}


bool isFormItemChecked(const String& id)
{
  return WebServer.arg(id) == F("on");
}

int getFormItemInt(const String& id)
{
  return getFormItemInt(id, 0);
}

float getFormItemFloat(const String& id)
{
  String val = WebServer.arg(id);
  if (!isFloat(val)) return 0.0;
  return val.toFloat();
}

bool isFormItem(const String& id)
{
  return (WebServer.arg(id).length() != 0);
}




void addHtmlError(const String& error){
  if (error.length()>0)
  {
    TXBuffer += F("<div class=\"");
    if (error.startsWith(F("Warn"))) {
      TXBuffer += F("warning");
    } else {
      TXBuffer += F("alert");
    }
    TXBuffer += F("\"><span class=\"closebtn\" onclick=\"this.parentElement.style.display='none';\">&times;</span>");
    TXBuffer += error;
    TXBuffer += F("</div>");
  }
  else
  {
    TXBuffer += jsToastMessageBegin;

    TXBuffer += F("Submitted");
    TXBuffer += jsToastMessageEnd;
  }
}

void addHtml(const String& html) {
  TXBuffer += html;
}

void addDisabled() {
  TXBuffer += F(" disabled");
}

void WebServerInit()
{

  WebServer.on("/", handle_root);
  WebServer.on(F("/advanced"), handle_advanced);
  WebServer.on(F("/config"), handle_config);
  WebServer.on(F("/control"), handle_control);
  WebServer.on(F("/controllers"), handle_controllers);
  WebServer.on(F("/devices"), handle_devices);
  WebServer.on(F("/download"), handle_download);
  WebServer.on(F("/dumpcache"), handle_dumpcache);
  WebServer.on(F("/cache_json"), handle_cache_json);
  WebServer.on(F("/cache_csv"), handle_cache_csv);
  WebServer.on(F("/factoryreset"), handle_factoryreset);
  WebServer.on(F("/favicon.ico"), handle_favicon);
  WebServer.on(F("/filelist"), handle_filelist);
  WebServer.on(F("/hardware"), handle_hardware);
  WebServer.on(F("/i2cscanner"), handle_i2cscanner);
  WebServer.on(F("/json"), handle_json);
  WebServer.on(F("/log"), handle_log);
  WebServer.on(F("/login"), handle_login);
  WebServer.on(F("/logjson"), handle_log_JSON);
#ifndef NOTIFIER_SET_NONE
  WebServer.on(F("/notifications"), handle_notifications);
#endif
  WebServer.on(F("/pinstates"), handle_pinstates);
  WebServer.on(F("/rules"), handle_rules_new);
  WebServer.on(F("/rules/"), Goto_Rules_Root);
  WebServer.on(F("/rules/add"), []()
  {
    handle_rules_edit(WebServer.uri(),true);
  });
  WebServer.on(F("/rules/backup"), handle_rules_backup);
  WebServer.on(F("/rules/delete"), handle_rules_delete);
#ifdef FEATURE_SD
  WebServer.on(F("/SDfilelist"), handle_SDfilelist);
#endif
  WebServer.on(F("/setup"), handle_setup);
  WebServer.on(F("/sysinfo"), handle_sysinfo);
#ifdef WEBSERVER_SYSVARS
  WebServer.on(F("/sysvars"), handle_sysvars);
#endif
#ifdef WEBSERVER_TIMINGSTATS
  WebServer.on(F("/timingstats"), handle_timingstats);
#endif
  WebServer.on(F("/tools"), handle_tools);
  WebServer.on(F("/upload"), HTTP_GET, handle_upload);
  WebServer.on(F("/upload"), HTTP_POST, handle_upload_post, handleFileUpload);
  WebServer.on(F("/wifiscanner"), handle_wifiscanner);

#ifdef WEBSERVER_NEW_UI
  WebServer.on(F("/factoryreset_json"), handle_factoryreset_json);
  WebServer.on(F("/filelist_json"), handle_filelist_json);
  WebServer.on(F("/i2cscanner_json"), handle_i2cscanner_json);
  WebServer.on(F("/node_list_json"), handle_nodes_list_json);
  WebServer.on(F("/pinstates_json"), handle_pinstates_json);
  WebServer.on(F("/sysinfo_json"), handle_sysinfo_json);
  WebServer.on(F("/timingstats_json"), handle_timingstats_json);
  WebServer.on(F("/upload_json"), HTTP_POST, handle_upload_json, handleFileUpload);
  WebServer.on(F("/wifiscanner_json"), handle_wifiscanner_json);
#endif

  WebServer.onNotFound(handleNotFound);

  #if defined(ESP8266)
  {
    uint32_t maxSketchSize;
    bool use2step;
    if (OTA_possible(maxSketchSize, use2step)) {
      httpUpdater.setup(&WebServer);
    }
  }
  #endif

  #if defined(ESP8266)
  if (Settings.UseSSDP)
  {
    WebServer.on(F("/ssdp.xml"), HTTP_GET, []() {
      WiFiClient client(WebServer.client());
      client.setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
      SSDP_schema(client);
    });
    SSDP_begin();
  }
  #endif
}

void setWebserverRunning(bool state) {
  if (webserver_state == state)
    return;
  if (state) {
    if (!webserver_init) {
      WebServerInit();
      webserver_init = true;
    }
    WebServer.begin();
    addLog(LOG_LEVEL_INFO, F("Webserver: start"));
  } else {
    WebServer.stop();
    addLog(LOG_LEVEL_INFO, F("Webserver: stop"));
  }
  webserver_state = state;
}


void getWebPageTemplateDefault(const String& tmplName, String& tmpl)
{
  tmpl.reserve(576);
  const bool addJS = true;
  const bool addMeta = true;
  if (tmplName == F("TmplAP"))
  {
    getWebPageTemplateDefaultHead(tmpl, !addMeta, !addJS);
    tmpl += F("<body>"
              "<header class='apheader'>"
              "<h1>Welcome to ESP Easy Mega AP</h1>"
              "</header>");
    getWebPageTemplateDefaultContentSection(tmpl);
    getWebPageTemplateDefaultFooter(tmpl);
  }
  else if (tmplName == F("TmplMsg"))
  {
    getWebPageTemplateDefaultHead(tmpl, !addMeta, !addJS);
    tmpl += F("<body>");
    getWebPageTemplateDefaultHeader(tmpl, F("{{name}}"), false);
    getWebPageTemplateDefaultContentSection(tmpl);
    getWebPageTemplateDefaultFooter(tmpl);
  }
  else if (tmplName == F("TmplDsh"))
  {
    getWebPageTemplateDefaultHead(tmpl, !addMeta, addJS);
    tmpl += F(
        "<body>"
        "{{content}}"
        "</body></html>"
            );
  }
  else
  {
    getWebPageTemplateDefaultHead(tmpl, addMeta, addJS);
    tmpl += F("<body class='bodymenu'>"
              "<span class='message' id='rbtmsg'></span>");
    getWebPageTemplateDefaultHeader(tmpl, F("{{name}} {{logo}}"), true);
    getWebPageTemplateDefaultContentSection(tmpl);
    getWebPageTemplateDefaultFooter(tmpl);
  }
}

void getWebPageTemplateDefaultHead(String& tmpl, bool addMeta, bool addJS) {
  tmpl += F("<!DOCTYPE html><html lang='en'>"
              "<head>"
                "<meta charset='utf-8'/>"
                "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "<title>{{name}}</title>");
  if (addMeta) tmpl += F("{{meta}}");
  if (addJS) tmpl += F("{{js}}");

  tmpl += F("{{css}}"
            "</head>");
}

void getWebPageTemplateDefaultHeader(String& tmpl, const String& title, bool addMenu) {
  tmpl += F("<header class='headermenu'>"
            "<h1>ESP Easy Mega: ");
  tmpl += title;
  tmpl += F("</h1><BR>");
  if (addMenu) tmpl += F("{{menu}}");
  tmpl += F("</header>");
}

void getWebPageTemplateDefaultContentSection(String& tmpl) {
  tmpl += F("<section>"
              "<span class='message error'>"
                "{{error}}"
              "</span>"
              "{{content}}"
            "</section>"
          );
}

void getWebPageTemplateDefaultFooter(String& tmpl) {
  tmpl += F("<footer>"
              "<br>"
              "<h6>Powered by <a href='http://www.letscontrolit.com' style='font-size: 15px; text-decoration: none'>Let's Control It</a> community</h6>"
            "</footer>"
            "</body></html>"
          );
}

void getErrorNotifications() {

  int nrMQTTenabled = 0;
  for (byte x = 0; x < CONTROLLER_MAX; x++) {
    if (Settings.Protocol[x] != 0) {
      byte ProtocolIndex = getProtocolIndex(Settings.Protocol[x]);
      if (Settings.ControllerEnabled[x] && Protocol[ProtocolIndex].usesMQTT) {
        ++nrMQTTenabled;
      }
    }
  }
  if (nrMQTTenabled > 1) {

    addHtmlError(F("Only one MQTT controller should be active."));
  }

}


#define MENU_INDEX_MAIN 0
#define MENU_INDEX_CONFIG 1
#define MENU_INDEX_CONTROLLERS 2
#define MENU_INDEX_HARDWARE 3
#define MENU_INDEX_DEVICES 4
#define MENU_INDEX_RULES 5
#define MENU_INDEX_NOTIFICATIONS 6
#define MENU_INDEX_TOOLS 7
static byte navMenuIndex = MENU_INDEX_MAIN;


void getWebPageTemplateVar(const String& varName )
{



  if (varName == F("name"))
  {
    TXBuffer += Settings.Name;
  }

  else if (varName == F("unit"))
  {
    TXBuffer += String(Settings.Unit);
  }

  else if (varName == F("menu"))
  {
    static const __FlashStringHelper* gpMenu[8][3] = {


      F("&#8962;"), F("Main"), F("/"),
      F("&#9881;"), F("Config"), F("/config"),
      F("&#128172;"), F("Controllers"), F("/controllers"),
      F("&#128204;"), F("Hardware"), F("/hardware"),
      F("&#128268;"), F("Devices"), F("/devices"),
      F("&#10740;"), F("Rules"), F("/rules"),
      F("&#9993;"), F("Notifications"), F("/notifications"),
      F("&#128295;"), F("Tools"), F("/tools"),
    };

    TXBuffer += F("<div class='menubar'>");

    for (byte i = 0; i < 8; i++)
    {
      if (i == MENU_INDEX_RULES && !Settings.UseRules)
        continue;
#ifdef NOTIFIER_SET_NONE
      if (i == MENU_INDEX_NOTIFICATIONS)
        continue;
#endif

      TXBuffer += F("<a class='menu");
      if (i == navMenuIndex)
        TXBuffer += F(" active");
      TXBuffer += F("' href='");
      TXBuffer += gpMenu[i][2];
      TXBuffer += "'>";
      TXBuffer += gpMenu[i][0];
      TXBuffer += F("<span class='showmenulabel'>");
      TXBuffer += gpMenu[i][1];
      TXBuffer += F("</span>");
      TXBuffer += F("</a>");
    }

    TXBuffer += F("</div>");
  }

  else if (varName == F("logo"))
  {
    if (SPIFFS.exists(F("esp.png")))
    {
      TXBuffer = F("<img src=\"esp.png\" width=48 height=48 align=right>");
    }
  }

  else if (varName == F("css"))
  {
    if (SPIFFS.exists(F("esp.css")))

    {
      TXBuffer = F("<link rel=\"stylesheet\" type=\"text/css\" href=\"esp.css\">");
    }
   else
    {
      TXBuffer += F("<style>");

      TXBuffer += DATA_ESPEASY_DEFAULT_MIN_CSS;
      TXBuffer += F("</style>");
    }
  }


  else if (varName == F("js"))
  {
    html_add_autosubmit_form();
  }

  else if (varName == F("error"))
  {

  }

  else if (varName == F("debug"))
  {

  }

  else
  {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("Templ: Unknown Var : ");
      log += varName;
      addLog(LOG_LEVEL_ERROR, log);
    }

  }

 }


void writeDefaultCSS(void)
{
  return;

  if (!SPIFFS.exists(F("esp.css")))
  {
    String defaultCSS;

    fs::File f = tryOpenFile(F("esp.css"), "w");
    if (f)
    {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("CSS  : Writing default CSS file to SPIFFS (");
        log += defaultCSS.length();
        log += F(" bytes)");
        addLog(LOG_LEVEL_INFO, log);
      }
      defaultCSS= PGMT(DATA_ESPEASY_DEFAULT_MIN_CSS);
      f.write((const unsigned char*)defaultCSS.c_str(), defaultCSS.length());
      f.close();
    }

  }
}





void addHeader(boolean showMenu, String& str)
{

}





void addFooter(const String& str)
{

}


int8_t level = 0;
int8_t lastLevel = -1;

void json_quote_name(const String& val) {
  if (lastLevel == level) TXBuffer += ",";
  if (val.length() > 0) {
    TXBuffer += '\"';
    TXBuffer += val;
    TXBuffer += '\"';
    TXBuffer += ':';
  }
}

void json_quote_val(const String& val) {
  TXBuffer += '\"';
  TXBuffer += val;
  TXBuffer += '\"';
}

void json_open(bool arr = false, const String& name = String()) {
  json_quote_name(name);
  TXBuffer += arr ? "[" : "{";
  lastLevel = level;
  level++;
}

void json_init() {
  level = 0;
  lastLevel = -1;
}

void json_close(bool arr = false) {
  TXBuffer += arr ? "]" : "}";
  level--;
  lastLevel = level;
}

void json_number(const String& name, const String& value) {
  json_quote_name(name);
  json_quote_val(value);
  lastLevel = level;
}

void json_prop(const String& name, const String& value) {
  json_quote_name(name);
  json_quote_val(value);
  lastLevel = level;
}

#ifdef WEBSERVER_NEW_UI
void handle_nodes_list_json() {
  if (!isLoggedIn()) return;
  TXBuffer.startJsonStream();
  json_init();
  json_open(true);
  for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
    {
      if (it->second.ip[0] != 0)
      {
        json_open();
        bool isThisUnit = it->first == Settings.Unit;
        if (isThisUnit)
          json_number(F("thisunit"), String(1));

        json_number(F("first"), String(it->first));
        json_prop(F("name"), isThisUnit ? Settings.Name : it->second.nodeName);
        if (it->second.build) json_prop(F("build"), String(it->second.build));
        json_prop(F("type"), getNodeTypeDisplayString(it->second.nodeType));
        json_prop(F("ip"), it->second.ip.toString());
        json_number(F("age"), String(it->second.age));
        json_close();
      }
    }
  json_close(true);
  TXBuffer.endStream();
}
#endif




void handle_root() {
  checkRAM(F("handle_root"));

  if (wifiSetup)
  {
    WebServer.send(200, F("text/html"), F("<meta HTTP-EQUIV='REFRESH' content='0; url=/setup'>"));
    return;
  }
  if (!isLoggedIn()) return;
  navMenuIndex = 0;


  if (loadFromFS(true, F("/index.htm.gz"))) return;
  if (loadFromFS(false, F("/index.htm.gz"))) return;
  if (loadFromFS(true, F("/index.htm"))) return;
  if (loadFromFS(false, F("/index.htm"))) return;

  TXBuffer.startStream();
  String sCommand = WebServer.arg(F("cmd"));
  boolean rebootCmd = strcasecmp_P(sCommand.c_str(), PSTR("reboot")) == 0;
  sendHeadandTail_stdtemplate(_HEAD, rebootCmd);

  int freeMem = ESP.getFreeHeap();


  if ((strcasecmp_P(sCommand.c_str(), PSTR("wifidisconnect")) != 0) && (rebootCmd == false)&& (strcasecmp_P(sCommand.c_str(), PSTR("reset")) != 0))
  {
    if (timerAPoff)
      timerAPoff = millis() + 2000L;



    printToWeb = true;
    printWebString = "";
    if (sCommand.length() > 0) {
      ExecuteCommand(VALUE_SOURCE_HTTP, sCommand.c_str());
    }




    TXBuffer += printWebString;
    TXBuffer += F("<form>");
    html_table_class_normal();
    addFormHeader(F("System Info"));

    addRowLabelValue(LabelType::UNIT_NR);
    addRowLabelValue(LabelType::GIT_BUILD);
    addRowLabel(getLabel(LabelType::LOCAL_TIME));
    if (systemTimePresent())
    {
      TXBuffer += getValue(LabelType::LOCAL_TIME);
    }
    else
      TXBuffer += F("<font color='red'>No system time source</font>");

    addRowLabel(getLabel(LabelType::UPTIME));
    {
        int minutes = wdcounter / 2;
        int days = minutes / 1440;
        minutes = minutes % 1440;
        int hrs = minutes / 60;
        minutes = minutes % 60;
        char strUpTime[40];
        sprintf_P(strUpTime, PSTR("%d days %d hours %d minutes"), days, hrs, minutes);
        TXBuffer += strUpTime;
    }
    addRowLabel(getLabel(LabelType::LOAD_PCT));
    if (wdcounter > 0)
    {
      TXBuffer += String(getCPUload());
      TXBuffer += F("% (LC=");
      TXBuffer += String(getLoopCountPerSec());
      TXBuffer += ')';
    }

    addRowLabel(F("Free Mem"));
    TXBuffer += String(freeMem);
    TXBuffer += " (";
    TXBuffer += String(lowestRAM);
    TXBuffer += F(" - ");
    TXBuffer += String(lowestRAMfunction);
    TXBuffer += ')';
    addRowLabel(F("Free Stack"));
    TXBuffer += String(getCurrentFreeStack());
    TXBuffer += " (";
    TXBuffer += String(lowestFreeStack);
    TXBuffer += F(" - ");
    TXBuffer += String(lowestFreeStackfunction);
    TXBuffer += ')';

    addRowLabelValue(LabelType::IP_ADDRESS);
    addRowLabel(getLabel(LabelType::WIFI_RSSI));
    if (WiFiConnected())
    {
      TXBuffer += String(WiFi.RSSI());
      TXBuffer += F(" dB (");
      TXBuffer += WiFi.SSID();
      TXBuffer += ')';
    }

    #ifdef FEATURE_MDNS
      html_TR_TD();
      TXBuffer += F("mDNS:<TD><a href='http://");
      TXBuffer += WifiGetHostname();
      TXBuffer += F(".local'>");
      TXBuffer += WifiGetHostname();
      TXBuffer += F(".local</a>");
      html_TD(3);
    #endif
    html_TR_TD();
    html_TD();
    addButton(F("sysinfo"), F("More info"));

    html_end_table();
    html_BR();
    html_BR();
    html_table_class_multirow_noborder();
    html_TR();
    html_table_header(F("Node List"));
    html_table_header("Name");
    html_table_header(getLabel(LabelType::BUILD_DESC));
    html_table_header("Type");
    html_table_header("IP", 160);
    html_table_header("Age");
    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
    {
      if (it->second.ip[0] != 0)
      {
        bool isThisUnit = it->first == Settings.Unit;
        if (isThisUnit)
          html_TR_TD_highlight();
        else
          html_TR_TD();

        TXBuffer += F("Unit ");
        TXBuffer += String(it->first);
        html_TD();
        if (isThisUnit)
          TXBuffer += Settings.Name;
        else
          TXBuffer += it->second.nodeName;
        html_TD();
        if (it->second.build)
          TXBuffer += String(it->second.build);
        html_TD();
        TXBuffer += getNodeTypeDisplayString(it->second.nodeType);
        html_TD();
        html_add_wide_button_prefix();
        TXBuffer += F("http://");
        TXBuffer += it->second.ip.toString();
        TXBuffer += "'>";
        TXBuffer += it->second.ip.toString();
        TXBuffer += "</a>";
        html_TD();
        TXBuffer += String( it->second.age);
      }
    }

    html_end_table();
    html_end_form();

    printWebString = "";
    printToWeb = false;
    sendHeadandTail_stdtemplate(_TAIL);
    TXBuffer.endStream();

  }
  else
  {





    if (strcasecmp_P(sCommand.c_str(), PSTR("wifidisconnect")) == 0)
    {
      addLog(LOG_LEVEL_INFO, F("WIFI : Disconnecting..."));
      cmd_within_mainloop = CMD_WIFI_DISCONNECT;
    }

    if (strcasecmp_P(sCommand.c_str(), PSTR("reboot")) == 0)
    {
      addLog(LOG_LEVEL_INFO, F("     : Rebooting..."));
      cmd_within_mainloop = CMD_REBOOT;
    }
   if (strcasecmp_P(sCommand.c_str(), PSTR("reset")) == 0)
    {
      addLog(LOG_LEVEL_INFO, F("     : factory reset..."));
      cmd_within_mainloop = CMD_REBOOT;
      TXBuffer += F("OK. Please wait > 1 min and connect to Acces point.<BR><BR>PW=configesp<BR>URL=<a href='http://192.168.4.1'>192.168.4.1</a>");
      TXBuffer.endStream();
      ExecuteCommand(VALUE_SOURCE_HTTP, sCommand.c_str());
    }

    TXBuffer += "OK";
    TXBuffer.endStream();

  }
}





void handle_config() {

   checkRAM(F("handle_config"));
   if (!isLoggedIn()) return;

   navMenuIndex = MENU_INDEX_CONFIG;
   TXBuffer.startStream();
   sendHeadandTail_stdtemplate(_HEAD);

  if (timerAPoff)
    timerAPoff = millis() + 2000L;


  String name = WebServer.arg(F("name"));
  name.trim();

  String iprangelow = WebServer.arg(F("iprangelow"));
  String iprangehigh = WebServer.arg(F("iprangehigh"));

  Settings.Delay = getFormItemInt(F("delay"), Settings.Delay);
  Settings.deepSleep = getFormItemInt(F("deepsleep"), Settings.deepSleep);
  String espip = WebServer.arg(F("espip"));
  String espgateway = WebServer.arg(F("espgateway"));
  String espsubnet = WebServer.arg(F("espsubnet"));
  String espdns = WebServer.arg(F("espdns"));
  Settings.Unit = getFormItemInt(F("unit"), Settings.Unit);

  String ssid = WebServer.arg(F("ssid"));


  if (ssid[0] != 0)
  {
    if (strcmp(Settings.Name, name.c_str()) != 0) {
      addLog(LOG_LEVEL_INFO, F("Unit Name changed."));
      if (CPluginCall(CPLUGIN_GOT_INVALID, 0)) {
        MQTTDisconnect();
      }
      MQTTclient_should_reconnect = true;
    }

    safe_strncpy(Settings.Name, name.c_str(), sizeof(Settings.Name));
    Settings.appendUnitToHostname(isFormItemChecked(F("appendunittohostname")));


    copyFormPassword(F("password"), SecuritySettings.Password, sizeof(SecuritySettings.Password));


    safe_strncpy(SecuritySettings.WifiSSID, ssid.c_str(), sizeof(SecuritySettings.WifiSSID));
    copyFormPassword(F("key"), SecuritySettings.WifiKey, sizeof(SecuritySettings.WifiKey));


    strncpy_webserver_arg(SecuritySettings.WifiSSID2, F("ssid2"));
    copyFormPassword(F("key2"), SecuritySettings.WifiKey2, sizeof(SecuritySettings.WifiKey2));


    copyFormPassword(F("apkey"), SecuritySettings.WifiAPKey, sizeof(SecuritySettings.WifiAPKey));



    SecuritySettings.IPblockLevel = getFormItemInt(F("ipblocklevel"));
    switch (SecuritySettings.IPblockLevel) {
      case LOCAL_SUBNET_ALLOWED:
      {
        IPAddress low, high;
        getSubnetRange(low, high);
        for (byte i=0; i < 4; ++i) {
          SecuritySettings.AllowedIPrangeLow[i] = low[i];
          SecuritySettings.AllowedIPrangeHigh[i] = high[i];
        }
        break;
      }
      case ONLY_IP_RANGE_ALLOWED:
      case ALL_ALLOWED:

        str2ip(iprangelow, SecuritySettings.AllowedIPrangeLow);

        str2ip(iprangehigh, SecuritySettings.AllowedIPrangeHigh);
        break;
    }

    Settings.deepSleepOnFail = isFormItemChecked(F("deepsleeponfail"));
    str2ip(espip, Settings.IP);
    str2ip(espgateway, Settings.Gateway);
    str2ip(espsubnet, Settings.Subnet);
    str2ip(espdns, Settings.DNS);
    addHtmlError(SaveSettings());
  }

  html_add_form();
  html_table_class_normal();

  addFormHeader(F("Main Settings"));

  Settings.Name[25] = 0;
  SecuritySettings.Password[25] = 0;
  addFormTextBox( F("Unit Name"), F("name"), Settings.Name, 25);
  addFormNumericBox( F("Unit Number"), F("unit"), Settings.Unit, 0, UNIT_NUMBER_MAX);
  addFormCheckBox(F("Append Unit Number to hostname"), F("appendunittohostname"), Settings.appendUnitToHostname());
  addFormPasswordBox(F("Admin Password"), F("password"), SecuritySettings.Password, 25);

  addFormSubHeader(F("Wifi Settings"));

  addFormTextBox( getLabel(LabelType::SSID), F("ssid"), SecuritySettings.WifiSSID, 31);
  addFormPasswordBox(F("WPA Key"), F("key"), SecuritySettings.WifiKey, 63);
  addFormTextBox( F("Fallback SSID"), F("ssid2"), SecuritySettings.WifiSSID2, 31);
  addFormPasswordBox( F("Fallback WPA Key"), F("key2"), SecuritySettings.WifiKey2, 63);
  addFormSeparator(2);
  addFormPasswordBox(F("WPA AP Mode Key"), F("apkey"), SecuritySettings.WifiAPKey, 63);


  addFormSubHeader(F("Client IP filtering"));
  {
    IPAddress low, high;
    getIPallowedRange(low, high);
    byte iplow[4];
    byte iphigh[4];
    for (byte i = 0; i < 4; ++i) {
      iplow[i] = low[i];
      iphigh[i] = high[i];
    }
    addFormIPaccessControlSelect(F("Client IP block level"), F("ipblocklevel"), SecuritySettings.IPblockLevel);
    addFormIPBox(F("Access IP lower range"), F("iprangelow"), iplow);
    addFormIPBox(F("Access IP upper range"), F("iprangehigh"), iphigh);
  }

  addFormSubHeader(F("IP Settings"));

  addFormIPBox(F("ESP IP"), F("espip"), Settings.IP);
  addFormIPBox(F("ESP GW"), F("espgateway"), Settings.Gateway);
  addFormIPBox(F("ESP Subnet"), F("espsubnet"), Settings.Subnet);
  addFormIPBox(F("ESP DNS"), F("espdns"), Settings.DNS);
  addFormNote(F("Leave empty for DHCP"));


  addFormSubHeader(F("Sleep Mode"));

  addFormNumericBox( F("Sleep awake time"), F("deepsleep"), Settings.deepSleep, 0, 255);
  addUnit(F("sec"));
  addHelpButton(F("SleepMode"));
  addFormNote(F("0 = Sleep Disabled, else time awake from sleep"));

  int dsmax = 4294;
#if defined(CORE_POST_2_5_0)
  dsmax = INT_MAX;
  if ((ESP.deepSleepMax()/1000000ULL) <= (uint64_t)INT_MAX)
    dsmax = (int)(ESP.deepSleepMax()/1000000ULL);
#endif
  addFormNumericBox( F("Sleep time"), F("delay"), Settings.Delay, 0, dsmax);
  {
    String maxSleeptimeUnit = F("sec (max: ");
    maxSleeptimeUnit += String(dsmax);
    maxSleeptimeUnit += ')';
    addUnit(maxSleeptimeUnit);
  }

  addFormCheckBox(F("Sleep on connection failure"), F("deepsleeponfail"), Settings.deepSleepOnFail);

  addFormSeparator(2);

  html_TR_TD();
  html_TD();
  addSubmitButton();
  html_end_table();
  html_end_form();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}





void handle_controllers() {
  checkRAM(F("handle_controllers"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_CONTROLLERS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  struct EventStruct TempEvent;

  byte controllerindex = getFormItemInt(F("index"), 0);
  boolean controllerNotSet = controllerindex == 0;
  --controllerindex;

  String usedns = WebServer.arg(F("usedns"));
  String controllerip = WebServer.arg(F("controllerip"));
  const int controllerport = getFormItemInt(F("controllerport"), 0);
  const int protocol = getFormItemInt(F("protocol"), -1);
  const int minimumsendinterval = getFormItemInt(F("minimumsendinterval"), 100);
  const int maxqueuedepth = getFormItemInt(F("maxqueuedepth"), 10);
  const int maxretry = getFormItemInt(F("maxretry"), 10);
  const int clienttimeout = getFormItemInt(F("clienttimeout"), CONTROLLER_CLIENTTIMEOUT_DFLT);



  if (protocol != -1 && !controllerNotSet)
  {
    MakeControllerSettings(ControllerSettings);

    if (Settings.Protocol[controllerindex] != protocol)
    {

      Settings.Protocol[controllerindex] = protocol;


      if (Settings.Protocol[controllerindex]!=0)
      {

        byte ProtocolIndex = getProtocolIndex(Settings.Protocol[controllerindex]);
        ControllerSettings.Port = Protocol[ProtocolIndex].defaultPort;
        ControllerSettings.MinimalTimeBetweenMessages = CONTROLLER_DELAY_QUEUE_DELAY_DFLT;
        ControllerSettings.ClientTimeout = CONTROLLER_CLIENTTIMEOUT_DFLT;

        if (Protocol[ProtocolIndex].usesTemplate)
          CPluginCall(ProtocolIndex, CPLUGIN_PROTOCOL_TEMPLATE, &TempEvent, dummyString);
        safe_strncpy(ControllerSettings.Subscribe, TempEvent.String1.c_str(), sizeof(ControllerSettings.Subscribe));
        safe_strncpy(ControllerSettings.Publish, TempEvent.String2.c_str(), sizeof(ControllerSettings.Publish));
        safe_strncpy(ControllerSettings.MQTTLwtTopic, TempEvent.String3.c_str(), sizeof(ControllerSettings.MQTTLwtTopic));
        safe_strncpy(ControllerSettings.LWTMessageConnect, TempEvent.String4.c_str(), sizeof(ControllerSettings.LWTMessageConnect));
        safe_strncpy(ControllerSettings.LWTMessageDisconnect, TempEvent.String5.c_str(), sizeof(ControllerSettings.LWTMessageDisconnect));
        TempEvent.String1 = "";
        TempEvent.String2 = "";
        TempEvent.String3 = "";
        TempEvent.String4 = "";
        TempEvent.String5 = "";






        ClearCustomControllerSettings(controllerindex);
      }

    }


    else
    {

      if (Settings.Protocol != 0)
      {

        byte ProtocolIndex = getProtocolIndex(Settings.Protocol[controllerindex]);
        TempEvent.ControllerIndex = controllerindex;
        TempEvent.ProtocolIndex = ProtocolIndex;
        CPluginCall(ProtocolIndex, CPLUGIN_WEBFORM_SAVE, &TempEvent, dummyString);
        ControllerSettings.UseDNS = usedns.toInt();
        if (ControllerSettings.UseDNS)
        {
          strncpy_webserver_arg(ControllerSettings.HostName, F("controllerhostname"));
          IPAddress IP;
          resolveHostByName(ControllerSettings.HostName, IP);
          for (byte x = 0; x < 4; x++)
            ControllerSettings.IP[x] = IP[x];
        }

        else
        {
          str2ip(controllerip, ControllerSettings.IP);
        }

        Settings.ControllerEnabled[controllerindex] = isFormItemChecked(F("controllerenabled"));
        ControllerSettings.Port = controllerport;
        strncpy_webserver_arg(SecuritySettings.ControllerUser[controllerindex], F("controlleruser"));

        copyFormPassword(F("controllerpassword"), SecuritySettings.ControllerPassword[controllerindex], sizeof(SecuritySettings.ControllerPassword[0]));
        strncpy_webserver_arg(ControllerSettings.Subscribe, F("controllersubscribe"));
        strncpy_webserver_arg(ControllerSettings.Publish, F("controllerpublish"));
        strncpy_webserver_arg(ControllerSettings.MQTTLwtTopic, F("mqttlwttopic"));
        strncpy_webserver_arg(ControllerSettings.LWTMessageConnect, F("lwtmessageconnect"));
        strncpy_webserver_arg(ControllerSettings.LWTMessageDisconnect, F("lwtmessagedisconnect"));
        ControllerSettings.MinimalTimeBetweenMessages = minimumsendinterval;
        ControllerSettings.MaxQueueDepth = maxqueuedepth;
        ControllerSettings.MaxRetry = maxretry;
        ControllerSettings.DeleteOldest = getFormItemInt(F("deleteoldest"), ControllerSettings.DeleteOldest);
        ControllerSettings.MustCheckReply = getFormItemInt(F("mustcheckreply"), ControllerSettings.MustCheckReply);
        ControllerSettings.ClientTimeout = clienttimeout;


        CPluginCall(ProtocolIndex, CPLUGIN_INIT, &TempEvent, dummyString);
      }
    }
    addHtmlError(SaveControllerSettings(controllerindex, ControllerSettings));
    addHtmlError(SaveSettings());
  }

  html_add_form();

  if (controllerNotSet)
  {
    html_table_class_multirow();
    html_TR();
    html_table_header("", 70);
    html_table_header("Nr", 50);
    html_table_header(F("Enabled"), 100);
    html_table_header(F("Protocol"));
    html_table_header("Host");
    html_table_header("Port");

    MakeControllerSettings(ControllerSettings);
    for (byte x = 0; x < CONTROLLER_MAX; x++)
    {
      LoadControllerSettings(x, ControllerSettings);
      html_TR_TD();
      html_add_button_prefix();
      TXBuffer += F("controllers?index=");
      TXBuffer += x + 1;
      TXBuffer += F("'>Edit</a>");
      html_TD();
      TXBuffer += getControllerSymbol(x);
      html_TD();
      if (Settings.Protocol[x] != 0)
      {
        addEnabled(Settings.ControllerEnabled[x]);

        html_TD();
        byte ProtocolIndex = getProtocolIndex(Settings.Protocol[x]);
        String ProtocolName = "";
        CPluginCall(ProtocolIndex, CPLUGIN_GET_DEVICENAME, 0, ProtocolName);
        TXBuffer += ProtocolName;

        html_TD();
        TXBuffer += ControllerSettings.getHost();
        html_TD();
        TXBuffer += ControllerSettings.Port;
      }
      else {
        html_TD(3);
      }
    }
    html_end_table();
    html_end_form();
  }
  else
  {
    html_table_class_normal();
    addFormHeader(F("Controller Settings"));
    addRowLabel(F("Protocol"));
    byte choice = Settings.Protocol[controllerindex];
    addSelector_Head(F("protocol"), true);
    addSelector_Item(F("- Standalone -"), 0, false, false, "");
    for (byte x = 0; x <= protocolCount; x++)
    {
      String ProtocolName = "";
      CPluginCall(x, CPLUGIN_GET_DEVICENAME, 0, ProtocolName);
      boolean disabled = false;
      addSelector_Item(ProtocolName,
                       Protocol[x].Number,
                       choice == Protocol[x].Number,
                       disabled,
                       "");
    }
    addSelector_Foot();

    addHelpButton(F("EasyProtocols"));
    if (Settings.Protocol[controllerindex])
    {
      MakeControllerSettings(ControllerSettings);
      LoadControllerSettings(controllerindex, ControllerSettings);
      byte choice = ControllerSettings.UseDNS;
      String options[2];
      options[0] = F("Use IP address");
      options[1] = F("Use Hostname");

      byte choice_delete_oldest = ControllerSettings.DeleteOldest;
      String options_delete_oldest[2];
      options_delete_oldest[0] = F("Ignore New");
      options_delete_oldest[1] = F("Delete Oldest");

      byte choice_mustcheckreply = ControllerSettings.MustCheckReply;
      String options_mustcheckreply[2];
      options_mustcheckreply[0] = F("Ignore Acknowledgement");
      options_mustcheckreply[1] = F("Check Acknowledgement");


      byte ProtocolIndex = getProtocolIndex(Settings.Protocol[controllerindex]);
      if (!Protocol[ProtocolIndex].Custom)
      {

        addFormSelector(F("Locate Controller"), F("usedns"), 2, options, NULL, NULL, choice, true);
        if (ControllerSettings.UseDNS)
        {
          addFormTextBox( F("Controller Hostname"), F("controllerhostname"), ControllerSettings.HostName, sizeof(ControllerSettings.HostName)-1);
        }
        else
        {
          addFormIPBox(F("Controller IP"), F("controllerip"), ControllerSettings.IP);
        }

        addFormNumericBox( F("Controller Port"), F("controllerport"), ControllerSettings.Port, 1, 65535);
        addFormNumericBox( F("Minimum Send Interval"), F("minimumsendinterval"), ControllerSettings.MinimalTimeBetweenMessages, 1, CONTROLLER_DELAY_QUEUE_DELAY_MAX);
        addUnit(F("ms"));
        addFormNumericBox( F("Max Queue Depth"), F("maxqueuedepth"), ControllerSettings.MaxQueueDepth, 1, CONTROLLER_DELAY_QUEUE_DEPTH_MAX);
        addFormNumericBox( F("Max Retries"), F("maxretry"), ControllerSettings.MaxRetry, 1, CONTROLLER_DELAY_QUEUE_RETRY_MAX);
        addFormSelector(F("Full Queue Action"), F("deleteoldest"), 2, options_delete_oldest, NULL, NULL, choice_delete_oldest, true);

        addFormSelector(F("Check Reply"), F("mustcheckreply"), 2, options_mustcheckreply, NULL, NULL, choice_mustcheckreply, true);
        addFormNumericBox( F("Client Timeout"), F("clienttimeout"), ControllerSettings.ClientTimeout, 10, CONTROLLER_CLIENTTIMEOUT_MAX);
        addUnit(F("ms"));

        if (Protocol[ProtocolIndex].usesAccount)
        {
          String protoDisplayName;
          if (!getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_USER, protoDisplayName)) {
            protoDisplayName = F("Controller User");
          }
          addFormTextBox(protoDisplayName, F("controlleruser"), SecuritySettings.ControllerUser[controllerindex], sizeof(SecuritySettings.ControllerUser[0])-1);
        }
        if (Protocol[ProtocolIndex].usesPassword)
        {
          String protoDisplayName;
          if (getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_PASS, protoDisplayName)) {

            addFormTextBox(protoDisplayName, F("controllerpassword"), SecuritySettings.ControllerPassword[controllerindex], sizeof(SecuritySettings.ControllerPassword[0])-1);
          } else {
            addFormPasswordBox(F("Controller Password"), F("controllerpassword"), SecuritySettings.ControllerPassword[controllerindex], sizeof(SecuritySettings.ControllerPassword[0])-1);
          }
        }

        if (Protocol[ProtocolIndex].usesTemplate || Protocol[ProtocolIndex].usesMQTT)
        {
          String protoDisplayName;
          if (!getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_SUBSCRIBE, protoDisplayName)) {
            protoDisplayName = F("Controller Subscribe");
          }
          addFormTextBox(protoDisplayName, F("controllersubscribe"), ControllerSettings.Subscribe, sizeof(ControllerSettings.Subscribe)-1);
        }

        if (Protocol[ProtocolIndex].usesTemplate || Protocol[ProtocolIndex].usesMQTT)
        {
          String protoDisplayName;
          if (!getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_PUBLISH, protoDisplayName)) {
            protoDisplayName = F("Controller Publish");
          }
          addFormTextBox(protoDisplayName, F("controllerpublish"), ControllerSettings.Publish, sizeof(ControllerSettings.Publish)-1);
        }

        if (Protocol[ProtocolIndex].usesMQTT)
        {
          String protoDisplayName;
          if (!getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_LWT_TOPIC, protoDisplayName)) {
            protoDisplayName = F("Controller lwl topic");
          }
          addFormTextBox(protoDisplayName, F("mqttlwttopic"), ControllerSettings.MQTTLwtTopic, sizeof(ControllerSettings.MQTTLwtTopic)-1);
        }

        if (Protocol[ProtocolIndex].usesMQTT)
        {
          String protoDisplayName;
          if (!getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_LWT_CONNECT_MESSAGE, protoDisplayName)) {
            protoDisplayName = F("LWT Connect Message");
          }
          addFormTextBox(protoDisplayName, F("lwtmessageconnect"), ControllerSettings.LWTMessageConnect, sizeof(ControllerSettings.LWTMessageConnect)-1);
        }

        if (Protocol[ProtocolIndex].usesMQTT)
        {
          String protoDisplayName;
          if (!getControllerProtocolDisplayName(ProtocolIndex, CONTROLLER_LWT_DISCONNECT_MESSAGE, protoDisplayName)) {
            protoDisplayName = F("LWT Disconnect Message");
          }
          addFormTextBox(protoDisplayName, F("lwtmessagedisconnect"), ControllerSettings.LWTMessageDisconnect, sizeof(ControllerSettings.LWTMessageDisconnect)-1);
        }
      }

      addFormCheckBox(F("Enabled"), F("controllerenabled"), Settings.ControllerEnabled[controllerindex]);

      TempEvent.ControllerIndex = controllerindex;
      TempEvent.ProtocolIndex = ProtocolIndex;
      CPluginCall(ProtocolIndex, CPLUGIN_WEBFORM_LOAD, &TempEvent,TXBuffer.buf);

    }

    addFormSeparator(2);
    html_TR_TD();
    html_TD();
    addButton(F("controllers"), F("Close"));
    addSubmitButton();
    html_end_table();
    html_end_form();
  }

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}




#ifndef NOTIFIER_SET_NONE
void handle_notifications() {
  checkRAM(F("handle_notifications"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_NOTIFICATIONS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  struct EventStruct TempEvent;



  byte notificationindex = getFormItemInt(F("index"), 0);
  boolean notificationindexNotSet = notificationindex == 0;
  --notificationindex;

  const int notification = getFormItemInt(F("notification"), -1);

  if (notification != -1 && !notificationindexNotSet)
  {
    MakeNotificationSettings(NotificationSettings);
    if (Settings.Notification[notificationindex] != notification)
    {
      Settings.Notification[notificationindex] = notification;
    }
    else
    {
      if (Settings.Notification != 0)
      {
        byte NotificationProtocolIndex = getNotificationProtocolIndex(Settings.Notification[notificationindex]);
        if (NotificationProtocolIndex!=NPLUGIN_NOT_FOUND)
          NPlugin_ptr[NotificationProtocolIndex](NPLUGIN_WEBFORM_SAVE, 0, dummyString);
        NotificationSettings.Port = getFormItemInt(F("port"), 0);
        NotificationSettings.Pin1 = getFormItemInt(F("pin1"), 0);
        NotificationSettings.Pin2 = getFormItemInt(F("pin2"), 0);
        Settings.NotificationEnabled[notificationindex] = isFormItemChecked(F("notificationenabled"));
        strncpy_webserver_arg(NotificationSettings.Domain, F("domain"));
        strncpy_webserver_arg(NotificationSettings.Server, F("server"));
        strncpy_webserver_arg(NotificationSettings.Sender, F("sender"));
        strncpy_webserver_arg(NotificationSettings.Receiver, F("receiver"));
        strncpy_webserver_arg(NotificationSettings.Subject, F("subject"));
        strncpy_webserver_arg(NotificationSettings.User, F("user"));
        strncpy_webserver_arg(NotificationSettings.Pass, F("pass"));
        strncpy_webserver_arg(NotificationSettings.Body, F("body"));

      }
    }

    addHtmlError(SaveNotificationSettings(notificationindex, (byte*)&NotificationSettings, sizeof(NotificationSettingsStruct)));
    addHtmlError(SaveSettings());
    if (WebServer.hasArg(F("test"))) {

      byte NotificationProtocolIndex = getNotificationProtocolIndex(Settings.Notification[notificationindex]);
      if (NotificationProtocolIndex != NPLUGIN_NOT_FOUND)
      {

        TempEvent.NotificationIndex = notificationindex;
        schedule_notification_event_timer(NotificationProtocolIndex, NPLUGIN_NOTIFY, &TempEvent);
      }
    }
  }

  html_add_form();

  if (notificationindexNotSet)
  {
    html_table_class_multirow();
    html_TR();
    html_table_header("", 70);
    html_table_header("Nr", 50);
    html_table_header(F("Enabled"), 100);
    html_table_header(F("Service"));
    html_table_header(F("Server"));
    html_table_header("Port");

    MakeNotificationSettings(NotificationSettings);
    for (byte x = 0; x < NOTIFICATION_MAX; x++)
    {
      LoadNotificationSettings(x, (byte*)&NotificationSettings, sizeof(NotificationSettingsStruct));
      NotificationSettings.validate();
      html_TR_TD();
      html_add_button_prefix();
      TXBuffer += F("notifications?index=");
      TXBuffer += x + 1;
      TXBuffer += F("'>Edit</a>");
      html_TD();
      TXBuffer += x + 1;
      html_TD();
      if (Settings.Notification[x] != 0)
      {
        addEnabled(Settings.NotificationEnabled[x]);

        html_TD();
        byte NotificationProtocolIndex = getNotificationProtocolIndex(Settings.Notification[x]);
        String NotificationName = F("(plugin not found?)");
        if (NotificationProtocolIndex!=NPLUGIN_NOT_FOUND)
        {
          NPlugin_ptr[NotificationProtocolIndex](NPLUGIN_GET_DEVICENAME, 0, NotificationName);
        }
        TXBuffer += NotificationName;
        html_TD();
        TXBuffer += NotificationSettings.Server;
        html_TD();
        TXBuffer += NotificationSettings.Port;
      }
      else {
        html_TD(3);
      }
    }
    html_end_table();
    html_end_form();
  }
  else
  {
    html_table_class_normal();
    addFormHeader(F("Notification Settings"));
    addRowLabel(F("Notification"));
    byte choice = Settings.Notification[notificationindex];
    addSelector_Head(F("notification"), true);
    addSelector_Item(F("- None -"), 0, false, false, "");
    for (byte x = 0; x <= notificationCount; x++)
    {
      String NotificationName = "";
      NPlugin_ptr[x](NPLUGIN_GET_DEVICENAME, 0, NotificationName);
      addSelector_Item(NotificationName,
                       Notification[x].Number,
                       choice == Notification[x].Number,
                       false,
                       "");
    }
    addSelector_Foot();

    addHelpButton(F("EasyNotifications"));

    if (Settings.Notification[notificationindex])
    {
      MakeNotificationSettings(NotificationSettings);
      LoadNotificationSettings(notificationindex, (byte*)&NotificationSettings, sizeof(NotificationSettingsStruct));
      NotificationSettings.validate();

      byte NotificationProtocolIndex = getNotificationProtocolIndex(Settings.Notification[notificationindex]);
      if (NotificationProtocolIndex!=NPLUGIN_NOT_FOUND)
      {

        if (Notification[NotificationProtocolIndex].usesMessaging)
        {
          addFormTextBox(F("Domain"), F("domain"), NotificationSettings.Domain, sizeof(NotificationSettings.Domain)-1);
          addFormTextBox(F("Server"), F("server"), NotificationSettings.Server, sizeof(NotificationSettings.Server)-1);
          addFormNumericBox(F("Port"), F("port"), NotificationSettings.Port, 1, 65535);

          addFormTextBox(F("Sender"), F("sender"), NotificationSettings.Sender, sizeof(NotificationSettings.Sender)-1);
          addFormTextBox(F("Receiver"), F("receiver"), NotificationSettings.Receiver, sizeof(NotificationSettings.Receiver)-1);
          addFormTextBox(F("Subject"), F("subject"), NotificationSettings.Subject, sizeof(NotificationSettings.Subject)-1);

          addFormTextBox(F("User"), F("user"), NotificationSettings.User, sizeof(NotificationSettings.User)-1);
          addFormTextBox(F("Pass"), F("pass"), NotificationSettings.Pass, sizeof(NotificationSettings.Pass)-1);

          addRowLabel(F("Body"));
          TXBuffer += F("<textarea name='body' rows='20' size=512 wrap='off'>");
          TXBuffer += NotificationSettings.Body;
          TXBuffer += F("</textarea>");
        }

        if (Notification[NotificationProtocolIndex].usesGPIO > 0)
        {
          addRowLabel(F("1st GPIO"));
          addPinSelect(false, "pin1", NotificationSettings.Pin1);
        }

        addRowLabel(F("Enabled"));
        addCheckBox(F("notificationenabled"), Settings.NotificationEnabled[notificationindex]);

        TempEvent.NotificationIndex = notificationindex;
        NPlugin_ptr[NotificationProtocolIndex](NPLUGIN_WEBFORM_LOAD, &TempEvent,TXBuffer.buf);
      }
    }

    addFormSeparator(2);

    html_TR_TD();
    html_TD();
    addButton(F("notifications"), F("Close"));
    addSubmitButton();
    addSubmitButton(F("Test"), F("test"));
    html_end_table();
    html_end_form();
  }
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}
#endif




void handle_hardware() {
  checkRAM(F("handle_hardware"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_HARDWARE;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  if (isFormItem(F("psda")))
  {
    Settings.Pin_status_led = getFormItemInt(F("pled"));
    Settings.Pin_status_led_Inversed = isFormItemChecked(F("pledi"));
    Settings.Pin_Reset = getFormItemInt(F("pres"));
    Settings.Pin_i2c_sda = getFormItemInt(F("psda"));
    Settings.Pin_i2c_scl = getFormItemInt(F("pscl"));
    Settings.InitSPI = isFormItemChecked(F("initspi"));
    Settings.Pin_sd_cs = getFormItemInt(F("sd"));
    int gpio = 0;

    while (gpio < MAX_GPIO && gpio < 17) {
      if (Settings.UseSerial && (gpio == 1 || gpio == 3)) {

      } else {
        int pinnr = -1;
        bool input, output, warning;
        if (getGpioInfo(gpio, pinnr, input, output, warning)) {
          String int_pinlabel = "p";
          int_pinlabel += gpio;
          Settings.PinBootStates[gpio] = getFormItemInt(int_pinlabel);
        }
      }
      ++gpio;
    }
    addHtmlError(SaveSettings());
  }

  TXBuffer += F("<form  method='post'>");
  html_table_class_normal();
  addFormHeader(F("Hardware Settings"), F("ESPEasy#Hardware_page"));

  addFormSubHeader(F("Wifi Status LED"));
  addFormPinSelect(formatGpioName_output("LED"), "pled", Settings.Pin_status_led);
  addFormCheckBox(F("Inversed LED"), F("pledi"), Settings.Pin_status_led_Inversed);
  addFormNote(F("Use &rsquo;GPIO-2 (D4)&rsquo; with &rsquo;Inversed&rsquo; checked for onboard LED"));

  addFormSubHeader(F("Reset Pin"));
  addFormPinSelect(formatGpioName_input(F("Switch")), "pres", Settings.Pin_Reset);
  addFormNote(F("Press about 10s for factory reset"));

  addFormSubHeader(F("I2C Interface"));
  addFormPinSelectI2C(formatGpioName_bidirectional("SDA"), F("psda"), Settings.Pin_i2c_sda);
  addFormPinSelectI2C(formatGpioName_output("SCL"), F("pscl"), Settings.Pin_i2c_scl);


  addFormSubHeader(F("SPI Interface"));
  addFormCheckBox(F("Init SPI"), F("initspi"), Settings.InitSPI);
  addFormNote(F("CLK=GPIO-14 (D5), MISO=GPIO-12 (D6), MOSI=GPIO-13 (D7)"));
  addFormNote(F("Chip Select (CS) config must be done in the plugin"));
#ifdef FEATURE_SD
  addFormPinSelect(formatGpioName_output("SD Card CS"), "sd", Settings.Pin_sd_cs);
#endif

  addFormSubHeader(F("GPIO boot states"));
  int gpio = 0;

  while (gpio < MAX_GPIO && gpio < 17) {
    bool enabled = true;
    if (Settings.UseSerial && (gpio == 1 || gpio == 3)) {

      enabled = false;
    }
    int pinnr = -1;
    bool input, output, warning;
    if (getGpioInfo(gpio, pinnr, input, output, warning)) {
      String label;
      label.reserve(32);
      label = F("Pin mode ");
      label += createGPIO_label(gpio, pinnr, input, output, warning);
      String int_pinlabel = "p";
      int_pinlabel += gpio;
      addFormPinStateSelect(label, int_pinlabel, Settings.PinBootStates[gpio], enabled);
    }
    ++gpio;
  }
  addFormSeparator(2);

  html_TR_TD();
  html_TD();
  addSubmitButton();
  html_TR_TD();
  html_end_table();
  html_end_form();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();

}




void addFormPinStateSelect(const String& label, const String& id, int choice, bool enabled)
{
  addRowLabel(label);
  addPinStateSelect(id, choice, enabled);
}

void addPinStateSelect(const String& name, int choice, bool enabled)
{
  String options[4] = { F("Default"), F("Output Low"), F("Output High"), F("Input") };
  addSelector(name, 4, options, NULL, NULL, choice, false, enabled);
}




void addFormIPaccessControlSelect(const String& label, const String& id, int choice)
{
  addRowLabel(label);
  addIPaccessControlSelect(id, choice);
}

void addIPaccessControlSelect(const String& name, int choice)
{
  String options[3] = { F("Allow All"), F("Allow Local Subnet"), F("Allow IP range") };
  addSelector(name, 3, options, NULL, NULL, choice, false);
}
# 1972 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
void setTaskDevice_to_TaskIndex(byte taskdevicenumber, byte taskIndex) {
  struct EventStruct TempEvent;
  TempEvent.TaskIndex = taskIndex;
  String dummy;


  PluginCall(PLUGIN_EXIT, &TempEvent, dummy);
  taskClear(taskIndex, false);
  ClearCustomTaskSettings(taskIndex);

  Settings.TaskDeviceNumber[taskIndex] = taskdevicenumber;
  if (taskdevicenumber != 0)
  {

    PluginCall(PLUGIN_SET_DEFAULTS, &TempEvent, dummy);
    PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, dummy);
  } else {

    taskClear(taskIndex, true);
  }
}

void setBasicTaskValues(byte taskIndex, unsigned long taskdevicetimer,
                        bool enabled, const String& name, int pin1, int pin2, int pin3) {
    LoadTaskSettings(taskIndex);
    byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[taskIndex]);
    if (taskdevicetimer > 0) {
      Settings.TaskDeviceTimer[taskIndex] = taskdevicetimer;
    } else {
      if (!Device[DeviceIndex].TimerOptional)
        Settings.TaskDeviceTimer[taskIndex] = Settings.Delay;
      else
        Settings.TaskDeviceTimer[taskIndex] = 0;
    }
    Settings.TaskDeviceEnabled[taskIndex] = enabled;
    safe_strncpy(ExtraTaskSettings.TaskDeviceName, name.c_str(), sizeof(ExtraTaskSettings.TaskDeviceName));
    if (pin1 >= 0) Settings.TaskDevicePin1[taskIndex] = pin1;
    if (pin2 >= 0) Settings.TaskDevicePin2[taskIndex] = pin2;
    if (pin3 >= 0) Settings.TaskDevicePin3[taskIndex] = pin3;
    SaveTaskSettings(taskIndex);
}


void handle_devices() {
  checkRAM(F("handle_devices"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_DEVICES;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);



  struct EventStruct TempEvent;



  byte taskdevicenumber;
  if (WebServer.hasArg(F("del")))
    taskdevicenumber=0;
  else
    taskdevicenumber = getFormItemInt(F("TDNUM"), 0);


  unsigned long taskdevicetimer = getFormItemInt(F("TDT"),0);
# 2084 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
  byte page = getFormItemInt(F("page"), 0);
  if (page == 0)
    page = 1;
  byte setpage = getFormItemInt(F("setpage"), 0);
  if (setpage > 0)
  {
    if (setpage <= (TASKS_MAX / TASKS_PER_PAGE))
      page = setpage;
    else
      page = TASKS_MAX / TASKS_PER_PAGE;
  }
  const int edit = getFormItemInt(F("edit"), 0);



  byte taskIndex = getFormItemInt(F("index"), 0);
  boolean taskIndexNotSet = taskIndex == 0;
  --taskIndex;

  byte DeviceIndex = 0;
  LoadTaskSettings(taskIndex);

  if (edit != 0 && !taskIndexNotSet)
  {
    if (Settings.TaskDeviceNumber[taskIndex] != taskdevicenumber)
    {

      setTaskDevice_to_TaskIndex(taskdevicenumber, taskIndex);
    }
    else if (taskdevicenumber != 0)
    {
      Settings.TaskDeviceNumber[taskIndex] = taskdevicenumber;
      int pin1 = -1;
      int pin2 = -1;
      int pin3 = -1;
      update_whenset_FormItemInt(F("taskdevicepin1"), pin1);
      update_whenset_FormItemInt(F("taskdevicepin2"), pin2);
      update_whenset_FormItemInt(F("taskdevicepin3"), pin3);
      setBasicTaskValues(taskIndex, taskdevicetimer,
                         isFormItemChecked(F("TDE")), WebServer.arg(F("TDN")),
                         pin1, pin2, pin3);
      Settings.TaskDevicePort[taskIndex] = getFormItemInt(F("TDP"), 0);

      for (byte controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
      {
        Settings.TaskDeviceID[controllerNr][taskIndex] = getFormItemInt(String(F("TDID")) + (controllerNr + 1));
        Settings.TaskDeviceSendData[controllerNr][taskIndex] = isFormItemChecked(String(F("TDSD")) + (controllerNr + 1));
      }

      DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[taskIndex]);
      if (Device[DeviceIndex].PullUpOption)
        Settings.TaskDevicePin1PullUp[taskIndex] = isFormItemChecked(F("TDPPU"));

      if (Device[DeviceIndex].InverseLogicOption)
        Settings.TaskDevicePin1Inversed[taskIndex] = isFormItemChecked(F("TDPI"));

      for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++)
      {
        strncpy_webserver_arg(ExtraTaskSettings.TaskDeviceFormula[varNr], String(F("TDF")) + (varNr + 1));
        ExtraTaskSettings.TaskDeviceValueDecimals[varNr] = getFormItemInt(String(F("TDVD")) + (varNr + 1));
        strncpy_webserver_arg(ExtraTaskSettings.TaskDeviceValueNames[varNr], String(F("TDVN")) + (varNr + 1));






      }
# 2160 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
      TempEvent.TaskIndex = taskIndex;
      if (ExtraTaskSettings.TaskIndex != TempEvent.TaskIndex)
        PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, dummyString);


      PluginCall(PLUGIN_WEBFORM_SAVE, &TempEvent, dummyString);


      for (byte x=0; x < CONTROLLER_MAX; x++)
        {
          TempEvent.ControllerIndex = x;
          if (Settings.TaskDeviceSendData[TempEvent.ControllerIndex][TempEvent.TaskIndex] &&
            Settings.ControllerEnabled[TempEvent.ControllerIndex] && Settings.Protocol[TempEvent.ControllerIndex])
            {
              TempEvent.ProtocolIndex = getProtocolIndex(Settings.Protocol[TempEvent.ControllerIndex]);
              CPluginCall(TempEvent.ProtocolIndex, CPLUGIN_TASK_CHANGE_NOTIFICATION, &TempEvent, dummyString);
            }
        }
    }
    if (taskdevicenumber != 0) {


      addHtmlError(SaveTaskSettings(taskIndex));
      addHtmlError(SaveSettings());
      if (Settings.TaskDeviceEnabled[taskIndex]) {
        PluginCall(PLUGIN_INIT, &TempEvent, dummyString);
      }
    }
  }


  if (taskIndexNotSet)
  {
    html_add_script(true);
    TXBuffer += DATA_UPDATE_SENSOR_VALUES_DEVICE_PAGE_JS;
    html_add_script_end();
    html_table_class_multirow();
    html_TR();
    html_table_header("", 70);

    if (TASKS_MAX != TASKS_PER_PAGE)
    {
      html_add_button_prefix();
      TXBuffer += F("devices?setpage=");
      if (page > 1)
        TXBuffer += page - 1;
      else
        TXBuffer += page;
      TXBuffer += F("'>&lt;</a>");
      html_add_button_prefix();
      TXBuffer += F("devices?setpage=");
      if (page < (TASKS_MAX / TASKS_PER_PAGE))
        TXBuffer += page + 1;
      else
        TXBuffer += page;
      TXBuffer += F("'>&gt;</a>");
    }

    html_table_header("Task", 50);
    html_table_header(F("Enabled"), 100);
    html_table_header(F("Device"));
    html_table_header("Name");
    html_table_header("Port");
    html_table_header(F("Ctr (IDX)"), 100);
    html_table_header("GPIO", 70);
    html_table_header(F("Values"));

    String deviceName;

    for (byte x = (page - 1) * TASKS_PER_PAGE; x < ((page) * TASKS_PER_PAGE); x++)
    {
      html_TR_TD();
      html_add_button_prefix();
      TXBuffer += F("devices?index=");
      TXBuffer += x + 1;
      TXBuffer += F("&page=");
      TXBuffer += page;
      TXBuffer += F("'>Edit</a>");
      html_TD();
      TXBuffer += x + 1;
      html_TD();

      if (Settings.TaskDeviceNumber[x] != 0)
      {
        LoadTaskSettings(x);
        DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[x]);
        TempEvent.TaskIndex = x;
        addEnabled( Settings.TaskDeviceEnabled[x]);

        html_TD();
        TXBuffer += getPluginNameFromDeviceIndex(DeviceIndex);
        html_TD();
        TXBuffer += ExtraTaskSettings.TaskDeviceName;
        html_TD();

        byte customConfig = false;
        customConfig = PluginCall(PLUGIN_WEBFORM_SHOW_CONFIG, &TempEvent,TXBuffer.buf);
        if (!customConfig)
          if (Device[DeviceIndex].Ports != 0)
            TXBuffer += formatToHex_decimal(Settings.TaskDevicePort[x]);

        html_TD();

        if (Device[DeviceIndex].SendDataOption)
        {
          boolean doBR = false;
          for (byte controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
          {
            byte ProtocolIndex = getProtocolIndex(Settings.Protocol[controllerNr]);
            if (Settings.TaskDeviceSendData[controllerNr][x])
            {
              if (doBR)
                TXBuffer += F("<BR>");
              TXBuffer += getControllerSymbol(controllerNr);
              if (Protocol[ProtocolIndex].usesID && Settings.Protocol[controllerNr] != 0)
              {
                TXBuffer += " (";
                TXBuffer += Settings.TaskDeviceID[controllerNr][x];
                TXBuffer += ')';
                if (Settings.TaskDeviceID[controllerNr][x] == 0)
                  TXBuffer += F(" " HTML_SYMBOL_WARNING);
              }
              doBR = true;
            }
          }
        }

        html_TD();

        if (Settings.TaskDeviceDataFeed[x] == 0)
        {
          if (Device[DeviceIndex].Type == DEVICE_TYPE_I2C)
          {
            TXBuffer += F("GPIO-");
            TXBuffer += Settings.Pin_i2c_sda;
            TXBuffer += F("<BR>GPIO-");
            TXBuffer += Settings.Pin_i2c_scl;
          }
          if (Device[DeviceIndex].Type == DEVICE_TYPE_ANALOG)
            TXBuffer += F("ADC (TOUT)");

          if (Settings.TaskDevicePin1[x] != -1)
          {
            TXBuffer += F("GPIO-");
            TXBuffer += Settings.TaskDevicePin1[x];
          }

          if (Settings.TaskDevicePin2[x] != -1)
          {
            TXBuffer += F("<BR>GPIO-");
            TXBuffer += Settings.TaskDevicePin2[x];
          }

          if (Settings.TaskDevicePin3[x] != -1)
          {
            TXBuffer += F("<BR>GPIO-");
            TXBuffer += Settings.TaskDevicePin3[x];
          }
        }

        html_TD();
        byte customValues = false;
        customValues = PluginCall(PLUGIN_WEBFORM_SHOW_VALUES, &TempEvent,TXBuffer.buf);
        if (!customValues)
        {
          for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++)
          {
            if (Settings.TaskDeviceNumber[x] != 0)
            {
              if (varNr > 0)
                TXBuffer += F("<div class='div_br'></div>");
              TXBuffer += F("<div class='div_l' id='valuename_");
              TXBuffer += x;
              TXBuffer += '_';
              TXBuffer += varNr;
              TXBuffer += "'>";
              TXBuffer += ExtraTaskSettings.TaskDeviceValueNames[varNr];
              TXBuffer += F(":</div><div class='div_r' id='value_");
              TXBuffer += x;
              TXBuffer += '_';
              TXBuffer += varNr;
              TXBuffer += "'>";
              TXBuffer += formatUserVarNoCheck(x, varNr);
              TXBuffer += "</div>";
            }
          }
        }
      }
      else {
        html_TD(6);
      }

    }
    html_end_table();
    html_end_form();

  }

  else
  {
    LoadTaskSettings(taskIndex);
    DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[taskIndex]);
    TempEvent.TaskIndex = taskIndex;

    html_add_form();
    html_table_class_normal();
    addFormHeader(F("Task Settings"));


    TXBuffer += F("<TR><TD style='width:150px;' align='left'>Device:<TD>");


    if (Settings.TaskDeviceNumber[taskIndex] == 0 )
    {

      addDeviceSelect("TDNUM", Settings.TaskDeviceNumber[taskIndex]);

    }

    else
    {

      TXBuffer += F("<input type='hidden' name='TDNUM' value='");
      TXBuffer += Settings.TaskDeviceNumber[taskIndex];
      TXBuffer += "'>";


      TXBuffer += getPluginNameFromDeviceIndex(DeviceIndex);

      addHelpButton(String(F("Plugin")) + Settings.TaskDeviceNumber[taskIndex]);
      addRTDPluginButton(Settings.TaskDeviceNumber[taskIndex]);


      if (Device[DeviceIndex].Number == 3 && taskIndex >= 4)
        {
          addFormNote(F("This plugin is only supported on task 1-4 for now"));
        }

      addFormTextBox( F("Name"), F("TDN"), ExtraTaskSettings.TaskDeviceName, NAME_FORMULA_LENGTH_MAX);

      addFormCheckBox(F("Enabled"), F("TDE"), Settings.TaskDeviceEnabled[taskIndex]);


      if (!Device[DeviceIndex].Custom && Settings.TaskDeviceDataFeed[taskIndex] == 0 &&
          ((Device[DeviceIndex].Ports != 0) ||
           (Device[DeviceIndex].PullUpOption) ||
           (Device[DeviceIndex].InverseLogicOption) ||
           (Device[DeviceIndex].connectedToGPIOpins())) )
      {
        addFormSubHeader((Device[DeviceIndex].SendDataOption) ? F("Sensor") : F("Actuator"));

        if (Device[DeviceIndex].Ports != 0)
          addFormNumericBox(F("Port"), F("TDP"), Settings.TaskDevicePort[taskIndex]);

        if (Device[DeviceIndex].PullUpOption)
        {
          addFormCheckBox(F("Internal PullUp"), F("TDPPU"), Settings.TaskDevicePin1PullUp[taskIndex]);
          #if defined(ESP8266)
          if ((Settings.TaskDevicePin1[taskIndex] == 16) || (Settings.TaskDevicePin2[taskIndex] == 16) || (Settings.TaskDevicePin3[taskIndex] == 16))
            addFormNote(F("PullDown for GPIO-16 (D0)"));
          #endif
        }

        if (Device[DeviceIndex].InverseLogicOption)
        {
          addFormCheckBox(F("Inversed Logic"), F("TDPI"), Settings.TaskDevicePin1Inversed[taskIndex]);
          addFormNote(F("Will go into effect on next input change."));
        }


        TempEvent.String1 = F("1st GPIO");
        TempEvent.String2 = F("2nd GPIO");
        TempEvent.String3 = F("3rd GPIO");
        PluginCall(PLUGIN_GET_DEVICEGPIONAMES, &TempEvent, dummyString);

        if (Device[DeviceIndex].connectedToGPIOpins()) {
          if (Device[DeviceIndex].Type >= DEVICE_TYPE_SINGLE)
            addFormPinSelect(TempEvent.String1, F("taskdevicepin1"), Settings.TaskDevicePin1[taskIndex]);
          if (Device[DeviceIndex].Type >= DEVICE_TYPE_DUAL)
            addFormPinSelect(TempEvent.String2, F("taskdevicepin2"), Settings.TaskDevicePin2[taskIndex]);
          if (Device[DeviceIndex].Type == DEVICE_TYPE_TRIPLE)
            addFormPinSelect(TempEvent.String3, F("taskdevicepin3"), Settings.TaskDevicePin3[taskIndex]);
        }
      }


      if (Settings.TaskDeviceDataFeed[taskIndex] == 0) {
        String webformLoadString;
        PluginCall(PLUGIN_WEBFORM_LOAD, &TempEvent,webformLoadString);
        if (webformLoadString.length() > 0) {
          String errorMessage;
          PluginCall(PLUGIN_GET_DEVICENAME, &TempEvent, errorMessage);
          errorMessage += F(": Bug in PLUGIN_WEBFORM_LOAD, should not append to string, use addHtml() instead");
          addHtmlError(errorMessage);
        }
      }


      if (Device[DeviceIndex].SendDataOption)
      {
        addFormSubHeader(F("Data Acquisition"));

        for (byte controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
        {
          if (Settings.Protocol[controllerNr] != 0)
          {
            String id = F("TDSD");
            id += controllerNr + 1;

            html_TR_TD(); TXBuffer += F("Send to Controller ");
            TXBuffer += getControllerSymbol(controllerNr);
            html_TD();
            addCheckBox(id, Settings.TaskDeviceSendData[controllerNr][taskIndex]);

            byte ProtocolIndex = getProtocolIndex(Settings.Protocol[controllerNr]);
            if (Protocol[ProtocolIndex].usesID && Settings.Protocol[controllerNr] != 0)
            {
              addRowLabel(F("IDX"));
              id = F("TDID");
              id += controllerNr + 1;
              addNumericBox(id, Settings.TaskDeviceID[controllerNr][taskIndex], 0, DOMOTICZ_MAX_IDX);
            }
          }
        }
      }

      addFormSeparator(2);

      if (Device[DeviceIndex].TimerOption)
      {

        addFormNumericBox( F("Interval"), F("TDT"), Settings.TaskDeviceTimer[taskIndex], 0, 65535);
        addUnit(F("sec"));
        if (Device[DeviceIndex].TimerOptional)
          TXBuffer += F(" (Optional for this Device)");
      }


      if (!Device[DeviceIndex].Custom && Device[DeviceIndex].ValueCount > 0)
      {
        addFormSubHeader(F("Values"));
        html_end_table();
        html_table_class_normal();


        TXBuffer += F("<TR><TH style='width:30px;' align='center'>#");
        html_table_header("Name");

        if (Device[DeviceIndex].FormulaOption)
        {
          html_table_header(F("Formula"), F("EasyFormula"), 0);
        }

        if (Device[DeviceIndex].FormulaOption || Device[DeviceIndex].DecimalsOnly)
        {
          html_table_header(F("Decimals"), 30);
        }


        for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++)
        {
          html_TR_TD();
          TXBuffer += varNr + 1;
          html_TD();
          String id = F("TDVN");
          id += (varNr + 1);
          addTextBox(id, ExtraTaskSettings.TaskDeviceValueNames[varNr], NAME_FORMULA_LENGTH_MAX);

          if (Device[DeviceIndex].FormulaOption)
          {
            html_TD();
            String id = F("TDF");
            id += (varNr + 1);
            addTextBox(id, ExtraTaskSettings.TaskDeviceFormula[varNr], NAME_FORMULA_LENGTH_MAX);
          }

          if (Device[DeviceIndex].FormulaOption || Device[DeviceIndex].DecimalsOnly)
          {
            html_TD();
            String id = F("TDVD");
            id += (varNr + 1);
            addNumericBox(id, ExtraTaskSettings.TaskDeviceValueDecimals[varNr], 0, 6);
          }
        }
      }
    }

    addFormSeparator(4);

    html_TR_TD();
    TXBuffer += F("<TD colspan='3'>");
    html_add_button_prefix();
    TXBuffer += F("devices?setpage=");
    TXBuffer += page;
    TXBuffer += F("'>Close</a>");
    addSubmitButton();
    TXBuffer += F("<input type='hidden' name='edit' value='1'>");
    TXBuffer += F("<input type='hidden' name='page' value='1'>");


    if (Settings.TaskDeviceNumber[taskIndex] != 0 )
      addSubmitButton(F("Delete"), F("del"));

    html_end_table();
    html_end_form();
  }


  checkRAM(F("handle_devices"));
#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
    String log = F("DEBUG: String size:");
    log += String(TXBuffer.sentBytes);
    addLog(LOG_LEVEL_DEBUG_DEV, log);
  }
#endif
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}


byte sortedIndex[DEVICES_MAX + 1];



void addDeviceSelect(const String& name, int choice)
{

  for (byte x = 0; x <= deviceCount; x++)
    sortedIndex[x] = x;
  sortDeviceArray();

  String deviceName;

  addSelector_Head(name, true);
  addSelector_Item(F("- None -"), 0, false, false, "");
  for (byte x = 0; x <= deviceCount; x++)
  {
    byte deviceIndex = sortedIndex[x];
    if (Plugin_id[deviceIndex] != 0)
      deviceName = getPluginNameFromDeviceIndex(deviceIndex);

#ifdef PLUGIN_BUILD_DEV
    int num = Plugin_id[deviceIndex];
    String plugin = "P";
    if (num < 10) plugin += '0';
    if (num < 100) plugin += '0';
    plugin += num;
    plugin += F(" - ");
    deviceName = plugin + deviceName;
#endif

    addSelector_Item(deviceName,
                     Device[deviceIndex].Number,
                     choice == Device[deviceIndex].Number,
                     false,
                     "");
  }
  addSelector_Foot();
}




boolean arrayLessThan(const String& ptr_1, const String& ptr_2)
{
  unsigned int i = 0;
  while (i < ptr_1.length())
  {
    if (ptr_2.length() < i)
    {
      return true;
    }
    else
    {
      const char check1 = (char)ptr_1[i];
      const char check2 = (char)ptr_2[i];
      if (check1 == check2) {

        i++;
      } else {
        return (check2 > check1);
      }
    }
  }
  return false;
}





void sortDeviceArray()
{
  int innerLoop ;
  int mainLoop ;
  for ( mainLoop = 1; mainLoop <= deviceCount; mainLoop++)
  {
    innerLoop = mainLoop;
    while (innerLoop >= 1)
    {
      if (arrayLessThan(
        getPluginNameFromDeviceIndex(sortedIndex[innerLoop]),
        getPluginNameFromDeviceIndex(sortedIndex[innerLoop - 1])))
      {
        byte temp = sortedIndex[innerLoop - 1];
        sortedIndex[innerLoop - 1] = sortedIndex[innerLoop];
        sortedIndex[innerLoop] = temp;
      }
      innerLoop--;
    }
  }
}

void addFormPinSelect(const String& label, const String& id, int choice)
{
  addRowLabel(label, String("tr_")+id);
  addPinSelect(false, id, choice);
}


void addFormPinSelectI2C(const String& label, const String& id, int choice)
{

  addRowLabel(label, String("tr_")+id);
  addPinSelect(true, id, choice);
}





String createGPIO_label(int gpio, int pinnr, bool input, bool output, bool warning) {
  if (gpio < 0) return F("- None -");
  String result;
  result.reserve(24);
  result = F("GPIO-");
  result += gpio;
  if (pinnr >= 0) {
    result += F(" (D");
    result += pinnr;
    result += ')';
  }
  if (input != output) {
    result += ' ';
    result += input ? F(HTML_SYMBOL_INPUT) : F(HTML_SYMBOL_OUTPUT);
  }
  if (warning) {
    result += ' ';
    result += F(HTML_SYMBOL_WARNING);
  }
  bool serialPinConflict = (Settings.UseSerial && (gpio == 1 || gpio == 3));
  if (serialPinConflict) {
    if (gpio == 1) { result += F(" TX0"); }
    if (gpio == 3) { result += F(" RX0"); }
  }
  return result;
}

void addPinSelect(boolean forI2C, String id, int choice)
{
  #ifdef ESP32
    #define NR_ITEMS_PIN_DROPDOWN 35
  #else
    #define NR_ITEMS_PIN_DROPDOWN 14
  #endif

  String * gpio_labels = new String[NR_ITEMS_PIN_DROPDOWN];
  int * gpio_numbers = new int[NR_ITEMS_PIN_DROPDOWN];


  int i = 0;
  int gpio = -1;
  while (i < NR_ITEMS_PIN_DROPDOWN && gpio <= MAX_GPIO) {
    int pinnr = -1;
    bool input, output, warning;
    if (getGpioInfo(gpio, pinnr, input, output, warning) || i == 0) {
      gpio_labels[i] = createGPIO_label(gpio, pinnr, input, output, warning);
      gpio_numbers[i] = gpio;
      ++i;
    }
    ++gpio;
  }
  renderHTMLForPinSelect(gpio_labels, gpio_numbers, forI2C, id, choice, NR_ITEMS_PIN_DROPDOWN);
  delete[] gpio_numbers;
  delete[] gpio_labels;
  #undef NR_ITEMS_PIN_DROPDOWN
}





void renderHTMLForPinSelect(String options[], int optionValues[], boolean forI2C, const String& id, int choice, int count) {
  addSelector_Head(id, false);
  for (byte x = 0; x < count; x++)
  {
    boolean disabled = false;

    if (optionValues[x] != -1)
    {
      if (!forI2C && ((optionValues[x] == Settings.Pin_i2c_sda) || (optionValues[x] == Settings.Pin_i2c_scl)))
        disabled = true;
      if (Settings.UseSerial && ((optionValues[x] == 1) || (optionValues[x] == 3)))
        disabled = true;
    }
    addSelector_Item(options[x],
                     optionValues[x],
                     choice == optionValues[x],
                     disabled,
                     "");
  }
  addSelector_Foot();
}


void addFormSelectorI2C(const String& id, int addressCount, const int addresses[], int selectedIndex)
{
  String options[addressCount];
  for (byte x = 0; x < addressCount; x++)
  {
    options[x] = formatToHex_decimal(addresses[x]);
    if (x == 0)
      options[x] += F(" - (default)");
  }
  addFormSelector(F("I2C Address"), id, addressCount, options, addresses, NULL, selectedIndex, false);
}

void addFormSelector(const String& label, const String& id, int optionCount, const String options[], const int indices[], int selectedIndex)
{
  addFormSelector(label, id, optionCount, options, indices, NULL, selectedIndex, false);
}

void addFormSelector(const String& label, const String& id, int optionCount, const String options[], const int indices[], int selectedIndex, bool reloadonchange)
{
  addFormSelector(label, id, optionCount, options, indices, NULL, selectedIndex, reloadonchange);
}

void addFormSelector(const String& label, const String& id, int optionCount, const String options[], const int indices[], const String attr[], int selectedIndex, boolean reloadonchange)
{
  addRowLabel(label);
  addSelector(id, optionCount, options, indices, attr, selectedIndex, reloadonchange);
}

void addFormSelector_script(const String& label, const String& id, int optionCount, const String options[], const int indices[], const String attr[], int selectedIndex, const String& onChangeCall)
{
  addRowLabel(label);
  addSelector_Head(id, onChangeCall, false);
  addSelector_options(optionCount, options, indices, attr, selectedIndex);
  addSelector_Foot();
}

void addSelector(const String& id, int optionCount, const String options[], const int indices[], const String attr[], int selectedIndex, boolean reloadonchange) {
  addSelector(id, optionCount, options, indices, attr, selectedIndex, reloadonchange, true);
}

void addSelector(const String& id, int optionCount, const String options[], const int indices[], const String attr[], int selectedIndex, boolean reloadonchange, bool enabled)
{

  addSelector_Head(id, reloadonchange, !enabled);
  addSelector_options(optionCount, options, indices, attr, selectedIndex);
  addSelector_Foot();
}

void addSelector_options(int optionCount, const String options[], const int indices[], const String attr[], int selectedIndex)
{
  int index;
  for (byte x = 0; x < optionCount; x++)
  {
    if (indices)
      index = indices[x];
    else
      index = x;
    TXBuffer += F("<option value=");
    TXBuffer += index;
    if (selectedIndex == index)
      TXBuffer += F(" selected");
    if (attr)
    {
      TXBuffer += ' ';
      TXBuffer += attr[x];
    }
    TXBuffer += '>';
    TXBuffer += options[x];
    TXBuffer += F("</option>");
  }
}

void addSelector_Head(const String& id, boolean reloadonchange) {
  addSelector_Head(id, reloadonchange, false);
}

void addSelector_Head(const String& id, boolean reloadonchange, bool disabled)
{
  if (reloadonchange) {
    addSelector_Head(id, (const String) F("return dept_onchange(frmselect)"), disabled);
  } else {
    addSelector_Head(id, (const String) "", disabled);
  }
}

void addSelector_Head(const String& id, const String& onChangeCall, bool disabled)
{
  TXBuffer += F("<select class='wide' name='");
  TXBuffer += id;
  TXBuffer += F("' id='");
  TXBuffer += id;
  TXBuffer += '\'';
  if (disabled) {
    addDisabled();
  }
  if (onChangeCall.length() > 0) {
    TXBuffer += F(" onchange='");
    TXBuffer += onChangeCall;
    TXBuffer += '\'';
  }
  TXBuffer += '>';
}


void addSelector_Item(const String& option, int index, boolean selected, boolean disabled, const String& attr)
{
  TXBuffer += F("<option value=");
  TXBuffer += index;
  if (selected)
    TXBuffer += F(" selected");
  if (disabled)
    addDisabled();
  if (attr && attr.length() > 0)
  {
    TXBuffer += ' ';
    TXBuffer += attr;
  }
  TXBuffer += '>';
  TXBuffer += option;
  TXBuffer += F("</option>");
}


void addSelector_Foot()
{
  TXBuffer += F("</select>");
}


void addUnit(const String& unit)
{
  TXBuffer += F(" [");
  TXBuffer += unit;
  TXBuffer += "]";
}

void addRowLabel(const String& label)
{
  addRowLabel(label, "");
}

void addRowLabel(const String& label, const String& id)
{
  if (id.length() > 0) {
    TXBuffer += F("<TR id='");
    TXBuffer += id;
    TXBuffer += F("'><TD>");
  } else {
    html_TR_TD();
  }
  TXBuffer += label;
  TXBuffer += ':';
  html_TD();
}


void addRowLabel_copy(const String& label) {
  TXBuffer += F("<TR>");
  html_copyText_TD();
  TXBuffer += label;
  TXBuffer += ':';
  html_copyText_marker();
  html_copyText_TD();
}

void addRowLabelValue(LabelType::Enum label) {
  addRowLabel(getLabel(label));
  TXBuffer += getValue(label);
}

void addRowLabelValue_copy(LabelType::Enum label) {
  addRowLabel_copy(getLabel(label));
  TXBuffer += getValue(label);
}

void addButton(const String &url, const String &label) {
  addButton(url, label, "");
}

void addButton(const String &url, const String &label, const String& classes) {
  addButton(url, label, classes, true);
}

void addButton(const String &url, const String &label, const String& classes, bool enabled)
{
  html_add_button_prefix(classes, enabled);
  TXBuffer += url;
  TXBuffer += "'>";
  TXBuffer += label;
  TXBuffer += F("</a>");
}

void addButton(class StreamingBuffer &buffer, const String &url, const String &label)
{
  addButtonWithSvg(buffer, url, label, "", false);
}

void addButtonWithSvg(class StreamingBuffer &buffer, const String &url, const String &label, const String& svgPath, bool needConfirm) {
  bool hasSVG = svgPath.length() > 0;
  buffer += F("<a class='button link' href='");
  buffer += url;
  if (hasSVG) {
    buffer += F("' alt='");
    buffer += label;
  }
  if (needConfirm) {
    buffer += F("' onclick='return confirm(\"Are you sure?\")");
  }
  buffer += F("'>");
  if (hasSVG) {
    buffer += F("<svg width='24' height='24' viewBox='-1 -1 26 26' style='position: relative; top: 5px;'>");
    buffer += svgPath;
    buffer += F("</svg>");
  } else {
    buffer += label;
  }
  buffer += F("</a>");
}

void addSaveButton(const String &url, const String &label)
{
  addSaveButton(TXBuffer, url, label);
}

void addSaveButton(class StreamingBuffer &buffer, const String &url, const String &label)
{
#ifdef BUILD_MINIMAL_OTA
  addButtonWithSvg(buffer, url, label
     , ""
     , false);
#else
  addButtonWithSvg(buffer, url, label
     , F("<path d='M19 12v7H5v-7H3v7c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2v-7h-2zm-6 .67l2.59-2.58L17 11.5l-5 5-5-5 1.41-1.41L11 12.67V3h2v9.67z'  stroke='white' fill='white' ></path>")
     , false);
#endif
}

void addDeleteButton(const String &url, const String &label)
{
  addSaveButton(TXBuffer, url, label);
}

void addDeleteButton(class StreamingBuffer &buffer, const String &url, const String &label)
{
#ifdef BUILD_MINIMAL_OTA
  addButtonWithSvg(buffer, url, label
     , ""
     , true);
#else
  addButtonWithSvg(buffer, url, label
    , F("<path fill='none' d='M0 0h24v24H0V0z'></path><path d='M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM8 9h8v10H8V9zm7.5-5l-1-1h-5l-1 1H5v2h14V4h-3.5z' stroke='white' fill='white' ></path>")
    , true);
#endif
}

void addWideButton(const String &url, const String &label) {
  addWideButton(url, label, "", true);
}

void addWideButton(const String &url, const String &label, const String &classes) {
  addWideButton(url, label, classes, true);
}

void addWideButton(const String &url, const String &label, const String &classes, bool enabled)
{
  html_add_wide_button_prefix(classes, enabled);
  TXBuffer += url;
  TXBuffer += "'>";
  TXBuffer += label;
  TXBuffer += F("</a>");
}

void addSubmitButton()
{
  addSubmitButton(F("Submit"), "");
}


void addSubmitButton(const String &value, const String &name) {
  addSubmitButton(value, name, "");
}

void addSubmitButton(const String &value, const String &name, const String &classes)
{
  TXBuffer += F("<input class='button link");
  if (classes.length() > 0) {
    TXBuffer += ' ';
    TXBuffer += classes;
  }
  TXBuffer += F("' type='submit' value='");
  TXBuffer += value;
  if (name.length() > 0) {
    TXBuffer += F("' name='");
    TXBuffer += name;
  }
  TXBuffer += F("'><div id='toastmessage'></div><script type='text/javascript'>toasting();</script>");
}


void addCopyButton(const String &value, const String &delimiter, const String &name)
{
  TXBuffer += jsClipboardCopyPart1;
  TXBuffer += value;
  TXBuffer += jsClipboardCopyPart2;
  TXBuffer += delimiter;
  TXBuffer += jsClipboardCopyPart3;

  TXBuffer += F("<button class='button link' onclick='setClipboard()'>");
  TXBuffer += name;
  TXBuffer += " (";
  html_copyText_marker();
  TXBuffer += ')';
  TXBuffer += F("</button>");
}





void addTableSeparator(const String& label, int colspan, int h_size) {
  addTableSeparator(label, colspan, h_size, "");
}

void addTableSeparator(const String& label, int colspan, int h_size, const String& helpButton) {
  TXBuffer += F("<TR><TD colspan=");
  TXBuffer += colspan;
  TXBuffer += "><H";
  TXBuffer += h_size;
  TXBuffer += '>';
  TXBuffer += label;
  if (helpButton.length() > 0)
    addHelpButton(helpButton);
  TXBuffer += "</H";
  TXBuffer += h_size;
  TXBuffer += F("></TD></TR>");
}

void addFormHeader(const String& header, const String& helpButton)
{
  html_TR();
  html_table_header(header, helpButton, 225);
  html_table_header("");
}

void addFormHeader(const String& header)
{
  addFormHeader(header, "");
}





void addFormSubHeader(const String& header)
{
  addTableSeparator(header, 2, 3);
}





void addFormNote(const String& text)
{
  html_TR_TD();
  html_TD();
  TXBuffer += F("<div class='note'>Note: ");
  TXBuffer += text;
  TXBuffer += F("</div>");
}





void addFormSeparator(int clspan)
{
 TXBuffer += F("<TR><TD colspan='");
 TXBuffer += clspan;
 TXBuffer += F("'><hr>");
}





void addCheckBox(const String& id, boolean checked) {
  addCheckBox(id, checked, false);
}

void addCheckBox(const String& id, boolean checked, bool disabled)
{
  TXBuffer += F("<label class='container'>&nbsp;");
  TXBuffer += F("<input type='checkbox' id='");
  TXBuffer += id;
  TXBuffer += F("' name='");
  TXBuffer += id;
  TXBuffer += '\'';
  if (checked)
    TXBuffer += F(" checked");
  if (disabled) addDisabled();
  TXBuffer += F("><span class='checkmark");
  if (disabled) addDisabled();
  TXBuffer += F("'></span></label>");
}

void addFormCheckBox(const String& label, const String& id, boolean checked) {
  addFormCheckBox(label, id, checked, false);
}

void addFormCheckBox_disabled(const String& label, const String& id, boolean checked) {
  addFormCheckBox(label, id, checked, true);
}

void addFormCheckBox(const String& label, const String& id, boolean checked, bool disabled)
{
  addRowLabel(label);
  addCheckBox(id, checked, disabled);
}

void addFormCheckBox(LabelType::Enum label, boolean checked, bool disabled) {
  addFormCheckBox(getLabel(label), getInternalLabel(label), checked, disabled);
}

void addFormCheckBox_disabled(LabelType::Enum label, boolean checked) {
  addFormCheckBox(label, checked, true);
}




void addNumericBox(const String& id, int value, int min, int max)
{
  TXBuffer += F("<input class='widenumber' type='number' name='");
  TXBuffer += id;
  TXBuffer += '\'';
  if (min != INT_MIN)
  {
    TXBuffer += F(" min=");
    TXBuffer += min;
  }
  if (max != INT_MAX)
  {
    TXBuffer += F(" max=");
    TXBuffer += max;
  }
  TXBuffer += F(" value=");
  TXBuffer += value;
  TXBuffer += '>';
}

void addNumericBox(const String& id, int value)
{
  addNumericBox(id, value, INT_MIN, INT_MAX);
}

void addFormNumericBox(const String& label, const String& id, int value, int min, int max)
{
  addRowLabel(label);
  addNumericBox(id, value, min, max);
}

void addFormNumericBox(const String& label, const String& id, int value)
{
  addFormNumericBox(label, id, value, INT_MIN, INT_MAX);
}

void addFloatNumberBox(const String& id, float value, float min, float max)
{
  TXBuffer += F("<input type='number' name='");
  TXBuffer += id;
  TXBuffer += '\'';
  TXBuffer += F(" min=");
  TXBuffer += min;
  TXBuffer += F(" max=");
  TXBuffer += max;
  TXBuffer += F(" step=0.01");
  TXBuffer += F(" style='width:5em;' value=");
  TXBuffer += value;
  TXBuffer += '>';
}

void addFormFloatNumberBox(const String& label, const String& id, float value, float min, float max)
{
  addRowLabel(label);
  addFloatNumberBox(id, value, min, max);
}


void addTextBox(const String& id, const String& value, int maxlength)
{
  addTextBox(id, value, maxlength, false);
}

void addTextBox(const String& id, const String& value, int maxlength, bool readonly)
{
  addTextBox(id, value, maxlength, false, false, "");
}

void addTextBox(const String& id, const String& value, int maxlength, bool readonly, bool required)
{
  addTextBox(id, value, maxlength, false, false, "");
}

void addTextBox(const String& id, const String& value, int maxlength, bool readonly, bool required, const String& pattern)
{
  TXBuffer += F("<input class='wide' type='text' name='");
  TXBuffer += id;
  TXBuffer += F("' maxlength=");
  TXBuffer += maxlength;
  TXBuffer += F(" value='");
  TXBuffer += value;
  TXBuffer += '\'';
  if(readonly){
    TXBuffer += F(" readonly ");
  }
  if(required){
    TXBuffer += F(" required ");
  }
  if(pattern.length()>0){
    TXBuffer += F("pattern = '");
    TXBuffer += pattern;
    TXBuffer += '\'';
  }
  TXBuffer += '>';
}

void addFormTextBox(const String& label, const String& id, const String& value, int maxlength)
{
  addRowLabel(label);
  addTextBox(id, value, maxlength);
}

void addFormTextBox(const String& label, const String& id, const String& value, int maxlength, bool readonly)
{
  addRowLabel(label);
  addTextBox(id, value, maxlength, readonly);
}

void addFormTextBox(const String& label, const String& id, const String& value, int maxlength, bool readonly, bool required)
{
  addRowLabel(label);
  addTextBox(id, value, maxlength, readonly, required);
}

void addFormTextBox(const String& label, const String& id, const String& value, int maxlength, bool readonly, bool required, const String& pattern)
{
  addRowLabel(label);
  addTextBox(id, value, maxlength, readonly, required, pattern);
}


void addFormPasswordBox(const String& label, const String& id, const String& password, int maxlength)
{
  addRowLabel(label);
  TXBuffer += F("<input class='wide' type='password' name='");
  TXBuffer += id;
  TXBuffer += F("' maxlength=");
  TXBuffer += maxlength;
  TXBuffer += F(" value='");
  if (password != "")
    TXBuffer += F("*****");

  TXBuffer += "'>";
}

void copyFormPassword(const String& id, char* pPassword, int maxlength)
{
  String password = WebServer.arg(id);
  if (password == F("*****"))
    return;
  safe_strncpy(pPassword, password.c_str(), maxlength);
}

void addFormIPBox(const String& label, const String& id, const byte ip[4])
{
  bool empty_IP =(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);

  addRowLabel(label);
  TXBuffer += F("<input class='wide' type='text' name='");
  TXBuffer += id;
  TXBuffer += F("' value='");
  if (!empty_IP){
    TXBuffer += formatIP(ip);
  }
  TXBuffer += "'>";
}



void addHelpButton(const String& url) {
  if (url.startsWith("RTD")) {
    addRTDHelpButton(url.substring(3));
  } else {
    addHelpButton(url, false);
  }
}

void addRTDHelpButton(const String& url)
{
  addHelpButton(url, true);
}

void addHelpButton(const String& url, bool isRTD)
{
  addHtmlLink(
    F("button help"),
    makeDocLink(url, isRTD),
    isRTD ? F("&#8505;") : F("&#10068;"));
}

void addRTDPluginButton(int taskDeviceNumber) {
  String url;
  url.reserve(16);
  url = F("Plugin/P");
  if (taskDeviceNumber < 100) url += '0';
  if (taskDeviceNumber < 10) url += '0';
  url += String(taskDeviceNumber);
  url += F(".html");
  addRTDHelpButton(url);

  switch (taskDeviceNumber) {
    case 76:
    case 77:
      addHtmlLink(
        F("button help"),
        makeDocLink(F("Reference/Safety.html"), true),
        F("&#9889;"));
      break;

  }
}

String makeDocLink(const String& url, bool isRTD) {
  String result;
  if (!url.startsWith(F("http"))) {
    if (isRTD) {
      result += F("https://espeasy.readthedocs.io/en/latest/");
    } else {
      result += F("http://www.letscontrolit.com/wiki/index.php/");
    }
  }
  result += url;
  return result;
}

void addHtmlLink(const String& htmlclass, const String& url, const String& label) {
  TXBuffer += F(" <a class='");
  TXBuffer += htmlclass;
  TXBuffer += F("' href='");
  TXBuffer += url;
  TXBuffer += F("' target='_blank'>");
  TXBuffer += label;
  TXBuffer += F("</a>");
}

void addEnabled(boolean enabled)
{
  TXBuffer += F("<span class='enabled ");
  if (enabled)
    TXBuffer += F("on'>&#10004;");
  else
    TXBuffer += F("off'>&#10060;");
  TXBuffer += F("</span>");
}






void wrap_html_tag(const String& tag, const String& text) {
  TXBuffer += '<';
  TXBuffer += tag;
  TXBuffer += '>';
  TXBuffer += text;
  TXBuffer += "</";
  TXBuffer += tag;
  TXBuffer += '>';
}

void html_B(const String& text) {
  wrap_html_tag("b", text);
}

void html_I(const String& text) {
  wrap_html_tag("i", text);
}

void html_U(const String& text) {
  wrap_html_tag("u", text);
}

void html_TR_TD_highlight() {
  TXBuffer += F("<TR class=\"highlight\">");
  html_TD();
}

void html_TR_TD() {
  html_TR();
  html_TD();
}

void html_BR() {
  TXBuffer += F("<BR>");
}

void html_TR() {
  TXBuffer += F("<TR>");
}

void html_TR_TD_height(int height) {
  html_TR();
  TXBuffer += F("<TD HEIGHT=\"");
  TXBuffer += height;
  TXBuffer += "\">";
}

void html_TD() {
  html_TD(1);
}

void html_TD(int td_cnt) {
  for (int i = 0; i < td_cnt; ++i) {
    TXBuffer += F("<TD>");
  }
}

static int copyTextCounter = 0;

void html_reset_copyTextCounter() {
  copyTextCounter = 0;
}

void html_copyText_TD() {
  ++copyTextCounter;
  TXBuffer += F("<TD id='copyText_");
  TXBuffer += copyTextCounter;
  TXBuffer += "'>";
}


void html_copyText_marker() {
  TXBuffer += F("&#x022C4;");
}

void html_add_estimate_symbol() {
  TXBuffer += F(" &#8793; ");
}

void html_table_class_normal() {
  html_table(F("normal"));
}

void html_table_class_multirow() {
  html_table(F("multirow"), true);
}

void html_table_class_multirow_noborder() {
  html_table(F("multirow"), false);
}

void html_table(const String& tableclass) {
  html_table(tableclass, false);
}

void html_table(const String& tableclass, bool boxed) {
  TXBuffer += F("<table class='");
  TXBuffer += tableclass;
  TXBuffer += '\'';
  if (boxed) {
    TXBuffer += F("' border=1px frame='box' rules='all'");
  }
  TXBuffer += '>';
}

void html_table_header(const String& label) {
  html_table_header(label, 0);
}

void html_table_header(const String& label, int width) {
  html_table_header(label, "", width);
}

void html_table_header(const String& label, const String& helpButton, int width) {
  TXBuffer += F("<TH");
  if (width > 0) {
    TXBuffer += F(" style='width:");
    TXBuffer += String(width);
    TXBuffer += F("px;'");
  }
  TXBuffer += '>';
  TXBuffer += label;
  if (helpButton.length() > 0)
    addHelpButton(helpButton);
  TXBuffer += F("</TH>");
}

void html_end_table() {
  TXBuffer += F("</table>");
}

void html_end_form() {
  TXBuffer += F("</form>");
}

void html_add_button_prefix() {
  html_add_button_prefix("", true);
}

void html_add_button_prefix(const String& classes, bool enabled) {
  TXBuffer += F(" <a class='button link");
  if (classes.length() > 0) {
    TXBuffer += ' ';
    TXBuffer += classes;
  }
  if (!enabled) {
    addDisabled();
  }
  TXBuffer += '\'';
  if (!enabled) {
    addDisabled();
  }
  TXBuffer += F(" href='");
}

void html_add_wide_button_prefix() {
  html_add_wide_button_prefix("", true);
}

void html_add_wide_button_prefix(const String& classes, bool enabled) {
  String wide_classes;
  wide_classes.reserve(classes.length() + 5);
  wide_classes = F("wide ");
  wide_classes += classes;
  html_add_button_prefix(wide_classes, enabled);
}

void html_add_form() {
  TXBuffer += F("<form name='frmselect' method='post'>");
}


void html_add_autosubmit_form() {
  TXBuffer += F("<script><!--\n"
           "function dept_onchange(frmselect) {frmselect.submit();}"
           "\n//--></script>");
}

void html_add_script(const String& script, bool defer) {
  html_add_script(defer);
  addHtml(script);
  html_add_script_end();
}

void html_add_script(bool defer) {
  TXBuffer += F("<script");
  if (defer) {
    TXBuffer += F(" defer");
  }
  TXBuffer += F(" type='text/JavaScript'>");
}

void html_add_script_end() {
  TXBuffer += F("</script>");
}





void addTaskSelect(const String& name, int choice)
{
  String deviceName;

  TXBuffer += F("<select id='selectwidth' name='");
  TXBuffer += name;
  TXBuffer += F("' onchange='return dept_onchange(frmselect)'>");

  for (byte x = 0; x < TASKS_MAX; x++)
  {
    deviceName = "";
    if (Settings.TaskDeviceNumber[x] != 0 )
    {
      byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[x]);

      if (Plugin_id[DeviceIndex] != 0)
        deviceName = getPluginNameFromDeviceIndex(DeviceIndex);
    }
    LoadTaskSettings(x);
    TXBuffer += F("<option value='");
    TXBuffer += x;
    TXBuffer += '\'';
    if (choice == x)
      TXBuffer += F(" selected");
    if (Settings.TaskDeviceNumber[x] == 0)
      addDisabled();
    TXBuffer += '>';
    TXBuffer += x + 1;
    TXBuffer += F(" - ");
    TXBuffer += deviceName;
    TXBuffer += F(" - ");
    TXBuffer += ExtraTaskSettings.TaskDeviceName;
    TXBuffer += F("</option>");
  }
}






void addTaskValueSelect(const String& name, int choice, byte TaskIndex)
{
  TXBuffer += F("<select id='selectwidth' name='");
  TXBuffer += name;
  TXBuffer += "'>";

  byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[TaskIndex]);

  for (byte x = 0; x < Device[DeviceIndex].ValueCount; x++)
  {
    TXBuffer += F("<option value='");
    TXBuffer += x;
    TXBuffer += '\'';
    if (choice == x)
      TXBuffer += F(" selected");
    TXBuffer += '>';
    TXBuffer += ExtraTaskSettings.TaskDeviceValueNames[x];
    TXBuffer += F("</option>");
  }
}






void handle_log() {
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  html_table_class_normal();
  TXBuffer += F("<TR><TH id=\"headline\" align=\"left\">Log");
  addCopyButton(F("copyText"), "", F("Copy log to clipboard"));
  TXBuffer += F("</TR></table><div  id='current_loglevel' style='font-weight: bold;'>Logging: </div><div class='logviewer' id='copyText_1'></div>");
  TXBuffer += F("Autoscroll: ");
  addCheckBox(F("autoscroll"), true);
  TXBuffer += F("<BR></body>");

  html_add_script(true);
  TXBuffer += DATA_FETCH_AND_PARSE_LOG_JS;
  html_add_script_end();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
  }




void handle_log_JSON() {
  if (!isLoggedIn()) return;
  TXBuffer.startJsonStream();
  String webrequest = WebServer.arg(F("view"));
  TXBuffer += F("{\"Log\": {");
  if (webrequest == F("legend")) {
    TXBuffer += F("\"Legend\": [");
    for (byte i = 0; i < LOG_LEVEL_NRELEMENTS; ++i) {
      if (i != 0)
        TXBuffer += ',';
      TXBuffer += '{';
      int loglevel;
      stream_next_json_object_value(F("label"), getLogLevelDisplayStringFromIndex(i, loglevel));
      stream_last_json_object_value(F("loglevel"), String(loglevel));
    }
    TXBuffer += F("],\n");
  }
  TXBuffer += F("\"Entries\": [");
  bool logLinesAvailable = true;
  int nrEntries = 0;
  unsigned long firstTimeStamp = 0;
  unsigned long lastTimeStamp = 0;
  while (logLinesAvailable) {
    String reply = Logging.get_logjson_formatted(logLinesAvailable, lastTimeStamp);
    if (reply.length() > 0) {
      TXBuffer += reply;
      if (nrEntries == 0) {
        firstTimeStamp = lastTimeStamp;
      }
      ++nrEntries;
    }

  }
  TXBuffer += F("],\n");
  long logTimeSpan = timeDiff(firstTimeStamp, lastTimeStamp);
  long refreshSuggestion = 1000;
  long newOptimum = 1000;
  if (nrEntries > 2 && logTimeSpan > 1) {


    newOptimum = logTimeSpan * (LOG_STRUCT_MESSAGE_LINES / 2);
    newOptimum = newOptimum / (nrEntries - 1);
  }
  if (newOptimum < refreshSuggestion) refreshSuggestion = newOptimum;
  if (refreshSuggestion < 100) {

    refreshSuggestion = 100;
  }
  stream_next_json_object_value(F("TTL"), String(refreshSuggestion));
  stream_next_json_object_value(F("timeHalfBuffer"), String(newOptimum));
  stream_next_json_object_value(F("nrEntries"), String(nrEntries));
  stream_next_json_object_value(F("SettingsWebLogLevel"), String(Settings.WebLogLevel));
  stream_last_json_object_value(F("logTimeSpan"), String(logTimeSpan));
  TXBuffer += F("}\n");
  TXBuffer.endStream();
  updateLogLevelCache();
}




void addWideButtonPlusDescription(const String& url, const String& buttonText, const String& description)
{
  html_TR_TD_height(30);
  addWideButton(url, buttonText);
  html_TD();
  TXBuffer += description;
}

void handle_tools() {
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  String webrequest = WebServer.arg(F("cmd"));

  TXBuffer += F("<form>");
  html_table_class_normal();

  addFormHeader(F("Tools"));

  addFormSubHeader(F("Command"));
  html_TR_TD();
  TXBuffer += F("<TR><TD style='width: 180px'>");
  TXBuffer += F("<input class='wide' type='text' name='cmd' value='");
  TXBuffer += webrequest;
  TXBuffer += "'>";
  html_TD();
  addSubmitButton();
  addHelpButton(F("ESPEasy_Command_Reference"));
  html_TR_TD();

  printToWeb = true;
  printWebString = "";

  if (webrequest.length() > 0)
  {
    struct EventStruct TempEvent;
    webrequest=parseTemplate(webrequest,webrequest.length());
    parseCommandString(&TempEvent, webrequest);
    TempEvent.Source = VALUE_SOURCE_WEB_FRONTEND;
    if (!PluginCall(PLUGIN_WRITE, &TempEvent, webrequest))
      ExecuteCommand(VALUE_SOURCE_WEB_FRONTEND, webrequest.c_str());
  }

  if (printWebString.length() > 0)
  {
    TXBuffer += F("<TR><TD colspan='2'>Command Output<BR><textarea readonly rows='10' wrap='on'>");
    TXBuffer += printWebString;
    TXBuffer += F("</textarea>");
  }

  addFormSubHeader(F("System"));

  addWideButtonPlusDescription(F("/?cmd=reboot"), F("Reboot"), F("Reboots ESP"));
  addWideButtonPlusDescription(F("log"), F("Log"), F("Open log output"));
  addWideButtonPlusDescription(F("sysinfo"), F("Info"), F("Open system info page"));
  addWideButtonPlusDescription(F("advanced"), F("Advanced"), F("Open advanced settings"));
  addWideButtonPlusDescription(F("json"), F("Show JSON"), F("Open JSON output"));
  #ifdef WEBSERVER_TIMINGSTATS
  addWideButtonPlusDescription(F("timingstats"), F("Timing stats"), F("Open timing statistics of system"));
  #endif
  addWideButtonPlusDescription(F("pinstates"), F("Pin state buffer"), F("Show Pin state buffer"));
  addWideButtonPlusDescription(F("sysvars"), F("System Variables"), F("Show all system variables and conversions"));

  addFormSubHeader(F("Wifi"));

  addWideButtonPlusDescription(F("/?cmd=wificonnect"), F("Connect"), F("Connects to known Wifi network"));
  addWideButtonPlusDescription(F("/?cmd=wifidisconnect"), F("Disconnect"), F("Disconnect from wifi network"));
  addWideButtonPlusDescription(F("wifiscanner"), F("Scan"), F("Scan for wifi networks"));

  addFormSubHeader(F("Interfaces"));

  addWideButtonPlusDescription(F("i2cscanner"), F("I2C Scan"), F("Scan for I2C devices"));

  addFormSubHeader(F("Settings"));

  addWideButtonPlusDescription(F("upload"), F("Load"), F("Loads a settings file"));
  addFormNote(F("(File MUST be renamed to \"config.dat\" before upload!)"));
  addWideButtonPlusDescription(F("download"), F("Save"), F("Saves a settings file"));

#ifdef WEBSERVER_NEW_UI
  #if defined(ESP8266)
    if ((SpiffsFreeSpace() / 1024) > 50) {
      TXBuffer += F("<TR><TD>");
      TXBuffer += F("<script>function downloadUI() { fetch('https://raw.githubusercontent.com/letscontrolit/espeasy_ui/master/build/index.htm.gz').then(r=>r.arrayBuffer()).then(r => {var f=new FormData();f.append('file', new File([new Blob([new Uint8Array(r)])], 'index.htm.gz'));f.append('edit', 1);fetch('/upload',{method:'POST',body:f}).then(() => {window.location.href='/';});}); }</script>");
      TXBuffer += F("<a class=\"button link wide\" onclick=\"downloadUI()\">Download new UI</a>");
      TXBuffer += F("</TD><TD>Download new UI(alpha)</TD></TR>");
    }
  #endif
#endif

#if defined(ESP8266)
  {
    {
      uint32_t maxSketchSize;
      bool use2step;
      bool otaEnabled = OTA_possible(maxSketchSize, use2step);
      addFormSubHeader(F("Firmware"));
      html_TR_TD_height(30);
      addWideButton(F("update"), F("Update Firmware"), "", otaEnabled);
      addHelpButton(F("EasyOTA"));
      html_TD();
      TXBuffer += F("Load a new firmware");
      if (otaEnabled) {
        if (use2step) {
          TXBuffer += F(" <b>WARNING</b> only use 2-step OTA update.");
        }
      } else {
        TXBuffer += F(" <b>WARNING</b> OTA not possible.");
      }
      TXBuffer += F(" Max sketch size: ");
      TXBuffer += maxSketchSize / 1024;
      TXBuffer += F(" kB");
    }
  }
#endif

  addFormSubHeader(F("Filesystem"));

  addWideButtonPlusDescription(F("filelist"), F("File browser"), F("Show files on internal flash file system"));
  addWideButtonPlusDescription(F("/factoryreset"), F("Factory Reset"), F("Select pre-defined configuration or full erase of settings"));
#ifdef FEATURE_SD
  addWideButtonPlusDescription(F("SDfilelist"), F("SD Card"), F("Show files on SD-Card"));
#endif

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb = false;
}

#ifdef WEBSERVER_NEW_UI



void handle_pinstates_json() {
  checkRAM(F("handle_pinstates"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();

  bool comma_between = false;
  TXBuffer += F("[{");
  for (std::map<uint32_t,portStatusStruct>::iterator it=globalMapPortStatus.begin(); it!=globalMapPortStatus.end(); ++it)
  {
    if( comma_between ) {
      TXBuffer += ",{";
    } else {
      comma_between=true;
    }

    const uint16_t plugin = getPluginFromKey(it->first);
    const uint16_t port = getPortFromKey(it->first);

    stream_next_json_object_value(F("plugin"), String(plugin));
    stream_next_json_object_value(F("port"), String(port));
    stream_next_json_object_value(F("state"), String(it->second.state));
    stream_next_json_object_value(F("task"), String(it->second.task));
    stream_next_json_object_value(F("monitor"), String(it->second.monitor));
    stream_next_json_object_value(F("command"), String(it->second.command));
    stream_last_json_object_value(F("init"), String(it->second.init));
  }

  TXBuffer += F("]");
# 3997 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
    TXBuffer.endStream();
}
#endif

void handle_pinstates() {
  checkRAM(F("handle_pinstates"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);



  html_table_class_multirow();
  html_TR();
  html_table_header(F("Plugin"), F("Official_plugin_list"), 0);
  html_table_header("GPIO");
  html_table_header("Mode");
  html_table_header(F("Value/State"));
  html_table_header(F("Task"));
  html_table_header(F("Monitor"));
  html_table_header(F("Command"));
  html_table_header("Init");
  for (std::map<uint32_t,portStatusStruct>::iterator it=globalMapPortStatus.begin(); it!=globalMapPortStatus.end(); ++it)
  {
    html_TR_TD(); TXBuffer += "P";
    const uint16_t plugin = getPluginFromKey(it->first);
    const uint16_t port = getPortFromKey(it->first);

    if (plugin < 100)
    {
      TXBuffer += '0';
    }
    if (plugin < 10)
    {
      TXBuffer += '0';
    }
    TXBuffer += plugin;
    html_TD();
    TXBuffer += port;
    html_TD();
    TXBuffer += getPinModeString(it->second.mode);
    html_TD();
    TXBuffer += it->second.state;
    html_TD();
    TXBuffer += it->second.task;
    html_TD();
    TXBuffer += it->second.monitor;
    html_TD();
    TXBuffer += it->second.command;
    html_TD();
    TXBuffer += it->second.init;
  }
# 4079 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
    html_end_table();
    sendHeadandTail_stdtemplate(_TAIL);
    TXBuffer.endStream();
}

#ifdef WEBSERVER_NEW_UI



void handle_i2cscanner_json() {
  checkRAM(F("handle_i2cscanner"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();
  TXBuffer += "[{";

  bool firstentry = true;
  byte error, address;
  for (address = 1; address <= 127; address++ )
  {
    if (firstentry) {
      firstentry = false;
    } else {
      TXBuffer += ",{";
    }

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    stream_next_json_object_value(F("addr"), String(formatToHex(address)));
    stream_last_json_object_value(F("status"), String(error));
  }
  TXBuffer += "]";
  TXBuffer.endStream();
}
#endif

void handle_i2cscanner() {
  checkRAM(F("handle_i2cscanner"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  html_table_class_multirow();
  html_table_header(F("I2C Addresses in use"));
  html_table_header(F("Supported devices"));

  byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 1; address <= 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      TXBuffer += "<TR><TD>";
      TXBuffer += formatToHex(address);
      TXBuffer += "<TD>";
      switch (address)
      {
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x25:
        case 0x26:
        case 0x27:
          TXBuffer += F("PCF8574<BR>MCP23017<BR>LCD");
          break;
        case 0x23:
          TXBuffer += F("PCF8574<BR>MCP23017<BR>LCD<BR>BH1750");
          break;
        case 0x24:
          TXBuffer += F("PCF8574<BR>MCP23017<BR>LCD<BR>PN532");
          break;
        case 0x29:
          TXBuffer += F("TSL2561");
          break;
        case 0x38:
        case 0x3A:
        case 0x3B:
        case 0x3E:
        case 0x3F:
          TXBuffer += F("PCF8574A");
          break;
        case 0x39:
          TXBuffer += F("PCF8574A<BR>TSL2561<BR>APDS9960");
          break;
        case 0x3C:
        case 0x3D:
          TXBuffer += F("PCF8574A<BR>OLED");
          break;
        case 0x40:
          TXBuffer += F("SI7021<BR>HTU21D<BR>INA219<BR>PCA9685");
          break;
        case 0x41:
        case 0x42:
        case 0x43:
          TXBuffer += F("INA219");
          break;
        case 0x44:
        case 0x45:
          TXBuffer += F("SHT30/31/35");
          break;
        case 0x48:
        case 0x4A:
        case 0x4B:
          TXBuffer += F("PCF8591<BR>ADS1115<BR>LM75A");
          break;
        case 0x49:
          TXBuffer += F("PCF8591<BR>ADS1115<BR>TSL2561<BR>LM75A");
          break;
        case 0x4C:
        case 0x4E:
        case 0x4F:
          TXBuffer += F("PCF8591<BR>LM75A");
          break;
        case 0x4D:
          TXBuffer += F("PCF8591<BR>MCP3221<BR>LM75A");
          break;
        case 0x5A:
          TXBuffer += F("MLX90614<BR>MPR121");
          break;
        case 0x5B:
          TXBuffer += F("MPR121");
          break;
        case 0x5C:
          TXBuffer += F("DHT12<BR>AM2320<BR>BH1750<BR>MPR121");
          break;
        case 0x5D:
          TXBuffer += F("MPR121");
          break;
        case 0x60:
          TXBuffer += F("Adafruit Motorshield v2<BR>SI1145");
          break;
        case 0x70:
          TXBuffer += F("Adafruit Motorshield v2 (Catchall)<BR>HT16K33");
          break;
        case 0x71:
        case 0x72:
        case 0x73:
        case 0x74:
        case 0x75:
          TXBuffer += F("HT16K33");
          break;
        case 0x76:
          TXBuffer += F("BME280<BR>BMP280<BR>MS5607<BR>MS5611<BR>HT16K33");
          break;
        case 0x77:
          TXBuffer += F("BMP085<BR>BMP180<BR>BME280<BR>BMP280<BR>MS5607<BR>MS5611<BR>HT16K33");
          break;
        case 0x7f:
          TXBuffer += F("Arduino PME");
          break;
      }
      nDevices++;
    }
    else if (error == 4)
    {
      html_TR_TD(); TXBuffer += F("Unknown error at address ");
      TXBuffer += formatToHex(address);
    }
  }

  if (nDevices == 0)
    TXBuffer += F("<TR>No I2C devices found");

  html_end_table();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

#ifdef WEBSERVER_NEW_UI



void handle_wifiscanner_json() {
  checkRAM(F("handle_wifiscanner"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();
  TXBuffer += "[{";
  bool firstentry = true;
  int n = WiFi.scanNetworks(false, true);
  for (int i = 0; i < n; ++i)
  {
    if (firstentry) firstentry = false;
    else TXBuffer += ",{";

    stream_next_json_object_value(getLabel(LabelType::SSID), WiFi.SSID(i));
    stream_next_json_object_value(getLabel(LabelType::BSSID), WiFi.BSSIDstr(i));
    stream_next_json_object_value(getLabel(LabelType::CHANNEL), String(WiFi.channel(i)));
    stream_next_json_object_value(getLabel(LabelType::WIFI_RSSI), String(WiFi.RSSI(i)));
    String authType;
    switch (WiFi.encryptionType(i)) {
    #ifdef ESP32
      case WIFI_AUTH_OPEN: authType = F("open"); break;
      case WIFI_AUTH_WEP: authType = F("WEP"); break;
      case WIFI_AUTH_WPA_PSK: authType = F("WPA/PSK"); break;
      case WIFI_AUTH_WPA2_PSK: authType = F("WPA2/PSK"); break;
      case WIFI_AUTH_WPA_WPA2_PSK: authType = F("WPA/WPA2/PSK"); break;
      case WIFI_AUTH_WPA2_ENTERPRISE: authType = F("WPA2 Enterprise"); break;
    #else
      case ENC_TYPE_WEP: authType = F("WEP"); break;
      case ENC_TYPE_TKIP: authType = F("WPA/PSK"); break;
      case ENC_TYPE_CCMP: authType = F("WPA2/PSK"); break;
      case ENC_TYPE_NONE: authType = F("open"); break;
      case ENC_TYPE_AUTO: authType = F("WPA/WPA2/PSK"); break;
    #endif
      default:
        break;
    }
    if (authType.length() > 0) {
      stream_last_json_object_value(F("auth"), authType);
    }
  }
  TXBuffer += "]";
  TXBuffer.endStream();
}
#endif

void handle_wifiscanner() {
  checkRAM(F("handle_wifiscanner"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_table_class_multirow();
  html_TR();
  html_table_header(getLabel(LabelType::SSID));
  html_table_header(getLabel(LabelType::BSSID));
  html_table_header("info");

  int n = WiFi.scanNetworks(false, true);
  if (n == 0)
    TXBuffer += F("No Access Points found");
  else
  {
    for (int i = 0; i < n; ++i)
    {
      html_TR_TD();
      TXBuffer += formatScanResult(i, "<TD>");
    }
  }

  html_end_table();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}





void handle_login() {
  checkRAM(F("handle_login"));
  if (!clientIPallowed()) return;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  String webrequest = WebServer.arg(F("password"));
  TXBuffer += F("<form method='post'>");
  html_table_class_normal();
  TXBuffer += F("<TR><TD>Password<TD>");
  TXBuffer += F("<input class='wide' type='password' name='password' value='");
  TXBuffer += webrequest;
  TXBuffer += "'>";
  html_TR_TD();
  html_TD();
  addSubmitButton();
  html_TR_TD();
  html_end_table();
  html_end_form();

  if (webrequest.length() != 0)
  {
    char command[80];
    command[0] = 0;
    webrequest.toCharArray(command, 80);


    if ((strcasecmp(command, SecuritySettings.Password) == 0) || (SecuritySettings.Password[0] == 0))
    {
      WebLoggedIn = true;
      WebLoggedInTimer = 0;
      TXBuffer = F("<script>window.location = '.'</script>");
    }
    else
    {
      TXBuffer += F("Invalid password!");
      if (Settings.UseRules)
      {
        String event = F("Login#Failed");
        rulesProcessing(event);
      }
    }
  }

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb = false;
}





void handle_control() {
  checkRAM(F("handle_control"));
  if (!clientIPallowed()) return;


  String webrequest = WebServer.arg(F("cmd"));


  String command = parseString(webrequest, 1);
  addLog(LOG_LEVEL_INFO,String(F("HTTP: ")) + webrequest);
  webrequest=parseTemplate(webrequest,webrequest.length());
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG,String(F("HTTP after parseTemplate: ")) + webrequest);
#endif

  bool handledCmd = false;
  if (command == F("event"))
  {
    eventBuffer = webrequest.substring(6);
    handledCmd = true;
  }
  else if (command.equalsIgnoreCase(F("taskrun")) ||
           command.equalsIgnoreCase(F("taskvalueset")) ||
           command.equalsIgnoreCase(F("taskvaluetoggle")) ||
           command.equalsIgnoreCase(F("let")) ||
           command.equalsIgnoreCase(F("logPortStatus")) ||
           command.equalsIgnoreCase(F("jsonportstatus")) ||
           command.equalsIgnoreCase(F("rules"))) {
    ExecuteCommand(VALUE_SOURCE_HTTP,webrequest.c_str());
    handledCmd = true;
  }

  if (handledCmd) {
    TXBuffer.startStream("*");
    TXBuffer += "OK";
    TXBuffer.endStream();
   return;
  }

  struct EventStruct TempEvent;
  parseCommandString(&TempEvent, webrequest);
  TempEvent.Source = VALUE_SOURCE_HTTP;

  printToWeb = true;
  printWebString = "";

  bool unknownCmd = false;
  if (PluginCall(PLUGIN_WRITE, &TempEvent, webrequest));
  else if (remoteConfig(&TempEvent, webrequest));
  else unknownCmd = true;

  if (printToWebJSON)
    TXBuffer.startJsonStream();
  else
    TXBuffer.startStream();

  if (unknownCmd)
 TXBuffer += F("Unknown or restricted command!");
  else
 TXBuffer += printWebString;

  TXBuffer.endStream();

  printWebString = "";
  printToWeb = false;
  printToWebJSON = false;
}





void stream_to_json_value(const String& value) {
  if (value.length() == 0 || !isFloat(value)) {
    TXBuffer += '\"';
    TXBuffer += value;
    TXBuffer += '\"';
  } else {
    TXBuffer += value;
  }
}

void stream_to_json_object_value(const String& object, const String& value) {
  TXBuffer += '\"';
  TXBuffer += object;
  TXBuffer += "\":";
  stream_to_json_value(value);
}

String jsonBool(bool value) {
  return toString(value);
}


void stream_next_json_object_value(const String& object, const String& value) {
  TXBuffer += to_json_object_value(object, value);
  TXBuffer += ",\n";
}


void stream_last_json_object_value(const String& object, const String& value) {
  TXBuffer += to_json_object_value(object, value);
  TXBuffer += "\n}";
}

void stream_next_json_object_value(LabelType::Enum label) {
  stream_next_json_object_value(getLabel(label), getValue(label));
}

void stream_last_json_object_value(LabelType::Enum label) {
  stream_last_json_object_value(getLabel(label), getValue(label));
}





void handle_json()
{
  const int taskNr = getFormItemInt(F("tasknr"), -1);
  const bool showSpecificTask = taskNr > 0;
  bool showSystem = true;
  bool showWifi = true;
  bool showDataAcquisition = true;
  bool showTaskDetails = true;
  bool showNodes = true;
  {
    String view = WebServer.arg("view");
    if (view.length() != 0) {
      if (view == F("sensorupdate")) {
        showSystem = false;
        showWifi = false;
        showDataAcquisition = false;
        showTaskDetails = false;
        showNodes =false;
      }
    }
  }
  TXBuffer.startJsonStream();
  if (!showSpecificTask)
  {
    TXBuffer += '{';
    if (showSystem) {
      TXBuffer += F("\"System\":{\n");
      stream_next_json_object_value(LabelType::BUILD_DESC);
      stream_next_json_object_value(LabelType::GIT_BUILD);
      stream_next_json_object_value(LabelType::SYSTEM_LIBRARIES);
      stream_next_json_object_value(LabelType::PLUGINS);
      stream_next_json_object_value(LabelType::PLUGIN_DESCRIPTION);
      stream_next_json_object_value(LabelType::LOCAL_TIME);
      stream_next_json_object_value(LabelType::UNIT_NR);
      stream_next_json_object_value(LabelType::UNIT_NAME);
      stream_next_json_object_value(LabelType::UPTIME);
      stream_next_json_object_value(LabelType::BOOT_TYPE);
      stream_next_json_object_value(LabelType::RESET_REASON);

      if (wdcounter > 0)
      {
          stream_next_json_object_value(LabelType::LOAD_PCT);
          stream_next_json_object_value(LabelType::LOOP_COUNT);
      }
      stream_next_json_object_value(LabelType::CPU_ECO_MODE);

      #ifdef CORE_POST_2_5_0
      stream_next_json_object_value(LabelType::HEAP_MAX_FREE_BLOCK);
      stream_next_json_object_value(LabelType::HEAP_FRAGMENTATION);
      #endif
      stream_last_json_object_value(LabelType::FREE_MEM);
      TXBuffer += ",\n";
    }
    if (showWifi) {
      TXBuffer += F("\"WiFi\":{\n");
      #if defined(ESP8266)
        stream_next_json_object_value(LabelType::HOST_NAME);
      #endif
      stream_next_json_object_value(LabelType::IP_CONFIG);
      stream_next_json_object_value(LabelType::IP_ADDRESS);
      stream_next_json_object_value(LabelType::IP_SUBNET);
      stream_next_json_object_value(LabelType::GATEWAY);
      stream_next_json_object_value(LabelType::STA_MAC);
      stream_next_json_object_value(LabelType::DNS_1);
      stream_next_json_object_value(LabelType::DNS_1);
      stream_next_json_object_value(LabelType::SSID);
      stream_next_json_object_value(LabelType::BSSID);
      stream_next_json_object_value(LabelType::CHANNEL);
      stream_next_json_object_value(LabelType::CONNECTED_MSEC);
      stream_next_json_object_value(LabelType::LAST_DISCONNECT_REASON);
      stream_next_json_object_value(LabelType::LAST_DISC_REASON_STR);
      stream_next_json_object_value(LabelType::NUMBER_RECONNECTS);
      stream_next_json_object_value(LabelType::FORCE_WIFI_BG);
      stream_next_json_object_value(LabelType::RESTART_WIFI_LOST_CONN);
#ifdef ESP8266
      stream_next_json_object_value(LabelType::FORCE_WIFI_NOSLEEP);
#endif
#ifdef SUPPORT_ARP
      stream_next_json_object_value(LabelType::PERIODICAL_GRAT_ARP);
#endif
      stream_next_json_object_value(LabelType::CONNECTION_FAIL_THRESH);
      stream_last_json_object_value(LabelType::WIFI_RSSI);
      TXBuffer += ",\n";
    }
    if(showNodes) {
      bool comma_between=false;
      for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
      {
        if (it->second.ip[0] != 0)
        {
          if( comma_between ) {
            TXBuffer += ',';
          } else {
            comma_between=true;
            TXBuffer += F("\"nodes\":[\n");
          }

          TXBuffer += '{';
          stream_next_json_object_value(F("nr"), String(it->first));
          stream_next_json_object_value(F("name"),
              (it->first != Settings.Unit) ? it->second.nodeName : Settings.Name);

          if (it->second.build) {
            stream_next_json_object_value(F("build"), String(it->second.build));
          }

          if (it->second.nodeType) {
            String platform = getNodeTypeDisplayString(it->second.nodeType);
            if (platform.length() > 0)
              stream_next_json_object_value(F("platform"), platform);
          }
          stream_next_json_object_value(F("ip"), it->second.ip.toString());
          stream_last_json_object_value(F("age"), String( it->second.age ));
        }
      }
      if(comma_between) {
        TXBuffer += F("],\n");
      }
    }
  }

  byte firstTaskIndex = 0;
  byte lastTaskIndex = TASKS_MAX - 1;
  if (showSpecificTask)
  {
    firstTaskIndex = taskNr - 1;
    lastTaskIndex = taskNr - 1;
  }
  byte lastActiveTaskIndex = 0;
  for (byte TaskIndex = firstTaskIndex; TaskIndex <= lastTaskIndex; TaskIndex++) {
    if (Settings.TaskDeviceNumber[TaskIndex])
      lastActiveTaskIndex = TaskIndex;
  }

  if (!showSpecificTask) TXBuffer += F("\"Sensors\":[\n");
  unsigned long ttl_json = 60;
  for (byte TaskIndex = firstTaskIndex; TaskIndex <= lastActiveTaskIndex; TaskIndex++)
  {
    if (Settings.TaskDeviceNumber[TaskIndex])
    {
      byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[TaskIndex]);
      const unsigned long taskInterval = Settings.TaskDeviceTimer[TaskIndex];
      LoadTaskSettings(TaskIndex);
      TXBuffer += F("{\n");

      if (Device[DeviceIndex].ValueCount != 0) {
        if (ttl_json > taskInterval && taskInterval > 0 && Settings.TaskDeviceEnabled[TaskIndex]) {
          ttl_json = taskInterval;
        }
        TXBuffer += F("\"TaskValues\": [\n");
        for (byte x = 0; x < Device[DeviceIndex].ValueCount; x++)
        {
          TXBuffer += '{';
          stream_next_json_object_value(F("ValueNumber"), String(x + 1));
          stream_next_json_object_value(F("Name"), String(ExtraTaskSettings.TaskDeviceValueNames[x]));
          stream_next_json_object_value(F("NrDecimals"), String(ExtraTaskSettings.TaskDeviceValueDecimals[x]));
          stream_last_json_object_value(F("Value"), formatUserVarNoCheck(TaskIndex, x));
          if (x < (Device[DeviceIndex].ValueCount - 1))
            TXBuffer += ",\n";
        }
        TXBuffer += F("],\n");
      }
      if (showSpecificTask) {
        stream_next_json_object_value(F("TTL"), String(ttl_json * 1000));
      }
      if (showDataAcquisition) {
        TXBuffer += F("\"DataAcquisition\": [\n");
        for (byte x = 0; x < CONTROLLER_MAX; x++)
        {
          TXBuffer += '{';
          stream_next_json_object_value(F("Controller"), String(x + 1));
          stream_next_json_object_value(F("IDX"), String(Settings.TaskDeviceID[x][TaskIndex]));
          stream_last_json_object_value(F("Enabled"), jsonBool(Settings.TaskDeviceSendData[x][TaskIndex]));
          if (x < (CONTROLLER_MAX - 1))
            TXBuffer += ",\n";
        }
        TXBuffer += F("],\n");
      }
      if (showTaskDetails) {
        stream_next_json_object_value(F("TaskInterval"), String(taskInterval));
        stream_next_json_object_value(F("Type"), getPluginNameFromDeviceIndex(DeviceIndex));
        stream_next_json_object_value(F("TaskName"), String(ExtraTaskSettings.TaskDeviceName));
      }
      stream_next_json_object_value(F("TaskEnabled"), jsonBool(Settings.TaskDeviceEnabled[TaskIndex]));
      stream_last_json_object_value(F("TaskNumber"), String(TaskIndex + 1));
      if (TaskIndex != lastActiveTaskIndex)
        TXBuffer += ',';
      TXBuffer += '\n';
    }
  }
  if (!showSpecificTask) {
    TXBuffer += F("],\n");
    stream_last_json_object_value(F("TTL"), String(ttl_json * 1000));
  }

  TXBuffer.endStream();
}





void stream_timing_stats_json(unsigned long count, unsigned long minVal, unsigned long maxVal, float avg) {
  stream_next_json_object_value(F("count"), String(count));
  stream_next_json_object_value(F("min"), String(minVal));
  stream_next_json_object_value(F("max"), String(maxVal));
  stream_next_json_object_value(F("avg"), String(avg));
}

void stream_plugin_function_timing_stats_json(
      const String& object,
      unsigned long count, unsigned long minVal, unsigned long maxVal, float avg) {
  TXBuffer += "{\"";
  TXBuffer += object;
  TXBuffer += "\":{";
  stream_timing_stats_json(count, minVal, maxVal, avg);
  stream_last_json_object_value(F("unit"), F("usec"));
}

void stream_plugin_timing_stats_json(int pluginId) {
  String P_name = "";
  Plugin_ptr[pluginId](PLUGIN_GET_DEVICENAME, NULL, P_name);
  TXBuffer += '{';
  stream_next_json_object_value(F("name"), P_name);
  stream_next_json_object_value(F("id"), String(pluginId));
  stream_json_start_array(F("function"));
}

void stream_json_start_array(const String& label) {
  TXBuffer += '\"';
  TXBuffer += label;
  TXBuffer += F("\": [\n");
}

void stream_json_end_array_element(bool isLast) {
  if (isLast) {
    TXBuffer += "]\n";
  } else {
    TXBuffer += ",\n";
  }
}

void stream_json_end_object_element(bool isLast) {
  TXBuffer += '}';
  if (!isLast) {
    TXBuffer += ',';
  }
  TXBuffer += '\n';
}

#ifdef WEBSERVER_NEW_UI
void handle_timingstats_json() {
  TXBuffer.startJsonStream();
  TXBuffer += '{';
  jsonStatistics(false);
  TXBuffer += '}';
  TXBuffer.endStream();
}
#endif




void format_using_threshhold(unsigned long value) {
  float value_msec = value / 1000.0;
  if (value > TIMING_STATS_THRESHOLD) {
    html_B(String(value_msec, 3));
  } else {
    TXBuffer += String(value_msec, 3);
  }
}

void stream_html_timing_stats(const TimingStats& stats, long timeSinceLastReset) {
    unsigned long minVal, maxVal;
    unsigned int c = stats.getMinMax(minVal, maxVal);

    html_TD();
    TXBuffer += c;
    html_TD();
    float call_per_sec = static_cast<float>(c) / static_cast<float>(timeSinceLastReset) * 1000.0;
    TXBuffer += call_per_sec;
    html_TD();
    format_using_threshhold(minVal);
    html_TD();
    format_using_threshhold(stats.getAvg());
    html_TD();
    format_using_threshhold(maxVal);
}



long stream_timing_statistics(bool clearStats) {
  long timeSinceLastReset = timePassedSince(timingstats_last_reset);
  for (auto& x: pluginStats) {
      if (!x.second.isEmpty()) {
          const int pluginId = x.first/256;
          String P_name = "";
          Plugin_ptr[pluginId](PLUGIN_GET_DEVICENAME, NULL, P_name);
          if (x.second.thresholdExceeded(TIMING_STATS_THRESHOLD)) {
            html_TR_TD_highlight();
          } else {
            html_TR_TD();
          }
          TXBuffer += F("P_");
          TXBuffer += Device[pluginId].Number;
          TXBuffer += '_';
          TXBuffer += P_name;
          html_TD();
          TXBuffer += getPluginFunctionName(x.first%256);
          stream_html_timing_stats(x.second, timeSinceLastReset);
          if (clearStats) x.second.reset();
      }
  }
  for (auto& x: controllerStats) {
      if (!x.second.isEmpty()) {
          const int pluginId = x.first/256;
          String C_name = "";
          CPluginCall(pluginId, CPLUGIN_GET_DEVICENAME, NULL, C_name);
          if (x.second.thresholdExceeded(TIMING_STATS_THRESHOLD)) {
            html_TR_TD_highlight();
          } else {
            html_TR_TD();
          }
          TXBuffer += F("C_");
          TXBuffer += Protocol[pluginId].Number;
          TXBuffer += '_';
          TXBuffer += C_name;
          html_TD();
          TXBuffer += getCPluginCFunctionName(x.first%256);
          stream_html_timing_stats(x.second, timeSinceLastReset);
          if (clearStats) x.second.reset();
      }
  }
  for (auto& x: miscStats) {
      if (!x.second.isEmpty()) {
          if (x.second.thresholdExceeded(TIMING_STATS_THRESHOLD)) {
            html_TR_TD_highlight();
          } else {
            html_TR_TD();
          }
          TXBuffer += getMiscStatsName(x.first);
          html_TD();
          stream_html_timing_stats(x.second, timeSinceLastReset);
          if (clearStats) x.second.reset();
      }
  }
  if (clearStats) {
    timingstats_last_reset = millis();
  }
  return timeSinceLastReset;
}

#ifdef WEBSERVER_TIMINGSTATS
void handle_timingstats() {
  checkRAM(F("handle_timingstats"));
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_table_class_multirow();
  html_TR();
  html_table_header(F("Description"));
  html_table_header(F("Function"));
  html_table_header(F("#calls"));
  html_table_header(F("call/sec"));
  html_table_header(F("min (ms)"));
  html_table_header(F("Avg (ms)"));
  html_table_header(F("max (ms)"));

  long timeSinceLastReset = stream_timing_statistics(true);
  html_end_table();

  html_table_class_normal();
  const float timespan = timeSinceLastReset / 1000.0;
  addFormHeader(F("Statistics"));
  addRowLabel(F("Start Period"));
  struct tm startPeriod = addSeconds(tm, -1.0 * timespan, false);
  TXBuffer += getDateTimeString(startPeriod, '-', ':', ' ', false);
  addRowLabelValue(LabelType::LOCAL_TIME);
  addRowLabel(F("Time span"));
  TXBuffer += String(timespan);
  TXBuffer += " sec";
  html_end_table();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}
#endif




void handle_advanced() {
  checkRAM(F("handle_advanced"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  int timezone = getFormItemInt(F("timezone"));
  int dststartweek = getFormItemInt(F("dststartweek"));
  int dststartdow = getFormItemInt(F("dststartdow"));
  int dststartmonth = getFormItemInt(F("dststartmonth"));
  int dststarthour = getFormItemInt(F("dststarthour"));
  int dstendweek = getFormItemInt(F("dstendweek"));
  int dstenddow = getFormItemInt(F("dstenddow"));
  int dstendmonth = getFormItemInt(F("dstendmonth"));
  int dstendhour = getFormItemInt(F("dstendhour"));
  String edit = WebServer.arg(F("edit"));


  if (edit.length() != 0)
  {
    Settings.MessageDelay = getFormItemInt(F("messagedelay"));
    Settings.IP_Octet = WebServer.arg(F("ip")).toInt();
    strncpy_webserver_arg(Settings.NTPHost, F("ntphost"));
    Settings.TimeZone = timezone;
    TimeChangeRule dst_start(dststartweek, dststartdow, dststartmonth, dststarthour, timezone);
    if (dst_start.isValid()) { Settings.DST_Start = dst_start.toFlashStoredValue(); }
    TimeChangeRule dst_end(dstendweek, dstenddow, dstendmonth, dstendhour, timezone);
    if (dst_end.isValid()) { Settings.DST_End = dst_end.toFlashStoredValue(); }
    str2ip(WebServer.arg(F("syslogip")).c_str(), Settings.Syslog_IP);
    Settings.UDPPort = getFormItemInt(F("udpport"));

    Settings.SyslogFacility = getFormItemInt(F("syslogfacility"));
    Settings.UseSerial = isFormItemChecked(F("useserial"));
    setLogLevelFor(LOG_TO_SYSLOG, getFormItemInt(getInternalLabel(LabelType::SYSLOG_LOG_LEVEL)));
    setLogLevelFor(LOG_TO_SERIAL, getFormItemInt(getInternalLabel(LabelType::SERIAL_LOG_LEVEL)));
    setLogLevelFor(LOG_TO_WEBLOG, getFormItemInt(getInternalLabel(LabelType::WEB_LOG_LEVEL)));
#ifdef FEATURE_SD
    setLogLevelFor(LOG_TO_SDCARD, getFormItemInt(getInternalLabel(LabelType::SD_LOG_LEVEL)));
#endif
    Settings.UseValueLogger = isFormItemChecked(F("valuelogger"));
    Settings.BaudRate = getFormItemInt(F("baudrate"));
    Settings.UseNTP = isFormItemChecked(F("usentp"));
    Settings.DST = isFormItemChecked(F("dst"));
    Settings.WDI2CAddress = getFormItemInt(F("wdi2caddress"));
    Settings.UseSSDP = isFormItemChecked(F("usessdp"));
    Settings.WireClockStretchLimit = getFormItemInt(F("wireclockstretchlimit"));
    Settings.UseRules = isFormItemChecked(F("userules"));
    Settings.ConnectionFailuresThreshold = getFormItemInt(F("cft"));
    Settings.MQTTRetainFlag = isFormItemChecked(F("mqttretainflag"));
    Settings.ArduinoOTAEnable = isFormItemChecked(F("arduinootaenable"));
    Settings.UseRTOSMultitasking = isFormItemChecked(F("usertosmultitasking"));
    Settings.MQTTUseUnitNameAsClientId = isFormItemChecked(F("mqttuseunitnameasclientid"));
    Settings.uniqueMQTTclientIdReconnect(isFormItemChecked(F("uniquemqttclientidreconnect")));
    Settings.Latitude = getFormItemFloat(F("latitude"));
    Settings.Longitude = getFormItemFloat(F("longitude"));
    Settings.OldRulesEngine(isFormItemChecked(F("oldrulesengine")));
    Settings.ForceWiFi_bg_mode(isFormItemChecked(getInternalLabel(LabelType::FORCE_WIFI_BG)));
    Settings.WiFiRestart_connection_lost(isFormItemChecked(getInternalLabel(LabelType::RESTART_WIFI_LOST_CONN)));
    Settings.EcoPowerMode(isFormItemChecked(getInternalLabel(LabelType::CPU_ECO_MODE)));
    Settings.WifiNoneSleep(isFormItemChecked(getInternalLabel(LabelType::FORCE_WIFI_NOSLEEP)));
#ifdef SUPPORT_ARP
    Settings.gratuitousARP(isFormItemChecked(getInternalLabel(LabelType::PERIODICAL_GRAT_ARP)));
#endif

    addHtmlError(SaveSettings());
    if (systemTimePresent())
      initTime();
  }

  TXBuffer += F("<form  method='post'>");
  html_table_class_normal();

  addFormHeader(F("Advanced Settings"), F("RTDTools/Tools.html#advanced"));

  addFormSubHeader(F("Rules Settings"));

  addFormCheckBox(F("Rules"), F("userules"), Settings.UseRules);
  addFormCheckBox(F("Old Engine"), F("oldrulesengine"), Settings.OldRulesEngine());

  addFormSubHeader(F("Controller Settings"));

  addFormCheckBox(F("MQTT Retain Msg"), F("mqttretainflag"), Settings.MQTTRetainFlag);
  addFormNumericBox( F("Message Interval"), F("messagedelay"), Settings.MessageDelay, 0, INT_MAX);
  addUnit(F("ms"));
  addFormCheckBox(F("MQTT use unit name as ClientId"), F("mqttuseunitnameasclientid"), Settings.MQTTUseUnitNameAsClientId);
  addFormCheckBox(F("MQTT change ClientId at reconnect"), F("uniquemqttclientidreconnect"), Settings.uniqueMQTTclientIdReconnect());

  addFormSubHeader(F("NTP Settings"));

  addFormCheckBox(F("Use NTP"), F("usentp"), Settings.UseNTP);
  addFormTextBox( F("NTP Hostname"), F("ntphost"), Settings.NTPHost, 63);

  addFormSubHeader(F("DST Settings"));
  addFormDstSelect(true, Settings.DST_Start);
  addFormDstSelect(false, Settings.DST_End);
  addFormNumericBox(F("Timezone Offset (UTC +)"), F("timezone"), Settings.TimeZone, -720, 840);
  addUnit(F("minutes"));
  addFormCheckBox(F("DST"), F("dst"), Settings.DST);

  addFormSubHeader(F("Location Settings"));
  addFormFloatNumberBox(F("Latitude"), F("latitude"), Settings.Latitude, -90.0, 90.0);
  addUnit(F("&deg;"));
  addFormFloatNumberBox(F("Longitude"), F("longitude"), Settings.Longitude, -180.0, 180.0);
  addUnit(F("&deg;"));

  addFormSubHeader(F("Log Settings"));

  addFormIPBox(F("Syslog IP"), F("syslogip"), Settings.Syslog_IP);
  addFormLogLevelSelect(getLabel(LabelType::SYSLOG_LOG_LEVEL), getInternalLabel(LabelType::SYSLOG_LOG_LEVEL), Settings.SyslogLevel);
  addFormLogFacilitySelect(F("Syslog Facility"),F("syslogfacility"), Settings.SyslogFacility);
  addFormLogLevelSelect(getLabel(LabelType::SERIAL_LOG_LEVEL), getInternalLabel(LabelType::SERIAL_LOG_LEVEL), Settings.SerialLogLevel);
  addFormLogLevelSelect(getLabel(LabelType::WEB_LOG_LEVEL), getInternalLabel(LabelType::WEB_LOG_LEVEL), Settings.WebLogLevel);

#ifdef FEATURE_SD
  addFormLogLevelSelect(getLabel(LabelType::SD_LOG_LEVEL), getInternalLabel(LabelType::SD_LOG_LEVEL), Settings.SDLogLevel);

  addFormCheckBox(F("SD Card Value Logger"), F("valuelogger"), Settings.UseValueLogger);
#endif


  addFormSubHeader(F("Serial Settings"));

  addFormCheckBox(F("Enable Serial port"), F("useserial"), Settings.UseSerial);
  addFormNumericBox(F("Baud Rate"), F("baudrate"), Settings.BaudRate, 0, 1000000);


  addFormSubHeader(F("Inter-ESPEasy Network"));

  addFormNumericBox(F("UDP port"), F("udpport"), Settings.UDPPort, 0, 65535);



  addFormSubHeader(F("Special and Experimental Settings"));

  addFormNumericBox(F("Fixed IP Octet"), F("ip"), Settings.IP_Octet, 0, 255);

  addFormNumericBox(F("WD I2C Address"), F("wdi2caddress"), Settings.WDI2CAddress, 0, 127);
  TXBuffer += F(" (decimal)");

  addFormNumericBox(F("I2C ClockStretchLimit"), F("wireclockstretchlimit"), Settings.WireClockStretchLimit);
  #if defined(FEATURE_ARDUINO_OTA)
  addFormCheckBox(F("Enable Arduino OTA"), F("arduinootaenable"), Settings.ArduinoOTAEnable);
  #endif
  #if defined(ESP32)
    addFormCheckBox_disabled(F("Enable RTOS Multitasking"), F("usertosmultitasking"), Settings.UseRTOSMultitasking);
  #endif

  addFormCheckBox_disabled(F("Use SSDP"), F("usessdp"), Settings.UseSSDP);

  addFormNumericBox(getLabel(LabelType::CONNECTION_FAIL_THRESH), F("cft"), Settings.ConnectionFailuresThreshold, 0, 100);
#ifdef ESP8266
  addFormCheckBox(LabelType::FORCE_WIFI_BG, Settings.ForceWiFi_bg_mode());
#endif
#ifdef ESP32

  addFormCheckBox_disabled(LabelType::FORCE_WIFI_BG, Settings.ForceWiFi_bg_mode());
#endif

  addFormCheckBox(LabelType::RESTART_WIFI_LOST_CONN, Settings.WiFiRestart_connection_lost());
#ifdef ESP8266
  addFormCheckBox(LabelType::FORCE_WIFI_NOSLEEP, Settings.WifiNoneSleep());
#endif
  addFormNote(F("Change WiFi sleep settings requires reboot to activate"));
#ifdef SUPPORT_ARP
  addFormCheckBox(LabelType::PERIODICAL_GRAT_ARP, Settings.gratuitousARP());
#endif
  addFormCheckBox(LabelType::CPU_ECO_MODE, Settings.EcoPowerMode());
  addFormNote(F("Node may miss receiving packets with Eco mode enabled"));
  addFormSeparator(2);

  html_TR_TD();
  html_TD();
  addSubmitButton();
  TXBuffer += F("<input type='hidden' name='edit' value='1'>");
  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}

void addFormDstSelect(bool isStart, uint16_t choice) {
  String weekid = isStart ? F("dststartweek") : F("dstendweek");
  String dowid = isStart ? F("dststartdow") : F("dstenddow");
  String monthid = isStart ? F("dststartmonth") : F("dstendmonth");
  String hourid = isStart ? F("dststarthour") : F("dstendhour");

  String weeklabel = isStart ? F("Start (week, dow, month)") : F("End (week, dow, month)");
  String hourlabel = isStart ? F("Start (localtime, e.g. 2h&rarr;3h)") : F("End (localtime, e.g. 3h&rarr;2h)");

  String week[5] = {F("Last"), F("1st"), F("2nd"), F("3rd"), F("4th")};
  int weekValues[5] = {0, 1, 2, 3, 4};
  String dow[7] = {F("Sun"), F("Mon"), F("Tue"), F("Wed"), F("Thu"), F("Fri"), F("Sat")};
  int dowValues[7] = {1, 2, 3, 4, 5, 6, 7};
  String month[12] = {F("Jan"), F("Feb"), F("Mar"), F("Apr"), F("May"), F("Jun"), F("Jul"), F("Aug"), F("Sep"), F("Oct"), F("Nov"), F("Dec")};
  int monthValues[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

  uint16_t tmpstart(choice);
  uint16_t tmpend(choice);
  if (!TimeChangeRule(choice, 0).isValid()) {
    getDefaultDst_flash_values(tmpstart, tmpend);
  }
  TimeChangeRule rule(isStart ? tmpstart : tmpend, 0);
  addRowLabel(weeklabel);
  addSelector(weekid, 5, week, weekValues, NULL, rule.week, false);
  TXBuffer += F("<BR>");
  addSelector(dowid, 7, dow, dowValues, NULL, rule.dow, false);
  TXBuffer += F("<BR>");
  addSelector(monthid, 12, month, monthValues, NULL, rule.month, false);

  addFormNumericBox(hourlabel, hourid, rule.hour, 0, 23);
  addUnit(isStart ? F("hour &#x21b7;") : F("hour &#x21b6;"));
}

void addFormLogLevelSelect(const String& label, const String& id, int choice)
{
  addRowLabel(label);
  addLogLevelSelect(id, choice);
}

void addLogLevelSelect(const String& name, int choice)
{
  String options[LOG_LEVEL_NRELEMENTS + 1];
  int optionValues[LOG_LEVEL_NRELEMENTS + 1] = {0};
  options[0] = getLogLevelDisplayString(0);
  optionValues[0] = 0;
  for (int i = 0; i < LOG_LEVEL_NRELEMENTS; ++i) {
    options[i + 1] = getLogLevelDisplayStringFromIndex(i, optionValues[i + 1]);
  }
  addSelector(name, LOG_LEVEL_NRELEMENTS + 1, options, optionValues, NULL, choice, false);
}

void addFormLogFacilitySelect(const String& label, const String& id, int choice)
{
  addRowLabel(label);
  addLogFacilitySelect(id, choice);
}

void addLogFacilitySelect(const String& name, int choice)
{
  String options[12] = { F("Kernel"), F("User"), F("Daemon"), F("Message"), F("Local0"), F("Local1"), F("Local2"), F("Local3"), F("Local4"), F("Local5"), F("Local6"), F("Local7")};
  int optionValues[12] = { 0, 1, 3, 5, 16, 17, 18, 19, 20, 21, 22, 23 };
  addSelector(name, 12, options, optionValues, NULL, choice, false);
}





boolean isLoggedIn()
{
  String www_username = F(DEFAULT_ADMIN_USERNAME);
  if (!clientIPallowed()) return false;
  if (SecuritySettings.Password[0] == 0) return true;
  if (!WebServer.authenticate(www_username.c_str(), SecuritySettings.Password))







  {
#ifdef CORE_PRE_2_5_0

    HTTPAuthMethod mode = BASIC_AUTH;
#else
    HTTPAuthMethod mode = DIGEST_AUTH;
#endif
    String message = F("Login Required (default user: ");
    message += www_username;
    message += ')';
    WebServer.requestAuthentication(mode, message.c_str());
    return false;
  }
  return true;
}

void handle_dumpcache() {
  if (!isLoggedIn()) return;

  #ifdef USES_C016
# 5193 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
    C016_startCSVdump();
    unsigned long timestamp;
    byte controller_idx;
    byte TaskIndex;
    byte sensorType;
    byte valueCount;
    float val1;
    float val2;
    float val3;
    float val4;

    TXBuffer.startStream();
    TXBuffer += F("UNIX timestamp;contr. idx;sensortype;taskindex;value count");
    for (int i = 0; i < TASKS_MAX; ++i) {
      LoadTaskSettings(i);
      for (int j = 0; j < VARS_PER_TASK; ++j) {
        TXBuffer += ';';
        TXBuffer += ExtraTaskSettings.TaskDeviceName;
        TXBuffer += '#';
        TXBuffer += ExtraTaskSettings.TaskDeviceValueNames[j];
      }
    }
    TXBuffer += F("<BR>");
    float csv_values[VARS_PER_TASK * TASKS_MAX];
    for (int i = 0; i < VARS_PER_TASK * TASKS_MAX; ++i) {
      csv_values[i] = 0.0;
    }

    while (C016_getCSVline(timestamp, controller_idx, TaskIndex, sensorType,
                           valueCount, val1, val2, val3, val4)) {
      TXBuffer += timestamp;
      TXBuffer += ';';
      TXBuffer += controller_idx;
      TXBuffer += ';';
      TXBuffer += sensorType;
      TXBuffer += ';';
      TXBuffer += TaskIndex;
      TXBuffer += ';';
      TXBuffer += valueCount;
      int valindex = TaskIndex * VARS_PER_TASK;
      csv_values[valindex++] = val1;
      csv_values[valindex++] = val2;
      csv_values[valindex++] = val3;
      csv_values[valindex++] = val4;
      for (int i = 0; i < VARS_PER_TASK * TASKS_MAX; ++i) {
        TXBuffer += ';';
        if (csv_values[i] == 0.0) {
          TXBuffer += '0';
        } else {
          TXBuffer += String(csv_values[i], 6);
        }
      }
      TXBuffer += F("<BR>");
      delay(0);
    }
    TXBuffer.endStream();

  #endif
}

void handle_cache_json() {
  if (!isLoggedIn()) return;

  #ifdef USES_C016
  TXBuffer.startJsonStream();
  TXBuffer += F("{\"columns\": [");


  stream_to_json_value(F("UNIX timestamp"));
  TXBuffer += ',';
  stream_to_json_value(F("UTC timestamp"));
  TXBuffer += ',';
  stream_to_json_value(F("task index"));
  for (int i = 0; i < TASKS_MAX; ++i) {
    LoadTaskSettings(i);
    for (int j = 0; j < VARS_PER_TASK; ++j) {
      String label = ExtraTaskSettings.TaskDeviceName;
      label += '#';
      label += ExtraTaskSettings.TaskDeviceValueNames[j];
      TXBuffer += ',';
      stream_to_json_value(label);
    }
  }
  TXBuffer += F("],\n");
  C016_startCSVdump();
  TXBuffer += F("\"files\": [");
  bool islast = false;
  int filenr = 0;
  while (!islast) {
    String currentFile = C016_getCacheFileName(islast);
    if (currentFile.length() > 0) {
      if (filenr != 0) {
        TXBuffer += ',';
      }
      stream_to_json_value(currentFile);
      ++filenr;
    }
  }
  TXBuffer += F("],\n");
  stream_last_json_object_value(F("nrfiles"), String(filenr));
  TXBuffer += F("\n");
  TXBuffer.endStream();
  #endif
}

void handle_cache_csv() {
  if (!isLoggedIn()) return;

}





void handle_download()
{
  checkRAM(F("handle_download"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;




  fs::File dataFile = tryOpenFile(F(FILE_CONFIG), "r");
  if (!dataFile)
    return;

  String str = F("attachment; filename=config_");
  str += Settings.Name;
  str += "_U";
  str += Settings.Unit;
  str += F("_Build");
  str += BUILD;
  str += '_';
  if (systemTimePresent())
  {
    str += getDateTimeString('\0', '\0', '\0');
  }
  str += F(".dat");

  WebServer.sendHeader(F("Content-Disposition"), str);
  WebServer.streamFile(dataFile, F("application/octet-stream"));
  dataFile.close();
}





byte uploadResult = 0;
void handle_upload() {
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  TXBuffer += F("<form enctype='multipart/form-data' method='post'><p>Upload settings file:<br><input type='file' name='datafile' size='40'></p><div><input class='button link' type='submit' value='Upload'></div><input type='hidden' name='edit' value='1'></form>");
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb = false;
}





void handle_upload_post() {
  checkRAM(F("handle_upload_post"));
  if (!isLoggedIn()) return;

  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();



  if (uploadResult == 1)
  {
    TXBuffer += F("Upload OK!<BR>You may need to reboot to apply all settings...");
    LoadSettings();
  }

  if (uploadResult == 2)
    TXBuffer += F("<font color=\"red\">Upload file invalid!</font>");

  if (uploadResult == 3)
    TXBuffer += F("<font color=\"red\">No filename!</font>");


  TXBuffer += F("Upload finished");
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb = false;
}

#ifdef WEBSERVER_NEW_UI
void handle_upload_json() {
  checkRAM(F("handle_upload_post"));
  uint8_t result = uploadResult;
  if (!isLoggedIn()) result = 255;

  TXBuffer.startJsonStream();
  TXBuffer += "{";
  stream_next_json_object_value(F("status"), String(result));
  TXBuffer += "}";

  TXBuffer.endStream();
}
#endif




fs::File uploadFile;
void handleFileUpload() {
  checkRAM(F("handleFileUpload"));
  if (!isLoggedIn()) return;

  static boolean valid = false;

  HTTPUpload& upload = WebServer.upload();

  if (upload.filename.c_str()[0] == 0)
  {
    uploadResult = 3;
    return;
  }

  if (upload.status == UPLOAD_FILE_START)
  {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Upload: START, filename: ");
      log += upload.filename;
      addLog(LOG_LEVEL_INFO, log);
    }
    valid = false;
    uploadResult = 0;
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {

    if (upload.totalSize == 0)
    {
      if (strcasecmp(upload.filename.c_str(), FILE_CONFIG) == 0)
      {
        struct TempStruct {
          unsigned long PID;
          int Version;
        } Temp;
        for (unsigned int x = 0; x < sizeof(struct TempStruct); x++)
        {
          byte b = upload.buf[x];
          memcpy((byte*)&Temp + x, &b, 1);
        }
        if (Temp.Version == VERSION && Temp.PID == ESP_PROJECT_PID)
          valid = true;
      }
      else
      {

        valid = true;
      }
      if (valid)
      {
        String filename;
#if defined(ESP32)
        filename += '/';
#endif
        filename += upload.filename;

        tryDeleteFile(filename);
        uploadFile = tryOpenFile(filename.c_str(), "w");

      }
    }
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Upload: WRITE, Bytes: ");
      log += upload.currentSize;
      addLog(LOG_LEVEL_INFO, log);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (uploadFile) uploadFile.close();
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Upload: END, Size: ");
      log += upload.totalSize;
      addLog(LOG_LEVEL_INFO, log);
    }
  }

  if (valid)
    uploadResult = 1;
  else
    uploadResult = 2;

}





bool loadFromFS(boolean spiffs, String path) {

  checkRAM(F("loadFromFS"));
  if (!isLoggedIn()) return false;

  statusLED(true);

  String dataType = F("text/plain");
  if (path.endsWith("/")) path += F("index.htm");

  if (path.endsWith(F(".src"))) path = path.substring(0, path.lastIndexOf("."));
  else if (path.endsWith(F(".htm")) || path.endsWith(F(".htm.gz"))) dataType = F("text/html");
  else if (path.endsWith(F(".css")) || path.endsWith(F(".css.gz"))) dataType = F("text/css");
  else if (path.endsWith(F(".js")) || path.endsWith(F(".js.gz"))) dataType = F("application/javascript");
  else if (path.endsWith(F(".png")) || path.endsWith(F(".png.gz"))) dataType = F("image/png");
  else if (path.endsWith(F(".gif")) || path.endsWith(F(".gif.gz"))) dataType = F("image/gif");
  else if (path.endsWith(F(".jpg")) || path.endsWith(F(".jpg.gz"))) dataType = F("image/jpeg");
  else if (path.endsWith(F(".ico"))) dataType = F("image/x-icon");
  else if (path.endsWith(F(".txt")) ||
           path.endsWith(F(".dat"))) dataType = F("application/octet-stream");
  else if (path.endsWith(F(".esp"))) return handle_custom(path);

#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("HTML : Request file ");
    log += path;
    addLog(LOG_LEVEL_DEBUG, log);
  }
#endif

#if !defined(ESP32)
  path = path.substring(1);
#endif

  if (spiffs)
  {
    fs::File dataFile = tryOpenFile(path.c_str(), "r");
    if (!dataFile)
      return false;


    WebServer.sendHeader(F("Cache-Control"), F("max-age=3600, public"));
    WebServer.sendHeader(F("Vary"),"*");
    WebServer.sendHeader(F("ETag"), F("\"2.0.0\""));

    if (path.endsWith(F(".dat")))
      WebServer.sendHeader(F("Content-Disposition"), F("attachment;"));

    WebServer.streamFile(dataFile, dataType);
    dataFile.close();
  }
  else
  {
#ifdef FEATURE_SD
    File dataFile = SD.open(path.c_str());
    if (!dataFile)
      return false;
    if (path.endsWith(F(".DAT")))
      WebServer.sendHeader(F("Content-Disposition"), F("attachment;"));
    WebServer.streamFile(dataFile, dataType);
    dataFile.close();
#else

    return false;
#endif
  }
  statusLED(true);
  return true;
}




boolean handle_custom(String path) {

  checkRAM(F("handle_custom"));
  if (!clientIPallowed()) return false;

#if !defined(ESP32)
  path = path.substring(1);
#endif


  fs::File dataFile = tryOpenFile(path.c_str(), "r");
  const bool dashboardPage = path.startsWith(F("dashboard"));
  if (!dataFile && !dashboardPage) {
    return false;
  }

  if (dashboardPage)
  {

    byte unit = getFormItemInt(F("unit"));
    byte btnunit = getFormItemInt(F("btnunit"));
    if(!unit) unit = btnunit;
    if (unit && unit != Settings.Unit)
    {
      NodesMap::iterator it = Nodes.find(unit);
      if (it != Nodes.end()) {
        TXBuffer.startStream();
        sendHeadandTail(F("TmplDsh"),_HEAD);
        TXBuffer += F("<meta http-equiv=\"refresh\" content=\"0; URL=http://");
        TXBuffer += it->second.ip.toString();
        TXBuffer += F("/dashboard.esp\">");
        sendHeadandTail(F("TmplDsh"),_TAIL);
        TXBuffer.endStream();
        return true;
      }
    }

    TXBuffer.startStream();
    sendHeadandTail(F("TmplDsh"),_HEAD);
    html_add_autosubmit_form();
    html_add_form();


    addSelector_Head(F("unit"), true);
    byte choice = Settings.Unit;
    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
    {
      if (it->second.ip[0] != 0 || it->first == Settings.Unit)
      {
        String name = String(it->first) + F(" - ");
        if (it->first != Settings.Unit)
          name += it->second.nodeName;
        else
          name += Settings.Name;
        addSelector_Item(name, it->first, choice == it->first, false, "");
      }
    }
    addSelector_Foot();


    byte prev=Settings.Unit;
    byte next=Settings.Unit;
    NodesMap::iterator it;
    for (byte x = Settings.Unit-1; x > 0; x--) {
      it = Nodes.find(x);
      if (it != Nodes.end()) {
        if (it->second.ip[0] != 0) {prev = x; break;}
      }
    }
    for (byte x = Settings.Unit+1; x < UNIT_MAX; x++) {
      it = Nodes.find(x);
      if (it != Nodes.end()) {
        if (it->second.ip[0] != 0) {next = x; break;}
      }
    }

    html_add_button_prefix();
    TXBuffer += path;
    TXBuffer += F("?btnunit=");
    TXBuffer += prev;
    TXBuffer += F("'>&lt;</a>");
    html_add_button_prefix();
    TXBuffer += path;
    TXBuffer += F("?btnunit=");
    TXBuffer += next;
    TXBuffer += F("'>&gt;</a>");
  }


  String webrequest = WebServer.arg(F("cmd"));
  if (webrequest.length() > 0 ){
    struct EventStruct TempEvent;
    parseCommandString(&TempEvent, webrequest);
    TempEvent.Source = VALUE_SOURCE_HTTP;

    if (PluginCall(PLUGIN_WRITE, &TempEvent, webrequest));
    else if (remoteConfig(&TempEvent, webrequest));
    else if (webrequest.startsWith(F("event")))
      ExecuteCommand(VALUE_SOURCE_HTTP, webrequest.c_str());


    PluginCall(PLUGIN_TEN_PER_SECOND, 0, dummyString);
  }


  if (dataFile)
  {
    String page = "";
    page.reserve(dataFile.size());
    while (dataFile.available())
      page += ((char)dataFile.read());

    TXBuffer += parseTemplate(page,0);
    dataFile.close();
  }
  else
  {
    if (dashboardPage)
    {

      TXBuffer += F("<meta name='viewport' content='width=width=device-width, initial-scale=1'><STYLE>* {font-family:sans-serif; font-size:16pt;}.button {margin:4px; padding:4px 16px; background-color:#07D; color:#FFF; text-decoration:none; border-radius:4px}</STYLE>");
      html_table_class_normal();
      for (byte x = 0; x < TASKS_MAX; x++)
      {
        if (Settings.TaskDeviceNumber[x] != 0)
          {
            LoadTaskSettings(x);
            byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[x]);
            html_TR_TD();
            TXBuffer += ExtraTaskSettings.TaskDeviceName;
            for (byte varNr = 0; varNr < VARS_PER_TASK; varNr++)
              {
                if ((Settings.TaskDeviceNumber[x] != 0) && (varNr < Device[DeviceIndex].ValueCount) && ExtraTaskSettings.TaskDeviceValueNames[varNr][0] !=0)
                {
                  if (varNr > 0)
                    html_TR_TD();
                  html_TD();
                  TXBuffer += ExtraTaskSettings.TaskDeviceValueNames[varNr];
                  html_TD();
                  TXBuffer += String(UserVar[x * VARS_PER_TASK + varNr], ExtraTaskSettings.TaskDeviceValueDecimals[varNr]);
                }
              }
          }
      }
    }
  }
  sendHeadandTail(F("TmplDsh"),_TAIL);
  TXBuffer.endStream();
  return true;
}


#ifdef WEBSERVER_NEW_UI



void handle_filelist_json() {
  checkRAM(F("handle_filelist"));
  if (!clientIPallowed()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();

  String fdelete = WebServer.arg(F("delete"));
  if (tryDeleteFile(fdelete)) {
    #if defined(ESP32)

    #endif
    #if defined(ESP8266)
    checkRuleSets();
    #endif
  }

  const int pageSize = 25;
  int startIdx = 0;

  String fstart = WebServer.arg(F("start"));
  if (fstart.length() > 0)
  {
    startIdx = atoi(fstart.c_str());
  }
  int endIdx = startIdx + pageSize - 1;

  TXBuffer += "[{";
  bool firstentry = true;
  #if defined(ESP32)
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    int count = -1;
    while (file and count < endIdx)
    {
      if(!file.isDirectory()){
        ++count;

        if (count >= startIdx)
        {
          if (firstentry) {
            firstentry = false;
          } else {
            TXBuffer += ",{";
          }
          stream_next_json_object_value(F("fileName"), String(file.name()));
          stream_next_json_object_value(F("index"), String(startIdx));
          stream_last_json_object_value(F("size"), String(file.size()));
        }
      }
      file = root.openNextFile();
    }
  #endif
  #if defined(ESP8266)
  fs::Dir dir = SPIFFS.openDir("");

  int count = -1;
  while (dir.next())
  {
    ++count;

    if (count < startIdx)
    {
      continue;
    }

    if (firstentry) {
      firstentry = false;
    } else {
      TXBuffer += ",{";
    }

    stream_next_json_object_value(F("fileName"), String(dir.fileName()));

    fs::File f = dir.openFile("r");
    if (f) {
      stream_next_json_object_value(F("size"), String(f.size()));
      f.close();
    }

    stream_last_json_object_value(F("index"), String(startIdx));

    if (count >= endIdx)
    {
      break;
    }
  }

  #endif
  TXBuffer += "]";
  TXBuffer.endStream();
}
#endif

void handle_filelist() {
  checkRAM(F("handle_filelist"));
  if (!clientIPallowed()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  String fdelete = WebServer.arg(F("delete"));
  if (tryDeleteFile(fdelete))
  {
    checkRuleSets();
  }
  #ifdef USES_C016
  if (WebServer.hasArg(F("delcache"))) {
    while (C016_deleteOldestCacheBlock()) {
      delay(1);
    }
    while (GarbageCollection()) {
      delay(1);
    }
  }
  #endif
  const int pageSize = 25;
  int startIdx = 0;
  String fstart = WebServer.arg(F("start"));
  if (fstart.length() > 0)
  {
    startIdx = atoi(fstart.c_str());
  }
  int endIdx = startIdx + pageSize - 1;
  html_table_class_multirow();
  html_table_header("", 50);
  html_table_header(F("Filename"));
  html_table_header(F("Size"), 80);
  int count = -1;

  bool moreFilesPresent = false;
  bool cacheFilesPresent = false;

#if defined(ESP8266)

  fs::Dir dir = SPIFFS.openDir("");
  while (dir.next() && count < endIdx)
  {
    ++count;
    if (count >= startIdx)
    {
      int filesize = -1;
      fs::File f = dir.openFile("r");
      if (f) {
        filesize = f.size();
      }
      if (!cacheFilesPresent && getCacheFileCountFromFilename(dir.fileName()) != -1)
      {
        cacheFilesPresent = true;
      }
      handle_filelist_add_file(dir.fileName(), filesize, startIdx);
    }
  }
  moreFilesPresent = dir.next();
#endif
#if defined(ESP32)
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file && count < endIdx)
  {
    if(!file.isDirectory()){
      ++count;
      if (count >= startIdx)
      {
        if (!cacheFilesPresent && getCacheFileCountFromFilename(file.name()) != -1)
        {
          cacheFilesPresent = true;
        }
        handle_filelist_add_file(file.name(), file.size(), startIdx);
      }
    }
    file = root.openNextFile();
  }
  moreFilesPresent = file;
#endif

  int start_prev = -1;
  if (startIdx > 0)
  {
    start_prev = startIdx < pageSize ? 0 : startIdx - pageSize;
  }
  int start_next = -1;
  if (count >= endIdx && moreFilesPresent) {
    start_next = endIdx + 1;
  }
  handle_filelist_buttons(start_prev, start_next, cacheFilesPresent);
}

void handle_filelist_add_file(const String& filename, int filesize, int startIdx) {
  html_TR_TD();
  if (filename != F(FILE_CONFIG) && filename != F(FILE_SECURITY) && filename != F(FILE_NOTIFICATION))
  {
    html_add_button_prefix();
    TXBuffer += F("filelist?delete=");
    TXBuffer += filename;
    if (startIdx > 0)
    {
      TXBuffer += F("&start=");
      TXBuffer += startIdx;
    }
    TXBuffer += F("'>Del</a>");
  }

  TXBuffer += F("<TD><a href=\"");
  TXBuffer += filename;
  TXBuffer += "\">";
  TXBuffer += filename;
  TXBuffer += F("</a>");
  html_TD();
  if (filesize >= 0) {
    TXBuffer += String(filesize);
  }
}

void handle_filelist_buttons(int start_prev, int start_next, bool cacheFilesPresent) {
  html_end_table();
  html_end_form();
  html_BR();
  addButton(F("/upload"), F("Upload"));
  if (start_prev >= 0)
  {
    html_add_button_prefix();
    TXBuffer += F("/filelist?start=");
    TXBuffer += start_prev;
    TXBuffer += F("'>Previous</a>");
  }
  if (start_next >= 0)
  {
    html_add_button_prefix();
    TXBuffer += F("/filelist?start=");
    TXBuffer += start_next;
    TXBuffer += F("'>Next</a>");
  }
  if (cacheFilesPresent) {
    html_add_button_prefix(F("red"), true);
    TXBuffer += F("filelist?delcache");
    TXBuffer += F("'>Delete Cache Files</a>");
  }
  TXBuffer += F("<BR><BR>");
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}




#ifdef FEATURE_SD
void handle_SDfilelist() {
  checkRAM(F("handle_SDfilelist"));
  if (!clientIPallowed()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();


  String fdelete = "";
  String ddelete = "";
  String change_to_dir = "";
  String current_dir = "";
  String parent_dir = "";

  for (uint8_t i = 0; i < WebServer.args(); i++) {
    if (WebServer.argName(i) == F("delete"))
    {
      fdelete = WebServer.arg(i);
    }
    if (WebServer.argName(i) == F("deletedir"))
    {
      ddelete = WebServer.arg(i);
    }
    if (WebServer.argName(i) == F("chgto"))
    {
      change_to_dir = WebServer.arg(i);
    }
  }

  if (fdelete.length() > 0)
  {
    SD.remove((char*)fdelete.c_str());
  }
  if (ddelete.length() > 0)
  {
    SD.rmdir((char*)ddelete.c_str());
  }
  if (change_to_dir.length() > 0)
  {
    current_dir = change_to_dir;
  }
  else
  {
    current_dir = "/";
  }

  File root = SD.open(current_dir.c_str());
  root.rewindDirectory();
  File entry = root.openNextFile();
  parent_dir = current_dir;
  if (!current_dir.equals("/"))
  {
# 6033 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
    parent_dir.remove(parent_dir.lastIndexOf("/", parent_dir.lastIndexOf("/") - 1) + 1);
  }



  String subheader = "SD Card: " + current_dir;
  addFormSubHeader(subheader);
  html_BR();
  html_table_class_multirow();
  html_table_header("", 50);
  html_table_header("Name");
  html_table_header("Size");
  html_TR_TD();
  TXBuffer += F("<TD><a href=\"SDfilelist?chgto=");
  TXBuffer += parent_dir;
  TXBuffer += F("\">..");
  TXBuffer += F("</a>");
  html_TD();
  while (entry)
  {
    if (entry.isDirectory())
    {
      char SDcardChildDir[80];
      html_TR_TD();

      String child_dir = current_dir + entry.name();
      child_dir.toCharArray(SDcardChildDir, child_dir.length()+1);
      File child = SD.open(SDcardChildDir);
      File dir_has_entry = child.openNextFile();

      if (!dir_has_entry)
      {
        TXBuffer += F("<a class='button link' onclick=\"return confirm('Delete this directory?')\" href=\"SDfilelist?deletedir=");
        TXBuffer += current_dir;
        TXBuffer += entry.name();
        TXBuffer += '/';
        TXBuffer += F("&chgto=");
        TXBuffer += current_dir;
        TXBuffer += F("\">Del</a>");
      }
      TXBuffer += F("<TD><a href=\"SDfilelist?chgto=");
      TXBuffer += current_dir;
      TXBuffer += entry.name();
      TXBuffer += '/';
      TXBuffer += "\">";
      TXBuffer += entry.name();
      TXBuffer += F("</a>");
      html_TD();
      TXBuffer += F("dir");
      dir_has_entry.close();
    }
    else
    {
      html_TR_TD();
      if (entry.name() != String(F(FILE_CONFIG)).c_str() && entry.name() != String(F(FILE_SECURITY)).c_str())
      {
        TXBuffer += F("<a class='button link' onclick=\"return confirm('Delete this file?')\" href=\"SDfilelist?delete=");
        TXBuffer += current_dir;
        TXBuffer += entry.name();
        TXBuffer += F("&chgto=");
        TXBuffer += current_dir;
        TXBuffer += F("\">Del</a>");
      }
      TXBuffer += F("<TD><a href=\"");
      TXBuffer += current_dir;
      TXBuffer += entry.name();
      TXBuffer += "\">";
      TXBuffer += entry.name();
      TXBuffer += F("</a>");
      html_TD();
      TXBuffer += entry.size();
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  html_end_table();
  html_end_form();

  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}
#endif





void handleNotFound() {
  checkRAM(F("handleNotFound"));

  if (wifiSetup)
  {
    WebServer.send(200, F("text/html"), F("<meta HTTP-EQUIV='REFRESH' content='0; url=/setup'>"));
    return;
  }

  if (!isLoggedIn()) return;
  if (handle_rules_edit(WebServer.uri())) return;
  if (loadFromFS(true, WebServer.uri())) return;
  if (loadFromFS(false, WebServer.uri())) return;
  String message = F("URI: ");
  message += WebServer.uri();
  message += F("\nMethod: ");
  message += (WebServer.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += WebServer.args();
  message += "\n";
  for (uint8_t i = 0; i < WebServer.args(); i++) {
    message += F(" NAME:");
    message += WebServer.argName(i);
    message += F("\n VALUE:");
    message += WebServer.arg(i);
    message += '\n';
  }
  WebServer.send(404, F("text/plain"), message);
}





void handle_setup() {
  checkRAM(F("handle_setup"));

  TXBuffer.startStream();
  sendHeadandTail(F("TmplAP"));

  addHeader(false,TXBuffer.buf);

  if (WiFiConnected())
  {
    addHtmlError(SaveSettings());
    String host = formatIP(WiFi.localIP());
    TXBuffer += F("<BR>ESP is connected and using IP Address: <BR><h1>");
    TXBuffer += host;
    TXBuffer += F("</h1><BR><BR>Connect your laptop / tablet / phone<BR>back to your main Wifi network and<BR><BR>");
    TXBuffer += F("<a class='button' href='http://");
    TXBuffer += host;
    TXBuffer += F("/config'>Proceed to main config</a><BR><BR>");

    sendHeadandTail(F("TmplAP"),true);
    TXBuffer.endStream();

    wifiSetup = false;

    timerAPoff = millis() + 60000L;
    return;
  }

  static byte status = 0;
  static int n = 0;
  static byte refreshCount = 0;
  String ssid = WebServer.arg(F("ssid"));
  String other = WebServer.arg(F("other"));
  String password = WebServer.arg(F("pass"));

  if (other.length() != 0)
  {
    ssid = other;
  }


  if (status == 0 && ssid.length() != 0 )
  {
    safe_strncpy(SecuritySettings.WifiKey, password.c_str(), sizeof(SecuritySettings.WifiKey));
    safe_strncpy(SecuritySettings.WifiSSID, ssid.c_str(), sizeof(SecuritySettings.WifiSSID));
    wifiSetupConnect = true;
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String reconnectlog = F("WIFI : Credentials Changed, retry connection. SSID: ");
      reconnectlog += ssid;
      addLog(LOG_LEVEL_INFO, reconnectlog);
    }
    status = 1;
    refreshCount = 0;
  }

  TXBuffer += F("<BR><h1>Wifi Setup wizard</h1>");
  html_add_form();

  if (status == 0)
  {
    WiFiMode_t cur_wifimode = WiFi.getMode();
    if (n == 0)
      n = WiFi.scanNetworks(false, true);
    setWifiMode(cur_wifimode);
    if (n == 0)
      TXBuffer += F("No Access Points found");
    else
    {
      html_table_class_multirow();
      html_TR();
      html_table_header(F("Pick"), 50);
      html_table_header(F("Network info"));
      for (int i = 0; i < n; ++i)
      {
        html_TR_TD(); TXBuffer += F("<label class='container2'>");
        TXBuffer += F("<input type='radio' name='ssid' value='");
        {
          String escapeBuffer = WiFi.SSID(i);
          htmlStrongEscape(escapeBuffer);
          TXBuffer += escapeBuffer;
        }
        TXBuffer += '\'';
        if (WiFi.SSID(i) == ssid)
          TXBuffer += F(" checked ");
        TXBuffer += F("><span class='dotmark'></span></label><TD>");
        TXBuffer += formatScanResult(i, "<BR>");
        TXBuffer += "";
      }
      html_end_table();
    }

    TXBuffer += F("<BR><label class='container2'>other SSID:<input type='radio' name='ssid' id='other_ssid' value='other' ><span class='dotmark'></span></label>");
    TXBuffer += F("<input class='wide' type ='text' name='other' value='");
    TXBuffer += other;
    TXBuffer += F("'><BR><BR>");

    addFormSeparator (2);

    TXBuffer += F("<BR>Password:<BR><input class='wide' type ='text' name='pass' value='");
    TXBuffer += password;
    TXBuffer += F("'><BR><BR>");

    addSubmitButton(F("Connect"),"");
  }

  if (status == 1)
  {
    if (refreshCount > 0)
    {
      status = 0;


      TXBuffer += F("<a class='button' href='setup'>Back to Setup</a><BR><BR>");
    }
    else
    {
      int wait = 20;
      if (refreshCount != 0)
        wait = 3;
      TXBuffer += F("Please wait for <h1 id='countdown'>20..</h1>");
      TXBuffer += F("<script type='text/JavaScript'>");
      TXBuffer += F("function timedRefresh(timeoutPeriod) {");
      TXBuffer += F( "var timer = setInterval(function() {");
      TXBuffer += F( "if (timeoutPeriod > 0) {");
      TXBuffer += F( "timeoutPeriod -= 1;");
      TXBuffer += F( "document.getElementById('countdown').innerHTML = timeoutPeriod + '..' + '<br />';");
      TXBuffer += F( "} else {");
      TXBuffer += F( "clearInterval(timer);");
      TXBuffer += F( "window.location.href = window.location.href;");
      TXBuffer += F( "};");
      TXBuffer += F( "}, 1000);");
      TXBuffer += F("};");
      TXBuffer += F("timedRefresh(");
      TXBuffer += wait;
      TXBuffer += F(");");
      html_add_script_end();
      TXBuffer += F("seconds while trying to connect");
    }
    refreshCount++;
  }

  html_end_form();
  sendHeadandTail(F("TmplAP"),true);
  TXBuffer.endStream();
  delay(10);
}




void addPreDefinedConfigSelector() {
  DeviceModel active_model = ResetFactoryDefaultPreference.getDeviceModel();
  addSelector_Head("fdm", true);
  for (byte x = 0; x < DeviceModel_MAX; ++x) {
    DeviceModel model = static_cast<DeviceModel>(x);
    addSelector_Item(
      getDeviceModelString(model),
      x,
      model == active_model,
      !modelMatchingFlashSize(model),
      ""
    );
  }
  addSelector_Foot();
}

#ifdef WEBSERVER_NEW_UI
void handle_factoryreset_json() {
  if (!isLoggedIn()) return;
  TXBuffer.startJsonStream();
  TXBuffer+="{";

  if (WebServer.hasArg("fdm")) {
    DeviceModel model = static_cast<DeviceModel>(getFormItemInt("fdm"));
    if (modelMatchingFlashSize(model)) {
      setFactoryDefault(model);
    }
  }
  if (WebServer.hasArg("kun")) {
    ResetFactoryDefaultPreference.keepUnitName(isFormItemChecked("kun"));
  }
  if (WebServer.hasArg("kw")) {
    ResetFactoryDefaultPreference.keepWiFi(isFormItemChecked("kw"));
  }
  if (WebServer.hasArg("knet")) {
    ResetFactoryDefaultPreference.keepNetwork(isFormItemChecked("knet"));
  }
  if (WebServer.hasArg("kntp")) {
    ResetFactoryDefaultPreference.keepNTP(isFormItemChecked("kntp"));
  }
  if (WebServer.hasArg("klog")) {
    ResetFactoryDefaultPreference.keepLogSettings(isFormItemChecked("klog"));
  }

  if (WebServer.hasArg(F("savepref"))) {

    applyFactoryDefaultPref();
    addHtmlError(SaveSettings());
    stream_last_json_object_value(F("status"), F("ok"));
  }
  if (WebServer.hasArg(F("performfactoryreset"))) {

      applyFactoryDefaultPref();
      stream_last_json_object_value(F("status"), F("ok"));
      TXBuffer+="}";
      TXBuffer.endStream();

      ResetFactory();
  } else {
    stream_last_json_object_value(F("status"), F("error"));
  }
  TXBuffer+="}";
  TXBuffer.endStream();
}
#endif




void handle_factoryreset() {
  checkRAM(F("handle_factoryreset"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_add_form();
  html_table_class_normal();
  html_TR();
  addFormHeader(F("Factory Reset"));

  if (WebServer.hasArg("fdm")) {
    DeviceModel model = static_cast<DeviceModel>(getFormItemInt("fdm"));
    if (modelMatchingFlashSize(model)) {
      setFactoryDefault(model);
    }
  }
  if (WebServer.hasArg("kun")) {
    ResetFactoryDefaultPreference.keepUnitName(isFormItemChecked("kun"));
  }
  if (WebServer.hasArg("kw")) {
    ResetFactoryDefaultPreference.keepWiFi(isFormItemChecked("kw"));
  }
  if (WebServer.hasArg("knet")) {
    ResetFactoryDefaultPreference.keepNetwork(isFormItemChecked("knet"));
  }
  if (WebServer.hasArg("kntp")) {
    ResetFactoryDefaultPreference.keepNTP(isFormItemChecked("kntp"));
  }
  if (WebServer.hasArg("klog")) {
    ResetFactoryDefaultPreference.keepLogSettings(isFormItemChecked("klog"));
  }

  if (WebServer.hasArg(F("savepref"))) {

    applyFactoryDefaultPref();
    addHtmlError(SaveSettings());
  }
  if (WebServer.hasArg(F("performfactoryreset"))) {

      applyFactoryDefaultPref();

      ResetFactory();
  } else {

    addTableSeparator(F("Settings to keep"), 2, 3);

    addRowLabel(F("Keep Unit/Name"));
    addCheckBox("kun", ResetFactoryDefaultPreference.keepUnitName());

    addRowLabel(F("Keep WiFi config"));
    addCheckBox("kw", ResetFactoryDefaultPreference.keepWiFi());

    addRowLabel(F("Keep Network config"));
    addCheckBox("knet", ResetFactoryDefaultPreference.keepNetwork());

    addRowLabel(F("Keep NTP/DST config"));
    addCheckBox("kntp", ResetFactoryDefaultPreference.keepNTP());

    addRowLabel(F("Keep log config"));
    addCheckBox("klog", ResetFactoryDefaultPreference.keepLogSettings());

    addTableSeparator(F("Pre-defined configurations"), 2, 3);
    addRowLabel(F("Pre-defined config"));
    addPreDefinedConfigSelector();


    html_TR_TD();
    html_TD();
    addSubmitButton(F("Save Preferences"), F("savepref"));


    html_TR_TD_height(30);

    addTableSeparator(F("Immediate full reset"), 2, 3);
    addRowLabel(F("Erase settings files"));
    addSubmitButton(F("Factory Reset"), F("performfactoryreset"), F("red"));
  }

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();

}




void handle_rules() {
  checkRAM(F("handle_rules"));
  if (!isLoggedIn() || !Settings.UseRules) return;
  navMenuIndex = MENU_INDEX_RULES;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();
  static byte currentSet = 1;

  const byte rulesSet = getFormItemInt(F("set"), 1);

  #if defined(ESP8266)
    String fileName = F("rules");
  #endif
  #if defined(ESP32)
    String fileName = F("/rules");
  #endif
  fileName += rulesSet;
  fileName += F(".txt");


  checkRAM(F("handle_rules"));



  if (WebServer.args() > 0)
  {
    String log = F("Rules : Save rulesSet: ");
    log += rulesSet;
    log += F(" currentSet: ");
    log += currentSet;

    if (currentSet == rulesSet)
    {
      String rules = WebServer.arg(F("rules"));
      log += F(" rules.length(): ");
      log += rules.length();
      if (rules.length() > RULES_MAX_SIZE)
        TXBuffer += F("<span style=\"color:red\">Data was not saved, exceeds web editor limit!</span>");
      else
      {
# 6512 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
          fs::File f = tryOpenFile(fileName, "w");
          if (f)
          {
            log += F(" Write to file: ");
            log += fileName;
            f.print(rules);
            f.close();

          }

      }
    }
    else
    {
      if (!SPIFFS.exists(fileName))
      {
        log += F(" Create new file: ");
        log += fileName;
        fs::File f = tryOpenFile(fileName, "w");
        if (f) f.close();
      }
    }
    addLog(LOG_LEVEL_INFO, log);

    log = F(" Webserver args:");
    for (int i = 0; i < WebServer.args(); ++i) {
      log += ' ';
      log += i;
      log += F(": '");
      log += WebServer.argName(i);
      log += F("' length: ");
      log += WebServer.arg(i).length();
    }
    addLog(LOG_LEVEL_INFO, log);
  }

  if (rulesSet != currentSet)
    currentSet = rulesSet;

  TXBuffer += F("<form name = 'frmselect' method = 'post'>");
  html_table_class_normal();
  html_TR();
  html_table_header(F("Rules"));

  byte choice = rulesSet;
  String options[RULESETS_MAX];
  int optionValues[RULESETS_MAX];
  for (byte x = 0; x < RULESETS_MAX; x++)
  {
    options[x] = F("Rules Set ");
    options[x] += x + 1;
    optionValues[x] = x + 1;
  }

   html_TR_TD();
  addSelector(F("set"), RULESETS_MAX, options, optionValues, NULL, choice, true);
  addHelpButton(F("Tutorial_Rules"));



  int size = 0;
  fs::File f = tryOpenFile(fileName, "r");
  if (f)
  {
    size = f.size();
    if (size > RULES_MAX_SIZE)
       TXBuffer += F("<span style=\"color:red\">Filesize exceeds web editor limit!</span>");
    else
    {
       html_TR_TD(); TXBuffer += F("<textarea name='rules' rows='30' wrap='off'>");
      while (f.available())
      {
        String c((char)f.read());
        htmlEscape(c);
         TXBuffer += c;
      }
       TXBuffer += F("</textarea>");
    }
    f.close();
  }

  html_TR_TD(); TXBuffer += F("Current size: ");
  TXBuffer += size;
  TXBuffer += F(" characters (Max ");
  TXBuffer += RULES_MAX_SIZE;
  TXBuffer += ')';

  addFormSeparator(2);

  html_TR_TD();
  addSubmitButton();
  addButton(fileName, F("Download to file"));
  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();

  checkRuleSets();
}

#ifdef WEBSERVER_NEW_UI




void handle_sysinfo_json() {
  checkRAM(F("handle_sysinfo"));
  if (!isLoggedIn()) return;
  TXBuffer.startJsonStream();
  json_init();
  json_open();
  json_open(false, F("general"));
    json_number(F("unit"), String(Settings.Unit));
    json_prop(F("time"), getDateTimeString('-', ':', ' '));

  char strUpTime[40];
  int minutes = wdcounter / 2;
  int days = minutes / 1440;
  minutes = minutes % 1440;
  int hrs = minutes / 60;
  minutes = minutes % 60;
  sprintf_P(strUpTime, PSTR("%d days %d hours %d minutes"), days, hrs, minutes);
    json_prop(F("uptime"), strUpTime);
    json_number(F("cpu_load"), String(getCPUload()));
    json_number(F("loop_count"), String(getLoopCountPerSec()));
  json_close();

  int freeMem = ESP.getFreeHeap();
  json_open(false, F("mem"));
    json_number(F("free"), String(freeMem));
    json_number(F("low_ram"), String(lowestRAM));
    json_prop(F("low_ram_fn"), String(lowestRAMfunction));
    json_number(F("stack"), String(getCurrentFreeStack()));
    json_number(F("low_stack"), String(lowestFreeStack));
    json_prop(F("low_stack_fn"), lowestFreeStackfunction);
  json_close();
  json_open(false, F("boot"));
    json_prop(F("last_cause"), getLastBootCauseString());
    json_number(F("counter"), String(RTC.bootCounter));
    json_prop(F("reset_reason"), getResetReasonString());
  json_close();
  json_open(false, F("wifi"));

  #if defined(ESP8266)
    byte PHYmode = wifi_get_phy_mode();
  #endif
  #if defined(ESP32)
    byte PHYmode = 3;
  #endif
  switch (PHYmode)
  {
    case 1:
        json_prop(F("type"), F("802.11B"));
      break;
    case 2:
      json_prop(F("type"), F("802.11G"));
      break;
    case 3:
      json_prop(F("type"), F("802.11N"));
      break;
  }
    json_number(F("rssi"), String(WiFi.RSSI()));
    json_prop(F("dhcp"), useStaticIP() ? getLabel(LabelType::IP_CONFIG_STATIC) : getLabel(LabelType::IP_CONFIG_DYNAMIC));
    json_prop(F("ip"), formatIP(WiFi.localIP()));
    json_prop(F("subnet"), formatIP(WiFi.subnetMask()));
    json_prop(F("gw"), formatIP(WiFi.gatewayIP()));
    json_prop(F("dns1"), formatIP(WiFi.dnsIP(0)));
    json_prop(F("dns2"), formatIP(WiFi.dnsIP(1)));
    json_prop(F("allowed_range"), describeAllowedIPrange());


    uint8_t mac[] = {0, 0, 0, 0, 0, 0};
    uint8_t* macread = WiFi.macAddress(mac);
    char macaddress[20];
    formatMAC(macread, macaddress);

    json_prop(F("sta_mac"), macaddress);

    macread = WiFi.softAPmacAddress(mac);
    formatMAC(macread, macaddress);

    json_prop(F("ap_mac"), macaddress);
    json_prop(F("ssid"), WiFi.SSID());
    json_prop(F("bssid"), WiFi.BSSIDstr());
    json_number(F("channel"), String(WiFi.channel()));
    json_prop(F("connected"), format_msec_duration(timeDiff(lastConnectMoment, millis())));
    json_prop(F("ldr"), getLastDisconnectReason());
    json_number(F("reconnects"), String(wifi_reconnects));
  json_close();

  json_open(false, F("firmware"));
    json_prop(F("build"), String(BUILD));
    json_prop(F("notes"), F(BUILD_NOTES));
    json_prop(F("libraries"), getSystemLibraryString());
    json_prop(F("git_version"), F(BUILD_GIT));
    json_prop(F("plugins"), getPluginDescriptionString());
    json_prop(F("md5"), String(CRCValues.compileTimeMD5[0],HEX));
    json_number(F("md5_check"), String(CRCValues.checkPassed()));
    json_prop(F("build_time"), String(CRCValues.compileTime));
    json_prop(F("filename"), String(CRCValues.binaryFilename));
  json_close();

  json_open(false, F("esp"));

  #if defined(ESP8266)
    json_prop(F("chip_id"), String(ESP.getChipId(), HEX));
    json_number(F("cpu"), String(ESP.getCpuFreqMHz()));
  #endif
  #if defined(ESP32)


    uint64_t chipid=ESP.getEfuseMac();
    uint32_t ChipId1 = (uint16_t)(chipid>>32);
    String espChipIdS(ChipId1, HEX);
    espChipIdS.toUpperCase();

    json_prop(F("chip_id"), espChipIdS);
    json_prop(F("cpu"), String(ESP.getCpuFreqMHz()));

    String espChipIdS1(ChipId1, HEX);
    espChipIdS1.toUpperCase();
    json_prop(F("chip_id1"), espChipIdS1);

  #endif
  #ifdef ARDUINO_BOARD
  json_prop(F("board"), ARDUINO_BOARD);
  #endif
  json_close();
  json_open(false, F("storage"));

  #if defined(ESP8266)
    uint32_t flashChipId = ESP.getFlashChipId();


    json_number(F("chip_id"), String(flashChipId));

    if (flashChipVendorPuya())
    {
      if (puyaSupport()) {
        json_prop(F("vendor"), F("puya, supported"));
      } else {
        json_prop(F("vendor"), F("puya, error"));
      }
    }
    uint32_t flashDevice = (flashChipId & 0xFF00) | ((flashChipId >> 16) & 0xFF);
    json_number(F("device"), String(flashDevice));
  #endif
    json_number(F("real_size"), String(getFlashRealSizeInBytes() / 1024));
    json_number(F("ide_size"), String(ESP.getFlashChipSize() / 1024));


  #if defined(ESP8266)
    json_number(F("flash_speed"), String(ESP.getFlashChipSpeed() / 1000000));

    FlashMode_t ideMode = ESP.getFlashChipMode();
    switch (ideMode) {
      case FM_QIO: json_prop(F("mode"), F("QIO")); break;
      case FM_QOUT: json_prop(F("mode"), F("QOUT")); break;
      case FM_DIO: json_prop(F("mode"), F("DIO")); break;
      case FM_DOUT: json_prop(F("mode"), F("DOUT")); break;
      default:
          json_prop(F("mode"), getUnknownString()); break;
    }
  #endif

    json_number(F("writes"), String(RTC.flashDayCounter));
    json_number(F("flash_counter"), String(RTC.flashCounter));
    json_number(F("sketch_size"), String(ESP.getSketchSize() / 1024));
    json_number(F("sketch_free"), String(ESP.getFreeSketchSpace() / 1024));

    json_number(F("spiffs_size"), String(SpiffsTotalBytes() / 1024));
    json_number(F("spiffs_free"), String(SpiffsFreeSpace() / 1024));
  json_close();
  json_close();

  TXBuffer.endStream();
}
#endif

void handle_sysinfo() {
  checkRAM(F("handle_sysinfo"));
  if (!isLoggedIn()) return;
  navMenuIndex = MENU_INDEX_TOOLS;
  html_reset_copyTextCounter();
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  int freeMem = ESP.getFreeHeap();

  addHeader(true, TXBuffer.buf);
  TXBuffer += printWebString;
  TXBuffer += F("<form>");


  html_table_class_normal();


  html_TR();
  html_table_header(F("System Info"), 225);
  TXBuffer += "<TH>";
  addCopyButton(F("copyText"), F("\\n"), F("Copy info to clipboard") );

  TXBuffer += githublogo;
  html_add_script(false);
  TXBuffer += DATA_GITHUB_CLIPBOARD_JS;
  html_add_script_end();

  addRowLabelValue(LabelType::UNIT_NR);

  if (systemTimePresent())
  {
     addRowLabelValue(LabelType::LOCAL_TIME);
  }

  addRowLabel(getLabel(LabelType::UPTIME));
  {
    char strUpTime[40];
    int minutes = wdcounter / 2;
    int days = minutes / 1440;
    minutes = minutes % 1440;
    int hrs = minutes / 60;
    minutes = minutes % 60;
    sprintf_P(strUpTime, PSTR("%d days %d hours %d minutes"), days, hrs, minutes);
    TXBuffer += strUpTime;
  }

  addRowLabel(getLabel(LabelType::LOAD_PCT));
  if (wdcounter > 0)
  {
     TXBuffer += getCPUload();
     TXBuffer += F("% (LC=");
     TXBuffer += getLoopCountPerSec();
     TXBuffer += ')';
  }
  addRowLabelValue(LabelType::CPU_ECO_MODE);

  addRowLabel(F("Free Mem"));
  TXBuffer += freeMem;
  TXBuffer += " (";
  TXBuffer += lowestRAM;
  TXBuffer += F(" - ");
  TXBuffer += lowestRAMfunction;
  TXBuffer += ')';
  addRowLabel(F("Free Stack"));
  TXBuffer += getCurrentFreeStack();
  TXBuffer += " (";
  TXBuffer += lowestFreeStack;
  TXBuffer += F(" - ");
  TXBuffer += lowestFreeStackfunction;
  TXBuffer += ')';
#ifdef CORE_POST_2_5_0
  addRowLabelValue(LabelType::HEAP_MAX_FREE_BLOCK);
  addRowLabelValue(LabelType::HEAP_FRAGMENTATION);
  TXBuffer += '%';
#endif


  addRowLabel(F("Boot"));
  TXBuffer += getLastBootCauseString();
  TXBuffer += " (";
  TXBuffer += RTC.bootCounter;
  TXBuffer += ')';
  addRowLabelValue(LabelType::RESET_REASON);
  addRowLabelValue(LabelType::LAST_TASK_BEFORE_REBOOT);
  addRowLabelValue(LabelType::SW_WD_COUNT);

  addTableSeparator(F("Network"), 2, 3, F("Wifi"));

  if (WiFiConnected())
  {
     addRowLabel(F("Wifi"));
    #if defined(ESP8266)
      byte PHYmode = wifi_get_phy_mode();
    #endif
    #if defined(ESP32)
      byte PHYmode = 3;
    #endif
    switch (PHYmode)
    {
      case 1:
         TXBuffer += F("802.11B");
        break;
      case 2:
         TXBuffer += F("802.11G");
        break;
      case 3:
         TXBuffer += F("802.11N");
        break;
    }
     TXBuffer += F(" (RSSI ");
     TXBuffer += WiFi.RSSI();
     TXBuffer += F(" dB)");
  }
  addRowLabelValue(LabelType::IP_CONFIG);
  addRowLabelValue(LabelType::IP_ADDRESS_SUBNET);
  addRowLabelValue(LabelType::GATEWAY);
  addRowLabelValue(LabelType::CLIENT_IP);
  addRowLabelValue(LabelType::DNS);
  addRowLabelValue(LabelType::ALLOWED_IP_RANGE);
  addRowLabel(getLabel(LabelType::STA_MAC));

  {
    uint8_t mac[] = {0, 0, 0, 0, 0, 0};
    uint8_t* macread = WiFi.macAddress(mac);
    char macaddress[20];
    formatMAC(macread, macaddress);
    TXBuffer += macaddress;

    addRowLabel(getLabel(LabelType::AP_MAC));
    macread = WiFi.softAPmacAddress(mac);
    formatMAC(macread, macaddress);
    TXBuffer += macaddress;
  }

  addRowLabel(getLabel(LabelType::SSID));
  TXBuffer += WiFi.SSID();
  TXBuffer += " (";
  TXBuffer += WiFi.BSSIDstr();
  TXBuffer += ')';

  addRowLabelValue(LabelType::CHANNEL);
  addRowLabelValue(LabelType::CONNECTED);
  addRowLabel(getLabel(LabelType::LAST_DISCONNECT_REASON));
  TXBuffer += getValue(LabelType::LAST_DISC_REASON_STR);
  addRowLabelValue(LabelType::NUMBER_RECONNECTS);

  addTableSeparator(F("WiFi Settings"), 2, 3);
  addRowLabelValue(LabelType::FORCE_WIFI_BG);
  addRowLabelValue(LabelType::RESTART_WIFI_LOST_CONN);
#ifdef ESP8266
  addRowLabelValue(LabelType::FORCE_WIFI_NOSLEEP);
#endif
#ifdef SUPPORT_ARP
  addRowLabelValue(LabelType::PERIODICAL_GRAT_ARP);
#endif
  addRowLabelValue(LabelType::CONNECTION_FAIL_THRESH);

  addTableSeparator(F("Firmware"), 2, 3);

  addRowLabelValue_copy(LabelType::BUILD_DESC);
  TXBuffer += ' ';
  TXBuffer += F(BUILD_NOTES);

  addRowLabelValue_copy(LabelType::SYSTEM_LIBRARIES);
  addRowLabelValue_copy(LabelType::GIT_BUILD);
  addRowLabelValue_copy(LabelType::PLUGINS);
  TXBuffer += getPluginDescriptionString();

  bool filenameDummy = String(CRCValues.binaryFilename).startsWith(F("ThisIsTheDummy"));
  if (!filenameDummy) {
    addRowLabel(F("Build Md5"));
    for (byte i = 0; i<16; i++) TXBuffer += String(CRCValues.compileTimeMD5[i],HEX);

     addRowLabel(F("Md5 check"));
    if (! CRCValues.checkPassed())
       TXBuffer += F("<font color = 'red'>fail !</font>");
    else TXBuffer += F("passed.");
  }
  addRowLabel_copy(getLabel(LabelType::BUILD_TIME));
  TXBuffer += String(CRCValues.compileDate);
  TXBuffer += ' ';
  TXBuffer += String(CRCValues.compileTime);

  addRowLabel_copy(getLabel(LabelType::BINARY_FILENAME));
  if (filenameDummy) {
    TXBuffer += F("<b>Self built!</b>");
  } else {
    TXBuffer += String(CRCValues.binaryFilename);
  }

  addTableSeparator(F("System Status"), 2, 3);
  {

    addRowLabelValue(LabelType::SYSLOG_LOG_LEVEL);
    addRowLabelValue(LabelType::SERIAL_LOG_LEVEL);
    addRowLabelValue(LabelType::WEB_LOG_LEVEL);
    #ifdef FEATURE_SD
    addRowLabelValue(LabelType::SD_LOG_LEVEL);
    #endif
  }


  addTableSeparator(F("ESP board"), 2, 3);

  addRowLabel(getLabel(LabelType::ESP_CHIP_ID));
  #if defined(ESP8266)
    TXBuffer += ESP.getChipId();
    TXBuffer += F(" (0x");
    String espChipId(ESP.getChipId(), HEX);
    espChipId.toUpperCase();
    TXBuffer += espChipId;
    TXBuffer += ')';

    addRowLabel(getLabel(LabelType::ESP_CHIP_FREQ));
    TXBuffer += ESP.getCpuFreqMHz();
    TXBuffer += F(" MHz");
  #endif
  #if defined(ESP32)
    TXBuffer += F(" (0x");
    uint64_t chipid=ESP.getEfuseMac();
    uint32_t ChipId1 = (uint16_t)(chipid>>32);
    String espChipIdS(ChipId1, HEX);
    espChipIdS.toUpperCase();
    TXBuffer += espChipIdS;
    ChipId1 = (uint32_t)chipid;
    String espChipIdS1(ChipId1, HEX);
    espChipIdS1.toUpperCase();
    TXBuffer += espChipIdS1;
    TXBuffer += ')';

    addRowLabel(getLabel(LabelType::ESP_CHIP_FREQ));
    TXBuffer += ESP.getCpuFreqMHz();
    TXBuffer += F(" MHz");
  #endif
  #ifdef ARDUINO_BOARD
  addRowLabel(getLabel(LabelType::ESP_BOARD_NAME));
  TXBuffer += ARDUINO_BOARD;
  #endif

  addTableSeparator(F("Storage"), 2, 3);

  addRowLabel(getLabel(LabelType::FLASH_CHIP_ID));
  #if defined(ESP8266)
    uint32_t flashChipId = ESP.getFlashChipId();


    TXBuffer += F("Vendor: ");
    TXBuffer += formatToHex(flashChipId & 0xFF);

    if (flashChipVendorPuya())
    {
      TXBuffer += F(" (PUYA");
      if (puyaSupport()) {
        TXBuffer += F(", supported");
      } else {
        TXBuffer += F(HTML_SYMBOL_WARNING);
      }
      TXBuffer += ')';
    }
    TXBuffer += F(" Device: ");
    uint32_t flashDevice = (flashChipId & 0xFF00) | ((flashChipId >> 16) & 0xFF);
    TXBuffer += formatToHex(flashDevice);
  #endif
  uint32_t realSize = getFlashRealSizeInBytes();
  uint32_t ideSize = ESP.getFlashChipSize();

  addRowLabel(getLabel(LabelType::FLASH_CHIP_REAL_SIZE));
  TXBuffer += realSize / 1024;
  TXBuffer += F(" kB");

  addRowLabel(getLabel(LabelType::FLASH_IDE_SIZE));
  TXBuffer += ideSize / 1024;
  TXBuffer += F(" kB");


  #if defined(ESP8266)
    addRowLabel(getLabel(LabelType::FLASH_IDE_SPEED));
    TXBuffer += ESP.getFlashChipSpeed() / 1000000;
    TXBuffer += F(" MHz");

    FlashMode_t ideMode = ESP.getFlashChipMode();
    addRowLabel(getLabel(LabelType::FLASH_IDE_MODE));
    switch (ideMode) {
      case FM_QIO: TXBuffer += F("QIO"); break;
      case FM_QOUT: TXBuffer += F("QOUT"); break;
      case FM_DIO: TXBuffer += F("DIO"); break;
      case FM_DOUT: TXBuffer += F("DOUT"); break;
      default:
          TXBuffer += getUnknownString(); break;
    }
  #endif

   addRowLabel(getLabel(LabelType::FLASH_WRITE_COUNT));
   TXBuffer += RTC.flashDayCounter;
   TXBuffer += F(" daily / ");
   TXBuffer += RTC.flashCounter;
   TXBuffer += F(" boot");

   addRowLabel(getLabel(LabelType::SKETCH_SIZE));
  #if defined(ESP8266)
   TXBuffer += ESP.getSketchSize() / 1024;
   TXBuffer += F(" kB (");
   TXBuffer += ESP.getFreeSketchSpace() / 1024;
   TXBuffer += F(" kB free)");
  #endif

  addRowLabel(getLabel(LabelType::SPIFFS_SIZE));
  TXBuffer += SpiffsTotalBytes() / 1024;
  TXBuffer += F(" kB (");
  TXBuffer += SpiffsFreeSpace() / 1024;
  TXBuffer += F(" kB free)");

  addRowLabel(F("Page size"));
  TXBuffer += String(SpiffsPagesize());

  addRowLabel(F("Block size"));
  TXBuffer += String(SpiffsBlocksize());

  addRowLabel(F("Number of blocks"));
  TXBuffer += String(SpiffsTotalBytes() / SpiffsBlocksize());

  {
  #if defined(ESP8266)
    fs::FSInfo fs_info;
    SPIFFS.info(fs_info);
    addRowLabel(F("Maximum open files"));
    TXBuffer += String(fs_info.maxOpenFiles);

    addRowLabel(F("Maximum path length"));
    TXBuffer += String(fs_info.maxPathLength);

  #endif
  }

#ifndef BUILD_MINIMAL_OTA
  if (showSettingsFileLayout) {
    addTableSeparator(F("Settings Files"), 2, 3);
    html_TR_TD();
    TXBuffer += F("Layout Settings File");
    html_TD();
    getConfig_dat_file_layout();
    html_TR_TD();
    html_TD();
    TXBuffer += F("(offset / size per item / index)");

    for (int st = 0; st < SettingsType_MAX; ++st) {
      SettingsType settingsType = static_cast<SettingsType>(st);
      html_TR_TD();
      TXBuffer += getSettingsTypeString(settingsType);
      html_TD();
      getStorageTableSVG(settingsType);
    }
  }
#endif

  #ifdef ESP32
   addTableSeparator(F("Partitions"), 2, 3,
     F("https://dl.espressif.com/doc/esp-idf/latest/api-guides/partition-tables.html"));

   addRowLabel(F("Data Partition Table"));


   getPartitionTableSVG(ESP_PARTITION_TYPE_DATA, 0x5856e6);

   addRowLabel(F("App Partition Table"));


   getPartitionTableSVG(ESP_PARTITION_TYPE_APP, 0xab56e6);
  #endif

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}

void addSysVar_html(const String& input) {
  html_TR_TD();
  TXBuffer += F("<pre>");
  TXBuffer += F("<xmp>");
  TXBuffer += input;
  TXBuffer += F("</xmp>");
  TXBuffer += F("</pre>");
  html_TD();
  String replacement(input);
  parseSystemVariables(replacement, false);
  parseStandardConversions(replacement, false);
  TXBuffer += replacement;
  html_TD();
  replacement = input;
  parseSystemVariables(replacement, true);
  parseStandardConversions(replacement, true);
  TXBuffer += replacement;
  delay(0);
}

#ifdef WEBSERVER_SYSVARS



void handle_sysvars() {
  checkRAM(F("handle_sysvars"));
  if (!isLoggedIn()) return;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  addHeader(true, TXBuffer.buf);

  html_BR();
  TXBuffer += F("<p>This page may load slow.<BR>Do not load too often, since it may affect performance of the node.</p>");
  html_BR();


  html_table_class_normal();
  html_TR();
  html_table_header(F("System Variables"));
  html_table_header(F("Normal"));
  html_table_header(F("URL encoded"), F("ESPEasy_System_Variables"), 0);

  addTableSeparator(F("Constants"), 3, 3);
  addSysVar_html(F("%CR%"));
  addSysVar_html(F("%LF%"));
  addSysVar_html(F("%SP%"));
  addSysVar_html(F("%R%"));
  addSysVar_html(F("%N%"));

  addTableSeparator(F("Network"), 3, 3);
  addSysVar_html(F("%mac%"));
#if defined(ESP8266)
  addSysVar_html(F("%mac_int%"));
#endif
  addSysVar_html(F("%ip4%"));
  addSysVar_html(F("%ip%"));
  addSysVar_html(F("%rssi%"));
  addSysVar_html(F("%ssid%"));
  addSysVar_html(F("%bssid%"));
  addSysVar_html(F("%wi_ch%"));

  addTableSeparator(F("System"), 3, 3);
  addSysVar_html(F("%unit%"));
  addSysVar_html(F("%sysload%"));
  addSysVar_html(F("%sysheap%"));
  addSysVar_html(F("%sysname%"));
#if FEATURE_ADC_VCC
  addSysVar_html(F("%vcc%"));
#endif

  addTableSeparator(F("System status"), 3, 3);

  addSysVar_html(F("%iswifi%"));
  addSysVar_html(F("%isntp%"));
  addSysVar_html(F("%ismqtt%"));
#ifdef USES_P037
  addSysVar_html(F("%ismqttimp%"));
#endif

  addTableSeparator(F("Time"), 3, 3);
  addSysVar_html(F("%lcltime%"));
  addSysVar_html(F("%lcltime_am%"));
  addSysVar_html(F("%systm_hm%"));
  addSysVar_html(F("%systm_hm_am%"));
  addSysVar_html(F("%systime%"));
  addSysVar_html(F("%systime_am%"));
  addSysVar_html(F("%sysbuild_date%"));
  addSysVar_html(F("%sysbuild_time%"));
  addTableSeparator(F("System"), 3, 3);
  addSysVar_html(F("%sysyear%  // %sysyear_0%"));
  addSysVar_html(F("%sysyears%"));
  addSysVar_html(F("%sysmonth% // %sysmonth_0%"));
  addSysVar_html(F("%sysday%   // %sysday_0%"));
  addSysVar_html(F("%syshour%  // %syshour_0%"));
  addSysVar_html(F("%sysmin%   // %sysmin_0%"));
  addSysVar_html(F("%syssec%   // %syssec_0%"));
  addSysVar_html(F("%syssec_d%"));
  addSysVar_html(F("%sysweekday%"));
  addSysVar_html(F("%sysweekday_s%"));
  addTableSeparator(F("System"), 3, 3);
  addSysVar_html(F("%uptime%"));
  addSysVar_html(F("%unixtime%"));
  addSysVar_html(F("%sunset%"));
  addSysVar_html(F("%sunset-1h%"));
  addSysVar_html(F("%sunrise%"));
  addSysVar_html(F("%sunrise+10m%"));

  addTableSeparator(F("Custom Variables"), 3, 3);
  for (byte i = 0; i < CUSTOM_VARS_MAX; ++i) {
    addSysVar_html("%v"+toString(i+1,0)+'%');
  }

  addTableSeparator(F("Special Characters"), 3, 2);
  addTableSeparator(F("Degree"), 3, 3);
  addSysVar_html(F("{D}"));
  addSysVar_html(F("&deg;"));

  addTableSeparator(F("Angle quotes"), 3, 3);
  addSysVar_html(F("{<<}"));
  addSysVar_html(F("&laquo;"));
  addFormSeparator(3);
  addSysVar_html(F("{>>}"));
  addSysVar_html(F("&raquo;"));
  addTableSeparator(F("Greek letter Mu"), 3, 3);
  addSysVar_html(F("{u}"));
  addSysVar_html(F("&micro;"));
  addTableSeparator(F("Currency"), 3, 3);
  addSysVar_html(F("{E}"));
  addSysVar_html(F("&euro;"));
  addFormSeparator(3);
  addSysVar_html(F("{Y}"));
  addSysVar_html(F("&yen;"));
  addFormSeparator(3);
  addSysVar_html(F("{P}"));
  addSysVar_html(F("&pound;"));
  addFormSeparator(3);
  addSysVar_html(F("{c}"));
  addSysVar_html(F("&cent;"));

  addTableSeparator(F("Math symbols"), 3, 3);
  addSysVar_html(F("{^1}"));
  addSysVar_html(F("&sup1;"));
  addFormSeparator(3);
  addSysVar_html(F("{^2}"));
  addSysVar_html(F("&sup2;"));
  addFormSeparator(3);
  addSysVar_html(F("{^3}"));
  addSysVar_html(F("&sup3;"));
  addFormSeparator(3);
  addSysVar_html(F("{1_4}"));
  addSysVar_html(F("&frac14;"));
  addFormSeparator(3);
  addSysVar_html(F("{1_2}"));
  addSysVar_html(F("&frac12;"));
  addFormSeparator(3);
  addSysVar_html(F("{3_4}"));
  addSysVar_html(F("&frac34;"));
  addFormSeparator(3);
  addSysVar_html(F("{+-}"));
  addSysVar_html(F("&plusmn;"));
  addFormSeparator(3);
  addSysVar_html(F("{x}"));
  addSysVar_html(F("&times;"));
  addFormSeparator(3);
  addSysVar_html(F("{..}"));
  addSysVar_html(F("&divide;"));

  addTableSeparator(F("Standard Conversions"), 3, 2);

  addSysVar_html(F("Wind Dir.:    %c_w_dir%(123.4)"));
  addSysVar_html(F("{D}C to {D}F: %c_c2f%(20.4)"));
  addSysVar_html(F("m/s to Bft:   %c_ms2Bft%(5.1)"));
  addSysVar_html(F("Dew point(T,H): %c_dew_th%(18.6,67)"));
  addFormSeparator(3);
  addSysVar_html(F("cm to imperial: %c_cm2imp%(190)"));
  addSysVar_html(F("mm to imperial: %c_mm2imp%(1900)"));
  addFormSeparator(3);
  addSysVar_html(F("Mins to days: %c_m2day%(1900)"));
  addSysVar_html(F("Mins to dh:   %c_m2dh%(1900)"));
  addSysVar_html(F("Mins to dhm:  %c_m2dhm%(1900)"));
  addSysVar_html(F("Secs to dhms: %c_s2dhms%(100000)"));

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}
#endif




String URLEncode(const char* msg)
{
  const char *hex = "0123456789abcdef";
  String encodedMsg = "";

  while (*msg != '\0') {
    if ( ('a' <= *msg && *msg <= 'z')
         || ('A' <= *msg && *msg <= 'Z')
         || ('0' <= *msg && *msg <= '9')
         || ('-' == *msg) || ('_' == *msg)
         || ('.' == *msg) || ('~' == *msg) ) {
      encodedMsg += *msg;
    } else {
      encodedMsg += '%';
      encodedMsg += hex[*msg >> 4];
      encodedMsg += hex[*msg & 15];
    }
    msg++;
  }
  return encodedMsg;
}


String getControllerSymbol(byte index)
{
  String ret = F("<p style='font-size:20px'>&#");
  ret += 10102 + index;
  ret += F(";</p>");
  return ret;
}

String getValueSymbol(byte index)
{
  String ret = F("&#");
  ret += 10112 + index;
  ret += ';';
  return ret;
}


void handle_favicon() {
  checkRAM(F("handle_favicon"));
  WebServer.send_P(200, PSTR("image/x-icon"), favicon_8b_ico, favicon_8b_ico_len);
}

void createSvgRectPath(unsigned int color, int xoffset, int yoffset, int size, int height, int range, float SVG_BAR_WIDTH) {
  float width = SVG_BAR_WIDTH * size / range;
  if (width < 2) width = 2;
  TXBuffer += formatToHex(color, F("<path fill=\"#"));
  TXBuffer += F("\" d=\"M");
  TXBuffer += toString(SVG_BAR_WIDTH * xoffset / range, 2);
  TXBuffer += ' ';
  TXBuffer += yoffset;
  TXBuffer += 'h';
  TXBuffer += toString(width, 2);
  TXBuffer += 'v';
  TXBuffer += height;
  TXBuffer += 'H';
  TXBuffer += toString(SVG_BAR_WIDTH * xoffset / range, 2);
  TXBuffer += F("z\"/>\n");
}

void createSvgTextElement(const String& text, float textXoffset, float textYoffset) {
  TXBuffer += F("<text style=\"line-height:1.25\" x=\"");
  TXBuffer += toString(textXoffset, 2);
  TXBuffer += F("\" y=\"");
  TXBuffer += toString(textYoffset, 2);
  TXBuffer += F("\" stroke-width=\".3\" font-family=\"sans-serif\" font-size=\"8\" letter-spacing=\"0\" word-spacing=\"0\">\n");
  TXBuffer += F("<tspan x=\"");
  TXBuffer += toString(textXoffset, 2);
  TXBuffer += F("\" y=\"");
  TXBuffer += toString(textYoffset, 2);
  TXBuffer += "\">";
  TXBuffer += text;
  TXBuffer += F("</tspan>\n</text>");
}

unsigned int getSettingsTypeColor(SettingsType settingsType) {
  switch (settingsType) {
    case BasicSettings_Type:
      return 0x5F0A87;
    case TaskSettings_Type:
      return 0xEE6352;
    case CustomTaskSettings_Type:
      return 0x59CD90;
    case ControllerSettings_Type:
      return 0x3FA7D6;
    case CustomControllerSettings_Type:
      return 0xFAC05E;
    case NotificationSettings_Type:
      return 0xF79D84;
    default:
      break;
  }
  return 0;
}

#define SVG_BAR_HEIGHT 16
#define SVG_BAR_WIDTH 400

void write_SVG_image_header(int width, int height) {
  write_SVG_image_header(width, height, false);
}

void write_SVG_image_header(int width, int height, bool useViewbox) {
  TXBuffer += F("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"");
  TXBuffer += width;
  TXBuffer += F("\" height=\"");
  TXBuffer += height;
  TXBuffer += F("\" version=\"1.1\"");
  if (useViewbox) {
    TXBuffer += F(" viewBox=\"0 0 100 100\"");
  }
  TXBuffer += '>';
}
# 7489 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer.ino"
#ifndef BUILD_MINIMAL_OTA
void getConfig_dat_file_layout() {
  const int shiftY = 2;
  float yOffset = shiftY;
  write_SVG_image_header(SVG_BAR_WIDTH + 250, SVG_BAR_HEIGHT + shiftY);

  int max_index, offset, max_size;
  int struct_size = 0;


  const uint32_t realSize = getFileSize(TaskSettings_Type);
  createSvgRectPath(0xcdcdcd, 0, yOffset, realSize, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);

  for (int st = 0; st < SettingsType_MAX; ++st) {
    SettingsType settingsType = static_cast<SettingsType>(st);
    if (settingsType != NotificationSettings_Type) {
      unsigned int color = getSettingsTypeColor(settingsType);
      getSettingsParameters(settingsType, 0, max_index, offset, max_size, struct_size);
      for (int i = 0; i < max_index; ++i) {
        getSettingsParameters(settingsType, i, offset, max_size);

        createSvgRectPath(color, offset, yOffset, max_size, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);
      }
    }
  }

  float textXoffset = SVG_BAR_WIDTH + 2;
  float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;
  createSvgTextElement(F("Config.dat"), textXoffset, textYoffset);
  TXBuffer += F("</svg>\n");
}

void getStorageTableSVG(SettingsType settingsType) {
  uint32_t realSize = getFileSize(settingsType);
  unsigned int color = getSettingsTypeColor(settingsType);
  const int shiftY = 2;

  int max_index, offset, max_size;
  int struct_size = 0;
  getSettingsParameters(settingsType, 0, max_index, offset, max_size, struct_size);
  if (max_index == 0) return;

  write_SVG_image_header(SVG_BAR_WIDTH + 250, (max_index + 1) * SVG_BAR_HEIGHT + shiftY);
  float yOffset = shiftY;
  for (int i = 0; i < max_index; ++i) {
    getSettingsParameters(settingsType, i, offset, max_size);

    createSvgRectPath(0xcdcdcd, 0, yOffset, realSize, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);

    createSvgRectPath(color, offset, yOffset, max_size, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);

    float textXoffset = SVG_BAR_WIDTH + 2;
    float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;
    createSvgTextElement(formatHumanReadable(offset, 1024), textXoffset, textYoffset);
    textXoffset = SVG_BAR_WIDTH + 60;
    createSvgTextElement(formatHumanReadable(max_size, 1024), textXoffset, textYoffset);
    textXoffset = SVG_BAR_WIDTH + 130;
    createSvgTextElement(String(i), textXoffset, textYoffset);
    yOffset += SVG_BAR_HEIGHT;
  }

  createSvgRectPath(0xcdcdcd, 0, yOffset, max_size, SVG_BAR_HEIGHT - 2, max_size, SVG_BAR_WIDTH);

  if (struct_size != 0) {
    createSvgRectPath(color, 0, yOffset, struct_size, SVG_BAR_HEIGHT - 2, max_size, SVG_BAR_WIDTH);
  }

  float textXoffset = SVG_BAR_WIDTH + 2;
  float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;
  if (struct_size != 0) {
    String text = formatHumanReadable(struct_size, 1024);
    text += '/';
    text += formatHumanReadable(max_size, 1024);
    text += F(" per item");
    createSvgTextElement(text, textXoffset, textYoffset);
  } else {
    createSvgTextElement(F("Variable size"), textXoffset, textYoffset);
  }
  TXBuffer += F("</svg>\n");
}
#endif

#ifdef ESP32


int getPartionCount(byte pType) {
  esp_partition_type_t partitionType = static_cast<esp_partition_type_t>(pType);
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, ESP_PARTITION_SUBTYPE_ANY, NULL);
  int nrPartitions = 0;
  if (_mypartiterator) {
    do {
      ++nrPartitions;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != NULL);
  }
  esp_partition_iterator_release(_mypartiterator);
  return nrPartitions;
}

void getPartitionTableSVG(byte pType, unsigned int partitionColor) {
  int nrPartitions = getPartionCount(pType);
  if (nrPartitions == 0) return;
  const int shiftY = 2;

  uint32_t realSize = getFlashRealSizeInBytes();
  esp_partition_type_t partitionType = static_cast<esp_partition_type_t>(pType);
  const esp_partition_t * _mypart;
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, ESP_PARTITION_SUBTYPE_ANY, NULL);
  write_SVG_image_header(SVG_BAR_WIDTH + 250, nrPartitions * SVG_BAR_HEIGHT + shiftY);
  float yOffset = shiftY;
  if (_mypartiterator) {
    do {
      _mypart = esp_partition_get(_mypartiterator);
      createSvgRectPath(0xcdcdcd, 0, yOffset, realSize, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);
      createSvgRectPath(partitionColor, _mypart->address, yOffset, _mypart->size, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);
      float textXoffset = SVG_BAR_WIDTH + 2;
      float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;
      createSvgTextElement(formatHumanReadable(_mypart->size, 1024), textXoffset, textYoffset);
      textXoffset = SVG_BAR_WIDTH + 60;
      createSvgTextElement(_mypart->label, textXoffset, textYoffset);
      textXoffset = SVG_BAR_WIDTH + 130;
      createSvgTextElement(getPartitionType(_mypart->type, _mypart->subtype), textXoffset, textYoffset);
      yOffset += SVG_BAR_HEIGHT;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != NULL);
  }
  TXBuffer += F("</svg>\n");
  esp_partition_iterator_release(_mypartiterator);
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/WebServer_Rules.ino"
#ifndef WEBSERVER_RULES_DEBUG
  #define WEBSERVER_RULES_DEBUG 1
#endif
#ifdef BUILD_MINIMAL_OTA
  #undef WEBSERVER_RULES_DEBUG
#endif




void handle_rules_new() {
  if (!isLoggedIn() || !Settings.UseRules) return;
  if (!clientIPallowed()) return;
  if (Settings.OldRulesEngine())
  {
    handle_rules();
    return;
  }
  checkRAM(F("handle_rules"));
  navMenuIndex = 5;
  TXBuffer.startStream();
  sendHeadandTail(F("TmplStd"),_HEAD);


  #if defined(ESP8266)
    String rootPath = F("rules");
  #endif
  #if defined(ESP32)

    String rootPath = F("/rules");
  #endif


  const int rulesListPageSize = 25;
  int startIdx = 0;

  String fstart = WebServer.arg(F("start"));
  if (fstart.length() > 0)
  {
    startIdx = atoi(fstart.c_str());
  }
  int endIdx = startIdx + rulesListPageSize - 1;


  html_table_class_multirow();
  html_TR();
  html_table_header(F("Event Name"));
  html_table_header(F("Filename"));
  html_table_header(F("Size"));
  TXBuffer += F("<TH>Actions");
  addSaveButton(TXBuffer,F("/rules/backup"), F("Backup"));
  TXBuffer += F("</TH></TR>");



  int count = -1;
  HandlerFileInfo renderDetail = [ &count,endIdx](fileInfo fi){
    #ifdef WEBSERVER_RULES_DEBUG
    Serial.print(F("Start generation of: "));
    Serial.println(fi.Name);
    #endif
    if (fi.isDirectory)
    {
      TXBuffer += F("<TR><TD>");
    }
    else
    {
      count++;
      TXBuffer += F("<TR><TD style='text-align:right'>");
    }

    TXBuffer += FileNameToEvent(fi.Name);
    if (fi.isDirectory)
    {
      TXBuffer += F("</TD><TD></TD><TD></TD><TD>");
      addSaveButton(TXBuffer
        , String(F("/rules/backup?directory=")) + URLEncode(fi.Name.c_str())
        , F("Backup")
        );
    }
    else
    {
      String encodedPath = URLEncode((fi.Name + F(".txt")).c_str());

      TXBuffer += F("</TD><TD><a href='");
      TXBuffer += fi.Name;
      TXBuffer += F(".txt");
      TXBuffer += "'>";
      TXBuffer += fi.Name;
      TXBuffer += F(".txt");
      TXBuffer += F("</a></TD>");


      TXBuffer += F("<TD>");
      TXBuffer += fi.Size;
      TXBuffer += F("</TD>");

      TXBuffer += F("<TD>");
      addSaveButton(TXBuffer
        , String(F("/rules/backup?fileName=")) + encodedPath
        , F("Backup")
        );

      addDeleteButton(TXBuffer
        , String(F("/rules/delete?fileName=")) + encodedPath
        , F("Delete")
        );

    }
    TXBuffer += F("</TD></TR>");
    #ifdef WEBSERVER_RULES_DEBUG
    Serial.print(F("End generation of: "));
    Serial.println(fi.Name);
    #endif

    return count <= endIdx;
  };


  bool hasMore = EnumerateFileAndDirectory(rootPath
    , startIdx
    , renderDetail);
  TXBuffer += F("<TR><TD>");
  addButton(F("/rules/add"),F("Add"));
  TXBuffer += F("</TD><TD></TD><TD></TD><TD></TD></TR>");
  TXBuffer += F("</table>");

  if (startIdx > 0)
  {
    int showIdx = startIdx - rulesListPageSize;
    if (showIdx < 0) showIdx = 0;
    addButton(TXBuffer
      , String(F("/rules?start=")) + String(showIdx)
      , F("Previous"));
  }
  if (hasMore && count >= endIdx)
  {
    addButton(TXBuffer
      , String(F("/rules?start=")) + String(endIdx + 1)
      , F("Next"));
  }

  sendHeadandTail(F("TmplStd"),_TAIL);
  TXBuffer.endStream();
  checkRAM(F("handle_rules"));
}

void handle_rules_backup() {
  if(Settings.OldRulesEngine())
  {
    Goto_Rules_Root();
    return;
  }
  #ifdef WEBSERVER_RULES_DEBUG
  Serial.println(F("handle rules backup"));
  #endif
  if (!isLoggedIn() || !Settings.UseRules) return;
  if (!clientIPallowed()) return;
  checkRAM(F("handle_rules_backup"));
  String directory = WebServer.arg(F("directory"));
  String fileName = WebServer.arg(F("fileName"));
  String error;
  if (directory.length() > 0)
  {
    HandlerFileInfo downloadHandler = [&error](fileInfo fi){
      if(!fi.isDirectory)
      {
         if(!Rule_Download(fi.Name))
         {
            error += String(F("Invalid path: ")) + fi.Name;
         }
      }
      return true;
    };
    EnumerateFileAndDirectory(directory
      , 0
      , downloadHandler);
  }
  else if(fileName.length() > 0 ) {
    fileName = fileName.substring(0,fileName.length()-4);
    if(!Rule_Download(fileName))
    {
      error = String(F("Invalid path: ")) + fileName;
    }
  }
  else
  {
    error = F("Invalid parameters");
  }
  if (error.length() > 0) {
    TXBuffer.startStream();
    sendHeadandTail(F("TmplMsg"),_HEAD);
    addHtmlError(error);
    TXBuffer.endStream();
  }

  checkRAM(F("handle_rules_backup"));
}

void handle_rules_delete() {
  if (!isLoggedIn() || !Settings.UseRules) return;
  if (!clientIPallowed()) return;
  if(Settings.OldRulesEngine())
  {
    Goto_Rules_Root();
    return;
  }
  checkRAM(F("handle_rules_delete"));
  String fileName = WebServer.arg(F("fileName"));
  fileName = fileName.substring(0,fileName.length()-4);
  bool removed = false;
  #ifdef WEBSERVER_RULES_DEBUG
  Serial.println(F("handle_rules_delete"));
  Serial.print(F("File name: "));
  Serial.println(fileName);
  #endif

  if(fileName.length() > 0 )
  {
    removed = SPIFFS.remove(fileName);
  }
  if(removed)
  {
    WebServer.sendHeader(F("Location"), F("/rules"),true);
    WebServer.send(302, F("text/plain"),F(""));
  }
  else
  {
    String error = String(F("Delete rule Invalid path: ")) + fileName;
    addLog(LOG_LEVEL_ERROR, error);
    TXBuffer.startStream();
    sendHeadandTail(F("TmplMsg"),_HEAD);
    addHtmlError(error);
    sendHeadandTail(F("TmplMsg"),_TAIL);
    TXBuffer.endStream();
  }
  checkRAM(F("handle_rules_delete"));
}

bool handle_rules_edit(const String& originalUri)
{
  return handle_rules_edit(originalUri, false);
}

bool handle_rules_edit(String originalUri, bool isAddNew) {

  if (!isLoggedIn() || !Settings.UseRules) return false;
  originalUri.toLowerCase();
  checkRAM(F("handle_rules"));
  bool handle = false;

  #ifdef WEBSERVER_RULES_DEBUG
  Serial.println(originalUri);
  Serial.println(F("handle_rules_edit"));
  #endif

  if(isAddNew || (originalUri.startsWith(F("/rules/"))
    && originalUri.endsWith(F(".txt")))){

      if(Settings.OldRulesEngine())
      {
        Goto_Rules_Root();
        return true;
      }

      String eventName;
      String fileName;
      bool isOverwrite = false;
      bool isNew = false;
      String rules;
      String error;


      if(isAddNew)
      {
        eventName = WebServer.arg(F("eventName"));
        fileName += EventToFileName(eventName);
      }
      else
      {
        #if defined(ESP8266)
        fileName = F("rules/");
        #endif
        #if defined(ESP32)
        fileName = F("/rules/");
        #endif
        fileName += originalUri.substring(7,originalUri.length()-4);
        eventName = FileNameToEvent(fileName);
      }
      #ifdef WEBSERVER_RULES_DEBUG
      Serial.print(F("File name: "));
      Serial.println(fileName);
      #endif
      bool isEdit = SPIFFS.exists(fileName);
      if (WebServer.args() > 0)
      {
        rules = WebServer.arg(F("rules"));
        isNew = WebServer.arg(F("IsNew")) == F("yes");

        if (isEdit && isNew) {
          error = String(F("There is another rule with the same name."))
            + fileName;
          addLog(LOG_LEVEL_ERROR, error);
          isAddNew = true;
          isOverwrite = true;
        }

        else if (rules.length() > RULES_MAX_SIZE)
        {
          error = String(F("Data was not saved, exceeds web editor limit! "))
            + fileName;
          addLog(LOG_LEVEL_ERROR, error);
        }

        else
        {
          fs::File f = tryOpenFile(fileName, "w");
          if (f)
          {
            addLog(LOG_LEVEL_INFO, String(F(" Write to file: ")) + fileName);
            f.print(rules);
            f.close();
          }
          if (isAddNew) {
            WebServer.sendHeader(F("Location"), F("/rules"),true);
            WebServer.send(302, F("text/plain"),F(""));
            return true;
          }
        }
      }
      navMenuIndex = 5;
      TXBuffer.startStream();
      sendHeadandTail(F("TmplStd"));
      if (error.length()>0) {
        addHtmlError(error);
      }
      TXBuffer += F("<form name = 'editRule' method = 'post'><table class='normal'><TR><TH align='left' colspan='2'>Edit Rule");

      TXBuffer += F("<input type='hidden' id='IsNew' name='IsNew' value='");
      TXBuffer += isAddNew
        ? F("yes")
        : F("no");
      TXBuffer += F("'>");

      bool isReadOnly = !isOverwrite && ((isEdit && !isAddNew && !isNew) || (isAddNew && isNew));
      #ifdef WEBSERVER_RULES_DEBUG
      Serial.print(F("Is Overwrite: "));
      Serial.println(isOverwrite);
      Serial.print(F("Is edit: "));
      Serial.println(isEdit);
      Serial.print(F("Is addnew: "));
      Serial.println(isAddNew);
      Serial.print(F("Is New: "));
      Serial.println(isNew);
      Serial.print(F("Is Read Only: "));
      Serial.println(isReadOnly);
      #endif

      addFormTextBox(F("Event name")
        , F("eventName")
        , eventName
        , RULE_MAX_FILENAME_LENGTH
        , isReadOnly
        , isAddNew
      , F("[A-Za-z]+#.+")
      );
      addHelpButton(F("Tutorial_Rules"));


      TXBuffer += F("<TR><TD colspan='2'>");
      int size = 0;
      if(!isOverwrite)
      {
        rules = "";
        fs::File f = tryOpenFile(fileName, "r");
        if (f)
        {
          size = f.size();
          if (size < RULES_MAX_SIZE)
          {
            rules.reserve(size);
            while (f.available())
            {
              rules += (char)f.read();
            }
          }
          f.close();
        }
      }
      if (size > RULES_MAX_SIZE)
        TXBuffer += F("<span style=\"color:red\">Filesize exceeds web editor limit!</span>");
      else
      {
        TXBuffer += F("<textarea name='rules' rows='30' wrap='off'>");
        String c(rules);
        htmlEscape(c);
        TXBuffer += c;
        TXBuffer += F("</textarea>");
      }

      TXBuffer += F("<TR><TD colspan='2'>");

      html_TR_TD(); TXBuffer += F("Current size: ");
      TXBuffer += size;
      TXBuffer += F(" characters (Max ");
      TXBuffer += RULES_MAX_SIZE;
      TXBuffer += F(")");
      addFormSeparator(2);
      html_TR_TD();
      addSubmitButton();

      TXBuffer += F("</table></form>");

      sendHeadandTail(F("TmplStd"),true);
      TXBuffer.endStream();
  }

  checkRAM(F("handle_rules"));
  return handle;
}

bool Rule_Download(const String& path)
{
  #ifdef WEBSERVER_RULES_DEBUG
  Serial.print(F("Rule_Download path: "));
  Serial.println(path);
  #endif
  fs::File dataFile = tryOpenFile(path, "r");
  if (!dataFile)
  {
    addLog(LOG_LEVEL_ERROR, String(F("Invalid path: ")) + path);
    return false;
  }
  String filename = path + String(F(".txt"));
  filename.replace(RULE_FILE_SEPARAROR,'_');
  String str = String(F("attachment; filename=")) + filename;
  WebServer.sendHeader(F("Content-Disposition"), str);
  WebServer.sendHeader(F("Cache-Control"), F("max-age=3600, public"));
  WebServer.sendHeader(F("Vary"),"*");
  WebServer.sendHeader(F("ETag"), F("\"2.0.0\""));

  WebServer.streamFile(dataFile, F("application/octet-stream"));
  dataFile.close();
  return true;
}

void Goto_Rules_Root() {
  WebServer.sendHeader(F("Location"), F("/rules"),true);
  WebServer.send(302, F("text/plain"),F(""));
}

bool EnumerateFileAndDirectory(String& rootPath
  , int skip
  , HandlerFileInfo handler)
{
  bool hasMore = false;
  int count = 0;
  bool next = true;
  #ifdef ESP8266
  fs::Dir dir = SPIFFS.openDir(rootPath);
  Serial.print(F("Enumerate files of "));
  Serial.println(rootPath);
  while (next && dir.next()) {

    if (count++ < skip) {
      continue;
    }

    if(!dir.fileName().startsWith(rootPath + "/" ))
      continue;
    fileInfo fi;
    fi.Name = dir.fileName() ;
    fs::File f = dir.openFile("r");
    fi.Size = f.size();
    f.close();
    next = handler(fi);
  }
  hasMore = dir.next();
  #endif
  #ifdef ESP32
  File root = SPIFFS.open(rootPath);
  if (root)
  {
    File file = root.openNextFile();
    while (next && file) {
      if (count > skip) {
        fileInfo fi;
        fi.Name = file.name();
        fi.Size = file.size();
        fi.isDirectory = file.isDirectory();
        next = handler(fi);
      }
      if(!file.isDirectory())
        ++count;
      file = root.openNextFile();
    }
  }
  else
  {
    addLog(LOG_LEVEL_ERROR, F("Invalid root."));
  }
  #endif
  return hasMore;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C001.ino"
#ifdef USES_C001




#define CPLUGIN_001 
#define CPLUGIN_ID_001 1
#define CPLUGIN_NAME_001 "Domoticz HTTP"


bool CPlugin_001(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_001;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 8080;
        Protocol[protocolCount].usesID = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_001);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        C001_DelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        if (event->idx != 0)
        {

          String url;

          switch (event->sensorType)
          {
            case SENSOR_TYPE_SWITCH:
              url = F("/json.htm?type=command&param=switchlight&idx=");
              url += event->idx;
              url += F("&switchcmd=");
              if (UserVar[event->BaseVarIndex] == 0)
                url += F("Off");
              else
                url += F("On");
              break;
            case SENSOR_TYPE_DIMMER:
              url = F("/json.htm?type=command&param=switchlight&idx=");
              url += event->idx;
              url += F("&switchcmd=");
              if (UserVar[event->BaseVarIndex] == 0) {
                url += ("Off");
              } else {
                url += F("Set%20Level&level=");
                url += UserVar[event->BaseVarIndex];
              }
              break;

            case SENSOR_TYPE_SINGLE:
            case SENSOR_TYPE_LONG:
            case SENSOR_TYPE_DUAL:
            case SENSOR_TYPE_TRIPLE:
            case SENSOR_TYPE_QUAD:
            case SENSOR_TYPE_TEMP_HUM:
            case SENSOR_TYPE_TEMP_BARO:
            case SENSOR_TYPE_TEMP_EMPTY_BARO:
            case SENSOR_TYPE_TEMP_HUM_BARO:
            case SENSOR_TYPE_WIND:
            default:
              url = F("/json.htm?type=command&param=udevice&idx=");
              url += event->idx;
              url += F("&nvalue=0");
              url += F("&svalue=");
              url += formatDomoticzSensorType(event);
              break;
          }


          url += F("&rssi=");
          url += mapRSSItoDomoticz();
          #if FEATURE_ADC_VCC
            url += F("&battery=");
            url += mapVccToDomoticz();
          #endif

          success = C001_DelayHandler.addToQueue(C001_queue_element(event->ControllerIndex, url));
          scheduleNextDelayQueue(TIMER_C001_DELAY_QUEUE, C001_DelayHandler.getNextScheduleTime());
        }
        else
        {
          addLog(LOG_LEVEL_ERROR, F("HTTP : IDX cannot be zero!"));
        }
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c001_delay_queue();
        delay(0);
        break;
      }
  }
  return success;
}

bool do_process_c001_delay_queue(int controller_number, const C001_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;


  String request = create_http_request_auth(controller_number, element.controller_idx, ControllerSettings, F("GET"), element.txt);

#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG, element.txt);
#endif
  return send_via_http(controller_number, client, request, ControllerSettings.MustCheckReply);
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C002.ino"
#ifdef USES_C002





#define CPLUGIN_002 
#define CPLUGIN_ID_002 2
#define CPLUGIN_NAME_002 "Domoticz MQTT"

#include <ArduinoJson.h>

bool CPlugin_002(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
    {
      Protocol[++protocolCount].Number = CPLUGIN_ID_002;
      Protocol[protocolCount].usesMQTT = true;
      Protocol[protocolCount].usesTemplate = true;
      Protocol[protocolCount].usesAccount = true;
      Protocol[protocolCount].usesPassword = true;
      Protocol[protocolCount].defaultPort = 1883;
      Protocol[protocolCount].usesID = true;
      break;
    }

    case CPLUGIN_GET_DEVICENAME:
    {
      string = F(CPLUGIN_NAME_002);
      break;
    }

    case CPLUGIN_INIT:
    {
      MakeControllerSettings(ControllerSettings);
      LoadControllerSettings(event->ControllerIndex, ControllerSettings);
      MQTTDelayHandler.configureControllerSettings(ControllerSettings);
      break;
    }

    case CPLUGIN_PROTOCOL_TEMPLATE:
    {
      event->String1 = F("domoticz/out");
      event->String2 = F("domoticz/in");
      break;
    }

    case CPLUGIN_PROTOCOL_RECV:
    {




      byte ControllerID = findFirstEnabledControllerWithId(CPLUGIN_ID_002);

      if (ControllerID < CONTROLLER_MAX) {
        DynamicJsonDocument root(512);
        deserializeJson(root, event->String2.c_str());

        if (!root.isNull())
        {
          unsigned int idx = root[F("idx")];
          float nvalue = root[F("nvalue")];
          long nvaluealt = root[F("nvalue")];



          const char *svalue1 = root[F("svalue1")];



          const char *switchtype = root[F("switchType")];

          if (nvalue == 0) {
            nvalue = nvaluealt;
          }

          if ((int)switchtype == 0) {
            switchtype = "?";
          }

          for (byte x = 0; x < TASKS_MAX; x++) {

            if (Settings.TaskDeviceEnabled[x] && (Settings.TaskDeviceID[ControllerID][x] == idx))
            {
              String action = "";

              switch (Settings.TaskDeviceNumber[x]) {
                case 1:
                {
                  action = F("inputSwitchState,");
                  action += x;
                  action += ',';
                  action += nvalue;
                  break;
                }
                case 29:
                {
                  action = "";
                  int baseVar = x * VARS_PER_TASK;

                  if (strcasecmp_P(switchtype, PSTR("dimmer")) == 0)
                  {
                    int pwmValue = UserVar[baseVar];
                    action = F("pwm,");
                    action += Settings.TaskDevicePin1[x];
                    action += ',';

                    switch ((int)nvalue)
                    {
                      case 0:
                        pwmValue = 0;
                        UserVar[baseVar] = pwmValue;
                        break;
                      case 1:
                        pwmValue = UserVar[baseVar];
                        break;
                      case 2:
                        pwmValue = 10 * atol(svalue1);
                        UserVar[baseVar] = pwmValue;
                        break;
                    }
                    action += pwmValue;
                  } else {
                    UserVar[baseVar] = nvalue;
                    action = F("gpio,");
                    action += Settings.TaskDevicePin1[x];
                    action += ',';
                    action += nvalue;
                  }
                  break;
                }
                default:
                  break;
              }

              if (action.length() > 0) {
                struct EventStruct TempEvent;
                TempEvent.TaskIndex = x;
                parseCommandString(&TempEvent, action);
                PluginCall(PLUGIN_WRITE, &TempEvent, action);


                if (Settings.UseRules) {
                  createRuleEvents(x);
                }
              }
            }
          }
          LoadTaskSettings(event->TaskIndex);
        }
      }
      break;
    }

    case CPLUGIN_PROTOCOL_SEND:
    {
      if (event->idx != 0)
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
# 174 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C002.ino"
        DynamicJsonDocument root(200);
        root[F("idx")] = event->idx;
        root[F("RSSI")] = mapRSSItoDomoticz();
          #if FEATURE_ADC_VCC
        root[F("Battery")] = mapVccToDomoticz();
          #endif

        switch (event->sensorType)
        {
          case SENSOR_TYPE_SWITCH:
            root[F("command")] = String(F("switchlight"));

            if (UserVar[event->BaseVarIndex] == 0) {
              root[F("switchcmd")] = String(F("Off"));
            }
            else {
              root[F("switchcmd")] = String(F("On"));
            }
            break;
          case SENSOR_TYPE_DIMMER:
            root[F("command")] = String(F("switchlight"));

            if (UserVar[event->BaseVarIndex] == 0) {
              root[F("switchcmd")] = String(F("Off"));
            }
            else {
              root[F("Set%20Level")] = UserVar[event->BaseVarIndex];
            }
            break;

          case SENSOR_TYPE_SINGLE:
          case SENSOR_TYPE_LONG:
          case SENSOR_TYPE_DUAL:
          case SENSOR_TYPE_TRIPLE:
          case SENSOR_TYPE_QUAD:
          case SENSOR_TYPE_TEMP_HUM:
          case SENSOR_TYPE_TEMP_BARO:
          case SENSOR_TYPE_TEMP_EMPTY_BARO:
          case SENSOR_TYPE_TEMP_HUM_BARO:
          case SENSOR_TYPE_WIND:
          default:
            root[F("nvalue")] = 0;
            root[F("svalue")] = formatDomoticzSensorType(event);
            break;
        }

        String json;
        serializeJson(root, json);
#ifndef BUILD_NO_DEBUG
        String log = F("MQTT : ");
        log += json;
        addLog(LOG_LEVEL_DEBUG, log);
#endif

        String pubname = ControllerSettings.Publish;
        parseControllerVariables(pubname, event, false);

        if (!MQTTpublish(event->ControllerIndex, pubname.c_str(), json.c_str(), Settings.MQTTRetainFlag))
        {
          connectionFailures++;
        }
        else if (connectionFailures) {
          connectionFailures--;
        }
      }
      else
      {
        String log = F("MQTT : IDX cannot be zero!");
        addLog(LOG_LEVEL_ERROR, log);
      }
      break;
    }

    case CPLUGIN_FLUSH:
    {
      processMQTTdelayQueue();
      delay(0);
      break;
    }
  }
  return success;
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C003.ino"
#ifdef USES_C003




#define CPLUGIN_003 
#define CPLUGIN_ID_003 3
#define CPLUGIN_NAME_003 "Nodo Telnet"

bool CPlugin_003(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_003;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 23;
        Protocol[protocolCount].usesID = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_003);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        C003_DelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {

        String url = F("variableset ");
        url += event->idx;
        url += ",";
        url += formatUserVarNoCheck(event, 0);
        url += "\n";
        success = C003_DelayHandler.addToQueue(C003_queue_element(event->ControllerIndex, url));
        scheduleNextDelayQueue(TIMER_C003_DELAY_QUEUE, C003_DelayHandler.getNextScheduleTime());

        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c003_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}

bool do_process_c003_delay_queue(int controller_number, const C003_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  bool success = false;
  char log[80];
  addLog(LOG_LEVEL_DEBUG, String(F("TELNT : connecting to ")) + ControllerSettings.getHostPortString());

  WiFiClient client;
  if (!ControllerSettings.connectToHost(client))
  {
    connectionFailures++;
    strcpy_P(log, PSTR("TELNT: connection failed"));
    addLog(LOG_LEVEL_ERROR, log);
    return success;
  }
  statusLED(true);
  if (connectionFailures)
    connectionFailures--;



  client.print(" \n");

  unsigned long timer = millis() + 200;
  while (!client_available(client) && !timeOutReached(timer))
    delay(1);

  timer = millis() + 1000;
  while (client_available(client) && !timeOutReached(timer) && !success)
  {


    String line;
    safeReadStringUntil(client, line, '\n');

    if (line.startsWith(F("Enter your password:")))
    {
      success = true;
      strcpy_P(log, PSTR("TELNT: Password request ok"));
      addLog(LOG_LEVEL_DEBUG, log);
    }
    delay(1);
  }

  strcpy_P(log, PSTR("TELNT: Sending pw"));
  addLog(LOG_LEVEL_DEBUG, log);
  client.println(SecuritySettings.ControllerPassword[element.controller_idx]);
  delay(100);
  while (client_available(client))
    client.read();

  strcpy_P(log, PSTR("TELNT: Sending cmd"));
  addLog(LOG_LEVEL_DEBUG, log);
  client.print(element.txt);
  delay(10);
  while (client_available(client))
    client.read();

  strcpy_P(log, PSTR("TELNT: closing connection"));
  addLog(LOG_LEVEL_DEBUG, log);

  client.stop();
  return success;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C004.ino"
#ifdef USES_C004




#define CPLUGIN_004 
#define CPLUGIN_ID_004 4
#define CPLUGIN_NAME_004 "ThingSpeak"

bool CPlugin_004(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_004;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_004);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        C004_DelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_GET_PROTOCOL_DISPLAY_NAME:
      {
        success = true;
        switch (event->idx) {
          case CONTROLLER_USER:
            string = F("ThingHTTP Name");
            break;
          case CONTROLLER_PASS:
            string = F("API Key");
            break;
          default:
            success = false;
            break;
        }
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        success = C004_DelayHandler.addToQueue(C004_queue_element(event));
        scheduleNextDelayQueue(TIMER_C004_DELAY_QUEUE, C004_DelayHandler.getNextScheduleTime());

        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c004_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}

bool do_process_c004_delay_queue(int controller_number, const C004_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;

  String postDataStr = F("api_key=");
  postDataStr += SecuritySettings.ControllerPassword[element.controller_idx];

  byte valueCount = getValueCountFromSensorType(element.sensorType);
  for (byte x = 0; x < valueCount; x++)
  {
    postDataStr += F("&field");
    postDataStr += element.idx + x;
    postDataStr += "=";
    postDataStr += formatUserVarNoCheck(element.TaskIndex, x);
  }
  String hostName = F("api.thingspeak.com");
  if (ControllerSettings.UseDNS)
    hostName = ControllerSettings.HostName;

  String postStr = do_create_http_request(
    hostName, F("POST"),
    F("/update"),
    "",
    F("Content-Type: application/x-www-form-urlencoded\r\n"),
    postDataStr.length());
  postStr += postDataStr;

  return send_via_http(controller_number, client, postStr, ControllerSettings.MustCheckReply);
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C005.ino"
#ifdef USES_C005




#define CPLUGIN_005 
#define CPLUGIN_ID_005 5
#define CPLUGIN_NAME_005 "OpenHAB MQTT"

bool CPlugin_005(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_005;
        Protocol[protocolCount].usesMQTT = true;
        Protocol[protocolCount].usesTemplate = true;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 1883;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_005);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        MQTTDelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = F("/%sysname%/#");
        event->String2 = F("/%sysname%/%tskname%/%valname%");
        break;
      }

    case CPLUGIN_PROTOCOL_RECV:
      {
        byte ControllerID = findFirstEnabledControllerWithId(CPLUGIN_ID_005);
        if (ControllerID == CONTROLLER_MAX) {

          break;
        } else {
          String cmd;
          struct EventStruct TempEvent;
          TempEvent.TaskIndex = event->TaskIndex;
          bool validTopic = false;
          const int lastindex = event->String1.lastIndexOf('/');
          const String lastPartTopic = event->String1.substring(lastindex + 1);
          if (lastPartTopic == F("cmd")) {
            cmd = event->String2;
            parseCommandString(&TempEvent, cmd);
            TempEvent.Source = VALUE_SOURCE_MQTT;
            validTopic = true;
          } else {
            if (lastindex > 0) {

              if (isFloat(event->String2) && isInt(lastPartTopic)) {
                int prevLastindex = event->String1.lastIndexOf('/', lastindex - 1);
                cmd = event->String1.substring(prevLastindex + 1, lastindex);
                TempEvent.Par1 = lastPartTopic.toInt();
                TempEvent.Par2 = event->String2.toFloat();
                TempEvent.Par3 = 0;
                validTopic = true;
              }
            }
          }
          if (validTopic) {

            String command = parseString(cmd, 1);
            if (command == F("event")) {
            eventBuffer = cmd.substring(6);
            } else if (!PluginCall(PLUGIN_WRITE, &TempEvent, cmd)) {
              remoteConfig(&TempEvent, cmd);
            }
          }
        }
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        if (!ControllerSettings.checkHostReachable(true)) {
            success = false;
            break;
        }
        statusLED(true);

        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        String pubname = ControllerSettings.Publish;
        parseControllerVariables(pubname, event, false);

        String value = "";

        byte valueCount = getValueCountFromSensorType(event->sensorType);
        for (byte x = 0; x < valueCount; x++)
        {
          String tmppubname = pubname;
          tmppubname.replace(F("%valname%"), ExtraTaskSettings.TaskDeviceValueNames[x]);
          value = formatUserVarNoCheck(event, x);

          MQTTpublish(event->ControllerIndex, tmppubname.c_str(), value.c_str(), Settings.MQTTRetainFlag);
#ifndef BUILD_NO_DEBUG
          String log = F("MQTT : ");
          log += tmppubname;
          log += ' ';
          log += value;
          addLog(LOG_LEVEL_DEBUG, log);
#endif
        }
        break;
      }

    case CPLUGIN_FLUSH:
      {
        processMQTTdelayQueue();
        delay(0);
        break;
      }

  }

  return success;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C006.ino"
#ifdef USES_C006




#define CPLUGIN_006 
#define CPLUGIN_ID_006 6
#define CPLUGIN_NAME_006 "PiDome MQTT"

bool CPlugin_006(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_006;
        Protocol[protocolCount].usesMQTT = true;
        Protocol[protocolCount].usesTemplate = true;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = false;
        Protocol[protocolCount].defaultPort = 1883;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_006);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        MQTTDelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = F("/Home/#");
        event->String2 = F("/hooks/devices/%id%/SensorData/%valname%");
        break;
      }

    case CPLUGIN_PROTOCOL_RECV:
      {


        String tmpTopic = event->String1.substring(1);
        String topicSplit[10];
        int SlashIndex = tmpTopic.indexOf('/');
        byte count = 0;
        while (SlashIndex > 0 && count < 10 - 1)
        {
          topicSplit[count] = tmpTopic.substring(0, SlashIndex);
          tmpTopic = tmpTopic.substring(SlashIndex + 1);
          SlashIndex = tmpTopic.indexOf('/');
          count++;
        }
        topicSplit[count] = tmpTopic;

        String name = topicSplit[4];
        String cmd = topicSplit[5];
        struct EventStruct TempEvent;
        TempEvent.TaskIndex = event->TaskIndex;
        TempEvent.Par1 = topicSplit[6].toInt();
        TempEvent.Par2 = 0;
        TempEvent.Par3 = 0;
        if (event->String2 == F("false") || event->String2 == F("true"))
        {
          if (event->String2 == F("true"))
            TempEvent.Par2 = 1;
        }
        else
          TempEvent.Par2 = event->String2.toFloat();
        if (name == Settings.Name)
        {
          PluginCall(PLUGIN_WRITE, &TempEvent, cmd);
        }
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        if (!WiFiConnected(10)) {
          success = false;
          break;
        }
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);

        statusLED(true);

        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        String pubname = ControllerSettings.Publish;
        parseControllerVariables(pubname, event, false);

        String value = "";

        byte valueCount = getValueCountFromSensorType(event->sensorType);
        for (byte x = 0; x < valueCount; x++)
        {
          String tmppubname = pubname;
          tmppubname.replace(F("%valname%"), ExtraTaskSettings.TaskDeviceValueNames[x]);
          value = formatUserVarNoCheck(event, x);
          MQTTpublish(event->ControllerIndex, tmppubname.c_str(), value.c_str(), Settings.MQTTRetainFlag);
        }
        break;
      }

    case CPLUGIN_FLUSH:
      {
        processMQTTdelayQueue();
        delay(0);
        break;
      }

  }
  return success;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C007.ino"
#ifdef USES_C007




#define CPLUGIN_007 
#define CPLUGIN_ID_007 7
#define CPLUGIN_NAME_007 "Emoncms"

bool CPlugin_007(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_007;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_007);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        C007_DelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        const byte valueCount = getValueCountFromSensorType(event->sensorType);
        if (valueCount == 0 || valueCount > 3) {
          addLog(LOG_LEVEL_ERROR, F("emoncms : Unknown sensortype or too many sensor values"));
          break;
        }
        success = C007_DelayHandler.addToQueue(C007_queue_element(event));
        scheduleNextDelayQueue(TIMER_C007_DELAY_QUEUE, C007_DelayHandler.getNextScheduleTime());
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c007_delay_queue();
        delay(0);
        break;
      }


  }
  return success;
}

bool do_process_c007_delay_queue(int controller_number, const C007_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;

  String url = F("/emoncms/input/post.json?node=");
  url += Settings.Unit;
  url += F("&json=");
  const byte valueCount = getValueCountFromSensorType(element.sensorType);
  for (byte i = 0; i < valueCount; ++i) {
    url += (i == 0) ? F("{") : F(",");
    url += F("field");
    url += element.idx + i;
    url += ":";
    url += formatUserVarNoCheck(element.TaskIndex, i);
  }
  url += "}";
  url += F("&apikey=");
  url += SecuritySettings.ControllerPassword[element.controller_idx];

  if (Settings.SerialLogLevel >= LOG_LEVEL_DEBUG_MORE)
    serialPrintln(url);

  return send_via_http(controller_number, client,
    create_http_get_request(controller_number, ControllerSettings, url),
    ControllerSettings.MustCheckReply);
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C008.ino"
#ifdef USES_C008




#define CPLUGIN_008 
#define CPLUGIN_ID_008 8
#define CPLUGIN_NAME_008 "Generic HTTP"
#include <ArduinoJson.h>

bool CPlugin_008(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_008;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesTemplate = true;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_008);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        C008_DelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = "";
        event->String2 = F("demo.php?name=%sysname%&task=%tskname%&valuename=%valname%&value=%value%");
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {

        byte valueCount = getValueCountFromSensorType(event->sensorType);
        C008_queue_element element(event, valueCount);
        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);

        for (byte x = 0; x < valueCount; x++)
        {
          bool isvalid;
          String formattedValue = formatUserVar(event, x, isvalid);
          if (isvalid) {
            element.txt[x] = "/";
            element.txt[x] += ControllerSettings.Publish;
            parseControllerVariables(element.txt[x], event, true);

            element.txt[x].replace(F("%valname%"), URLEncode(ExtraTaskSettings.TaskDeviceValueNames[x]));
            element.txt[x].replace(F("%value%"), formattedValue);
#ifndef BUILD_NO_DEBUG
            addLog(LOG_LEVEL_DEBUG_MORE, element.txt[x]);
#endif
          }
        }
        success = C008_DelayHandler.addToQueue(element);
        scheduleNextDelayQueue(TIMER_C008_DELAY_QUEUE, C008_DelayHandler.getNextScheduleTime());
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c008_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}




bool do_process_c008_delay_queue(int controller_number, const C008_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  while (element.txt[element.valuesSent] == "") {


    if (element.checkDone(true))
      return true;
  }

  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;

  String request = create_http_request_auth(controller_number, element.controller_idx, ControllerSettings, F("GET"), element.txt[element.valuesSent]);
  return element.checkDone(send_via_http(controller_number, client, request, ControllerSettings.MustCheckReply));
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C009.ino"
#ifdef USES_C009
# 29 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C009.ino"
#define CPLUGIN_009 
#define CPLUGIN_ID_009 9
#define CPLUGIN_NAME_009 "FHEM HTTP"
#include <ArduinoJson.h>

bool CPlugin_009(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_009;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesTemplate = false;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].usesID = false;
        Protocol[protocolCount].defaultPort = 8383;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_009);
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        byte valueCount = getValueCountFromSensorType(event->sensorType);
        C009_queue_element element(event);

        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);

        for (byte x = 0; x < valueCount; x++)
        {
          element.txt[x] = formatUserVarNoCheck(event, x);
        }
        success = C009_DelayHandler.addToQueue(element);
        scheduleNextDelayQueue(TIMER_C009_DELAY_QUEUE, C009_DelayHandler.getNextScheduleTime());
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c009_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}




bool do_process_c009_delay_queue(int controller_number, const C009_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;

  LoadTaskSettings(element.TaskIndex);
  String jsonString;
  {

    DynamicJsonDocument root(1024);
    root[F("module")] = String(F("ESPEasy"));
    root[F("version")] = String(F("1.04"));


    JsonObject data = root.createNestedObject(String(F("data")));
    JsonObject ESP = data.createNestedObject(String(F("ESP")));
    ESP[F("name")] = Settings.Name;
    ESP[F("unit")] = Settings.Unit;
    ESP[F("version")] = Settings.Version;
    ESP[F("build")] = Settings.Build;
    ESP[F("build_notes")] = String(F(BUILD_NOTES));
    ESP[F("build_git")] = String(F(BUILD_GIT));
    ESP[F("node_type_id")] = NODE_TYPE_ID;
    ESP[F("sleep")] = Settings.deepSleep;





    ESP[F("ip")] = WiFi.localIP().toString();


    JsonObject SENSOR = data.createNestedObject(String(F("SENSOR")));
    byte valueCount = getValueCountFromSensorType(element.sensorType);

    for (byte x = 0; x < valueCount; x++)
    {


      JsonObject val = SENSOR.createNestedObject(String(x));
      val[F("deviceName")] = getTaskDeviceName(element.TaskIndex);
      val[F("valueName")] = ExtraTaskSettings.TaskDeviceValueNames[x];
      val[F("type")] = element.sensorType;
      val[F("value")] = element.txt[x];
    }


    serializeJson(root, jsonString);
  }


  String request = create_http_request_auth(
      controller_number, element.controller_idx, ControllerSettings,
      F("POST"), F("/ESPEasy"), jsonString.length());
  request += jsonString;

  return send_via_http(controller_number, client, request, ControllerSettings.MustCheckReply);
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C010.ino"
#ifdef USES_C010




#define CPLUGIN_010 
#define CPLUGIN_ID_010 10
#define CPLUGIN_NAME_010 "Generic UDP"

bool CPlugin_010(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_010;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesTemplate = true;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = false;
        Protocol[protocolCount].defaultPort = 514;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_010);
        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = "";
        event->String2 = F("%sysname%_%tskname%_%valname%=%value%");
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        byte valueCount = getValueCountFromSensorType(event->sensorType);
        C010_queue_element element(event, valueCount);
        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);

        for (byte x = 0; x < valueCount; x++)
        {
          bool isvalid;
          String formattedValue = formatUserVar(event, x, isvalid);
          if (isvalid) {
            element.txt[x] = "";
            element.txt[x] += ControllerSettings.Publish;
            parseControllerVariables(element.txt[x], event, false);
            element.txt[x].replace(F("%valname%"), ExtraTaskSettings.TaskDeviceValueNames[x]);
            element.txt[x].replace(F("%value%"), formattedValue);
            addLog(LOG_LEVEL_DEBUG_MORE, element.txt[x]);
          }
        }
        success = C010_DelayHandler.addToQueue(element);
        scheduleNextDelayQueue(TIMER_C010_DELAY_QUEUE, C010_DelayHandler.getNextScheduleTime());
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c010_delay_queue();
        delay(0);
        break;
      }


  }
  return success;
}





bool do_process_c010_delay_queue(int controller_number, const C010_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  while (element.txt[element.valuesSent] == "") {


    if (element.checkDone(true))
      return true;
  }
  WiFiUDP C010_portUDP;
  if (!beginWiFiUDP_randomPort(C010_portUDP)) return false;
  if (!try_connect_host(controller_number, C010_portUDP, ControllerSettings))
    return false;

  C010_portUDP.write(
    (uint8_t*)element.txt[element.valuesSent].c_str(),
              element.txt[element.valuesSent].length());
  bool reply = C010_portUDP.endPacket();
  C010_portUDP.stop();
  if (ControllerSettings.MustCheckReply)
    return element.checkDone(reply);
  return element.checkDone(true);
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C011.ino"
#ifdef USES_C011






#define CPLUGIN_011 
#define CPLUGIN_ID_011 11
#define CPLUGIN_NAME_011 "Generic HTTP Advanced [TESTING]"

#define C011_HTTP_METHOD_MAX_LEN 16
#define C011_HTTP_URI_MAX_LEN 240
#define C011_HTTP_HEADER_MAX_LEN 256
#define C011_HTTP_BODY_MAX_LEN 512

struct C011_ConfigStruct
{
  void zero_last() {
    HttpMethod[C011_HTTP_METHOD_MAX_LEN - 1] = 0;
    HttpUri[C011_HTTP_URI_MAX_LEN - 1] = 0;
    HttpHeader[C011_HTTP_HEADER_MAX_LEN - 1] = 0;
    HttpBody[C011_HTTP_BODY_MAX_LEN - 1] = 0;
  }

  char HttpMethod[C011_HTTP_METHOD_MAX_LEN] = {0};
  char HttpUri[C011_HTTP_URI_MAX_LEN] = {0};
  char HttpHeader[C011_HTTP_HEADER_MAX_LEN] = {0};
  char HttpBody[C011_HTTP_BODY_MAX_LEN] = {0};
};

bool CPlugin_011(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_011;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_011);
        break;
      }

    case CPLUGIN_WEBFORM_LOAD:
      {
        String escapeBuffer;

        C011_ConfigStruct customConfig;

        LoadCustomControllerSettings(event->ControllerIndex,(byte*)&customConfig, sizeof(customConfig));
        customConfig.zero_last();
        String methods[] = { F("GET"), F("POST"), F("PUT"), F("HEAD"), F("PATCH") };
        string += F("<TR><TD>HTTP Method :<TD><select name='P011httpmethod'>");
        for (byte i = 0; i < 5; i++)
        {
          string += F("<option value='");
          string += methods[i] + "'";
          string += methods[i].equals(customConfig.HttpMethod) ? F(" selected='selected'") : F("");
          string += '>';
          string += methods[i];
          string += F("</option>");
        }
        string += F("</select>");

        string += F("<TR><TD>HTTP URI:<TD><input type='text' name='P011httpuri' size=80 maxlength='");
        string += C011_HTTP_URI_MAX_LEN-1;
        string += F("' value='");
        string += customConfig.HttpUri;

        string += "'>";

        string += F("<TR><TD>HTTP Header:<TD><textarea name='P011httpheader' rows='4' cols='50' maxlength='");
        string += C011_HTTP_HEADER_MAX_LEN-1;
        string += "'>";
        escapeBuffer=customConfig.HttpHeader;
        htmlEscape(escapeBuffer);
        string += escapeBuffer;
        string += F("</textarea>");

        string += F("<TR><TD>HTTP Body:<TD><textarea name='P011httpbody' rows='8' cols='50' maxlength='");
        string += C011_HTTP_BODY_MAX_LEN-1;
        string += "'>";
        escapeBuffer=customConfig.HttpBody;
        htmlEscape(escapeBuffer);
        string += escapeBuffer;
        string += F("</textarea>");
        break;
      }

    case CPLUGIN_WEBFORM_SAVE:
      {
        C011_ConfigStruct customConfig;
        String httpmethod = WebServer.arg(F("P011httpmethod"));
        String httpuri = WebServer.arg(F("P011httpuri"));
        String httpheader = WebServer.arg(F("P011httpheader"));
        String httpbody = WebServer.arg(F("P011httpbody"));

        strlcpy(customConfig.HttpMethod, httpmethod.c_str(), sizeof(customConfig.HttpMethod));
        strlcpy(customConfig.HttpUri, httpuri.c_str(), sizeof(customConfig.HttpUri));
        strlcpy(customConfig.HttpHeader, httpheader.c_str(), sizeof(customConfig.HttpHeader));
        strlcpy(customConfig.HttpBody, httpbody.c_str(), sizeof(customConfig.HttpBody));
        customConfig.zero_last();
        SaveCustomControllerSettings(event->ControllerIndex,(byte*)&customConfig, sizeof(customConfig));
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
       success = Create_schedule_HTTP_C011(event);
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c011_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}




bool do_process_c011_delay_queue(int controller_number, const C011_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;

  return send_via_http(controller_number, client, element.txt, ControllerSettings.MustCheckReply);
}




boolean Create_schedule_HTTP_C011(struct EventStruct *event)
{
  int controller_number = CPLUGIN_ID_011;
  if (!WiFiConnected(10)) {
    return false;
  }
  MakeControllerSettings(ControllerSettings);
  LoadControllerSettings(event->ControllerIndex, ControllerSettings);

  C011_ConfigStruct customConfig;
  LoadCustomControllerSettings(event->ControllerIndex,(byte*)&customConfig, sizeof(customConfig));
  customConfig.zero_last();

  WiFiClient client;
  if (!try_connect_host(controller_number, client, ControllerSettings))
    return false;

  if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
    PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

  String payload = create_http_request_auth(
    controller_number, event->ControllerIndex, ControllerSettings,
    String(customConfig.HttpMethod), customConfig.HttpUri);


  removeExtraNewLine(payload);
  if (strlen(customConfig.HttpHeader) > 0) {
    payload += customConfig.HttpHeader;
    removeExtraNewLine(payload);
  }
  ReplaceTokenByValue(payload, event);

  if (strlen(customConfig.HttpBody) > 0)
  {
    String body = String(customConfig.HttpBody);
    ReplaceTokenByValue(body, event);
    payload += F("Content-Length: ");
    payload += String(body.length());
    addNewLine(payload);
    addNewLine(payload);
    payload += body;
  }
  addNewLine(payload);

  bool success = C011_DelayHandler.addToQueue(C011_queue_element(event->ControllerIndex, payload));
  scheduleNextDelayQueue(TIMER_C011_DELAY_QUEUE, C011_DelayHandler.getNextScheduleTime());
  return success;
}



void DeleteNotNeededValues(String &s, byte numberOfValuesWanted)
{
 numberOfValuesWanted++;
 for (byte i=1; i < 5; i++)
 {
    String startToken=String(F("%")) + i + F("%");
    String endToken=String(F("%/")) + i + F("%");


    if (i<numberOfValuesWanted)
    {

      s.replace(startToken, "");
      s.replace(endToken, "");
    }
    else
    {

      int startIndex=s.indexOf(startToken);
      int endIndex=s.indexOf(endToken);
      while(startIndex != -1 && endIndex != -1 && endIndex>startIndex)
    {
        String p = s.substring(startIndex,endIndex+4);

    s.replace(p, "");


        startIndex=s.indexOf(startToken);
        endIndex=s.indexOf(endToken);
    }
    }
 }
}
# 245 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C011.ino"
void ReplaceTokenByValue(String& s, struct EventStruct *event)
{



 addLog(LOG_LEVEL_DEBUG_MORE, F("HTTP before parsing: "));
 addLog(LOG_LEVEL_DEBUG_MORE, s);
  const byte valueCount = getValueCountFromSensorType(event->sensorType);
  DeleteNotNeededValues(s,valueCount);

 addLog(LOG_LEVEL_DEBUG_MORE, F("HTTP after parsing: "));
 addLog(LOG_LEVEL_DEBUG_MORE, s);

  parseControllerVariables(s, event, true);

 addLog(LOG_LEVEL_DEBUG_MORE, F("HTTP after replacements: "));
 addLog(LOG_LEVEL_DEBUG_MORE, s);
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C012.ino"
#ifdef USES_C012






#define CPLUGIN_012 
#define CPLUGIN_ID_012 12
#define CPLUGIN_NAME_012 "Blynk HTTP [TESTING]"

bool CPlugin_012(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_012;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_012);
        break;
      }

     case CPLUGIN_PROTOCOL_SEND:
      {

        byte valueCount = getValueCountFromSensorType(event->sensorType);
        C012_queue_element element(event, valueCount);
        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);

        for (byte x = 0; x < valueCount; x++)
        {
          bool isvalid;
          String formattedValue = formatUserVar(event, x, isvalid);
          if (isvalid) {
            element.txt[x] = F("update/V");
            element.txt[x] += event->idx + x;
            element.txt[x] += F("?value=");
            element.txt[x] += formattedValue;
            addLog(LOG_LEVEL_DEBUG_MORE, element.txt[x]);
          }
        }
        success = C012_DelayHandler.addToQueue(element);
        scheduleNextDelayQueue(TIMER_C012_DELAY_QUEUE, C012_DelayHandler.getNextScheduleTime());
        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c012_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}




bool do_process_c012_delay_queue(int controller_number, const C012_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  while (element.txt[element.valuesSent] == "") {


    if (element.checkDone(true))
      return true;
  }
  if (!WiFiConnected()) {
    return false;
  }
  return element.checkDone(Blynk_get(element.txt[element.valuesSent], element.controller_idx));
}

boolean Blynk_get(const String& command, byte controllerIndex, float *data )
{
  MakeControllerSettings(ControllerSettings);
  LoadControllerSettings(controllerIndex, ControllerSettings);

  if ((SecuritySettings.ControllerPassword[controllerIndex][0] == 0)) {
    addLog(LOG_LEVEL_ERROR, F("Blynk : No password set"));
    return false;
  }

  WiFiClient client;
  if (!try_connect_host(CPLUGIN_ID_012, client, ControllerSettings))
    return false;



  char request[300] = {0};
  sprintf_P(request,
            PSTR("GET /%s/%s HTTP/1.1\r\n Host: %s \r\n Connection: close\r\n\r\n"),
            SecuritySettings.ControllerPassword[controllerIndex],
            command.c_str(),
            ControllerSettings.getHost().c_str());
  addLog(LOG_LEVEL_DEBUG, request);
  client.print(request);
  bool success = !ControllerSettings.MustCheckReply;
  if (ControllerSettings.MustCheckReply || data) {
    unsigned long timer = millis() + 200;
    while (!client_available(client) && !timeOutReached(timer))
      delay(1);

    char log[80] = {0};
    timer = millis() + 1500;

    while (client_available(client) && !success && !timeOutReached(timer)) {
      String line;
      safeReadStringUntil(client, line, '\n');
      addLog(LOG_LEVEL_DEBUG_MORE, line);

      if (line.substring(0, 15) == F("HTTP/1.1 200 OK")) {
        strcpy_P(log, PSTR("HTTP : Success"));
        if (!data) success = true;
      }
      else if (line.substring(0, 24) == F("HTTP/1.1 400 Bad Request")) {
        strcpy_P(log, PSTR("HTTP : Unauthorized"));
      }
      else if (line.substring(0, 25) == F("HTTP/1.1 401 Unauthorized")) {
        strcpy_P(log, PSTR("HTTP : Unauthorized"));
      }
      addLog(LOG_LEVEL_DEBUG, log);


      if (data && line.startsWith("["))
      {
        String strValue = line;
        byte pos = strValue.indexOf('"',2);
        strValue = strValue.substring(2, pos);
        strValue.trim();
        float value = strValue.toFloat();
        *data = value;
        success = true;

        char value_char[5] = {0};
        strValue.toCharArray(value_char, 5);
        sprintf_P(log, PSTR("Blynk get - %s => %s"),command.c_str(), value_char );
        addLog(LOG_LEVEL_DEBUG, log);
      }
      delay(0);
    }
  }
  addLog(LOG_LEVEL_DEBUG, F("HTTP : closing connection (012)"));

  client.flush();
  client.stop();


  unsigned long timer = millis() + Settings.MessageDelay;
  while (!timeOutReached(timer))
              backgroundtasks();

  return success;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C013.ino"
#ifdef USES_C013




#define CPLUGIN_013 
#define CPLUGIN_ID_013 13
#define CPLUGIN_NAME_013 "ESPEasy P2P Networking"

WiFiUDP C013_portUDP;

struct C013_SensorInfoStruct
{
  byte header = 255;
  byte ID = 3;
  byte sourcelUnit;
  byte destUnit;
  byte sourceTaskIndex;
  byte destTaskIndex;
  byte deviceNumber;
  char taskName[26];
  char ValueNames[VARS_PER_TASK][26];
};

struct C013_SensorDataStruct
{
  byte header = 255;
  byte ID = 5;
  byte sourcelUnit;
  byte destUnit;
  byte sourceTaskIndex;
  byte destTaskIndex;
  float Values[VARS_PER_TASK];
};


bool CPlugin_013(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_013;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesTemplate = false;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = false;
        Protocol[protocolCount].defaultPort = 65501;
        Protocol[protocolCount].usesID = false;
        Protocol[protocolCount].Custom = true;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_013);
        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = "";
        event->String2 = "";
        break;
      }

    case CPLUGIN_INIT:
      {

        break;
      }

    case CPLUGIN_TASK_CHANGE_NOTIFICATION:
      {
        C013_SendUDPTaskInfo(0, event->TaskIndex, event->TaskIndex);
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        C013_SendUDPTaskData(0, event->TaskIndex, event->TaskIndex);
        break;
      }

    case CPLUGIN_UDP_IN:
      {
        C013_Receive(event);
        break;
      }
# 101 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C013.ino"
  }
  return success;
}





void C013_SendUDPTaskInfo(byte destUnit, byte sourceTaskIndex, byte destTaskIndex)
{
  if (!WiFiConnected(10)) {
    return;
  }
  struct C013_SensorInfoStruct infoReply;
  infoReply.sourcelUnit = Settings.Unit;
  infoReply.sourceTaskIndex = sourceTaskIndex;
  infoReply.destTaskIndex = destTaskIndex;
  LoadTaskSettings(infoReply.sourceTaskIndex);
  infoReply.deviceNumber = Settings.TaskDeviceNumber[infoReply.sourceTaskIndex];
  strcpy(infoReply.taskName, getTaskDeviceName(infoReply.sourceTaskIndex).c_str());
  for (byte x = 0; x < VARS_PER_TASK; x++)
    strcpy(infoReply.ValueNames[x], ExtraTaskSettings.TaskDeviceValueNames[x]);

  if (destUnit != 0)
  {
    infoReply.destUnit = destUnit;
    C013_sendUDP(destUnit, (byte*)&infoReply, sizeof(C013_SensorInfoStruct));
    delay(10);
  } else {
    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it) {
      if (it->first != Settings.Unit) {
        infoReply.destUnit = it->first;
        C013_sendUDP(it->first, (byte*)&infoReply, sizeof(C013_SensorInfoStruct));
        delay(10);
      }
    }
  }
  delay(50);
}

void C013_SendUDPTaskData(byte destUnit, byte sourceTaskIndex, byte destTaskIndex)
{
  if (!WiFiConnected(10)) {
    return;
  }
  struct C013_SensorDataStruct dataReply;
  dataReply.sourcelUnit = Settings.Unit;
  dataReply.sourceTaskIndex = sourceTaskIndex;
  dataReply.destTaskIndex = destTaskIndex;
  for (byte x = 0; x < VARS_PER_TASK; x++)
    dataReply.Values[x] = UserVar[dataReply.sourceTaskIndex * VARS_PER_TASK + x];

  if (destUnit != 0)
  {
    dataReply.destUnit = destUnit;
    C013_sendUDP(destUnit, (byte*) &dataReply, sizeof(C013_SensorDataStruct));
    delay(10);
  } else {
    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it) {
      if (it->first != Settings.Unit) {
        dataReply.destUnit = it->first;
        C013_sendUDP(it->first, (byte*) &dataReply, sizeof(C013_SensorDataStruct));
        delay(10);
      }
    }
  }
  delay(50);
}




void C013_sendUDP(byte unit, byte* data, byte size)
{
  if (!WiFiConnected(10)) {
    return;
  }
  NodesMap::iterator it;
  if (unit != 255) {
    it = Nodes.find(unit);
    if (it == Nodes.end())
      return;
    if (it->second.ip[0] == 0)
      return;
  }
#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
    String log = F("C013 : Send UDP message to ");
    log += unit;
    addLog(LOG_LEVEL_DEBUG_MORE, log);
  }
#endif

  statusLED(true);

  IPAddress remoteNodeIP;
  if (unit == 255)
    remoteNodeIP = {255, 255, 255, 255};
  else
    remoteNodeIP = it->second.ip;
  if (!beginWiFiUDP_randomPort(C013_portUDP)) return;
  if (C013_portUDP.beginPacket(remoteNodeIP, Settings.UDPPort) == 0) return;
  C013_portUDP.write(data, size);
  C013_portUDP.endPacket();
  C013_portUDP.stop();
}

void C013_Receive(struct EventStruct *event) {
  if (event->Par2 < 6) return;
#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
    if (event->Data[1] > 1 && event->Data[1] < 6)
    {
      String log = (F("C013 : msg "));
      for (byte x = 1; x < 6; x++)
      {
        log += ' ';
        log += (int)event->Data[x];
      }
      addLog(LOG_LEVEL_DEBUG_MORE, log);
    }
  }
#endif

  switch (event->Data[1]) {
    case 2:
      {

        break;
      }

    case 3:
      {
        struct C013_SensorInfoStruct infoReply;
        if (static_cast<size_t>(event->Par2) < sizeof(C013_SensorInfoStruct)) {
#ifndef BUILD_NO_DEBUG
          addLog(LOG_LEVEL_DEBUG, F("C013_Receive: Received data smaller than C013_SensorInfoStruct, discarded"));
#endif
        } else {
          memcpy((byte*)&infoReply, (byte*)event->Data, sizeof(C013_SensorInfoStruct));



          if (Settings.TaskDeviceNumber[infoReply.destTaskIndex] == 0)
          {
            taskClear(infoReply.destTaskIndex, false);
            Settings.TaskDeviceNumber[infoReply.destTaskIndex] = infoReply.deviceNumber;
            Settings.TaskDeviceDataFeed[infoReply.destTaskIndex] = 1;
            for (byte x = 0; x < CONTROLLER_MAX; x++)
              Settings.TaskDeviceSendData[x][infoReply.destTaskIndex] = false;
            strcpy(ExtraTaskSettings.TaskDeviceName, infoReply.taskName);
            for (byte x = 0; x < VARS_PER_TASK; x++)
              strcpy( ExtraTaskSettings.TaskDeviceValueNames[x], infoReply.ValueNames[x]);
            ExtraTaskSettings.TaskIndex = infoReply.destTaskIndex;
            SaveTaskSettings(infoReply.destTaskIndex);
            SaveSettings();
          }
        }
        break;
      }

    case 4:
      {

        break;
      }

    case 5:
      {
        struct C013_SensorDataStruct dataReply;
        if (static_cast<size_t>(event->Par2) < sizeof(C013_SensorDataStruct)) {
#ifndef BUILD_NO_DEBUG
          addLog(LOG_LEVEL_DEBUG, F("C013_Receive: Received data smaller than C013_SensorDataStruct, discarded"));
#endif
        } else {
          memcpy((byte*)&dataReply, (byte*)event->Data, sizeof(C013_SensorDataStruct));


          if (Settings.TaskDeviceDataFeed[dataReply.destTaskIndex] != 0)
          {
            for (byte x = 0; x < VARS_PER_TASK; x++)
            {
              UserVar[dataReply.destTaskIndex * VARS_PER_TASK + x] = dataReply.Values[x];
            }
            if (Settings.UseRules)
              createRuleEvents(dataReply.destTaskIndex);
          }
        }
        break;
      }
  }
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C014.ino"
#ifdef USES_C014




#define CPLUGIN_014 
#define CPLUGIN_ID_014 14



#define CPLUGIN_014_V4 

#ifdef CPLUGIN_014_V3
  #define CPLUGIN_014_HOMIE_VERSION "3.0.0"
  #define CPLUGIN_NAME_014 "Homie MQTT (Version 3.0.1)"
#endif
#ifdef CPLUGIN_014_V4
  #define CPLUGIN_014_HOMIE_VERSION "4.0.0"
  #define CPLUGIN_NAME_014 "Homie MQTT (Version 4.0.0 dev)"
#endif


#define CPLUGIN_014_SUBSCRIBE "homie/%sysname%/+/+/set"
#define CPLUGIN_014_PUBLISH "homie/%sysname%/%tskname%/%valname%"

#define CPLUGIN_014_BASE_TOPIC "homie/%sysname%/#"
#define CPLUGIN_014_BASE_VALUE "homie/%sysname%/%device%/%node%/%property%"
#define CPLUGIN_014_INTERVAL "90"
#define CPLUGIN_014_SYSTEM_DEVICE "SYSTEM"
#define CPLUGIN_014_CMD_VALUE "cmd"
#define CPLUGIN_014_GPIO_VALUE "gpio"
#define CPLUGIN_014_CMD_VALUE_NAME "Command"

byte msgCounter=0;



bool CPlugin_014_sendMQTTmsg(String& topic, const char* payload, int& errorCounter) {
        bool mqttReturn = MQTTpublish(CPLUGIN_ID_014, topic.c_str(), payload, true);
        if (mqttReturn) msgCounter++;
          else errorCounter++;
        if (loglevelActiveFor(LOG_LEVEL_INFO) && mqttReturn) {
          String log = F("C014 : msg T:");
          log += topic;
          log += F(" P: ");
          log += payload;
          addLog(LOG_LEVEL_DEBUG_MORE, log+" success!");
        }
        if (loglevelActiveFor(LOG_LEVEL_INFO) && !mqttReturn) {
          String log = F("C014 : msg T:");
          log += topic;
          log += F(" P: ");
          log += payload;
          addLog(LOG_LEVEL_ERROR, log+" ERROR!");
        }
        return mqttReturn;
}


bool CPlugin_014_sendMQTTdevice(String tmppubname, const char* topic, const char* payload, int& errorCounter) {
        tmppubname.replace(F("#"), topic);
        bool mqttReturn = MQTTpublish(CPLUGIN_ID_014, tmppubname.c_str(), payload, true);
        if (mqttReturn) msgCounter++;
          else errorCounter++;
        if (loglevelActiveFor(LOG_LEVEL_INFO) && mqttReturn) {
          String log = F("C014 : T:");
          log += topic;
          log += F(" P: ");
          log += payload;
          addLog(LOG_LEVEL_DEBUG_MORE, log+" success!");
        }
        if (loglevelActiveFor(LOG_LEVEL_INFO) && !mqttReturn) {
          String log = F("C014 : T:");
          log += topic;
          log += F(" P: ");
          log += payload;
          addLog(LOG_LEVEL_ERROR, log+" ERROR!");
        }
        processMQTTdelayQueue();
        return mqttReturn;

}


bool CPlugin_014_sendMQTTnode(String tmppubname, const char* node, const char* value, const char* topic, const char* payload, int& errorCounter) {
        tmppubname.replace(F("%device%"), node);
        tmppubname.replace(F("%node%"), value);
        tmppubname.replace(F("/%property%"), topic);
        bool mqttReturn = MQTTpublish(CPLUGIN_ID_014, tmppubname.c_str(), payload, true);
        if (mqttReturn) msgCounter++;
          else errorCounter++;
        if (loglevelActiveFor(LOG_LEVEL_INFO) && mqttReturn) {
          String log = F("C014 : V:");
          log += value;
          log += F(" T: ");
          log += topic;
          log += F(" P: ");
          log += payload;
          addLog(LOG_LEVEL_DEBUG_MORE, log+" success!");
        }
        if (loglevelActiveFor(LOG_LEVEL_INFO) && !mqttReturn) {
          String log = F("C014 : V:");
          log += value;
          log += F(" T: ");
          log += topic;
          log += F(" P: ");
          log += payload;
          addLog(LOG_LEVEL_ERROR, log+" ERROR!");
        }
        processMQTTdelayQueue();
        return mqttReturn;

}


void CPLUGIN_014_addToList(String& valuesList, const char* node)
{
  if (valuesList.length()>0) valuesList += F(",");
  valuesList += node;
}


int CPlugin_014_getPluginNr(String& setNodeName)
{
  byte x = 0;

  while (x < TASKS_MAX)
  {
    LoadTaskSettings(x);
    if (strcmp(ExtraTaskSettings.TaskDeviceName,setNodeName.c_str())==0) return x;
    x++;
  }
  return x;
}


int CPlugin_014_getValueNr(int DeviceIndex, String& valueName)
{
  LoadTaskSettings(DeviceIndex);

  for (byte varNr = 0; varNr < Device[getDeviceIndex(Settings.TaskDeviceNumber[DeviceIndex])].ValueCount; varNr++)
  {
    if (strcmp(ExtraTaskSettings.TaskDeviceValueNames[varNr],valueName.c_str())==0) return varNr;
  }
  return -1;
}

bool CPlugin_014(byte function, struct EventStruct *event, String& string)
{
  bool success = false;
  int errorCounter = 0;
  String log = "";
  String pubname = "";
  String tmppubname = "";

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_014;
        Protocol[protocolCount].usesMQTT = true;
        Protocol[protocolCount].usesTemplate = true;
        Protocol[protocolCount].usesAccount = true;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 1883;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_014);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        MQTTDelayHandler.configureControllerSettings(ControllerSettings);
        break;
      }

    case CPLUGIN_INTERVAL:
      {
        if (MQTTclient.connected())
        {
          errorCounter = 0;

          pubname = CPLUGIN_014_BASE_TOPIC;
          pubname.replace(F("%sysname%"), Settings.Name);

#ifdef CPLUGIN_014_V3

          CPlugin_014_sendMQTTdevice(pubname,"$stats/uptime",toString((wdcounter / 2)*60,0).c_str(),errorCounter);


          float RssI = WiFi.RSSI();
          RssI = isnan(RssI) ? -100.0 : RssI;
          RssI = min(max(2 * (RssI + 100.0), 0.0), 100.0);

          CPlugin_014_sendMQTTdevice(pubname,"$stats/signal",toString(RssI,1).c_str(),errorCounter);
#endif

          if (errorCounter>0)
          {

            CPlugin_014_sendMQTTdevice(pubname,"$state","alert",errorCounter);
            success = false;
          } else {

            CPlugin_014_sendMQTTdevice(pubname,"$state","ready",errorCounter);
            success = true;
          }
          if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
            String log = F("C014 : $stats information sent with ");
            if (errorCounter>0) log+=errorCounter;
              else log+=F("no");
            log+=F(" errors! (");
            log+=msgCounter;
            log+=F(" messages)");
            msgCounter=0;
            addLog(LOG_LEVEL_DEBUG, log);
          }
        }
        break;
      }

    case CPLUGIN_GOT_CONNECTED:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        if (!ControllerSettings.checkHostReachable(true)) {
            success = false;
            break;
        }
        statusLED(true);


        pubname = CPLUGIN_014_BASE_TOPIC;
        pubname.replace(F("%sysname%"), Settings.Name);
        int deviceCount = 1;
        int nodeCount = 1;
        errorCounter = 0;

        if (lastBootCause!=BOOT_CAUSE_DEEP_SLEEP)
        {
          String nodename = CPLUGIN_014_BASE_VALUE;
          nodename.replace(F("%sysname%"), Settings.Name);
          String nodesList = "";
          String valuesList = "";
          byte DeviceIndex = 0;
          String deviceName = "";
          String valueName = "";
          String unitName = "";


          CPlugin_014_sendMQTTdevice(pubname,"$state","init",errorCounter);


          CPlugin_014_sendMQTTdevice(pubname,"$homie",CPLUGIN_014_HOMIE_VERSION,errorCounter);


          CPlugin_014_sendMQTTdevice(pubname,"$name",Settings.Name,errorCounter);


#ifdef CPLUGIN_014_V3
          CPlugin_014_sendMQTTdevice(pubname,"$localip",formatIP(WiFi.localIP()).c_str(),errorCounter);


          CPlugin_014_sendMQTTdevice(pubname,"$mac",WiFi.macAddress().c_str(),errorCounter);


          #if defined(ESP8266)
            CPlugin_014_sendMQTTdevice(pubname,"$implementation","ESP8266",errorCounter);
          #endif
          #if defined(ESP32)
            CPlugin_014_sendMQTTdevice(pubname,"$implementation","ESP32",errorCounter);
          #endif


          CPlugin_014_sendMQTTdevice(pubname,"$fw/version",toString(Settings.Build,0).c_str(),errorCounter);


          CPlugin_014_sendMQTTdevice(pubname,"$fw/name",getNodeTypeDisplayString(NODE_TYPE_ID).c_str(),errorCounter);


          CPlugin_014_sendMQTTdevice(pubname,"$stats/interval",CPLUGIN_014_INTERVAL,errorCounter);
#endif


          CPLUGIN_014_addToList(nodesList,CPLUGIN_014_SYSTEM_DEVICE);
          CPLUGIN_014_addToList(valuesList,CPLUGIN_014_CMD_VALUE);


          CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, "$name", "", CPLUGIN_014_SYSTEM_DEVICE, errorCounter);


          CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, CPLUGIN_014_CMD_VALUE, "/$name", CPLUGIN_014_CMD_VALUE_NAME, errorCounter);

          CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, CPLUGIN_014_CMD_VALUE, "/$datatype", "string", errorCounter);


          CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, CPLUGIN_014_CMD_VALUE, "/$settable", "true", errorCounter);




          int gpio = 0;

          while (gpio < MAX_GPIO && gpio < 17) {
            if (Settings.PinBootStates[gpio]>0)
            {
              nodeCount++;
              valueName = CPLUGIN_014_GPIO_VALUE;
              valueName += toString(gpio,0);
              CPLUGIN_014_addToList(valuesList,valueName.c_str());


              CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, valueName.c_str(), "/$name", valueName.c_str(), errorCounter);

              CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, valueName.c_str(), "/$datatype", "boolean", errorCounter);
              if (Settings.PinBootStates[gpio]<3)
              {

                CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, valueName.c_str(), "/$settable", "true", errorCounter);
              }
            }
            ++gpio;
          }


          CPlugin_014_sendMQTTnode(nodename, CPLUGIN_014_SYSTEM_DEVICE, "$properties", "", valuesList.c_str(), errorCounter);
          valuesList = "";
          deviceCount++;


          for (byte x = 0; x < TASKS_MAX; x++)
          {
            if (Settings.TaskDeviceNumber[x] != 0)
            {
              LoadTaskSettings(x);
              DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[x]);
              deviceName = ExtraTaskSettings.TaskDeviceName;

              if (Settings.TaskDeviceEnabled[x])
              {
                valuesList="";

                if (!Device[DeviceIndex].SendDataOption)
                {
                  if (Device[DeviceIndex].Number==86)
                  {
                    for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++) {
                      if (Settings.TaskDeviceNumber[x] != 0) {
                        if (ExtraTaskSettings.TaskDeviceValueNames[varNr][0]!=0) {
                          CPLUGIN_014_addToList(valuesList,ExtraTaskSettings.TaskDeviceValueNames[varNr]);

                          CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$settable", "true", errorCounter);

                          valueName = F("Homie Receiver: ");
                          valueName += ExtraTaskSettings.TaskDeviceValueNames[varNr];
                          CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$name",valueName.c_str(), errorCounter);

                          unitName = "";
                          switch(Settings.TaskDevicePluginConfig[x][varNr]) {
                            case 0: valueName = F("integer");
                                    if (ExtraTaskSettings.TaskDevicePluginConfig[varNr]!=0 || ExtraTaskSettings.TaskDevicePluginConfig[varNr+5]!=0) {
                                      unitName = ExtraTaskSettings.TaskDevicePluginConfig[varNr];
                                      unitName += ":";
                                      unitName += ExtraTaskSettings.TaskDevicePluginConfig[varNr+Device[DeviceIndex].ValueCount];
                                    }
                                    break;
                            case 1: valueName = F("float");
                                    if (ExtraTaskSettings.TaskDevicePluginConfig[varNr]!=0 || ExtraTaskSettings.TaskDevicePluginConfig[varNr+5]!=0) {
                                      unitName = ExtraTaskSettings.TaskDevicePluginConfig[varNr];
                                      unitName += ":";
                                      unitName += ExtraTaskSettings.TaskDevicePluginConfig[varNr+Device[DeviceIndex].ValueCount];
                                    }
                                    break;
                            case 2: valueName = F("boolean"); break;
                            case 3: valueName = F("string"); break;
                            case 4: valueName = F("enum");
                                    unitName = ExtraTaskSettings.TaskDeviceFormula[varNr];
                                    break;
                            case 5: valueName = F("color");
                                    unitName = F("rgb");
                                    break;
                            case 6: valueName = F("color");
                                    unitName = F("hsv");
                                    break;
                          }
                          CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$datatype", valueName.c_str(), errorCounter);
                          if (unitName!="") CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$format", unitName.c_str(), errorCounter);
                          nodeCount++;
                        }
                      }
                    }
                  }
                } else {


                  byte customValues = false;
                  if (!customValues)
                  {
                    for (byte varNr = 0; varNr < Device[DeviceIndex].ValueCount; varNr++)
                    {
                      if (Settings.TaskDeviceNumber[x] != 0)
                      {
                        if (ExtraTaskSettings.TaskDeviceValueNames[varNr][0]!=0)
                        {
                          CPLUGIN_014_addToList(valuesList,ExtraTaskSettings.TaskDeviceValueNames[varNr]);


                          CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$name", ExtraTaskSettings.TaskDeviceValueNames[varNr], errorCounter);

                          CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$datatype", "float", errorCounter);

                          if (Device[DeviceIndex].Number==33)
                            CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), ExtraTaskSettings.TaskDeviceValueNames[varNr], "/$settable", "true", errorCounter);

                          nodeCount++;
# 441 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C014.ino"
                        }
                      }
                    }
                  } else {
                    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
                      String log = F("C014 : Device has custom values: ");
                      log += getPluginNameFromDeviceIndex(Settings.TaskDeviceNumber[x]);
                      addLog(LOG_LEVEL_DEBUG, log+" not implemented!")
                    }
                  }
                }
                if (valuesList!="")
                {


                  CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), "$name", "",ExtraTaskSettings.TaskDeviceName, errorCounter);


                  CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), "$type", "", getPluginNameFromDeviceIndex(DeviceIndex).c_str(), errorCounter);


                  CPLUGIN_014_addToList(nodesList,deviceName.c_str());
                  deviceCount++;

                  CPlugin_014_sendMQTTnode(nodename, deviceName.c_str(), "$properties", "", valuesList.c_str(), errorCounter);
                  valuesList="";
                }
              } else {
                if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
                  String log = F("C014 : Device Disabled: ");
                  log += getPluginNameFromDeviceIndex(Settings.TaskDeviceNumber[x]);
                  addLog(LOG_LEVEL_DEBUG, log+" not propagated!")
                }
              }
            }
          }



          CPlugin_014_sendMQTTdevice(pubname, "$nodes", nodesList.c_str(), errorCounter);
        }

        if (errorCounter>0)
        {

          CPlugin_014_sendMQTTdevice(pubname,"$state","alert",errorCounter);
          success = false;
        } else {

          CPlugin_014_sendMQTTdevice(pubname,"$state","ready",errorCounter);
          success = true;
        }
        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log = F("C014 : autodiscover information of ");
          log += deviceCount;
          log += F(" Devices and ");
          log += nodeCount;
          log += F(" Nodes sent with ");
          if (errorCounter>0) log+=errorCounter;
            else log+=F("no");
          log+=F(" errors! (");
          log+=msgCounter;
          log+=F(" messages)");
          msgCounter = 0;
          errorCounter = 0;
          addLog(LOG_LEVEL_INFO, log);
        }
        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = F(CPLUGIN_014_SUBSCRIBE);
        event->String2 = F(CPLUGIN_014_PUBLISH);
        break;
      }

    case CPLUGIN_GOT_INVALID:
      {
        pubname = CPLUGIN_014_BASE_TOPIC;
        pubname.replace(F("%sysname%"), Settings.Name);

        success = CPlugin_014_sendMQTTdevice(pubname,"$state","disconnected",errorCounter);
        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log = F("C014 : Device: ");
          log += Settings.Name;
          if (success) log += F(" got invalid (disconnected).");
            else log += F(" got invaild (disconnect) failed!");
          addLog(LOG_LEVEL_INFO, log);
        }
        break;
      }

    case CPLUGIN_FLUSH:
      {
        pubname = CPLUGIN_014_BASE_TOPIC;
        pubname.replace(F("%sysname%"), Settings.Name);

        success = CPlugin_014_sendMQTTdevice(pubname,"$state","sleeping",errorCounter);
        break;
      }

    case CPLUGIN_PROTOCOL_RECV:
      {
        byte ControllerID = findFirstEnabledControllerWithId(CPLUGIN_ID_014);
        bool validTopic = false;
        if (ControllerID == CONTROLLER_MAX) {

          break;
        } else {
          String cmd;
          int valueNr=0;
          int deviceNr=0;
          struct EventStruct TempEvent;
          TempEvent.TaskIndex = event->TaskIndex;
          TempEvent.Source = VALUE_SOURCE_MQTT;
          int lastindex = event->String1.lastIndexOf('/');
          errorCounter = 0;
          if (event->String1.substring(lastindex + 1) == F("set"))
          {
            pubname = event->String1.substring(0,lastindex);
            lastindex = pubname.lastIndexOf('/');
            String nodeName = pubname.substring(0,lastindex);
            String valueName = pubname. substring(lastindex+1);
            lastindex = nodeName.lastIndexOf('/');
            nodeName = nodeName. substring(lastindex+1);
            if (loglevelActiveFor(LOG_LEVEL_INFO)) {
              log=F("C014 : MQTT received: ");
              log+=F("/set: N: ");
              log+=nodeName;
              log+=F(" V: ");
              log+=valueName;
            }
            if (nodeName == F(CPLUGIN_014_SYSTEM_DEVICE))
            {
              if (valueName.substring(0,strlen(CPLUGIN_014_GPIO_VALUE)) == F(CPLUGIN_014_GPIO_VALUE))
              {
                cmd = ("GPIO,");
                cmd += valueName.substring(strlen(CPLUGIN_014_GPIO_VALUE)).toInt();
                if (event->String2=="true" || event->String2=="1") cmd += F(",1");
                  else cmd += F(",0");
                validTopic = true;
              } else if (valueName==CPLUGIN_014_CMD_VALUE)
              {
                cmd = event->String2;
                validTopic = true;
              } else
              {
                cmd = F("SYSTEM/");
                cmd += valueName;
                cmd += F(" unknown!");
              }
            } else
            {
              deviceNr=CPlugin_014_getPluginNr(nodeName);
              int pluginID=Device[getDeviceIndex(Settings.TaskDeviceNumber[deviceNr])].Number;

              if (pluginID==33)
              {
                valueNr = CPlugin_014_getValueNr(deviceNr,valueName);
                if (valueNr > -1)
                {
                  cmd = F("DummyValueSet,");
                  cmd += (deviceNr+1);
                  cmd += F(",");
                  cmd += (valueNr+1);
                  cmd += F(",");
                  cmd += event->String2;
                  validTopic = true;
                }
              } else if (pluginID==86) {
                valueNr = CPlugin_014_getValueNr(deviceNr,valueName);
                cmd = F("event,");
                cmd += valueName;
                cmd += "=";
                if (Settings.TaskDevicePluginConfig[deviceNr][valueNr]==3) {
                  cmd += '"';
                  cmd += event->String2;
                  cmd += '"';
                } else {
                  if (Settings.TaskDevicePluginConfig[deviceNr][valueNr]==4) {
                    String enumList = ExtraTaskSettings.TaskDeviceFormula[event->Par2-1];
                    int i = 1;
                    while (parseString(enumList,i)!="") {
                      if (parseString(enumList,i)==event->String2) break;
                      i++;
                    }
                    cmd += i;
                    cmd += ",";
                  }
                  cmd += event->String2;
                }
                validTopic = true;
              }
            }

            if (validTopic) {
              parseCommandString(&TempEvent, cmd);
              if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                log+=F(" cmd: ");
                log+=cmd;
                addLog(LOG_LEVEL_INFO, log+ F(" OK"));
              }
            } else {
              if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                addLog(LOG_LEVEL_INFO, log+ F(" INVALID MSG"));
              }
            }
          }

          if (validTopic) {

            String command = parseString(cmd, 1);
            if (command == F("event"))
            {
              eventBuffer = cmd.substring(6);
              if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                log=F("C014 : deviceNr:");
                log+=deviceNr;
                log+=F(" valueNr:");
                log+=valueNr;
                log+=F(" valueType:");
                log+=Settings.TaskDevicePluginConfig[deviceNr][valueNr];
                log+=F(" Event: ");
                log+=eventBuffer;
                addLog(LOG_LEVEL_INFO, log);
              }
            } else {
              if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                log=F("C014 : PluginCall:");
              }
              if (!PluginCall(PLUGIN_WRITE, &TempEvent, cmd)) {
                remoteConfig(&TempEvent, cmd);
                if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                  log +=F(" failed! remoteConfig?");
                }
              } else {
                if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                  log +=F(" OK!");
                }
              }
              if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                addLog(LOG_LEVEL_INFO, log);
              }
            }

          }
        }
        success = validTopic;
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        if (!ControllerSettings.checkHostReachable(true)) {
            success = false;
            break;
        }
        statusLED(true);

        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        String pubname = ControllerSettings.Publish;
        parseControllerVariables(pubname, event, false);

        String value = "";

        byte valueCount = getValueCountFromSensorType(event->sensorType);
        for (byte x = 0; x < valueCount; x++)
        {
          String tmppubname = pubname;
          tmppubname.replace(F("%valname%"), ExtraTaskSettings.TaskDeviceValueNames[x]);
          value = formatUserVarNoCheck(event, x);

          MQTTpublish(event->ControllerIndex, tmppubname.c_str(), value.c_str(), Settings.MQTTRetainFlag);
          if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
            String log = F("C014 : Sent to ");
            log += tmppubname;
            log += ' ';
            log += value;
            addLog(LOG_LEVEL_DEBUG, log);
          }
        }
        break;
      }

      case CPLUGIN_ACKNOWLEDGE:
      {
        LoadTaskSettings(event->Par1-1);
# 768 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C014.ino"
        success = false;
        if (string!="") {
          String commandName = parseString(string, 1);
          if (commandName == F("gpio"))
          {
            int port = event -> Par1;
            int valueInt = event -> Par2;
            String valueBool = "false";
            if (valueInt==1) valueBool = "true";

            String topic = CPLUGIN_014_PUBLISH;
            topic.replace(F("%sysname%"), Settings.Name);
            topic.replace(F("%tskname%"), CPLUGIN_014_SYSTEM_DEVICE);
            topic.replace(F("%valname%"), CPLUGIN_014_GPIO_VALUE + toString(port,0));

            success = MQTTpublish(CPLUGIN_ID_014, topic.c_str(), valueBool.c_str(), false);
            if (loglevelActiveFor(LOG_LEVEL_INFO) && success) {
              String log = F("C014 : Acknowledged GPIO");
              log += port;
              log += F(" value:");
              log += valueBool;
              log += F(" (");
              log += valueInt;
              log += F(")");
              addLog(LOG_LEVEL_INFO, log+" success!");
            }
            if (loglevelActiveFor(LOG_LEVEL_ERROR) && !success) {
              String log = F("C014 : Acknowledged GPIO");
              log += port;
              log += F(" value:");
              log += valueBool;
              log += F(" (");
              log += valueInt;
              log += F(")");
              addLog(LOG_LEVEL_ERROR, log+" ERROR!");
            }
          } else
          {
            String topic = CPLUGIN_014_PUBLISH;
            topic.replace(F("%sysname%"), Settings.Name);
            int deviceIndex = event->Par1;
            LoadTaskSettings(deviceIndex-1);
            String deviceName = ExtraTaskSettings.TaskDeviceName;
            topic.replace(F("%tskname%"), deviceName);
            String valueName = ExtraTaskSettings.TaskDeviceValueNames[event->Par2-1];
            topic.replace(F("%valname%"), valueName);
            String valueStr = "";
            int valueInt = 0;

            if ((commandName == F("taskvalueset")) || (commandName == F("dummyvalueset")))
            {
              valueStr = toString(UserVar[event->BaseVarIndex+event->Par2-1],ExtraTaskSettings.TaskDeviceValueDecimals[event->Par2-1]);
              success = MQTTpublish(CPLUGIN_ID_014, topic.c_str(), valueStr.c_str(), false);
              if (loglevelActiveFor(LOG_LEVEL_INFO) && success) {
                String log = F("C014 : Acknowledged: ");
                log += deviceName;
                log += F(" var: ");
                log += valueName;
                log += F(" topic: ");
                log += topic;
                log += F(" value: ");
                log += valueStr;
                addLog(LOG_LEVEL_INFO, log+" success!");
              }
              if (loglevelActiveFor(LOG_LEVEL_ERROR) && !success) {
                String log = F("C014 : Aacknowledged: ");
                log += deviceName;
                log += F(" var: ");
                log += valueName;
                log += F(" topic: ");
                log += topic;
                log += F(" value: ");
                log += valueStr;
                addLog(LOG_LEVEL_ERROR, log+" ERROR!");
              }
            } else if (parseString(commandName, 1) == F("homievalueset")) {
              switch (Settings.TaskDevicePluginConfig[deviceIndex-1][event->Par2-1]) {
                case 0:
                  valueInt = static_cast<int>(UserVar[event->BaseVarIndex+event->Par2-1]);
                  valueStr = toString(UserVar[event->BaseVarIndex+event->Par2-1],0);
                  break;
                case 1:
                  valueStr = toString(UserVar[event->BaseVarIndex+event->Par2-1],ExtraTaskSettings.TaskDeviceValueDecimals[event->Par2-1]);
                  break;
                case 2:
                  if ( UserVar[event->BaseVarIndex+event->Par2-1] == 1) valueStr="true";
                    else valueStr = "false";
                  break;
                case 3:

                  valueStr = parseStringToEndKeepCase(string,4);
                  break;
                case 4:
                  valueInt = static_cast<int>(UserVar[event->BaseVarIndex+event->Par2-1]);
                  valueStr = parseStringKeepCase(ExtraTaskSettings.TaskDeviceFormula[event->Par2-1],valueInt);
                  break;
                case 5:

                  valueStr = parseStringToEnd(string,4);
                  break;
                case 6:

                  valueStr = parseStringToEnd(string,4);
                  break;
              }
              success = MQTTpublish(CPLUGIN_ID_014, topic.c_str(), valueStr.c_str(), false);
              if (loglevelActiveFor(LOG_LEVEL_INFO) && success) {
                String log = F("C014 : homie acknowledge: ");
                log += deviceName;
                log += F(" deviceNr:");
                log += deviceIndex;
                log += F(" valueNr:");
                log += event->Par2;
                log += F(" valueName:");
                log += valueName;
                log += F(" valueType:");
                log += Settings.TaskDevicePluginConfig[deviceIndex-1][event->Par2-1];
                log += F(" topic:");
                log += topic;
                log += F(" valueInt:");
                log += valueInt;
                log += F(" valueStr:");
                log += valueStr;
                addLog(LOG_LEVEL_INFO, log+" success!");
              }
              if (loglevelActiveFor(LOG_LEVEL_ERROR) && !success) {
                String log = F("C014 : homie acknowledge: ");
                log += deviceName;
                log += F(" var: ");
                log += valueName;
                log += F(" topic: ");
                log += topic;
                log += F(" value: ");
                log += valueStr;
                addLog(LOG_LEVEL_ERROR, log+" failed!");
              }
            } else
            {
# 916 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C014.ino"
              success = false;
            }
          }
        }
      }
  }

  return success;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C015.ino"
#ifdef USES_C015
# 21 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C015.ino"
#define CPLUGIN_015 
#define CPLUGIN_ID_015 15
#define _BLYNK_USE_DEFAULT_FREE_RAM 
#define BLYNK_TIMEOUT_MS 2000UL
#define BLYNK_HEARTBEAT 30
#define CPLUGIN_015_RECONNECT_INTERVAL 60000

#ifdef CPLUGIN_015_SSL
  #include <BlynkSimpleEsp8266_SSL.h>
  #define CPLUGIN_NAME_015 "Blynk SSL [TESTING]"

  #define CPLUGIN_015_DEFAULT_THUMBPRINT "FD C0 7D 8D 47 97 F7 E3 07 05 D3 4E E3 BB 8E 3D C0 EA BE 1C"
  #define C015_LOG_PREFIX "BL (ssl): "
#else
 #include <BlynkSimpleEsp8266.h>
 #define CPLUGIN_NAME_015 "Blynk [TESTING]"
 #define C015_LOG_PREFIX "BL: "
#endif


static unsigned long _C015_LastConnectAttempt[CONTROLLER_MAX] = {0,0,0};

void CPlugin_015_handleInterrupt() {



  backgroundtasks();
}


void Blynk_Run_c015(){

    if (Blynk.connected())
      Blynk.run();
}


bool CPlugin_015(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_015;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = true;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_015);
        break;
      }

    case CPLUGIN_INIT:
      {

       if (Blynk.connected()){
          addLog(LOG_LEVEL_INFO, F(C015_LOG_PREFIX "disconnect from server"));
          Blynk.disconnect();
       }
       break;
      }

    #ifdef CPLUGIN_015_SSL
      case CPLUGIN_WEBFORM_LOAD:
        {
          char thumbprint[60];
          LoadCustomControllerSettings(event->ControllerIndex,(byte*)&thumbprint, sizeof(thumbprint));
          if (strlen(thumbprint) != 59)
              strcpy(thumbprint, CPLUGIN_015_DEFAULT_THUMBPRINT);
          addFormTextBox(F("Server thumbprint string"), F("c015_thumbprint"), thumbprint, 60);
          success = true;
          break;
        }
    #endif

    case CPLUGIN_WEBFORM_SAVE:
      {
        success = true;
        if (isFormItemChecked(F("controllerenabled"))){
          for (byte i = 0; i < CONTROLLER_MAX; ++i) {
            byte ProtocolIndex = getProtocolIndex(Settings.Protocol[i]);
            if (i != event->ControllerIndex && Protocol[ProtocolIndex].Number == 15 && Settings.ControllerEnabled[i]) {
              success = false;


              addHtmlError(F("Only one enabled instance of blynk controller is supported"));
              break;
            }
          }

          _C015_LastConnectAttempt[event->ControllerIndex] = 0;

          #ifdef CPLUGIN_015_SSL
            char thumbprint[60];
            String error = F("Specify server thumbprint with exactly 59 symbols string like " CPLUGIN_015_DEFAULT_THUMBPRINT);
            if (!safe_strncpy(thumbprint, WebServer.arg("c015_thumbprint"), 60) || strlen(thumbprint) != 59) {
              addHtmlError(error);
            }
            SaveCustomControllerSettings(event->ControllerIndex,(byte*)&thumbprint, sizeof(thumbprint));
          #endif
        }
        break;
      }

     case CPLUGIN_PROTOCOL_SEND:
      {
        if (!Settings.ControllerEnabled[event->ControllerIndex])
          break;


        byte valueCount = getValueCountFromSensorType(event->sensorType);
        C015_queue_element element(event, valueCount);
        if (ExtraTaskSettings.TaskIndex != event->TaskIndex)
          PluginCall(PLUGIN_GET_DEVICEVALUENAMES, event, dummyString);

        for (byte x = 0; x < valueCount; x++)
        {
          bool isvalid;
          String formattedValue = formatUserVar(event, x, isvalid);
          if (!isvalid)

            formattedValue = F("");

          String valueName = ExtraTaskSettings.TaskDeviceValueNames[x];
          String valueFullName = ExtraTaskSettings.TaskDeviceName;
          valueFullName += F(".");
          valueFullName += valueName;
          String vPinNumberStr = valueName.substring(1, 4);
          int vPinNumber = vPinNumberStr.toInt();
          String log = F(C015_LOG_PREFIX);
          log += Blynk.connected()? F("(online): ") : F("(offline): ");
          if (vPinNumber > 0 && vPinNumber < 256){
            log += F("send ");
            log += valueFullName;
            log += F(" = ");
            log += formattedValue;
            log += F(" to blynk pin v");
            log += vPinNumber;
          }
          else{
            vPinNumber = -1;
            log += F("error got vPin number for ");
            log += valueFullName;
            log += F(", got not valid value: ");
            log += vPinNumberStr;
          }
          addLog(LOG_LEVEL_INFO, log);
          element.vPin[x] = vPinNumber;
          element.txt[x] = formattedValue;
        }
        success = C015_DelayHandler.addToQueue(element);
        scheduleNextDelayQueue(TIMER_C015_DELAY_QUEUE, C015_DelayHandler.getNextScheduleTime());
        break;
      }
  }
  return success;
}





bool do_process_c015_delay_queue(int controller_plugin_number, const C015_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  if (!Settings.ControllerEnabled[element.controller_idx])

    return true;

  if (!WiFiConnected()) {
    return false;
  }

  if (!Blynk_keep_connection_c015(element.controller_idx, ControllerSettings))
    return false;

  while (element.vPin[element.valuesSent] == -1) {


    if (element.checkDone(true))
      return true;
  }

  return element.checkDone(Blynk_send_c015(element.txt[element.valuesSent], element.vPin[element.valuesSent]));
}


boolean Blynk_keep_connection_c015(int controllerIndex, ControllerSettingsStruct& ControllerSettings){
  if (!WiFiConnected())
    return false;

  if (!Blynk.connected()){
    String auth = SecuritySettings.ControllerPassword[controllerIndex];
    boolean connectDefault = false;

    if (timePassedSince(_C015_LastConnectAttempt[controllerIndex]) < CPLUGIN_015_RECONNECT_INTERVAL){

      return false;
    }
    _C015_LastConnectAttempt[controllerIndex] = millis();

    #ifdef CPLUGIN_015_SSL
      char thumbprint[60];
      LoadCustomControllerSettings(controllerIndex,(byte*)&thumbprint, sizeof(thumbprint));
      if (strlen(thumbprint) != 59){
          addLog(LOG_LEVEL_INFO, C015_LOG_PREFIX "Saved thumprint value is not correct:");
          addLog(LOG_LEVEL_INFO, thumbprint);
          strcpy(thumbprint, CPLUGIN_015_DEFAULT_THUMBPRINT);
          addLog(LOG_LEVEL_INFO, C015_LOG_PREFIX "using default one:");
          addLog(LOG_LEVEL_INFO, thumbprint);
      }
    #endif

    String log = F(C015_LOG_PREFIX);

    if (ControllerSettings.UseDNS){
      String hostName = ControllerSettings.getHost();
      if (hostName.length() != 0){
        log += F("Connecting to custom blynk server ");
        log += ControllerSettings.getHostPortString();
        Blynk.config(auth.c_str(),
                     CPlugin_015_handleInterrupt,
                     hostName.c_str(),
                     ControllerSettings.Port
                     #ifdef CPLUGIN_015_SSL
                        ,thumbprint
                     #endif
                   );
      }
      else{
        log += F("Custom blynk server name not specified. ");
        connectDefault = true;
      }
    }
    else{
      IPAddress ip = ControllerSettings.getIP();
      if ((ip[0] + ip[1] + ip[2] + ip[3]) > 0){
        log += F("Connecting to custom blynk server ");
        log += ControllerSettings.getHostPortString();
        Blynk.config(auth.c_str(),
                     CPlugin_015_handleInterrupt,
                     ip,
                     ControllerSettings.Port
                     #ifdef CPLUGIN_015_SSL
                        ,thumbprint
                     #endif
                   );
      }
      else{
        log += F("Custom blynk server ip not specified. ");
        connectDefault = true;
      }
    }
    addLog(LOG_LEVEL_INFO, log);

    if (connectDefault){
      addLog(LOG_LEVEL_INFO, F(C015_LOG_PREFIX "Connecting to default server"));
      Blynk.config(auth.c_str(),
                   CPlugin_015_handleInterrupt,
                   BLYNK_DEFAULT_DOMAIN
                   #ifdef CPLUGIN_015_SSL
                      ,BLYNK_DEFAULT_PORT_SSL
                      ,thumbprint
                   #else
                      ,BLYNK_DEFAULT_PORT
                   #endif
                  );
    }

    #ifdef CPLUGIN_015_SSL
      if (!Blynk.connect()){
        if (!_blynkWifiClient.verify(thumbprint, BLYNK_DEFAULT_DOMAIN)){
          addLog(LOG_LEVEL_INFO, F(C015_LOG_PREFIX "thumbprint check FAILED! Check thumbprint in device settings and server thumbprint"));
          addLog(LOG_LEVEL_INFO, thumbprint);
        }
      }
    #else
      Blynk.connect();
    #endif
  }

  return Blynk.connected();
}


String Command_Blynk_Set_c015(struct EventStruct *event, const char* Line){


  if (!Blynk.connected())
      return F("Not connected to blynk server");

  int vPin = event->Par1;

  if ((vPin < 0) || (vPin > 255)){
    String err = F("Not correct blynk vPin number ");
    err += vPin;
    return err;
  }

  String data = parseString(Line, 3, true, false);

  if (data.length() == 0){
    String err = F("Skip sending empty data to blynk vPin ");
    err += vPin;
    return err;
  }

  String log = F(C015_LOG_PREFIX "(online): send blynk pin v");
  log += vPin;
  log += F(" = ");
  log += data;
  addLog(LOG_LEVEL_INFO, log);

  Blynk.virtualWrite(vPin, data);
  return return_command_success();
}


boolean Blynk_send_c015(const String& value, int vPin )
{
  Blynk.virtualWrite(vPin, value);
  unsigned long timer = millis() + Settings.MessageDelay;
  while (!timeOutReached(timer))
              backgroundtasks();
  return true;
}


BLYNK_WRITE_DEFAULT() {
  byte vPin = request.pin;
  float pinValue = param.asFloat();
  String log = F(C015_LOG_PREFIX "server set v");
  log += vPin;
  log += F(" to ");
  log += pinValue;
  addLog(LOG_LEVEL_INFO, log);
  String eventCommand = F("blynkv");
  eventCommand += vPin;
  eventCommand += F("=");
  eventCommand += pinValue;
  rulesProcessing(eventCommand);
}

BLYNK_CONNECTED() {



  String eventCommand = F("blynk_connected");
  rulesProcessing(eventCommand);

}


BLYNK_APP_CONNECTED() {
  String eventCommand = F("blynk_app_connected");
  rulesProcessing(eventCommand);

}


BLYNK_APP_DISCONNECTED() {
  String eventCommand = F("blynk_app_disconnected");
  rulesProcessing(eventCommand);

}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C016.ino"
#ifdef USES_C016
# 27 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C016.ino"
#define CPLUGIN_016 
#define CPLUGIN_ID_016 16
#define CPLUGIN_NAME_016 "Cache Controller [Experimental]"


ControllerCache_struct ControllerCache;

bool CPlugin_016(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case CPLUGIN_PROTOCOL_ADD:
      {
        Protocol[++protocolCount].Number = CPLUGIN_ID_016;
        Protocol[protocolCount].usesMQTT = false;
        Protocol[protocolCount].usesTemplate = true;
        Protocol[protocolCount].usesAccount = false;
        Protocol[protocolCount].usesPassword = false;
        Protocol[protocolCount].defaultPort = 80;
        Protocol[protocolCount].usesID = false;
        break;
      }

    case CPLUGIN_GET_DEVICENAME:
      {
        string = F(CPLUGIN_NAME_016);
        break;
      }

    case CPLUGIN_INIT:
      {
        MakeControllerSettings(ControllerSettings);
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        C016_DelayHandler.configureControllerSettings(ControllerSettings);
        ControllerCache.init();
        break;
      }

    case CPLUGIN_WEBFORM_LOAD:
      {

        break;
      }

    case CPLUGIN_WEBFORM_SAVE:
      {

        break;
      }

    case CPLUGIN_PROTOCOL_TEMPLATE:
      {
        event->String1 = "";
        event->String2 = "";
        break;
      }

    case CPLUGIN_PROTOCOL_SEND:
      {

        byte valueCount = getValueCountFromSensorType(event->sensorType);
        C016_queue_element element(event, valueCount, getUnixTime());
        success = ControllerCache.write((uint8_t*)&element, sizeof(element));







        break;
      }

    case CPLUGIN_FLUSH:
      {
        process_c016_delay_queue();
        delay(0);
        break;
      }

  }
  return success;
}






bool do_process_c016_delay_queue(int controller_number, const C016_queue_element& element, ControllerSettingsStruct& ControllerSettings) {
  return true;







}





bool C016_startCSVdump() {
  ControllerCache.resetpeek();
  return ControllerCache.isInitialized();
}

String C016_getCacheFileName(bool& islast) {
  return ControllerCache.getPeekCacheFileName(islast);
}

bool C016_deleteOldestCacheBlock() {
  return ControllerCache.deleteOldestCacheBlock();
}

bool C016_getCSVline(
  unsigned long& timestamp,
  byte& controller_idx,
  byte& TaskIndex,
  byte& sensorType,
  byte& valueCount,
  float& val1,
  float& val2,
  float& val3,
  float& val4)
{
  C016_queue_element element;
  bool result = ControllerCache.peek((uint8_t*)&element, sizeof(element));
  timestamp = element.timestamp;
  controller_idx = element.controller_idx;
  TaskIndex = element.TaskIndex;
  sensorType = element.sensorType;
  valueCount = element.valueCount;
  val1 = element.values[0];
  val2 = element.values[1];
  val3 = element.values[2];
  val4 = element.values[3];
  return result;
}

#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C017.ino"
#ifdef USES_C017
# 16 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_C017.ino"
#define CPLUGIN_017 
#define CPLUGIN_ID_017 17
#define CPLUGIN_NAME_017 "Zabbix"
#include <ArduinoJson.h>

bool CPlugin_017(byte function, struct EventStruct *event, String &string)
{
  bool success = false;

  switch (function)
  {
  case CPLUGIN_PROTOCOL_ADD:
  {
    Protocol[++protocolCount].Number = CPLUGIN_ID_017;
    Protocol[protocolCount].usesMQTT = false;
    Protocol[protocolCount].usesTemplate = false;
    Protocol[protocolCount].usesAccount = false;
    Protocol[protocolCount].usesPassword = false;
    Protocol[protocolCount].usesID = false;
    Protocol[protocolCount].defaultPort = 10051;
    break;
  }

  case CPLUGIN_GET_DEVICENAME:
  {
    string = F(CPLUGIN_NAME_017);
    break;
  }

  case CPLUGIN_PROTOCOL_SEND:
  {
    byte valueCount = getValueCountFromSensorType(event->sensorType);
    C017_queue_element element(event);

    MakeControllerSettings(ControllerSettings);
    LoadControllerSettings(event->ControllerIndex, ControllerSettings);

    for (byte x = 0; x < valueCount; x++)
    {
      element.txt[x] = formatUserVarNoCheck(event, x);
    }
    success = C017_DelayHandler.addToQueue(element);
    scheduleNextDelayQueue(TIMER_C017_DELAY_QUEUE, C017_DelayHandler.getNextScheduleTime());
    break;
  }

  case CPLUGIN_FLUSH:
  {
    process_c017_delay_queue();
    delay(0);
    break;
  }
  }
  return success;
}

bool do_process_c017_delay_queue(int controller_number, const C017_queue_element &element, ControllerSettingsStruct &ControllerSettings)
{
  byte valueCount = getValueCountFromSensorType(element.sensorType);
  if (valueCount == 0)
    return true;

  if (!WiFiConnected(10))
  {
    return false;
  }

  WiFiClient client;
  if (!ControllerSettings.connectToHost(client))
  {
    connectionFailures++;
    addLog(LOG_LEVEL_ERROR, String(F("ZBX: Cannot connect")));
    return false;
  }
  statusLED(true);
  if (connectionFailures)
    connectionFailures--;

  LoadTaskSettings(element.TaskIndex);

  const size_t capacity = JSON_ARRAY_SIZE(VARS_PER_TASK) + JSON_OBJECT_SIZE(2) + VARS_PER_TASK * JSON_OBJECT_SIZE(3) + VARS_PER_TASK * 50;
  DynamicJsonDocument root(capacity);


  root[F("request")] = F("sender data");
  JsonArray data = root.createNestedArray(F("data"));

  for (uint8_t i = 0; i < valueCount; i++)
  {
    if (ExtraTaskSettings.TaskDeviceValueNames[i][0] == 0)
      continue;
    JsonObject block = data.createNestedObject();
    block[F("host")] = Settings.Name;
    block[F("key")] = ExtraTaskSettings.TaskDeviceValueNames[i];
    block[F("value")] = atof(element.txt[i].c_str());
  }


  char packet_header[] = "ZBXD\1";
  String JSON_packet_content="";

  serializeJson(root, JSON_packet_content);
  uint64_t payload_len = JSON_packet_content.length();



  client.write(packet_header, sizeof(packet_header) - 1);
  client.write((char *)&payload_len, sizeof(payload_len));
  client.write(JSON_packet_content.c_str(), payload_len);

  client.stop();
  return true;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_CPlugin_DomoticzHelper.ino"






String humStatDomoticz(struct EventStruct *event, byte rel_index) {
  const int hum = UserVar[event->BaseVarIndex + rel_index];

  if (hum < 30) { return formatUserVarDomoticz(2); }

  if (hum < 40) { return formatUserVarDomoticz(0); }

  if (hum < 59) { return formatUserVarDomoticz(1); }
  return formatUserVarDomoticz(3);
}

int mapRSSItoDomoticz() {
  long rssi = WiFi.RSSI();

  if (-50 < rssi) { return 10; }

  if (rssi <= -98) { return 0; }
  rssi = rssi + 97;
  return (rssi / 5) + 1;
}

int mapVccToDomoticz() {
  #if FEATURE_ADC_VCC


  if (vcc < 2.6) { return 0; }
  return (vcc - 2.6) * 100;
  #else
  return 255;
  #endif
}


String formatUserVarDomoticz(struct EventStruct *event, byte rel_index) {
  String text = formatUserVarNoCheck(event, rel_index);

  text += ';';
  return text;
}

String formatUserVarDomoticz(int value) {
  String text;

  text += value;
  text.trim();
  text += ';';
  return text;
}

String formatDomoticzSensorType(struct EventStruct *event) {
  String values;

  switch (event->sensorType)
  {
    case SENSOR_TYPE_SINGLE:
      values = formatUserVarDomoticz(event, 0);
      break;
    case SENSOR_TYPE_LONG:
      values = (unsigned long)UserVar[event->BaseVarIndex] + ((unsigned long)UserVar[event->BaseVarIndex + 1] << 16);
      break;
    case SENSOR_TYPE_DUAL:
      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      break;
    case SENSOR_TYPE_TEMP_HUM:



      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += humStatDomoticz(event, 1);
      break;
    case SENSOR_TYPE_TEMP_HUM_BARO:



      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += humStatDomoticz(event, 1);
      values += formatUserVarDomoticz(event, 2);
      values += formatUserVarDomoticz(0);
      break;
    case SENSOR_TYPE_TEMP_BARO:



      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += formatUserVarDomoticz(0);
      values += formatUserVarDomoticz(0);
      break;
    case SENSOR_TYPE_TEMP_EMPTY_BARO:



      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 2);
      values += formatUserVarDomoticz(0);
      values += formatUserVarDomoticz(0);
      break;
    case SENSOR_TYPE_TRIPLE:
      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += formatUserVarDomoticz(event, 2);
      break;
    case SENSOR_TYPE_QUAD:
      values = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += formatUserVarDomoticz(event, 2);
      values += formatUserVarDomoticz(event, 3);
      break;
    case SENSOR_TYPE_WIND:



      values = formatUserVarDomoticz(event, 0);
      values += getBearing(UserVar[event->BaseVarIndex]);
      values += ";";

      values += toString((UserVar[event->BaseVarIndex + 1] * 10), ExtraTaskSettings.TaskDeviceValueDecimals[1]);
      values += ";";
      values += toString((UserVar[event->BaseVarIndex + 2] * 10), ExtraTaskSettings.TaskDeviceValueDecimals[2]);
      values += ";";
      values += formatUserVarDomoticz(0);
      values += formatUserVarDomoticz(0);
      break;
    case SENSOR_TYPE_SWITCH:
    case SENSOR_TYPE_DIMMER:


      break;
    default:
    {
      String log = F("Domoticz Controller: Not yet implemented sensor type: ");
      log += event->sensorType;
      log += F(" idx: ");
      log += event->idx;
      addLog(LOG_LEVEL_ERROR, log);
      break;
    }
  }


  int index_last_char = values.length() - 1;

  if ((index_last_char > 0) && (values.charAt(index_last_char) == ';')) {
    values.setCharAt(index_last_char, ' ');
  }
  values.trim();
  {
    String log = F(" Domoticz: Sensortype: ");
    log += event->sensorType;
    log += F(" idx: ");
    log += event->idx;
    log += F(" values: ");
    log += values;
    addLog(LOG_LEVEL_INFO, log);
  }
  return values;
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_CPlugin_SensorTypeHelper.ino"



byte getValueCountFromSensorType(byte sensorType)
{
  switch (sensorType)
  {
    case SENSOR_TYPE_NONE:
      return 0;
    case SENSOR_TYPE_SINGLE:
    case SENSOR_TYPE_SWITCH:
    case SENSOR_TYPE_DIMMER:
      return 1;
    case SENSOR_TYPE_LONG:
      return 1;
    case SENSOR_TYPE_TEMP_HUM:
    case SENSOR_TYPE_TEMP_BARO:
    case SENSOR_TYPE_DUAL:
      return 2;
    case SENSOR_TYPE_TEMP_HUM_BARO:
    case SENSOR_TYPE_TEMP_EMPTY_BARO:
    case SENSOR_TYPE_TRIPLE:
    case SENSOR_TYPE_WIND:
      return 3;
    case SENSOR_TYPE_QUAD:
      return 4;
  }
  addLog(LOG_LEVEL_ERROR, F("getValueCountFromSensorType: Unknown sensortype"));
  return 0;
}

String getSensorTypeLabel(byte sensorType) {
  switch (sensorType) {
    case SENSOR_TYPE_SINGLE: return F("Single");
    case SENSOR_TYPE_TEMP_HUM: return F("Temp / Hum");
    case SENSOR_TYPE_TEMP_BARO: return F("Temp / Baro");
    case SENSOR_TYPE_TEMP_HUM_BARO: return F("Temp / Hum / Baro");
    case SENSOR_TYPE_DUAL: return F("Dual");
    case SENSOR_TYPE_TRIPLE: return F("Triple");
    case SENSOR_TYPE_QUAD: return F("Quad");
    case SENSOR_TYPE_SWITCH: return F("Switch");
    case SENSOR_TYPE_DIMMER: return F("Dimmer");
    case SENSOR_TYPE_LONG: return F("Long");
    case SENSOR_TYPE_WIND: return F("Wind");
  }
  return "";
}

void sensorTypeHelper_webformLoad_allTypes(struct EventStruct *event, byte pconfigIndex)
{
  byte optionValues[11];

  optionValues[0] = SENSOR_TYPE_SINGLE;
  optionValues[1] = SENSOR_TYPE_TEMP_HUM;
  optionValues[2] = SENSOR_TYPE_TEMP_BARO;
  optionValues[3] = SENSOR_TYPE_TEMP_HUM_BARO;
  optionValues[4] = SENSOR_TYPE_DUAL;
  optionValues[5] = SENSOR_TYPE_TRIPLE;
  optionValues[6] = SENSOR_TYPE_QUAD;
  optionValues[7] = SENSOR_TYPE_SWITCH;
  optionValues[8] = SENSOR_TYPE_DIMMER;
  optionValues[9] = SENSOR_TYPE_LONG;
  optionValues[10] = SENSOR_TYPE_WIND;
  sensorTypeHelper_webformLoad(event, pconfigIndex, 11, optionValues);
}

void sensorTypeHelper_webformLoad_header()
{
  addFormSubHeader(F("Output Configuration"));
}

void sensorTypeHelper_webformLoad_simple(struct EventStruct *event, byte pconfigIndex)
{
  sensorTypeHelper_webformLoad_header();

  byte optionValues[4];
  optionValues[0] = SENSOR_TYPE_SINGLE;
  optionValues[1] = SENSOR_TYPE_DUAL;
  optionValues[2] = SENSOR_TYPE_TRIPLE;
  optionValues[3] = SENSOR_TYPE_QUAD;
  sensorTypeHelper_webformLoad(event, pconfigIndex, 4, optionValues);
}

void sensorTypeHelper_webformLoad(struct EventStruct *event, byte pconfigIndex, int optionCount, const byte options[])
{
  byte choice = PCONFIG(pconfigIndex);
  byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[event->TaskIndex]);

  if (getValueCountFromSensorType(choice) != Device[DeviceIndex].ValueCount) {

    choice = Device[DeviceIndex].VType;
    PCONFIG(pconfigIndex) = choice;
  }
  addRowLabel(F("Output Data Type"));
  addSelector_Head(PCONFIG_LABEL(pconfigIndex), false);

  for (byte x = 0; x < optionCount; x++)
  {
    String name = getSensorTypeLabel(options[x]);
    bool disabled = false;
    addSelector_Item(name,
                     options[x],
                     choice == options[x],
                     disabled,
                     "");
  }
  addSelector_Foot();
  addFormNote(F("Changing 'Output Data Type' may affect behavior of some controllers (e.g. Domoticz)"));


}

void sensorTypeHelper_saveSensorType(struct EventStruct *event, byte pconfigIndex)
{
  pconfig_webformSave(event, pconfigIndex);
  sensorTypeHelper_setSensorType(event, pconfigIndex);
  ExtraTaskSettings.clearUnusedValueNames(getValueCountFromSensorType(PCONFIG(pconfigIndex)));
}

void sensorTypeHelper_setSensorType(struct EventStruct *event, byte pconfigIndex)
{
  byte sensorType = PCONFIG(pconfigIndex);
  byte DeviceIndex = getDeviceIndex(Settings.TaskDeviceNumber[event->TaskIndex]);

  Device[DeviceIndex].VType = sensorType;
  Device[DeviceIndex].ValueCount = getValueCountFromSensorType(sensorType);
}

void sensorTypeHelper_saveOutputSelector(struct EventStruct *event, byte pconfigIndex, byte valueIndex, const String& defaultValueName)
{
  if (defaultValueName.equals(ExtraTaskSettings.TaskDeviceValueNames[valueIndex])) {
    ZERO_FILL(ExtraTaskSettings.TaskDeviceValueNames[valueIndex]);
  }
  pconfig_webformSave(event, pconfigIndex);
}

void pconfig_webformSave(struct EventStruct *event, byte pconfigIndex)
{
  PCONFIG(pconfigIndex) = getFormItemInt(PCONFIG_LABEL(pconfigIndex), 0);
}

void sensorTypeHelper_loadOutputSelector(
  struct EventStruct *event, byte pconfigIndex, byte valuenr,
  int optionCount, const String options[])
{
  byte choice = PCONFIG(pconfigIndex);
  String label = F("Value ");

  label += (valuenr + 1);
  addFormSelector(label, PCONFIG_LABEL(pconfigIndex), optionCount, options, NULL, choice);
}
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_N001_Email.ino"
#ifdef USES_N001




#define NPLUGIN_001 
#define NPLUGIN_ID_001 1
#define NPLUGIN_NAME_001 "Email (SMTP)"

#define NPLUGIN_001_TIMEOUT 5000



boolean NPlugin_001(byte function, struct EventStruct *event, String& string)
{
 boolean success = false;

 switch (function) {
 case NPLUGIN_PROTOCOL_ADD:
 {
  Notification[++notificationCount].Number = NPLUGIN_ID_001;
  Notification[notificationCount].usesMessaging = true;
  Notification[notificationCount].usesGPIO = 0;
  break;
 }

 case NPLUGIN_GET_DEVICENAME:
 {
  string = F(NPLUGIN_NAME_001);
  break;
 }
# 49 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_N001_Email.ino"
 case NPLUGIN_NOTIFY:
 {
  MakeNotificationSettings(NotificationSettings);
  LoadNotificationSettings(event->NotificationIndex, (byte*)&NotificationSettings, sizeof(NotificationSettingsStruct));
    NotificationSettings.validate();
  String subject = NotificationSettings.Subject;
  String body = "";
  if (event->String1.length() > 0)
   body = event->String1;
  else
   body = NotificationSettings.Body;
  subject = parseTemplate(subject, subject.length());
  body = parseTemplate(body, body.length());
  NPlugin_001_send(NotificationSettings, subject, body);
  success = true;
  break;
 }
 }
 return success;
}

boolean NPlugin_001_send(const NotificationSettingsStruct& notificationsettings, const String& aSub, String& aMesg)
{

 boolean myStatus = false;


 WiFiClient client;
 client.setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
 String aHost = notificationsettings.Server;
 addLog(LOG_LEVEL_DEBUG, String(F("EMAIL: Connecting to ")) + aHost + notificationsettings.Port);
 if (!connectClient(client, aHost.c_str(), notificationsettings.Port)) {
  addLog(LOG_LEVEL_ERROR, String(F("EMAIL: Error connecting to ")) + aHost + notificationsettings.Port);
  myStatus = false;
 }else {
  String mailheader = F(
   "From: $nodename <$emailfrom>\r\n"
   "To: $ato\r\n"
   "Subject: $subject\r\n"
   "Reply-To: $nodename <$emailfrom>\r\n"
   "MIME-VERSION: 1.0\r\n"
   "Content-type: text/html; charset=UTF-8\r\n"
   "X-Mailer: EspEasy v$espeasyversion\r\n\r\n"
   );

    String email_address = notificationsettings.Sender;
  int pos_less = email_address.indexOf('<');
  if (pos_less == -1) {

   mailheader.replace(String(F("$nodename")), Settings.Name);
   mailheader.replace(String(F("$emailfrom")), notificationsettings.Sender);
  } else {
   String senderName = email_address.substring(0, pos_less);
   senderName.replace("\"", "");
   String address = email_address.substring(pos_less + 1);
   address.replace("<", "");
   address.replace(">", "");
   address.trim();
   senderName.trim();
   mailheader.replace(String(F("$nodename")), senderName);
   mailheader.replace(String(F("$emailfrom")), address);
  }

  mailheader.replace(String(F("$nodename")), Settings.Name);
  mailheader.replace(String(F("$emailfrom")), notificationsettings.Sender);
  mailheader.replace(String(F("$ato")), notificationsettings.Receiver);
  mailheader.replace(String(F("$subject")), aSub);
  mailheader.replace(String(F("$espeasyversion")), String(BUILD));
  aMesg.replace("\r", F("<br/>"));



  while (true) {
   if (!NPlugin_001_MTA(client, "", F("220 "))) break;
   if (!NPlugin_001_MTA(client, String(F("EHLO ")) + notificationsettings.Domain, F("250 "))) break;
   if (!NPlugin_001_Auth(client, notificationsettings.User, notificationsettings.Pass)) break;
   if (!NPlugin_001_MTA(client, String(F("MAIL FROM:<")) + notificationsettings.Sender + ">", F("250 "))) break;

   bool nextAddressAvailable = true;
   int i = 0;
   String emailTo = "";
   if (!getNextMailAddress(notificationsettings.Receiver, emailTo, i)) {
    addLog(LOG_LEVEL_ERROR, F("Email: No recipient given"));
    break;
   }
   while (nextAddressAvailable) {
    String mailFound = F("Email: To ");
    mailFound += emailTo;
    addLog(LOG_LEVEL_INFO, mailFound);
    if (!NPlugin_001_MTA(client, String(F("RCPT TO:<")) + emailTo + ">", F("250 "))) break;
    ++i;
    nextAddressAvailable = getNextMailAddress(notificationsettings.Receiver, emailTo, i);
   }

   if (!NPlugin_001_MTA(client, F("DATA"), F("354 "))) break;
   if (!NPlugin_001_MTA(client, mailheader + aMesg + String(F("\r\n.\r\n")), F("250 "))) break;

   myStatus = true;
   break;
  }

  client.flush();
  client.stop();

  if (myStatus == true) {
   addLog(LOG_LEVEL_INFO, F("EMAIL: Connection Closed Successfully"));
  }else {
   String log = F("EMAIL: Connection Closed With Error. Used header: ");
   log += mailheader;
   addLog(LOG_LEVEL_ERROR, log);
  }
 }
 return myStatus;
}

boolean NPlugin_001_Auth(WiFiClient& client, const String& user, const String& pass)
{
 if (user.length() == 0 || pass.length() == 0) {

  return true;
 }
 if (!NPlugin_001_MTA(client, String(F("AUTH LOGIN")), F("334 "))) return false;
 base64 encoder;
 String auth;
 auth = encoder.encode(user);
 if (!NPlugin_001_MTA(client, auth, F("334 "))) return false;
 auth = encoder.encode(pass);
 if (!NPlugin_001_MTA(client, auth, F("235 "))) return false;

 return true;
}

boolean NPlugin_001_MTA(WiFiClient& client, const String& aStr, const String &aWaitForPattern)
{
 addLog(LOG_LEVEL_DEBUG, aStr);

 if (aStr.length()) client.println(aStr);


 unsigned long timer = millis() + NPLUGIN_001_TIMEOUT;
 backgroundtasks();
 while (true) {
  if (timeOutReached(timer)) {
   String log = F("NPlugin_001_MTA: timeout. ");
   log += aStr;
   addLog(LOG_LEVEL_ERROR, log);
   return false;
  }

  delay(0);


  String line;
  safeReadStringUntil(client, line, '\n');

  addLog(LOG_LEVEL_DEBUG, line);

  if (line.indexOf(aWaitForPattern) >= 0) {
   return true;
  }
 }

 return false;
}

bool getNextMailAddress(const String& data, String& address, int index)
{
 int found = 0;
 int strIndex[] = { 0, -1 };
 const int maxIndex = data.length() - 1;

 for (int i = 0; i <= maxIndex && found <= index; i++) {
  if (data.charAt(i) == ',' || i == maxIndex) {
   found++;
   strIndex[0] = strIndex[1] + 1;
   strIndex[1] = (i == maxIndex) ? i + 1 : i;
  }
 }
 if (found > index) {
  address = data.substring(strIndex[0], strIndex[1]);
  return true;
 }
 return false;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_N002_Buzzer.ino"
#ifdef USES_N002




#define NPLUGIN_002 
#define NPLUGIN_ID_002 2
#define NPLUGIN_NAME_002 "Buzzer"

boolean NPlugin_002(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case NPLUGIN_PROTOCOL_ADD:
      {
        Notification[++notificationCount].Number = NPLUGIN_ID_002;
        Notification[notificationCount].usesMessaging = false;
        Notification[notificationCount].usesGPIO=1;
        break;
      }

    case NPLUGIN_GET_DEVICENAME:
      {
        string = F(NPLUGIN_NAME_002);
        break;
      }
# 45 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_N002_Buzzer.ino"
    case NPLUGIN_NOTIFY:
      {
        MakeNotificationSettings(NotificationSettings);
        LoadNotificationSettings(event->NotificationIndex, (byte*)&NotificationSettings, sizeof(NotificationSettingsStruct));
        NotificationSettings.validate();

        #ifndef ESP32

        tone_espEasy(NotificationSettings.Pin1, 500, 500);
        #endif
        success = true;
      }

  }
  return success;
}
#endif
# 1 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_P001_Switch.ino"
#ifdef USES_P001
# 31 "C:/Users/Albert/Documents/GitHub/ESPEasy/src/_P001_Switch.ino"
#define PLUGIN_001 
#define PLUGIN_ID_001 1
#define PLUGIN_NAME_001 "Switch input - Switch"
#define PLUGIN_VALUENAME1_001 "State"
#ifdef USE_SERVO
  Servo servo1;
  Servo servo2;
#endif

#define PLUGIN_001_TYPE_SWITCH 0
#define PLUGIN_001_TYPE_DIMMER 3
#define PLUGIN_001_BUTTON_TYPE_NORMAL_SWITCH 0
#define PLUGIN_001_BUTTON_TYPE_PUSH_ACTIVE_LOW 1
#define PLUGIN_001_BUTTON_TYPE_PUSH_ACTIVE_HIGH 2
#define PLUGIN_001_DOUBLECLICK_MIN_INTERVAL 1000
#define PLUGIN_001_DOUBLECLICK_MAX_INTERVAL 3000
#define PLUGIN_001_LONGPRESS_MIN_INTERVAL 1000
#define PLUGIN_001_LONGPRESS_MAX_INTERVAL 5000
#define PLUGIN_001_DC_DISABLED 0
#define PLUGIN_001_DC_LOW 1
#define PLUGIN_001_DC_HIGH 2
#define PLUGIN_001_DC_BOTH 3
#define PLUGIN_001_LONGPRESS_DISABLED 0
#define PLUGIN_001_LONGPRESS_LOW 1
#define PLUGIN_001_LONGPRESS_HIGH 2
#define PLUGIN_001_LONGPRESS_BOTH 3

boolean Plugin_001_read_switch_state(struct EventStruct *event) {
  byte pinNumber = CONFIG_PIN1;
  const uint32_t key = createKey(PLUGIN_ID_001, pinNumber);
  if (existPortStatus(key)) {
    return Plugin_001_read_switch_state(pinNumber, globalMapPortStatus[key].mode);
  }
  return false;
}

boolean Plugin_001_read_switch_state(byte pinNumber, byte pinMode) {
  bool canRead = false;
  switch (pinMode)
  {
    case PIN_MODE_UNDEFINED:
    case PIN_MODE_INPUT:
    case PIN_MODE_INPUT_PULLUP:
    case PIN_MODE_OUTPUT:
      canRead = true;
      break;
    case PIN_MODE_PWM:
      break;
    case PIN_MODE_SERVO:
      break;
    case PIN_MODE_OFFLINE:
      break;
    default:
      break;
  }
  if (!canRead) return false;


  return digitalRead(pinNumber) == HIGH;
}

boolean Plugin_001(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;





  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_001;
        Device[deviceCount].Type = DEVICE_TYPE_SINGLE;
        Device[deviceCount].VType = SENSOR_TYPE_SWITCH;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = true;
        Device[deviceCount].InverseLogicOption = true;
        Device[deviceCount].FormulaOption = false;
        Device[deviceCount].ValueCount = 1;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].TimerOptional = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_001);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_001));
        break;
      }

    case PLUGIN_GET_DEVICEGPIONAMES:
      {







        event->String1 = formatGpioName_bidirectional("");
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {

        const uint32_t key = createKey(PLUGIN_ID_001,CONFIG_PIN1);
        if (existPortStatus(key)) {
          globalMapPortStatus[key].previousTask = event->TaskIndex;
        }

        String options[2];
        options[0] = F("Switch");
        options[1] = F("Dimmer");
        int optionValues[2] = { PLUGIN_001_TYPE_SWITCH, PLUGIN_001_TYPE_DIMMER };
        const byte switchtype =