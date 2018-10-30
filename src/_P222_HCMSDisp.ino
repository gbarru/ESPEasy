
#ifdef USES_P222
//#######################################################################################################
//#################################### Plugin 222: HCMS_2915 ############################################
//#######################################################################################################

//uncomment one of the following as needed
#ifdef PLUGIN_BUILD_DEV
//#ifdef PLUGIN_BUILD_TESTING

#define PLUGIN_222
#define PLUGIN_ID_222     222               //plugin id
#define PLUGIN_NAME_222   "Display - HCMS-2915 [DEVELOPMENT]"     //"Plugin Name" is what will be dislpayed in the selection list
//#define PLUGIN_VALUENAME1_xxx "output1"     //variable output of the plugin. The label is in quotation marks
//#define PLUGIN_VALUENAME2_xxx "output2"     //multiple outputs are supported
#define PLUGIN_222_DEBUG  true             //set to true for extra log info in the debug
//#define PLUGIN_VALUENAME1_222 "dataPin"
//#define PLUGIN_VALUENAME2_222 "registerSelect"
//#define PLUGIN_VALUENAME3_222 "clockPin"
//#define PLUGIN_VALUENAME4_222 "chipEnable"
//#define PLUGIN_VALUENAME5_222 "reset"
//#define PLUGIN_VALUENAME6_222 "displayLenght"
//#define PLUGIN_VALUENAME7_222 "brightness"

//#ifndef CONFIG
//#define CONFIG(n) (Settings.TaskDevicePluginConfig[event->TaskIndex][n])
//#endif

#include <LedDisplay.h>

/*
PIN/port configuration is stored in the following:
Settings.TaskDevicePin1[event->TaskIndex] - The first GPIO pin selected within the task
Settings.TaskDevicePin2[event->TaskIndex] - The second GPIO pin selected within the task
Settings.TaskDevicePin3[event->TaskIndex] - The third GPIO pin selected within the task
Settings.TaskDevicePort[event->TaskIndex] - The port in case the device has multiple in/out pins

Custom configuration is stored in the following:
Settings.TaskDevicePluginConfig[event->TaskIndex][x]
x can be between 1 - 8 and can store values between -32767 - 32768 (16 bit)
*/
LedDisplay *Plugin_222_M = NULL;

boolean Plugin_222(byte function, struct EventStruct *event, String& string)
{
  //function: reason the plugin was called
  //event: ??add description here??
  // string: ??add description here??
  boolean success = false;
  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
        Device[++deviceCount].Number = PLUGIN_ID_222;
        Device[deviceCount].Type = DEVICE_TYPE_TRIPLE;  //how the device is connected
        Device[deviceCount].VType = SENSOR_TYPE_NONE; //type of value the plugin will return, used only for Domoticz
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = false;
        Device[deviceCount].ValueCount = 0;             //number of output variables. The value should match the number of keys PLUGIN_VALUENAME1_xxx
        Device[deviceCount].SendDataOption = false;
        Device[deviceCount].TimerOption = false;
        Device[deviceCount].TimerOptional = false;
        Device[deviceCount].GlobalSyncOption = true;
        Device[deviceCount].DecimalsOnly = true;
        break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_222);
      break;
    }

    case PLUGIN_GET_DEVICEGPIONAMES:
    {
      event->String1 = F("GPIO &rarr; dataPin");
      event->String2 = F("GPIO &rarr; registerSelect");
      event->String3 = F("GPIO &rarr; clockPin");
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
/*
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_222));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_222));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_222));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_222));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[4], PSTR(PLUGIN_VALUENAME5_222));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[5], PSTR(PLUGIN_VALUENAME6_222));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[6], PSTR(PLUGIN_VALUENAME7_222));
*/
    break;
    }

      //called when the user opens the module configuration page
      //it allows to add a new row for each output variable of the plugin

    case PLUGIN_WEBFORM_LOAD:
    {
      //Settings.TaskDevicePluginConfig[event->TaskIndex][x] (custom configuration)
      //String dropdown[5] = { F("option1"), F("option2"), F("option3"), F("option4")};
      //addFormSelector(string, F("drop-down menu"), F("plugin_xxx_displtype"), 4, dropdown, NULL, Settings.TaskDevicePluginConfig[event->TaskIndex][0]);
      addFormPinSelect(F("GPIO &rarr; ChipEnable"), F("chipEnable"), (Settings.TaskDevicePluginConfig[event->TaskIndex][3]));
      addFormPinSelect(F("GPIO &rarr; Reset"), F("reset"), (Settings.TaskDevicePluginConfig[event->TaskIndex][4]));
      addFormNote(F("HCMS:  1st=DataPin, 2nd=registerSelect, 3rd= clockPin, 4th= Enable 5th=Reset"));
      addFormNumericBox(F("DisplayLenght"), F("displaylenght"), (int)Settings.TaskDevicePluginConfig[event->TaskIndex][5], 0, 15);
      addFormNumericBox(F("Brightness"), F("brightness"), (int)Settings.TaskDevicePluginConfig[event->TaskIndex][6], 0, 15);
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      //this case defines the code to be executed when the form is submitted
      //the plugin settings should be saved to Settings.TaskDevicePluginConfig[event->TaskIndex][x]
      //ping configuration should be read from Settings.TaskDevicePin1[event->TaskIndex] and stored
      //      CONFIG(0) = getFormItemInt(F("dataPin"));
      //      CONFIG(1) = getFormItemInt(F("registerSelect"));
      //      CONFIG(2) = getFormItemInt(F("clockPin"));
      //      CONFIG(3) = getFormItemInt(F("chipEnable"));
      //      CONFIG(4) = getFormItemInt(F("reset"));
      //      CONFIG(5) = getFormItemInt(F("displayLenght"));
      Settings.TaskDevicePluginConfig[event->TaskIndex][3] = (int8_t)getFormItemInt(F("chipEnable"));
      Settings.TaskDevicePluginConfig[event->TaskIndex][4] = (int8_t)getFormItemInt(F("reset"));
      //  Settings.TaskDevicePin[event->TaskIndex][3] = (int8_t) getFormItemInt(F("ChipEnable"));
      //  Settings.TaskDevicePin[event->TaskIndex][4] = (int8_t) getFormItemInt(F("Reset"));

      Settings.TaskDevicePluginConfig[event->TaskIndex][5] = getFormItemInt(F("displaylenght"));
      Settings.TaskDevicePluginConfig[event->TaskIndex][6] = getFormItemInt(F("brightness"));

      String log = F("HCMS : SAVE HCMS Pins : ");
      log += Settings.TaskDevicePin1[event->TaskIndex]; log += F(" ");
      log += Settings.TaskDevicePin2[event->TaskIndex]; log += F(" ");
      log += Settings.TaskDevicePin3[event->TaskIndex]; log += F(" ");
      log += Settings.TaskDevicePluginConfig[event->TaskIndex][3]; log += F(" ");
      log += Settings.TaskDevicePluginConfig[event->TaskIndex][4]; log += F(" ");
      log += Settings.TaskDevicePluginConfig[event->TaskIndex][5]; log += F(" ");
      log += Settings.TaskDevicePluginConfig[event->TaskIndex][6]; log += F(" ");
      addLog(LOG_LEVEL_DEBUG, log);

      success = true;
      break;
    }

    case PLUGIN_INIT:
    {
      if (!Plugin_222_M){
        Plugin_222_M = new LedDisplay(Settings.TaskDevicePin1[event->TaskIndex], Settings.TaskDevicePin2[event->TaskIndex], Settings.TaskDevicePin3[event->TaskIndex], Settings.TaskDevicePluginConfig[event->TaskIndex][3], Settings.TaskDevicePluginConfig[event->TaskIndex][4], Settings.TaskDevicePluginConfig[event->TaskIndex][5]);
        // initialize the display library:
        Plugin_222_M->begin();
        // set the brightness of the display:
        Plugin_222_M->setBrightness(Settings.TaskDevicePluginConfig[event->TaskIndex][6]);
        Plugin_222_M->clear();

        String log = F("HCMS : Init HCMS ");
        addLog(LOG_LEVEL_DEBUG, log);
        }

      success = true;
      break;
    }

    case PLUGIN_READ:
    {

      success = true;
      break;
    }

    case PLUGIN_WRITE:
    {
      //this case defines code to be executed when the plugin executes an action (command).
      //Commands can be accessed via rules or via http.
      //As an example, http://192.168.1.12//control?cmd=dothis
      //implies that there exists the comamnd "dothis"
      //  if (plugin_not_initialised)
      //    break;
/*
      //parse string to extract the command
      String tmpString  = string;
      int argIndex = tmpString.indexOf(',');
      if (argIndex)
        tmpString = tmpString.substring(0, argIndex);

      String tmpStr = string;
      int comma1 = tmpStr.indexOf(',');
      if (tmpString.equalsIgnoreCase(F("dothis"))) {
        //do something
*/
      Plugin_222_M->setBrightness(Settings.TaskDevicePluginConfig[event->TaskIndex][6]);
      Plugin_222_M->clear();
      Plugin_222_M->home();
      Plugin_222_M->print("Hello");

        success = true;     //set to true only if plugin has executed a command successfully
    break;
    }

	  case PLUGIN_EXIT:
    {
	  //perform cleanup tasks here. For example, free memory
	  break;
	  }

    case PLUGIN_ONCE_A_SECOND:
    {
      Plugin_222_M->clear();
      Plugin_222_M->home();
      Plugin_222_M->print(hour());
      Plugin_222_M->print(":");
      Plugin_222_M->print(minute());
      Plugin_222_M->print(":");
      Plugin_222_M->print(second());

      success = true;
    break;
    }

    case PLUGIN_TEN_PER_SECOND:
    {
      //code to be executed 10 times per second. Tasks which require fast response can be added here
      //be careful on what is added here. Heavy processing will result in slowing the module down!

      success = true;
      break;
    }
  }   // switch
  return success;

}     //function

//implement plugin specific procedures and functions here
/*
void p222_do_sth_useful()
{
  //code
}
*/
#endif PLUGIN_BUILD_DEVELOPMENT
//#endif PLUGIN_BUILD_TESTING
#endif // USES_P222
