#ifndef progmemhelper_h
#define progmemhelper_h

//const char pgm_garis_batas[] PROGMEM = " | ";

//const char pgm_table_header[] PROGMEM = "[[\"Time\",\"Min\",\"Avg\",\"Max\"]";
//const char pgm_bar_spasi[] PROGMEM = " Bar\",\"";

//const char pgm_error_1_monitored[] PROGMEM = "Error 1 Monitored";
//const char pgm_error_1_NOT_monitored[] PROGMEM = "Error 1 NOT Monitored";

const char pgm_MANUAL[] PROGMEM = "MANUAL";

const char pgm_NEAR_ZERO[] PROGMEM = "NEAR_ZERO";
const char pgm_NORMAL[] PROGMEM = "NORMAL";
const char pgm_BELOW[] PROGMEM = "BELOW";
const char pgm_ABOVE[] PROGMEM = "ABOVE";
const char pgm_UNDER[] PROGMEM = "UNDER";
const char pgm_OVER[] PROGMEM = "OVER";
const char pgm_vof[] PROGMEM = "vof";
const char pgm_PUMP_ON[] PROGMEM = "PUMP_ON";
const char pgm_PUMP_OFF[] PROGMEM = "PUMP_OFF";

const char pgm_error_1[] PROGMEM = "ERROR-1";
const char pgm_error_2[] PROGMEM = "ERROR-2";
const char pgm_error_3[] PROGMEM = "ERROR-3";
const char pgm_error_4[] PROGMEM = "ERROR-4";
const char pgm_error_5[] PROGMEM = "ERROR-5";

const char pgm_TIMENOTSET[] PROGMEM = "TIME NOT SET";
const char pgm_TIMENEEDSYNC[] PROGMEM = "TIME NEED SYNC";



const char ZERO [] PROGMEM = "0";
const char FALSE [] PROGMEM = "false";
const char ONE [] PROGMEM = "1";
const char TRUE [] PROGMEM = "true";
const char CONNECTED [] PROGMEM = "CONNECTED";
const char pgm_DISCONNECTED [] PROGMEM = "DISCONNECTED";

const char* const PAYLOAD_TABLE[] PROGMEM =
{
  ZERO,       //0
  FALSE,      //1
  ONE,        //2
  TRUE,       //3
  CONNECTED   //4
};

const char STS_mqttCONNECTED [] PROGMEM             = "sts/boosterpump/mqttstatus";
const char STS_statepump [] PROGMEM                 = "sts/boosterpump/statepump";
const char STS_stateError1 [] PROGMEM               = "sts/boosterpump/stateError1";
const char STS_stateError2 [] PROGMEM               = "sts/boosterpump/stateError2";
const char STS_stateError3 [] PROGMEM               = "sts/boosterpump/stateError3";
const char STS_stateError4 [] PROGMEM               = "sts/boosterpump/stateError4";
const char STS_stateError5 [] PROGMEM               = "sts/boosterpump/stateError5";
const char STS_Irms [] PROGMEM                      = "sts/boosterpump/Irms";
const char STS_pressure [] PROGMEM                  = "sts/boosterpump/pressure";
const char CB_MQTTCONNECTED [] PROGMEM              = "boosterpump/mqttstatus";

const char* const PUBLISH_TOPIC_TABLE[] PROGMEM =
{
  STS_mqttCONNECTED,      //0
  STS_statepump,          //1
  STS_stateError1,        //2
  STS_stateError2,        //3
  STS_stateError3,        //4
  STS_stateError4,        //5
  STS_stateError5,        //6
  STS_Irms,               //7
  STS_pressure,           //8
  CB_MQTTCONNECTED,       //9
};

const char CMD_CURRENT_SENSOR_INSTALLED [] PROGMEM  = "cmd/boosterpump/CURRENT_SENSOR_INSTALLED";
const char CMD_PRESSURE_NEAR_ZERO [] PROGMEM  = "cmd/boosterpump/pressure_NEAR_ZERO";
const char CMD_PRESSURE_LOW [] PROGMEM        = "cmd/boosterpump/pressure_LOW";
const char CMD_PRESSURE_HIGH [] PROGMEM       = "cmd/boosterpump/pressure_HIGH";
const char CMD_CURRENT_LOW [] PROGMEM       = "cmd/boosterpump/current_LOW";
const char CMD_CURRENT_HIGH [] PROGMEM       = "cmd/boosterpump/current_HIGH";


const char CB_SUBSCRIBEMQTTTOPIC [] PROGMEM         = "cmd/boosterpump/#";

const char* const SUBSCRIBE_TOPIC_TABLE[] PROGMEM =
{
  CB_SUBSCRIBEMQTTTOPIC,  //0
};

#endif



