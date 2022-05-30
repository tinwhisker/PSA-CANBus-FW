#include <ESP32CAN.h>
#include <CAN_config.h>
#include <JunsunPSACANRemote.h>
#include <BluetoothSerial.h>
#include "ansi.h"

#define _DICT_CRC_ 16
#include <Dictionary.h>

Dictionary &canCapt = *(new Dictionary());


// MCU >> HU
#define CH_CMD_BACKLIGHT_INFO 0x01;
#define CH_CMD_STEERING_WHEEL_KEY 0x02
#define CH_CMD_AIR_CONDITIONING_INFO 0x21
#define CH_CMD_STEERING_WHEEL_ANGLE 0x29
#define CH_CMD_FULL_RADAR_INFO 0x30
#define CH_CMD_REVERSE_RADAR_INFO 0x32
#define CH_CMD_COMPUTE_INFO_PAGE0 0x33
#define CH_CMD_COMPUTE_INFO_PAGE1 0x34
#define CH_CMD_COMPUTE_INFO_PAGE2 0x35
#define CH_CMD_EXTERN_TEMP 0x36
#define CH_CMD_ALARM_RECORD_INFO 0x37
#define CH_CMD_VEHICLE_STATUS 0x38
#define CH_CMD_VEHICLE_FUNC_INFO 0x39
#define CH_CMD_VEHICLE_DIAGNOSTIC_INFO 0x3A
#define CH_CMD_RECORD_SPEED_VALUE 0x3B
#define CH_CMD_SPEED_INFO 0x3D
#define CH_CMD_SPEED_ALARM_DIALOG 0x3F
#define CH_CMD_PROTOCOL_VERSION_INFO 0x7F

// HU >> MCU
#define HC_CMD_AC_CONTROL 0x8A
#define HC_CMD_VEHICLE_PARAM_SETTING 0x80
#define HC_CMD_COMPUTE_PARAM_SETTING 0x82
#define HC_CMD_REQ_VEHICLE_ALARM_INFO 0x85
#define HC_CMD_REQ_FUNCTION_STATUS_INFO 0x86
#define HC_CMD_VEHICLE_DISGNOSTIC_INFO 0x87
#define HC_CMD_SET_RECORD_SPEED 0x88
#define HC_CMD_SET_SPEED_INFO 0x89
#define HC_CMD_AC_CONTROL 0x8A
#define HC_CMD_SCREEN_DISPLAY 0x8C
#define HC_CMD_REQ_CONTROL_INFO 0x8F
#define HC_CMD_SET_SPEEDOMETER 0x99
#define HC_CMD_SET_TIME 0xA6

#define SWC_KEY_NONE 0x00
#define SWC_KEY_MENU 0x02
#define SWC_KEY_MENU_UP 0x03
#define SWC_KEY_MENU_DOWN 0x04
#define SWC_KEY_OK 0x07 //OK
#define SWC_KEY_ESC 0x08 //ESC
#define SWC_KEY_MODE 0x10 //Mode
#define SWC_KEY_SRC 0x11 //Source
#define SWC_KEY_SEEK_DOWN 0x12 //Seek+
#define SWC_KEY_SEEK_UP 0x13 //Seek-
#define SWC_KEY_VOLUME_UP 0x14 //Volume+
#define SWC_KEY_VOLUME_DOWN 0x15 //Volume-
#define SWC_KEY_MUTE 0x16 //Mute
#define SWC_KEY_MEMO_UP 0x17 //上一曲
#define SWC_KEY_MEMO_DOWN 0x18 //下一曲
#define SWC_KEY_PAGE_SW 0x20 //电脑信息页切换
#define SWC_KEY_MENU4 0x21 //菜单
#define SWC_KEY_MEMO 0x22 //记忆速度界面
#define SWC_KEY_BT 0x23 //蓝牙按键
#define SWC_KEY_PUSH_TO_TALK 0x29 //
#define SWC_KEY_VEHICLE_SETTING 0x2A //车身设置
#define SWC_KEY_VEHICLE_NAVI 0x2B 
#define SWC_KEY_MUSIC 0x2C 
#define SWC_KEY_BLUETOOTH 0x2D 
#define SWC_KEY_APPS 0x2E 
#define SWC_KEY_AIR_CONDTION_CONTROL 0x2F 
#define SWC_KEY_PHONE_ACCEPT 0x30 //电话接听	
#define SWC_KEY_PHONE_HANGUP 0x31 //电话挂断
#define SWC_KEY_NAVI 0x32 //Navi
#define SWC_KEY_RADIO 0x33 //收音
#define SWC_KEY_SETUP 0x34 //设置
#define SWC_KEY_ADDR 0x35 //Addr
#define SWC_KEY_MEDIA 0x36
#define SWC_KEY_TRAF 0x37
#define SWC_KEY_UP 0x38
#define SWC_KEY_DOWN 0x39
#define SWC_KEY_LEFT  0x40
#define SWC_KEY_RIGHT 0x41
#define SWC_KEY_SCROLL_UP 0x42
#define SWC_KEY_SCROLL_DOWN 0x43
#define SWC_KEY_NUM1 0x44
#define SWC_KEY_NUM2 0x45
#define SWC_KEY_NUM3 0x46
#define SWC_KEY_NUM4 0x47
#define SWC_KEY_NUM5 0x48
#define SWC_KEY_NUM6 0x49
#define SWC_KEY_SRC2 0x4A
#define SWC_KEY_BAND 0x50
#define SWC_KEY_LIST 0x51
#define SWC_KEY_SOUND 0x52
#define SWC_KEY_TA 0x53
#define SWC_KEY_DARK 0x54
#define SWC_KEY_EJECT 0x55
#define SWC_KEY_RIGHT2 0x56
#define SWC_KEY_LEFT2 0x57
#define SWC_KEY_UP2	0x58
#define SWC_KEY_DOWN2 0x59
#define SWC_KEY_MENU2 0x5A
#define SWC_KEY_MENU3 0x5B
#define SWC_KEY_OK2	0x5C
#define SWC_KEY_MUTE2 0x5D
#define SWC_KEY_BACK 0x5E
#define SWC_KEY_CHECK 0x60
#define SWC_KEY_POWER 0x80

#define AC_LOWEST_TEMP_C 0.5f
#define AC_HIGHEST_TEMP_C 127.0f

#define AC_LOWEST_TEMP_F 1.0f
#define AC_HIGHEST_TEMP_F 254.0f

#define EXTERN_LOWEST_TEMP  -50.0f
#define EXTERN_HIGHEST_TEMP 77.5f


#define IO_LED           14 //Running LED
#define IO_HUenable      25 //Car ignition signal
#define IO_HUrear        27 //Reverse gear signal
#define IO_HUill         26 //Illumimation signal

#define CAN_TX           GPIO_NUM_5
#define CAN_RX           GPIO_NUM_4
#define CAN_STBY         13

#define SYS_BUTTON       0

#define SERIAL_SPEED     19200
JunsunPSACANRemote*      remote;

CAN_device_t CAN_cfg;                // CAN Config
const int rx_queue_size = 10;       // Receive Queue size
CAN_frame_t canMsgRcv;
CAN_frame_t canMsgSend;

bool canDump,debugOut;

struct Canbus_previous_val {
    byte rpm[2]; // odometer info
    byte ext_temp,ver;
    uint16_t disc_btn;
    bool ign,ill,rear,brake; // I/O status
};

struct Car_data {
    
    uint16_t I_speed, I_rpm; // odometer info
    uint16_t T_fuelC,T_fuelR,TA_Mil,TB_Mil; // trip
    byte TA_Spd,TB_Spd,TA_Fuelc,TB_Fuelc; // trip
    bool ign,ill,rear,brake,I_Eng; // I/O status
    byte AC_L,AC_R,AC_FAN,exttemp,Coolant,INF_MSG; // A/C values
    
    byte Doors,SysStateA,LightState,SysStateB,SysStateC,SysStateD;
    bool AnyDoor;
    bool UPDATE_STATE;
    
    bool AC_DOWN,AC_UP,AC_FRONT,AC_Recycle,AC_WINDSH,AC_AUTO,AC_COMP,AC_OFF,AC_Mono,AC_AFAN,AC_SFAN; 
    bool H_UPD_AC,H_UPD_IO,H_UPD_DOOR,H_UPD_INF,H_UPD_Trip; //update flag
    bool C_UPD_AC; //update flag
};

BluetoothSerial SerialBT;

ANSI ansi(&SerialBT);

int t;
double d;

Car_data Car;

uint16_t ScrollVal,ScrollValPrior;

void CAN_setup();
void CAN_loop();
void AC_decode_2004();
int Temperature_decode_CAN2004(byte tin);
void HU_button(byte button);
void Disc(int count);
uint8_t Radar_Decode(uint8_t val, bool isCenter);
void InfoMsg(uint8_t msg);


void setup() {
  pinMode(IO_LED,       OUTPUT);
  pinMode(IO_HUenable,  OUTPUT);
  pinMode(IO_HUrear,    OUTPUT);
  pinMode(IO_HUill,     OUTPUT);
  pinMode(CAN_STBY,     OUTPUT);

  pinMode(SYS_BUTTON,   INPUT_PULLUP);
  
  digitalWrite(IO_LED,      HIGH); //Inverted output  
  digitalWrite(IO_HUenable, LOW);
  digitalWrite(IO_HUrear,   LOW);
  digitalWrite(IO_HUill,    LOW);
  digitalWrite(CAN_STBY,    HIGH); //High == Standby mode

  Serial.begin(SERIAL_SPEED);  
  remote = new JunsunPSACANRemote(Serial);

  CAN_setup();

  SerialBT.begin("PSA_CAN");
  digitalWrite(IO_HUenable, HIGH);
}


void CAN_setup() {
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = CAN_TX;
  CAN_cfg.rx_pin_id = CAN_RX;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  digitalWrite(CAN_STBY, LOW);
  ESP32Can.CANInit();
}

void update_IO() {
  digitalWrite(IO_LED, !Car.ign); // Diagnostic LED 
  digitalWrite(IO_HUill, Car.ill);// put !ill if needs to reversed
  Car.H_UPD_IO=false; //updated

  //digitalWrite(IO_HUenable, Car.ign); // put !ign if needs to reversed
  bool curHUState = digitalRead(IO_HUenable);
  
  if (!Car.ign && curHUState) {
    //Door open, ign off, HU on.
    SerialBT.end();
    digitalWrite(IO_HUenable, LOW);
  }
  
  if (Car.ign){// && !curHUState) {
    SerialBT.begin("PSA_CAN"); //Bluetooth device name
    digitalWrite(IO_HUenable, HIGH);
 }
}



void update_STATUS() {
  uint8_t sysState[] = { CH_CMD_VEHICLE_STATUS, Car.Doors, Car.SysStateA, Car.LightState, Car.SysStateB, Car.SysStateC, Car.SysStateD };
  remote->SendData(sysState, JS_ARRAY_SIZE(sysState));

  Car.UPDATE_STATE = false;
}

void update_TRIP() {

  //remote->SendButtonCode(TripComputer);
  //update_STATUS();
  Car.H_UPD_Trip = false;
}

void update_AC() {
  byte acState;
  byte blowerState;
  byte fanState;

  if (Car.AC_Recycle) bitSet(acState, 5);
  bitSet(acState, 2); //Dual

  if (Car.AC_DOWN)   bitSet(blowerState, 5);
  if (Car.AC_UP)     bitSet(blowerState, 7);
  if (Car.AC_FRONT)  bitSet(blowerState, 6);

  if (Car.AC_WINDSH) bitSet(fanState, 7);

  uint8_t acData[] = { CH_CMD_AIR_CONDITIONING_INFO, acState, blowerState, Car.AC_L, Car.AC_R, fanState };
  remote->SendData(acData, JS_ARRAY_SIZE(acData));
}

void update_INFO() {
  uint8_t infoDat[] = { CH_CMD_VEHICLE_FUNC_INFO, Car.INF_MSG };
  remote->SendData(infoDat, JS_ARRAY_SIZE(infoDat));

  Car.H_UPD_INF = false;  
}

void setCarButtonState(uint8_t message[]) {
  bool buttonState = (message[2] == 0x01) ? true : false;
  switch (message[1]) {
    //Parking
    case 0x01: //Parking aux switch

      break;
    case 0x0c: //Blindzone detection

      break;
    case 0x0d: //Engine start/stop

      break;
    case 0x0e: //Welcome function

      break;
    case 0x0f: //Door opening option

      break;
    case 0x06: //Reversing radar stop

      break;
    //Driver assist systems
    case 0x10: //Tire pressure calibration

      break;
    case 0x02: //Rear window wipers

      break;
    case 0x09: //Sound
      //uint8_t selectedSound = message[2];

      break;
    case 0x0a: //Fuel consump unit

      break;
    case 0x0b: //Language

      break;
    case 0x11: //Driving assistance

      break;
    //Light settings
    case 0x03: //Daytime lighting switch

      break;
    case 0x04: //Headlight delay off time

      break;
    case 0x05: //Ambience lighting brightness

      break;
    case 0x07: //"Walk-me-home" lighting time

      break;
    case 0x08: //Welcome lighting time

      break;
  }
}

void headunitRequestHandler(uint8_t message[], uint8_t messageLength) {

  /*SerialBT.print("HU:");
  for (int i = 0; i < messageLength; i++) {
    SerialBT.print(" ");
    SerialBT.print(String(message[i], HEX));
  }
  SerialBT.println("");*/
  
  switch(message[0]) {
    case HC_CMD_VEHICLE_PARAM_SETTING:
      

      break;
    case HC_CMD_COMPUTE_PARAM_SETTING:
      // 3 bytes
      // 0x40 0x00 0x00 - clear page 1
      // 0x20 0x00 0x00 - clear page 2
      // 0x80 0x00 0x?? - Set milage??
      // 0x00 
      break;
    case HC_CMD_REQ_VEHICLE_ALARM_INFO:

      break;
    case HC_CMD_REQ_FUNCTION_STATUS_INFO:

      break;
    case HC_CMD_VEHICLE_DISGNOSTIC_INFO:
      // 1 byte: ex: 0xFC
      break;
    case HC_CMD_SET_RECORD_SPEED:

      break;
    case HC_CMD_SET_SPEED_INFO:

      break;
    case HC_CMD_AC_CONTROL:
      // 2 bytes, ex: 0x09 0x02
      // 0x0C 0x01 = 'Off' Pressed
      // 0x0C 0x00 = 'Off' Released
      // 0x0C 0x01 = 'Recirc' Pressed
      // 0x0C 0x00 = 'Recirc' Released
      // 0x04 0x01 = Left side up
      // 0x04 0x02 = Left side down
      // 0x05 0x01 = Right side up
      // 0x05 0x02 = Right side down
      // 0x0A 0x01 = Fan Increase
      // 0x0A 0x02 = Fan Decrease
      // 0x09 0x00 = Fan Soft
      // 0x09 0x01 = Fan Medium
      // 0x09 0x02 = Fan Full      
      break;
    case HC_CMD_SCREEN_DISPLAY:

      break;
    case HC_CMD_REQ_CONTROL_INFO:
      switch(message[1]) {
        case CH_CMD_COMPUTE_INFO_PAGE0:
          remote->SendTripDataCar(Car.T_fuelR, Car.TA_Fuelc, Car.T_fuelC);
          break;
        case CH_CMD_COMPUTE_INFO_PAGE1:
          //remote->SendTripData1(2345, 11.7, 159);
          break;          
        case CH_CMD_COMPUTE_INFO_PAGE2:
          //remote->SendTripData2(2345, 11.7, 159);
          break;
      }
      // 1 byte, ex: 0x21
      // 0x21 = Aircon info request
      // 0x38 = "parking and shunting" / Lighting info request
      break;
    case HC_CMD_SET_SPEEDOMETER:

      break;
    case HC_CMD_SET_TIME:
      // 5 bytes. Ignore? Would BSI benefit from time?
      // 0x15 0x0C  0x0C 0x16 0x07
      // Year Month Day  Hour Minute
      break;
    default:

      break;
  } 
}

void loop() {
  CAN_loop();

  bool sysBtn = digitalRead(SYS_BUTTON);
  if (!sysBtn) canDump = true;

  if (Car.H_UPD_IO) update_IO();
  if (Car.UPDATE_STATE) update_STATUS();
  if (Car.H_UPD_Trip) update_TRIP();
  if (Car.H_UPD_INF) update_INFO();

  //From HU
  if (Serial.available()) {
    uint8_t inChar = (uint8_t)Serial.read();
    if (inChar == 0xFD) {
      uint8_t packetLen = (uint8_t)Serial.read();
      uint8_t message[packetLen-2] = {0};
      
      for (int i = 0; i < packetLen-2; i++) message[i] = (uint8_t)Serial.read();

      headunitRequestHandler(message, JS_ARRAY_SIZE(message));
    }
  }    

  //From Debug
  if (SerialBT.available()) {
    uint8_t inChar = (uint8_t)SerialBT.read();

    if (inChar == 0xFD) {
      uint8_t packetLen = (uint8_t)SerialBT.read();
      uint8_t message[packetLen];
      
      for (int i = 0; i < packetLen; i++) message[i] = (uint8_t)SerialBT.read();

      if (debugOut) {
        SerialBT.print("DEBUG:");
        for (int i = 0; i < packetLen; i++) SerialBT.print(" " + String(message[i], HEX));
        SerialBT.println("");
      }

      remote->SendData(message, JS_ARRAY_SIZE(message));
      return;
    }



    if (inChar == 'z') {
      canDump = !canDump;
      ansi.clearScreen();
      ansi.gotoXY(2, 10);
      ansi.print("CANBUS Dumping:    ");
      ansi.gotoXY(2, 26);
      ansi.print((canDump)?"ON":"OFF");
      
    }

    if (inChar == 'x') {
      debugOut = !debugOut;
    }

    if (inChar == 't') {
      // DISPLAY TEMPERATURE (dummy)
      ansi.gotoXY(6, 10);
      ansi.print("TEMP:       ");
      ansi.gotoXY(6, 16);
      t = random(100);
      if (t > 70) ansi.foreground(ansi.red);
      ansi.print(t);
      ansi.foreground(ansi.white);
    
      // DISPLAY HUMIDITY (dummy)
      ansi.gotoXY(7, 10);
      ansi.print(" HUM:       ");
      ansi.gotoXY(7, 16);
      t = random(100);
      if (t > 50) ansi.foreground(ansi.yellow);
      ansi.print(t);
      ansi.foreground(ansi.white);
    
      // DISPLAY UV (dummy)
      ansi.gotoXY(8, 10);
      ansi.print("  UV:       ");
      ansi.gotoXY(8, 16);
      d = random(10000) * 0.01;
      if (d > 30) ansi.foreground(ansi.green);
      if (d > 50) ansi.foreground(ansi.yellow);
      if (d > 70) ansi.foreground(ansi.red);
      ansi.print(d, 2);
      ansi.foreground(ansi.white);
    
      // DISPLAY bargraph (dummy)
      ansi.gotoXY(10, 10);
      ansi.print(" BAR:");
      ansi.gotoXY(10, 16);
      int x = random(10);
      for (int i = 0; i < 10; i++) ansi.print(i <= x ? ">" : " ");
    
      // DISPLAY password (dummy)
      ansi.gotoXY(12, 10);
      ansi.print("PASS:");
      char buffer[20];
      for (int i = 0; i < 16; i++) 
      {
        int x = random(62);
        if (x < 26) buffer[i] = 'A' + random(26);
        if (26 <= x && x < 52) buffer[i] = 'a' + random(26);
        if (52 <= x) buffer[i] = '0' + random(10);
      }
      buffer[16] = 0;
      ansi.gotoXY(12, 16);
      ansi.print(buffer);
    
      // DISPLAY TIME (dummy)
      ansi.gotoXY(2, 10);
      ansi.print("TIME:         ");
      ansi.gotoXY(2, 16);
      ansi.print(millis()/1000);
    
      delay(1000);
    }

  }
}

void CAN_loop() {    
  if (xQueueReceive(CAN_cfg.rx_queue, &canMsgRcv, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    switch (canMsgRcv.MsgID) {
        case 0x0B6: //0x0B6 rpm speed Fuel consumtion info (50ms)
          if (canMsgRcv.FIR.B.DLC == 8) {
            if (Car.I_rpm != (((canMsgRcv.data.u8[0] << 5) + (canMsgRcv.data.u8[1]) >> 3))) {
              Car.I_rpm = (((canMsgRcv.data.u8[0] << 5) + (canMsgRcv.data.u8[1]) >> 3)); //canMsgRcv.data.u8[0] + canMsgRcv.data.u8[1] * 256; // to be verified
              Car.I_Eng = ( Car.I_rpm > 0 );
              Car.I_speed = (((canMsgRcv.data.u8[2] << 5) + (canMsgRcv.data.u8[3]) >> 3)); //canMsgRcv.data.u8[2] + canMsgRcv.data.u8[3] * 256; // to be verified divide by 10 for speed 
              //Car.H_UPD_IO = true;
            }
          }
          break;
        case 0x0E1: //0x0E1 parktronic
          if (Car.rear && Car.I_Eng) {//bitRead(canMsgRcv.data.u8[5], 1)) { //Parktronic window showing
            uint8_t ptRL = canMsgRcv.data.u8[3] >> 5;
            uint8_t ptRC = (canMsgRcv.data.u8[3] >> 2) & 7;
            uint8_t ptRR = canMsgRcv.data.u8[4] >> 5;
            uint8_t ptFL = (canMsgRcv.data.u8[4] >> 2) & 7;
            uint8_t ptFC = canMsgRcv.data.u8[5] >> 5;
            uint8_t ptFR = (canMsgRcv.data.u8[5] >> 2) & 7;

            uint8_t ptronic[] = { CH_CMD_FULL_RADAR_INFO, ptFL, ptFC, ptFR, ptRL, ptRC, ptRR };
            remote->SendData(ptronic, JS_ARRAY_SIZE(ptronic));
          }
          break;
        case 0x0F6: //0x0F6 bsi TEMP COOLANT ?EXT? reverse gear ignition odometer

          if (Car.ign != bitRead(canMsgRcv.data.u8[0], 3)) {  //Ignition on
            Car.ign = bitRead(canMsgRcv.data.u8[0], 3);
            Car.H_UPD_IO = true;
          }

          if (Car.rear != bitRead(canMsgRcv.data.u8[7], 7)) { //Reverse enabled
            Car.rear = bitRead(canMsgRcv.data.u8[7], 7);
            Car.H_UPD_IO = true;
          }

          if (canMsgRcv.data.u8[6] != Car.exttemp) { 
              Car.exttemp = canMsgRcv.data.u8[6]; // divide by 2 and -39.5 for °c
              int8_t cTemp = (Car.exttemp / 2) - 39.5;
              remote->SendTemperature(cTemp);
              SerialBT.println("Temperature: " + String(cTemp, DEC) + "°c");
          }

          Car.Coolant = canMsgRcv.data.u8[1]; // add 39 for °c
          break;
        case 0x128: //0x128 Dashboard lights
            if (Car.ill != ((canMsgRcv.data.u8[4] >> 5) & 7)) { //Mask for any illumination
              Car.ill = !Car.ill;
              Car.H_UPD_IO = true;
            }
            //SerialBT.println("128 Door State:" + String(canMsgRcv.data.u8[1], HEX));
            if (Car.AnyDoor != bitRead(canMsgRcv.data.u8[1], 4)) { //If a door opens...
              Car.AnyDoor = !Car.AnyDoor;
              Car.H_UPD_IO = true;
            }
          break;
        /* wheel position to the right ID 162 A2 XX 00 00 00 00 where XX varies 1A, 1B, 1C ..... FE, 00*/
        case 0x1A1: //0x1A1 Informational message
          if (Car.INF_MSG != canMsgRcv.data.u8[1] && canMsgRcv.data.u8[1] != 0xFF) {
            Car.INF_MSG = canMsgRcv.data.u8[1]; //send as is to hu
            InfoMsg(Car.INF_MSG);   //Bluetooth send
            //if (canMsgRcv.data.u8[2] == 1) Car.H_UPD_INF = 1; // ambigous     //'Show' flag
            if (canMsgRcv.data.u8[0] == 0x80) Car.H_UPD_INF = 1;  // ambigous //'Display' flag
          }
          break;
        case 0x1D0: //0x1D0 Climate control information
          if (canMsgRcv.FIR.B.DLC == 7 && Car.I_Eng) { // No fan activated if the engine is not ON on old models
            //if (canMsgRcv.data.u8[0]&&64)   //REAR_WINDSHIELD_HEAT
            AC_decode_2004();
          }
          break;
        case 0x1ED: //0x1ED Display conditioning commands
          Car.AC_Mono = (canMsgRcv.data.u8[0] == 0x00);
          Car.AC_OFF = (canMsgRcv.data.u8[0] == 0x18);
          break;
        case 0x21F: //0x21F Radio remote control under the steering wheel
          if bitRead(canMsgRcv.data.u8[0], 7) remote->SendButtonCode(NextTrack); //forward tuning
          if bitRead(canMsgRcv.data.u8[0], 6) remote->SendButtonCode(PreviousTrack);//backward tuning
          if bitRead(canMsgRcv.data.u8[0], 3) remote->SendButtonCode(VolumeUp);//volume UP
          if bitRead(canMsgRcv.data.u8[0], 2) remote->SendButtonCode(VolumeDown);//volume DOWN
          if bitRead(canMsgRcv.data.u8[0], 1) remote->SendButtonCode(Source);//Source
                  
          ScrollValPrior = ScrollVal;
          ScrollVal = canMsgRcv.data.u8[1]; // need to be tested
          if (ScrollValPrior != ScrollVal) SerialBT.println("ScrollValue: " + String(ScrollVal, HEX));
          if ((ScrollVal < 8 && ScrollValPrior > 200) || (ScrollVal > ScrollValPrior)) { //If Scr is greater than orig, or overflowed.
            remote->SendButtonCode(NextAlbum); //Next preset
          } else if ((ScrollVal > 200 && ScrollValPrior < 8) || (ScrollVal < ScrollValPrior)) { //If Scr is lesser than orig, or underflowed.
            remote->SendButtonCode(PreviousAlbum); //Previous preset
          }

          break;
        /*case 0x220: //0x220 Door status
          SerialBT.println("220 Door State:" + String(canMsgRcv.data.u8[0], HEX));
          if (canMsgRcv.data.u8[0] != Car.Doors) {
            Car.Doors = canMsgRcv.data.u8[0];

            SerialBT.println("FL Door: " + bitRead(Car.Doors, 7)?"Open":"Closed");
            SerialBT.println("FR Door: " + bitRead(Car.Doors, 6)?"Open":"Closed");
            SerialBT.println("RL Door: " + bitRead(Car.Doors, 5)?"Open":"Closed");
            SerialBT.println("RR Door: " + bitRead(Car.Doors, 4)?"Open":"Closed");
            SerialBT.println("Boot Door: " + bitRead(Car.Doors, 3)?"Open":"Closed");

            Car.UPDATE_STATE=true;
          }  
          break;*/
        case 0x221: //0x221 Trip computer info instant
          if (canMsgRcv.FIR.B.DLC == 7) {
             Car.T_fuelC=canMsgRcv.data.u8[1]+canMsgRcv.data.u8[2]*256; //divide by 10 for L/100KM
              Car.T_fuelR=canMsgRcv.data.u8[3]+canMsgRcv.data.u8[4]*256;
            if (bitRead(canMsgRcv.data.u8[0], 3) || bitRead(canMsgRcv.data.u8[0], 0)) { //trip or voice command button
              remote->SendButtonCode(TripComputer);
              Car.UPDATE_STATE=true;
            }
          }
          break;
        case 0x261: //0x261 Trip computer info A
          Car.TA_Spd=canMsgRcv.data.u8[0];
          Car.TA_Mil=(canMsgRcv.data.u8[1]+canMsgRcv.data.u8[2]*256)/4;
          Car.TA_Fuelc=canMsgRcv.data.u8[3]; //divide by 10 for L/100KM
          Car.UPDATE_STATE = true;  
          break;
        case 0x2A1: //0x261 Trip computer info B
          Car.TB_Spd=canMsgRcv.data.u8[0];
          Car.TB_Mil=(canMsgRcv.data.u8[1]+canMsgRcv.data.u8[2]*256)/4;
          Car.TB_Fuelc=canMsgRcv.data.u8[3]; //divide by 10 for L/100KM
          Car.UPDATE_STATE = true;
          break;
        default:
          if (canDump) { //ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
            
            if (!canCapt[String(canMsgRcv.MsgID, HEX)])
              canCapt(String(canMsgRcv.MsgID, HEX), String(canCapt.count(), DEC));  //Add the ID into a free position.

              int xPos = canCapt[String(canMsgRcv.MsgID, HEX)].toInt()+4;
              ansi.gotoXY(xPos, 10);
              ansi.print("0x                             ");
              //ansi.print("0xFFF 8 FF FF FF FF FF FF FF FF");
              
              ansi.gotoXY(xPos, 12);
              ansi.print(String(canMsgRcv.MsgID, HEX));
  
              ansi.gotoXY(xPos, 16);
              ansi.print(canMsgRcv.FIR.B.DLC);
  
              for (int x=0; x < canMsgRcv.FIR.B.DLC; x++) {
                ansi.gotoXY(xPos, 18+(x*3));
                ansi.print(String(canMsgRcv.data.u8[x], HEX));
              }
            }
          break;             
        } //End Switch
    } //End ValidQueue

    //????
    /*
    if ( Car.C_UPD_AC) {
        // 1E3	7	1C 30 0D 0D 00 00 05 to be tested could be AC
        
        canMsgSend.FIR.B.FF = CAN_frame_std;
        canMsgSend.MsgID = 0x1E3;
        canMsgSend.FIR.B.DLC = 7;
        canMsgSend.data.u8[0] = 0x1C; //sniffed values
        canMsgSend.data.u8[1] = 0x30; //sniffed values
        canMsgSend.data.u8[2] = Car.AC_L;
        canMsgSend.data.u8[3] = Car.AC_R;
        canMsgSend.data.u8[4] = 0x00; //sniffed values
        canMsgSend.data.u8[5] = 0x00; //sniffed values
        canMsgSend.data.u8[6] = 0x05; //sniffed values
        ESP32Can.CANWriteFrame(&canMsgSend);
        Car.C_UPD_AC=false;
    }
    */
}

void AC_decode_2004() { //see 12D too
  Car.AC_L = Temperature_decode_CAN2004(canMsgRcv.data.u8[5]);
  Car.AC_R = Temperature_decode_CAN2004(canMsgRcv.data.u8[6]); //12d 4 should be AC controller 64 for high
  Car.AC_FAN = canMsgRcv.data.u8[2]; //0f:off 00>07 1 >8  /12d 1 0>64 (64=100 dec) maybe %
  // Position Fan

  Car.AC_DOWN   = false;
  Car.AC_UP     = false;
  Car.AC_FRONT  = false;
  switch ((canMsgRcv.data.u8[3] >> 4) & 7) {
    case 0x1:                                           //Up (when windshield blowing enabled)
      Car.AC_UP     = bitRead(canMsgRcv.data.u8[4], 4);
      break;
    case 0x2:                                           // Down
      Car.AC_DOWN   = true;
      break;
    case 0x3:                                           // Front
      Car.AC_FRONT  = true;
      break;  
    case 0x4:                                           // Up                             
      Car.AC_UP     = true;
      break;
    case 0x5:                                           // Front + Down
      Car.AC_DOWN   = true;
      Car.AC_FRONT  = true;
      break;
    case 0x6:                                           // Up + Down
      Car.AC_DOWN   = true;
      Car.AC_UP     = true;
      break;
  }

  Car.AC_Recycle = bitRead(canMsgRcv.data.u8[4], 5);
  Car.AC_WINDSH = bitRead(canMsgRcv.data.u8[4], 4);

  //Unconverted
  Car.AC_AFAN = !(canMsgRcv.data.u8[0] & 32); //autofan
  Car.AC_AUTO = !(canMsgRcv.data.u8[0] & 2); //auto A/C
  //Car.AC_WINDSH = !(canMsgRcv.data.u8[0] & 1); //windsh
  Car.AC_SFAN = !(canMsgRcv.data.u8[0] & 128); // Fan off
}

int Temperature_decode_CAN2004(byte tin) {
  if (tin ==  0) return 0; //	LO
  if (tin >1 && tin <6) return ((tin + 13) * 10);  // 2>5	15>18°
  if (tin >5 && tin <17) return ((tin + 31) * 5); // 6…16	18.5…23.5° (0.5° step)
  if (tin >16 && tin <21) return ((tin + 7) * 10);  //17…20	24…27°
  return 255; //	LO
}

void HU_button(byte button) {
  uint8_t but[] = { 0x02, button};
  remote->SendData(but, 2);
}

void InfoMsg(uint8_t msg) {
  switch(msg) {
    case 0x00:
      SerialBT.println("Diagnosis ok");
      break;
    case 0x01:
      SerialBT.println("Engine temperature too high");
      break;
    case 0x03:
      SerialBT.println("Coolant circuit level too low");
      break;
    case 0x04:
      SerialBT.println("Check engine oil level");
      break;
    case 0x05:
      SerialBT.println("Engine oil pressure too low");
      break;
    case 0x08:
      SerialBT.println("Braking system faulty");
      break;
    case 0x0A:
      SerialBT.println("Air suspension ok");
      break;
    case 0x0B:
      SerialBT.println("Door, boot, bonnet and fuel tank open");
      break;
    case 0x0D:
      SerialBT.println("Tyre puncture(s) detected");
      break;
    case 0x0F:
      SerialBT.println("Risk of particle filter blocking");
      break;
    case 0x11:
      SerialBT.println("Suspension faulty: max.speed 90 km/h");
      break;
    case 0x12:
      SerialBT.println("Suspension faulty");
      break;
    case 0x13:
      SerialBT.println("Power steering faulty");
      break;
    case 0x14:
      SerialBT.println("10km/h!");
      break;
    case 0x61:
      SerialBT.println("Handbrake on");
      break;
    case 0x62:
      SerialBT.println("Handbrake off");
      break;
    case 0x64:
      SerialBT.println("Handbrake control faulty: auto handbrake activated");
      break;
    case 0x67:
      SerialBT.println("Brake pads worn");
      break;
    case 0x68:
      SerialBT.println("Handbrake faulty");
      break;
    case 0x69:
      SerialBT.println("Mobile deflector faulty");
      break;
    case 0x6A:
      SerialBT.println("ABS braking system faulty");
      break;
    case 0x6B:
      SerialBT.println("ESP / ASR system faulty");
      break;
    case 0x6C:
      SerialBT.println("Suspension faulty");
      break;
    case 0x6D:
      SerialBT.println("Power steering faulty");
      break;
    case 0x6E:
      SerialBT.println("Gearbox faulty");
      break;
    case 0x6F:
      SerialBT.println("Cruise (speed) control system faulty");
      break;
    case 0x73:
      SerialBT.println("Ambient brightness sensor faulty");
      break;
    case 0x74:
      SerialBT.println("Sidelamp bulb(s) faulty");
      break;
    case 0x75:
      SerialBT.println("Automatic headlamp adjustment faulty");
      break;
    case 0x76:
      SerialBT.println("Directional headlamps faulty");
      break;
    case 0x78:
      SerialBT.println("Airbag faulty");
      break;
    case 0x79:
      SerialBT.println("Active bonnet faulty");
      break;
    case 0x7A:
      SerialBT.println("Gearbox faulty");
      break;
    case 0x7B:
      SerialBT.println("Apply foot on brake and lever in position N");
      break;
    case 0x7D:
      SerialBT.println("Presence of water in diesel fuel filter");
      break;
    case 0x7E:
      SerialBT.println("Engine management system faulty");
      break;
    case 0x7F:
      SerialBT.println("Depollution system faulty");
      break;
    case 0x81:
      SerialBT.println("Particle filter additive level too low");
      break;
    case 0x83:
      SerialBT.println("Electronic anti-theft faulty");
      break;
    case 0x86:
      SerialBT.println("Right hand side door faulty");
      break;
    case 0x87:
      SerialBT.println("Left hand side door faulty");
      break;
    case 0x89:
      SerialBT.println("Space measuring system faulty");
      break;
    case 0x8A:
      SerialBT.println("Battery charge or electrical supply faulty");
      break;
    case 0x8D:
      SerialBT.println("Tyre pressure(s) too low");
      break;
    case 0x92:
      SerialBT.println("Warning message without text");
      break;
    case 0x95:
      SerialBT.println("Info message without text");
      break;
    case 0x96:
      SerialBT.println("Info message without text");
      break;
    case 0x97:
      SerialBT.println("Anti-wander system lane-crossing warning device faulty");
      break;
    case 0x9D:
      SerialBT.println("Foglamp bulb(s) faulty");
      break;
    case 0x9E:
      SerialBT.println("Direction indicator(s) faulty");
      break;
    case 0xA0:
      SerialBT.println("Sidelamp bulb(s) faulty");
      break;
    case 0xA1:
      SerialBT.println("Parking lamps active");
      break;
    case 0xCD:
      SerialBT.println("Cruise control not possible: speed too low");
      break;
    case 0xCE:
      SerialBT.println("Control activation not possible: enter the speed");
      break;
    case 0xD1:
      SerialBT.println("Active bonnet deployed");
      break;
    case 0xD2:
      SerialBT.println("Front seat belts not fastened");
      break;
    case 0xD3:
      SerialBT.println("Rear right hand passenger seat belts fastened");
      break;
    case 0xD7:
      SerialBT.println("Place automatic gearbox in position P");
      break;
    case 0xD8:
      SerialBT.println("Risk of ice");
      break;
    case 0xD9:
      SerialBT.println("Handbrake!");
      break;
    case 0xDE:
      SerialBT.println("Door, boot, bonnet and fuel tank open");
      break;
    case 0xDF:
      SerialBT.println("Screen wash fluid level too low");
      break;
    case 0xE0:
      SerialBT.println("Fuel level too low");
      break;
    case 0xE1:
      SerialBT.println("Fuel circuit deactivated");
      break;
    case 0xE3:
      SerialBT.println("Remote control battery flat");
      break;
    case 0xE4:
      SerialBT.println("Check and re-initialise tyre pressure");
      break;
    case 0xE5:
      SerialBT.println("Tyre pressure(s) not monitored");
      break;
    case 0xE7:
      SerialBT.println("High speed, check tyre pressures correct");
      break;
    case 0xE8:
      SerialBT.println("Tyre pressure(s) too low");
      break;
    case 0xEA:
      SerialBT.println("Hands-free starting system faulty");
      break;
    case 0xEB:
      SerialBT.println("Starting phase has failed (consult handbook)");
      break;
    case 0xEC:
      SerialBT.println("Prolonged starting in progress");
      break;
    case 0xED:
      SerialBT.println("Starting impossible: unlock the steering");
      break;
    case 0xEF:
      SerialBT.println("Remote control detected");
      break;
    case 0xF0:
      SerialBT.println("Diagnosis in progress...");
      break;
    case 0xF1:
      SerialBT.println("Diagnosis completed");
      break;
    case 0xF2:
    case 0xF3:
    case 0xF4:
    case 0xF5:
    case 0xF6:
      SerialBT.println("Message without text");
      break;
    case 0xF7:
      SerialBT.println("Rear LH passenger seatbelt unfastened");
      break;
    case 0xF8:
      SerialBT.println("Rear center passenger seatbelt unfastened");
      break;
    case 0xF9:
      SerialBT.println("Rear RH passenger seatbelt unfastened");
        break;
    default:
      SerialBT.println("Unknown Message: 0x" + String(msg, HEX));
      break;
  }
}
