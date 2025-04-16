#include <SoftwareSerial.h>
#include <Arduino.h>

#include <LoRa_E220.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <Adafruit_BMP085.h>



#define DESTINATION_ADDH 0
#define DESTINATION_ADDL 2
#define CHANNEL 23


struct Data
{
  float temperature;
// x,y,z ekseni ivme
  float Accelx;
  float Accely;
  float Accelz;
  //x,y,z ekseni gyro
  float Gyrox;  
  float Gyroy;
  float Gyroz;
 // yükseklik ve basınç
  float altitude;
  float pressure;

};





// Arduino RX <-- e220 TX, Arduino TX --> e220 RX
LoRa_E220 e220ttl(&Serial2, 15, 23, 18); // RX, TX, AUX, M0, M1
Adafruit_MPU6050 mpu;
//ikinci i2c hattı tanımlanır
TwoWire I2Ciki = TwoWire(1); // I2C1
Adafruit_BMP085 bmp;

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);




void setup()
{
  Serial.begin(9600);
  I2Ciki.begin(25, 26, 100000); // SDA=25, SCL=26

  e220ttl.begin();
  mpu.begin();

 // Sensöre bağlanmaya çalısıyoruz,bağlanamazsak asağıdaki yazıyı yazdırıyoruz
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //MPU sensor ayarlaroı
  Serial.println("MPU6050 Found!");
  delay(100);
//BMP sensor başladımı kontrol edilir
  

if (bmp.begin(BMP085_ULTRAHIGHRES, &I2Ciki)) {
  Serial.println("BMP180 init success");
} else {
  Serial.println("BMP180 init fail");
  while(1);
}

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);




  //  ----------------------- FIXED SENDER -----------------------


  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);

  //  ----------------------- FIXED SENDER -----------------------
  configuration.ADDH = 0;
  configuration.ADDL = 1;

  configuration.CHAN = CHANNEL;

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_200_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;
  //
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);
  c.close();
}
void loop()
{

   //senserden veri okuma islemi için kütüphane komutu kullanılır, açı =a, hız =g, sıcaklık =temp değiskenlerine atanır
   sensors_event_t a, g, temp; 
   //Değiskenleri adreslerine atama islemi yapılır
   mpu.getEvent(&a, &g, &temp);


  // Paketimizi oluşturuyoruz
  Data data;
  data.temperature = temp.temperature;
  // X ekseni ivme pakete yazılır.
  data.Accelx = a.acceleration.x;
  // Y ekseni ivme pakete yazılır.
  data.Accely = a.acceleration.y;
  // Z ekseni ivme pakete yazılır.
  data.Accelz = a.acceleration.z;
  //x ekseni gyro pakete yazılır.
  data.Gyrox = g.gyro.x;
  // Y ekseni gyro pakete yazılır.
  data.Gyroy = g.gyro.y;
  // Z ekseni gyro pakete yazılır.
  data.Gyroz = g.gyro.z;
  // Basınç pakete yazılır.
  data.pressure = bmp.readPressure();
// Yükseklik pakete yazılır.
  data.altitude = bmp.readAltitude();

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(data.pressure);
  Serial.println(" Pa");

  Serial.print("Real altitude = ");
  Serial.print(data.altitude);
  Serial.println(" meters");

  Serial.print("Acceleration X: ");
  Serial.print( data.Accelx);
  Serial.print(", Y: ");
  Serial.print(data.Accely);
  Serial.print(", Z: ");
  Serial.print(data.Accely);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(data.Gyrox);
  Serial.print(", Y: ");
  Serial.print(data.Gyroy);
  Serial.print(", Z: ");
  Serial.print(data.Gyroz);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  Serial.println(" \n");

  Serial.println("");


  // String message = "Sicaklik: " + String(temperature) + "°C, Nem: " + String(humidity) + "%";

  // İlk mesaj gönderilir.
  ResponseStatus rs = e220ttl.sendFixedMessage(DESTINATION_ADDH, DESTINATION_ADDL, CHANNEL, &data, sizeof(Data));
  // Check If there is some problem of succesfully send
  Serial.println("Veri gönderme durumu: " + rs.getResponseDescription());
  delay(500);
}

void printParameters(struct Configuration configuration)
{
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(configuration.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));
  Serial.print(configuration.OPTION.subPacketSetting, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));
  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));
  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));
  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));
  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));
  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

  Serial.println("----------------------------------------");
}
void printModuleInformation(struct ModuleInformation moduleInformation)
{
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD: "));
  Serial.print(moduleInformation.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(moduleInformation.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(moduleInformation.LENGHT, DEC);

  Serial.print(F("Model no.: "));
  Serial.println(moduleInformation.model, HEX);
  Serial.print(F("Version  : "));
  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));
  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}