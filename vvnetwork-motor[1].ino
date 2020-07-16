// This #include statement was automatically added by the Particle IDE.
#include <ModbusMaster-Particle.h>


#include <blynk.h>
char auth[] = "Sjo1mNu68Y6V1cWo9VgVs8Io9RkQO7QE"; 
BlynkTimer timer;
bool sendtoblynkenable = 1;

WidgetLED led1(V12);


//SYSTEM_MODE(MANUAL);

SerialLogHandler logHandler(9600,LOG_LEVEL_WARN, {
    {"app", LOG_LEVEL_TRACE},
    {"system", LOG_LEVEL_INFO}
});


// are these the correct pins for Argon board??
#define MAX485_DE      12
#define MAX485_RE_NEG  11

#define MAX485_TX 9
#define MAX485_RX 10

// instantiate ModbusMaster object
ModbusMaster node;

// create software serial port
// SoftwareSerial modbusSerial(MAX485_RX, MAX485_TX); //RX, TX

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  Time.zone(-2.5);
  Blynk.begin(auth);
  timer.setInterval(5000L, sendinfo);
  timer.setInterval(2000L, readTest);
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus RTU Ultrasonic communication runs at 9600 baud
  //modbusSerial.begin(9600);
  //Serial1.begin(9600);
    
  // Modbus slave ID 1
  node.begin(1, Serial1);
  node.setSpeed(9600);
  node.enableDebug();
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  node.idle(idle);
}

typedef union {
  uint16_t dblBytes[2];
  uint32_t bigNum;
} UINT32_ARRAY;


void idle() {
    delay(10); // in case slave only replies after 10ms
    Particle.process(); // avoids letting the connection close if open
}

void loop()
{
  Blynk.run();
  timer.run();
}

void sendinfo()
{
    if(sendtoblynkenable == 1)
    {
        Blynk.virtualWrite(V0, Time.format(Time.local(), "%r - %a %D"));
        //Blynk.virtualWrite(V20, mm);
        //Blynk.virtualWrite(V21, temp);
        //Blynk.virtualWrite(V22, gal); 
        //Blynk.virtualWrite(V4, ??); not used
        //if(gal < fullsetpoint)
        //{
        //    led1.off();
        //}
    }
}

uint8_t test_num = 0;

// Will cycle between running the first and second test on each interval.
void readTest() {
  switch(test_num % 2) {
      case 0:
        readTemps();
        break;
      case 1:
        readInputRegisters();
        break;
      default:
        Log.warn("No test for %0x", test_num);
        break;
  }
  test_num++;
}

void readTemps() {
  uint8_t result;
  uint16_t data;
  // Read the register at address FC(0x03) Register(40420) Address(419) HEX(1A3) (Temperature Compensation)
  Log.info("Reading temperature compensation register...");
  result = node.readHoldingRegisters(419, 1);
  
  if (result == node.ku8MBSuccess) {  
    Log.info("Temperature Compensation Status: %0x", node.getResponseBuffer(0));
  } else {
    Log.warn("Failed to read temperature compensation register!");
    Log.warn("Result: %0x", result);
  }
}

void readInputRegisters() {
  uint8_t result;
  uint16_t data;
  // Read 11 registers starting at FC(0x04) Register(30299) Address(298) HEX(12A)
  Log.info("Reading all input registers...");
  result = node.readInputRegisters(298, 11);

  if (result == node.ku8MBSuccess) {
    Log.info("ModelType: %0x", node.getResponseBuffer(0));
    
    Log.info("Raw Distance/Level Reading (in mm, unsigned): %0x", node.getResponseBuffer(1)); 
    //Particle.publish("Raw Distance/Level Reading in mm, unsigned", data[1]);
    //mm = (node.getResponseBuffer(1));
    //Blynk.virtualWrite(V1, node.getResponseBuffer(1));
    
    Log.info("Temperature Reading (in C, signed): %0x", node.getResponseBuffer(3));
    //temp = (node.getResponseBuffer(3));
    //temp = temp * 1.8F + 32;
    
    UINT32_ARRAY val;
    val.dblBytes[0] = node.getResponseBuffer(4);
    val.dblBytes[1] = node.getResponseBuffer(5);
    Log.info("Calculated (raw): %0x", val.bigNum);

    Log.info("Version: %0x", node.getResponseBuffer(8));
    Serial.println(highByte(data));

    Log.info("Signal Strength: %0x", lowByte(data));

    data = node.getResponseBuffer(10);
    Log.info("Trip 1 Alarm: %0x", highByte(data));
    Log.info("Trip 1 Status: %0x", lowByte(data));

    data = node.getResponseBuffer(11);
    Log.info("Trip 2 Alarm: %0x", highByte(data));
    Log.info("Trip 2 Status: %0x", lowByte(data));
  
  } else {
    Log.warn("Failed to read input registers...");
    Log.warn("Result: %0x", result);
  }
}