/**
 * Adapted MotorModbus to work with Argon
 * Argon only supports a single hardware serial (Serial1)
 * and a USBSerial (Serial)
 * https://docs.particle.io/reference/device-os/firmware/argon/#serial
 * Additionaly, it does not support the softwareserial libraries for arduino.
**/

// This #include statement was automatically added by the Particle IDE.
#include <ModbusMaster-Particle.h>

#include <blynk.h>
//char auth[] = "cvxyCmbzdECipLp5VXu7BqLWlLAVYhM8"; 
BlynkTimer timer;
bool sendtoblynkenable = 0;

//WidgetLED led1(V12);

// Changed pins for argon
#define MAX485_DE      2
#define MAX485_RE_NEG  3

#define MAX_TRIES 5

SerialLogHandler logHandler(9600,LOG_LEVEL_WARN, {
    {"app", LOG_LEVEL_TRACE},
    {"system", LOG_LEVEL_INFO}
});

// instantiate ModbusMaster object
ModbusMaster node;

// Particle limits topic names to 64 and publish events messages to 622 characters
char *topic = "vvnetwork";
char toPublish[512];

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
  //Time.zone(-2.5);
  //Blynk.begin(auth);
  //timer.setInterval(5000L, sendinfo);
  timer.setInterval(12000L, readTest);
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus RTU Ultrasonic communication runs at 9600 baud
  Serial1.begin(9600);
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
  //Blynk.run();
  timer.run();
}

void sendinfo()
{
    if(sendtoblynkenable == 1)
    {
        //Blynk.virtualWrite(V100, Time.format(Time.local(), "%r - %a %D"));
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

void publish(char* msg) {
  if (!Particle.publish(topic, toPublish)) {
    Log.info("Failed to publish");
    Log.info(toPublish);
  }
}

void readTest() {
  uint8_t result = 0xE2;
  uint16_t data = 0;
  uint8_t tries = 0;

  // Read coils at function address FC(0x01) Register(00001) Address(0) Hex(0x0000) (read/write configuration bits)
  publish("Reading configuration bits");
  tries = 0;
  do {
    result = node.readCoils(0, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    sprintf(
      toPublish,
      "Coils Raw Data: %d\n"
      "Disk on CAM=0 CRANK=1: %d\n"
      "Test for proper disk YES=1: %d\n"
      "Enable secondary Diags YES=1: %d\n"
      "Energy Bit0 00=~160 01=~170: %d\n"
      "Energy Bit1 10=~180 11=~190: %d\n"
      "SLAVE: %d\n"
      "reserved: %d\n"
      "reserved: %d\n",
      data,
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read coils\n"
      "Result: %d",
      result
    );

    publish(toPublish);
  }

  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);

  // Read discrete inputs at function address FC(0x02) Register(10001) Address(0) Hex(0x0000) (Read-Only Status Bits)
  // Switched from coils to discrete inputs
  publish("Reading Discrete Inputs");
  tries = 0;
  do {
    result = node.readDiscreteInputs(0, 24);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    sprintf(
      toPublish,
      "1st Raw data: %d\n"
      "Syncing: %d\n"
      "InSync1: %d\n"
      "InSync2: %d\n"
      "Purging: %d\n"
      "Trying: %d\n"
      "Firing: %d\n"
      "LockOut: %d\n"
      "FIRED: %d",
      lowByte(data),
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );

    publish(toPublish);
    
    data = highByte(data);
    sprintf(
      toPublish,
      "2nd Raw data: %d\n"
      "Cranking: %d\n"
      "Running: %d\n"
      "Wrong Disk: %d\n"
      "GLead Shutdown Grounded: %d\n"
      "Remote Shutdown Present: %d\n"
      "GLead Shutdown Logged: %d\n"
      "Remote Shutdown Logged: %d\n"
      "Overspeed Shutdown Logged: %d\n",
      data,
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );
    
    publish(toPublish);
    
    data = node.getResponseBuffer(1);
    sprintf(
      toPublish,
      "3rd Raw data: %d\n"
      "WDOG1 Reset Latched: %d\n"
      "WDOG2 Reset Event: %d\n"
      "CheckSum Error: %d\n"
      "LOW Supply Voltage: %d\n"
      "No Charge: %d\n"
      "Open Primary: %d\n"
      "Shorted Primary: %d\n"
      "Open Secondary: %d",
      data,
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read discrete inputs\n"
      "Result: %d",
      result
    );

    publish(toPublish);
  }

  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);

  publish("Reading Input Bit Mirrors");
  
  tries = 0;
  do {
    result = node.readInputRegisters(0, 4);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);
  
  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    sprintf(
      toPublish,
      "Input Bit Mirror 10016-10001: %d\n"
      "Input Bit Mirror 10032-10017: %d\n"
      "Input Bit Mirror 10048-10033: %d\n"
      "Input Bit Mirror 10064-10049: %d\n",
      data,
      node.getResponseBuffer(1),
      node.getResponseBuffer(2),
      node.getResponseBuffer(3)
    );
    
    publish(toPublish);
    
    sprintf(
      toPublish,
      "Bit mirrored: "
      "1st Raw data: %d\n"
      "Syncing: %d\n"
      "InSync1: %d\n"
      "InSync2: %d\n"
      "Purging: %d\n"
      "Trying: %d\n"
      "Firing: %d\n"
      "LockOut: %d\n"
      "FIRED: %d",
      lowByte(data),
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );

    publish(toPublish);
    
    data = highByte(data);
    sprintf(
      toPublish,
      "Bit mirrored: "
      "2nd Raw data: %d\n"
      "Cranking: %d\n"
      "Running: %d\n"
      "Wrong Disk: %d\n"
      "GLead Shutdown Grounded: %d\n"
      "Remote Shutdown Present: %d\n"
      "GLead Shutdown Logged: %d\n"
      "Remote Shutdown Logged: %d\n"
      "Overspeed Shutdown Logged: %d\n",
      data,
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );
    
    publish(toPublish);
    
    data = node.getResponseBuffer(1);
    sprintf(
      toPublish,
      "Bit mirrored: "
      "3rd Raw data: %d\n"
      "WDOG1 Reset Latched: %d\n"
      "WDOG2 Reset Event: %d\n"
      "CheckSum Error: %d\n"
      "LOW Supply Voltage: %d\n"
      "No Charge: %d\n"
      "Open Primary: %d\n"
      "Shorted Primary: %d\n"
      "Open Secondary: %d",
      data,
      bitRead(data, 0),
      bitRead(data, 1),
      bitRead(data, 2),
      bitRead(data, 3),
      bitRead(data, 4),
      bitRead(data, 5),
      bitRead(data, 6),
      bitRead(data, 7)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read input bit mirrors!\n"
      "Result: %d",
      result
    );
    
    publish(toPublish);
  }

  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);

  // Read the 16 registers at function address FC(0x03) Register(30005) Address(4) HEX(0x0004) (Read only status registers starting with RPM)
  publish("Reading read only status registers...");
  
  tries = 0;
  do {
    result = node.readInputRegisters(4, 8); // << seems to be correct original: result = node.readHoldingRegisters(4, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    sprintf(
      toPublish,
      "RPM: %d\n"
      "Timing: %f DEG\n"
      "Switch Position: %d\n"
      "Current Loop Input: %f mA\n"
      "Disk Observed X+1: %d\n"
      "Insertion Retard: %f DEG\n"
      "Switch Retard: %f DEG\n"
      "Loop Retard: %f DEG",
      node.getResponseBuffer(0),
      ((signed)node.getResponseBuffer(1) / 10.0f),
      node.getResponseBuffer(2),
      (node.getResponseBuffer(3) / 10.0f),
      node.getResponseBuffer(4),
      (node.getResponseBuffer(5) / 10.0f),
      (node.getResponseBuffer(6) / 10.0f),
      (node.getResponseBuffer(7) / 10.0f)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read 1st set read-only status registers!\n"
      "Result: %d",
      result
    );

    publish(toPublish);
  }

  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);

  tries = 0;
  do {
    result = node.readInputRegisters(12, 8); //<<correct now original: result = node.readHoldingRegisters(12, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {  
    sprintf(
      toPublish,
      "RPM Retard: %f DEG\n"
      "Total Retard: %f DEG\n"
      "Cycle Counter HI: %d\n"
      "Cycle Counter LO: %d\n"
      "Supply Voltage: %f Volts\n"
      "Spark Ref. Num. Output 1: %d\n"
      "Spark Ref. Num. Output 2: %d\n"
      "Spark Ref. Num. Output 3: %d\n",
      (node.getResponseBuffer(0) / 10.0f),
      (node.getResponseBuffer(1) / 10.0f),
      node.getResponseBuffer(2),
      node.getResponseBuffer(3),
      (node.getResponseBuffer(4) / 10.0f),
      node.getResponseBuffer(5),
      node.getResponseBuffer(6),
      node.getResponseBuffer(7)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read 2nd set read-only status registers!\n"
      "Result: %d",
      result
    );

    publish(toPublish);
  }

  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);
  
  // Read 8 read/write registers starting at function address FC(0x04) Register(40005) Address(4) HEX(0x0004)
  Particle.publish("Reading read/write input registers..."); //origianl: publish("Reading read/write input registers...");

  tries = 0;
  do {
    result = node.readHoldingRegisters(4, 8);  // <<corrrect original: result = node.readInputRegisters(4, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    sprintf(
      toPublish,
      "Disk+1: %d\n"
      "Disk Lineup to TDC: %f DEG\n"
      "Insertion Ret MIN=2.0 DEG: %f DEG\n"
      "Purge Delay Cycles 0-255: %d\n"
      "RPM Over Speed Setpoint: %d\n"
      "RPM Crank to Run Threshold: %d\n"
      "Low Supply Voltage Limit: %fV\n"
      "SLAVE ANGLE: %f DEG\n",
      node.getResponseBuffer(0),
      (node.getResponseBuffer(1) / 10.0f),
      (node.getResponseBuffer(2) / 10.0f),
      node.getResponseBuffer(3),
      node.getResponseBuffer(4),
      node.getResponseBuffer(5),
      (node.getResponseBuffer(6) / 10.0f),
      (node.getResponseBuffer(7) / 10.0f)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read input registers...\n"
      "Result: %d",
      result
    );

    publish(toPublish);
  }
  
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);

  // Read 7 misc read/write registers starting at function address FC(0x04) Register(40122) Address(121) HEX(0x0079)
  publish("Reading misc read/write input registers...");

  tries = 0;
  do {
    result = node.readHoldingRegisters(121, 7); //<<mostly working original: result = node.readInputRegisters(121, 7);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(4);

    sprintf(
      toPublish,
      "Crank Counter: %d\n"
      "Start Counter: %d\n"
      "Cycle Counter HIGH: %d\n"
      "Cycle Counter LOW: %d\n"
      "BAUD (fixed 9600): %d\n"
      "NODEID (fixed: n81:node1): %d\n"
      "Cold Boot (powerup) Count: %d\n"
      "Warm Boot (reset) Count: %f\n",
      node.getResponseBuffer(0),
      node.getResponseBuffer(1),
      node.getResponseBuffer(2),
      node.getResponseBuffer(3),
      highByte(data),
      lowByte(data),
      node.getResponseBuffer(5),
      (node.getResponseBuffer(6) / 10.0f)
    );

    publish(toPublish);
  } else {
    sprintf(
      toPublish,
      "Failed to read misc input registers...\n"
      "Result: %d",
      result
    );

    publish(toPublish);
  }
  
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  delay(15000);
}