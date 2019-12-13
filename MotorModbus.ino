//12/12/2019 run Code

#include <ModbusMaster.h>

/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial1'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/
// These can be the same pin
#define MAX485_DE      22
#define MAX485_RE_NEG  23

#define MAX_TRIES 5

// instantiate ModbusMaster object
ModbusMaster node;

#define modbusSerial Serial1

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
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus RTU Ultrasonic communication runs at 9600 baud
  modbusSerial.begin(9600);
  Serial.begin(115200);
 
  // Modbus slave ID 1
  node.begin(1, modbusSerial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

typedef union {
  uint16_t dblBytes[2];
  uint32_t bigNum;
} UINT32_ARRAY;


void loop()
{
  readTest();
}

void readTest() {
  uint8_t result = 0xE2;
  uint16_t data;
  uint8_t tries = 0;

  // Read coils at function address FC(0x01) Register(10001) Address(0) Hex(0x0000) (Read-Only Status Bits)
  Serial.println("Reading status bits...");
  do {
    result = node.readCoils(0, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  // Not 100% on the order of bits
  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    Serial.print("1st Raw data: ");Serial.println(data);
    Serial.print("Syncing: ");Serial.println(bitRead(data, 0));
    Serial.print("InSync1: ");Serial.println(bitRead(data, 1));
    Serial.print("InSync2: ");Serial.println(bitRead(data, 2));
    Serial.print("Purging: ");Serial.println(bitRead(data, 3));
    Serial.print("Trying: ");Serial.println(bitRead(data, 4));
    Serial.print("Firing: ");Serial.println(bitRead(data, 5));
    Serial.print("LockOut: ");Serial.println(bitRead(data, 6));
    Serial.print("FIRED: ");Serial.println(bitRead(data, 7));
  } else {
    Serial.println("Failed to read 1st set of coils");
    Serial.print("Result: ");Serial.println(result);
  }
  
  delay(1000);
  Serial.flush();

  tries = 0;
  do {
    result = node.readCoils(8, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    Serial.print("2nd Raw data: ");Serial.println(data);
    Serial.print("Cranking: ");Serial.println(bitRead(data, 0));
    Serial.print("Running: ");Serial.println(bitRead(data, 1));
    Serial.print("Wrong Disk: ");Serial.println(bitRead(data, 2));
    Serial.print("GLead Shutdown Grounded: ");Serial.println(bitRead(data, 3));
    Serial.print("Remote Shutdown Present: ");Serial.println(bitRead(data, 4));
    Serial.print("GLead Shutdown Logged: ");Serial.println(bitRead(data, 5));
    Serial.print("Remote Shutdown Logged: ");Serial.println(bitRead(data, 6));
    Serial.print("Overspeed Shutdown Logged: ");Serial.println(bitRead(data, 7));
  } else {
    Serial.println("Failed to read 2nd set of coils");
    Serial.print("Result: ");Serial.println(result);
  }
  
  delay(1000);
  Serial.flush();

  tries = 0;
  do {
    result = node.readCoils(16, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    Serial.print("3rd Raw data: ");Serial.println(data);
    Serial.print("WDOG1 Reset Latched: ");Serial.println(bitRead(data, 0));
    Serial.print("WDOG2 Reset Event: ");Serial.println(bitRead(data, 1));
    Serial.print("CheckSum Error: ");Serial.println(bitRead(data, 2));
    Serial.print("LOW Supply Voltage: ");Serial.println(bitRead(data, 3));
    Serial.print("No Charge: ");Serial.println(bitRead(data, 4));
    Serial.print("Open Primary: ");Serial.println(bitRead(data, 5));
    Serial.print("Shorted Primary: ");Serial.println(bitRead(data, 6));
    Serial.print("Open Secondary: ");Serial.println(bitRead(data, 7));
  } else {
    Serial.println("Failed to read 3rd set of coils");
    Serial.print("Result: ");Serial.println(result);
  }

  delay(1000);
  Serial.flush();

  // Read the 16 registers at function address FC(0x03) Register(30005) Address(4) HEX(0x0004) (Read only status registers starting with RPM)
  Serial.println("Reading read only status registers...");
  
  tries = 0;
  do {
    result = node.readHoldingRegisters(4, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    Serial.print("RPM: ");Serial.println(node.getResponseBuffer(0));
    Serial.print("Timing: ");Serial.print(((signed)node.getResponseBuffer(1)) / 10.0);Serial.println(" DEG");
    Serial.print("Switch Position: ");Serial.println(node.getResponseBuffer(2));
    Serial.print("Current Loop Input: ");Serial.print(node.getResponseBuffer(3) / 10.0);Serial.println(" mA");
    Serial.print("Disk Observed X+1: ");Serial.println(node.getResponseBuffer(4));
    Serial.print("Insertion Retard: ");Serial.print(node.getResponseBuffer(5) / 10.0);Serial.println(" DEG");
    Serial.print("Switch Retard: ");Serial.print(node.getResponseBuffer(6) / 10.0);Serial.println(" DEG");
    Serial.print("Loop Retard: ");Serial.print(node.getResponseBuffer(7) / 10.0);Serial.println(" DEG");
  } else {
    Serial.println("Failed to read 1st set read-only status registers!");
    Serial.print("Result: ");Serial.println(result);
  }

  delay(1000);
  Serial.flush();

  tries = 0;
  do {
    result = node.readHoldingRegisters(12, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {  
    Serial.print("RPM Retard: ");Serial.print(node.getResponseBuffer(8) / 10.0);Serial.println(" DEG");
    Serial.print("Total Retard: ");Serial.print(node.getResponseBuffer(9) / 10.0);Serial.println(" DEG");
    Serial.print("Cycle Counter HI: ");Serial.println(node.getResponseBuffer(10));
    Serial.print("Cycle Counter LO: ");Serial.println(node.getResponseBuffer(11));
    Serial.print("Supply Voltage: ");Serial.print(node.getResponseBuffer(12) / 10.0);Serial.println(" Volts");
    Serial.print("Spark Ref. Num. Output 1: ");Serial.println(node.getResponseBuffer(13));
    Serial.print("Spark Ref. Num. Output 2: ");Serial.println(node.getResponseBuffer(14));
    Serial.print("Spark Ref. Num. Output 3: ");Serial.println(node.getResponseBuffer(15));
  } else {
    Serial.println("Failed to read 2nd set read-only status registers!");
    Serial.print("Result: ");Serial.println(result);
  }

  delay(1000);
  Serial.flush();
  
  // Read 8 read/write registers starting at function address FC(0x04) Register(40005) Address(4) HEX(0x0004)
  Serial.println("Reading read/write input registers...");

  tries = 0;
  do {
    result = node.readInputRegisters(4, 8);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    Serial.print("Disk+1: ");
    Serial.println(node.getResponseBuffer(0));
    
    Serial.print("Disk Lineup to TDC: ");
    Serial.print(node.getResponseBuffer(1) / 10.0); 
    Serial.println(" DEG");

    Serial.print("Insertion Ret MIN=2.0 DEG: ");
    Serial.print(node.getResponseBuffer(2) / 10.0);
    Serial.println(" DEG");

    Serial.print("Purge Delay Cycles 0-255: ");
    Serial.println(node.getResponseBuffer(3));

    Serial.print("RPM Over Speed Setpoint: ");
    Serial.println(node.getResponseBuffer(4));

    Serial.print("RPM Crank to Run Threshold: ");
    Serial.println(node.getResponseBuffer(5));

    Serial.print("Low Supply Voltage Limit: ");
    Serial.print(node.getResponseBuffer(6) / 10.0);
    Serial.println("V");

    Serial.print("SLAVE ANGLE: ");
    Serial.print(node.getResponseBuffer(7) / 10.0);
    Serial.println(" DEG");
  
  } else {
    Serial.println("Failed to read input registers...");
        Serial.print("Result: ");Serial.println(result);
  }
  
  delay(1000);
  Serial.flush();

    // Read 7 misc read/write registers starting at function address FC(0x04) Register(40122) Address(121) HEX(0x0079)
  Serial.println("Reading misc read/write input registers...");

  tries = 0;
  do {
    result = node.readInputRegisters(121, 7);
    tries++;
  }
  while (tries < MAX_TRIES && result != node.ku8MBSuccess);

  if (result == node.ku8MBSuccess) {
    Serial.print("Crank Counter: ");
    Serial.println(node.getResponseBuffer(0));
    
    Serial.print("Start Counter: ");
    Serial.println(node.getResponseBuffer(1)); 

    Serial.print("Cycle Counter HIGH: ");
    Serial.println(node.getResponseBuffer(2));

    Serial.print("Cycle Counter LOW: ");
    Serial.println(node.getResponseBuffer(3));

    data = node.getResponseBuffer(4);
    Serial.print("BAUD (fixed 9600): ");
    Serial.println(highByte(data));
    
    Serial.print("NODEID (fixed n81:node1): ");
    Serial.println(lowByte(data));

    Serial.print("Cold Boot (powerup) Count: ");
    Serial.println(node.getResponseBuffer(5));

    Serial.print("Warm Boot (reset) Count: ");
    Serial.print(node.getResponseBuffer(6) / 10.0);  
  } else {
    Serial.println("Failed to read misc input registers...");
    Serial.print("Result: ");Serial.println(result);
  }
  
  delay(1000);
  Serial.flush();
}
