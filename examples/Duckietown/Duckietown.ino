#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_NONE
#include <SerialDXL.h>

// Duck basic config
#define DUCK_MODEL 100
#define DUCK_FIRMWARE 100
#define DUCK_MMAP_SIZE 3 // Use 3 variable

// Duck pins
#define DUCK_LED 13
#define DUCK_A_1B 5
#define DUCK_B_1B 6
#define DUCK_A_1A 11
#define DUCK_B_1A 12

/**
 * @brief Duck control using DXL communication protocol
 * 
 */
class Duck: public DeviceDXL<DUCK_MODEL, DUCK_FIRMWARE, DUCK_MMAP_SIZE>
{
  public:
    Duck():
    DeviceDXL(), // Call parent constructor
    command_(MMap::Access::RW, MMap::Storage::RAM), // Led command
    pwmCh1_(MMap::Access::RW, MMap::Storage::RAM), // PWM channel 1 command
    pwmCh2_(MMap::Access::RW, MMap::Storage::RAM) // PWM channel 2 command
    {
      // Config pins
      pinMode(DUCK_LED, OUTPUT);
      pinMode(DUCK_A_1B, OUTPUT);
      pinMode(DUCK_A_1A, OUTPUT);
      pinMode(DUCK_B_1B, OUTPUT);
      pinMode(DUCK_B_1A, OUTPUT);
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init();
      mmap_.registerVariable(&command_);
      mmap_.registerVariable(&pwmCh1_);
      mmap_.registerVariable(&pwmCh2_);
      mmap_.init();
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      INFO_PRINT("id: ");INFO_PRINTLN_RAW(id_.data);
      
      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */
    }

    void update()
    {
      // LED Update
      uint8_t ledCmd = command_.data == 1U ? HIGH : LOW;
      digitalWrite(DUCK_LED, ledCmd);

      // PWM Channel A update
      uint8_t forwardCmd = pwmCh1_.data > 0 ? HIGH : LOW;
      uint8_t duty = forwardCmd == HIGH ? pwmCh1_.data : -pwmCh1_.data;
      digitalWrite(DUCK_A_1A, forwardCmd);
      analogWrite(DUCK_A_1B, duty);

      // PWM Channel B update
      forwardCmd = pwmCh2_.data > 0 ? HIGH : LOW;
      duty = forwardCmd == HIGH ? pwmCh2_.data : -pwmCh2_.data;
      digitalWrite(DUCK_B_1A, forwardCmd);
      analogWrite(DUCK_B_1B, duty);
    }

    inline bool onReset(){;}

    inline void setTX(){;}

    inline void setRX(){;}

  private:    
    // LED variable
    MMap::Integer<UInt8, 0, 1, 1>::type command_;
    // PWM channels
    MMap::Integer<Int16, -255, 255, 0>::type pwmCh1_;
    MMap::Integer<Int16, -255, 255, 0>::type pwmCh2_;
};


Duck duck;
SerialDXL<Duck> serialDxl;

void setup() {
  Serial.begin(57600);
  duck.init();
  duck.reset();
  duck.mmap_.serialize();

  // Init serial communication using Dynamixel format
  serialDxl.init(&Serial ,&duck);
  
}

void loop() {
  // Update msg buffer
  while (Serial.available()){
    serialDxl.process(Serial.read());
  }
  
  duck.mmap_.deserialize();
  duck.update();
  duck.mmap_.serialize();
  
}
