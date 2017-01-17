#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_NONE
#include <SerialDXL.h>

// Duck DXL basic config
#define DUCK_MODEL 100
#define DUCK_FIRMWARE 100
#define DUCK_MMAP_SIZE 3 // Use 3 variable

/**
 * @brief Duck control using DXL communication protocol
 * 
 * @param reset_pin Pin for reset device
 * @param led_pin LED pin.
 */
class Duck: public DeviceDXL<DUCK_MODEL, DUCK_FIRMWARE, DUCK_MMAP_SIZE>
{
  public:
    Duck(uint8_t led_pin):
    DeviceDXL(), // Call parent constructor
    led_pin_(led_pin),        // LED pin
    command_(MMap::Access::RW, MMap::Storage::RAM), // Led command
    pwmCh1_(MMap::Access::RW, MMap::Storage::RAM), // PWM channel 1 command
    pwmCh2_(MMap::Access::RW, MMap::Storage::RAM) // PWM channel 2 command
    {
      // Config pins
      pinMode(led_pin_, OUTPUT);
      pinMode(5, OUTPUT);
      pinMode(11, OUTPUT);
      pinMode(6, OUTPUT);
      pinMode(12, OUTPUT);
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
      uint8_t ledCmd = command_.data == 1 ? HIGH : LOW;
      digitalWrite(led_pin_, ledCmd);

      // PWM Channel 1 update
      uint8_t forwardCmd = pwmCh1_.data >= 0 ? HIGH : LOW;
      uint8_t duty = forwardCmd == HIGH ? pwmCh1_.data : -pwmCh1_.data;
      digitalWrite(11, forwardCmd);
      analogWrite(5, duty);

      // PWM Channel 2 update
      forwardCmd = pwmCh2_.data >= 0 ? HIGH : LOW;
      duty = forwardCmd == HIGH ? pwmCh2_.data : -pwmCh2_.data;
      digitalWrite(12, forwardCmd);
      analogWrite(6, duty);
      
    }

    inline bool onReset(){;}

    inline void setTX(){;}

    inline void setRX(){;}

  private:
    const uint8_t led_pin_; // LED pin
    float float_raw;
    
    // LED variable
    MMap::Integer<UInt8, 0, 1, 1>::type command_;
    // PWM channels
    MMap::Integer<Int16, -255, 255, 0>::type pwmCh1_;
    MMap::Integer<Int16, -255, 255, 0>::type pwmCh2_;
};


Duck duck(13);
SerialDXL<Duck> serialDxl;

void setup() {
  Serial.begin(115200);
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
