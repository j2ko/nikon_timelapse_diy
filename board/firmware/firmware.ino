#include "Thread.h"
#include "ThreadController.h"

#ifdef DEBUG
#define LOG(X) Serial.print(X);
#define LOGN(X) Serial.println(X);
#else 
#define LOG(X)
#define LOGN(X)
#endif

//TODO: 
//- refactor. Introduce command registry
//- introduce clear way to serialize/deserialize object to/from TLV

const int PIN_CAPTURE = 9;

ThreadController controller = ThreadController();

enum Command {
  START = 0x1,
  GET_STATUS = 0x2,
  STOP  = 0x3,
};

enum Mode { 
  TIMEBASED= 0x1,
  COUNTBASED = 0x2   
};

struct TimeBasedMode {
  uint32_t time;
  uint8_t interval;
};

struct CountBasedMode {
  uint16_t count;
  uint8_t interval;
};

typedef struct {
  uint8_t tag;
  uint8_t length;
  uint8_t value[255];
} TLV;

/*
 *  Class for testing purpose
 */
class BlinkMainLed : public Thread {
    boolean isOn;
public:
    BlinkMainLed() {      
      isOn = digitalRead(LED_BUILTIN) == HIGH ;      
    }

    void run() {              
       digitalWrite(LED_BUILTIN, isOn ? LOW : HIGH);  
       isOn = !isOn;         
       runned();
    }
};


/*
 * Used to receive TLV packets from serial
 * TODO: error handling
 */ 
class SerialChannel : public Thread {  
public:
  class SerialCommandHandler {
  public:
    virtual void onCommandReceived(int tag, byte len, byte *buf) = 0;
  };

  enum ReadState {
    READ_TAG,
    READ_LEN,
    READ_VALUE
  };

  SerialChannel(SerialCommandHandler *handler): handler_(handler) {
    
  }
    
  void run() {
    if (Serial.available() > 0) {
      uint8_t inByte = Serial.read();
      uint8_t next_state;
      switch(state) {
        case READ_TAG:
          message.tag = inByte;            
          next_state = READ_LEN;          
          break;
        case READ_LEN:
          message.length = inByte;
          if (message.length == 0) {//empty payload
            next_state = READ_TAG;
            handler_->onCommandReceived(message.tag, message.length, message.value);
          } else {
            next_state = READ_VALUE;
          }
          break;       
        case READ_VALUE:
          message.value[actualy_read++] = inByte;
          
          if (actualy_read == message.length) {
            next_state = READ_TAG;
            actualy_read = 0;
            handler_->onCommandReceived(message.tag, message.length, message.value);
          } else {
            next_state = READ_VALUE;
          }
          break;
      }
     
      state = next_state;
    }

    runned();
  }
private:
  SerialCommandHandler * handler_; 
  ReadState state = READ_TAG;
  TLV message;
  size_t actualy_read = 0; 
};

class Task : public Thread {
  bool isFirstRun = true;
public: 
  class Callback {
  public:
    virtual void OnTaskStarted(Task *taks) = 0;
    virtual void OnTaskFinished(Task *task) = 0;    
  };
  
  Task(Callback *cb): cb_(cb) {
    
  }
    
private:
  virtual void run() {
    if (isFirstRun) {
      isFirstRun = false;
      onStarted();
    }
    iteration();    
    runned();
  }
   
protected:
  virtual void iteration() = 0;
  
  virtual void onStarted() {
    cb_->OnTaskStarted(this);
  }
  
  virtual void onFinished() {
    digitalWrite(PIN_CAPTURE, HIGH);
    cb_->OnTaskFinished(this);
  }
  
  Callback* cb_;
};

class TakeCaptureTask : public Task {
  uint16_t capturesTaken;
public:
  TakeCaptureTask(Callback* cb) : Task(cb), capturesTaken(0) {   
  }
  
  virtual void getInfo(TLV& info) = 0;
protected: 

  void performCapture() {
    LOG("[+]");
    digitalWrite(PIN_CAPTURE, HIGH);        
    delay(100); //need to rid off delays
    digitalWrite(PIN_CAPTURE, LOW);
    capturesTaken++;
    LOG("[-]");
  }

  long getCapturesTaken() {
    return capturesTaken;
  }
};

class TimeBasedTask : public TakeCaptureTask {
  long timeout_;
  long deadline_;
  uint8_t interval_;
public:
  TimeBasedTask(Callback *cb, long time, uint8_t interval): TakeCaptureTask(cb), timeout_(time), interval_(interval) {    
    setInterval(((uint32_t)interval)*1000);
    LOGN(String("TimeBasedTask [time ") + time  + "][ interval " + interval + "]");
  }

  virtual void getInfo(TLV& info) {
    info.tag = TIMEBASED;
    info.length = 0x7;
    long remaining = (deadline_ - millis());
    //TODO: running time also could be appropriate.
    //remaining time
    info.value[0] = remaining >> 24;
    info.value[1] = (remaining >> 16) & 0xFF;
    info.value[2] = (remaining >> 8) & 0xFF;
    info.value[3] = remaining & 0xFF;
    //interval
    info.value[4] = interval_;
    //captures taken so far
    uint16_t capTaken = getCapturesTaken();
    info.value[5] = capTaken >> 8;
    info.value[6] = capTaken & 0xFF;
  }
  
protected:
  virtual void onStarted() {
    deadline_ = millis() + timeout_ * 1000;
    Task::onStarted();    
  }
  
  virtual void iteration() {
    if (deadline_ < millis()) {
      onFinished();
      return;
    }

    performCapture();    
  }
};

class CountBasedTask : public TakeCaptureTask {
  int32_t count_;
  uint8_t interval_;
public:
  CountBasedTask(Callback* cb, uint16_t count, uint8_t interval): TakeCaptureTask(cb), count_(count), interval_(interval) {    
    setInterval(((uint32_t)interval)*1000);
    LOGN(String("CountBasedTask [count ") + count  + "][ interval " + interval + "]");
  }

  virtual void getInfo(TLV& info) {
    info.tag = COUNTBASED;
    info.length = 0x5;
    uint16_t remaining = count_;
    //TODO: running time also could be appropriate.
    //remaining amaunt of captures.
    info.value[0] = (remaining >> 8) & 0xFF;
    info.value[1] = remaining & 0xFF;
    //interval
    info.value[2] = interval_;
    //captures taken so far
    uint16_t capTaken = getCapturesTaken();
    info.value[3] = (capTaken >> 8) & 0xFF;
    info.value[4] = capTaken & 0xFF;
  }
  
  void iteration() {    
    if (--count_ < 0)  {
      onFinished();
      return;
    }
    
    performCapture();
  }
};

class CommandHandler : public SerialChannel::SerialCommandHandler, public Task::Callback {
  BlinkMainLed blink_main_led_;
  TakeCaptureTask* running_task_;
public:
  CommandHandler() {
    blink_main_led_.setInterval(500);
  }
  
  virtual void onCommandReceived(int tag, byte len, byte *buf) {
    LOGN(String("RECEIVED") + tag + " with len " + len);
    switch (tag) {
      case START:
        setTask(parseTask(len, buf));
        break;
      case GET_STATUS:
        requestTaskInfo(running_task_);
        break;
      case STOP:
        setTask(nullptr);
        break;
    }
  }
  
private:
  void requestTaskInfo(TakeCaptureTask* task) {
    TLV tlv;
    if (task == nullptr) {
      tlv.tag = 0x00;
      tlv.length = 0x00;
    } else {
      task->getInfo(tlv);
    }
    
    respondWithTLV(tlv);
  }

  void respondWithTLV(const TLV& response) {
    Serial.write(response.tag);
    Serial.write(response.length);
    for (int i = 0; i < response.length; ++i) {   
      Serial.write(response.value[i]);
    }    
  }
  
  Task* parseTask(byte len, byte *buff) {
    uint8_t mode = buff[0];
    switch(mode) {
      case TIMEBASED: {
        uint32_t time;
        time  = (uint32_t)buff[1] << 24;
        time |= (uint32_t)buff[2] << 16;
        time |= (uint32_t)buff[3] << 8;
        time |= (uint32_t)buff[4];
        byte interval = buff[5];
        return new TimeBasedTask(this, time, interval);
      }
      case COUNTBASED: {
        short count;
        count  = (uint16_t)buff[1] << 8;
        count |= (uint16_t)buff[2];
        uint8_t interval = buff[3];
        return new CountBasedTask(this, count, interval);
      }
    }

    return nullptr;
  }
  
  virtual void OnTaskStarted(Task *taks) {
    LOGN("TASK STARTED");
    controller.add(&blink_main_led_);
  }
  
  virtual void OnTaskFinished(Task *task) {
    LOGN("TASK FINISHED");
    controller.remove(&blink_main_led_);
    setTask(nullptr);
  }

  void setTask(TakeCaptureTask *task) {
    if (running_task_ != nullptr) {
        controller.remove(running_task_);
        controller.remove(&blink_main_led_);     
        delete running_task_;
    }

    running_task_ = task;
    controller.add(task);
  }
};

CommandHandler command_handler;
SerialChannel serial_channel(&command_handler);

void setup() { 
  Serial.begin(9600);
  pinMode(PIN_CAPTURE, OUTPUT);
  digitalWrite(PIN_CAPTURE, HIGH);
  serial_channel.setInterval(100);
  controller.add(&serial_channel); 
}

void loop() {
   controller.run();
}
