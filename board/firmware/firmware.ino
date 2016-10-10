#include "Thread.h"
#include "ThreadController.h"

#ifdef DEBUG
#define LOG(X) Serial.print(X);
#define LOGN(X) Serial.println(X);
#else 
#define LOG(X)
#define LOGN(X)
#endif

const int PIN_CAPTURE = 9;

ThreadController controller = ThreadController();

enum Command {
  START = 0x1,
  STOP  = 0x2,  
};

enum Mode { 
  TIMEBASED= 0x1,
  COUNTBASED = 0x2   
};

struct TimeBasedMode {
  long time;
  byte interval;
};

struct CountBasedMode {
  short count;
  byte interval;
};

typedef struct {
    byte tag;
    byte length;
    byte value[255];
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
      byte inByte = Serial.read();
      int next_state;
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
  int actualy_read = 0; 
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
    cb_->OnTaskFinished(this);
  }
  
  Callback* cb_;
};

class TakeCaptureTask : public Task {
public:
  TakeCaptureTask(Callback* cb) : Task(cb) {   
  }
protected: 
  void performCapture() {
    LOG("[+]");
    //this could be separated by states
    digitalWrite(PIN_CAPTURE, HIGH);
    delay(50); //need to rid off delays
    digitalWrite(PIN_CAPTURE, LOW);
    delay(50); 
    digitalWrite(PIN_CAPTURE, HIGH);
  }
};

class TimeBasedTask : public TakeCaptureTask {
  long timeout_;
  long deadline_;
  int interval_;
public:
  TimeBasedTask(Callback *cb, long time, int interval): TakeCaptureTask(cb), timeout_(time), interval_(interval) {    
    setInterval(interval * 1000);
    LOGN(String("TimeBasedTask [time ") + time  + "][ interval " + interval + "]");
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
  int count_;
  int interval_;  
public:
  CountBasedTask(Callback* cb, int count, int interval): TakeCaptureTask(cb), count_(count), interval_(interval) {    
    setInterval(interval*1000);
    LOGN(String("CountBasedTask [count ") + count  + "][ interval " + interval + "]");
  }

  void iteration() {    
    if (--count_ <= 0)  {
      onFinished();
      return;
    }
    
    performCapture();
  }
};

class CommandHandler : public SerialChannel::SerialCommandHandler, public Task::Callback {
  BlinkMainLed blink_main_led_;
  Task* running_task_;
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
      case STOP:
        setTask(nullptr);
        break;
    }
  }
  
private:
  Task* parseTask(byte len, byte *buff) {
    byte mode = buff[0];
    switch(mode) {
      case TIMEBASED: {
        long time;
        time  = (long)buff[1] << 24;
        time |= (long)buff[2] << 16;
        time |= (long)buff[3] << 8;
        time |= (long)buff[4];        
        byte interval = buff[5];
        return new TimeBasedTask(this, time, interval);
      }
      case COUNTBASED: {
        short count;
        count  = (short)buff[1] << 8;
        count |= (short)buff[2];
        byte interval = buff[3];
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

  void setTask(Task *task) {
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
  
  serial_channel.setInterval(100);
  controller.add(&serial_channel); 
}

void loop() {
   controller.run();
}
