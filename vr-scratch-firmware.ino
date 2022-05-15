#include "Arduino.h"
#include "HardwareSerial.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// #define ENABLE_DEBUG

#if defined(ENABLE_DEBUG) && defined(HAVE_HWSERIAL3)
  #define INTERFACE   Serial3
  #define DEBUG       Serial
  #define DEBUG_PRINT(args)        DEBUG.print(args)
  #define DEBUG_PRINTLN(args)      DEBUG.println(args)
  #define DEBUG_PRINT_FMT(args, fmt) DEBUG.print(args, fmt)
  #define DEBUG_PRINTLN_FMT(args, fmt) DEBUG.println(args, fmt)
#elif !defined(ENABLE_DEBUG) && defined(HAVE_HWSERIAL3)
  #define INTERFACE   Serial3
  #undef  DEBUG
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINT_FMT(...)
  #define DEBUG_PRINTLN_FMT(...)
#else
  #define INTERFACE   Serial
  #undef  DEBUG
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINT_FMT(...)
  #define DEBUG_PRINTLN_FMT(...)
#endif

#define FLUSHING_TIME (100U)

#define ERR_OK   (0u)
#define ERR_EXEC (1u)

#define NUMPIXELS (1)
#define RGB_PIN   (4)

#define LCD_I2C_ADDR    (0x20)
#define LCD_COLS        (16)
#define LCD_ROWS        (2)

// Define enum ----------------------------------------------------------------

enum MessageErrStatus : uint8_t {
    MSG_ERR_TIMEOUT,
};

enum MessageReplyType : uint8_t {
    MSG_REPLY_OK,
    MSG_REPLY_ERR,
    MSG_REPLY_INFO,
    MSG_REPLY_DATA,
    MSG_REPLY_QUERY
};

enum MessageState : uint8_t {
    INIT_MESSAGE_STATE = 0,
    MESSAGE_WAIT_NEW_DATA,
    MESSAGE_HEADER_READY,
    MESSAGE_HEADER_TIMEOUT,
    MESSAGE_HEADER_ERR,
    MESSAGE_BODY_READY,
    MESSAGE_BODY_TIMEOUT,
    MESSAGE_BODY_ERR,
    MESSAGE_END_OF_STATE,
};

enum MessageID : uint8_t {
    MSG_ID_PUT_LEDRGB = 0,
    MSG_ID_PUT_LEDTRAFFIC,
    MSG_ID_PUT_DCMOTOR,
    MSG_ID_PUT_LCD,
    MSG_ID_PUT_BUZZER,
    MSG_ID_PUT_SERVO,
    MSG_ID_GET_ULTRASONIC,
    MSG_ID_GET_GAS,
    MSG_ID_GET_PHOTORES,
    MSG_ID_GET_TEMPURATURE,
    MSG_ID_GET_IR_SENSOR,
    MSG_ID_GET_VAR_RES,
    MSG_ID_GET_BUT_STATE,
    MSG_ID_SIZE,
};

enum PortType : uint8_t {
    NON_TYPE = 0,
    LED_TRAFFIC,
    ULTRASONIC,
    SERVO,
    GAS,
    PHOTORES,
    TEMPURATURE,
    IR_SENSOR,
    VAR_RESISTOR,
    BUTTON,
};

enum MotorPort : uint8_t {
    MOTOR_PORT_1 = 0,
    MOTOR_PORT_2,
    MPORT_NUMSIZE,
};


// Define struct --------------------------------------------------------------

struct Msg {
    MessageState state_ = INIT_MESSAGE_STATE;
    uint16_t header_ = 0;
    char tx_[32] = {0};
    char rx_[32] = {0};    
};

typedef int(*msg_cb)(Msg&);

struct MsgHandleCb {
    MessageID    id_;
    msg_cb  callback_;
};

class MotorPortHandler {
public:
    enum Orientation : bool {
        CW = LOW,
        CCW = HIGH,
    };

    struct PortCfg {
        uint8_t dir_pin;
        uint8_t pwr_pin;
    };

#define CW_POWER(x)    (x)
#define CCW_POWER(x)   (255 - x)

    MotorPortHandler(const PortCfg &cfg)
        : dir_io_(cfg.dir_pin), pwr_io_(cfg.pwr_pin), direction_(CW), power_(0) 
    { }

    void init(void) {
        pinMode(dir_io_, OUTPUT);
        digitalWrite(dir_io_, direction_);
        analogWrite(pwr_io_, power_);
    }

    void setDirection(const Orientation dir) {
        if (direction_ == dir) return;
        direction_ = dir;
        digitalWrite(dir_io_, direction_);
        analogWrite(pwr_io_, (direction_ == Orientation::CW) ? CW_POWER(power_) : CCW_POWER(power_));
    }

    void setPower(const uint8_t val) {
        power_ = val;
        analogWrite(pwr_io_, (direction_ == Orientation::CW) ? CW_POWER(power_) : CCW_POWER(power_));
    }

private:
    uint8_t dir_io_;
    uint8_t pwr_io_;
    bool    direction_;
    uint8_t power_;
};


// Function prototype ---------------------------------------------------------

int cmdPutLedRGb(Msg& msg);
int cmdPutLedTraffic(Msg& msg);
int cmdPutDcMotor(Msg& msg);
int cmdPutLcd(Msg& msg);
int cmdPutBuzzer(Msg& msg);
int cmdPutServos(Msg& msg);
int cmdGetUltrasonic(Msg& msg);
int cmdGetGas(Msg& msg);
int cmdGetPhotoRes(Msg& msg);
int cmdGetTemp(Msg& msg);
int cmdGetIRSensor(Msg& msg);
int cmdGetVarRes(Msg& msg);
int cmdGetButState(Msg& msg);

inline void setMessageState(MessageState state);
inline MessageState messageState(void);
inline bool isInMessageState(MessageState state);
void flushingRxSerial(void);
inline MessageID checkMsgIdValid(Msg& msg);

// Declare a global objects and variables -------------------------------------

struct Msg msg;

const MsgHandleCb cb_tbl[MSG_ID_SIZE] = {
    { MSG_ID_PUT_LEDRGB,        cmdPutLedRGb },
    { MSG_ID_PUT_LEDTRAFFIC,    cmdPutLedTraffic },
    { MSG_ID_PUT_DCMOTOR,       cmdPutDcMotor },
    { MSG_ID_PUT_LCD,           cmdPutLcd },
    { MSG_ID_PUT_BUZZER,        cmdPutBuzzer },
    { MSG_ID_PUT_SERVO,         cmdPutServos },
    { MSG_ID_GET_ULTRASONIC,    cmdGetUltrasonic },
    { MSG_ID_GET_GAS,           cmdGetGas },
    { MSG_ID_GET_PHOTORES,      cmdGetPhotoRes },
    { MSG_ID_GET_TEMPURATURE,   cmdGetTemp },
    { MSG_ID_GET_IR_SENSOR,     cmdGetIRSensor },
    { MSG_ID_GET_VAR_RES,       cmdGetVarRes },
    { MSG_ID_GET_BUT_STATE,     cmdGetButState }
};

Adafruit_NeoPixel pixel(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

unsigned char port_init_type = NON_TYPE;

const MotorPortHandler::PortCfg portCfg[MPORT_NUMSIZE] =
{
    { .dir_pin = 8, .pwr_pin = 5 },
    { .dir_pin = 7, .pwr_pin = 6 }
};

MotorPortHandler dcmotor[2] =
{
    MotorPortHandler(portCfg[MOTOR_PORT_1]),
    MotorPortHandler(portCfg[MOTOR_PORT_2]),
};

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

void setup() {
    pixel.begin();
    pixel.setPixelColor(0, Adafruit_NeoPixel::Color(0,0,0));
    pixel.show();

    for (int i=0;i < MPORT_NUMSIZE; ++i) 
        dcmotor[i].init();

#ifdef ENABLE_DEBUG
    DEBUG.begin(115200);
#endif
    INTERFACE.begin(115200);
}

void loop()
{
    // Start receive an incoming message
    while (!INTERFACE.available());

    uint16_t message_size;
    if (INTERFACE.readBytes((char*)&msg.header_, 2) < 2) {
        DEBUG_PRINTLN("Failed to receive message header");
        setMessageState(MESSAGE_HEADER_TIMEOUT);        
    }
    else {
        DEBUG_PRINT("Message header size: ");
        DEBUG_PRINTLN(msg.header_);
        setMessageState(MESSAGE_HEADER_READY);
    }

    if (isInMessageState(MESSAGE_HEADER_READY)) {
        // Prepare to recv body message
        memset((void*)msg.rx_, 0,(size_t)msg.header_);
        
        while(!INTERFACE.available());

        size_t recv_bytes;
        if (recv_bytes = INTERFACE.readBytes(msg.rx_, msg.header_) < msg.header_) {
            DEBUG_PRINT("Receive body message timeout: receive ");
            DEBUG_PRINTLN(recv_bytes);
            setMessageState(MESSAGE_BODY_TIMEOUT);
        }
        else {
            setMessageState(MESSAGE_BODY_READY);
        }
    }

    if (isInMessageState(MESSAGE_BODY_READY)) {
        parseMsgBody(msg);
    }

    if ((msg.state_ == MESSAGE_HEADER_TIMEOUT) || (msg.state_ == MESSAGE_BODY_TIMEOUT))
    {
        DEBUG_PRINTLN("Read incoming message header: Timeout");
        flushingRxSerial();
    }
}

inline void setMessageState(MessageState state)
{
    if ((state > INIT_MESSAGE_STATE) && (state < MESSAGE_END_OF_STATE))
        msg.state_ = state;
    else
        DEBUG_PRINTLN("setMessageState: wrong value state");
}

inline MessageState messageState() {
    return msg.state_;
}

inline bool isInMessageState(MessageState state) {
    return (msg.state_ == state) ? true : false;
}

void flushingRxSerial(void) {
    unsigned long ts = millis();
    while(millis() - ts < FLUSHING_TIME) {
        if (INTERFACE.available())
            INTERFACE.read();
    }
}

inline MessageID checkMsgIdValid(Msg& msg)
{
    MessageID id_ = (MessageID)msg.rx_[0];

    DEBUG_PRINT("Message ID: ");
    DEBUG_PRINTLN(id_);

    if (id_ >= 0 && id_ < MSG_ID_SIZE) {
        return id_;
    }
    else {
        msg.header_ = 2;
        msg.tx_[0] = MSG_REPLY_ERR;
        msg.tx_[1] = MSG_ERR_TIMEOUT;
        replyMessage(msg);
        return (MessageID)-1;
    }
}

inline msg_cb getCallback(MessageID id)
{
    int i = 0;
    while(cb_tbl[i].id_ != id && ++i);
    DEBUG_PRINT("Callback table return index ");
    DEBUG_PRINTLN(i);
    return cb_tbl[i].callback_;
}

inline void parseMsgBody (Msg& msg)
{
    MessageID msg_id = checkMsgIdValid(msg);
    if (msg_id == -1) {
        DEBUG_PRINTLN("Message ID is not valid");
        return;
    }

    msg_cb cb = getCallback(msg_id);
    if (cb(msg) != ERR_OK) {
        DEBUG_PRINTLN("parseMsgBody: execute cb failed");
    }
}

void replyMessage(const Msg& msg)
{
    INTERFACE.write((char*)&msg.header_, sizeof(uint16_t));
    INTERFACE.write(msg.tx_, msg.header_);
}

int cmdPutLedRGb(Msg& msg)
{
    auto &r = msg.rx_[1];
    auto &g = msg.rx_[2];
    auto &b = msg.rx_[3];

    pixel.setPixelColor(0, Adafruit_NeoPixel::Color(r,g,b));
    pixel.show();

    INTERFACE.write((char*)&msg.header_, sizeof(uint16_t));
    INTERFACE.write(msg.rx_, msg.header_);

    return ERR_OK;
}

int cmdPutLedTraffic(Msg& msg)
{
    auto &port     = msg.rx_[1];
    auto &l_green  = msg.rx_[2];
    auto &l_yellow = msg.rx_[3];
    auto &l_red    = msg.rx_[4];

    return ERR_OK;
}

int cmdPutDcMotor(Msg& msg)
{
    const auto& cmd = msg.rx_[1];
    auto& port = msg.rx_[2];
    const char* param_ptr = &msg.rx_[3];

    using Orientation = MotorPortHandler::Orientation;

    MotorPort p = (MotorPort)(port % MPORT_NUMSIZE);
    Orientation direction;
    uint8_t power;
    uint8_t i;

    switch (cmd)
    {
    case 0x00: // Set direction
        direction = (param_ptr[0] % 2) ? Orientation::CW : Orientation::CCW;
        dcmotor[p].setDirection(direction);
        break;

    case 0x01: // Set power
        power = param_ptr[0];
        dcmotor[p].setPower(power);
        break;

    case 0x02: // Set full
        direction = (param_ptr[0] % 2) ? Orientation::CW : Orientation::CCW;
        power = param_ptr[0];
        dcmotor[p].setDirection(direction);
        dcmotor[p].setPower(power);
        break;

    case 0x03: // Set all port full
        direction = (param_ptr[0] % 2) ? Orientation::CW : Orientation::CCW;
        power = param_ptr[0];
        for (i=0;i < MPORT_NUMSIZE;++i) {
            dcmotor[i].setDirection(direction);
            dcmotor[i].setPower(power);
        }
        break;
    
    default:
        break;
    }

    return ERR_OK;
}

int cmdPutBuzzer(Msg& msg)
{
    return ERR_OK;
}

int cmdPutServos(Msg& msg)
{
    return ERR_OK;
}

int cmdGetUltrasonic(Msg& msg)
{
    return ERR_OK;
}

int cmdGetGas(Msg& msg)
{
    return ERR_OK;
}

int cmdGetPhotoRes(Msg& msg)
{
    return ERR_OK;
}

int cmdGetTemp(Msg& msg)
{
    return ERR_OK;
}

int cmdGetIRSensor(Msg& msg)
{
    return ERR_OK;
}

int cmdGetVarRes(Msg& msg)
{
    return ERR_OK;
}

int cmdGetButState(Msg& msg)
{
    return ERR_OK;
}

int cmdPutLcd(Msg& msg)
{
    return ERR_OK;
}
