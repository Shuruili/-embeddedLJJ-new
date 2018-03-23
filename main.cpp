#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//golbal variables
int8_t intState =0;
int8_t orState =0;
Thread commOutT(osPriorityNormal,1024);
Thread commIn(osPriorityNormal,1024);
Thread motorCtrlT(osPriorityNormal,1024);
volatile uint64_t newKey;
Mutex newKey_mutex;
Mutex velocity_mutex;
const int period = 2000;
uint32_t pulse_width = 1700;
int32_t motorPosition = 0;
int32_t set_motorPosition = 0;
float set_rotation;
int32_t E_r = 0;
int32_t E_r_temp = 0;
int32_t E_r_diff;
int8_t sgn = 1;
int32_t motorPower;
int32_t velocity = 0;
int8_t velocity_iter = 0;
float set_v = 100*6;
float ys;
float yr;
float y;
bool limit = true;

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Initialise buffer for incoming characters
Queue<void,8> inCharQ;

//Construct data type message_t
typedef struct{
    uint8_t code;
    uint32_t data;
    } message_t;
    
//Initialise mailing between threads
Mail<message_t,16> outMessages;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Set a given drive state
void motorOut(int8_t driveState, uint32_t pulse_width){
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(pulse_width);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(pulse_width);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(pulse_width);
    if (driveOut & 0x20) L3H = 0; 
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0,1700);
    wait(2.0);
    //Get the rotor state
    return readRotorState();
}

//Declare and initialise the input sequence and hash
uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)((int)sequence + 48);
uint64_t* nonce = (uint64_t*)((int)sequence + 56);
uint8_t hash[32];

//put Message function
void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
}

void commOutFn(){
    while(1){
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        if(pMessage->code == 3){
            int32_t v = int(pMessage->data)/6;
            pc.printf("Message %d with velocity %d revolutions per seconds\n\r",pMessage->code, v);
        }else if(pMessage->code == 4){
            int32_t p = int(pMessage->data)/6;
            pc.printf("Message %d with motor position at %d revolutions\n\r",pMessage->code, p);
        }else{
            pc.printf("Message %d with data 0x%016x\n\r",pMessage->code, pMessage->data);
        }
        outMessages.free(pMessage);
    }
}

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

void dec_cmdFn(){
    pc.attach(&serialISR);
    char newCmd [18] = {};
    uint8_t i = 0;
    while(1){
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        newCmd[i] = newChar;
        pc.printf("%c",newChar);
        i++;
        if(i > 17){
            printf("command is too long");
            i = 0;
        }
        else if(newChar == '\r'){
            newCmd[i-1] = '\0';
            pc.printf("cmd = %s\n\r", newCmd);
            i = 0;
            if (newCmd[0] == 'K'){
                newKey_mutex.lock();
                sscanf(newCmd,"K%x",&newKey);
                newKey_mutex.unlock();
            }
            else if(newCmd[0] == 'V'){
                sscanf(newCmd,"V%f",&set_v);
                set_v = set_v * 6 + 10*6;           //ADDED A FIXED CONSTANT AT THE END DUE TO MOTOR POWER NOT ENNOUGH
                if(set_v != 0){
                    limit = true;
                }
                else{
                    limit = false;
                }
            }
            else if(newCmd[0] == 'R'){
                sscanf(newCmd,"R%f",&set_rotation);
                set_motorPosition = motorPosition + set_rotation*6 + 6*6; //ADDED A FIXED CONSTANT AT THE END DUE TO MOTOR DEFICIT
                int8_t cs = readRotorState();
                if(set_rotation >= 0){
                    motorOut((cs-orState+1+6)%6,pulse_width);
                }else{
                    motorOut((cs-orState-1+6)%6,pulse_width);
                }
            }
        }
    }
}

void motorISR(){
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState();
    motorPower = int(y);
    if(limit == false){
        motorOut((rotorState-orState+lead+6)%6,pulse_width);
    }else{
        if(abs(motorPower)>1700) motorPower = 1700;        //1700 BECAUSE PULSE WIDTH OF 1000 IS TOO LOW FOR OUR MOTOR TO ROTATE AT A HIGH SPEED
        motorOut((rotorState-orState+lead+6)%6,abs(motorPower));
    }
    if(rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
}

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

void select_y(){
    if(velocity < 0){
        if(ys > yr){
            y = ys;
        }else{
            y = yr;
        }
    }else{
        if(ys < yr){
            y = ys;
        }else{
            y = yr;
        }
    }
}

void motorCtrlFn(){
    Ticker motorCtrlTicker;
    int32_t Old_motorPosition = 0;
    int8_t kp = 21;
    int8_t kd = 18;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    while(1){
        motorCtrlT.signal_wait(0x1);
        int32_t temp = motorPosition;
        velocity_mutex.lock();
        velocity = (temp - Old_motorPosition)*10;
        ys = kp*(set_v - abs(velocity))*sgn;
        velocity_mutex.lock();
        E_r = set_motorPosition - temp;
        if(E_r >= 0){
            sgn = 1;
        }else{
            sgn = -1;
        }
        E_r_diff = (E_r - E_r_temp)*10;
        yr = kp*E_r + kd*E_r_diff;
        select_y();
        if(y >= 0){
            lead = 2;
        }else{
            lead = -2;
        }
        Old_motorPosition = temp;
        E_r_temp = E_r;
        velocity_iter++;
    }
}
    
//Main
int main() {
    commOutT.start(commOutFn);
    commIn.start(dec_cmdFn);
    motorCtrlT.start(motorCtrlFn);
    SHA256 SHA;
    uint8_t* hash_pointer;
    uint8_t* input_pointer;
    hash_pointer = hash;
    input_pointer = sequence;
    Timer time;
    int counter, hash_count;
    
    L1L.period_us(period);
    L2L.period_us(period);
    L3L.period_us(period);
    
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    I1.fall(&motorISR);
    I2.rise(&motorISR);
    I3.fall(&motorISR);
    I1.rise(&motorISR);
    I2.fall(&motorISR);
    I3.rise(&motorISR);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        SHA.computeHash(hash_pointer,input_pointer,64);
        hash_count++;
        (*nonce)++;
 
        if(hash[0] == 0 && hash[1] == 0){
            pc.printf("\r");
            putMessage(1,*nonce);
            putMessage(2,*key);
        }

        if(velocity_iter >= 10){
            putMessage(3,velocity);
            putMessage(4,motorPosition);
            velocity_iter = 0;
        }
    }
}
