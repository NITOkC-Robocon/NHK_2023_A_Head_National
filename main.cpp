#include "mbed.h"
#include "RotaryEncoder.h"
#include "DriveMotor.h"

#define NECK_RX_MAX 1722
#define NECK_RX_MIN 1338
#define NECK_RX_ACCELERATION 0.2
#define NECK_RY_THRESHOLD 15
#define NECK_RY_MAXSPEED 0.85
#define NECK_RY_ACCELERATION 0.4
#define NECK_RZ_THRESHOLD 25
#define NECK_RZ_MAXSPEED 0.85
#define NECK_RZ_ACCELERATION 0.8
#define CHIN_MAX 1900
#define CHIN_MIN 1360
#define CHIN_ACCELERATION 0.2

RawSerial PC(USBTX, USBRX);
RawSerial XBee(PC_10, PC_11, 230400);

SPISlave SPI_main(PA_7, PA_6, PA_5, PA_4);

DriveMotor M_neckry(PC_6, PC_8);
DriveMotor M_neckrz(PB_8, PC_9);

RotaryEncoder RE_neckry(PA_14, PA_13);
RotaryEncoder RE_neckrz(PC_3, PC_2);

PwmOut S_neckrx(PA_9);
PwmOut S_chin(PA_8);

void initRobot(void);
void driveNeckRX(void);
void driveNeckRY(void);
void driveNeckRZ(void);
void driveChin(void);
inline void receiveSignal(void);

int extended_sign_pool = 0x00;
int chin_sign_pool = 0x00;
int neck_rx_sign_pool = 0x00;
int neck_ry_sign_pool = 0x00;
int neck_rz_sign_pool = 0x00;

int extended_sign = 0x00;
int chin_sign = 0x00;
int neck_rx_sign = 0x80;
int neck_ry_sign = 0x00;
int neck_rz_sign = 0x80;
int check_sum_sign = 0x00;
int reply_sign = 0x0000;

int check_sum_correct = 0;

// main() runs in its own thread in the OS
int main()
{
    initRobot();
    while (true) {
        // PC.printf("\033[Hneck-ry%5d\r\nneck-rz%5d\r\n", RE_neckry.Get_Count(), RE_neckrz.Get_Count());
        // PC.printf("\033[Hextended_sign_pool: %#04X\r\nchin_sign_pool    : %#04X\r\nneck_rx_sign_pool : %#04X\r\nneck_ry_sign_pool : %#04X\r\nneck_rz_sign_pool : %#04X", extended_sign, chin_sign_pool, neck_rx_sign_pool, neck_ry_sign_pool, neck_rz_sign_pool);
        driveNeckRX();
        driveNeckRY();
        driveNeckRZ();
        driveChin();

        if(SPI_main.receive()) {
            if(SPI_main.read() == 0xFF00) {
                if(check_sum_correct) {
                    SPI_main.reply(0x00FF);
                }
                else {
                    SPI_main.reply(0x0000);
                }
            }
        }        
    }
}

void initRobot(void){    
    XBee.attach(&receiveSignal, SerialBase::RxIrq);
    M_neckry.setPeriod_us(500);
    M_neckrz.setPeriod_us(500);
    S_chin.period_ms(20);
    S_neckrx.period_ms(20);
    SPI_main.format(16, 1);
    SPI_main.frequency(20000000);
}

void driveNeckRX(void) {
    static int neck_rx_pulse_us_now = 1472;
    int neck_rx_pulse_us;

    neck_rx_pulse_us = (int)(NECK_RX_MAX - neck_rx_sign * 1.5);
    if(neck_rx_pulse_us < NECK_RX_MIN) neck_rx_pulse_us = NECK_RX_MIN;

    neck_rx_pulse_us_now += (int)((neck_rx_pulse_us - neck_rx_pulse_us_now) * NECK_RX_ACCELERATION);
    S_neckrx.pulsewidth_us(neck_rx_pulse_us_now);

    // PC.printf("%d\r\n", RE_neckry.Get_Count());
}

void driveNeckRY(void) {
    int target_count = - neck_ry_sign * 2;
    double current_count = RE_neckry.Get_Count();
    double current_speed = M_neckry.read();
    double target_speed;
    
    if(target_count - NECK_RY_THRESHOLD > current_count) {
        target_speed = (target_count - current_count) / 30;
        if(target_speed > NECK_RY_MAXSPEED) {
            target_speed = NECK_RY_MAXSPEED;
        }
    }
    else if(target_count + NECK_RZ_THRESHOLD < current_count) {
        target_speed = (target_count - current_count) / 30;
        if(target_speed < -NECK_RY_MAXSPEED) {
            target_speed = -NECK_RY_MAXSPEED;
        }
    }
    else target_speed = 0;

    current_speed += (target_speed - current_speed) * NECK_RY_ACCELERATION;
    M_neckry.drive(current_speed); 
}

void driveNeckRZ(void) {
    int target_count = (int)((neck_rz_sign - 0x80) * 1.34);
    double current_count = -RE_neckrz.Get_Count();
    double current_speed = M_neckrz.read();
    double target_speed;
    
    if(target_count - NECK_RZ_THRESHOLD > current_count) {
        target_speed = (target_count - current_count) / 50;
        if(target_speed > NECK_RZ_MAXSPEED) {
            target_speed = NECK_RZ_MAXSPEED;
        }
    }
    else if(target_count + NECK_RZ_THRESHOLD < current_count) {
        target_speed = (target_count - current_count) / 50;
        if(target_speed < -NECK_RZ_MAXSPEED) {
            target_speed = -NECK_RZ_MAXSPEED;
        }
    }
    else target_speed = 0;

    current_speed += (target_speed - current_speed) * NECK_RZ_ACCELERATION;
    // PC.printf("target :%4d\r\nM_neckrz:%5.2lf\r\n", target_count, current_speed);
    M_neckrz.drive(current_speed); 
}

void driveChin(void) {
    static int chin_pulse_us_now = 1530;
    int chin_pulse_us;
    
    chin_pulse_us = (int)(CHIN_MIN + chin_sign * 2.5);
    if(chin_pulse_us > CHIN_MAX) chin_pulse_us = CHIN_MAX;

    chin_pulse_us_now += (int)((chin_pulse_us - chin_pulse_us_now) * CHIN_ACCELERATION);
    
    S_chin.pulsewidth_us(chin_pulse_us_now);
    // PC.printf("neck-ry:%4d\r\n",chin_pulse_us_now);
}

inline void receiveSignal(void) {
    int read_sign = XBee.getc();
    static int octets = 0;
    static int check_sum = 0x00;

    if(read_sign == 0xFF) {
        octets = 0;
    }
    if (read_sign >= 0){
        switch(octets) {
            case 0: check_sum = 0x00; ++octets; break;
            case 1: extended_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 2: chin_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 3: neck_ry_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 4: neck_rx_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 5: neck_rz_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 6: 
                check_sum_sign = read_sign;
                if(((check_sum % 0x100) == read_sign) || ((check_sum == 0xFF) && (read_sign == 0xFE))) {
                    check_sum_correct = 1;
                    extended_sign = extended_sign_pool;
                    int encoder_lock = (extended_sign >> 1) % 0b10;
                    if(!encoder_lock) {
                        chin_sign = chin_sign_pool;
                        neck_rx_sign = neck_rx_sign_pool;
                        neck_ry_sign = neck_ry_sign_pool;
                        neck_rz_sign = neck_rz_sign_pool;
                    }
                }
                else {
                    check_sum_correct = 0;
                }
                octets = 0; 
                break;
        }
    }  
}
