/*****************************************************************************************************

                                        Assigment 3 Embedded Software
                                                Markus 
                                                 



                    This Software is a Car Control Software as no Real Car can be connected it Simulates
                    a car as well.
                    For the Display it uses an LCD Screen
                    For the Speedometer it uses an Servo
                    The Emulation and Synchronisation is implemented using TimerThreads and Semaphores
                    for the reason of existing thread limits, conventional timers are helping to slicing
                    timeslots

@version 1.4
@updateDate 30.03.2016
@author Markus

******************************************************************************************************/

#include "mbed.h"
#include "MCP23017.h"                                               // include 16-bit parallel I/O header file
#include "WattBob_TextLCD.h"                                        // include 2*16 character display header file
#include "rtos.h"
#include "Servo.h"
#include <deque>


//Unfortunately the definition doesnt work, either here or anywhere else...
//#define OS_TIMERCBQS 20 // define a new number of timers we want +1 timer!
//#define OS_TASKCNT 20 // maximum threads


MCP23017            *par_port;                                      // pointer to 16-bit parallel I/O object
WattBob_TextLCD     *lcd;                                           // pointer to 2*16 chacater LCD object
Serial              serpc(USBTX, USBRX);                            // serial usb connection tx, rx

AnalogIn AinBreak(p15);                                             // Port for the Break Value
AnalogIn AinAccel(p16);                                             // Port for the Accelerator Value

DigitalIn DinSwitchEngine(p11);                                     // Port for the Engine Switch
DigitalIn DinSwitchLight(p12);                                      // Port for the Light Switch
DigitalIn DinSwitchRindic(p13);                                     // Port for the Right Indicator
DigitalIn DinSwitchLindic(p14);                                     // Port for the Left Indicator

Servo Odometer(p26);
DigitalOut LEDSpeedWarning(p8);

DigitalOut DoutLEDLight(LED1);                                      // Output Port for LED1
DigitalOut DoutLEDLeft(LED2);                                       // Output Port for LED2
DigitalOut DoutLEDRight(LED3);                                      // Output Port for LED3
DigitalOut DoutLEDEngine(LED4);                                     // Output Port for LED4


void Timer1_void(void const *args);                                 // Timer 1
void Timer2_void(void const *args);                                 // Timer 2
void Timer3_void(void const *args);                                 // Timer 3


                                                                    // Task Functions
void task1_break_accelerate();
void task2_read_show_engine_state();
void task3_show_odometer();
void task4_speed_warning();
void task5_update_odometer();
void task6_fill_mail_queue();
void task7_dump_mail_to_serial();
void task8_read_single_side_light();
void task9_read_indicators();
void task10_calc_avg_speed();
void task11_emulate_car();


                                                                    // Init Variables
int Convert_Hz_to_Ms(double Hz);

float accerlator(0);
float speed(0);
float avgSpeed(0);
float brake(0);
float dist(0);

bool engine(0);

bool indicator_L(1);
bool indicator_R(1);

bool sw_timer1(0);
bool sw_timer11(0);
bool sw_timer2(0);
bool sw_timer21(0);
int sw_timer3(4);                                                   // sw_timer3 initalize with a first run

std::deque<float> AvgSpeedDB;                                       // used for storing the average speed

                                                        
Semaphore SemAvgSpeedDB(1);                                         // declare used Semaphores
Semaphore SemAvgSpeed(1);
Semaphore SemSpeed(1);
Semaphore SemBreak_Accelerate(1);
Semaphore SemDistance(1);
Semaphore SemEngine(1);
Semaphore SemMailCnT(1);

typedef struct {
    float    speed;
    float    accel;
    float brake;
} mail_t;

int mailcounter(0);                                                 // counts the mails in the queue
Mail<mail_t, 100> mail_box;                                         // the mail queue has a maximum size of 100 mails

int main()
{
    // 20.00 Hz = 00050 ms
    // 10.00 Hz = 00100 ms
    // 05.00 Hz = 00200 ms
    // 02.00 Hz = 00500 ms
    // 01.00 Hz = 01000 ms
    // 00.50 Hz = 02000 ms
    // 00.20 Hz = 05000 ms
    // 00.05 Hz = 20000 ms

    serpc.baud(19200);                                              // setup the bautrate
    serpc.printf("Init Software\r\n");
    par_port = new MCP23017(p9, p10, 0x40);                         // initialise 16-bit I/O chip (0x40 = 64)
    lcd = new WattBob_TextLCD(par_port);                            // initialise 2*26 char display
    par_port->write_bit(1,BL_BIT);                                  // turn LCD backlight ON
    lcd->cls();                                                     // clear display
    lcd->locate(0,0);                                               // set cursor to location (0,0) - top left corner

    RtosTimer Timer1(Timer1_void,osTimerPeriodic,(void *)NULL);     // create the necesarry timers to overcome a thread issue (max threads)
    Timer1.start(Convert_Hz_to_Ms(20.0));

    RtosTimer Timer2(Timer2_void,osTimerPeriodic,(void *)NULL);
    Timer2.start(Convert_Hz_to_Ms(2.0));

    RtosTimer Timer3(Timer3_void,osTimerPeriodic,(void *)NULL);
    Timer3.start(Convert_Hz_to_Ms(0.2));

    Thread::wait(osWaitForever);

}

/*
##############################################################
Timer 1 runs at 20 Hz, but starts tasks at 20 Hz, 10 Hz, 5 Hz
    task11_emulate_car();
    task1_break_accelerate();
    task10_calc_avg_speed();
#############################################################
*/

void Timer1_void(void const *args)
{
    task11_emulate_car();                                           // runs every time, so at 20 Hz
    sw_timer1 = !sw_timer1;
    if(sw_timer1) {                                                 // runs just every second time, so at 10 hz
        task1_break_accelerate();
        sw_timer11 = !sw_timer11;
        if(sw_timer11) {                                            // runs just every fourth time, so at 5 hz
            task10_calc_avg_speed();
        }
    }
}


/*
##############################################################
Timer 2 runs at 2 Hz, but starts tasks at 2 Hz, 1 Hz, 0.5 Hz

    task2_read_show_engine_state();
    task5_update_odometer();
    Updates Indicators

    task3_show_odometer();
    task8_read_single_side_light();

    task4_speed_warning();
    task9_read_indicators();
#############################################################
*/
void Timer2_void(void const *args) // timer runs at 2 hz
{
    task2_read_show_engine_state();
    task5_update_odometer();
    sw_timer2 = !sw_timer2;

    if(indicator_L && indicator_R ) {                               // Sets the Left and Right Inidcators
        DoutLEDLeft=!DoutLEDRight;                                  // needs to get the inverted status of led1 before led1 is changed
        DoutLEDRight=!DoutLEDRight;
    } else if (!indicator_R && !indicator_L) {
        DoutLEDLeft=0;
        DoutLEDRight=0;
    }

    if(sw_timer2) {                                                 // runs just every second time, so at 1 hz
        task3_show_odometer();
        task8_read_single_side_light();
        sw_timer21 = !sw_timer21;

        if (!indicator_R && indicator_L) {                          // switch the left / right indicator
            DoutLEDRight=0;
            DoutLEDLeft=!DoutLEDLeft;
        } else if(indicator_R && !indicator_L) {
            DoutLEDRight=!DoutLEDRight;
            DoutLEDLeft=0;
        }


        if(sw_timer21) {                                            // runs just every second time, so at 0.5 hz
            task4_speed_warning();
            task9_read_indicators();
        }
    }

}


/*
##############################################################
Timer 3 runs at 0.2 Hz, but starts tasks at 0.2 Hz and 0.05 Hz
    task6_fill_mail_queue();
    task7_dump_mail_to_serial();
##############################################################
*/
void Timer3_void(void const *args)                                  // timer runs at 0.2 hz
{
    task6_fill_mail_queue();
    if((sw_timer3%4)==0) {                                          // task runs at 0.05 Hz
        task7_dump_mail_to_serial();                                // dump the queue to serial
        sw_timer3=0;                                                // reset the timer
    }
    sw_timer3++;
}


/*
Reads the brake / acceleration of the car
*/
void task1_break_accelerate()
{
                                                                        // Let the Semaphores wait
    SemBreak_Accelerate.wait();
    
    accerlator = AinAccel;                                              // save the accerlator value
    brake = AinBreak;                                                   // save the brake value
                                                                        // Let the Semaphores release
    SemBreak_Accelerate.release();
}


/*
Reads the Engine On/Off Switch and displays its state
*/
void task2_read_show_engine_state()
{
                                                                        // Let the Semaphores wait
    SemEngine.wait();
    
    engine = DinSwitchEngine;                                           // read the engine state
    DoutLEDEngine = engine;                                             // write the engine state
                                                                        // Let the Semaphores release
    SemEngine.release();
}


/*
Updates the Odometer (Servo Motor)
*/
void task3_show_odometer()
{
                                                                        // Let the Semaphores wait
    SemAvgSpeed.wait();                                                 
    Odometer = avgSpeed/250.0;                                          // Calculate the odometer
                                                                        // Let the Semaphores release
    SemAvgSpeed.release();
}


/*
Indicates a Speed warning at 75 Mph
*/
void task4_speed_warning()
{
                                                                        // Let the Semaphores wait
    SemAvgSpeed.wait();
    if(avgSpeed>75.0)                                                   // check our speed
        LEDSpeedWarning = !LEDSpeedWarning;                             // and switch the Warning on/off
    else
        LEDSpeedWarning = 0;
                                                                         // Let the Semaphores release
    SemAvgSpeed.release();
}


/*
Updates the LCD Display
*/
void task5_update_odometer()
{
                                                                    // Let the Semaphores wait
    SemDistance.wait();
    SemAvgSpeed.wait();

    lcd->locate(0,0);                                               // set cursor to location (0,0) - top left corner
    lcd->printf("s: %5.0f",avgSpeed);
    lcd->locate(1,0);
    lcd->printf("d: %5.0f",dist);
                                                                    // Let the Semaphores release
    SemDistance.release();
    SemAvgSpeed.release();
}



/*
Reads the Left and Right Inidcator
*/
void task6_fill_mail_queue()
{
                                                                                // Let the Semaphores wait
    SemMailCnT.wait();
    SemBreak_Accelerate.wait();
    SemSpeed.wait();

    mail_t *mail = mail_box.alloc();                                            // reserve the space for our new message
    mail->speed = speed;                                                        // fill with values
    mail->accel = accerlator;
    mail->brake = brake;
    mail_box.put(mail);                                                         // put the new message into the mail queue
    mailcounter++;

                                                                                // Let the Semaphores release
    SemBreak_Accelerate.release();
    SemSpeed.release();
    SemMailCnT.release();
}

/*
Reads the Mail Queue and Sends the Content to the Serial Port
*/
void task7_dump_mail_to_serial()
{
                                                                                // Let the Semaphores wait
    SemMailCnT.wait();

    while(mailcounter) {                                                        // as long as we got mail

        osEvent evt = mail_box.get();                                           // we are getting them
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;                                // print the mail to serial
            serpc.printf("\nspeed: %.0f \n\r"   , mail->speed);
            serpc.printf("accerlator: %.2f\n\r"     , mail->accel);
            serpc.printf("brake: %.2f\n\r", mail->brake);
            mail_box.free(mail);                                                // clear up the mailbox
        }
        mailcounter--;
    }

                                                                                // Release the Semaphores
    SemMailCnT.release();
}

/*
Single Side Light
*/
void task8_read_single_side_light()
{
    DoutLEDLight = DinSwitchLight;                                             // Reading the value
}


/*
Reads the Left and Right Inidcator
*/
void task9_read_indicators()
{
    indicator_R = DinSwitchRindic;                                            // Reading the value
    indicator_L = DinSwitchLindic;                                            // Reading the value
}


/*
Calculates the Average Speed
*/
void task10_calc_avg_speed()
{

                                                                              // Let the Semaphores wait
    SemAvgSpeed.wait();
    SemAvgSpeedDB.wait();

    float sum(0);
    for(deque<float>::const_iterator i = AvgSpeedDB.begin(); i != AvgSpeedDB.end(); ++i)
        sum+= *i;                                                             // calculate the average by iterating over the queue
    avgSpeed = sum/AvgSpeedDB.size();

                                                                              // Release the Semaphores
    SemAvgSpeedDB.release();
    SemAvgSpeed.release();
}


/*
Emulates the car
*/
void task11_emulate_car()
{
                                                                              // Let the Semaphores wait
    SemAvgSpeed.wait();
    SemAvgSpeedDB.wait();
    SemDistance.wait();
    SemBreak_Accelerate.wait();
    SemSpeed.wait();
    SemEngine.wait();

    if(accerlator<=brake || !engine)                                        // are we braking more than accelerating? is the engine on?
        speed = 0;
    else
        speed = (accerlator-brake) *0.5 +speed;
    if(speed>250)
        speed=250;                                                           // maximum speed
    if(AvgSpeedDB.size()>=4)                                                 // if we already got 4 values, we have to
        AvgSpeedDB.pop_front();                                              // make space by deleting the oldest value
    AvgSpeedDB.push_back(speed);                                             // safe a new reading

    dist += speed * 1.0/20.0;                                                // runs at 20 Hz so we have to take this into account

                                                                             // Release the Semaphores
    SemDistance.release();
    SemAvgSpeed.release();
    SemAvgSpeedDB.release();
    SemBreak_Accelerate.release();
    SemSpeed.release();
    SemEngine.release();
}



/*
Function used for converting Hz to Ms for a Steps
*/
int Convert_Hz_to_Ms(double Hz)
{
    return 1000.0 / Hz;
}

