#include "mbed.h"
#include "C12832.h"
typedef enum {initialisation, setting_time, current_time, world_time, stopwatch, countdown_timer} ProgramState;
ProgramState state = initialisation;
class RGBLED {
private:
    DigitalOut red, green, blue;

public:
    RGBLED(PinName redPin, PinName greenPin, PinName bluePin) : red(redPin), green(greenPin), blue(bluePin) {Off();}

    void Red() {
        red = 0;
        green = 1;
        blue = 1;
    }

    void Yellow() {
        red = 0;
        green = 0;
        blue = 1;
    }

    void Blue() {
        red = 1;
        green = 1;
        blue = 0;
    }
    void Green(){
        red = 1;
        green = 0;
        blue = 1;
    }

    void Off() {
        red = 1;
        green = 1;
        blue = 1;
    }
    void toggle_green(){
        green = !green;
    }
    void stop_green(){
        green = 1;
    }
};
RGBLED led(D5,D9,D8);
class Potentiometer                                 //Begin Potentiometer class definition
{
private:                                            //Private data member declaration
    AnalogIn inputSignal;                           //Declaration of AnalogIn object
    float VDD, currentSampleNorm, currentSampleVolts; //Float variables to speficy the value of VDD and most recent samples

public:                                             // Public declarations
    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {}   //Constructor - user provided pin name assigned to AnalogIn...
                                                                        //VDD is also provided to determine maximum measurable voltage
    float amplitudeVolts(void)                      //Public member function to measure the amplitude in volts
    {
        return (inputSignal.read()*VDD);            //Scales the 0.0-1.0 value by VDD to read the input in volts
    }
    
    float amplitudeNorm(void)                       //Public member function to measure the normalised amplitude
    {
        return inputSignal.read();                  //Returns the ADC value normalised to range 0.0 - 1.0
    }
    
    void sample(void)                               //Public member function to sample an analogue voltage
    {
        currentSampleNorm = inputSignal.read();       //Stores the current ADC value to the class's data member for normalised values (0.0 - 1.0)
        currentSampleVolts = currentSampleNorm * VDD; //Converts the normalised value to the equivalent voltage (0.0 - 3.3 V) and stores this information
    }
    
    float getCurrentSampleVolts(void)               //Public member function to return the most recent sample from the potentiometer (in volts)
    {
        return currentSampleVolts;                  //Return the contents of the data member currentSampleVolts
    }
    
    float getCurrentSampleNorm(void)                //Public member function to return the most recent sample from the potentiometer (normalised)
    {
        return currentSampleNorm;                   //Return the contents of the data member currentSampleNorm  
    }

};

class SamplingPotentiometer : public Potentiometer 
{ 
private: 
    float samplingFrequency, samplingPeriod;
    Ticker sampler; 
public: 
    SamplingPotentiometer(PinName p, float v, float fs) : Potentiometer(p,v),samplingFrequency(fs){
        samplingPeriod = 1.0f / fs;
        sampler.attach(callback(this, &SamplingPotentiometer::sample), samplingPeriod);
    };


};

class Speaker {
private:
    PwmOut outputSignal;
    Ticker speakerTicker;
    bool isOn;

    void toggleSpeaker() {
        if (outputSignal == 0){
            outputSignal = 0.5; 
    }
        else{
            outputSignal = 0.5;
        }
    }

public:
    Speaker(PinName pin) : outputSignal(pin) {
        outputSignal.period(1.0 / 3000.0); // 设置PWM周期以产生3000Hz频率
        outputSignal = 0.0; // 初始时关闭扬声器
        isOn = false;
    }

    void on() {
        if (!isOn) {
            speakerTicker.attach(callback(this, &Speaker::toggleSpeaker), 1.0 / 3000.0);//Turn on Ticker
            isOn = true;
        }
    }

    void off() {
        if (isOn) {
            speakerTicker.detach(); // Turn off Ticker
            outputSignal = 0;
            isOn = false;
        }
    }

    bool getState() const {
        return isOn;
    }
};

Speaker speaker(D6);


class clocking
{
private:
    int time_hour, time_min, time_sec, addhour, addmin, addsec;
    Ticker ticker;
public:
    clocking(int h, int m, int s) : time_hour(h),time_min(m),time_sec(s){}
    static void tick(clocking* instance) {
        instance->countup(); // or instance->countdown();
    }
    void reset(){
        time_hour = 0;
        time_min = 0;
        time_sec = 0;
    }
    void addtime(int adh,int adm){
        addhour = adh + time_hour;
        addmin = adm + time_min;
        if (addhour >= 24) {
            addhour = addhour - 24;
        } else if (addhour < 0){
            addhour = addhour + 24;
        };
        if (addmin >= 60){
            addmin = addmin - 60;
        } else if (addmin < 0) {
            addmin = addmin + 60;
        }
    }
    void settime(int hou,int min, int sec){
        time_hour = hou;
        time_min = min;
        time_sec = sec;
    }
    void startTicker(float interval) {
        ticker.attach(callback(this,&clocking::tick), interval);
    }

    void stopTicker() {
        ticker.detach();
    }
    void countup(void){
        time_sec = time_sec + 1;
        if (time_sec >= 60) {
            time_sec = 0;
            time_min = time_min + 1;
            if (time_min >= 60) {
                time_min = 0;
                time_hour = time_hour + 1;
                if (time_hour >= 24) {
                    time_hour = 0;
                }
            }
        }
    }
    void countdown(void){
        time_sec= time_sec - 1;
        if (time_sec < 0) {
            time_sec = 59;
            time_min = time_min - 1;
            if (time_min < 0) {
                time_min = 59;
                time_hour = time_hour - 1;
                if (time_hour < 0) {
                    time_hour = 23;
                }
            }
        }

    }
    int getaddhour(void){
        return addhour;
    }
    int getaddmin(void){
        return addmin;
    }
    int getseconds(void){
        return time_sec;
    }
    int getmins(void){
        return time_min;
    }
    int gethours(void){
        return time_hour;
    }
    int gettotalsecs(void){
        return time_hour*3600 + time_min*60 + time_sec;
    }
};

Timer timer;
Ticker grticker;
typedef enum {check, set} timeset;
timeset state_timeset = check;
typedef enum {Taulaga, Sitka,SanFrancisco, Colorado, NewYork, Palmas, Lagoa, Manchester, Berlin,Cairo,Tehran, Munbai, Beijing, Sydney, Alo} area;
area state_area = Manchester;
typedef enum{begin, stop} watchstop;
watchstop state_watch = begin;
typedef enum {setPeriod, countdownbegin, countdownover} downtimer;
downtimer state_downtimer = setPeriod;
void fireISR() {

    //static bool downtimestate = 0;
    //bool stopwatchstate = 1;

    switch (state) {
        case (setting_time):
            switch (state_timeset) {
                case (check):
                    state_timeset = set;
                    break;
                case (set):
                    state_timeset = check;
                    break;
            }
            break;

        case (stopwatch):
            switch (state_watch) {
                case (begin) :
                    if (timer.read() == 0) {
                        timer.start();
                        led.Blue();
                    } else {
                        timer.stop();
                        state_watch = stop;
                        led.Off();
                    }
                    break;
                case(stop) :
                    timer.reset();
                    state_watch = begin;
                    break;
            }
            break;

        case (countdown_timer):
            switch (state_downtimer) {
                case setPeriod:
                    state_downtimer = countdownbegin;
                    grticker.detach();
                    led.stop_green();
                    break;
                case countdownbegin:
                    state_downtimer = setPeriod;
                    break;
                case countdownover:
                    state_downtimer = setPeriod;
                    speaker.off();
                    led.stop_green();
                default:
                    state_downtimer = setPeriod;
                    break;
            }
            break;

    }
}
void upISR() 
{

 switch (state) {                       //This statement determines which state to advance to, depending on the current program state.      

        case (initialisation) :
            state = setting_time;
            break;

        case (setting_time) :
            state = countdown_timer;
            break;

        case (countdown_timer) :
            state = stopwatch;
            break;
        
        case (stopwatch) :
            state = world_time;
            break;

        case (world_time):
            state = current_time;
            break;

        case (current_time) :
            state = setting_time;
            break;

             //Specifices a default state in case of error
 }

}
void downISR()
{  

 switch (state) {                       //This statement determines which state to advance to, depending on the current program state.      

        case (initialisation) :
            state = setting_time;
            break;

        case (setting_time) :
            state = current_time;
            break;

        case (current_time) :
            state = world_time;
            break;
        
        case (world_time) :
            state = stopwatch;
            break;

        case (stopwatch):
            state = countdown_timer;
            break;

        case (countdown_timer) :
            state = setting_time;
            break;

             //Specifices a default state in case of error
 }

}



int main() {
    C12832 lcd(D11,D13,D12,D7,D10);
    InterruptIn fire(D4),up(A2),down(A3);  
    SamplingPotentiometer sp1(A0,23.2,20);
    SamplingPotentiometer sp2(A1,59.2,20);
    SamplingPotentiometer cty(A0,14.2,20);
    SamplingPotentiometer setdownmin(A0,60.2,20);
    SamplingPotentiometer setdownsec(A1,60.2,20);
    int hour = 0,min = 0;
    int country = 0;
    int settingmin = 0, settingsec = 0,totalsec = 0,setdowntotalsec = 0;
    timer.reset();
    clocking settimeparameter(hour, min, 0);
    clocking cd_clock(0,settingmin,settingsec);
    up.rise(&upISR);
    down.rise(&downISR);
    fire.rise(&fireISR);

    settimeparameter.startTicker(1.0);
    while(1) {
        country = cty.getCurrentSampleVolts();
        state_area = static_cast<area>(country);
        totalsec =  cd_clock.getmins() * 60 + cd_clock.getseconds();
        settingmin = setdownmin.getCurrentSampleVolts();
        settingsec = setdownsec.getCurrentSampleVolts();
        
        if(totalsec > 0){
            if (state_downtimer == countdownbegin) {
                if (totalsec == setdowntotalsec){
                    cd_clock.countdown();
                }
                else {
                    cd_clock.countdown();
                    wait(1.0);
                }
                
            
                
            }
        }
        else  { 
            if (state_downtimer != setPeriod){
            speaker.on();
            grticker.detach();
            led.Green();
            state = countdown_timer;
            state_downtimer = countdownover;

            lcd.locate(0,0);
            lcd.printf("Time Period Elapsed         ");
            lcd.locate(0,9);
            lcd.printf("                           ");
            lcd.locate(0,18);
            lcd.printf("                           ");
            }  
        }
    
        switch (state) {
            lcd.locate(0,0);
            lcd.printf("                          ");
            lcd.locate(0, 9);
            lcd.printf("                           ");
            lcd.locate(0, 18);
            lcd.printf("                            ");
            case (initialisation) :                 //Initialisation state
                state = setting_time;
                break;                              //Exit switch statement
            case (setting_time):
                switch (state_timeset) {
                    case check:
                        lcd.locate(0, 0);
                        lcd.printf("Press Fire to set time           ");
                        lcd.locate(0, 9);
                        lcd.printf(     "%02d:%02d:%02d           ",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf("                            ");
                        break;
                    case set:
                        hour = sp1.getCurrentSampleVolts();
                        min = sp2.getCurrentSampleVolts();
                        lcd.locate(0, 0);
                        lcd.printf("Set new time (HH:MM)           ");
                        lcd.locate(0, 9);
                        lcd.printf(     "%02d:%02d               ",hour,min);
                        settimeparameter.settime(hour, min , 0);
                        lcd.locate(0, 18);
                        lcd.printf("                            ");
                        break;
                    default:
                        state_timeset = check;
                }
                break;
            case (current_time) :
                lcd.locate(0, 0);
                lcd.printf("Current Time                  ");
                lcd.locate(0,9);
                lcd.printf(     "%02d:%02d:%02d        ",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                lcd.locate(0, 18);
                lcd.printf("                            ");          
                break;
            case (world_time):
                switch (state_area) {
                    case Taulaga:
                        lcd.locate(0,0);
                        lcd.printf("Taulaga      (GMT-11)   ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-11,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Sitka:
                        lcd.locate(0,0);
                        lcd.printf("Sitka      (GMT-9)         ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-9,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case SanFrancisco:
                        lcd.locate(0,0);
                        lcd.printf("SanFrancisco     (GMT-8)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-8,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Colorado:
                        lcd.locate(0,0);
                        lcd.printf("Colorado   (GMT-7)        ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-7,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case NewYork:
                        lcd.locate(0,0);
                        lcd.printf("New York   (GMT-5)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-5,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Palmas:
                        lcd.locate(0,0);
                        lcd.printf("Palmas       (GMT-3)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-3,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Lagoa:
                        lcd.locate(0,0);
                        lcd.printf("Lagoa      (GMT-1)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(-1,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Manchester:
                        lcd.locate(0,0);
                        lcd.printf("Manchester (GMT)      ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(0,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Berlin:
                        lcd.locate(0,0);
                        lcd.printf("Berlin     (GMT+1)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(1,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Cairo:
                        lcd.locate(0,0);
                        lcd.printf("Cairo     (GMT+2)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(2,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Tehran:
                        lcd.locate(0,0);
                        lcd.printf("Tehran     (GMT+3.5)  ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(3,30);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Munbai:
                        lcd.locate(0,0);
                        lcd.printf("Munbai     (GMT+5.5)   ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(5,30);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Beijing:
                        lcd.locate(0,0);
                        lcd.printf("Beijing   (GMT+8)    ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(8,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Sydney:
                        lcd.locate(0,0);
                        lcd.printf("Sydney     (GMT+10)   ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(10,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    case Alo:
                        lcd.locate(0,0);
                        lcd.printf("Alo        (GMT+12)   ");
                        lcd.locate(0,9);
                        settimeparameter.addtime(12,0);
                        lcd.printf(     "%02d:%02d:%02d             ",settimeparameter.getaddhour(),settimeparameter.getaddmin(),settimeparameter.getseconds());
                        lcd.locate(0, 18);
                        lcd.printf(     "%02d:%02d:%02d  (Manchester)",settimeparameter.gethours(),settimeparameter.getmins(),settimeparameter.getseconds());
                        break;
                    default:
                        state_area = Manchester;
                }
                break;
                
                break;
            case (stopwatch):
               if (timer.read() == 0 or state_watch == stop){
                    lcd.locate(0,0);
                    lcd.printf("Stopwatch inactive           ");
                    lcd.locate(0, 9);
                    lcd.printf("Last time %.2f S                 ",timer.read());
                } else {
                    led.Blue();
                    lcd.locate(0,0);
                    lcd.printf("Stopwatch Running           ");
                    lcd.locate(0, 9);
                    lcd.printf("time %.2f S                 ",timer.read());
                }
                lcd.locate(0, 18);
                lcd.printf("                            ");
                break;
            case (countdown_timer):
               if(state_downtimer == setPeriod){
            //sampler2.detach();
                     grticker.detach();
                     lcd.locate(0,0);
                     lcd.printf("Set Countdown Period     ", cd_clock.getmins(), cd_clock.getseconds());    
                     lcd.locate(0, 9);
                     lcd.printf("%02d:%02d                ", cd_clock.getmins(), cd_clock.getseconds());
                     lcd.locate(0,18);
                     lcd.printf("                         ");
                     cd_clock.settime(0, settingmin, settingsec);
              //settingmin = setdownmin.getCurrentSampleVolts();
              //settingsec = setdownsec.getCurrentSampleVolts();
                     setdowntotalsec = settingmin*60 + settingsec;
                     totalsec =  cd_clock.getmins() * 60 + cd_clock.getseconds();

        }
               if(state_downtimer == countdownbegin){
            //sampler2.attach(&mydown, &Clock::tick_down,1);
                     grticker.attach(callback(&led, &RGBLED::toggle_green), 0.5);           //led green toggle
                     lcd.locate(0,0);
                     lcd.printf("Countdown Timer Running     "  );
                     lcd.locate(0,9);
                     lcd.printf("%d  /  %d s             ",totalsec,setdowntotalsec);
                     lcd.locate(0,18);
                     lcd.printf("                     ");
                     if (totalsec == 0) {
                         state_downtimer = countdownover;
                }
        }
               if(state_downtimer == countdownover){
                     speaker.on();
                     grticker.detach();
                     led.Green();

                     lcd.locate(0,0);
                     lcd.printf("Time Period Elapsed         ");
                     lcd.locate(0,9);
                     lcd.printf("                            ");
                     lcd.locate(0,18);
                     lcd.printf("                            ");           
        }

                break;
            default:
                state = initialisation;
        }
    }
}