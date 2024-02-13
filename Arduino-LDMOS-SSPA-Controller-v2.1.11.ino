/*                                                                                             */
/*  Arduino Code (Version 2.1.11) for S21RC LDMOS Controller V1.x. PCB.                        */
/*  This board and code has been tested with DXWORLD-E protection board,                       */
/*  DXWORLD-E LPF Board. Any LDMOS SSPA system with similar IO should work.                    */
/*                                                                                             */
/*  Created by Fazlay Rabby S21RC, 24 NOV, 2023.                                               */
/*  Released into the public domain.                                                           */
/*                                                                                             */
/*  Feel free to use/change the Gerber file for the PCB,                                       */
/*  display file(nextion TFT) for the display and all codes here.                              */
/*                                                                                             */
/*  If you need any clarification or help please issue a ticket in github.                     */
/*                                                                                             */
/* For PCB Version 1.x, Code V2.1.x Lite version custom built based over CODE V 2.0 for DF1JM */

/* Simple graphical interface to show the status of the SSPA only,                            */
/* no software protection except Tempareture.                                                  */

/* ======== NOTE ========= */

// VR VM: use two fix value resistor instead of a potentiometer, 150K from VCC pin to Middle pin, 10K from GND to middle pin.  pin1---[/\/\]---pin2---[/\/\]---pin3
//  value of Res_1 and Res_2 please update in the code below to match the exact resistor values used.

// VR FW: use two fix value resistor instead of a potentiometer, 100K + 100K pin1---[/\/\]---pin2---[/\/\]---pin3
// VR RE: use two fix value resistor instead of a potentiometer, 100K + 100K pin1---[/\/\]---pin2---[/\/\]---pin3

// All wire connecting the controller must go through snap-on Ferrite clamp (RFI filter)

// PCB NOTE: (1) for PCB Version V1.3 and prior: Swap pin A6 and A5 on the PCB by cutting the trace.
//           (2) for PCB version V1.6 no need for swapping, corrected.
// on board LM2596HV module reported getting very hot and some cases exploded, please use external 60v to 12v converter module.
// on board 7805 regulator need proper heatsink. You may mount it of the board if needed.
// Remove the UART/Display connection when Arduino is connected to computer for programming, as they use same RX/TX port.
// Remove arduino USB/UART from computer when Display is connected.

/* ======== CHANGE LOG ========= */
/*
 * 2.1.11 Lite (11 Feb 2024):
 * SWR calculation corrected. Power ratio was inverted before.
 * Second Temperature sensor code added [only graph, numeric not implimented as that need display/HMI change]
 *
 * 2.1.10 Lite (11 Dec 2023):
 * nput pullup added to code for the 3 error input (LED): for OM testing the code without the PCB (PCB has pullup resistors). (reported by F5OTZ)
 * 2.1.09 Lite:
 * Solved the current meter stuck to high current after releasing PTT (Reported by DF1JM).
 *

*/
#include <EEPROM.h>
#include "math.h"
#include <OneWire.h>
#include <DallasTemperature.h>  //https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "EasyNextionLibrary.h" // https://github.com/Seithan/EasyNextionLibrary

/* ======= Arduino NANO PIN assignment ======== */

#define ROTARY_POS A7
#define PWR_FWD A2
#define PWR_REF A1
#define ID A6 // please see note (1) above.
// #define TEMP1_PIN A6 //please see note (2) above.
#define LED_PO 3
#define LED_I 2
#define VCC A0
#define LED_SWR 4
#define BIAS_OFF 17 // Use 12v relay on AUX header/port on PCB. Common>12V Bias, N.O>to AMP bias
#define PTT 18      // D18/A4
#define band1 5
#define band2 6
#define band3 7
#define band4 8
#define band5 11
#define band6 12
#define band7 13
#define antenna2 10 // Antenna 2 relay pin of arduino
#define ONE_WIRE_BUS 19

/* ======= Nextion Display Variables ========

Below are the variables inside the nextion HMI display
Please see screenshot to understand better.


MAIN PAGE:

a1 : Antenna 2 switch (Dual state button)
r1 : Band Touch enable (Dual state button)
r2 : Band selected from RIG (Dual state button)

b1 : Band 1 (Dual state button)
b2 : Band 2 (Dual state button)
b3 : Band 3 (Dual state button)
b4 : Band 4 (Dual state button)
b5 : Band 5 (Dual state button)
b6 : Band 6 (Dual state button)
b7 : Band 7 (Dual state button)

e1: On Air indicator (Dual state button)
e2: Over current indicator (Dual state button)
e3: High SWR indicator (Dual state button)
e4: High Temp indicator (Dual state button)
e5: Over power indicator (Dual state button)
e6: Over voltage notification (Dual state button)

C0: Setup page button (Dual state button)
op: place to display owner callsign (Text, 7 char max)
Cw: Display power output in watt (Number).
Cs: Display SWR (Xfloat).
Ct: Display tempareture (C) (Xfloat)

Gw: Power output bar (Progress bar)
Gs: SWR bar (Progress bar)
Gt1: Temp bar (Progress bar)
Gt2: Temp bar (Progress bar)

Cv: Display Voltage (Xfloat)
Ca: Display Current (Xfloat)
Ce: Display Efficiency (Number)
Cf: Fan speed percentage (Number)
Cb: Display Current LPF Bank (Number)

Er1: Notification (Text)

*/

EasyNex myNex(Serial);

/* ======= Tempareture ======== */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
int resolution = 9;
unsigned long lastTempRequest = 0;

bool dtemp2_status = 1; // 1 if second digital sensor present, 0 for one sensor.
float temperature = 0.0;
float temperature2 = 0.0;
float last_temperature = 0.0;
float last_temperature2 = 0.0;

/* ======= PWM FAN ======== */
const byte OC1A_PIN = 9;
// const byte OC1B_PIN = 10;
word PWM_FREQ_KHZ = 25; // Set currently to 25kHZ for Dalas 12V 4 wire Fan
word TCNT1_TOP = 16000 / (2 * PWM_FREQ_KHZ);

/* USER DEFINED VARIABLES, Change value if needed */

int graph_maxWatt;
int graph_maxTemp;
int graph_maxSwr;
int TdelayInMillis = 0;
int TdisdelayInMillis = 1000; // refresh Temp display every N mili sec, set as per your liking
int VdelayInMillis = 2000;    // refresh V display every N mili sec, set as per your liking
int IdelayInMillis = 1000;    // refresh I display every N mili sec, set as per your liking
int EdelayInMillis = 1000;    // refresh Efficiency display every N mili sec, set as per your liking
int PdelayInMillis = 250;     // check power/swr every N mili sec.
int DiagdelayInMillis = 500;
float Res_1 = 150000.00; // Set R1 of voltage devider (VR VM pot, center to VDD) (you can use 150K resistor)
float Res_2 = 10000.00;  // Set R2 of voltage devider (VR VM pot, center to GND) (you can use 10K  resistor)
int RL = 1800;           // Set RL value for R_IS. for 3K3 it should be 3300. Due to the series 1K resistor in DXworld the calculation
// is not possible to get due to variable resistor placed there. please play with it to get nearest result.

// Calibration factors for SWR/PO
unsigned long calibrationP;
int calibPower;
int Vdiode = 61; // if we assume FWD voltage diode = 0,3v : Vdiode = 0,3 v / 5.0 v  x 1024
int T_pepHOLD;   // msec pep hold time before return

int calibvoltADC;
int maxcalibV;

int Eff = 0;
int display_page = 0;

/* ======== Other variables ======== */
bool touch_status = true;
bool rig_status = false;
bool PTT_status = false;
bool error_i_status = false;
bool error_swr_status = false;
bool error_po_status = false;
bool error_temp_status = false;
bool antenna2_status = false;
bool alarm = false;
bool rig_band = false;

int protection_po;
int protection_swr = 3;
int protection_temp;

int current_band = 0;
int band_pos = 0;
int temp_heatsink = 0;

unsigned long lastPRequest = 0;
unsigned long lastTRequest = 0;
unsigned long lastVRequest = 0;
unsigned long lastIRequest = 0;
unsigned long lastERequest = 0;
unsigned long lastTdisRequest = 0;
unsigned long lastDiagRequest = 0;

char callsign[8];
char new_callsign[8] = "abcd";
char error_text[20] = "";

int default_value = 0;

int V = 0;
int lastV = 0;

int I;
int lastI = 0;

float ref_vdd = (5.00 / 1024);

int last_r1 = 0;

int graph_Watt = 0;
int graph_Temp = 0;
int graph_Temp2 = 0;
int graph_Swr = 0;

/* ======== power and swr related variable ======== */

long lastTpep = 0; // update PEP display

unsigned long power_fwd = 0; // power forward (watts)
unsigned long power_ref = 0; // power reflected (watts)
int raw_fwd = 0;             // power forward for SWR calculation
int raw_ref = 0;             // power reflected for SWR calculation

int power_fwd_max = 0; // power forward max (for peak hold)
int power_ref_max = 0; // power reflected max (for peak hold)

float SWR = 0;         // SWR
float power_ratio = 0; // Power ratio P forward / P refl
int swr_display = 10;  // swr x 10 showing in display

/* === PWM Function === */

int PWM = 0;
int last_PWM = 0;

void setPwmDuty(byte duty)
{
    OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

/* === Volt meter Function === */

float V_now()
{
    float currentV;
    currentV = (analogRead(VCC) * ref_vdd) / (Res_2 / (Res_1 + Res_2));
    currentV = (analogRead(VCC) * ref_vdd) / (Res_2 / (Res_1 + Res_2));
    return currentV;
}

float fwd_now()
{
    float currentFWD;

    for (int n = 0; n <= 5; n++)
    {
        currentFWD = analogRead(PWR_FWD);
        // Takes 5 reading and use the last one. It is possible to do average if needed here.
    }
    return currentFWD;
}

float ref_now()
{
    float currentREF;

    for (int n = 0; n <= 5; n++)
    {
        currentREF = analogRead(PWR_REF);
        // Takes 5 reading and use the last one. It is possible to do average if needed here.
    }
    return currentREF;
}

void read_volt()
{
    float vcc;

    V = ((V_now() * 10) + 0.5); // Float x 10 and convert to int with rounding

    if ((millis() - lastVRequest) >= VdelayInMillis && lastV != V)
    {
        myNex.writeNum("Cv.val", V);
        lastVRequest = millis();
        lastV = V;
    }
}

/* === SWR/Po calculation Function === */
void read_power()
{
    float Veff = V_now();
    float Iprot = I_now();

    raw_fwd = fwd_now();
    raw_ref = ref_now();

    if (raw_fwd > 20)
    { // only correct for diode voltage when more than zero
        power_fwd = raw_fwd + Vdiode;
        power_fwd = power_fwd * power_fwd;
        power_fwd = power_fwd / calibrationP;
    }
    else
        power_fwd = 0;

    if (raw_ref > 20)
    { // only correct for diode voltage when more than zero
        power_ref = raw_ref + Vdiode;
        power_ref = power_ref * power_ref;
        power_ref = power_ref / calibrationP;
    }
    else
        power_ref = 0;

    if (power_ref > 0 && !power_fwd == 0)
    {
        power_ratio = power_ref / power_fwd; //Theory corrected
        SWR = abs((1 + sqrt(power_ratio)) / (1 - sqrt(power_ratio)));
    }
    else
        SWR = 1;

    // hold peak

    if (power_fwd >= power_fwd_max)
    {
        lastTpep = millis();
        power_fwd_max = power_fwd;
    }

    if (millis() > (lastTpep + T_pepHOLD))
        power_fwd_max = power_fwd; // clear the peak after hold time

    if (Veff != 0 && Iprot != 0)
        Eff = 100 * (power_fwd / (Veff * Iprot));
}

/* === SWR/Po display Function === */
// Refresh screen every N miliseconds (N = PdelayInMillis)
void display_power(bool active)
{

    if (active)
    {
        if ((millis() - lastPRequest) >= PdelayInMillis)
        {

            swr_display = (SWR * 10) + 0.5; //// Float x 10 and convert to int with rounding
            if (swr_display < 10)
                swr_display = 10; // SWR cannot be lower than 1.0

            float graph_limit_watt = (graph_maxWatt / 100.00);
            graph_Watt = (power_fwd_max / graph_limit_watt);

            float graph_limit_swr = ((graph_maxSwr - 1) / 100.00);
            float swr_forgraph = SWR - 1;
            graph_Swr = swr_forgraph / graph_limit_swr;

            myNex.writeNum("Cw.val", power_fwd_max);
            myNex.writeNum("Gw.val", graph_Watt);
            myNex.writeNum("Cs.val", swr_display);
            myNex.writeNum("Gs.val", graph_Swr);
            myNex.writeNum("Ce.val", Eff);

            power_fwd_max = 0;
            graph_Watt = 0;
            swr_display = 10;
            graph_Swr = 0;
            Eff = 0;

            lastPRequest = millis();
        }
    }
    else
    {
        if ((millis() - lastPRequest) >= PdelayInMillis)
        {
            power_fwd_max = 0;
            graph_Watt = 0;
            swr_display = 10;
            graph_Swr = 0;
            Eff = 0;

            myNex.writeNum("Cw.val", power_fwd_max);
            myNex.writeNum("Gw.val", graph_Watt);
            myNex.writeNum("Cs.val", swr_display);
            myNex.writeNum("Gs.val", graph_Swr);
            myNex.writeNum("Ce.val", Eff);
            lastPRequest = millis();
        }
    }
}

/* =========== Send band info to display ====================== */

void display_band(int Nband)
{

    myNex.writeNum("Cb.val", Nband);
    switch (Nband)
    {
    case 1:
        myNex.writeNum("b1.val", 1);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
        break;
    case 2:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 1);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
        break;
    case 3:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 1);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
        break;
    case 4:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 1);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
        break;
    case 5:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 1);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
        break;
    case 6:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 1);
        myNex.writeNum("b7.val", 0);
        break;
    case 7:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 1);
        break;
    }
}

/* =========== LPF Relay control ====================== */

void lpf_relay(int Nrelay)
{
    EEPROM.put(0, Nrelay); // Update/write band info to Memory
    switch (Nrelay)
    {
    case 0:
        digitalWrite(band1, LOW);
        digitalWrite(band2, LOW);
        digitalWrite(band3, LOW);
        digitalWrite(band4, LOW);
        digitalWrite(band5, LOW);
        digitalWrite(band6, LOW);
        digitalWrite(band7, LOW);
        break;
    case 1:
        digitalWrite(band1, HIGH);
        digitalWrite(band2, LOW);
        digitalWrite(band3, LOW);
        digitalWrite(band4, LOW);
        digitalWrite(band5, LOW);
        digitalWrite(band6, LOW);
        digitalWrite(band7, LOW);
        break;
    case 2:
        digitalWrite(band1, LOW);
        digitalWrite(band2, HIGH);
        digitalWrite(band3, LOW);
        digitalWrite(band4, LOW);
        digitalWrite(band5, LOW);
        digitalWrite(band6, LOW);
        digitalWrite(band7, LOW);
        break;
    case 3:
        digitalWrite(band1, LOW);
        digitalWrite(band2, LOW);
        digitalWrite(band3, HIGH);
        digitalWrite(band4, LOW);
        digitalWrite(band5, LOW);
        digitalWrite(band6, LOW);
        digitalWrite(band7, LOW);
        break;
    case 4:
        digitalWrite(band1, LOW);
        digitalWrite(band2, LOW);
        digitalWrite(band3, LOW);
        digitalWrite(band4, HIGH);
        digitalWrite(band5, LOW);
        digitalWrite(band6, LOW);
        digitalWrite(band7, LOW);
        break;
    case 5:
        digitalWrite(band1, LOW);
        digitalWrite(band2, LOW);
        digitalWrite(band3, LOW);
        digitalWrite(band4, LOW);
        digitalWrite(band5, HIGH);
        digitalWrite(band6, LOW);
        digitalWrite(band7, LOW);
        break;
    case 6:
        digitalWrite(band1, LOW);
        digitalWrite(band2, LOW);
        digitalWrite(band3, LOW);
        digitalWrite(band4, LOW);
        digitalWrite(band5, LOW);
        digitalWrite(band6, HIGH);
        digitalWrite(band7, LOW);
        break;
    case 7:
        digitalWrite(band1, LOW);
        digitalWrite(band2, LOW);
        digitalWrite(band3, LOW);
        digitalWrite(band4, LOW);
        digitalWrite(band5, LOW);
        digitalWrite(band6, LOW);
        digitalWrite(band7, HIGH);
        break;
    }

    current_band = Nrelay;
    display_band(Nrelay);
}

/* === Monitor and show PTT status on display === */
void onair()
{
    int PTT_en = digitalRead(PTT);

    if (PTT_en == 0 && PTT_status != true)
    {
        PTT_status = HIGH;
        myNex.writeNum("e1.val", 1);
        LCDband_disable();
    }

    else if (PTT_en == 1 && PTT_status != false)
    {
        myNex.writeNum("e1.val", 0);
        PTT_status = LOW;
        LCDband_enable();
    }
}

void setup_onair()
{
    int PTT_en = digitalRead(PTT);

    if (PTT_en == 0 && PTT_status != true)
    {
        PTT_status = HIGH;
        myNex.writeNum("sq0.picc", 6);
    }

    else if (PTT_en == 1 && PTT_status != false)
    {
        myNex.writeNum("sq0.picc", 5);
        PTT_status = LOW;
    }
}

/* === Monitor and show over current error on display === */
void error_i()
{
    int error_i_en = digitalRead(LED_I);
    // error_i_en = 0; //test
    if (error_i_en == LOW && error_i_status == false)
    {
        protection_en(true, true, 2);

        error_i_status = true;
    }
    else if (error_i_en == HIGH && error_i_status == true)
    {
        protection_en(false, true, 2);
        error_i_status = false;
    }
}

/* === Monitor and show over SWR error on display === */
void error_swr()
{
    int error_swr_en = digitalRead(LED_SWR);
    // error_swr_en = 0; //test
    if (error_swr_en == LOW && error_swr_status == false)
    {
        protection_en(true, true, 3);

        error_swr_status = true;
    }
    else if (error_swr_en == HIGH && error_swr_status == true)
    {
        protection_en(false, true, 3);
        error_swr_status = false;
    }
}

/* === Monitor and show over Power error on display === */
void error_po()
{
    int error_po_en = digitalRead(LED_PO);
    // error_po_en = 0; //test ****************************************
    if (error_po_en == LOW && error_po_status == false)
    {
        protection_en(true, true, 5);

        error_po_status = true;
    }
    else if (error_po_en == HIGH && error_po_status == true)
    {
        protection_en(false, true, 5);
        error_po_status = false;
    }
}

void protection_en(bool protection, bool fault_type, int fault_code)
{
    if (protection)
    {
        digitalWrite(BIAS_OFF, LOW); // Bias off Relay connected to AUX on PCB
        alarm = true;

        myNex.writeStr("Er1.txt", "SSPA in Protection mode");

        switch (fault_code)
        {
        case 0:
            break;
        case 1:
            // Fault code 1: High Voltage
            myNex.writeNum("e6.val", 1);
            break;
        case 2:
            // Fault code 2: High Current
            myNex.writeNum("e2.val", 1);
            break;
        case 3:
            // Fault code 3: High SWR
            myNex.writeNum("e3.val", 1);
            break;
        case 4:
            // Fault code 4:

            break;
        case 5:
            // Fault code 5: High power output
            myNex.writeNum("e5.val", 1);
            break;
        case 6:
            // Fault code 6:

            break;
        case 7:
            // Fault code 7: High Temp
            myNex.writeNum("e4.val", 1);
            break;
        }
    }
    else
    {
        digitalWrite(BIAS_OFF, HIGH); // Bias off Relay connected to AUX on PCB
        alarm = false;

        myNex.writeStr("Er1.txt", "");

        switch (fault_code)
        {
        case 0:
            break;
        case 1:
            // Fault code 1: High Voltage
            myNex.writeNum("e6.val", 0);
            break;
        case 2:
            // Fault code 2: High Current
            myNex.writeNum("e2.val", 0);
            break;
        case 3:
            // Fault code 3: High SWR
            myNex.writeNum("e3.val", 0);
            break;
        case 4:
            // Fault code 4:
            // myNex.writeNum("e8.picc", main_pic_value);
            break;
        case 5:
            // Fault code 5: High power output
            myNex.writeNum("e5.val", 0);
            break;
        case 6:
            // Fault code 6:
            // myNex.writeNum("e7.picc", main_pic_value);
            break;
        case 7:
            // Fault code 7: High Temp
            myNex.writeNum("e4.val", 0);
            break;
        }
    }
}

/* === Monitor current from protection board  === */
float I_now()
{
    float currentI;

    for (int n = 0; n <= 5; n++)
    {
        currentI = analogRead(ID);
    }

    currentI = currentI * ref_vdd;
    currentI = currentI / RL;
    currentI = currentI * 13000;
    return currentI;
}

void read_ID()
{
    I = ((I_now() * 10) + 0.5); // Float x 10 and convert to int with rounding

    if ((millis() - lastIRequest) >= IdelayInMillis && lastI != I)
    {
        myNex.writeNum("Ca.val", I);
        lastIRequest = millis();
        lastI = I;
    }
}

/* === Set FAN speed according to temperature === */
void fanspeed()
{
    if (temperature < 25.0)
        PWM = 20;
    else if (temperature > 25.0 && temperature < 30.0)
        PWM = 40;
    else if (temperature > 30.0 && temperature < 40.0)
        PWM = 60;
    else if (temperature >= 40.0 && temperature < 45.0)
        PWM = 70;
    else if (temperature >= 45.0 && temperature < 50.0)
        PWM = 80;
    else if (temperature >= 50.0)
        PWM = 100;

    if (PWM != last_PWM)
    {
        setPwmDuty(PWM);
        last_PWM = PWM;
        int fan_speed = PWM;
        myNex.writeNum("Cf.val", fan_speed);
    }
}

/* === temperature Monitor  === */
void read_temp()
{
    if (millis() - lastTempRequest >= TdelayInMillis)
    {
        temperature = sensors.getTempCByIndex(0);
        if (dtemp2_status)
            temperature2 = sensors.getTempCByIndex(1);
        else
            temperature2 = 0.0;
        sensors.requestTemperatures();
        lastTempRequest = millis();
    }

    if (temperature >= float(protection_temp) || temperature2 >= float(protection_temp))
    {
        setPwmDuty(100);
        protection_en(true, true, 7);
        error_temp_status = true;
    }

    
    if (error_temp_status){
      if (temperature <= (protection_temp - 2) && temperature2 <= (protection_temp - 2) )
      {
          //(protection_temp-x): change the -2 to other value for temp error remove value
          protection_en(false, true, 7);
      }
    }

    if ((millis() - lastTdisRequest) >= TdisdelayInMillis)
    {
        int temp_display = (temperature * 10) + 0.5; // Float x 10 and convert to int with rounding
        // int temp2_display = (temperature2 * 10) + 0.5;   // Float x 10 and convert to int with rounding
        myNex.writeNum("Ct.val", temp_display);
        // myNex.writeNum("Ct1.val", temp2_display);
        float graph_limit = (graph_maxTemp / 100.00);
        graph_Temp = (temperature / graph_limit);
        graph_Temp2 = (temperature2 / graph_limit);
        myNex.writeNum("Gt1.val", graph_Temp);
        myNex.writeNum("Gt2.val", graph_Temp2);
        lastTdisRequest = millis();
        last_temperature = temperature;
        last_temperature2 = temperature2;
    }
    fanspeed();
}

/* === Enabling touch functions of Display === */
void LCDband_enable()
{
    if (touch_status != true)
    {
        myNex.writeStr("tsw b1,1");
        myNex.writeStr("tsw b2,1");
        myNex.writeStr("tsw b3,1");
        myNex.writeStr("tsw b4,1");
        myNex.writeStr("tsw b5,1");
        myNex.writeStr("tsw b6,1");
        myNex.writeStr("tsw b7,1");
        myNex.writeStr("tsw a0,1");
        myNex.writeStr("tsw a1,1");
        touch_status = true;
    }
}

/* === Disable band switch touch functions=== */
void bandswitch_disable()
{
    if (rig_status != true)
    {
        myNex.writeStr("tsw b1,0");
        myNex.writeStr("tsw b2,0");
        myNex.writeStr("tsw b3,0");
        myNex.writeStr("tsw b4,0");
        myNex.writeStr("tsw b5,0");
        myNex.writeStr("tsw b6,0");
        myNex.writeStr("tsw b7,0");
        rig_status = true;
    }
}

/* === Disabling touch functions of Display === */
void LCDband_disable()
{
    if (touch_status != false)
    {
        // myNex.writeStr("tsw 255,0");
        myNex.writeStr("tsw b1,0");
        myNex.writeStr("tsw b2,0");
        myNex.writeStr("tsw b3,0");
        myNex.writeStr("tsw b4,0");
        myNex.writeStr("tsw b5,0");
        myNex.writeStr("tsw b6,0");
        myNex.writeStr("tsw b7,0");
        touch_status = false;
    }
}

/* === Rotary band switch === */

int band_switch()
{
    int band_select;
    // read rotary value
    int band_value;

    for (int n = 0; n <= 5; n++)
    {
        band_value = analogRead(ROTARY_POS);
    }

    if (band_value <= 100)
    {
        if (last_r1 != 0)
        {
            last_r1 = 0;

            myNex.writeNum("r1.val", 1);
            myNex.writeNum("r2.val", 0);
        }

        if (PTT_status != HIGH)
            LCDband_enable();
        band_select = current_band;
    }
    else if (band_value >= 101 && band_value <= 200)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;

            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 1;
    }
    else if (band_value >= 201 && band_value <= 350)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;

            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 2;
    }
    else if (band_value >= 351 && band_value <= 500)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;

            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 3;
    }
    else if (band_value >= 501 && band_value <= 650)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;

            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 4;
    }
    else if (band_value >= 651 && band_value <= 800)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;

            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 5;
    }
    else if (band_value >= 801 && band_value <= 950)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;

            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 6;
    }
    else if (band_value >= 951 && band_value <= 1024)
    {
        if (last_r1 != 1)
        {
            last_r1 = 1;
            myNex.writeNum("r1.val", 0);
            myNex.writeNum("r2.val", 1);
        }
        LCDband_disable();
        band_select = 7;
    }
    else
    {
    }
    return band_select;
}

int band_rig()
{
    int band_value;
    band_value = current_band;
    return band_value;
}

void band_position()
{
    // rig_band = myNex.readNumber("r2.val");

    if (rig_band == true)
    {
        LCDband_disable();
        band_pos = band_rig();
    }
    else
    {
        band_pos = band_switch();
    }

    if (current_band != band_pos && PTT_status != HIGH)
    {
        lpf_relay(band_pos);
    }
}

void band_position_touch()
{
    if (current_band != band_pos && PTT_status != HIGH)
    {
        lpf_relay(band_pos);
    }
}

/* === Trigger from Nextion Display  === */
void trigger1()
{
    band_pos = 1;
    band_position_touch();
}

void trigger2()
{
    band_pos = 2;
    band_position_touch();
}

void trigger3()
{
    band_pos = 3;
    band_position_touch();
}

void trigger4()
{
    band_pos = 4;
    band_position_touch();
}

void trigger5()
{
    band_pos = 5;
    band_position_touch();
}

void trigger6()
{
    band_pos = 6;
    band_position_touch();
}

void trigger7()
{
    band_pos = 7;
    band_position_touch();
}

// Antenna2 switch
void trigger8()
{
    if (antenna2_status == LOW)
    {
        digitalWrite(antenna2, HIGH);
        myNex.writeNum("a1.val", 1);
        antenna2_status = HIGH;
    }
    else
    {
        digitalWrite(antenna2, LOW);
        myNex.writeNum("a1.val", 0);
        antenna2_status = LOW;
    }
}

void trigger9()
{
    // display_page=1;
    loadEEPROM();
    myNex.writeStr("st10.txt", callsign);
    myNex.writeNum("s0.val", graph_maxWatt);
    myNex.writeNum("s1.val", graph_maxSwr);
    myNex.writeNum("s2.val", graph_maxTemp);
    myNex.writeNum("s3.val", protection_po);
    myNex.writeNum("s4.val", protection_swr);
    myNex.writeNum("s5.val", protection_temp);
    myNex.writeNum("s6.val", T_pepHOLD);
    myNex.writeNum("s7.val", PWM_FREQ_KHZ);
    myNex.writeNum("s8.val", calibPower);
    myNex.writeNum("s9.val", calibvoltADC);
    display_page = 1;
}

void trigger10()
{ // 0A

    graph_maxWatt = myNex.readNumber("s0.val");
    graph_maxSwr = myNex.readNumber("s1.val");
    graph_maxTemp = myNex.readNumber("s2.val");
    protection_po = myNex.readNumber("s3.val");
    protection_swr = myNex.readNumber("s4.val");
    protection_temp = myNex.readNumber("s5.val");
    T_pepHOLD = myNex.readNumber("s6.val");
    PWM_FREQ_KHZ = myNex.readNumber("s7.val");
    calibPower = myNex.readNumber("s8.val");
    calibvoltADC = myNex.readNumber("s9.val");
    strcpy(new_callsign, (char *)myNex.readStr("st10.txt").c_str());
    if (strcmp(callsign, new_callsign) != 0)
        strcpy(callsign, new_callsign);
    powerCalibration();
    saveEEPROM();
    display_page = 1;
}

void trigger11()
{ // 0B

    powerCalibration();
    normalrun();
    // band_position();
    lpf_relay(band_pos);
    myNex.writeStr("op.txt", callsign);

    display_page = 0;
    myNex.writeStr("Er1.txt", " ");
}

void trigger12()
{ // 0C
    rig_status = false;
    rig_band = true;
    bandswitch_disable();
}

void trigger13()
{ // 0D
    touch_status = false;
    rig_band = false;
    LCDband_enable();
}

void trigger14()
{ // 0E
    display_page = 2;
}

void trigger15()
{ // 0F
    strcpy(new_callsign, (char *)myNex.readStr("st10.txt").c_str());
    if (strcmp(callsign, new_callsign) != 0)
        strcpy(callsign, new_callsign);
}

void trigger16()
{ // 10
    calibvoltADC = 1;
}

void powerCalibration()
{
    // calibrationP = ((calibvoltADC+Vdiode)/calibPower)*(calibvoltADC+Vdiode);    // Assume 3.3v = 1000w in 50R. calibrationP = (3.3 / 5.0 x 1024 + Vdiode)x(3.3 / 5.0 x 1024 + Vdiode) / 1000w
    calibrationP = calibvoltADC + Vdiode;
    calibrationP = calibrationP * calibrationP;
    calibrationP = calibrationP / calibPower;
}

void loadEEPROM()
{

    EEPROM.get(0, current_band);
    EEPROM.get(110, calibvoltADC);
    EEPROM.get(10, calibPower);
    EEPROM.get(15, graph_maxWatt);
    EEPROM.get(20, graph_maxSwr);
    EEPROM.get(25, graph_maxTemp);
    EEPROM.get(30, protection_po);
    EEPROM.get(35, protection_swr);
    EEPROM.get(40, protection_temp);
    EEPROM.get(45, T_pepHOLD);
    EEPROM.get(50, PWM_FREQ_KHZ);
    // EEPROM.get(55, callsign);

    for (int n = 0; n < 8; n++)
    {
        callsign[n] = EEPROM.read(n + 55);
    }
    callsign[sizeof(callsign)] = '\0';
}

void saveEEPROM()
{
    EEPROM.put(110, calibvoltADC);
    EEPROM.put(10, calibPower);
    EEPROM.put(15, graph_maxWatt);
    EEPROM.put(20, graph_maxSwr);
    EEPROM.put(25, graph_maxTemp);
    EEPROM.put(30, protection_po);
    EEPROM.put(35, protection_swr);
    EEPROM.put(40, protection_temp);
    EEPROM.put(45, T_pepHOLD);
    EEPROM.put(50, PWM_FREQ_KHZ);

    for (int n = 0; n < sizeof(callsign); n++) // automatically adjust for number of digits
    {
        EEPROM.update(n + 55, callsign[n]);
    }
}

void PWMsetup()
{
    /* === FAN PWM setup === */
    TCNT1_TOP = 16000 / (2 * PWM_FREQ_KHZ);
    pinMode(OC1A_PIN, OUTPUT); // 25/31KHz fan pwm

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << CS10);
    ICR1 = TCNT1_TOP;
}

void default_write()
{
    current_band = 1;
    calibvoltADC = 100;
    calibPower = 900;
    graph_maxWatt = 1000;
    graph_maxSwr = 4;
    graph_maxTemp = 60;
    protection_po = 1000;
    protection_swr = 3;
    protection_temp = 55;
    T_pepHOLD = 200;
    PWM_FREQ_KHZ = 25;
    strcpy(callsign, "ABCDE");
    default_value = 2023;

    EEPROM.put(0, current_band);
    EEPROM.put(110, calibvoltADC);
    EEPROM.put(10, calibPower);
    EEPROM.put(15, graph_maxWatt);
    EEPROM.put(20, graph_maxSwr);
    EEPROM.put(25, graph_maxTemp);
    EEPROM.put(30, protection_po);
    EEPROM.put(35, protection_swr);
    EEPROM.put(40, protection_temp);
    EEPROM.put(45, T_pepHOLD);
    EEPROM.put(50, PWM_FREQ_KHZ);

    for (int n = 0; n < sizeof(callsign); n++) // automatically adjust for number of digits
    {
        EEPROM.update(n + 55, callsign[n]);
    }
    EEPROM.put(100, default_value);
}

void setup()
{
    delay(100);
    myNex.begin(115200); // start Nextion Display. baud rate should match settings in HMI/TFT
    PWMsetup();

    /* ==== Temp sensor setup ==== */
    sensors.begin(); // Temp sensor
    sensors.getAddress(tempDeviceAddress, 0);
    sensors.setResolution(tempDeviceAddress, resolution);
    if (dtemp2_status)
    {
        sensors.getAddress(tempDeviceAddress, 1);
        sensors.setResolution(tempDeviceAddress, resolution);
    }

    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    TdelayInMillis = 750 / (1 << (12 - resolution));
    lastTempRequest = millis();

    pinMode(band1, OUTPUT);
    pinMode(band2, OUTPUT);
    pinMode(band3, OUTPUT);
    pinMode(band4, OUTPUT);
    pinMode(band5, OUTPUT);
    pinMode(band6, OUTPUT);
    pinMode(band7, OUTPUT);
    pinMode(BIAS_OFF, OUTPUT);
    pinMode(antenna2, OUTPUT); // Added in V2.1.11, was missing previously, reported by Laurent.
    

    pinMode(PTT, INPUT_PULLUP);
    pinMode(LED_PO, INPUT_PULLUP);
    pinMode(LED_I, INPUT_PULLUP);
    pinMode(LED_SWR, INPUT_PULLUP);
    pinMode(VCC, INPUT);
    EEPROM.get(100, default_value);
    if (default_value != 2023)
        default_write(); // write default values first time in new microcontroller.
    // default_write(); // UNCOMMENT TO WRITE DEFAULT VALUES TO EEPROM, and reupload again with commented out.
    loadEEPROM();
    powerCalibration();
    band_position(); // LPF Band position
    lpf_relay(current_band);
    protection_en(false, false, 0);
    myNex.writeStr("op.txt", callsign);
    digitalWrite(BIAS_OFF, HIGH);
    myNex.writeNum("r1.val", 1);
    myNex.writeNum("r2.val", 0);

    myNex.writeStr("Er1.txt", error_text);
}

void loop()
{

    if (PTT_status != HIGH)
        myNex.NextionListen();

    if (display_page == 0)
    {
        normalrun();
    }

    if (display_page == 1)
    {
        setup_onair(); // Check PTT status

        if (PTT_status)
        {

            maxcalibV = (analogRead(PWR_FWD));
            maxcalibV = (analogRead(PWR_FWD));
            if (maxcalibV > calibvoltADC)
                calibvoltADC = maxcalibV;
            myNex.writeNum("s9.val", calibvoltADC);
        }
        delay(100);
    }

    if (display_page == 2)
    {

        powerCalibration();
        if ((millis() - lastDiagRequest) >= DiagdelayInMillis)
        {

            raw_fwd = fwd_now();
            raw_ref = ref_now();

            if (raw_fwd > 20)
            { // only correct for diode voltage when more than zero
                power_fwd = raw_fwd + Vdiode;
                power_fwd = power_fwd * power_fwd;
                power_fwd = power_fwd / calibrationP;
            }
            else
                power_fwd = 0;

            if (raw_fwd > 20)
            { // only correct for diode voltage when more than zero
                power_ref = raw_ref + Vdiode;
                power_ref = power_ref * power_ref;
                power_ref = power_ref / calibrationP;
            }
            else
                power_ref = 0;

            if (power_ref > 0 && power_fwd > 0)
            {
                power_ratio = power_ref / power_fwd; // Theory corrected.

                SWR = abs((1 + sqrt(power_ratio)) / (1 - sqrt(power_ratio)));
            }
            else
            {
                SWR = 1;
                power_ratio = 0;
            }

            myNex.writeNum("D10.val", calibvoltADC);
            myNex.writeNum("D11.val", calibPower);
            myNex.writeNum("D9.val", calibrationP);
            myNex.writeNum("D0.val", raw_fwd);
            myNex.writeNum("D2.val", raw_ref);
            myNex.writeNum("D1.val", power_fwd);
            myNex.writeNum("D3.val", power_ref);
            int ratio = (power_ratio * 10) + 0.5;
            myNex.writeNum("Dx1.val", ratio);
            int swr = (SWR * 10) + 0.5;
            myNex.writeNum("Dx2.val", swr);
            int raw_V = analogRead(VCC);
            myNex.writeNum("D6.val", raw_V);
            int V = (((raw_V * ref_vdd) / (Res_2 / (Res_1 + Res_2))) * 10) + 0.5;
            myNex.writeNum("Dx3.val", V);
            int raw_I = analogRead(ID);
            myNex.writeNum("D7.val", raw_I);
            int I = ((raw_I * ref_vdd * 13000 / RL) * 10) + 0.5;
            myNex.writeNum("Dx4.val", I);
            int rotary = analogRead(ROTARY_POS);
            myNex.writeNum("D8.val", rotary);
            int rot_v = (rotary * ref_vdd * 10) + 0.5;
            myNex.writeNum("Dx5.val", rot_v);
            lastDiagRequest = millis();
        }
        display_page = 2;
    }
}

void normalrun()
{
    band_position(); // LPF Band position
    if (!alarm)
        onair(); // Check PTT status
    read_volt(); // Read Volt
    read_ID();   // Read current
    if (PTT_status)
    {
        read_power();        // Read Po and SWR
        display_power(true); // Display power
    }
    else
        display_power(false); // Display zero power

    read_temp(); // Read Temp and display
    error_i();   // Check Error: I
    error_swr(); // Check Error: SWR
    error_po();  // Check Error: Po
}
