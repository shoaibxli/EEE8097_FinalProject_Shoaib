#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <onewire.h>

#define NUM_SAMPLES 10  // Number of samples for averaging

uint16_t adcValues[NUM_SAMPLES] = {0};  // Array to store ADC samples
uint8_t currentIndex = 0;               // Current index in the ADC sample array
uint8_t ignoreCount = 10;   // Counter for initial samples to ignore

uint8_t sensorN[8] = {0x28, 0x39, 0x14, 0x1c, 0x0e, 0x00, 0x00, 0xd8};
uint8_t sensorE[8] = {0x28, 0x16, 0x4c, 0x1d, 0x0e, 0x00, 0x00, 0x6e};
uint8_t sensorW[8] = {0x28 ,0x2c, 0xd6 ,0x1c ,0x0e ,0x00 ,0x00 ,0xc9};
uint8_t sensorS[8] = {0x28, 0xF3, 0x6A, 0x1c, 0x0e, 0x00, 0x00, 0x47};
uint8_t sensorNeck[8] = {0x28, 0x44, 0x50, 0x1c, 0x0e, 0x00, 0x00, 0xfc};
uint8_t sensorB[8] = {0x28, 0x03, 0x84, 0x1c, 0x0e, 0x00, 0x00, 0x4D};


float tempS, tempW, tempE, tempN, tempNeck, tempB, tempAvg;

float setPoint = 28.0; // Target temperature in Celsius
int heaterPWM = 0;
// PID Coefficients
float kp_temp = 0.95, ki_temp = 0.05, kd_temp = 0.01; // Temperature PID coefficients
float kp_volt = 7.0, ki_volt = 2.0, kd_volt = 0; // Voltage PID coefficients

// PID States
float integral_temp = 0, previousError_temp = 0;
float integral_volt = 0, previousError_volt = 0;

char bufferN[20],bufferE[20],bufferW[20],bufferS[20],bufferNeck[20],bufferB[20],bufferAvg[20],bufferPWM[20],bufferV[20],bufferTime[20],bufferFBV[20];

float time = 0;

int primaryPidController(float currentTemp);
int secondaryPidController(float setVolt, float actualVoltage);
void initClock();
void initPWM();
void setPWMDutyCycle(unsigned int dutyCycle);
void ser_output(char *str);
void initUART();
void initADC(void);
uint16_t readADC(void);
uint16_t getAverageADC(void);
void setResolutions();
void printValues();


void main(void)
{
    initClock(); // Initiate 1Mhz Clock
    initPWM(); // Initiate PWM on PIN 1.3
    initUART();
    initADC();
    setResolutions();

//    heaterPWM = 128*.75;
//    setPWMDutyCycle(heaterPWM);
//    int direction = -2; // 1 for increasing brightness, -1 for decreasing brightness

    while (1)
    {
        setPWMDutyCycle(0); // Pause PWM

        //   Temperature Reading
        tempN = ds_getTemp(sensorN); // Read Temperature from sensor 1 south
        tempE = ds_getTemp(sensorE); // Read Temperature from sensor 2 west
        tempW = ds_getTemp(sensorW); // Read Temperature from sensor 3 east
        tempS = ds_getTemp(sensorS); // Read Temperature from sensor 4 north
        tempNeck = ds_getTemp(sensorNeck); // Read Temperature from sensor 1 south
        tempB = ds_getTemp(sensorB); // Read Temperature from sensor 2 west
        tempAvg = (tempN+tempE+tempW+tempS+tempNeck+tempB)/6; // Average Temperature
//        tempAvg = 26;

        // PID Calculations
        // PRIMARY PID:  sets PWM according to target temperature
        heaterPWM = primaryPidController(tempAvg);
        setPWMDutyCycle(heaterPWM); // Update new PWM

//        heaterPWM += direction;
//        if (heaterPWM == 128 || heaterPWM == 0)
//        {
//            direction = -direction; // Reverse direction at limits
//        }

        // SECONDARY PID: sets new PWM according to target voltage
        float heaterVolt = ((float)heaterPWM/ 128) * 12.0; // PWM value (0 to 1027) back to actual voltage (0 to 12V)

        __delay_cycles(1000000); // wait 1sec before reading ADC
        uint16_t adcValue = readADC(); // Primary set volt
        float measuredVoltage = 12 - ((adcValue * 12) / 4095.0); // Convert ADC reading to voltage


        int heaterPWMadj = secondaryPidController(heaterVolt, measuredVoltage); // SECONDARY PID
        setPWMDutyCycle(heaterPWMadj); // Update new PWM

        //check new voltage accuracy
        __delay_cycles(1000000); // wait 1sec before reading adc
        uint16_t newAdcValue = readADC(); // Secondary new volt
        float newMeasuredVoltage =  12 - ((newAdcValue * 12) / 4095.0); // Convert ADC reading to voltage

//        printValues();
        // print for graphs
        sprintf(bufferAvg, "%.2f %.1f \t", tempAvg, time); // Temperature x Time
//        sprintf(bufferPWM, "%d %.2f \t \r\n", heaterPWM, heaterVolt); // PWM x Voltage
        sprintf(bufferPWM, "%d %.2f \t %.2f \t %d %.2f\r\n", heaterPWM, heaterVolt, measuredVoltage, heaterPWMadj, newMeasuredVoltage); // PWM x Voltage
//        sprintf(bufferPWM, "Expected Voltage = %.2f \t Measured Voltage = %.2f \r\n", heaterVolt, newMeasuredVoltage); // PWM x Voltage

        ser_output(bufferAvg);
        ser_output(bufferPWM);

//         Time interval increment
        time += 5.6;
        __delay_cycles(3000000); // Adjust delay for test speed
    }
}

int primaryPidController(float currentTemp)
{
    float error = setPoint - currentTemp;

    // Proportional term
    float P = kp_temp * error;

    // Integral term
    integral_temp += error;
    float I = ki_temp * integral_temp;
    // Clamp integral to avoid windup
    if (integral_temp > 164) integral_temp = 164;
    if (integral_temp < 0) integral_temp = 0;

    // Derivative term
    float D = kd_temp * (error - previousError_temp);

    // Calculate total output
    float output = P + I + D;

    // Clamp the output to the range 0-127
    if (output > 12) output = 12;
    if (output < 0) output = 0;

    output = ((output / 12.0) * 128);

    previousError_temp = error;

    return output;
}

int secondaryPidController(float setVolt, float actualVoltage)  // Secondary PID Controller: PWM to Voltage Adjustment
{
//    float expectedVoltage = (setPwm / 2047.0) * 12.0;  // Scale PWM to expected voltage (0-12V)
    float error = setVolt - actualVoltage;

    // Proportional term
    float P = kp_volt * error;

    // Integral term
    integral_volt += error;
    float I = ki_volt * integral_volt;
//     Clamp integral to avoid windup
    if (integral_volt > 0.75) integral_volt = .75;
    if (integral_volt < 0) integral_volt = 0;

    // Derivative term
    float D = kd_volt * (error - previousError_volt);

    // Calculate total output adjustment
    int output = heaterPWM + P + I + D;

    // Clamp the output to the range 0-127
    if (output > 128) output = 128;
    if (output < 0) output = 0;

    previousError_volt = error;

//    // Debugging output
//    sprintf(bufferPWM, "Error: %.2f, P: %.2f, I: %.2f, D: %.2f, Output PWM: %d\n", error, P, I, D, output);
//    ser_output(bufferPWM);

    return output;
}

void initClock()
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // Disable GPIO high-impedance mode

    // Configure DCO to 1 MHz
    CSCTL0_H = CSKEY >> 8; // Unlock CS registers
    CSCTL1 = DCOFSEL_0 | DCORSEL; // Set DCO to 1 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // Set ACLK = VLO; SMCLK = DCO; MCLK = DCO
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // Set all dividers to 1
    CSCTL0_H = 0; // Lock CS registers
}

void initPWM()
{
    // Configure P1.3 as PWM output
    P1DIR |= BIT3; // Set P1.3 as output
    P1SEL0 |= BIT3; // Select Timer_A function for P1.3
    P1SEL1 &= ~BIT3; // Ensure correct function selection

    // Configure Timer_A1.2
    TA1CCR0 = 128-1; // PWM Period
    TA1CCTL2 = OUTMOD_7; // CCR2 reset/set
    TA1CCR2 = 0; // CCR2 PWM duty cycle (0%)
    TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;// SMCLK, up mode, clear TAR
}

void setPWMDutyCycle(unsigned int dutyCycle)
{
    TA1CCR2 = dutyCycle; // Set PWM duty cycle
}

void ser_output(char *str)
{
     while (*str != 0) {
         while (!(UCA0IFG & UCTXIFG)); // Wait for TX buffer to be ready
         UCA0TXBUF = *str++;           // Send character
     }
}

void initUART()
{
    // Configure UART pins
    P2SEL1 |= BIT0 | BIT1;    // Set P2.0 and P2.1 as UART pins
    P2SEL0 &= ~(BIT0 | BIT1);

    // Configure UART
    UCA0CTLW0 |= UCSWRST;     // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK
    UCA0BR0 = 52;              // 1MHz 9600 (see User's Guide)
    UCA0BR1 = 0;              // 1MHz 9600
    UCA0MCTLW = UCBRS5 | UCBRF0; // Modulation
    UCA0CTLW0 &= ~UCSWRST;    // Initialize eUSCI
    UCA0IE |= UCRXIE;
}

void initADC(void)
{
    P1SEL0 |= BIT5; // Set P1.5 to ADC function
    ADC12CTL0 &= ~ADC12ENC; // Disable ADC to configure
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON; // Sampling time, ADC on
    ADC12CTL1 = ADC12SHP; // Use sampling timer
    ADC12CTL2 = ADC12RES_2; // 12-bit conversion results
    ADC12MCTL0 = ADC12INCH_5; // A5 ADC input select; Vref=AVCC
    ADC12CTL0 |= ADC12ENC; // Enable ADC
}

uint16_t readADC(void)
{
    ADC12CTL0 |= ADC12SC; // Start conversion
    while (ADC12CTL1 & ADC12BUSY); // Wait for conversion to complete
    return ADC12MEM0;
}

uint16_t getAverageADC(void)
{
    uint32_t sum = 0;
    adcValues[currentIndex] = readADC();  // Read new ADC value and store it in the array

    currentIndex = (currentIndex + 1) % NUM_SAMPLES;  // Update the current index
    uint8_t i;
    for (i = 0; i < NUM_SAMPLES; i++) {
        sum += adcValues[i];
    }

    return (uint16_t)(sum / NUM_SAMPLES);  // Return the average value
}

void setResolutions()
{
    /*9-bit: 0x1F (Resolution: 0.5°C, Conversion Time: 93.75ms), 10-bit: 0x3F (Resolution: 0.25°C, Conversion Time: 187.5ms)
    11-bit: 0x5F (Resolution: 0.125°C, Conversion Time: 375ms),    12-bit: 0x7F (Resolution: 0.0625°C, Conversion Time: 750ms)*/
    ds_set_resolution(sensorN, 0x1F); // Set 9-bit resolution (0x1F)
    ds_set_resolution(sensorE, 0x1F); // Set 9-bit resolution (0x1F)
    ds_set_resolution(sensorW, 0x1F); // Set 9-bit resolution (0x1F)
    ds_set_resolution(sensorS, 0x1F); // Set 9-bit resolution (0x1F)
    ds_set_resolution(sensorNeck, 0x1F); // Set 9-bit resolution (0x1F)
    ds_set_resolution(sensorB, 0x1F); // Set 9-bit resolution (0x1F)
}

void printValues()
{
    // Printing all values
    sprintf(bufferN, "%.2f", tempN);
    ser_output("North Temp = ");
    ser_output(bufferN);
    ser_output("C\r\n");

//    sprintf(bufferE, "%.2f", tempE);
//    ser_output("East Temp = ");
//    ser_output(bufferE);
//    ser_output("C\r\n");
//
//    sprintf(bufferW, "%.2f", tempW);
//    ser_output("West Temp = ");
//    ser_output(bufferW);
//    ser_output("C\r\n");
//
//    sprintf(bufferS, "%.2f", tempS);
//    ser_output("South Temp = ");
//    ser_output(bufferS);
//    ser_output("C\r\n");
//
//    sprintf(bufferNeck, "%.2f", tempNeck);
//    ser_output("Neck Temp = ");
//    ser_output(bufferNeck);
//    ser_output("C\r\n");
//
//    sprintf(bufferB, "%.2f", tempB);
//    ser_output("Base Temp = ");
//    ser_output(bufferB);
//    ser_output("C\r\n\n");
//
//    sprintf(bufferAvg, "%.2f", tempAvg);
//    ser_output("Average Temp = ");
//    ser_output(bufferAvg);
//    ser_output("C\r\n\n");

//    sprintf(bufferPWM, "%d", heaterPWM);
//    ser_output("Heater PWM = ");
//    ser_output(bufferPWM);
//    ser_output("\r\n");
//
//    sprintf(bufferV, "%.2f", heaterVolt);
//    ser_output("Heater Voltage = ");
//    ser_output(bufferV);
//    ser_output("V\r\n");
//
//    sprintf(bufferTime, "%d", time);
//    ser_output("Time = ");
//    ser_output(bufferTime);
//    ser_output("\r\n\n");
}

// Read ADC Value
//        while (ignoreCount > 0) {
//            getAverageADC();  // Ignore the value
//            ignoreCount--;
//        }
//        uint16_t adcValue = getAverageADC();
//        float measuredVoltage = (adcValue * 3.3) / 4095.0; // Convert ADC reading to voltage
//        sprintf(bufferFBV, "%.2f\r\n", measuredVoltage);
//        ser_output(bufferFBV);
