#include "ADS1x15.h"

#define ADS1X15_ADDRESS                           0x48

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


int main(){

    if (!ads.begin(0, ADS1X15_ADDRESS)) {
       
        while (1)
            ;
    }

    while (1)
    {
        uint16_t adc0, adc1, adc2, adc3;
        float volts0, volts1, volts2, volts3;

        adc0 = ads.readADC_SingleEnded(0);
        adc1 = ads.readADC_SingleEnded(1);
        adc2 = ads.readADC_SingleEnded(2);
        adc3 = ads.readADC_SingleEnded(3);

        volts0 = ads.computeVolts(adc0);
        volts1 = ads.computeVolts(adc1);
        volts2 = ads.computeVolts(adc2);
        volts3 = ads.computeVolts(adc3);

        printf("adc0:%d volts0 %.2f\t|", adc0, volts0);
        printf("adc1:%d volts1 %.2f\t|", adc1, volts1);
        printf("adc2:%d volts2 %.2f\t|", adc2, volts2);
        printf("adc3:%d volts3 %.2f\n", adc3, volts3);

       // usleep(1E6);
    }
    return 0;
}