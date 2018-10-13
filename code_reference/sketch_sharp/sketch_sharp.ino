#include "Arduino.h"

const int sharp = A0;

void setup (){
    Serial.begin(115200);
}

void loop (){
    /*  dist = a*X^b  onde X eh a tensao
    *   usando os dados
    *      V    dis
    *     2.5   20
    *     2.0   30
    *     1.5   40 
    *     1.0   60
    *     0.5   130
    *  
    *   temos a = 60.7
    *         b = -1.12
    */

    float R = analogRead(sharp);
    float X = R/(1023 - 0)*(3.3 - 0) + 0;
    float DIST = 60.7 * pow(X, -1.12);


    Serial.print(R);
    Serial.print("____");
    Serial.print(X);
    Serial.print("____");
    Serial.println(DIST);

    delay(200);
}