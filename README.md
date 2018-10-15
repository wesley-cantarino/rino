# Objetivo

O objetivo é ter uma plataforma onde seja possível visualizar as leituras dos sensores, os valores das variáveis e inclusive poder interferir e controlar as ações do robô.  Tudo isto seria usando a parte wifi do esp.

Durante a primeira semana procurei saber como gravar arquivos na memória flash do ESP tanto o 32 quanto o 8266 (2 excelentes artigos são [este](https://tttapa.github.io/ESP8266/Chap11%20-%20SPIFFS.html) e [este](http://esp8266.github.io/Arduino/versions/2.0.0/doc/filesystem.html#file-system-object-spiffs)) mas em virtude do grau de complexidade e tendo em vista que não teria tempo para terminar, resolvi idealizar como poderia ser a parte de representação gráfica (da visão) do robô. Primeiramente usando Processing mas futuramente mudar para javaScrip e assim poder rodar em um navegador.

Tendo isso em mente, peguei 1 esp8266 (nodeMCU), 1 giroscópio (GY-521 MPU6050) e 1 Sharp 20-150 cm (GP2Y0A02YK0F) e montei da seguinte forma:


**[link para video](https://www.youtube.com/watch?v=HjF-_QqihHc)**

![](https://github.com/wesley-cantarino/rino/blob/master/IMG/pronto.jpeg)

![](https://github.com/wesley-cantarino/rino/blob/master/IMG/foto_do_video.png)


![](https://github.com/wesley-cantarino/rino/blob/master/IMG/montagem_IMG.png)


![](https://github.com/wesley-cantarino/rino/blob/master/IMG/proces.png)

OBS: o nível lógico do ESP é 3.3V. o giroscópio trabalha com 3.3V mas já o Sharp tem a saída < 3.3V com uma alimentação 4.5V e 5.5V. logo, tive que usar alimentação externa.

Agora, podemos separa em 4 passos. 1º entender como funciona Sharp, 2º entender como funciona o giroscópio, 3º escrever código do ESP e 4º escrever código do Processing.

# 1º passo. 
Olhando o [datasheet do sharp](https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf) vemos que tem uma saída não linear que e dado pela forma y = a * x^b. felizmente minha calculadora e consegue calcular o valor de a e de b. ![](https://github.com/wesley-cantarino/rino/blob/master/IMG/saida.jpg)

Escolhendo os valores de tensão como sendo meu x e y minha distância. E usando 

         V  ___   dist
        2.5 ___   20
        2.0 ___   30
        1.5 ___   40 
        1.0 ___   60
        0.5 ___   130
Temos a = 60.7 e b = -1.12<br>

No código tempos DIST = 60.7 * pow(X, -1.12). Onde X e dado por ((valor de leitura) /1023) * 3.3 dessa forma garantimos que valor X será dado em Volts.


[No código](https://github.com/wesley-cantarino/rino/blob/master/code_reference/sketch_sharp/sketch_sharp.ino)

```c
#include "Arduino.h"

const int sharp = A0;

void setup (){
    Serial.begin(115200);
}

void loop (){
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
```

# 2º passo.
O giroscópio segue o protocolo i2c e é bem chatinho de se fazer as leituras. Usei esses 2 sites como referência.<br>
[Este como principal.](https://www.instructables.com/id/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/) <br>
[Já este como auxiliar](https://www.instructables.com/id/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/)

Tendo como [resultado este código.](https://github.com/wesley-cantarino/rino/blob/master/code_reference/sketch_gyroscope/sketch_gyroscope.ino) 

```c
#include "Arduino.h"
#include "Wire.h" // This library allows you to communicate with I2C devices.
const int MPU_ADDR = 0x69; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data

int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.println(convert_int16_to_str(accelerometer_z));

  // delay
  delay(500);
}
```

# 3º passo
Agora já estamos na parte final da programação do ESP. Juntei código de leituras que foi construído nos passos anteriores e imprimir eles usando serial.print. Agora, a grande estratégia é como organizar esses dados para depois quando for fazer a leitura, eles já estiverem organizados. Minha organização é imprimir tipo (um número inteiro) mais “,” então o valor que é o dado e “;” para informar que acabou.  
Então o código ficou da [seguinte forma.](https://github.com/wesley-cantarino/rino/blob/master/sketch_3D_visor/esp8266_3D_visor/esp8266_3D_visor.ino)

```c
#include "Arduino.h"

const int sharp = A0;
        
/*******gyro***********/
 // This library allows you to communicate with I2C devices
 #include "Wire.h" 

 // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
 const int MPU_ADDR = 0x69;

 // variables for accelerometer raw data
 int16_t accelerometer_x, accelerometer_y, accelerometer_z;

 int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
 int16_t temperature; // variables for temperature data
 char tmp_str[7]; // temporary variable used in convert function

 /*  converts int16 to string. Moreover, resulting strings
  *   will have the same length in the debug monitor.
 */
 char* convert_int16_to_str(int16_t i) { 
   sprintf(tmp_str, "%6d", i);
   return tmp_str;
 }
/**********************/

void setup (){
    Serial.begin(115200);

    /******wire_conf*******/
     Wire.begin();
     Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
     Wire.write(0x6B); // PWR_MGMT_1 register
     Wire.write(0); // set to zero (wakes up the MPU-6050)
     Wire.endTransmission(true);
    /**********************/
}

void loop (){
    calc_dist ();
    calc_gyro();

    delay(500);
}

void calc_dist (){
    float R = analogRead(sharp);
    float X = R/(1023 - 0)*(3.3 - 0) + 0;
    float DIST = 60.7 * pow(X, -1.12);

    Serial.print("1"); Serial.print(",");
    Serial.print(DIST); Serial.print(";");
}

void calc_gyro (){
    Wire.beginTransmission(MPU_ADDR);

    // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.write(0x3B); 

    // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

    //"Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    //reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_x = Wire.read()<<8 | Wire.read(); 
    //reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_y = Wire.read()<<8 | Wire.read(); 
    //reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    accelerometer_z = Wire.read()<<8 | Wire.read(); 

    // print out data
    Serial.print("2"); Serial.print(",");
    Serial.print(convert_int16_to_str(accelerometer_x)); Serial.print(";");

    Serial.print("3"); Serial.print(",");
    Serial.print(convert_int16_to_str(accelerometer_y)); Serial.print(";");

    Serial.print("4"); Serial.print(",");
    Serial.print(convert_int16_to_str(accelerometer_z)); Serial.print(";");
}
```

# 4ºpasso
A parte rosada/vermelho claro é a abertura do sensor sharp esse paralelepípedo seria a estrutura onde o sensor está apoiado (neste caso o robô). 

Da mesmoa forma que foi feito para 1 sensor. É fácil aumentar para 2 ou mais sensores ou então imprimir na tela em qual if o código entrou.

O processing permite exportar o código como sendo um arquivo executável.

É possível mudar a visão do observador mexendo mouse. O desenho se inclina de acordo com a leitura do giroscópio e o arco aumenta ou diminui de acordo com a distância medida.
[Código usado no processing](https://github.com/wesley-cantarino/rino/blob/master/sketch_3D_visor/processing_3D_visor/processing_3D_visor.pde)

```processing
import peasy.*;  //para poder rotacionar a visão do observador no R3
PeasyCam cam;

/****************/
import processing.serial.*;
Serial myPort;
String one ="";
String dado ="";
String data="";
int index1=0;

int tipo;
float DADO;
/****************/

float dist = 0, X = 0, Y = 0, Z = 0;
int escala = 10;  /* escala = tamanho_desenho/tamanho_real */

void setup (){
  surface.setTitle("draw");
  size(1280, 720, P3D);

  myPort = new Serial(this, Serial.list()[0], 115200);

  lights();
  cam = new PeasyCam(this, width);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(5000);

  colorMode(HSB);
}

void draw (){
  background (#e1e1e1);

  if(myPort.available() > 10){
    data = myPort.readStringUntil(';');
    data = data.substring(0, data.length()-1);

    index1 = data.indexOf(",");
    one = data.substring(0, index1);
    dado = data.substring(index1+1, data.length());

    tipo = int(one);
    DADO = float(dado);

    if(tipo == 1)
      dist = DADO;
    else if(tipo == 2)
      X = DADO;
    else if(tipo == 3)
      Y = DADO;
    else if(tipo == 4)
      Z = DADO;

    //println(dist + " _ " + X + " _ " + Y + " _ " + Z);
  }

  pushMatrix();
    rotateY(radians(map(X, -16000, 16000, -90, 90)));
    rotateX(radians(map(Y, -16000, 16000, -90, 90)));

    draw_object();
  popMatrix();
}

void draw_object (){
  /*****draw_box****/
  fill(#bbb5b5);
  stroke(#555353);
  box(10 * escala, 10 * escala, 5 * escala);
  /*****************/

  /*****draw_arc****/
  fill(#ffb691);
  noStroke();

  arc(0, 0, 2*dist*escala, 2*dist*escala, radians(-15), radians(15), OPEN);
  beginShape();
    vertex(0, 0);
    vertex((dist*escala)*cos(radians(15)), -(dist*escala)*sin(radians(15)));
    vertex((dist*escala)*cos(radians(15)), (dist*escala)*sin(radians(15)));
  endShape(OPEN);

  fill(#050606);
  textSize(20);
  textAlign(CENTER, CENTER);
  pushMatrix();
    rotate(PI/2);
    text(int(dist) + "cm", 0, -(dist*escala + 20));
  popMatrix();
  /*****************/
}
```





