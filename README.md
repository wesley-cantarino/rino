# Objetivo

O objetivo é ter uma plataforma onde seja possível visualizar as leituras dos sensores, os valores das variáveis e inclusive poder interferir e controlar as ações do robô.  Tudo isto seria usando a parte wifi do esp.

Durante a primeira semana procurei saber como gravar arquivos na memória flash do ESP tanto o 32 quanto o 8266 (2 excelentes artigos são este e este) mas em virtude do grau de complexidade e tendo em vista que não teria tempo para terminar, resolvi idealizar como poderia ser a parte de representação gráfica do robô. Primeiramente usando Processing mas futuramente mudar para javaScrip e assim poder rodar em um navegador.

Tendo isso em mente, peguei 1 esp8266 (nodeMCU), 1 giroscópio (GY-521 MPU6050) e 1 Sharp 20-150 cm (GP2Y0A02YK0F) e montei da seguinte forma:


[link para video](https://www.youtube.com/watch?v=HjF-_QqihHc)

![](https://github.com/wesley-cantarino/rino/blob/master/IMG/pronto.jpeg)

![](https://github.com/wesley-cantarino/rino/blob/master/IMG/foto_do_video.png)


![](https://github.com/wesley-cantarino/rino/blob/master/IMG/montagem_IMG.png)


![](https://github.com/wesley-cantarino/rino/blob/master/IMG/proces.png)

OBS: o nível lógico do ESP é 3.3V. o giroscópio trabalha com 3.3V mas já o Sharp tem a saída < 3.3V com uma alimentação 4.5V e 5.5V. logo, tive que usar alimentação externa.

Agora, podemos separa em 4 passos. 1º entender como funciona Sharp, 2º entender como funciona o giroscópio, 3º escrever código do ESP e 4º escrever código do Processing.

# 1º passo. 
Olhando o [datasheet do sharp](https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf) vemos que tem uma saída não linear que e dado pela forma y = a * x^b. felizmente minha calculadora e consegue calcular o valor de a e de b. ![](https://github.com/wesley-cantarino/rino/blob/master/IMG/saida.jpg)
	Escolhendo os valores de tensão como sendo meu x e y minha distância. E usando 
        <br>V   ___  dist<br>
        <br>2.5 ___   20<br>
        <br>2.0 ___   30<br>
        <br>1.5 ___   40 <br>
        <br>1.0 ___    60<br>
        <br>0.5 ___   130<br>
Temos a = 60.7 e b = -1.12
No código tempos DIST = 60.7 * pow(X, -1.12). Onde X e dado por ((valor de leitura) /1023) * 3.3 dessa forma garantimos que valor X será dado em Volts.

[No código](https://github.com/wesley-cantarino/rino/blob/master/code_reference/sketch_sharp/sketch_sharp.ino)


# 2º passo.
O giroscópio segue o protocolo i2c e é bem chatinho de se fazer as leituras. Usei esses 2 sites como referência.
[este como principal.](https://www.instructables.com/id/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/)
[Já este como auxiliar](https://www.instructables.com/id/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/)

Tendo como [resultado este código.](https://github.com/wesley-cantarino/rino/blob/master/code_reference/sketch_gyroscope/sketch_gyroscope.ino) 


# 3º passo
Agora já estamos na parte final da programação do ESP. Juntei código de leituras que foi construído nos passos anteriores e imprimir eles usando serial.print. Agora, a grande estratégia é como organizar esses dados para depois quando for fazer a leitura, eles já estiverem organizados. Minha organização é imprimir tipo (um número inteiro) mais “,” então o valor que é o dado e “;” para informar que acabou.  
Então o código ficou da [seguinte forma.](https://github.com/wesley-cantarino/rino/blob/master/sketch_3D_visor/esp8266_3D_visor/esp8266_3D_visor.ino)


# 4ºpasso
A parte rosada/vermelho claro é a abertura do sensor sharp esse paralelepípedo seria a estrutura onde o sensor está apoiado (neste caso o robô). 

É possível mudar a visão do observador mexendo mouse. O desenho se inclina de acordo com a leitura do giroscópio e o arco aumenta ou diminui de acordo com a distância medida.
[Código usado no processing](https://github.com/wesley-cantarino/rino/blob/master/sketch_3D_visor/processing_3D_visor/processing_3D_visor.pde)

Da mesmoa forma que foi feito para 1 sensor. É fácil aumentar para 2 ou mais sensores ou então imprimir na tela em qual if o código entrou.



