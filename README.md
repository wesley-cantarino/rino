O objetivo é ter uma plataforma onde seja possível visualizar as leituras dos sensores, os valores das variáveis e inclusive poder interferir e controlar as ações do robô.  Tudo isto seria usando a parte wifi do esp.
Durante a primeira semana procurei saber como gravar arquivos na memória flash do ESP tanto o 32 quanto o 8266 (2 excelentes artigos são este e este) mas em virtude do grau de complexidade e tendo em vista que não teria tempo para terminar, resolvi idealizar como poderia ser a parte de representação gráfica do robô. Primeiramente usando Processing mas futuramente mudar para javaScrip e assim poder rodar em um navegador.
Tendo isso em mente, peguei 1 esp8266 (nodeMCU), 1 giroscópio (GY-521 MPU6050) e 1 Sharp 20-150 cm (GP2Y0A02YK0F) e montei da seguinte forma:
Foto geral:
Foto do fritzing:
OBS: o nível lógico do ESP é 3.3V. o giroscópio trabalha com 3.3V mas já o Sharp tem a saída < 3.3V com uma alimentação 4.5V e 5.5V. logo, tive que usar alimentação externa.

Agora, podemos separa em 4 passos. 1º entender como funciona Sharp, 2º entender como funciona o giroscópio, 3º escrever código do ESP e 4º escrever código do Processing.

1º passo. 
	Olhando o datasheet do sharp (https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf) vemos que tem uma saída não linear que e dado pela forma y = a * x^b. felizmente minha calculadora e consegue calcular o valor de a e de b. 
	Escolhendo os valores de tensão como sendo meu x e y minha distância. E usando 
         V   ___  dist
        2.5 ___   20
        2.0 ___   30
        1.5 ___   40 
        1.0 ___    60
        0.5 ___   130
Temos a = 60.7 e b = -1.12
No código tempos DIST = 60.7 * pow(X, -1.12). Onde X e dado por ((valor de leitura) /1023) * 3.3 dessa forma garantimos que valor X será dado em Volts.
	2º passo.
	O giroscópio segue o protocolo i2c e é bem chatinho de se fazer as leituras. Usei esses 2 sites como referência.
	https://www.instructables.com/id/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/
https://www.instructables.com/id/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/

Tendo como resultado este código. Auxiliar 


3º passo
Agora já estamos na parte final da programação do ESP. Juntei código de leituras que foi construído nos passos anteriores e imprimir eles usando serial.print. Agora, a grande estratégia é como organizar esses dados para depois quando for fazer a leitura, eles já estiverem organizados. Minha organização é imprimir tipo (um número inteiro) mais “,” então o valor que é o dado e “;” para informar que acabou.  
Então o código ficou da seguinte forma.

4ºpasso
A parte rosada/vermelho claro é a abertura do sensor sharp esse paralelepípedo seria a estrutura onde o sensor está apoiado (neste caso o robô). 
É possível mudar a visão o observador a visão o observador mexendo mouse. O desenho se inclina de acordo com a leitura do giroscópio e o arco aumenta ou diminui de acordo com a distância medida.
[Código usado no processing]

