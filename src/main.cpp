/* -------READ-------
  Resumo: 
  Luva para captar a vibração durante a aplicação da vibroterápia.
  Vibração pulmonar deve ocorrer em torno de 12 a 16 Hz
  Vibração manual 12 a 20 Hz
  Vibração automática 12 a 16 Hz
  Descrição:
  O sensor de pressão capta quando o aplicador pressiona o tórax 
  do paciente, uma vez precionado o sensor MPU capta então a vibração 
  do pulmão durante a aplicação da vibroterápia e entrega:
   - Frequência instantânea
   - Tempo de aplicação
   - Frequência média da aplicação  
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <arduinoFFT.h>
#include <BluetoothSerial.h>

// função que inicializa o bluetooth do ESP32
#if !defined (CONFIG_BT_ENABLED) || !defined (CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define SAMPLES 64              // Qntd de amostras para o FFT.
#define SAMPLING_FREQUENCY 200  // Hz, intervalo para o FFT.
#define Pin_RES 27              // Pino para sensor de pressão (resistivo)
#define LED_ON 2                // Pino para ligar o LED do ESP32

// Variáveis globais
BluetoothSerial Bluetooth;
// Biblioteca para calcular FFT e variáveis
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vals[SAMPLES], img[SAMPLES];
// Config. MPU6050
/* Fator de escala do acelerômetro MPU6050
  Full-Scale Range: /  Sensitivity Scale
        2g          /       16384
        4g          /       8192
        8g          /       4096
        16g         /       2048  
*/
//Endereco I2C do MPU6050
const int MPU = 0x68;
//variáveis para acelerometro e giroscópio.
int eX, eY, eZ, Tmp, GiX, GiY, GiZ;
long int time_init, time_range; // tempo de início da aplicação
double med_freq = 0;   // contador de amostras do range
double freq = 0;    // frequencia da aplicação
int cont = 0;       // contador de amostras
//variável para sensor de pressão.
int pressao = 0;
bool flag = false;
///////////////// Define ajusts /////////////////

////////////////  Functions Declare //////////////////

double    SolicMPU6050();
void      SeriallPrintS();
void      SetaFlag();
void      Finaliza();

////////////////  SETUP  //////////////////

void setup() {
  // configura BluetoothSerial
  Bluetooth.begin("ESP32_Fisio");
  Serial.begin(9600);   // Default speed in HC-06 modules
  // Serial.println(F("Type the AT commands:"));
  // seta pino 2 como saída
  pinMode(LED_ON, OUTPUT);
  digitalWrite(LED_ON, HIGH);
  delay(2000);
  digitalWrite(LED_ON, LOW);
  sampling_period_us = 25;//round(100000*(1.0/SAMPLING_FREQUENCY)); // intervalo para FFT
  // configurar MPU6050
  // configura a escala de variação do giroscópio em graus/segundos
  Wire.begin();
  // Inicializa o MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);
  Wire.endTransmission(true);
  delay(10);
  // seta FS_CEL em 3, referente a escala de 16g
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(3);
  Wire.endTransmission(true);
  delay(100);  
  // configura interrupção
  //attachInterrupt(Pin_RES, SetaFlag, RISING);
}
// loop 

////////////////  LOOP  //////////////////
/* Requisitos para loop
  Realizar a amostragem da frequência de vibração do pulmão, sempre que o sensor de pressão
  for pressionado por 1s. Finalizar após ser precionado por 1s novamente, para indicar que
  que está realziando a leitura, ligar o led 13 do ESP32*/


void loop(){
  if(flag){ // Flag ligada, inicia amostragem até a pressão indicando fim
    Serial.println("Iniciando leitura!");
    // capta as 64 amostras do sensor MPU6050
    for (int x = 0 ; x < SAMPLES ; x++){
      microseconds = micros();    // recebe tempo atual em microsegundos
      vals[x] = SolicMPU6050();  // solicita leitura do sensor MPU6050
      img[x] = 0;                 // inicializa imagem do FFT
      // espera o intervalo de amostragem
      while(micros() - microseconds < sampling_period_us){}
    }
    // calcula o FFT
    // define o tipo de janela
    FFT.Windowing(vals, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); 
    // calcula o FFT
    FFT.Compute(vals, img, SAMPLES, FFT_FORWARD);              
    // converte para magnitude  
    FFT.ComplexToMagnitude(vals, img, SAMPLES);                 
    // calcula o pico máximo do FFT
    freq = FFT.MajorPeak(vals, SAMPLES, SAMPLING_FREQUENCY);   
    // incrementa o contador de amostras
    cont++;
    // soma a frequência para calcular a média
    med_freq += freq; 
    Serial.print("Freq = "); 
    Serial.println(freq);
    delay(1);
    long int tempo = millis();
    while(analogRead(Pin_RES) > 40){
      // aguarda o fim da pressão
      if(tempo - millis() > 500){
        flag = false;
        Finaliza();
        delay(3000);
        break;
      }
    }

  } else{
    // flag desligada, escuta pressão para ligar flag e iniciar leitura
    Serial.println("Aguardando Inicialização!");
    Bluetooth.println("Aguardando Inicialização!");
    long int tempo = millis();
    while(analogRead(Pin_RES) > 40){
      Serial.println("Tempo: " + String((millis() - tempo)/1000) + "s");
      
      if(tempo - millis() > 3000 && flag == false){
        flag = true;
        digitalWrite(LED_ON, HIGH);
        time_init = millis(); 
        med_freq = 0;   
        delay(50);
      }
    }
  }
  delay(200); // delay para não sobrecarregar o ESP32
}

////////////////  FUNCTIONS  //////////////////

double SolicMPU6050(){
  double result;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  // Solicita os dados do sensor
  Wire.requestFrom(MPU, 14, true);
  // Armazena o valor dos sensores nas variaveis correspondentes
  eX = Wire.read()<<8|Wire.read();//0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  eY = Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  eZ = Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read()<<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GiX = Wire.read()<<8|Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GiY = Wire.read()<<8|Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GiZ = Wire.read()<<8|Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // Converte valores acelerometro e giroscopio para graus/segundos
  result = sqrt(pow(eX/2048, 2) + pow(eY/2048, 2) + pow(eZ/2048, 2)); // calcula a resultante
  return result;
}

void Finaliza(){
  // quando interromper a leitura, precisa gerar a média
  // calcula o tempo de aplicação
  time_range = millis() - time_init;
  // calcula média da frequência
  med_freq = med_freq / cont;
  cont = 0;
  // retorna o tempo de aplicação em segundo!
  Serial.print("Tempo de aplicação: ");
  Serial.print(time_range/1000);
  Serial.println(" segundos");
  Serial.print("Frequência média: ");
  Serial.println(med_freq);
  Bluetooth.print("Tempo de aplicação: ");
  Bluetooth.print(time_range/1000);
  Bluetooth.println(" segundos");
  Bluetooth.print("Frequência média: ");
  Bluetooth.println(med_freq);
  med_freq = 0;
  digitalWrite(LED_ON, LOW);
  delay(1000);
}

void SetaFlag(){
  if(flag){ // Flag ligada, finaliza amostragens e desliga flag!
    // quando interromper a leitura, precisa gerar a média
    // calcula o tempo de aplicação
    time_range = millis() - time_init;
    // calcula média da frequência
    med_freq = med_freq / cont;
    cont = 0;
    // retorna o tempo de aplicação em segundo!
    Serial.print("Tempo de aplicação: ");
    Serial.print(time_range/1000);
    Serial.println(" segundos");
    Serial.print("Frequência média: ");
    Serial.println(med_freq);
    med_freq = 0;
    delay(10);
    digitalWrite(LED_ON, LOW);
  } else { 
    // Liga flag para iniciar amostragem após 3s de pressão!
    long int tempo = millis();
    while(analogRead(Pin_RES) > 30){
      if(tempo - millis() > 3000){
        flag = true;
        digitalWrite(LED_ON, HIGH);
        time_init = millis(); 
        med_freq = 0;   
        delay(50);
        break;
      }
    }
  }
}
