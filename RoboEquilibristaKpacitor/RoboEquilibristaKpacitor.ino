/***********************************************************************
 * 
 *  ROBO EQUILIBRISTA ESCOLA KPACITOR
 *  
 *  MINICURSO: CONSTRUINDO UM ROBÔ EQUILIBRISTA
 * 
 *  WWW.KPACITOR.COM
 *  
 *  OUTUBRO DE 2017
 *  
 *  elderlucas@kpacitor.com
 *  
 *  
 ************************************************************************/

#include <Wire.h>
#include <Kalman.h>

uint8_t i2c_data[14];
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

uint32_t timer;

Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;

double KalAngleX;
double KalAngleY;
double KalAngleZ;

double gyroXangle;
double gyroYangle;



/************************************************************************
 * Setup Inicial com as Configurações para i2C.
 ************************************************************************/

void setup() {

  /* Inicializando a Serial para exibir mensagens de Debug */
  Serial.begin(115200);

  /* Inicializando o Barramento i2c para comunicação com a MPU6050 */
  Wire.begin();

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Freq = 400kHz.
#else
  TWBR = ((F_CPU/400000UL) - 16) / 2; // Freq = 400kHz
#endif

  i2c_data[0] = 7;      /* 0x19 - Taxa de amostragem  8kHz/(7 + 1) = 1000Hz */
  i2c_data[1] = 0x00;   /* 0x1A - Desabilitar FSYNC, Configurar o Filtro de ACC 260Hz, Configurar Filtro de Gyro 256Hz, Amostragem de 8Khz */
  i2c_data[2] = 0x00;   /* 0x1B - Configurar o fundo de escala do Gyro ±250deg/s - Faixa */
  i2c_data[3] = 0x00;   /* 0x1C - Configurar o fundo de escala do Acelerômetro para ±2g - Faixa */

  /* Configirações do i2c*/
  while(i2cWrite(0x19, i2c_data, 4, false));

  /* PLL tenha como referência o gyro de eixo X, Desabilitando Sleep Mode */
  while(i2cWrite(0x6B, 0x01, true));

  /* */
  while(i2cRead(0x75, i2c_data, 1));

  if(i2c_data[0] != 0x68){
    Serial.print("Erro. Placa desconhecida\n");
    while(1){
      Serial.print("Erro. Conecte a MPU6050 no barramento i2c\n");
    }
  }

  /* Tempo de estabilização do Sensor MPU6050 */
  delay(100);

  /* 1 - Leitura dos dados de Acc XYZ */
  while(i2cRead(0x3B, i2c_data, 14));

  /* 2 - Organizar os dados de Acc XYZ */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])

  /* 3 - Calculo de Pitch e Roll */  
  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* 4 - Inicialização do Filtro de Kalman XY */
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;
 
  timer = micros();

  //Serial.print("Fim Setup\n");

  init_motores();

  PMWControleMotores(0);
}

void loop() {

  /* Leitura dos Dados de Aceleração e Gyro do sensor MPU6050 */
  while(i2cRead(0x3B, i2c_data, 14));

  /*Aceleração*/
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])

  /*Giroscópio*/
  gyroX = (int16_t)((i2c_data[8] << 8) | i2c_data[9]); // ([ MSB ] [ LSB ])
  gyroY = (int16_t)((i2c_data[10] << 8) | i2c_data[11]); // ([ MSB ] [ LSB ])
  gyroZ = (int16_t)((i2c_data[12] << 8) | i2c_data[13]); // ([ MSB ] [ LSB ])

  /*
  Serial.print("AccXYZ"); Serial.print("\t");
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\n");
  Serial.print("GiroXYZ"); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\n");
  */  

  /******************* Filtro de Kalman *************************/
  
  /* Calculo do Delta Time */
  double dt = (double)(micros() - timer)/1000000;
  timer = micros();

  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* Convertendo de Rad/Segundo para Graus/Segundo Calculo da Taxa angular baseado no Giroscópio */
  gyroXangle = gyroX / 131.0; //deg/s
  gyroYangle = gyroY / 131.0; //deg/s

  /* Estimativa de Ângulo nos Eixos X e Y usando Filtro de Kalman */
  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt);

  /* Mensagens de Debug para verificação dos resultados obtidos com Filtro de Kalman e Calculos dos Angulos com os Acelerômetros */
  Serial.print(KalAngleY); Serial.print("\n"); //Angulo estimado com o filtro de Kalman
  Serial.print(pitch); Serial.print("\t"); //Angulo Calculado com os dados de aceleração da MPU6050
  


}
