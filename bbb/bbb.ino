#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu1(0x68); // Endereço I2C do primeiro MPU6050
MPU6050 mpu2(0x69); // Endereço I2C do segundo MPU6050

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializando o primeiro MPU6050
  mpu1.initialize();
  if (mpu1.testConnection()) {
    Serial.println("MPU6050-1 connection successful");
  } else {
    Serial.println("MPU6050-1 connection failed");
  }

  // Inicializando o segundo MPU6050
  //mpu2.initialize();
  //if (mpu2.testConnection()) {
  //  Serial.println("MPU6050-2 connection successful");
  //} else {
  //  Serial.println("MPU6050-2 connection failed");
  //}
}

void loop() {
  // Variáveis para armazenar os dados do primeiro MPU6050
  int16_t ax1, ay1, az1;
  int16_t gx1, gy1, gz1;

  // Variáveis para armazenar os dados do segundo MPU6050
  int16_t ax2, ay2, az2;
  int16_t gx2, gy2, gz2;

  // Lendo os valores do primeiro MPU6050
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);

  // Lendo os valores do segundo MPU6050
  //mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

  // Imprimindo os valores no Serial Monitor
  Serial.print("1,");
  Serial.print(ax1); Serial.print(",");
  Serial.print(ay1); Serial.print(",");
  Serial.print(az1); Serial.print(",");
  Serial.print(gx1); Serial.print(",");
  Serial.print(gy1); Serial.print(",");
  Serial.println(gz1);

  //Serial.print("MPU6050-2 a/g:,");
  //Serial.print(ax2); Serial.print(",");
  //Serial.print(ay2); Serial.print(",");
  //Serial.print(az2); Serial.print(",");
  //Serial.print(gx2); Serial.print(",");
  //Serial.print(gy2); Serial.print(",");
  //Serial.println(gz2);

  // Atraso para permitir uma leitura mais fácil dos valores
  delay(100);
}
