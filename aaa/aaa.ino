#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
while(true){
  delay (2000);
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful"); break;
  } else {
    Serial.println("MPU6050 connection failed");
  }
}
}

void loop() {
  // Variáveis para armazenar os dados
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Lendo os valores do acelerômetro e giroscópio
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Imprimindo os valores no Serial Monitor
  Serial.print("a/g:\t ax: ");
  Serial.print(ax); Serial.print("\t ay: ");
  Serial.print(ay); Serial.print("\t az: ");
  Serial.print(az); Serial.print("\t gx: ");
  Serial.print(gx); Serial.print("\t gy: ");
  Serial.print(gy); Serial.print("\t gz: ");
  Serial.println(gz);

  // Atraso para permitir uma leitura mais fácil dos valores
  delay(500);
}
