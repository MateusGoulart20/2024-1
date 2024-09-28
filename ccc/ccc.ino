#include <Wire.h>

#define MMA8451Q_ADDR 0x1C  // Endereço I2C do MMA8451Q
#define REG_CTRL_REG1 0x2A
#define REG_OUT_X_MSB 0x01

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializar o sensor MMA8451Q
  Wire.beginTransmission(MMA8451Q_ADDR);
  Wire.write(REG_CTRL_REG1);
  Wire.write(0x01);  // Configurar em modo ativo
  Wire.endTransmission();
}

void loop() {
  int16_t x = readAxis(REG_OUT_X_MSB);
  int16_t y = readAxis(REG_OUT_X_MSB + 2);
  int16_t z = readAxis(REG_OUT_X_MSB + 4);

  // Converter para g (1g = 9.81 m/s²)
  float x_g = (float)x / 4096.0;
  float y_g = (float)y / 4096.0;
  float z_g = (float)z / 4096.0;

  // Exibir os valores no Serial Monitor
  Serial.print("X: ");
  Serial.print(x_g);
  Serial.print(" g, Y: ");
  Serial.print(y_g);
  Serial.print(" g, Z: ");
  Serial.println(z_g);

  delay(500);  // Pausa de 500ms
}

// Função para ler valores de um eixo
int16_t readAxis(uint8_t reg) {
  Wire.beginTransmission(MMA8451Q_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MMA8451Q_ADDR, 2);
  int16_t value = Wire.read() << 8 | Wire.read();
  value >>= 2;  // Ajuste para 14 bits

  return value;
}
