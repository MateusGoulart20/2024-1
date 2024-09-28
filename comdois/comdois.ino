#include <Wire.h>
#include <math.h>  // Para funções trigonométricas como atan2
#include <MPU6050.h>

#define MMA8451Q_ADDR1 0x1C  // Endereço I2C do primeiro MMA8451Q
#define MMA8451Q_ADDR2 0x1D  // Endereço I2C do segundo MMA8451Q
#define REG_CTRL_REG1 0x2A
#define REG_OUT_X_MSB 0x01

MPU6050 mpu1(0x68); // Endereço I2C do primeiro MPU6050

double pitch, yaw, roll, radToDeg = 180.0 / M_PI;
int16_t x, y, z, rx, ry, rz;
float x_g, y_g, z_g, x_grau, y_grau, z_grau, mem_x, mem_y, mem_z, gx_offset = 0, gy_offset = 0, gz_offset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializar ambos os sensores MMA8451Q
  initSensor(MMA8451Q_ADDR1);
  initSensor(MMA8451Q_ADDR2);

  // Inicializar o MPU6050
  mpu1.initialize();
  if (mpu1.testConnection()) {
    Serial.println("MPU6050-1 connection successful");
  } else {
    Serial.println("MPU6050-1 connection failed");
  }
}

void loop() {
  delay(100);
  // Leitura do primeiro sensor
  Serial.print("1,");readAndPrintAxis(MMA8451Q_ADDR1);

  // Leitura do segundo sensor
  Serial.print("2,");readAndPrintAxis(MMA8451Q_ADDR2);

  Serial.print("0,"); readMPU6050();

}

// Função para inicializar o sensor
void initSensor(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(REG_CTRL_REG1);
  Wire.write(0x01);  // Configurar em modo ativo
  Wire.endTransmission();
}



// Função para ler e exibir valores dos eixos de um sensor
void readAndPrintAxis(uint8_t address) {
  x = readAxis(address, REG_OUT_X_MSB);
  y = readAxis(address, REG_OUT_X_MSB + 2);
  z = readAxis(address, REG_OUT_X_MSB + 4);

  // Converter para g (1g = 9.81 m/s²)
  x_g = (float)x / 4096.0;
  y_g = (float)y / 4096.0;
  z_g = (float)z / 4096.0;
  //Serial.print(x_g);Serial.print(",");Serial.print(y_g);Serial.print(",");Serial.println(z_g);
  /* Já não sei mais como esse código se comporta e deveria fazer 
  x_grau = (-y_g) * 90;
  y_grau = 0;
  
  if (x > 0) {
    z_grau = 90 * (1 - z_g);
  } else {
    z_grau = 90 * (3 + z_g);
  }
  */
  // tentar funcionar por outros tipos de manipulação
  y_grau = 0;
  x_grau = 0;
  if(z>0){
    z_grau = 90 * (x_g);
    //x_grau = 90 * (-y_g);
  }else{
    z_grau = 180+90 * (-x_g);
    //x_grau = 180+90 * (y_g);
  }
  
  Serial.print(x_grau); Serial.print(",");
  Serial.print(y_grau); Serial.print(",");
  Serial.print(z_grau);
  
  x_grau = sqrt(x_g*x_g+y_g*y_g+z_g*z_g) -1;
  //if(x_grau <0.02) x_grau = 0;

  Serial.print(",");
  //Serial.print(x_grau); Serial.print(",");
  Serial.print(x_g); Serial.print(",");
  Serial.print(y_g); Serial.print(",");
  Serial.println(z_g);
  
  
  //Serial.print(x_grau);Serial.print(",");Serial.print(y_grau);Serial.print(",");Serial.println(z_grau);
  // sendo a transformações de para pitch yaw e roll as corretas
  // Yaw não é possível determinar pois é giro.
  // As coordenadas esféricas (r,θ,φ){\displaystyle (r,\theta ,\varphi )} são (convenção norte-americana):
  // angulo teta sera  x(pitch) , n tera y(yaw), varphi sera z(roll)
  pitch = atan2(y_g, z_g) * radToDeg;
  roll = atan2(x_g, sqrt((y_g * y_g) + (z_g * z_g))) * radToDeg;
  yaw = atan2(y_g, x_g) * radToDeg;

  //Serial.print((float)roll);Serial.print(",");Serial.print((float)yaw);Serial.print(",");Serial.println((float)pitch);
}

// Função para ler valores de um eixo de um sensor específico
int16_t readAxis(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(address, 2);
  int16_t value = Wire.read() << 8 | Wire.read();
  value >>= 2;  // Ajuste para 14 bits

  return value;
}

// Função para ler e exibir valores do MPU6050
void readMPU6050() {
  mpu1.getMotion6(&x, &y, &z, &rx, &ry, &rz);
  x_g = (float)x / 16384.0;
  y_g = (float)y / 16384.0;
  z_g = (float)z / 16384.0;
  x_grau = (float)rx / 131.0;
  y_grau = (float)ry / 131.0;
  z_grau = (float)rz / 131.0;

  if (gx_offset) {
    x_grau -=gx_offset; y_grau -=gy_offset; z_grau -=gz_offset;
  }else{
    gx_offset = x_grau; gy_offset = y_grau; gz_offset = z_grau;
    x_grau = 0; y_grau = 0; z_grau = 0;
  }
  if( (x_grau > -2) && (x_grau < 2)) x_grau=0;
  if( (y_grau > -2) && (y_grau < 2)) y_grau=0;
  if( (z_grau > -2) && (z_grau < 2)) z_grau=0;
  Serial.print(x_g); Serial.print(",");
  Serial.print(y_g); Serial.print(",");
  Serial.print(z_g); Serial.print(",");
  Serial.print(x_grau); Serial.print(",");
  Serial.print(y_grau); Serial.print(",");
  Serial.println(z_grau);
}