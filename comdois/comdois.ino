#include <Wire.h>
#include <math.h>  // Para funções trigonométricas como atan2
#include <MPU6050.h> // Cat Eletronics

#define MMA8451Q_ADDR1 0x1C  // Endereço I2C do primeiro MMA8451Q
#define MMA8451Q_ADDR2 0x1D  // Endereço I2C do segundo MMA8451Q
#define REG_CTRL_REG1 0x2A
#define REG_OUT_X_MSB 0x01
#define GYRO_CONFIG 0x1B

MPU6050 mpu1(0x68); // Endereço I2C do primeiro MPU6050

#define RANGE 3.7
#define MULTIPLY 2
#define FILTRO_RANGE 0.087
#define CONTA_ESTABILIDADE 4

double pitch, yaw, roll, radToDeg = 180.0 / M_PI;
int16_t x, y, z, rx, ry, rz;
float x_g, y_g, z_g;
float 
  memoria[3] = {0,0,0},
  diferenca[3] = {0,0,0},
  aceleracao[3] = {0,0,0},
  integral[3] = {0,0,0},
  incremento[3] = {0,0,0},
  grau[3] = {0,0,0};
int 
  conta_zero[3] = {0,0,0},
  estado[3] = {0,0,0};
//bool ;

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
  Serial.print("1");readAndPrintAxis(MMA8451Q_ADDR1);

  // Leitura do segundo sensor
  Serial.print("2");readAndPrintAxis(MMA8451Q_ADDR2);

  Serial.print("0"); readMPU6050();

}

// Função para inicializar o sensor
void initSensor(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(REG_CTRL_REG1);
  Wire.write(0x01);  // Configurar em modo ativo
  Wire.endTransmission();
}

void limp(){grau[0]=0;grau[1]=0;grau[2]=0;}

// Função para ler e exibir valores dos eixos de um sensor
void readAndPrintAxis(uint8_t address) {
  limp();
  x = readAxis(address, REG_OUT_X_MSB);
  y = readAxis(address, REG_OUT_X_MSB + 2);
  z = readAxis(address, REG_OUT_X_MSB + 4);

  // Converter para g (1g = 9.81 m/s²)
  x_g = (float)x / 4096.0;
  y_g = (float)y / 4096.0;
  z_g = (float)z / 4096.0;
  //Serial.print(x_g);Serial.print(",");Serial.print(y_g);Serial.print(",");Serial.println(z_g);
  /* Já não sei mais como esse código se comporta e deveria fazer 
  grau[0] = (-y_g) * 90;
  grau[1] = 0;
  
  if (x > 0) {
    grau[2] = 90 * (1 - z_g);
  } else {
    grau[2] = 90 * (3 + z_g);
  }
  */
  // tentar funcionar por outros tipos de manipulação
  grau[1] = 0;
  grau[0] = 0;
  if(z>0){
    grau[2] = 90 * (x_g);
    //grau[0] = 90 * (-y_g);
  }else{
    grau[2] = 180+90 * (-x_g);
    //grau[0] = 180+90 * (y_g);
  }
  Serial.printf(",%f",grau[0]);
  Serial.printf(",%f",grau[1]);
  Serial.printf(",%f",grau[2]);
  
  //grau[0] = sqrt(x_g*x_g+y_g*y_g+z_g*z_g) -1;
  //if(grau[0] <0.02) grau[0] = 0;

  Serial.printf(",%f",x_g);
  Serial.printf(",%f",y_g);
  Serial.printf(",%f",z_g);
  
  //Serial.print(grau[0]);Serial.print(",");Serial.print(grau[1]);Serial.print(",");Serial.println(grau[2]);
  // sendo a transformações de para pitch yaw e roll as corretas
  // Yaw não é possível determinar pois é giro.
  // As coordenadas esféricas (r,θ,φ){\displaystyle (r,\theta ,\varphi )} são (convenção norte-americana):
  // angulo teta sera  x(pitch) , n tera y(yaw), varphi sera z(roll)
  pitch = atan2(y_g, z_g) * radToDeg;
  roll = atan2(x_g, sqrt((y_g * y_g) + (z_g * z_g))) * radToDeg;
  yaw = atan2(y_g, x_g) * radToDeg;

  //Serial.print((float)roll);Serial.print(",");Serial.print((float)yaw);Serial.print(",");Serial.println((float)pitch);
  Serial.println();
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

void filtro(int selecao){
  switch (estado[selecao]){
    case 0: // estavel
      conta_zero[selecao] = CONTA_ESTABILIDADE;
      aceleracao[selecao] = diferenca[selecao];
      incremento[selecao] = 0;
      if(diferenca[selecao]>0){ // deteccao de subida
        estado[selecao] = 1;
      }else
      if(diferenca[selecao]<0){ // deteccao de descida
        estado[selecao] = -1;
      }
    break;
    case 1: // subindo
      if(diferenca[selecao]>=0){
        incremento[selecao]=incremento[selecao]+diferenca[selecao];
        aceleracao[selecao]=incremento[selecao];
        if(diferenca[selecao]==0){
          if(conta_zero[selecao]==0){
            estado[selecao]=0;
          }
          conta_zero[selecao]--;
        }
      } else {
        aceleracao[selecao]=0;
      }
    break;
    case -1: // descendo
      if(diferenca[selecao]<=0){
        incremento[selecao]=incremento[selecao]+diferenca[selecao];
        aceleracao[selecao]=incremento[selecao];
        if(diferenca[selecao]==0){
          if(conta_zero[selecao]==0){
            estado[selecao]=0;
          }
          conta_zero[selecao]--;
        }
      } else {
        aceleracao[selecao]=0;
      }
    break;
  }
}

void filtroEstavel(int selecao){
  if( (diferenca[selecao]<FILTRO_RANGE) && (diferenca[selecao]>-FILTRO_RANGE) ) diferenca[selecao] = 0;
}


// Função para ler e exibir valores do MPU6050
void readMPU6050() {
  limp();
  mpu1.getMotion6(&x, &y, &z, &rx, &ry, &rz);
  x_g = (float)x / 16384.0;
  y_g = (float)y / 16384.0;
  z_g = (float)z / 16384.0;
  grau[0] = (float)rx / 131.0;
  grau[1] = (float)ry / 131.0;
  grau[2] = (float)rz / 131.0;

  // correcao de sensibilidade e desvio
  if( (grau[0] > -RANGE) && (grau[0] < RANGE)) {grau[0]=0;}else{grau[0]*=MULTIPLY;}
  if( (grau[1] > -RANGE) && (grau[1] < RANGE)) {grau[1]=0;}else{grau[1]*=MULTIPLY;}
  if( (grau[2] > -RANGE) && (grau[2] < RANGE)) {grau[2]=0;}else{grau[2]*=MULTIPLY;grau[2]+=MULTIPLY*3;}

  aceleracao[0] = x_g;
  aceleracao[1] = y_g;
  aceleracao[2] = z_g;

  
  diferenca[0] = aceleracao[0] - memoria[0];
  diferenca[1] = aceleracao[1] - memoria[1];
  diferenca[2] = aceleracao[2] - memoria[2];
  
  filtroEstavel(0);
  filtroEstavel(1);
  filtroEstavel(2);

  filtro(0); // tratar eixo x  
  filtro(1); // tratar eixo y
  filtro(2); // tratar eixo z

  memoria[0] = x_g;
  memoria[1] = y_g;
  memoria[2] = z_g;

//*
  Serial.printf(",%f",aceleracao[0]);
  Serial.printf(",%f",aceleracao[1]);
  Serial.printf(",%f",aceleracao[2]);

  Serial.printf(",%f",grau[0]);
  Serial.printf(",%f",grau[1]);
  Serial.printf(",%f",grau[2]);


//Serial.printf(",%f",memoria[0]);
//Serial.printf(",%f",memoria[1]);
//Serial.printf(",%f",memoria[2]);

 // Serial.printf(",%f",diferenca[0]);
 // Serial.printf(",%f",diferenca[1]);
 // Serial.printf(",%f",diferenca[2]);

//Serial.printf(",%d",estado[0]);
  //Serial.printf(",%d",estado[1]);
  //Serial.printf(",%d",estado[2]);
/*/
//*/  

  Serial.println();
}