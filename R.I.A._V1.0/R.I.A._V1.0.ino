// Programa para o robot R.I.A.
// PSA 23/24
// V 1.0

// Notas:
// Este programa está limitado a 180 graus por servo, sendo o mesmo capaz de mais.

// ----------------------------------------------------------------------------------------------------------------

// Include Wire Library for I2C
#include <Wire.h>

// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>

// Comando PS5
#include <ps5Controller.h>

// Math stuff
#include <math.h>

// MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();

// MPU6050
Adafruit_MPU6050 mpu;
float acc_error_x = 0;
float acc_error_y = 0;
float acc_error_z = 0;

float g_error_x = 0;
float g_error_y = 0;
float g_error_z = 0;

// Pins dos servos na driver
#define TOP1 0
#define TOP2 1
#define TOP3 2
#define TOP4 3

#define BOT1 4
#define BOT2 5
#define BOT3 6
#define BOT4 7

//Mínimos
#define MinT1 550
#define MinT2 80
#define MinT3 543
#define MinT4 80
#define MinB1 75
#define MinB2 545
#define MinB3 75
#define MinB4 555

//Máximos
#define MaxT1 105
#define MaxT2 540
#define MaxT3 75
#define MaxT4 550
#define MaxB1 520
#define MaxB2 105
#define MaxB3 505
#define MaxB4 105

//Posição base
#define HomeT1 510
#define HomeT2 125
#define HomeT3 510
#define HomeT4 145
#define HomeB1 110
#define HomeB2 530
#define HomeB3 97
#define HomeB4 555
// Ângulos calculados em radianos
float AT1;
float AT2;
float AT3;
float AT4;
float AB1;
float AB2;
float AB3;
float AB4;

float AB1_aux;
float AB2_aux;
float AB3_aux;
float AB4_aux;

// Ângulos convertidos para "pulses"
int AT1_p;
int AT2_p;
int AT3_p;
int AT4_p;
int AB1_p;
int AB2_p;
int AB3_p;
int AB4_p;

// Posições X e Y// Variáveis para cálculo da cinemática invertida
float X_1 = 0;
float X_2 = 0;
float X_3 = 0;
float X_4 = 0;

float Y_1 = 0;
float Y_2 = 0;
float Y_3 = 0;
float Y_4 = 0;

float X_1_target = 0;
float X_2_target = 0;
float X_3_target = 0;
float X_4_target = 0;

float Y_1_target = 0;
float Y_2_target = 0;
float Y_3_target = 0;
float Y_4_target = 0;


// Variáveis para cálculo da cinemática invertida
int TOP_L_1 = 55;  // mm
int BOT_L_1 = 55;  //mm

int TOP_L_2 = 55;  // mm
int BOT_L_2 = 55;  //mm

int TOP_L_3 = 55;  // mm
int BOT_L_3 = 55;  //mm

int TOP_L_4 = 55;  // mm
int BOT_L_4 = 55;  //mm

float r_1;
float r_2;
float r_3;
float r_4;

float sq_r_1;
float sq_r_2;
float sq_r_3;
float sq_r_4;

float K_1;
float K_2;
float K_3;
float K_4;

float H_1;
float H_2;
float H_3;
float H_4;

// Cálculos perliminares que dão jeito
float d_pi = 6.2831;
float pi = 3.142;
float i_2;

// Mudar modo de operação. True = movimento; False = Ajuste de altura.
bool mode = false;
bool cross_once = false;
bool square_once = false;
bool up_once = false;
bool down_once = false;

// Altura do robot;
float altura = 0;
float max_altura = 105;
float min_altura = 0;

int step = 1;

int count = 0;

bool boot = false;

float speed = 1;

int initial_height = 70;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  //  BAUDRATE
  delay(2000);
  Serial.println("Booting...");

  Serial.println("Starting PCA9685");
  servos.begin();
  servos.setPWMFreq(50);
  Serial.println("PCA9685 started");

  home();

  delay(1000);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("Calibrar MPU");
  sensors_event_t a, g, temp;
  Serial.println("antes do get event");
  mpu.getEvent(&a, &g, &temp);

  for (int cal_cycles = 0; cal_cycles <= 200; cal_cycles++) {
    Serial.println("Entrei no ciclo");
    Serial.print("cal_Cycle= ");
    Serial.println(cal_cycles);
    acc_error_x = acc_error_x + a.acceleration.x;
    acc_error_y = acc_error_y + a.acceleration.y;
    acc_error_z = acc_error_z + a.acceleration.z;

    g_error_x = g_error_x + g.gyro.x;
    g_error_y = g_error_y + g.gyro.y;
    g_error_z = g_error_z + g.gyro.z;
  }
  acc_error_z = 9.81 - acc_error_z;

  Serial.println("Calibração concluída!");
  Serial.println("Resultados:");
  Serial.print("acc_error_x: ");
  Serial.println(acc_error_x);
  Serial.print("acc_error_y: ");
  Serial.println(acc_error_y);
  Serial.print("acc_error_z: ");
  Serial.println(acc_error_z);

  Serial.print("g_error_x: ");
  Serial.println(g_error_x);
  Serial.print("g_error_y: ");
  Serial.println(g_error_y);
  Serial.print("g_error_z: ");
  Serial.println(g_error_z);

  

  // Colocar os servos na posição HOME. DESATIVAR NO FUTURO.
  home();

  // Desligar os servos. Medida de segurança.
  // shutdown();


  // Come memória. Desativar se houver problemas.
  Serial.println("Starting PS5 library");
  ps5.begin("e8:47:3a:25:ae:ec");
  Serial.println("Ready.");
}


void loop() {
  // put your main code here, to run repeatedly:
  while (ps5.isConnected() == true) {
    //Dados do acelerómetro
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Switch botão X, ligar e desligar robot.
    if (ps5.Cross() == true && cross_once == false) {
      mode = !mode;
      count = 0;
      cross_once = true;
      Serial.print("mode = ");
      Serial.println(mode);
    }
    if (ps5.Cross() == false && cross_once == true) {
      cross_once = false;
    }

    if (ps5.Square() == true && square_once == false) {
      square_once = true;
    }
    if (ps5.Square() == false && square_once == true) {
      square_once = false;
    }

    switch (mode) {
      case false:
        shutdown();
        boot = true;

        break;
      case (true):
        if (boot == true) {
          Y_1_target = initial_height;
          Y_2_target = initial_height;
          Y_3_target = initial_height;
          Y_4_target = initial_height;
          boot = false;
        }
        UpdateCoords();
        InvKin();
        MoveOutput();

        float vari = 0.2;  //rad
        float acc_x = a.acceleration.x;// + acc_error_x;
        float acc_y = a.acceleration.y;// + acc_error_y;
        Serial.print("acc_x= ");
        Serial.println(acc_x);
        if (acc_x > vari) {
          Y_1_target = Y_1_target + 0.1;
          Y_2_target = Y_2_target + 0.1;
          Y_3_target = Y_3_target - 0.1;
          Y_4_target = Y_4_target - 0.1;

        }
        if (acc_x < -vari) {
          Y_1_target = Y_1_target - 0.1;
          Y_2_target = Y_2_target - 0.1;
          Y_3_target = Y_3_target + 0.1;
          Y_4_target = Y_4_target + 0.1;

        }
        if (acc_y > vari) {
          Y_1_target = Y_1_target + 0.1;
          Y_2_target = Y_2_target - 0.1;
          Y_3_target = Y_3_target - 0.1;
          Y_4_target = Y_4_target + 0.1;
        }
        if (acc_y < -vari) {
          Y_1_target = Y_1_target - 0.1;
          Y_2_target = Y_2_target + 0.1;
          Y_3_target = Y_3_target + 0.1;
          Y_4_target = Y_4_target - 0.1;
        }
        
        break;
    }
  }
}



void UpdateCoords() {
  if (X_1 > X_1_target-(speed/2)) {
    X_1 = X_1 - speed;
  }
  if (X_2 > X_2_target-(speed/2)) {
    X_2 = X_2 - speed;
  }
  if (X_3 > X_3_target-(speed/2)) {
    X_3 = X_3 - speed;
  }
  if (X_4 > X_4_target-(speed/2)) {
    X_4 = X_4 - speed;
  }
  if (X_1 < X_1_target+(speed/2)) {
    X_1 = X_1 + speed;
  }
  if (X_2 < X_2_target+(speed/2)) {
    X_2 = X_2 + speed;
  }
  if (X_3 < X_3_target+(speed/2)) {
    X_3 = X_3 + speed;
  }
  if (X_4 < X_4_target+(speed/2)) {
    X_4 = X_4 + speed;
  }
  if (Y_1 > Y_1_target-(speed/2)) {
    Y_1 = Y_1 - speed;
  }
  if (Y_2 > Y_2_target-(speed/2)) {
    Y_2 = Y_2 - speed;
  }
  if (Y_3 > Y_3_target-(speed/2)) {
    Y_3 = Y_3 - speed;
  }
  if (Y_4 > Y_4_target-(speed/2)) {
    Y_4 = Y_4 - speed;
  }
  if (Y_1 < Y_1_target+(speed/2)) {
    Y_1 = Y_1 + speed;
  }
  if (Y_2 < Y_2_target+(speed/2)) {
    Y_2 = Y_2 + speed;
  }
  if (Y_3 < Y_3_target+(speed/2)) {
    Y_3 = Y_3 + speed;
  }
  if (Y_4 < Y_4_target+(speed/2)) {
    Y_4 = Y_4 + speed;
  }
}

void MoveOutput() {
  // Coloca os servos no seu valor final. Bloqueado a valores ente 0 e 180 graus, expressos em radianos.
  // Questão: O que é mais leve? Atualizar sempre todos os servos ou verificar quais mudaram de posição?

  // Servos topo
  AT1_p = mapf(AT1, 0, pi, HomeT1, MaxT1);
  AT2_p = mapf(AT2, 0, pi, HomeT2, MaxT2);
  AT3_p = mapf(AT3, 0, pi, HomeT3, MaxT3);
  AT4_p = mapf(AT4, 0, pi, HomeT4, MaxT4);

  // Servos base
  AB1_p = mapf(AB1, 0, pi, HomeB1, MaxB1);
  AB2_p = mapf(AB2, 0, pi, HomeB2, MaxB2);
  AB3_p = mapf(AB3, 0, pi, HomeB3, MaxB3);
  AB4_p = mapf(AB4, 0, pi, HomeB4, MaxB4);



  // Enviar pulsos para os servos todos
  servos.setPWM(TOP1, 0, AT1_p);
  servos.setPWM(TOP2, 0, AT2_p);
  servos.setPWM(TOP3, 0, AT3_p);
  servos.setPWM(TOP4, 0, AT4_p);
  servos.setPWM(BOT1, 0, AB1_p);
  servos.setPWM(BOT2, 0, AB2_p);
  servos.setPWM(BOT3, 0, AB3_p);
  servos.setPWM(BOT4, 0, AB4_p);
}

void InvKin() {

  // Pata 1
  r_1 = sq(X_1) + sq(Y_1);

  sq_r_1 = sqrt(r_1);

  AB1_aux = acos((r_1 - sq(TOP_L_1) - sq(BOT_L_1)) / (2 * TOP_L_1 * BOT_L_1));
  AB1 = pi - AB1_aux;

  if (X_1 >= 0) {
    AT1 = atan(Y_1 / X_1) - asin((BOT_L_1 * sin(AB1_aux)) / sq_r_1);
  }
  if (X_1 < 0) {
    K_1 = asin((BOT_L_1 * sin(AB1)) / sq_r_1);
    H_1 = asin(Y_1 / sq_r_1);
    AT1 = pi - H_1 - K_1;
  }

  // Pata 2 ----------------------------------------------------------------------
  r_2 = sq(X_2) + sq(Y_2);

  sq_r_2 = sqrt(r_2);

  AB2_aux = acos((r_2 - sq(TOP_L_2) - sq(BOT_L_2)) / (2 * TOP_L_2 * BOT_L_2));
  AB2 = pi - AB2_aux;

  if (X_2 >= 0) {
    AT2 = atan(Y_2 / X_2) - asin((BOT_L_2 * sin(AB2_aux)) / sq_r_2);
  }
  if (X_2 < 0) {
    K_2 = asin((BOT_L_2 * sin(AB2)) / sq_r_2);
    H_2 = asin(Y_2 / sq_r_2);
    AT2 = pi - H_2 - K_2;
  }

  // Pata 3 ----------------------------------------------------------------------
  if (X_3 != 0) {
    X_3 = -1 * X_3;
  }
  r_3 = sq(X_3) + sq(Y_3);
  sq_r_3 = sqrt(r_3);

  AB3_aux = acos((r_3 - sq(TOP_L_3) - sq(BOT_L_3)) / (2 * TOP_L_3 * BOT_L_3));
  AB3 = pi - AB3_aux;

  if (X_3 >= 0) {  // Não devia ser necessário alterar a lógica dos sinais, rever !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    AT3 = atan(Y_3 / X_3) - asin((BOT_L_3 * sin(AB3_aux)) / sq_r_3);
  }
  if (X_3 < 0) {
    K_3 = asin((BOT_L_3 * sin(AB3)) / sq_r_3);
    H_3 = asin(Y_3 / sq_r_3);
    AT3 = pi - H_3 - K_3;
  }

  // Pata 4 ----------------------------------------------------------------------
  if (X_4 != 0) {
    X_4 = -1 * X_4;
  }
  r_4 = sq(X_4) + sq(Y_4);
  sq_r_4 = sqrt(r_4);

  AB4_aux = acos((r_4 - sq(TOP_L_4) - sq(BOT_L_4)) / (2 * TOP_L_4 * BOT_L_4));
  AB4 = pi - AB4_aux;

  if (X_4 >= 0) {
    AT4 = atan(Y_4 / X_4) - asin((BOT_L_4 * sin(AB4_aux)) / sq_r_4);
  }
  if (X_4 < 0) {
    K_4 = asin((BOT_L_4 * sin(AB4)) / sq_r_4);
    H_4 = asin(Y_4 / sq_r_4);
    AT4 = pi - H_4 - K_4;
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void shutdown() {
  servos.setPWM(TOP1, 0, 0);
  servos.setPWM(TOP2, 0, 0);
  servos.setPWM(TOP3, 0, 0);
  servos.setPWM(TOP4, 0, 0);
  servos.setPWM(BOT1, 0, 0);
  servos.setPWM(BOT2, 0, 0);
  servos.setPWM(BOT3, 0, 0);
  servos.setPWM(BOT4, 0, 0);
}

// Retorna todos os servos à posição base.
void home() {
  servos.setPWM(TOP1, 0, HomeT1);
  servos.setPWM(TOP2, 0, HomeT2);
  servos.setPWM(TOP3, 0, HomeT3);
  servos.setPWM(TOP4, 0, HomeT4);
  servos.setPWM(BOT1, 0, HomeB1);
  servos.setPWM(BOT2, 0, HomeB2);
  servos.setPWM(BOT3, 0, HomeB3);
  servos.setPWM(BOT4, 0, HomeB4);
}
