//Victor Resende Lins
//PEB/COPPE - LEP
//03/02/2023
//Válvula proporcional Vinsp para controle VCV e PCV
//Vexp on-off
//Controle através de um motor nema17 c/ driver tb6600 via controle de PWM, utilizando Timer 1 do Arduino UNO

#include <BufferedOutput.h>  //imprime leitura do sensor no serial evitando block
#include <loopTimer.h>
#include <millisDelay.h>
#include <Wire.h>
#define ab_max 570
#define ab_max_Vexp 1000

//-----------------PID-------------------------------------------//
class PID {
  public:

    double erro, erro1;
    double sample;
    double kP, kI, kD;
    double P, I, D;
    double PID_;
    double setPoint;
    double fluxo_PID;
    long lastProcess;
    int pos;
    int modo;

    PID(double _kP, double _kI, double _kD) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
    }
    void addnewsample(double _sample) {
      sample = _sample;
    }
    void setSetPoint(double _setPoint) {
      setPoint = _setPoint;
    }
    void posicao(int _pos) {
      pos = _pos;
    }
    void Modo(int _modo) {
      modo = _modo;
    }
    void saidaPID(double _fluxo_PID) {
      fluxo_PID = _fluxo_PID;
    }
    double processo() {
      erro = sample - setPoint ;
      long currentProcess = micros();
      float deltaTime = (currentProcess - lastProcess) ;/// 1000.0;
      lastProcess = currentProcess;

      if (modo == '1') {        //pcv
        P = (erro) * kP;

        if (pos <= 0 || pos >= ab_max) {
          I = I;
        }
        else {
          I = I + kI * erro * deltaTime;
        }

        D = (erro - erro1) * kD / deltaTime;
        erro1 = erro;

        if (pos <= 0 || pos >= ab_max) {
          PID_ = PID_;
        }
        else {
          PID_ = P + I + D;
        }
      }
      else if (modo == '2') {       //vcv
        P = (erro) * kP;

        if ((pos <= 0) || (pos >= ab_max)) {
          I = I;
        }
        else {
          I = I + kI * erro * deltaTime;
        }

        D = (erro - erro1) * kD / deltaTime;
        erro1 = erro;

        PID_ = P + I + D;
        if (pos <= 0 || pos >= ab_max) {
          PID_ = 0.00;
        }
      }

      return PID_;
    }
    double Proporcional() {
      return P;
    }
    double Integrador() {
      return I;
    }
    double Derivador() {
      return D;
    }
};

//---------------------------------------------------------------//

//-----------------ROTINA---------------------------------------//

createBufferedOutput(bufferedOut, 64, DROP_UNTIL_EMPTY);  //aumentando o buffer

const int PIN_p = A2;     //sensor de pressão
String inputString = "";
double linhaBase_p, value_p, max_val_p, volt_p, ruido_lb, fluxo, pressao, fluxo_pid, pressao_pid, pos_corr, sp, sp_ = 0.00, ff;
unsigned long t, t2, t_ciclo;
int modo, cont_Vinsp = 0, cont_Vexp = 0;
bool flag;
//--------------Transdutor-----------------------// SENSIRION
/****** endereços para comunicação ******/
#define ADDR 0x40 // Endereço do sensor no barramento I2C
//Sensores
#define FLOW_REG_HIGH 0x10 // endereço de registro da leitura de fluxo
#define FLOW_REG_LOW 0x00
//Informações do sensor
#define SERIAL_REG_HIGH 0x31 // endereço de registro do número de série do sensor
#define SERIAL_REG_LOW 0xAE
/****** parâmetros do sensor para conversão de bits para unidades reais ******/
//Fluxo
#define OFFSET_FLOW 32768
#define SCALE_FLOW 120.0
float flow = 0.0; // convertido para ccm
union {
  uint8_t bytes[2];
  uint16_t integer = 0;
} flow_raw;
union {
  uint8_t bytes[4];
  uint32_t integer = 0L;
} serial;
//---------------Vinsp------------------------------//
const int dir = 12;             //direção
const int pul = 11;              //pwm
const int home_switch = 10;      //sensor de fim de curso
int intervalo = 25;       //intervalo entre as mudanças de estado do pulso em microseg
boolean pulso = LOW;  //estado do pulso
int ref = 0, x = 0;   //referência de posição
volatile int pos;
//---------------Vexp-----------------------------//
const int dir_Vexp = 8;
const int pul_Vexp = 9;
int home_switch_Vexp = 7;
int intervalo_Vexp = 25;
boolean pulso_Vexp = LOW;
int refVexp = 0, y = 0;
volatile int posVexp;

//-------- Parâmetros do PID--------------//
PID fluxoPID(5.00, 0.0005, 0.00, 0.0);//(5.00, 0.0005, 0.00, 0.0);  //GANHOS PID ->kp,ki.kd.kt
PID pressaoPID(.00, .0001, .00, 0.0);  //GANHOS PID ->kp,ki.kd.kt
//---------------------------------------//


//----------Parâmetros Ventilatórios--------//
unsigned long tct = 3  , tins = 1.5;       //us

//-----------------------------------------//
void setup() {
  Serial.begin(2000000);
  bufferedOut.connect(Serial);
  delay(1000);

  //SENSORES//
  Wire.begin();

  // posição zero:
  Serial.println("iniciando...");


  //transdutor de pressão//
  pinMode(PIN_p, INPUT);
  pinMode(home_switch, INPUT_PULLUP);
  pinMode(home_switch_Vexp, INPUT_PULLUP);


  //drivers dos motores//
  pinMode(dir, OUTPUT);
  pinMode(pul, OUTPUT);
  pinMode(dir_Vexp, OUTPUT);
  pinMode(pul_Vexp, OUTPUT);


  //mapeando posição dos cames
  //sensor de fim de curso -> zero Vinsp
  while (digitalRead(home_switch) == HIGH) {
    digitalWrite(dir, LOW);
    delayMicroseconds(intervalo);
    Passo();
  }

  delay(2000);

  //sensor de fim de curso -> zero Vexp
  while (digitalRead(home_switch_Vexp) == HIGH) {
    digitalWrite(dir_Vexp, HIGH);
    delayMicroseconds(intervalo);
    PassoVexp();
  }

  delay(2000);

  //Fechando completamente -> pos_max//
  pos = 0;
  posVexp = 0;
  int initial_pos = ab_max;
  t2 = micros();

  intervalo = 2000;

  while (pos < initial_pos) {
    digitalWrite(dir, HIGH);
    Passo();
    pos++;
  }
  ref = ab_max;
  digitalWrite(dir, LOW);
  digitalWrite(pul, LOW);

  //CALIBRAÇÃO DOS SENSORES//
  delay(5000);
  Serial.println("calibrando fluxo...");
  setup_sensorFluxo();

  //Corrigindo linha de base do sensor de pressão//
  Serial.println("calibrando pressão...");
  int  i = 0;
  for (i; i < 1000; i++) {
    value_p = analogRead(PIN_p);  //transdutor pressao
    // if (abs(value_p) > abs(max_val_p)) {
    //   max_val_p = value_p;
    // }
    linhaBase_p = linhaBase_p + value_p;
    delay(5);
  }
  linhaBase_p = linhaBase_p / i;

  t = 0.0;

  //--------Configurando registradores para controle PWM usando TIMER 1 do Arduino------//
  noInterrupts();
  TCCR1B = 0;
  TCCR1A = 0;   //prescalar do timer = 64
  TCNT1 = 0;

  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); //fast pwm e prescalar do timer = 64
  OCR1A = 25 - 1;  //interrupção
  TCNT1 = 0; // COUNTER INICIA EM 0
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);

  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(10, INPUT);
  pinMode(7, INPUT);


  Serial.println("1-PCV / 2- VCV");
  while (Serial.available() == 0) {}
  modo = Serial.read();
  delay(100);
}

//----------Funções-----------//
void PassoVexp() {
  pulso_Vexp = !pulso_Vexp;
  digitalWrite(pul_Vexp, pulso_Vexp);
  delayMicroseconds(intervalo_Vexp);
  pulso_Vexp = !pulso_Vexp;
  digitalWrite(pul_Vexp, pulso_Vexp);
  delayMicroseconds(intervalo_Vexp);
}
void Passo() {
  pulso = !pulso;
  digitalWrite(pul, pulso);
  delayMicroseconds(intervalo);
  pulso = !pulso;
  digitalWrite(pul, pulso);
  delayMicroseconds(intervalo);
}
//-----------------Inicialização do sensor de fluxo----------------//
void setup_sensorFluxo() {
  uint8_t crc;

  delay(5000);

  double last_sample = micros();// Para marcar o tempo gasto na inicialização

  //    shift_counter = 0;
  Wire.beginTransmission(ADDR);
  Wire.write(SERIAL_REG_HIGH); // número de série
  Wire.write(SERIAL_REG_LOW);
  Wire.endTransmission();

  Wire.requestFrom(ADDR, 6, true); //O número de série é como um inteiro de 32 bits em pares de bytes com CRC próprios
  Serial.print("No Serie: ");
  if (Wire.available() >= 6) {
    serial.bytes[3] = Wire.read();
    serial.bytes[2] = Wire.read();
    crc = Wire.read();
    serial.bytes[1] = Wire.read();
    serial.bytes[0] = Wire.read();
    crc = Wire.read();

    Serial.println(serial.integer);
  } else {
    Serial.println("Erro! Falha ao receber o número de série");
  }


  Serial.print("Tepo de inicialização: ");
  Serial.println(micros() - last_sample);

  last_sample = millis();

  Wire.beginTransmission(ADDR);
  Wire.write(FLOW_REG_HIGH);
  Wire.write(FLOW_REG_LOW);
  Wire.endTransmission();

  //A primeira leitura é inválida e demora 0.5 ms
  delay(1);
  Wire.requestFrom(ADDR, 3, true); // leitura nos dois primeiros bytes e crc no ao final
  while (Wire.available()) {
    Wire.read();
  }
}
//-----------------Leitura do sensor de fluxo----------------//
float le_fluxo() {
  uint8_t crc;
  //Leitura da medição de fluxo
  Wire.requestFrom(ADDR, 3, true);
  if (Wire.available() >= 3) {
    flow_raw.bytes[1] = Wire.read();
    flow_raw.bytes[0] = Wire.read();
    crc = Wire.read();
  }
  flow = (flow_raw.integer - OFFSET_FLOW) / SCALE_FLOW;
  return flow;
}
//-----------------Inputs via serial------------------------//
float processaInput() {  //leitura dda entrada do serial em array e depois transformando em float, para evitar bloqueio (reduzir delay)
  float numero = 0;
  while (Serial.available()) {
    char val = (char)Serial.read();
    if (val == '\n') {
      numero = atof(inputString.c_str());
      inputString = "";
    } else {
      inputString += val;
    }
  }
  return numero;
}
//-----------------PWM usando Timer1--------------------- //
volatile int stepCountA = ref;
volatile int stepCountB = refVexp;
ISR(TIMER1_COMPA_vect) {
  stepCountA = x - pos;
  stepCountB = y - posVexp;
  //-----Vinsp----//
  if ((stepCountA)) {
    if (PORTB & 0b00001000) {
      if (stepCountA > 0 && pos >= 0) {
        PORTB |=   0b00010000;  //high - abre    (dir)
        pos--;
      }
      else if (stepCountA < 0 && pos <= ab_max) {
        PORTB &=   ~0b00010000; //low - fecha   (dir)
        pos++;
      }
      PORTB ^=   0b00001000; //pino 11->Vinsp  (pwm)
    }
  } else {
    PORTB &= ~0b00001000;
  }
   //-----Vexp----//
  if ((stepCountB)) {
    if (PORTB & 0b00000010) {
      if (stepCountB > 0 & pos <= ab_max_Vexp) {
        PORTB &=   ~0b0000001;      //low - fecha    (dir)
        posVexp++;
      }
      else if (stepCountB > 0 & pos >= 0) {
        PORTB |=   0b00000001;        //high - abre   (dir)
        posVexp--;
      }
      PORTB ^=    0b00000010;  //pino 9->Vexp  (pwm)
    }
  } else {
    PORTB &= ~0b00000010;
  }
}
//-----------------Modos de Ventilação------------------//
  //--------modo volume controlado----------------//
void vcv() {   //modo volume controlado
  ff = -50.83 * sp + 508;  //FEEDFORWARD 14/11 ->SMF3300, mediamovel(janela=250)  ********refazer********
  fluxoPID.setSetPoint(sp);
  fluxoPID.addnewsample(fluxo);
  fluxoPID.Modo(modo);
  //Processo//
  pos_corr = (fluxoPID.processo());  //saida PID
  if (digitalRead(home_switch) == LOW) {
    ref = ref;
    pos = 0;
  }
  if ((sp == 0.00)) {
    ref = ab_max;
  }
  else {
    ref = ff + pos_corr;
  }
}
  //-----------modo pressão controlada-------------------//
void pcv() {                  
  pressaoPID.setSetPoint(sp);
  pressaoPID.addnewsample(pressao);
  pressaoPID.Modo(modo);
  //Processo//
  pos_corr = (pressaoPID.processo()); //saida PID
  if (sp == 0.00) {
    ref = ab_max;
  }
  else {
    ref =  pos_corr;
  }
}
//---------------LOOP-------------//
void loop() {
  //  loopTimer.check(Serial);
  //  bufferedOut.nextByteOut();
  if (modo == '1') {
    pcv();
  }
  else if (modo == '2') {
    vcv();
  }
  //laço para leitura da entrada na porta serial//
  while (Serial.available() > 0) {
    sp_ = processaInput();  //setpoint
  }
  //----------------amostragem  400 Hz----------------//
  if (micros() - t >= 2500) {
    fluxo = - le_fluxo() / 60.00; //leitura do transdutor de fluxo em lps

    value_p = analogRead(PIN_p);    //leitura do transdutor de pressão em cmH2O
    volt_p = (value_p - linhaBase_p) * (5.0 / 1023);
    pressao = volt_p * 3.411146E+1;
  //--------------------DEBUG----------------------//
    //    bufferedOut.print(millis());
    //    bufferedOut.print("\t");
    bufferedOut.print(pos_corr);
    bufferedOut.print("\t");
    bufferedOut.print(pressaoPID.Proporcional());
    bufferedOut.print("\t");
    bufferedOut.print(pressaoPID.Integrador(), 3);
    bufferedOut.print("\t");
    bufferedOut.print(pressaoPID.Derivador(), 3);
    bufferedOut.print("\t");
    bufferedOut.print(sp);
    bufferedOut.print("\t");
    bufferedOut.print(ref);
    bufferedOut.print("\t");
    bufferedOut.print(pos);
    bufferedOut.print("\t");
    ////    bufferedOut.print("\t");
    //     bufferedOut.print(fluxo_rlb,4);
    //    bufferedOut.print("\t");
    bufferedOut.print(fluxo, 4);
    bufferedOut.print("\t");
    bufferedOut.print(pressao, 1);
    //    bufferedOut.print(fluxo_rlb, 4);
    bufferedOut.print("\t");
    //    bufferedOut.println(stepCountA);
    //    bufferedOut.print("\t");
    bufferedOut.print(refVexp);
    //    bufferedOut.print("\t");
    bufferedOut.println(posVexp);
    //    bufferedOut.print("\t");
    //    bufferedOut.println(ref);

    t = micros();
  }
  //-----------Controle ins e exp---------//
  if ( micros() - t_ciclo > tct - tins) {
    if ( micros() - t_ciclo > tct) {
      flag = 0;
    } else  flag = 1;
    t_ciclo = micros();
  }
  //------Inspiração--------//
  if (flag == 0) {
    sp = sp_;
    refVexp = ab_max_Vexp;
    if (sp == 0.00) {
      refVexp = 0;
    }
    y = refVexp;
    if (posVexp == refVexp) {   //move a Vinsp apenas apos a Vexp concluir o movimento
      x = ref;
    }
  }
  //------Expiração--------//
  else {
    sp = 0.00;
    ref = ab_max;
    refVexp = 0;
    x = ref;
    if (pos == ref) {   //move a Vinsp apenas apos a Vexp concluir o movimento
      y = refVexp;
    }
  }

  fluxoPID.posicao(pos);        //atualização da posição no PID
}
