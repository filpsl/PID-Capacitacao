#include "Kalman.h"

// --- PINAGEM DO HARDWARE ---
const int PIN_ESQ_PWM = 5;
const int PIN_ESQ_GND = 3;
const int PIN_DIR_PWM = 9;
const int PIN_DIR_GND = 6;
const int PIN_SENSOR  = A0;

// --- CONSTANTES DO ROBÔ --- 
const int SETPOINT_DISTANCIA = 45;
const int VELOCIDADE_BASE = 160;
const int PID_MAX = 255 - VELOCIDADE_BASE;
const int I_MAX = PID_MAX / 2;
const unsigned long PERIODO_US = 40000;

// --- Inicialização do Filtro de Kalman
SimpleKalmanFilter filtroDist(4.0, 2.0, 0.3);

// Variáveis de Controle
float Kp = 0, Ki = 0, Kd = 0;
float erroAnterior, integralErro;
float dist, erro, pidOut;

// Saídas do Controle
float P, I, D, pwmEsq;

// Varáveis de Tempo
unsigned long agora, ultimo_comando;
float dt;

// --- FUNÇÕES AUXILIARES --- 
float lerDistanciaBruta() {
  int leitura = analogRead(PIN_SENSOR);

  // Equação de leitura
  float cm = 10650.08 * pow(leitura,-0.935) - 10;
  
  if (cm < 15) cm = 15;
  if (cm > 70) cm = 70;
  
  return cm;
}

float lerDistancia() {
  return filtroDist.updateEstimate(lerDistanciaBruta());
}
void iniciarFiltro(){
  const int N = 10;
  float soma = 0;
  for (int i = 0; i < N; i++){
    soma += lerDistanciaBruta();
    delay(40);
  }
  filtroDist.setEstimate(soma / N);
}

void acionarMotores(int pwmEsq) {
  analogWrite(PIN_DIR_PWM, VELOCIDADE_BASE); // roda direita fixa
  analogWrite(PIN_ESQ_PWM, pwmEsq);          // roda esquerda variável
  // Serial.println(pwmEsq);
}

void setup(){

  Serial.begin(115200);

  pinMode(PIN_ESQ_PWM, OUTPUT);
  pinMode(PIN_ESQ_GND, OUTPUT);
  pinMode(PIN_DIR_PWM, OUTPUT);
  pinMode(PIN_DIR_GND, OUTPUT);
  
  // Cria o "Terra" virtual para os motores
  digitalWrite(PIN_ESQ_GND, LOW);
  digitalWrite(PIN_DIR_GND, LOW);
  
  iniciarFiltro();
  
  dist = lerDistancia();
  erro = SETPOINT_DISTANCIA - dist; 
  erroAnterior = erro;
  ultimo_comando = micros();

  // Serial.println("EVA PID Simples Iniciado");
}

void loop(){
  // portão do controle
  agora = micros();
  if (agora - ultimo_comando < PERIODO_US) return;
  
  // cáculo do erro
  dist = lerDistancia();
  erro = SETPOINT_DISTANCIA - dist;
  // Serial.println(erro);
  // cálculos de tempo
  dt = (agora - ultimo_comando) / 1e6; // passando para segundos
  ultimo_comando = agora;
  
  // ---------------- ------CÁLCULO PID ------------------------- 
  // ------------------- FIM CÁLCULO PID ----------------------
  
  // Roda Esquerda (Controlada)
  pwmEsq = VELOCIDADE_BASE + (int)pidOut;
  acionarMotores(pwmEsq);
}

