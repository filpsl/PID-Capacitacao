#include "Kalman.h"
#include <time.h>


// --- PINAGEM DO HARDWARE ---
const int PIN_ESQ_PWM = 5;
const int PIN_ESQ_GND = 3;
const int PIN_DIR_PWM = 9;
const int PIN_DIR_GND = 6;
const int PIN_LED     = 7;
const int PIN_SENSOR  = A0;

// --- CONSTANTES DO ROBÔ --- 
const int SETPOINT_DISTANCIA = 45;
const int VELOCIDADE_BASE = 160;                

// --- Inicialização do Filtro de Kalman
SimpleKalmanFilter filtroDist(4.0, 2.0, 0.3);

// Variáveis de Controle
float Kp = 0, Ki = 0, Kd = 0;
float erroAnterior = 0, integralErro = 0;
float dist = 0, erro = 0, pid_out = 0;

// Varáveis de Tempo
clock_t agora, ultimo_comando, dt;

// --- FUNÇÕES AUXILIARES --- 
float lerDistancia() {
  int leitura = analogRead(PIN_SENSOR);
  
  // 1. PROTEÇÃO MATEMÁTICA (Do código novo/sugestão anterior)
  if (leitura <= 0) return 90; 

  // Sua equação calibrada
  float cm = 10650.08 * pow(leitura,-0.935) - 10;
  
  if (cm < 20) cm = 20;
  if (cm > 70) cm = 70;

  float cm_filtrado = filtroDist.updateEstimate(cm);
  return cm_filtrado;
}

void acionarMotores(int pwmEsq) {
  // Roda Direita (Fixa)
  analogWrite(PIN_DIR_PWM, VELOCIDADE_BASE);
  
  // Limites de segurança (0-255)
  if (pwmEsq > 255) pwmEsq = 255;
  if (pwmEsq < 0) pwmEsq = 0;

  analogWrite(PIN_ESQ_PWM, pwmEsq);
}

void setup(){
  // 2. CORREÇÃO: Serial no lugar certo e velocidade padrão
  Serial.begin(115200);

  // 3. A CORREÇÃO CRUCIAL (Copiada do seu código novo)
  // Sem isso, os motores flutuam e travam o Arduino
  pinMode(PIN_ESQ_PWM, OUTPUT);
  pinMode(PIN_ESQ_GND, OUTPUT);
  pinMode(PIN_DIR_PWM, OUTPUT);
  pinMode(PIN_DIR_GND, OUTPUT);
  
  // Cria o "Terra" virtual para os motores
  digitalWrite(PIN_ESQ_GND, LOW);
  digitalWrite(PIN_DIR_GND, LOW);
  
  Serial.println("EVA PID Simples Iniciado");

  // Para não dar ERRO
  ultimo_comando = clock();
}

void loop(){
  // -- CÁLCULO DO ERRO
  dist = lerDistancia();
  erro = SETPOINT_DISTANCIA - dist;

  // --- CÁLCULO PID ---
  float P = Kp * erro;
  
  integralErro += erro;
  // Anti-windup
  if(integralErro > 30) integralErro = 30; 
  if(integralErro < -30) integralErro = -30;
  
  float I = Ki * integralErro;

  // Calculando o Termo Derivativo
  agora = clock();
  dt = (double)(agora - ultimo_comando) / CLOCKS_PER_SEC;
  float D = Kd * (erro - erroAnterior) / dt;
  
  erroAnterior = erro;
  pid_out = P + I + D;
 
  // Roda Esquerda (Controlada)
  int pwmEsq = VELOCIDADE_BASE + (int)pid_out;
  // Debug limpo
  Serial.print("Dist: ");
  Serial.print(dist);
  Serial.print(" | I: ");
  Serial.println(I);

  acionarMotores(pwmEsq);
  
  // Registrando o momento do último comando
  ultimo_comando = clock();
  
  delay(10); // Estabilidade do loop
}
