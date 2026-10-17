#include <SPI.h>

// --- CLASSE KALMAN FILTER ---
class SimpleKalmanFilter {
  private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate = 0;
    float _last_estimate = 0;
    float _kalman_gain = 0;

  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) {
      _err_measure = mea_e;
      _err_estimate = est_e;
      _q = q;
    }

    float updateEstimate(float mea) {
      _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
      _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
      _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + abs(_last_estimate - _current_estimate) * _q;
      _last_estimate = _current_estimate;
      return _current_estimate;
    }
};

// --- CONFIGURAÇÕES ---
const int PIN_SENSOR = A0;
const int PIN_CS_SD  = 4;     // Seu pino CS
const char* NOME_ARQUIVO = "DADOS.CSV"; // Nome constante para facilitar

// Instancia o filtro: (IncertezaMedicao, IncertezaEstimativa, RuidoMovimento)
SimpleKalmanFilter filtroDist(4.0, 2.0, 0.3);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // 1. Leitura Bruta
  int leitura = analogRead(PIN_SENSOR);
  // Sua equação de calibração
  float cm_bruto = 10650.08 * pow(leitura, -0.935) - 10;
  
  // Clamp (Limites Físicos)
  if (cm_bruto < 5) cm_bruto = 5;
  if (cm_bruto > 80) cm_bruto = 80;

  // 2. Leitura Filtrada
  float cm_kalman = filtroDist.updateEstimate(cm_bruto);

  // 3. Exibir no Serial Plotter (Para ver em tempo real)
  // Formato: "Bruto:XX,Kalman:YY"
  Serial.print("Bruto:");
  Serial.print(cm_bruto);
  Serial.print(","); 
  Serial.print("Kalman:");
  Serial.println(cm_kalman);

  delay(50); // ~20 leituras por segundo
}