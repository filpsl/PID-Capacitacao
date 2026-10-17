// --- INÍCIO DA CLASSE KALMAN FILTER ---
class SimpleKalmanFilter {
  private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate = 0;
    float _last_estimate = 0;
    float _kalman_gain = 0;

  public:
    // construtor: (erro_medicao, erro_estimativa, ruido_processo)
    SimpleKalmanFilter(float mea_e, float est_e, float q) {
      _err_measure = mea_e;
      _err_estimate = est_e;
      _q = q;
    }

    float updateEstimate(float mea) {
      _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
      _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
      _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + _fabs(_last_estimate - _current_estimate) * _q;
      _last_estimate = _current_estimate;

      return _current_estimate;
    }

    void setEstimate(float est) {
      _current_estimate = est;
      _last_estimate = est;
    }
    
    // Auxiliar para valor absoluto float (para economizar lib math se precisar)
    float _fabs(float x) { return (x >= 0) ? x : -x; }
};
// --- FIM DA CLASSE KALMAN FILTER ---