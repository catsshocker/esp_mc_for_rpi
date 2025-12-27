class pidController
{
private:
    double kp;
    double ki;
    double kd;

    double lastErr;
    double errSum;

public:
    pidController(double p, double i, double d);
    ~pidController();
    void setKp(double p);
    void setKi(double i);
    void setKd(double d);
    double compute(double setpoint, double actual, double max, double min, double dt);
    void reset();
    double getKp() const;
    double getKi() const;
    double getKd() const;
};

pidController::pidController(double p, double i, double d)
    : kp(p), ki(i), kd(d), lastErr(0.0), errSum(0.0)
{
}

pidController::~pidController() = default;

void pidController::setKp(double p) { kp = p; }

void pidController::setKi(double i) { ki = i; }

void pidController::setKd(double d) { kd = d; }

double pidController::getKp() const { return kp; }
double pidController::getKi() const { return ki; }
double pidController::getKd() const { return kd; }

void pidController::reset()
{
    lastErr = 0.0;
    errSum = 0.0;
}

double pidController::compute(double setpoint, double actual, double max, double min, double dt)
{

    double err = setpoint - actual;
    errSum += err * dt;
    double dErr = (err - lastErr) / dt;

    double output = (kp * err) + (ki * errSum) + (kd * dErr);

    if (output > max)
        output = max;
    else if (output < min)
        output = min;

    lastErr = err;

    return output;
}
