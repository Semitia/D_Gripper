class Angle_sensor{
    private:
        int pin;
        float angle;
        float angle_bias;
    public:
        Angle_sensor() {}
        Angle_sensor(int ADC_pin){
            pin = ADC_pin;
        }
    float updateAngle(double voltage);
    void setBias(float input_bias);
};

float Angle_sensor::updateAngle(double voltage)
{
    angle = voltage*360.0/1024;
    return angle;
}

void Angle_sensor::setBias(float input_bias)
{
    angle_bias = input_bias;
    return;
}
