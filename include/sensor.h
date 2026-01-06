#ifndef SENSOR_H_
#define SENSOR_H_

template <typename T>
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual bool update(T* data) = 0;
};

#endif // SENSOR_H_
