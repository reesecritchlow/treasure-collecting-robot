#ifndef CUSTOM_SERVO_H
#define CUSTOM_SERVO_H

namespace LeftTiltServo {
    extern double last_position;
    void write(double position);
    void stopSignal();
}

namespace RightTiltServo {
    extern double last_position;
    void write(double position);
    void stopSignal();
}

namespace GrabServo {
    extern double last_position;
    void write(double position);
    void stopSignal();
}

#endif // CUSTOM_SERVO_H