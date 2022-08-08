#ifndef CUSTOM_SERVO_H
#define CUSTOM_SERVO_H

namespace LeftTiltServo {
    extern double last_position;
    void write(double position);
    void rawWrite(double position);
}

namespace RightTiltServo {
    extern double last_position;
    void write(double position);
    void rawWrite(double position);
}

namespace GrabServo {
    extern double last_position;
    void write(double position);
    void rawWrite(double position);
}

#endif // CUSTOM_SERVO_H