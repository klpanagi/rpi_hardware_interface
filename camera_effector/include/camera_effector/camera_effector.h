

#ifndef CAMERA_EFFECTOR_CAMERA_EFFECTOR_H
#define CAMERA_EFFECTOR_CAMERA_EFFECTOR_H

#include <wiringPi.h>
#include <softPwm.h>

namespace camera_effector
{

  class CameraEffector
  {
    private:

    public:
      CameraEffector();
      virtual ~CameraEffector();
      bool setup_pwm_pins();
  };

}

#endif // CAMERA_EFFECTOR_CAMERA_EFFECTOR_H
