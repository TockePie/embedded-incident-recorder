#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include <stdbool.h>

void Camera_Init(void);
bool Camera_StartCapture(void);
bool Camera_IsCaptureComplete(void);

#endif // CAMERA_H
