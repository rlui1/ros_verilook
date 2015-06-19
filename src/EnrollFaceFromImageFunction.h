#ifndef ENROLL_FACE_FROM_IMAGE_FUNCTION_H
#define ENROLL_FACE_FROM_IMAGE_FUNCTION_H

#include <Core/NTypes.h>
#include <NMedia.h>

NResult ObtainComponents(NChar *ipAddress, NChar *port);
NResult ReleaseComponents();
NResult EnrollFaceFromImageFunction(const char *templateFileName, void (*getImage)(HNImage*));

#endif
