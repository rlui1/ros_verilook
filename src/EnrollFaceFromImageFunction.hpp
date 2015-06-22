#ifndef ENROLL_FACE_FROM_IMAGE_FUNCTION_HPP
#define ENROLL_FACE_FROM_IMAGE_FUNCTION_HPP

#include <string>
#include <Core/NTypes.h>
#include <NMedia.h>

NResult ObtainComponents(const NChar *ipAddress, const NChar *port);
NResult ReleaseComponents();
NResult EnrollFaceFromImageFunction(std::string templateFileName,
  void (*getImage)(HNImage*), NRect *boundingRect);

#endif
