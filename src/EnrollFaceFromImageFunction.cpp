#include "EnrollFaceFromImageFunction.hpp"

#include <stdio.h>
#include <string>
#include <ros/ros.h>

#include <NCore.h>
#include <NBiometricClient.h>
#include <NBiometrics.h>
#include <NMedia.h>
#include <NLicensing.h>

#include "TutorialUtils.h"

const NChar * components = { N_T("Biometrics.FaceExtraction") };
const NChar * additionalComponents = N_T("Biometrics.FaceSegmentsDetection");

NResult ObtainComponents(NChar *ipAddress, NChar *port)
{
  NBool available = NFalse;
  NResult result = N_OK;
  NBool additionalObtained = NFalse;

  // check available licenses
  result = NLicenseObtainComponents(ipAddress, port, components, &available);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("(1) NLicenseObtainComponents() failed, result = %d\n"), result);
    return result;
  }
  if (!available)
  {
    printf(N_T("Licenses for %s not available\n"), components);
    result = N_E_NOT_ACTIVATED;
    return result;
  }

  // obtain licenses
  result = NLicenseObtainComponents(ipAddress, port, additionalComponents, &additionalObtained);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("(2) NLicenseObtainComponents() failed, result = %d\n"), result);
    return result;
  }
}

NResult ReleaseComponents()
{
  NResult result = N_OK;
  result = NLicenseReleaseComponents(additionalComponents);
  if (NFailed(result)) PrintErrorMsg(N_T("NLicenseReleaseComponents() failed, result = %d\n"), result);
  result = NLicenseReleaseComponents(components);
  if (NFailed(result)) PrintErrorMsg(N_T("NLicenseReleaseComponents() failed, result = %d\n"), result);
  return result;
}

NResult EnrollFaceFromImageFunction(std::string templateFileName,
   void (*getImage)(HNImage*), NRect *pBoundingRect)
{
  HNSubject hSubject = NULL;
  HNFace hFace = NULL;
  HNBiometricClient hBiometricClient = NULL;
  HNBuffer hBuffer = NULL;
  HNLAttributes hLAttributes = NULL;
  HNFileEnumerator hEnumerator = NULL;
  HNBiometricTask hBiometricTask = NULL;
  HNImage hImage = NULL;
  HNString hBiometricStatus = NULL;

  NResult result = N_OK;
  NInt facesDetected = 0;
  NBiometricStatus biometricStatus = nbsNone;

  NBool wasImageNull = NFalse;
  const NChar * szBiometricStatus = NULL;

  // create subject
  result = NSubjectCreate(&hSubject);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NSubjectCreate() failed (result = %d)!"), result);
    goto FINALLY;
  }

  // create face for the subject
  result = NFaceCreate(&hFace);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NFaceCreate() failed (result = %d)!"), result);
    goto FINALLY;
  }

  // set that face will be captured from image stream
  result = NBiometricSetHasMoreSamples(hFace, NTrue);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NBiometricSetHasMoreSamples() failed (result = %d)!"), result);
    goto FINALLY;
  }

  // set the face for the subject
  result = NSubjectAddFace(hSubject, hFace, NULL);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NSubjectAddFace() failed (result = %d)!"), result);
    goto FINALLY;
  }

  // create biometric client
  result = NBiometricClientCreate(&hBiometricClient);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NBiometricClientCreate() failed (result = %d)!"), result);
    goto FINALLY;
  }

  {
    NTemplateSize templateSize = ntsLarge;
    NBoolean parameter = NTrue;
    NBool hasEx = NFalse;

    // set template size to large
    result = NObjectSetPropertyP(hBiometricClient, N_T("Faces.TemplateSize"), N_TYPE_OF(NTemplateSize), naNone, &templateSize, sizeof(templateSize), 1, NTrue);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NObjectSetProperty() failed (result = %d)!"), result);
      goto FINALLY;
    }

    result = NLicenseIsComponentActivated(additionalComponents, &hasEx);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NLicenseIsComponentActivated() failed (result = %d)!"), result);
      goto FINALLY;
    }

    if (hasEx)
    {
      // set detect all facial features
      result = NObjectSetPropertyP(hBiometricClient, N_T("Faces.DetectAllFeaturePoints"), N_TYPE_OF(NBoolean), naNone, &parameter, sizeof(parameter), 1, NTrue);
      if (NFailed(result))
      {
        result = PrintErrorMsgWithLastError(N_T("NObjectSetProperty() failed (result = %d)!"), result);
        goto FINALLY;
      }
    }
  }

  // create template creation task
  result = NBiometricEngineCreateTask(hBiometricClient, nboCreateTemplate, hSubject, NULL, &hBiometricTask);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NBiometricEngineCreateTask() failed (result = %d)!"), result);
    goto FINALLY;
  }

  while (biometricStatus == nbsNone)
  {

    getImage(&hImage);

    if (!hImage)
    {
      wasImageNull = NTrue;
      break;
    }

    result = NFaceSetImage(hFace, hImage);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NFaceSetImage() failed (result = %d)!"), result);
      goto FINALLY;
    }

    result = NBiometricEnginePerformTask(hBiometricClient, hBiometricTask);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NBiometricEnginePerformTask() failed (result = %d)!"), result);
      goto FINALLY;
    }

    // retrieve the error message
    {
      HNError hError = NULL;
      result = NBiometricTaskGetError(hBiometricTask, &hError);
      if (NFailed(result))
      {
        result = PrintErrorMsgWithLastError(N_T("NBiometricTaskGetError() failed (result = %d)!"), result);
        goto FINALLY;
      }
      if (hError)
      {
        result = PrintErrorMsgWithError(N_T("task error:\n"), hError);
        {
          NResult result2 = NObjectSet(NULL, (HNObject*) &hError);
         if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
        }
      }
    }

    result = NBiometricTaskGetStatus(hBiometricTask, &biometricStatus);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NBiometricTaskGetStatus() failed (result = %d)!"), result);
      goto FINALLY;
    }

    result = NObjectSet(NULL, (HNObject*) &hImage);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NObjectSet() failed (result = %d)!"), result);
      goto FINALLY;
    }
  }

  // reset HasMoreSamples value since we finished extracting images
  result = NBiometricSetHasMoreSamples(hFace, NFalse);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NBiometricSetHasMoreSamples() failed (result = %d)!"), result);
    goto FINALLY;
  }

  // if loading was finished because MeadiaReaded had no more images we have to
  // finalize extraction by performing task after setting HasMoreSamples to false
  if (wasImageNull)
  {
    result = NBiometricEnginePerformTask(hBiometricClient, hBiometricTask);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NBiometricEnginePerformTask() failed (result = %d)!"), result);
      goto FINALLY;
    }

    result = NBiometricTaskGetStatus(hBiometricTask, &biometricStatus);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NBiometricTaskGetStatus() failed (result = %d)!"), result);
      goto FINALLY;
    }
  }

  if (biometricStatus == nbsOk)
  {
    // retrieve the template from subject
    result = NSubjectGetTemplateBuffer(hSubject, &hBuffer);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NSubjectGetTemplateBuffer() failed (result = %d)!"), result);
      goto FINALLY;
    }

    result = NFileWriteAllBytesCN(templateFileName.c_str(), hBuffer);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("failed to write template to file (result = %d)!"), result);
      goto FINALLY;
    }

    // retrieve the image from the face captured
    result = NFaceGetImage(hFace, &hImage);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NFaceGetImage() failed (result = %d)!"), result);
      goto FINALLY;
    }

    // save image to file
    result = NImageSaveToFileEx(hImage, (templateFileName + ".jpg").c_str(), NULL, NULL, 0);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NImageSaveToFileEx() failed (result = %d)!"), result);
      goto FINALLY;
    }

    ROS_INFO("template saved successfully");
  }
  else
  {
    // retrieve biometric status
    result = NEnumToStringP(N_TYPE_OF(NBiometricStatus), biometricStatus, NULL, &hBiometricStatus);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NEnumToStringP() failed (result = %d)!"), result);
      goto FINALLY;
    }

    result = NStringGetBuffer(hBiometricStatus, NULL, &szBiometricStatus);
    if (NFailed(result))
    {
     result = PrintErrorMsgWithLastError(N_T("NStringGetBuffer() failed (result = %d)!"), result);
      goto FINALLY;
    }

   ROS_INFO("template extraction failed!\nbiometric status = %s", szBiometricStatus);

    // retrieve the error message
    {
      HNError hError = NULL;
      result = NBiometricTaskGetError(hBiometricTask, &hError);
      if (NFailed(result))
      {
       result = PrintErrorMsgWithLastError(N_T("NBiometricTaskGetError() failed (result = %d)!"), result);
        goto FINALLY;
      }
      result = N_E_FAILED;
      if (hError)
      {
        result = PrintErrorMsgWithError(N_T("task error:\n"), hError);
        {
          NResult result2 = NObjectSet(NULL, (HNObject*) &hError);
          if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
        }
      }
    }

    goto FINALLY;
  }

  // retrieve the number of faces detected
  result = NSubjectGetFaceCount(hSubject, &facesDetected);
  if (NFailed(result))
  {
    result = PrintErrorMsgWithLastError(N_T("NSubjectGetFaceCount() failed (result = %d)!"), result);
    goto FINALLY;
  }

  if (facesDetected > 0)
  {
    // retrieve attributes array from hFace
    result = NFaceGetObject(hFace, 0, &hLAttributes);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NFaceGetObject() failed (result = %d)!"), result);
      goto FINALLY;
    }

    // retrieve face boundingRect information of the face from attributes array
    result = NLAttributesGetBoundingRect(hLAttributes, pBoundingRect);
    if (NFailed(result))
    {
      result = PrintErrorMsgWithLastError(N_T("NLAttributesGetBoundingRect() failed (result = %d)!"), result);
      goto FINALLY;
    }

    ROS_INFO("found face: location = (%d, %d), width = %d, height = %d",
      (*pBoundingRect).X, (*pBoundingRect).Y,
      (*pBoundingRect).Width, (*pBoundingRect).Height);
  }

  result = N_OK;
FINALLY:
  {
    NResult result2 = NObjectSet(NULL, (HNObject*) &hSubject);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hFace);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hBiometricClient);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hBuffer);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hLAttributes);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hEnumerator);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hBiometricTask);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NObjectSet(NULL, (HNObject*) &hImage);
    if (NFailed(result2)) PrintErrorMsg(N_T("NObjectSet() failed (result = %d)!"), result2);
    result2 = NStringSet(NULL, &hBiometricStatus);
    if (NFailed(result2)) PrintErrorMsg(N_T("NStringSet() failed (result = %d)!"), result2);
  }

  return result;
}
