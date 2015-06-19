# exit the script if any statement fails
set -e

SHORT_SDK_NAME=sdk
FULL_SDK_NAME=Neurotec_Biometric_5_1_SDK_Trial

if [ ! -d $SHORT_SDK_NAME ];
then
  if [ ! -f $SHORT_SDK_NAME.zip ];
  then
    wget -O $SHORT_SDK_NAME.zip http://download.neurotechnology.com/$FULL_SDK_NAME\_2015-03-02.zip
  fi
  unzip $SHORT_SDK_NAME.zip \
    "$FULL_SDK_NAME/Bin/Data/*" \
    "$FULL_SDK_NAME/Bin/Linux_x86_64/Activation/*" \
    "$FULL_SDK_NAME/Lib/Linux_x86_64/*" \
    "$FULL_SDK_NAME/Tutorials/Biometrics/C/*" \
    "$FULL_SDK_NAME/Tutorials/Common/Build/*" \
    "$FULL_SDK_NAME/Tutorials/Common/C/*" \
    "$FULL_SDK_NAME/Include/*"
  mv $FULL_SDK_NAME $SHORT_SDK_NAME
fi
