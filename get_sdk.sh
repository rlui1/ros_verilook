# exit the script if any statement fails
set -e

DIR_PKG=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

DIR_SDK=$DIR_PKG/sdk
FULL_SDK_NAME=Neurotec_Biometric_5_1_SDK_Trial

if [ ! -d $DIR_SDK ];
then
  # Extract sdk.zip if it's there, otherwise download it first.
  if [ ! -f $DIR_SDK.zip ];
  then
    wget -O $DIR_SDK.zip http://download.neurotechnology.com/$FULL_SDK_NAME\_2015-03-02.zip
  fi
  unzip $DIR_SDK.zip \
    "$FULL_SDK_NAME/Bin/Data/*" \
    "$FULL_SDK_NAME/Bin/Linux_x86_64/Activation/*" \
    "$FULL_SDK_NAME/Lib/Linux_x86_64/*" \
    "$FULL_SDK_NAME/Tutorials/Biometrics/C/Identify/*" \
    "$FULL_SDK_NAME/Tutorials/Common/Build/*" \
    "$FULL_SDK_NAME/Tutorials/Common/C/*" \
    "$FULL_SDK_NAME/Include/*" \
    -d $DIR_PKG
  mv $DIR_PKG/$FULL_SDK_NAME $DIR_SDK

  ( # Build binaries that we'll use
    cd $DIR_SDK/Tutorials/Biometrics/C/Identify
    patch Identify.c $DIR_PKG/patches/Identify.c.patch
    make
  )

  ( # Set the license server to be accessible from a remote machine
    cd $DIR_SDK/Bin/Linux_x86_64/Activation
    cp pgd.Sample.conf pgd.conf
    patch pgd.conf $DIR_PKG/patches/pgd.conf.patch
  )

  echo sdk is ready
else
  echo sdk folder already exists
fi
