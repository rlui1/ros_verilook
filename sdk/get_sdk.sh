# exit the script if any statement fails
set -e

DIR_SDK=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SHORT_SDK_NAME=sdk
FULL_SDK_NAME=Neurotec_Biometric_5_1_SDK_Trial

if [ ! -d $DIR_SDK/$SHORT_SDK_NAME ];
then
  # Extract sdk.zip if it's there,
  # otherwise download it first.
  if [ ! -f $DIR_SDK/$SHORT_SDK_NAME.zip ];
  then
    wget -O $DIR_SDK/$SHORT_SDK_NAME.zip \
         http://download.neurotechnology.com/$FULL_SDK_NAME\_2015-03-02.zip
  fi
  (
    cd $DIR_SDK
    unzip $SHORT_SDK_NAME.zip \
      "$FULL_SDK_NAME/Bin/Data/*" \
      "$FULL_SDK_NAME/Bin/Linux_x86_64/Activation/*" \
      "$FULL_SDK_NAME/Lib/Linux_x86_64/*" \
      "$FULL_SDK_NAME/Tutorials/Biometrics/C/Identify/*" \
      "$FULL_SDK_NAME/Tutorials/Common/Build/*" \
      "$FULL_SDK_NAME/Tutorials/Common/C/*" \
      "$FULL_SDK_NAME/Include/*" \
      -d $DIR_SDK
      mv $FULL_SDK_NAME $SHORT_SDK_NAME
  )

  ( # Build binaries that we'll use
    cd $DIR_SDK/$SHORT_SDK_NAME/Tutorials/Biometrics/C/Identify
    patch Identify.c $DIR_SDK/patches/Identify.c.patch
    make
  )

  ( # Set the license server to be accessible from a remote machine
    cd $DIR_SDK/$SHORT_SDK_NAME/Bin/Linux_x86_64/Activation
    cp pgd.Sample.conf pgd.conf
    patch pgd.conf $DIR_SDK/patches/pgd.conf.patch
  )

  echo sdk is ready
else
  echo $SHORT_SDK_NAME folder already exists
fi
