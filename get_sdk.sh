# exit the script if any statement fails
set -e

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

SHORT_SDK_NAME=sdk
FULL_SDK_NAME=Neurotec_Biometric_5_1_SDK_Trial

if [ ! -d $DIR/$SHORT_SDK_NAME ];
then
  if [ ! -f $DIR/$SHORT_SDK_NAME.zip ];
  then
    wget -O $DIR/$SHORT_SDK_NAME.zip http://download.neurotechnology.com/$FULL_SDK_NAME\_2015-03-02.zip
  fi
  unzip $DIR/$SHORT_SDK_NAME.zip \
    "$FULL_SDK_NAME/Bin/Data/*" \
    "$FULL_SDK_NAME/Bin/Linux_x86_64/Activation/*" \
    "$FULL_SDK_NAME/Lib/Linux_x86_64/*" \
    "$FULL_SDK_NAME/Tutorials/Biometrics/C/*" \
    "$FULL_SDK_NAME/Tutorials/Common/Build/*" \
    "$FULL_SDK_NAME/Tutorials/Common/C/*" \
    "$FULL_SDK_NAME/Include/*" \
    -d $DIR
  mv $DIR/$FULL_SDK_NAME $DIR/$SHORT_SDK_NAME

  # Build binaries that we'll use
  make -C $DIR/sdk/Tutorials/Biometrics/C/Identify

  # Set the license server to be accessible from a remote machine
  DIR_ACTIVATION=$DIR/$SHORT_SDK_NAME/Bin/Linux_x86_64/Activation
  cat $DIR_ACTIVATION/pgd.Sample.conf | sed 's/mode = single/#mode = single/' \
    | sed 's/#mode = server/mode = server/' > $DIR_ACTIVATION/pgd.conf
fi
