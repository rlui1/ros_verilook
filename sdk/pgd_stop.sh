#!/bin/bash

# exit the script if any statement fails
set -e

DIR_SDK=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
$DIR_SDK/sdk/Bin/Linux_x86_64/Activation/run_pgd.sh stop
