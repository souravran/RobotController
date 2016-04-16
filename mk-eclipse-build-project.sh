#!/bin/bash

SRC_DIR=$1
BUILD_DIR=$2
TOOLCHAIN_FILE=$3
BUILD_TYPE=$4

usage() {
    echo
    echo "$0 SRC_DIR BUILD_DIR TOOLCHAIN_FILE [BUILD_TYPE]"
    echo "     SRC_DIR          := Path to the source directory, the project should be created from."
    echo "     BUILD_DIR        := Path to the build directory, the project should be created in."
    #echo "     TOOLCHAIN_FILE   := The path of a Toolchain-File to use"
    echo "     BUILD_TYPE       := Debug, Release"
    echo
}

#if echo $TOOLCHAIN_FILE | grep -qv "86"
#then
#. /opt/poky/1.2.1/environment-setup-armv5te-poky-linux-gnueabi
#fi

if [ "x${SRC_DIR}" = "x" ]
then
    usage
    exit 1
fi
SRC_DIR=$(readlink -f ${SRC_DIR})

if [ "x${BUILD_DIR}" = "x" ]
then
    usage
    exit 1
fi
BUILD_DIR=$(readlink -f ${BUILD_DIR})

if [ "x${TOOLCHAIN_FILE}" = "x" ]
then
    usage
    exit 1
fi
TOOLCHAIN_FILE=$(readlink -f ${TOOLCHAIN_FILE})

if [ "x${BUILD_TYPE}" = "x" ]
then
    BUILD_TYPE=Release
fi


mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR} && cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=/usr -DECLIPSE_CDT4_GENERATE_SOURCE_PROJECT=FALSE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j4 ${SRC_DIR}

#-DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}

echo
echo "Eclipse build project successfully created. Please import the following directory"
echo "as an existing project into eclipse:"
echo
echo "  - The build project                  : $(readlink -f ${BUILD_DIR})"
echo
