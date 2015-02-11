#!/bin/bash
#
# This script creates the openPOWERLINK APC2100 deliverables:
# - Export needed stack library from oplk/stack/lib
# - Remove hardware directory (Antares Interface)
#
# ./release.sh [VERSION]

RELEASE_VERSION=$1
BASE_DIR=${PWD}
OPLK_DIR=oplk
RELEASE_DIR=oplk_apc2100

# Path to git repository
GIT_REPO=.

# Check if release dir is available, otherwise create it.
if [ ! -d "${RELEASE_DIR}" ]; then
    mkdir ${RELEASE_DIR}
    if [ $? -ne 0 ]; then
        echo "ERROR: ${RELEASE_DIR} could not be created!"
        exit -1
    fi
fi

# Create file that holds the revisions
REVISION_FILE=${RELEASE_DIR}/revision
REVISION=$(git rev-list HEAD -n 1)
REVISION_SHORT=$(git rev-list HEAD -n 1 | cut -c 1-7)

if [ -z "${RELEASE_VERSION}" ]; then
    echo "INFO: No release version set, use short hash (${REVISION_SHORT})."
    RELEASE_VERSION=rev${REVISION_SHORT}
fi

pushd ${OPLK_DIR} > /dev/null
REVISION_OPLK=$(git rev-list HEAD -n 1)
popd > /dev/null

echo "APC2100 REVISION: ${REVISION}" > ${REVISION_FILE}
echo "OPENPLK REVISION: ${REVISION_OPLK}" >> ${REVISION_FILE}

# Export git repository
echo "INFO: Release git repository in folder ${GIT_REPO} to ${RELEASE_DIR}"

pushd ${GIT_REPO} > /dev/null

git archive HEAD | tar -x -C ${BASE_DIR}/${RELEASE_DIR}/${GIT_REPO}
RET=$?

popd ${GIT_REPO} > /dev/null

if [ ${RET} -ne 0 ]; then
    exit ${RET}
else
    echo "INFO: Successfully released $i"
fi

# Go and get the stack library
STACK_LIB_NAME=oplkmnapp-pcieintf
STACK_LIB_DIR=${OPLK_DIR}/stack/lib

for i in $(find ${STACK_LIB_DIR} -type f -name "*${STACK_LIB_NAME}*")
do
    cp --parents $i ${RELEASE_DIR}
done

# copy stack includes
STACK_INCLUDES="\
${OPLK_DIR}/stack/include/oplk \
${OPLK_DIR}/stack/include/common \
"

for i in ${STACK_INCLUDES}
do
    cp -r --parents $i ${RELEASE_DIR}
done

# Now create tar
tar -cf ${RELEASE_DIR}_${RELEASE_VERSION}.tar ${RELEASE_DIR}
if [ $? -ne 0 ]; then
    echo "ERROR: Creating ${RELEASE_DIR}.tar failed!"
    exit -1
fi

# and remove dir
rm -rf ${RELEASE_DIR}

exit 0
