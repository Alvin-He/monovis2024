#! /usr/bin/bash

echo "Script to Deploy to a remote sysroot/location"

deployLoc=$1
if [[ -z $deployLoc ]]; then
    echo "No location path passed in, defaulting deploy location to /sysroot/deploy"
    deployLoc="/sysroot/deploy"
fi;

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $SCRIPT_DIR

echo "removing old artifacts"
for file in "$deployLoc"/*; do 
    echo "removing $file"
    rm -r "$file"
done;

# script will now fail on any error
set -e

echo "copying artifacts from ./cross/artifacts"
for file in "./cross/artifacts"/*; do 
    cp -v -r --no-preserve=ownership,timestamps "$file" "$deployLoc"
done; 

echo "copying configs"
for file in "./config"/*; do 
    cp -v -r --no-preserve=ownership,timestamps "$file" "$deployLoc"
done; 

echo "copying libraries"
echo "copying build server aarch64-linux-gnu"
mkdir "$deployLoc/libs/"
# for file in "/usr/aarch64-linux-gnu/lib/*\.so\.*"; do sharedLibs=$(echo $file); done
# for file in $sharedLibs; do 
#     cp -a -v --no-preserve=ownership,timestamps "$file" "$deployLoc/libs/"
# done;
# echo "patching copied libraries"
# for file in "$deployLoc/libs"/*; do 
#     patchelf --debug --set-rpath "\$ORIGIN" "$file"
# done;

for file in "/usr/aarch64-linux-gnu/lib"/*; do 
    cp -a -v --no-preserve=ownership,timestamps "$file" "$deployLoc/libs/"
done;
echo "patching copied libraries"
for file in "$deployLoc/libs/*\.so\.*"; do sharedLibs=$(echo $file); done
for file in $sharedLibs; do 
    patchelf --debug --set-rpath "\$ORIGIN" "$file"
done;

echo "finished"
