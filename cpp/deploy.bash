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

echo "copying artifacts"
for file in "./build/artifacts"/*; do 
    cp -v -r "$file" "$deployLoc"
done; 

echo "copying configs"
for file in "./config"/*; do 
    cp -v -r "$file" "$deployLoc"
done; 

echo "finished"
