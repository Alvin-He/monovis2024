#! /bin/bash

if [[ -z $1 ]]; then
    echo "no install log provided, please provide a mv generated install log"
    exit
fi

echo "WARNING: this will uninstall all installed files listed in $1"
read -p "Press enter to continue"

# clear old logs
log_name="${1}_uninstall.log"
echo "" > $log_name

while IFS= read -r line; do
    target_file=`expr match "$line" ".*->\s*'\(.*\)'"`;
    rm -v -r $target_file >> $log_name;
    dir_name="$(dirname "$line")"
    if [[ -d $dir_name ]] && [[ -n "$(ls -A "$dir_name")" ]]; then
        rm -v -r "$line" >> $log_name;
    fi;
done < $1
