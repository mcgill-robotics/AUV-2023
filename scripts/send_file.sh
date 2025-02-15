#!/bin/bash

SOURCE_PATH=""
DESTINATION_PATH=""

while getopts "s:d:h:" options; do
  case "${options}" in
    s)
      SOURCE_PATH=${OPTARG} ;;
    d)
      DESTINATION_PATH=${OPTARG} ;;
    h)
      echo "./send_file.sh -s <path-to-source> -d <path-to-destination-on-jetson>"
      exit ;;
  esac
done

if [[ SOURCE_PATH == "" ]]; then
  echo "Missing source file path!"
  exit
fi

if [[ DESTINATION_PATH == "" ]]; then
  echo "Missing destination path!"
  exit
fi

scp -pq "${SOURCE}" jetson@192.168.0.105:"${DESTINATION}"
