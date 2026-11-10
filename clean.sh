#!/bin/bash
rm -fr logs install build log

if [ "${1}" = "-c" ]; then
    rm -rf mediapipe
else
    echo "Not cleaning mediapipe use \`./clean.sh -c\`" to clean mediapipe.
fi