#!/bin/bash

echo "input grasp target name"
read input_word

echo "-----finding grasp "$input_word" target and adjust-----"

python adjust_base.py "$input_word"

if [ $? -eq 0 ]; then
    echo "Grasp Adjustment Successful"
else
    echo "Grasp Adjustment Failed"
    exit 1
fi

python init_grasp_arm.py

if [ $? -eq 0 ]; then
    echo "Grasp Adjustment Successful"
else
    echo "Grasp Adjustment Failed"
    exit 1
fi

echo "-----start grasping-----"

python camera_demo.py