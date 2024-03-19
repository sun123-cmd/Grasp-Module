#!/bin/bash
export CUDA_VISIBLE_DEVICES=1
python demo_camera_langsam.py --checkpoint_path ./checkpoints/checkpoint_detection.tar --filter oneeuro --debug