#!/bin/bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
conda activate r5
python udp/udp_node.py
