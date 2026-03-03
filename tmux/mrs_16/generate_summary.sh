#!/bin/bash
seq 1 82 | xargs -n1 -I{} python3 SIM_2/data.py --sim {}
