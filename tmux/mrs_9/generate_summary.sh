#!/bin/bash
seq 1 23 | xargs -n1 -I{} python3 data.py --sim {}
