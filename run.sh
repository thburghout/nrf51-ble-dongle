#!/bin/bash
#set -e
#make
#make merge
#nrfjprog -e
#nrfjprog --reset --program _build/merged_diver_dongle.hex
#nrfjprog -p

make
make nrf51822_s110 flash