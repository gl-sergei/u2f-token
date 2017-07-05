#!/bin/bash

openocd -f interface/stlink-v2.cfg -f target/stm32f1x-p.cfg -c "program build/u2f.elf verify reset exit"
