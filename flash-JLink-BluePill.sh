#!/bin/bash
cd `dirname $0`
JLinkExe -device STM32F103C8 -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash_BluePill.jlink
cd -
exit 0