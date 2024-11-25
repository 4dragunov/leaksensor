#!/bin/bash
#****************************************************************************
#*
#*  (C) 2024 Andrey Belyakov <andri.belyakov@simbirsoft.com>
#*
#*
#****************************************************************************
srec_cat ..\app\$1\leaks.hex -Intel -fill 0xFF 0x08002000 0x0800FFFC -STM32 0x0800FFFC -o ..\app\$1\leaks_app_witch_crc.hex -Intel
srec_cat ..\boot\$1\boot.hex -Intel ..\app\$1\leaks_appcancat_firmware.sh_witch_crc.hex -Intel -o full_leaks_firmware_$1.hex -Intel