#****************************************************************************
#*
#*  (C) 2024 Andrey Belyakov <andri.belyakov@simbirsoft.com>
#*
#*
#****************************************************************************
srec_cat.exe ..\app\%1\leaksdet.hex -Intel -fill 0xFF 0x08002000 0x0800FFFC -STM32 0x0800FFFC -o ..\..\app\%1\leaks_app_with_crc.hex -Intel
srec_cat.exe ..\boot\%1\boot.hex -Intel ..\app\%1\leaks_app_with_crc.hex -Intel -o full_leaks_firmware.hex -Intel