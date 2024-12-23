cmake.exe -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" -DTOOLCHAIN_PREFIX="C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.3 rel1" -DAPPLICATION="LeakDetector"   -DCLASS="classA"   -DCLASSB_ENABLED="ON" -DACTIVE_REGION="LORAMAC_REGION_EU868" -DREGION_EU868="ON" -DREGION_RU864="OFF" -DBOARD="SRWLD01" -DBOARD_RADIO="SX1276MB1LAS"

for /f "tokens=*" %%f in ('wmic cpu get NumberOfCores /value ^| find "="') do set %%f

cmake --build build --parallel %NumberOfCores%