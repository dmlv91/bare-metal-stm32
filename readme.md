Practical DIY embedded software development project based on STM32F0* MCU and Bosch BME680 environmental sensor. 
Main idea behind the project is to NOT use any SDK libraries if possible and write everything from scratch to achieve better knowledge and understanding of embedded software development, compiling, building and MCU work principles in general.

If someone would want to setup the same dev environment in vscode then here's what should be inserted into c_cpp_properties.json :

{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/PATH_TO_SDK/STM32CubeF0/Drivers/CMSIS/Include",
                "/PATH_TO_SDK/STM32CubeF0/Drivers/CMSIS/Device/ST/STM32F0xx/Include",
                "/PATH_TO_SDK/STM32CubeF0/Drivers/STM32F0xx_HAL_Driver/Inc",
                "${workspaceFolder}/bare-metal-stm32/projects/gpio_plus_interfaces/utils/**",
                "${workspaceFolder}/bare-metal-stm32/projects/gpio_plus_interfaces/sensor/**",
                "${workspaceFolder}/bare-metal-stm32/projects/gpio_plus_interfaces/**"
            ],
            "defines": [
                "STM32F091xC",
                "USE_I2C",
                // "USE_SPI"
            ],
            "compilerPath": "/opt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc",
            "cStandard": "c17",
            "cppStandard": "c++98",
            "intelliSenseMode": "linux-gcc-arm"
        }
    ],
    "version": 4
}