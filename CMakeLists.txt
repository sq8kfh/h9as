cmake_minimum_required(VERSION 3.10)

project(h9as VERSION 0.2)
set(NODE_TYPE 5)
set(MMCU atmega16m1)
set(FREQ 16000000)

if (MMCU STREQUAL atmega64m1)
    set(BOOTSTART 0xf800) #in bytes 64m1
    set(FUSE -U lfuse:w:0xf4:m -U hfuse:w:0xd5:m -U efuse:w:0xfe:m)
elseif(MMCU STREQUAL atmega32m1)
    set(BOOTSTART 0x7800) #in bytes 32m1
    set(FUSE -U lfuse:w:0xf4:m -U hfuse:w:0xd3:m -U efuse:w:0xfe:m)
elseif(MMCU STREQUAL atmega16m1)
    set(BOOTSTART 0x3800) #in bytes 16m1
    set(FUSE -U lfuse:w:0xf4:m -U hfuse:w:0xd3:m -U efuse:w:0xfe:m)
endif()

set(PROGRAMMER jtag3isp)

set(SOURCE_FILES main.c can/can.c can/can.h can/h9msg.h)

if(BOOTSTART)
    add_subdirectory(bootloader)
endif(BOOTSTART)

set(PATHS "/usr/local/bin/ /opt/local/bin/")

function(add_additional_clean_file FILENAME)
    get_directory_property(EXTRA_CLEAN_FILES ADDITIONAL_MAKE_CLEAN_FILES)
    set_directory_properties(
            PROPERTIES
            ADDITIONAL_MAKE_CLEAN_FILES "${EXTRA_CLEAN_FILES};${FILENAME}"
    )

    set_source_files_properties(${FILENAME} PROPERTIES GENERATED TRUE)
endfunction(add_additional_clean_file)

find_program(AVR_C_COMPILER avr-gcc PATHS ${PATHS})
find_program(AVR_OBJCOPY avr-objcopy PATHS ${PATHS})
find_program(AVR_SIZE avr-size PATHS ${PATHS})
find_program(AVRDUDE avrdude PATHS ${PATHS})
find_program(SREC_CAT srec_cat PATHS ${PATHS})

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)
set(CMAKE_C_COMPILER ${AVR_C_COMPILER})

if(NOT BUILD_DIRECTORY)
    set(BUILD_DIRECTORY /build CACHE STRING "Setting build directory to /build")
endif(NOT BUILD_DIRECTORY)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}${BUILD_DIRECTORY}")

if(NOT SOURCE_FILES)
    message(FATAL_ERROR "SOURCE_FILES variable is not defined")
endif(NOT SOURCE_FILES)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

if(NOT FREQ)
    message(FATAL_ERROR "FREQ variable is not defined")
endif(NOT FREQ)

if(NOT PROGRAMMER)
    message(FATAL_ERROR "PROGRAMMER variable is not defined")
endif(NOT PROGRAMMER)

if(NOT MMCU)
    message(FATAL_ERROR "MMCU variable is not defined")
endif(NOT MMCU)

configure_file(config.h.in config.h)

message(STATUS "Building ${PROJECT_NAME} for ${MMCU} with frequency ${FREQ}")
message(STATUS "Source files: ${SOURCE_FILES}")
message(STATUS "Using flags: ${CMAKE_C_FLAGS}")

set(ELF_FILENAME ${PROJECT_NAME}.elf)

set(ELF_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${ELF_FILENAME}")
set(HEX_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex")
set(EEP_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.eep")
set(HEX_WITH_BL_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}_with_bl.hex")

add_additional_clean_file(${ELF_FILE})
add_additional_clean_file(${HEX_FILE})
add_additional_clean_file(${EEP_FILE})

if(BOOTSTART)
    set(BOOTSTART_FLAG "-DBOOTSTART=${BOOTSTART}")
    add_dependencies(${PROJECT_NAME} h9avr-bootloader)
    set(HEX_WITH_BL_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}_with_bl.hex")
    add_additional_clean_file(${HEX_WITH_BL_FILE})
endif(BOOTSTART)

set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES
        OUTPUT_NAME ${ELF_FILENAME}
        COMPILE_FLAGS "-mmcu=${MMCU} -I. -gdwarf-2 ${BOOTSTART_FLAG} -DF_CPU=${FREQ}UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wno-unknown-pragmas -Wstrict-prototypes -Wundef -std=gnu99 -Wl,--gc-section"
        LINK_FLAGS "-mmcu=${MMCU} -I. -gdwarf-2 -DF_CPU=${FREQ}UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wno-unknown-pragmas -Wstrict-prototypes -Wundef -std=gnu99 -Wundef -MD -MP -MF .dep/main.elf.d --output ${ELF_FILE} -Wl,-lc,--entry=main"
)

add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom ${ELF_FILE} ${HEX_FILE}
        COMMAND ${AVR_OBJCOPY} -O ihex -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 ${ELF_FILE} ${EEP_FILE}
        COMMAND ${AVR_SIZE} ${ELF_FILE} --mcu=${MMCU} --format=avr
        COMMENT "Post processsing"
)

add_custom_target("flash"
        ${AVRDUDE} -p ${MMCU} -c ${PROGRAMMER} -U ${HEX_FILE}
        DEPENDS ${PROJECT_NAME}
        COMMENT "Flash uploading"
        )

if(BOOTSTART)
    add_custom_target(${PROJECT_NAME}_with_bl
            COMMAND ${SREC_CAT} ${HEX_FILE} -I bootloader/build/h9avr-bootloader.hex -I -o ${HEX_WITH_BL_FILE} -I
            DEPENDS ${PROJECT_NAME} h9avr-bootloader
            COMMENT "Flashs combinating"
            )

    add_custom_target("flash_all"
            ${AVRDUDE} -p ${MMCU} -c ${PROGRAMMER} -U ${HEX_WITH_BL_FILE}
            DEPENDS ${PROJECT_NAME}_with_bl
            COMMENT "Full flash uploading"
            )
endif(BOOTSTART)

add_custom_target("eeprom"
        ${AVRDUDE} -p ${MMCU} -c ${PROGRAMMER} -U eeprom:w:${EEP_FILE}
        DEPENDS ${PROJECT_NAME}
        COMMENT "Eeprom uploading"
        )

add_custom_target("fuse"
        ${AVRDUDE} -p ${MMCU} -c ${PROGRAMMER} ${FUSE}
        COMMENT "Fuse uploading"
        )
