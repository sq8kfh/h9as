cmake_minimum_required(VERSION 2.8.4)

project(h9-eu1)

set(PROGRAMMER jtag3isp)
set(MMCU atmega16m1)
set(FREQ 16000000)
set(BOOTSTART 0x3800) #in bytes

set(SOURCE_FILES main.c can/can.c can/common.c can/common.h can/h9msg.h can/h9node.h)

if(BOOTSTART)
    add_subdirectory(can)
endif(BOOTSTART)

set(CMAKE_SYSTEM_NAME Generic)

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
    add_dependencies(${PROJECT_NAME} h9-bootloader)
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
            COMMAND ${SREC_CAT} ${HEX_FILE} -I can/build/h9-bootloader.hex -I -o ${HEX_WITH_BL_FILE} -I
            DEPENDS ${PROJECT_NAME} h9-bootloader
            COMMENT "Flashs combinating"
            )

    add_custom_target("flashall"
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
