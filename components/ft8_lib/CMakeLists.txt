# ft8_lib (Karlis Goba, MIT license - see LICENSE in this folder), vendored
# from https://github.com/kgoba/ft8_lib (commit 9fec6ca, 2025-08-24).
# Only the decode-side sources are included (no encode.c - this receiver never
# transmits). Consumers include headers as "ft8/decode.h", "ft8/message.h"...,
# hence the component root (not ./ft8) is the public include directory.
idf_component_register(
    SRCS
        ft8/constants.c
        ft8/crc.c
        ft8/decode.c
        ft8/ldpc.c
        ft8/message.c
        ft8/text.c
    INCLUDE_DIRS .
)

# Upstream code is written for desktop gcc/clang; keep ESP-IDF's -Werror=all
# from turning its harmless warnings into build failures, and make float
# literals single precision (the ESP32-P4 FPU has no double-precision unit).
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-sign-compare
    -Wno-unused-const-variable
    -Wno-format
    -fsingle-precision-constant
)
