# Optional component: mbelib (AMBE+2 / IMBE vocoder, ISC licence) for DMR voice.
#
# NOT shipped in this repository: the AMBE+2 algorithm is covered by patents
# in some countries, so fetching and enabling it is the builder's decision.
#   1. tools/fetch_mbelib.sh        (clones it into components/mbelib/mbelib)
#   2. idf.py menuconfig -> SDR: DMR -> "DMR voice (mbelib, AMBE+2)"
# With the option off (default) this component builds nothing and DMR stays
# metadata-only.
set(MBE_DIR ${CMAKE_CURRENT_LIST_DIR}/mbelib)

if(CONFIG_DMR_VOICE_MBELIB)
    if(NOT EXISTS ${MBE_DIR}/mbelib.c)
        message(FATAL_ERROR "DMR voice is enabled in menuconfig but mbelib is missing: run tools/fetch_mbelib.sh")
    endif()
    file(GLOB MBE_SRCS ${MBE_DIR}/*.c)
    idf_component_register(SRCS ${MBE_SRCS} INCLUDE_DIRS mbelib)
    # Upstream desktop code: keep IDF's -Werror=all away from its warnings, and
    # keep float literals single precision for the P4's single-precision FPU.
    # mbe_fastmath.h swaps in a fast cosf() (see its header comment): without
    # it the vocoder alone can take half a core and the voice breaks up.
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -w -fsingle-precision-constant -O2
                           -include ${CMAKE_CURRENT_LIST_DIR}/mbe_fastmath.h)
else()
    idf_component_register()
endif()
