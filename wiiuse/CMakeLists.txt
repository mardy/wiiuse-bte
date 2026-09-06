target_include_directories(embedded-game-controller-objects PRIVATE
    ${CMAKE_SOURCE_DIR}/bt-embedded
)

if(ENABLE_COMPAT_FEATURES)
    set(WIIUSE_ALL_FEATURES ON)
endif()
configure_file(features.h.in features.h @ONLY)

add_library(wiiuse STATIC
    ${CMAKE_CURRENT_BINARY_DIR}/features.h
    wiiuse.h
    wpad.c
    wpad.h
)

target_include_directories(wiiuse PRIVATE
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}
)

target_compile_options(wiiuse PRIVATE -Wall)
if(WITH_DEBUG)
    target_compile_definitions(wiiuse PRIVATE WITH_DEBUG)
endif()

target_link_libraries(wiiuse PRIVATE
    embedded-game-controller-objects
    bt-embedded-objects
)

set(PUBLIC_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/features.h
    wiiuse.h
    wpad.h
)
set_target_properties(wiiuse PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
)

install(TARGETS wiiuse
    ARCHIVE DESTINATION "${OGC_ROOT}/lib/${OGC_SUBDIR}"
    PUBLIC_HEADER DESTINATION "${OGC_ROOT}/include/wiiuse"
)
