target_include_directories(embedded-game-controller-objects PRIVATE
    ${CMAKE_SOURCE_DIR}/bt-embedded
)

add_library(wiiuse STATIC
    wiiuse.h
    wpad.c
    wpad.h
)

target_include_directories(wiiuse PRIVATE
    ${CMAKE_SOURCE_DIR}
)

target_compile_options(wiiuse PRIVATE -Wall)
target_link_libraries(wiiuse PRIVATE
    embedded-game-controller-objects
    bt-embedded-objects
)

set(PUBLIC_HEADERS
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
