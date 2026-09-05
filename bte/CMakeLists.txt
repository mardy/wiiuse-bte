add_library(bte STATIC
    bte.c
)

target_compile_options(bte PRIVATE -Wall)
target_link_libraries(bte PRIVATE
    bt-embedded-objects
)

set_target_properties(bte PROPERTIES
    PUBLIC_HEADER bte.h
)

install(TARGETS bte
    ARCHIVE DESTINATION "${OGC_ROOT}/lib/${OGC_SUBDIR}"
    PUBLIC_HEADER DESTINATION "${OGC_ROOT}/include/bte"
)
