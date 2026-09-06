cmake_minimum_required(VERSION 3.13)

project(wiiuse-bte
    LANGUAGES C
)

option(WITH_DEBUG "Build with debugging enabled" OFF)
option(ENABLE_COMPAT_FEATURES "Build with all legacy features enabled" OFF)

set(CMAKE_C_STANDARD 11)

include(GNUInstallDirs)

include(FetchContent)

if (NOT EXISTS "${CMAKE_SOURCE_DIR}/bt-embedded/CMakeLists.txt")
    FetchContent_Populate(bt-embedded
        GIT_REPOSITORY https://github.com/embedded-game-controller/bt-embedded.git
        GIT_TAG        746f51f0ff91adfe07d9a3f986dcd4b4320f01f7
        SOURCE_DIR     ${CMAKE_SOURCE_DIR}/bt-embedded
    )
endif()
set(BUILD_EXAMPLE OFF CACHE INTERNAL "")
set(BUILD_TESTS OFF CACHE INTERNAL "")
set(WITH_OPENOBEX OFF CACHE INTERNAL "")
add_subdirectory(bt-embedded)

if (NOT EXISTS "${CMAKE_SOURCE_DIR}/embedded-game-controller/CMakeLists.txt")
    FetchContent_Populate(egc
        GIT_REPOSITORY https://github.com/embedded-game-controller/embedded-game-controller.git
        GIT_TAG        e0c73fdfde51e1556fcfb364c3893493c5f005f3
        SOURCE_DIR     ${CMAKE_SOURCE_DIR}/embedded-game-controller
    )
endif()
set(BUILD_EXAMPLE OFF CACHE INTERNAL "")
# To find our bt-embedded.pc file
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/wiiuse:$ENV{PKG_CONFIG_PATH}")
add_subdirectory(embedded-game-controller)

add_subdirectory(bte)
add_subdirectory(wiiuse)
