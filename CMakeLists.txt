cmake_minimum_required(VERSION 3.13)

project(wiiuse-bte
    LANGUAGES C
)

option(WITH_DEBUG "Build with debugging enabled" OFF)

set(CMAKE_C_STANDARD 11)

include(GNUInstallDirs)

include(FetchContent)

if (NOT EXISTS "${CMAKE_SOURCE_DIR}/bt-embedded/CMakeLists.txt")
    FetchContent_Populate(bt-embedded
        GIT_REPOSITORY https://github.com/embedded-game-controller/bt-embedded.git
        GIT_TAG        c7ebcdd17d740f3311b9764840b3f1ed2a49eb5f
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
        GIT_TAG        aacfc38ebe08e0f2a6e5e2c81558ee272ae452a1
        SOURCE_DIR     ${CMAKE_SOURCE_DIR}/embedded-game-controller
    )
endif()
set(BUILD_EXAMPLE OFF CACHE INTERNAL "")
# To find our bt-embedded.pc file
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/wiiuse:$ENV{PKG_CONFIG_PATH}")
add_subdirectory(embedded-game-controller)

add_subdirectory(bte)
add_subdirectory(wiiuse)
