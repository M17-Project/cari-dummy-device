# Locate libcari for this project, in this order:
#   1. an existing cari::cari target (superbuild / a parent project),
#   2. CARI_SOURCE_DIR, if set,
#   3. the git submodule in libcari/,
#   4. a libcari checkout next to this repository (../libcari),
#   5. an installed libcari (find_package(cari)).
# A source tree (2-4) is built as a static library and not installed, so the
# program is self-contained.
set(CARI_SOURCE_DIR "" CACHE PATH "libcari source tree to build in-tree")

function(_cari_add_source dir)
    # variables set here are seen by libcari only, not by this project
    set(BUILD_SHARED_LIBS OFF)
    set(CARI_BUILD_TESTS OFF)
    set(CARI_INSTALL OFF)
    add_subdirectory(${dir} ${CMAKE_BINARY_DIR}/libcari-build EXCLUDE_FROM_ALL)
    message(STATUS "libcari: building from ${dir}")
endfunction()

if(NOT TARGET cari::cari)
    set(_cari_sub "${PROJECT_SOURCE_DIR}/libcari")
    if(CARI_SOURCE_DIR)
        _cari_add_source(${CARI_SOURCE_DIR})
    elseif(EXISTS "${_cari_sub}/CMakeLists.txt")
        _cari_add_source(${_cari_sub})
    elseif(EXISTS "${PROJECT_SOURCE_DIR}/../libcari/CMakeLists.txt")
        _cari_add_source(${PROJECT_SOURCE_DIR}/../libcari)
    else()
        if(EXISTS "${_cari_sub}")
            message(STATUS "libcari/ is empty - run: git submodule update --init")
        endif()
        find_package(cari 2 REQUIRED)
        message(STATUS "libcari: using the installed package (${cari_DIR})")
    endif()
endif()
