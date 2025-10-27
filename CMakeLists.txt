cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Utility function for fetching dependencies in one line.
function(fetch_dependency AGIT_REPO AGIT_TAG ANAME)
    # Remaining args are variable=value pairs for cache options
    set(options)
    set(oneValueArgs)
    set(multiValueArgs)
    cmake_parse_arguments(FETCHDEP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Declare the dependency
    FetchContent_Declare(
        ${ANAME}
        GIT_REPOSITORY ${AGIT_REPO}
        GIT_TAG        ${AGIT_TAG}
        SOURCE_DIR     ${CMAKE_SOURCE_DIR}/dep/${ANAME}
    )
    
    # Apply cache variables with intelligent type detection and FORCE override
    # CRITICAL: Set these BEFORE FetchContent_Declare
    foreach(arg IN LISTS ARGN)
        string(REPLACE "=" ";" kv "${arg}")
        list(GET kv 0 key)
        list(GET kv 1 val)
       
        # Determine the appropriate cache variable type based on value
        if(val MATCHES "^(ON|OFF|TRUE|FALSE)$")
            # Boolean values
            set(${key} ${val} CACHE BOOL "Forced by fetch_dependency for ${ANAME}" FORCE)
        else()
            # Everything else as STRING
            set(${key} ${val} CACHE STRING "Forced by fetch_dependency for ${ANAME}" FORCE)
        endif()
    endforeach()
    
    # Actually fetch it - cache variables should now be available to subdirectory
    FetchContent_MakeAvailable(${ANAME})
endfunction()

include(FetchContent)

# Fetch Dependencies
fetch_dependency(https://github.com/assimp/assimp.git v5.4.3 assimp BUILD_SHARED_LIBS=OFF ASSIMP_INSTALL=OFF ASSIMP_INSTALL_PDB=OFF ASSIMP_BUILD_TESTS=OFF)
fetch_dependency(https://github.com/FermionicChaos/geodesy-math.git master geodesy-math)

set(PROJECT_NAME "geodesy-physics")

file(GLOB_RECURSE INC
    "inc/*.h"
)

file(GLOB_RECURSE SRC
    "src/*.h"
    "src/*.cpp"
    "src/*.c"
)

project(${PROJECT_NAME})

add_library(${PROJECT_NAME} ${INC} ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc/)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/dep/assimp/include/)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_BINARY_DIR}/_deps/assimp-build/include/)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/dep/geodesy-math/inc/)

target_link_libraries(${PROJECT_NAME} PUBLIC assimp)
target_link_libraries(${PROJECT_NAME} PUBLIC geodesy-math)
