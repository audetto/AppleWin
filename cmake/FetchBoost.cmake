include(FetchContent)

message(STATUS "Fetching Boost headers from source...")

# We only need boost headers, not the full build
FetchContent_Declare(
    BoostHeaders
    GIT_REPOSITORY https://github.com/boostorg/boost.git
    GIT_TAG        boost-1.89.0
    GIT_SHALLOW    TRUE
)

FetchContent_GetProperties(BoostHeaders)
if(NOT boostheaders_POPULATED)
    FetchContent_Populate(BoostHeaders)

    # Find all boost library include directories
    file(GLOB BOOST_LIB_DIRS "${boostheaders_SOURCE_DIR}/libs/*/include")

    # Set the Boost variables that FindBoost would normally set
    set(Boost_FOUND TRUE)
    set(BOOST_ROOT "${boostheaders_SOURCE_DIR}")
    set(Boost_INCLUDE_DIRS ${BOOST_LIB_DIRS})
    set(Boost_INCLUDE_DIR ${BOOST_LIB_DIRS})

    # Add all boost library include directories
    foreach(inc_dir ${BOOST_LIB_DIRS})
        include_directories(SYSTEM "${inc_dir}")
    endforeach()

    message(STATUS "Boost headers fetched to: ${boostheaders_SOURCE_DIR}")
    message(STATUS "Boost include dirs: ${BOOST_LIB_DIRS}")
endif()
