#--------------------------------------------------------
# The CMakeLists.txt for:                       pNavEKF
# Author(s):
#--------------------------------------------------------

SET(SRC
  NavEKF.cpp
  NavEKF_increment.cpp
  NavEKF_Info.cpp
  main.cpp
)

# Googletest CMake example begin
# ==============================
# Download and unpack googletest at configure time
# configure_file(tests/CMakeLists.txt.gtest googletest-download/CMakeLists.txt)
# execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
#   RESULT_VARIABLE result
#   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
# if(result)
#   message(FATAL_ERROR "CMake step for googletest failed: ${result}")
# endif()
# execute_process(COMMAND ${CMAKE_COMMAND} --build .
#   RESULT_VARIABLE result
#   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
# if(result)
#   message(FATAL_ERROR "Build step for googletest failed: ${result}")
# endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#
# # Add googletest directly to our build. This defines
# # the gtest and gtest_main targets.
# add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
#                  ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
#                  EXCLUDE_FROM_ALL)
# END Googletest block

ADD_EXECUTABLE(pNavEKF ${SRC})

TARGET_LINK_LIBRARIES(pNavEKF
    ${MOOS_LIBRARIES}
    apputil
    mbutil
    m
    pthread
    roboticscape
)

find_program(CTAGS ctags)
if (CTAGS)
	FIND_FILE(MAKE_CTAGS make_ctags.sh ../..)
    ADD_CUSTOM_TARGET(navekf_ctags ALL
        ${MAKE_CTAGS} .
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS pNavEKF
    )
endif(CTAGS)

SET(INCREMENT_TEST_SRC
    NavEKF_increment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/NavIncrementTest.cpp
)

ADD_EXECUTABLE(pNavEKF_NavIncrementTest ${INCREMENT_TEST_SRC})

TARGET_LINK_LIBRARIES(pNavEKF_NavIncrementTest
    ${MOOS_LIBRARIES}
    apputil
    mbutil
    m
    pthread
    roboticscape
    gtest
)

ADD_TEST(NAME increment_test COMMAND pNavEKF_NavIncrementTest)
