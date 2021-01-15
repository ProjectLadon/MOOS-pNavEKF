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

ADD_EXECUTABLE(pNavEKF ${SRC})

TARGET_LINK_LIBRARIES(pNavEKF
    ${MOOS_LIBRARIES}
    apputil
    mbutil
    m
    pthread
roboticscape)

find_program(CTAGS ctags)
if (CTAGS)
	FIND_FILE(MAKE_CTAGS make_ctags.sh ../..)
    ADD_CUSTOM_TARGET(navekf_ctags ALL
        ${MAKE_CTAGS} .
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS pNavEKF
    )
endif(CTAGS)
