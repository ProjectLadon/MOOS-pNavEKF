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
