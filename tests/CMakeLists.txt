MESSAGE(Included library for tests ${PROJECT_SOURCE_DIR}/src)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

add_executable(test_treeinertialparameters test_treeinertialparameters.cpp)
add_executable(test_chaininertialparameters test_chaininertialparameters.cpp)
add_executable(test_netwrenchregressor test_netwrenchregressor.cpp)
add_executable(test_kdl test_kdl.cpp)
 

target_link_libraries(test_treeinertialparameters rbipl orocos-kdl)
target_link_libraries(test_chaininertialparameters rbipl orocos-kdl)
target_link_libraries(test_netwrenchregressor rbipl orocos-kdl)
target_link_libraries(test_kdl rbipl orocos-kdl)
 
install(TARGETS test_chaininertialparameters
  # IMPORTANT: Add the bar executable to the "export-set"
  EXPORT RBIPLTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
