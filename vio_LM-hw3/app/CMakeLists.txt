add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)

add_executable(testCurveFitting_vulcan CurveFitting_hw3.cpp)
target_link_libraries(testCurveFitting_vulcan ${PROJECT_NAME}_backend)

