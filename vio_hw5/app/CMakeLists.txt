add_executable(testMonoBA TestMonoBA.cpp)
target_link_libraries(testMonoBA ${PROJECT_NAME}_backend ${GLOG_LIBRARIES})

add_executable(testCurveFitting5 CurveFitting.cpp)
target_link_libraries(testCurveFitting5 ${PROJECT_NAME}_backend ${GLOG_LIBRARIES})