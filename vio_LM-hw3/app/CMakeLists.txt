# mathgl
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/lib")
set(mathgl_DIR "/usr/local/lib")
find_package(MathGL2 REQUIRED)
include_directories(${MATHGL_INCLUDE_DIRS})
set(used_mgl_so
    ${mathgl_DIR}/libmgl.so
    ${mathgl_DIR}/libmgl-fltk.so
)

add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting 
    ${PROJECT_NAME}_backend
    ${MATHGL_LIBRARIES}
    ${used_mgl_so}
)

add_executable(testCurveFitting_vulcan CurveFitting_hw3.cpp)
target_link_libraries(testCurveFitting_vulcan 
    ${PROJECT_NAME}_backend 
    ${MATHGL_LIBRARIES}
    ${used_mgl_so}
)

