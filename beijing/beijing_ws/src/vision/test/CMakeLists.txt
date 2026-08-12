enable_testing()

add_executable(test_yolov8 model/test_yolov8.cpp)
target_link_libraries(test_yolov8 yolov8_model pthread
                                  gtest
                                  gtest_main
) 

add_executable(validate model/validator.cpp)
target_link_libraries(validate yolov8_model
) 

add_executable(test_utils base/test_utils.cpp)
target_link_libraries(test_utils booster_vision_base pthread
                                  gtest
                                  gtest_main
)

add_executable(test_intrin base/test_intrin.cpp)
target_link_libraries(test_intrin booster_vision_base pthread
                                  gtest
                                  gtest_main
)

add_executable(test_data_syncer base/test_data_syncer.cpp)
target_link_libraries(test_data_syncer booster_vision_base pthread
                                       gtest
                                       gtest_main
)

add_executable(test_color_classifier test_color_classifier.cpp)
target_link_libraries(test_color_classifier booster_vision_base pthread yolov8_model
)

add_test(NAME utest_utils COMMAND test_utils)
add_test(NAME utest_intrin COMMAND test_intrin)
add_test(NAME utest_data_syncer COMMAND test_data_syncer)
add_test(NAME utest_yolov8 COMMAND test_yolov8 ${PROJECT_SOURCE_DIR}/config/vision.yaml ${PROJECT_SOURCE_DIR}/test/imgs/detection/raw.jpg)

install(TARGETS test_yolov8
                test_utils
                test_intrin
                test_data_syncer
        DESTINATION lib/${PROJECT_NAME})
