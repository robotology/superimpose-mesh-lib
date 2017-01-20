set(TEST_TARGET_NAME test)

set(${TEST_TARGET_NAME}_SRC
        test.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_SRC})

target_link_libraries(${TEST_TARGET_NAME} LibTemplateCMake)

add_test(NAME ${TEST_TARGET_NAME} COMMAND ${TEST_TARGET_NAME})
