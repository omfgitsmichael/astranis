set(UTILS ${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_library(astranisInterview-libs
  ${UTILS}/simUtils.cpp
  ${UTILS}/controlUtils.cpp
  ${UTILS}/sensorUtils.cpp
)

target_include_directories(astranisInterview-libs PUBLIC
    ${PROJECT_BASE_DIR}/include
    ${PROJECT_BASE_DIR}/thirdParty/eigen
)

target_link_libraries(astranisInterview-libs PUBLIC
  eigen
)