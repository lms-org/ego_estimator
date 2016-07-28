set(SOURCES
    "src/ego_estimator.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/ego_estimator.h"
    "include/filter/measurement_model.h"
    "include/filter/system_model.h"
)

include_directories(
        include
        /usr/local/include
        /usr/include
        /usr/local/include/eigen3
        /usr/include/eigen3
)

add_definitions(-DEIGEN_NO_DEBUG)

add_library(ego_estimator MODULE ${SOURCES} ${HEADERS})
target_link_libraries(ego_estimator PRIVATE lmscore math_lib sensor_utils street_environment phoenix_CC2016_service)
