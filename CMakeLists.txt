set(SOURCES
    "src/ego_estimator.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/ego_estimator.h"
)

include_directories(include)
add_library(ego_estimator MODULE ${SOURCES} ${HEADERS})
target_link_libraries(ego_estimator PRIVATE lmscore)
