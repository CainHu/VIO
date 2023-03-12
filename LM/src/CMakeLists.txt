add_library(LM_src vertex.cpp edge.cpp problem.cpp)

target_include_directories(LM_src PUBLIC ${LM_SOURCE_DIR})

target_link_libraries(LM_src PRIVATE LM)