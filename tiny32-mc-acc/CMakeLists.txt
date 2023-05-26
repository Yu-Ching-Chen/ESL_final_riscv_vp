file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable(tiny32-mc-acc
        mc_main.cpp
        ${HEADERS})

target_link_libraries(tiny32-mc-acc rv32 platform-common gdb-mc
	${Boost_LIBRARIES} systemc pthread)

add_definitions("-DSC_INCLUDE_FX")
INSTALL(TARGETS tiny32-mc-acc RUNTIME DESTINATION bin)
