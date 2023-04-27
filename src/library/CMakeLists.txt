project(ec_library)

# compile ecat library test
add_library(ec_library STATIC
            ${PROJECT_SOURCE_DIR}/protocols/udp/ec_udp.cpp)

target_include_directories(ec_library
    PUBLIC
    include
    PRIVATE
    ${EXTERNAL}/spdlog/include
)

target_link_libraries(ec_library
    PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    pthread
    ecat_master::msgpck_proto
    magic_enum::magic_enum
    matlogger2::matlogger2
    )


install(TARGETS ec_library
    DESTINATION lib)
