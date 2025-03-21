SET(CURRENT_TARGET_NAME fcl_wrap)

find_package(fcl CONFIG REQUIRED)    
#find_package(Qhull CONFIG REQUIRED)
#find_package(Boost COMPONENTS filesystem serialization program_options)
#find_package(RobWork REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)


file(GLOB SRC_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

add_library(
    ${CURRENT_TARGET_NAME} 
    STATIC
    ${SRC_FILES} 
    ${HEADER_FILES}
)

target_compile_options(${CURRENT_TARGET_NAME} PRIVATE "/bigobj")

target_sources(${CURRENT_TARGET_NAME} PRIVATE ${RES_SHADER_FILES} ${RES_IMAGE_FILES})

target_link_libraries(${CURRENT_TARGET_NAME}
    PRIVATE
        Eigen3::Eigen
        #RW::sdurw_models RW::sdurw_loaders RW::sdurw_invkin RW::sdurw_proximitystrategies RW::sdurw_pathplanning RW::sdurw_pathoptimization RW::sdurw_pathplanners
        fcl
        octomap
)
