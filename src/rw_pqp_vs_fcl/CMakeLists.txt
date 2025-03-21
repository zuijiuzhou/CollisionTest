SET(CURRENT_TARGET_NAME rw_pqp_vs_fcl)

find_package(assimp CONFIG REQUIRED)
# find_package(sdl2 CONFIG REQUIRED)
find_package(SDL3 CONFIG REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)
find_package(fcl CONFIG REQUIRED)    
find_package(Qhull CONFIG REQUIRED)
find_package(Boost COMPONENTS filesystem serialization program_options)
find_package(RobWork REQUIRED)
find_package(glm REQUIRED)
find_package(Eigen3 REQUIRED)

qt_standard_project_setup()


file(GLOB SRC_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

add_executable(
    ${CURRENT_TARGET_NAME} 
    ${SRC_FILES} 
    ${HEADER_FILES}
)

target_compile_options(${CURRENT_TARGET_NAME} PRIVATE "/bigobj")

target_sources(${CURRENT_TARGET_NAME} PRIVATE ${RES_SHADER_FILES} ${RES_IMAGE_FILES})

target_link_libraries(${CURRENT_TARGET_NAME}
    PRIVATE
        assimp::assimp
        Eigen3::Eigen
        SDL3::SDL3 
        Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
        RW::sdurw_models RW::sdurw_loaders RW::sdurw_invkin RW::sdurw_proximitystrategies RW::sdurw_pathplanning RW::sdurw_pathoptimization RW::sdurw_pathplanners
        fcl
        fcl_wrap
)
