cmake_minimum_required(VERSION 3.20)
project(CAOJoltNative VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)

set(JOLT_ROOT "" CACHE PATH "Jolt source, build, or install directory")
set(JOLT_LIBRARY "" CACHE FILEPATH "Full path to the compiled Jolt library")
set(JOLT_INCLUDE_DIR "" CACHE PATH "Directory containing Jolt/Jolt.h")
option(CAO_FETCH_DEPENDENCIES "Fetch GLFW and nlohmann/json if unavailable" ON)
option(CAO_FETCH_JOLT "Download and compile Jolt when no local Jolt is supplied" ON)
set(CAO_RENDERER "VULKAN" CACHE STRING "Renderer backend (VULKAN or OPENGL)")
set_property(CACHE CAO_RENDERER PROPERTY STRINGS VULKAN OPENGL)
set(JOLT_GIT_TAG "v5.6.0" CACHE STRING "Jolt Physics git tag")

if(CAO_RENDERER STREQUAL "VULKAN")
  find_package(Vulkan REQUIRED)
else()
  find_package(OpenGL REQUIRED)
endif()

find_package(glfw3 CONFIG QUIET)
if(NOT glfw3_FOUND AND CAO_FETCH_DEPENDENCIES)
  FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glfw)
endif()

FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.9b
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(imgui)
add_library(cao-imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp)
target_include_directories(cao-imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
if(DEFINED glfw_SOURCE_DIR)
  target_include_directories(cao-imgui PRIVATE ${glfw_SOURCE_DIR}/include)
endif()
target_compile_definitions(cao-imgui PRIVATE GLFW_INCLUDE_NONE)
target_link_libraries(cao-imgui PUBLIC Vulkan::Vulkan)

find_package(nlohmann_json CONFIG QUIET)
if(NOT nlohmann_json_FOUND AND CAO_FETCH_DEPENDENCIES)
  FetchContent_Declare(json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(json)
endif()

if(JOLT_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${JOLT_ROOT}")
endif()
find_package(Jolt CONFIG QUIET)

if(NOT TARGET Jolt::Jolt AND NOT JOLT_ROOT AND NOT JOLT_LIBRARY AND CAO_FETCH_JOLT)
  FetchContent_Declare(jolt
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG ${JOLT_GIT_TAG}
    GIT_SHALLOW TRUE)
  FetchContent_GetProperties(jolt)
  if(NOT jolt_POPULATED)
    FetchContent_Populate(jolt)
    set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
    set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
    set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
    set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)
    set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)
    set(GENERATE_DEBUG_SYMBOLS OFF CACHE BOOL "" FORCE)
    set(CROSS_PLATFORM_DETERMINISTIC OFF CACHE BOOL "" FORCE)
    set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
    add_subdirectory(${jolt_SOURCE_DIR}/Build ${jolt_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
  if(TARGET Jolt AND NOT TARGET Jolt::Jolt)
    add_library(Jolt::Jolt ALIAS Jolt)
  endif()
endif()

if(NOT TARGET Jolt::Jolt)
  if(NOT JOLT_INCLUDE_DIR AND JOLT_ROOT)
    if(EXISTS "${JOLT_ROOT}/Jolt/Jolt.h")
      set(JOLT_INCLUDE_DIR "${JOLT_ROOT}")
    elseif(EXISTS "${JOLT_ROOT}/include/Jolt/Jolt.h")
      set(JOLT_INCLUDE_DIR "${JOLT_ROOT}/include")
    endif()
  endif()
  if(NOT JOLT_LIBRARY AND JOLT_ROOT)
    find_library(JOLT_LIBRARY NAMES Jolt JoltPhysics PATHS "${JOLT_ROOT}/lib" "${JOLT_ROOT}/Build/Release" "${JOLT_ROOT}/Build/Debug" NO_DEFAULT_PATH)
  endif()
  if(NOT JOLT_INCLUDE_DIR OR NOT JOLT_LIBRARY)
    message(FATAL_ERROR "Jolt was not found. Set -DJOLT_ROOT=/path/to/jolt, or set -DJOLT_INCLUDE_DIR and -DJOLT_LIBRARY.")
  endif()
  add_library(Jolt::Jolt UNKNOWN IMPORTED)
  set_target_properties(Jolt::Jolt PROPERTIES IMPORTED_LOCATION "${JOLT_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${JOLT_INCLUDE_DIR}")
endif()

if(NOT TARGET glfw AND NOT TARGET glfw3)
  message(FATAL_ERROR "GLFW was not found and CAO_FETCH_DEPENDENCIES is OFF")
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  message(FATAL_ERROR "nlohmann/json was not found and CAO_FETCH_DEPENDENCIES is OFF")
endif()

if(CAO_RENDERER STREQUAL "VULKAN")
  add_executable(cao-jolt src/vulkan_main.cpp)
  add_library(cao-editor-core
    src/editor/Scene.cpp
    src/editor/Picking.cpp
    src/editor/JoltBridge.cpp
    src/editor/UiController.cpp
    src/editor/RobotExport.cpp
    src/editor/RobotRecorder.cpp)
  target_include_directories(cao-editor-core PUBLIC src)
  add_library(cao-vulkan-renderer
    src/renderer/VulkanRenderer.cpp
    src/renderer/ShaderLoader.cpp
    src/renderer/VulkanResources.cpp)
  target_include_directories(cao-vulkan-renderer PUBLIC src)
  add_executable(cao-headless src/headless_main.cpp)
  target_include_directories(cao-headless PRIVATE src)
  target_link_libraries(cao-headless PRIVATE cao-editor-core)
else()
  add_executable(cao-jolt src/main.cpp)
endif()

target_include_directories(cao-jolt PRIVATE src)
if(TARGET glfw)
  set(CAO_GLFW_TARGET glfw)
else()
  set(CAO_GLFW_TARGET glfw3)
endif()

if(CAO_RENDERER STREQUAL "VULKAN")
  target_compile_definitions(cao-jolt PRIVATE CAO_RENDERER_VULKAN=1)
  target_link_libraries(cao-editor-core PUBLIC Jolt::Jolt nlohmann_json::nlohmann_json)
  target_link_libraries(cao-vulkan-renderer PUBLIC Vulkan::Vulkan cao-editor-core)
  target_link_libraries(cao-jolt PRIVATE Vulkan::Vulkan ${CAO_GLFW_TARGET}
    nlohmann_json::nlohmann_json Jolt::Jolt cao-editor-core cao-vulkan-renderer cao-imgui)
else()
  target_compile_definitions(cao-jolt PRIVATE CAO_RENDERER_OPENGL=1)
  target_link_libraries(cao-jolt PRIVATE OpenGL::GL ${CAO_GLFW_TARGET}
    nlohmann_json::nlohmann_json Jolt::Jolt)
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(cao-jolt PRIVATE dl pthread)
endif()

if(MSVC)
  target_compile_definitions(cao-jolt PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

find_program(GLSLC_EXECUTABLE glslc REQUIRED)
add_custom_target(cao-shaders ALL
  COMMAND ${GLSLC_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/cad.vert -o ${CMAKE_CURRENT_BINARY_DIR}/cad.vert.spv
  COMMAND ${GLSLC_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/cad.frag -o ${CMAKE_CURRENT_BINARY_DIR}/cad.frag.spv
  DEPENDS assets/shaders/cad.vert assets/shaders/cad.frag)

add_dependencies(cao-jolt cao-shaders)
if(TARGET cao-headless)
  add_dependencies(cao-headless Jolt)
endif()
if(TARGET cao-vulkan-renderer)
  add_dependencies(cao-vulkan-renderer cao-shaders)
endif()

install(TARGETS cao-jolt cao-headless RUNTIME DESTINATION bin)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cad.vert.spv
  ${CMAKE_CURRENT_BINARY_DIR}/cad.frag.spv
  DESTINATION share/cao-jolt/shaders)
