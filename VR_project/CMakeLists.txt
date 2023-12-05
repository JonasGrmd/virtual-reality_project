# CMakeList.txt : projet CMake pour VR_project, incluez la source et définissez
# la logique spécifique au projet ici.
#

# Ajoutez une source à l'exécutable de ce projet.
project("VR_project")

add_executable (VR_project "Sources/main.cpp" "Headers/object.h" "Headers/shader.h" "Headers/camera.h" "Shaders/vertexShader.txt" "Shaders/fragmentShader.txt" "Shaders/screenFragmentShader.txt" "Shaders/screenVertexShader.txt")
target_link_libraries(VR_project PUBLIC OpenGL::GL glfw glad LinearMath BulletCollision BulletDynamics)


if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET VR_project PROPERTY CXX_STANDARD 20)
endif()

add_compile_definitions(PATH_TO_OBJECTS="${CMAKE_CURRENT_SOURCE_DIR}/objects")
add_compile_definitions(PATH_TO_TEXTURE="${CMAKE_CURRENT_SOURCE_DIR}/textures")
add_compile_definitions(PATH_TO_SHADERS="${CMAKE_CURRENT_SOURCE_DIR}/Shaders")

# TODO: Ajoutez des tests et installez des cibles si nécessaire.