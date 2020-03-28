
cmake_minimum_required(VERSION 3.4.1)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI})

add_library( # Sets the name of the library.
             kebii_imageutils

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/ProcessImage.cpp )

find_library( # Sets the name of the path variable.
              log-lib
              log )

target_link_libraries( # Specifies the target library.
                       kebii_imageutils

                        jnigraphics
                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )