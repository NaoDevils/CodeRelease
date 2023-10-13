# enable warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wliteral-conversion
        -Wunreachable-code
        -Wno-switch                 # ignore warnings for unhandled enumeration values because numOf<ENUM>s is never used in switch statements
    )

    set(COMPILER_MOST_WARNINGS
        -Wconversion
        -Wno-sign-conversion
    )
    set(COMPILER_MORE_WARNINGS
        ${COMPILER_MOST_WARNINGS}
        -Wno-implicit-int-float-conversion # used very often and not critical
    )
    
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(
        /W4     # warnings level 4
        /wd4100 # unreferenced formal parameter
        /wd4458 # declaration of 'x' hides class member
    )
endif()
