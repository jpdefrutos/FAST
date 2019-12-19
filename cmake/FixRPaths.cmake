file(GLOB installedSOs "${CMAKE_INSTALL_PREFIX}/fast/lib/*.so*")
foreach(SO ${installedSOs})
    message("-- Setting runtime path of ${SO}")
    execute_process(COMMAND patchelf --set-rpath "$ORIGIN/../lib" ${SO})
endforeach()