# ament_cmake no longer provides ament_target_dependencies(). This links each
# named dependency's modern imported target if it exists, falling back to the
# classic <dep>_INCLUDE_DIRS / <dep>_LIBRARIES variables otherwise.
macro(ament_target_dependencies_compat _target)
  foreach(_dep ${ARGN})
    if(TARGET ${_dep}::${_dep})
      target_link_libraries(${_target} ${_dep}::${_dep})
    elseif(TARGET ${_dep})
      target_link_libraries(${_target} ${_dep})
    else()
      if(${_dep}_INCLUDE_DIRS)
        target_include_directories(${_target} PUBLIC ${${_dep}_INCLUDE_DIRS})
      endif()
      if(${_dep}_LIBRARIES)
        target_link_libraries(${_target} ${${_dep}_LIBRARIES})
      endif()
    endif()
  endforeach()
endmacro()
