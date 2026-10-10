# Writes git_version.h with the commit the firmware is built from: run on every build (CMakeLists),
# the file rewritten only when it changes. GIT_HASH: its first 7 hex digits, GIT_DIRTY: 1 when the
# tree had changes.
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${SOURCE_DIR}
                OUTPUT_VARIABLE hash OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git diff --quiet HEAD -- WORKING_DIRECTORY ${SOURCE_DIR}
                RESULT_VARIABLE changed ERROR_QUIET)
if(hash STREQUAL "")
  set(hash "0000000")
endif()
string(SUBSTRING "${hash}" 0 7 hash)
if(changed EQUAL 0)
  set(dirty 0)
else()
  set(dirty 1)
endif()
set(content "/* written by git_version.cmake */\n#define GIT_HASH \"${hash}\"\n#define GIT_DIRTY ${dirty}\n")
if(EXISTS ${OUT})
  file(READ ${OUT} old)
endif()
if(NOT "${old}" STREQUAL "${content}")
  file(WRITE ${OUT} "${content}")
endif()
