# -------------------------------------------------
# Usage notes
# -------------------------------------------------
# A fetcher project should set required and optional variables with details on the FMU to be created:
#   FMU_MODEL_IDENTIFIER (required)
#   FMU_MAIN_DIRECTORY (optional, use CMAKE_BINARY_DIR if not set)
#   FMU_DEPENDENCIES (optional)
#   FMU_RESOURCES_DIRECTORY (optional)
#   FMU_GUID_NAMESPACE (optional, default randomly-generated)
#   FMU_TESTING (optional, default OFF)
#   USE_CUSTOM_TYPESPLATFORM (optional, default OFF)
#   FMU_MSG_PREFIX (optional, default "")
#
# On return, this script makes the following variables available to the fetcher project:
#   FMI_PLATFORM
#   FMU_DIRECTORY
#   FMU_FILENAME
#   FMU_RUNTIME_OUTPUT_DIRECTORY
#   FMU_SOURCES
# -------------------------------------------------

# Set the minimum required C++ standard to C++14, allow C++17 if available
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_STANDARD 17)

# -------------------------------------------------

# Optional setting: prefix for all messages issued from this script (default: empty).
if(NOT FMU_MSG_PREFIX)
  set(FMU_MSG_PREFIX "")
endif()

# Optional setting: output location for the FMU directory (default: CMAKE_BINARY_DIR)
if(NOT FMU_MAIN_DIRECTORY)
  set(FMU_MAIN_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# Set directory and filename for the generated FMU
set(FMU_DIRECTORY "${FMU_MAIN_DIRECTORY}/${FMU_MODEL_IDENTIFIER}")
set(FMU_FILENAME "${FMU_DIRECTORY}/${FMU_MODEL_IDENTIFIER}.fmu")
message(STATUS "${FMU_MSG_PREFIX}FMU to be created: ${FMU_FILENAME}")

set(TESTING_ENABLED_OS "win64" "linux64")

# -------------------------------------------------
# Developer notes
# -------------------------------------------------
# The ${FMU_MODEL_IDENTIFIER} target generates the main shared library of the FMU.
# Among the various functions exposed by the library, there is one dedicated to the 'modelDescription.xml' generation.
# This function has to called by a second target (fmi_modeldescription) that instantiates the FmuComponent class of the library
# and makes it generate the 'modelDescription.xml' file.
# This should happen after the main target is built, but before the FMU is zipped.
#
# The testing relies on the fmuChecker tool: an additional target is calling it as pre-build step,
# while during the actual execution of the test, the _output_ of the fmuChecker is parsed to check the FMU compliance.


# -------------------------------------------------
# Set Architecture and Operating System Flags
# -------------------------------------------------

set(UNKNOWN_TARGET_SYSTEM FALSE)
if(WIN32)
    set(FMI_PLATFORM "win")
elseif(APPLE)
    set(FMI_PLATFORM "darwin")
elseif(CYGWIN)
    set(FMI_PLATFORM "cygwin")
elseif(UNIX)
    set(FMI_PLATFORM "linux")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(FMI_PLATFORM "${FMI_PLATFORM}64")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(FMI_PLATFORM "${FMI_PLATFORM}32")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(UNKNOWN_TARGET_SYSTEM)
    message(WARNING "Cannot retrieve target operating system or architecture. Please specify it manually through TARGET_SYSTEM variable (e.g. win64)")
    set(TARGET_SYSTEM "win64" CACHE STRING "Target operating system and architecture (e.g. win64, linux32, ...)")
    mark_as_advanced(CLEAR TARGET_SYSTEM)
else()
    message(STATUS "${FMU_MSG_PREFIX}Architecture: ${FMI_PLATFORM}")
    mark_as_advanced(FORCE TARGET_SYSTEM)
endif()

set(FMU_RUNTIME_OUTPUT_DIRECTORY "${FMU_DIRECTORY}/binaries/${FMI_PLATFORM}/$<0:>")

# -------------------------------------------------
# Generated GUID
# -------------------------------------------------

set(UUID_ALPHABET "0123456789abcdef")
string(RANDOM LENGTH 8  ALPHABET ${UUID_ALPHABET} FMU_NAMESPACE1)
string(RANDOM LENGTH 4  ALPHABET ${UUID_ALPHABET} FMU_NAMESPACE2)
string(RANDOM LENGTH 4  ALPHABET ${UUID_ALPHABET} FMU_NAMESPACE3)
string(RANDOM LENGTH 4  ALPHABET ${UUID_ALPHABET} FMU_NAMESPACE4)
string(RANDOM LENGTH 12 ALPHABET ${UUID_ALPHABET} FMU_NAMESPACE5)
set(FMU_GUID_NAMESPACE_RANDOM "${FMU_NAMESPACE1}-${FMU_NAMESPACE2}-${FMU_NAMESPACE3}-${FMU_NAMESPACE4}-${FMU_NAMESPACE5}")

set(FMU_GUID_NAMESPACE ${FMU_GUID_NAMESPACE_RANDOM} CACHE STRING "FMU Namespace used to generated FMU GUID.")
message(STATUS "${FMU_MSG_PREFIX}FMU_GUID_NAMESPACE: ${FMU_GUID_NAMESPACE}")
string(UUID FMU_GUID NAMESPACE ${FMU_GUID_NAMESPACE} NAME ${FMU_MODEL_IDENTIFIER} TYPE SHA1)
message(STATUS "${FMU_MSG_PREFIX}FMU_GUID: ${FMU_GUID}")

# -------------------------------------
# Target to compile the FMU source code
# -------------------------------------

set(FMU_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/FmuToolsCommon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/FmuToolsExport.h
    ${CMAKE_CURRENT_SOURCE_DIR}/FmuToolsExport.cpp)

if(USE_CUSTOM_TYPESPLATFORM)
    message(STATUS "${FMU_MSG_PREFIX}Using custom 'fmi2TypesPlatform'.")
    target_compile_definitions(${FMU_MODEL_IDENTIFIER} PUBLIC FMITYPESPLATFORM_CUSTOM)
    if (NOT EXISTS "${CMAKE_SOURCE_DIR}/TypesVariantsCustom.h")
        file(COPY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/TypesVariantsTemplate.h" "${CMAKE_SOURCE_DIR}/TypesVariantsCustom.h")
        message(STATUS "${FMU_MSG_PREFIX}Created template 'TypesVariantsCustom.h'. Please make your changes there.")
    endif()
    if (NOT EXISTS "${CMAKE_SOURCE_DIR}/fmi2TypesPlatformCustom.h")
        file(COPY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/fmi2_headers/fmi2TypesPlatform.h" "${CMAKE_SOURCE_DIR}/fmi2TypesPlatformCustom.h")
        message(STATUS "${FMU_MSG_PREFIX}Created template 'fmi2TypesPlatformCustom.h'. Please make your changes there.")
    endif()
    list(APPEND FMU_SOURCES ${CMAKE_SOURCE_DIR}/TypesVariantsCustom.h)
else()
    message(STATUS "${FMU_MSG_PREFIX}Using default fmi2TypesPlatform.")
    list(APPEND FMU_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/TypesVariantsDefault.h)
endif()

source_group("fmu_tools" FILES ${FMU_SOURCES})

# *************************************************************************************************
# Caller must add custom sources to the ${FMU_MODEL_IDENTIFIER} target with a call as follows:
#    'target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE <yoursourcefiles>)'
# *************************************************************************************************

message(STATUS "${FMU_MSG_PREFIX}Set up FMU creation")
message(STATUS "${FMU_MSG_PREFIX}   adding target '${FMU_MODEL_IDENTIFIER}'")
message(STATUS "${FMU_MSG_PREFIX}   setting runtime output dir to '${FMU_RUNTIME_OUTPUT_DIRECTORY}'")
message(STATUS "${FMU_MSG_PREFIX}   append FMU sources to '${FMU_MODEL_IDENTIFIER}' target through 'target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE <your_source_files>)'")


add_library(${FMU_MODEL_IDENTIFIER} SHARED ${FMU_SOURCES})

target_include_directories(${FMU_MODEL_IDENTIFIER} PRIVATE "${CMAKE_SOURCES_DIR}")
target_compile_definitions(${FMU_MODEL_IDENTIFIER} PUBLIC FMU_MODEL_IDENTIFIER="${FMU_MODEL_IDENTIFIER}")
target_compile_definitions(${FMU_MODEL_IDENTIFIER} PRIVATE FMU_GUID="${FMU_GUID}")

# Explicitly set the prefix on the generated FMU shared library to be empty on all platforms
# (as per the FMI specifications)

set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES RUNTIME_OUTPUT_NAME ${FMU_MODEL_IDENTIFIER})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES PREFIX "")

set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${FMU_RUNTIME_OUTPUT_DIRECTORY})

# ----------------------------------------------------
# Target to generate the FMU modelDescription XML file
# ----------------------------------------------------

if (NOT TARGET fmi_modeldescription)
    set(MODELDESCRIPTION_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../FmuToolsRuntimeLinking.h 
        ${CMAKE_CURRENT_SOURCE_DIR}/../FmuToolsModelDescription.cpp)
    source_group("" FILES ${MODELDESCRIPTION_SOURCES})
    
    set(fmi_modeldescription "fmi_modeldescription")
    add_executable(fmi_modeldescription ${MODELDESCRIPTION_SOURCES})

    message(STATUS "${FMU_MSG_PREFIX}Set up FMU generation of model description")
    message(STATUS "${FMU_MSG_PREFIX}   adding target '${fmi_modeldescription}'")

    target_link_libraries(fmi_modeldescription PRIVATE ${CMAKE_DL_LIBS})
    set_target_properties(fmi_modeldescription PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
    set_target_properties(fmi_modeldescription PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})
endif()

add_dependencies(${FMU_MODEL_IDENTIFIER} fmi_modeldescription)

# ---------------------------------------------
# Add custom commands to:
# - Copy external dependencies (if any)
# - Generate the FMU model description
# - Add the FMU resources directory (if any)
# - Create the FMU archive
# ---------------------------------------------

if (DEFINED FMU_DEPENDENCIES AND NOT FMU_DEPENDENCIES STREQUAL "")
    message(STATUS "${FMU_MSG_PREFIX}Found dependencies: ${FMU_DEPENDENCIES}")
    add_custom_command(
        TARGET ${FMU_MODEL_IDENTIFIER} PRE_BUILD
        COMMENT "\nCopy FMU dependencies '${FMU_DEPENDENCIES}' into '${FMU_RUNTIME_OUTPUT_DIRECTORY}'"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FMU_DEPENDENCIES} ${FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
else()
    message(STATUS "${FMU_MSG_PREFIX}No dependencies specified.")    
endif()

add_custom_command(TARGET ${FMU_MODEL_IDENTIFIER} POST_BUILD
    COMMENT "\nGenerating modelDescription.xml for '${FMU_RUNTIME_OUTPUT_DIRECTORY}/${FMU_MODEL_IDENTIFIER}${CMAKE_SHARED_LIBRARY_SUFFIX}' into '${FMU_DIRECTORY}'"
    COMMAND $<TARGET_FILE:fmi_modeldescription> ${FMU_RUNTIME_OUTPUT_DIRECTORY} ${FMU_MODEL_IDENTIFIER}${CMAKE_SHARED_LIBRARY_SUFFIX} ${FMU_DIRECTORY}
    BYPRODUCTS "${FMU_DIRECTORY}/modelDescription.xml"
    WORKING_DIRECTORY ${FMU_DIRECTORY}
)

if (DEFINED FMU_RESOURCES_DIRECTORY AND NOT FMU_RESOURCES_DIRECTORY STREQUAL "")
    if(EXISTS ${FMU_RESOURCES_DIRECTORY})
        message(STATUS "${FMU_MSG_PREFIX}Found resources folder: ${FMU_RESOURCES_DIRECTORY}")

        set(FMU_RESOURCES_DIR_OUT "${FMU_DIRECTORY}/resources")

        add_custom_command(TARGET ${FMU_MODEL_IDENTIFIER} POST_BUILD
          COMMENT "\nCopying ${FMU_RESOURCES_DIRECTORY} to ${FMU_RESOURCES_DIR_OUT}"
          COMMAND ${CMAKE_COMMAND} -E copy_directory "${FMU_RESOURCES_DIRECTORY}" "${FMU_RESOURCES_DIR_OUT}"
          BYPRODUCTS "${FMU_RESOURCES_DIR_OUT}"
          WORKING_DIRECTORY ${FMU_DIRECTORY}
        )
    else()
        message(STATUS "${FMU_MSG_PREFIX}Resources directory ${FMU_RESOURCES_DIRECTORY} not found.")
        set(FMU_RESOURCES_DIR_OUT "")
    endif()
else()
    message(STATUS "${FMU_MSG_PREFIX}No resources directory specified.")
    set(FMU_RESOURCES_DIR_OUT "")
endif()

add_custom_command(TARGET ${FMU_MODEL_IDENTIFIER} POST_BUILD
    COMMENT "\nCreating FMU file: ${FMU_MODEL_IDENTIFIER}.fmu"
    COMMAND ${CMAKE_COMMAND} -E tar "cf" "${FMU_DIRECTORY}/${FMU_MODEL_IDENTIFIER}.fmu" "${FMU_DIRECTORY}/modelDescription.xml" "${FMU_RUNTIME_OUTPUT_DIRECTORY}" "${FMU_RESOURCES_DIR_OUT}" "--format=zip"
    BYPRODUCTS "${FMU_DIRECTORY}/${FMU_MODEL_IDENTIFIER}.fmu"
    WORKING_DIRECTORY ${FMU_DIRECTORY}
)

# --------------------------------------------------------
# Target to run fmuCheck on the FMU and test result output 
# --------------------------------------------------------

if (FMU_TESTING)
    if(${FMI_PLATFORM} IN_LIST TESTING_ENABLED_OS)
        ## Call fmuChecker on the FMU, parse its output
        set(fmi_testchecker fmi_testchecker_${FMU_MODEL_IDENTIFIER})
        set(build_fmi_testchecker build_test_parseFmuChecker_${FMU_MODEL_IDENTIFIER})

        message(STATUS "${FMU_MSG_PREFIX}Testing enabled.")
        message(STATUS "${FMU_MSG_PREFIX}   adding target '${fmi_testchecker}'")

        set(FMUCHECKER_LOG "${CMAKE_BINARY_DIR}/${FMU_MODEL_IDENTIFIER}_log.txt")
        set(FMUCHECKER_RESULTS "${CMAKE_BINARY_DIR}/${FMU_MODEL_IDENTIFIER}_results.txt")

        set(TESTING_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/../FmuToolsTestCheckerOutput.cpp)
        source_group("" FILES ${TESTING_SOURCES})

        add_executable(${fmi_testchecker} ${TESTING_SOURCES})
        target_compile_definitions(${fmi_testchecker} PRIVATE FMUCHECKER_LOG="${FMUCHECKER_LOG}")
        add_dependencies(${fmi_testchecker} ${FMU_MODEL_IDENTIFIER})

        ## call the execution of fmuChecker
        add_custom_command(TARGET ${fmi_testchecker}
                        PRE_BUILD
                        COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/fmuChecker/fmuCheck.${FMI_PLATFORM}${CMAKE_EXECUTABLE_SUFFIX}" -e \"${FMUCHECKER_LOG}\" -o \"${FMUCHECKER_RESULTS}\" -k cs -s 1 -h 1e-3 \"${FMU_FILENAME}\"
                        COMMENT "\nTesting FMU with fmuChecker")

        if(PROJECT_IS_TOP_LEVEL)
            ## enable the testing
            enable_testing()
        endif()

        ## the 'fmuChecker_requirements' fixtures is registered
        ## 'build_fmi_testchecker' is set as a requirement of type SETUP i.e. called before the FIXTURES_REQUIRED target
        ## 'TestFmuComponent_${FMU_MODEL_IDENTIFIER}' is the one requiring the other targets
        add_test(NAME TestFmuComponent_${FMU_MODEL_IDENTIFIER}
                 COMMAND ${fmi_testchecker})

        add_test(${build_fmi_testchecker}
            "${CMAKE_COMMAND}"
            --build "${CMAKE_BINARY_DIR}"
            # --target ${fmi_testchecker
        )

        set_tests_properties(${build_fmi_testchecker} PROPERTIES FIXTURES_SETUP fmuChecker_requirements_${FMU_MODEL_IDENTIFIER})
        set_tests_properties(TestFmuComponent_${FMU_MODEL_IDENTIFIER} PROPERTIES FIXTURES_REQUIRED fmuChecker_requirements_${FMU_MODEL_IDENTIFIER})
    else()
        message(STATUS "${FMU_MSG_PREFIX}Testing not enabled. Available only for: ${TESTING_ENABLED_OS}")
    endif()
else()
    message(STATUS "${FMU_MSG_PREFIX}Testing disabled")
endif(FMU_TESTING)

# -------------------------------------------------
# Propagate variable to upper levels
# Make variables set in this script available to the fetcher projects
# -------------------------------------------------

set(FMI_PLATFORM ${FMI_PLATFORM} PARENT_SCOPE)
set(FMU_DIRECTORY ${FMU_DIRECTORY} PARENT_SCOPE)
set(FMU_FILENAME ${FMU_FILENAME} PARENT_SCOPE)
set(FMU_RUNTIME_OUTPUT_DIRECTORY "${FMU_RUNTIME_OUTPUT_DIRECTORY}" PARENT_SCOPE)
set(FMU_SOURCES ${FMU_SOURCES} PARENT_SCOPE)
