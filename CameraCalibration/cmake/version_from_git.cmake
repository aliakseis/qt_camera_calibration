function(version_from_git)
    message("Getting project version from Git repository...")

    find_package(Git)
    if (GIT_FOUND)
        message("Git executable is ${GIT_EXECUTABLE}")
    else()
        message(FATAL_ERROR "Git not found")
    endif()

    set(PROJECT_GIT_REPOSITORY "${CMAKE_CURRENT_SOURCE_DIR}/../.git")
    if(IS_DIRECTORY ${PROJECT_GIT_REPOSITORY})
        message("Project git repository found ${PROJECT_GIT_REPOSITORY}")
    else()
        message(FATAL_ERROR "Project git repository not found")
    endif()

    # Setting command that gets version from git repository

    set(GIT_COMMAND ${GIT_EXECUTABLE} "describe" "--long")
    execute_process(COMMAND ${GIT_COMMAND}
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                    OUTPUT_VARIABLE PROJECT_VERSION_GIT_FULL
                    RESULT_VARIABLE GIT_RESULT)

    if(${GIT_RESULT} STREQUAL "0")

        string(REGEX REPLACE "[ \t\r\n]" "" PROJECT_VERSION_GIT_FULL ${PROJECT_VERSION_GIT_FULL})
        string(REPLACE "v" "" PROJECT_VERSION_GIT_FULL ${PROJECT_VERSION_GIT_FULL})

        string(REGEX MATCH "^([0-9]+)\." MATCH_RESULT ${PROJECT_VERSION_GIT_FULL})
        set(PROJECT_VERSION_GIT_MAJOR ${CMAKE_MATCH_1})

        string(REGEX MATCH "\.([0-9]+)\-" MATCH_RESULT ${PROJECT_VERSION_GIT_FULL})
        set(PROJECT_VERSION_GIT_MINOR ${CMAKE_MATCH_1})

        string(REGEX MATCH "\-([0-9]+)\-" MATCH_RESULT ${PROJECT_VERSION_GIT_FULL})
        set(PROJECT_VERSION_GIT_PATCH ${CMAKE_MATCH_1})

        if("${PROJECT_VERSION_GIT_MAJOR}" STREQUAL "" OR
           "${PROJECT_VERSION_GIT_MINOR}" STREQUAL "" OR
           "${PROJECT_VERSION_GIT_PATCH}" STREQUAL "")
            message(FATAL_ERROR "Error extracting verson details")
        endif()

        message("Version string received from git is ${PROJECT_VERSION_GIT_FULL}")

    else()

        set(GIT_COMMAND ${GIT_EXECUTABLE} "rev-parse" "--short" "HEAD")
        execute_process(COMMAND ${GIT_COMMAND}
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                        OUTPUT_VARIABLE PROJECT_VERSION_GIT_FULL
                        RESULT_VARIABLE GIT_RESULT)

        if(${GIT_RESULT} STREQUAL "0")
            string(REGEX REPLACE "[ \t\r\n]" "" PROJECT_VERSION_GIT_FULL ${PROJECT_VERSION_GIT_FULL})
            message("Version string received from git is ${PROJECT_VERSION_GIT_FULL}")
        else()
            set(PROJECT_VERSION_GIT_FULL "git-version-is-empty" PARENT_SCOPE)
            message("Version string received from git is empty")
        endif()

        set(PROJECT_VERSION_GIT_MAJOR "0")
        set(PROJECT_VERSION_GIT_MINOR "0")
        set(PROJECT_VERSION_GIT_PATCH "0")

    endif()

    set(PROJECT_VERSION_GIT_FULL ${PROJECT_VERSION_GIT_FULL} PARENT_SCOPE)
    set(PROJECT_VERSION_GIT_MAJOR ${PROJECT_VERSION_GIT_MAJOR} PARENT_SCOPE)
    set(PROJECT_VERSION_GIT_MINOR ${PROJECT_VERSION_GIT_MINOR} PARENT_SCOPE)
    set(PROJECT_VERSION_GIT_PATCH ${PROJECT_VERSION_GIT_PATCH} PARENT_SCOPE)

endfunction()
