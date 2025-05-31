
macro(GitVersion)

    ########################### GIT Versions ########################### 

    # Get realpath (resolve symlinks if need be)
    if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.19.0")
        file(REAL_PATH ${CMAKE_CURRENT_SOURCE_DIR} RP_CURRENT_SOURCE_DIR)
    else ()
        #exec_program("realpath"
        #             ARGS "${CMAKE_CURRENT_SOURCE_DIR}"
        #             OUTPUT_VARIABLE RP_CURRENT_SOURCE_DIR)

        execute_process(
                COMMAND realpath "${CMAKE_CURRENT_SOURCE_DIR}"
                OUTPUT_VARIABLE RP_CURRENT_SOURCE_DIR
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        message("${RP_CURRENT_SOURCE_DIR}")
    endif ()

    # git version with tag [tag-ac-g#######(-dirty)]
    #   ac = additional commits (e.g. 0)
    #   ####### = uniquely abbreviated commit (e.g. 7fa1685)
    # exec_program("git"
    #              ${RP_CURRENT_SOURCE_DIR}
    #              ARGS "describe --tags --long --always --dirty"
    #              OUTPUT_VARIABLE GIT_TAG
    #              RETURN_VALUE RV_GIT_TAG)

    execute_process(
            COMMAND git describe --tags --long --always --dirty
            WORKING_DIRECTORY ${RP_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_TAG
            RESULT_VARIABLE RV_GIT_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(STRIP ${GIT_TAG} GIT_TAG)

    # git version without tag [#######(-dirty)]
    #   note: does not contain the g prefix
    # exec_program("git"
    #              ${RP_CURRENT_SOURCE_DIR}
    #              ARGS "describe --long --always --dirty --exclude '*'"
    #              OUTPUT_VARIABLE GIT_VER
    #              RETURN_VALUE RV_GIT_VER)

    execute_process(
            COMMAND git describe --long --always --dirty --exclude "*"
            WORKING_DIRECTORY ${RP_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_VER
            RESULT_VARIABLE RV_GIT_VER
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(STRIP ${GIT_VER} GIT_VER)

    # to use when nothing found (UK = unknown)
    set(UK_TAG "0.0.1") # tag
    set(UK_AC "unknown") # additional commits
    set(UK_UAC "0000000") # uniquely abbreviated commit
    if (${RV_GIT_TAG} MATCHES 0)
        if (${RV_GIT_VER} MATCHES 0)
            # both (with and without tags) commands succeeded
            if (GIT_TAG STREQUAL GIT_VER)
                # with and without tag are the same: no tag present
                set(GIT_VERSION ${UK_TAG}-${UK_AC}-g${GIT_VER})
            else ()
                # with and without tag are not the same: tag present
                set(GIT_VERSION ${GIT_TAG})
            endif ()
        else ()
            # only with tags command succeeded
            set(GIT_VERSION ${GIT_TAG})
        endif ()
    elseif (${RV_GIT_VER} MATCHES 0)
        # only without tags command succeeded
        set(GIT_VERSION ${UK_TAG}-${UK_AC}-g${GIT_VER})
    else ()
        # both (with and without tags) commands failed
        set(GIT_VERSION ${UK_TAG}-${UK_AC}-g${UK_UAC})
    endif ()

    set(MCX_VERSION ${GIT_VERSION}) # our version = git version

    ############ Last known previous commit version (LKPCV) ############

    # get git toplevel directory
    # exec_program("git"
    #              ${RP_CURRENT_SOURCE_DIR}
    #              ARGS "rev-parse --show-toplevel"
    #              OUTPUT_VARIABLE GIT_TOP_LEVEL_DIR
    #              RETURN_VALUE RV_GIT_TLD)

    execute_process(
            COMMAND git rev-parse --show-toplevel
            WORKING_DIRECTORY ${RP_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_TOP_LEVEL_DIR
            RESULT_VARIABLE RV_GIT_TLD
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    #message("GIT_TOP_LEVEL_DIR: " ${GIT_TOP_LEVEL_DIR})
    #message("RP_CURRENT_SOURCE_DIR: " ${RP_CURRENT_SOURCE_DIR})

    # check if source dir is the toplevel git
    if (${RV_GIT_TLD} MATCHES 0)
        string(STRIP ${GIT_TOP_LEVEL_DIR} GIT_TOP_LEVEL_DIR)

        # if this current source directory is not the same as the
        # git top level directory, we must be a subtree
        if (NOT GIT_TOP_LEVEL_DIR STREQUAL RP_CURRENT_SOURCE_DIR)
            # Get current base project name from url
            # exec_program("git"
            #              ${GIT_TOP_LEVEL_DIR}
            #              ARGS "remote get-url origin"
            #              OUTPUT_VARIABLE GIT_URL
            #              RETURN_VALUE RV_GIT_URL)

            execute_process(
                    COMMAND git remote get-url origin
                    WORKING_DIRECTORY ${GIT_TOP_LEVEL_DIR}
                    OUTPUT_VARIABLE GIT_URL
                    RESULT_VARIABLE RV_GIT_URL
                    OUTPUT_STRIP_TRAILING_WHITESPACE
            )

            # STO = subtree of
            if (${RV_GIT_URL} MATCHES 0)
                get_filename_component(STO ${GIT_URL} NAME_WE)
                string(APPEND STO "_")
            endif ()

            # check if the .mcx-lkpcv file exists, if it does we might
            # have the last known previous commit version (=LKPCV)
            if (EXISTS ${RP_CURRENT_SOURCE_DIR}/.mcx-lkpcv)
                file(READ ${RP_CURRENT_SOURCE_DIR}/.mcx-lkpcv MCX_LKPCV)
                string(STRIP ${MCX_LKPCV} MCX_LKPCV)

                # if the loaded string is empty lkpcv is also unknown
                string(LENGTH ${MCX_LKPCV} MCX_LKPCV_LEN)
                if (${MCX_LKPCV_LEN} LESS 1)
                    set(MCX_LKPCV ${UK_TAG}-${UK_AC}-g${UK_UAC})
                else ()
                    string(REGEX MATCHALL "[^-]+" MCX_LKPCV_SPLIT ${MCX_LKPCV})
                    list(LENGTH MCX_LKPCV_SPLIT MCX_LKPCV_SPLIT_LEN)
                    if (${MCX_LKPCV_SPLIT_LEN} LESS_EQUAL 2)
                        # We have the - character zero or one times so
                        # we don't have a tag, but do have the uac hash
                        set(MCX_LKPCV ${UK_TAG}-${UK_AC}-g${MCX_LKPCV})
                    endif ()
                endif ()
            else ()
                # lkpcv is also unknown
                set(MCX_LKPCV ${UK_TAG}-${UK_AC}-g${UK_UAC})
            endif ()

            set(MCX_VERSION ${MCX_LKPCV}) # our version = lkpcv version

            set(MCX_LKPCV_PREFIX "lkpcv::${MCX_VERSION}::[sto::${STO}")
            set(MCX_LKPCV_SUFFIX "]")
        else ()
            # not a subtree, so lets copy the pre-commit script and add it to git
            set(GIT_HOOKS_DIR .githooks)
            set(PC_FILENAME pre-commit)
            set(PC_CONTENTS "#!/bin/sh"
                    "#"
                    "# pre-commit hook to save the \"last known previous commit version\" into"
                    "# the git repo itself, so when we use subtrees atleast we have the"
                    "# \"last known previous commit version\" (lkpcv) of the sub-treed project"
                    " "
                    "# no --dirty, because it is last know previous commit version:"
                    "lkpcv=$(git describe --tags --long --always)"
                    "if [ $? -ne 0 ]"
                    "then"
                    "    exit -1 # failed"
                    "fi"
                    " "
                    "echo $lkpcv > .mcx-lkpcv"
                    "git add .mcx-lkpcv"
                    " "
                    "# keep our .mcx-lkpcv file when merging"
                    "git config --local merge.mcx_ours.driver true"
                    "grep -qxF '.mcx-lkpcv merge=mcx_ours' .gitattributes || echo '.mcx-lkpcv merge=mcx_ours' >> .gitattributes"
                    "git add .gitattributes"
                    " "
                    "echo \"\\nUpdated .mcx-lkpcv to $lkpcv\\n\""
                    "exit 0"
            )

            # make directory if needed
            set(GH_FULL_DIR ${RP_CURRENT_SOURCE_DIR}/${GIT_HOOKS_DIR})
            if (NOT EXISTS ${GH_FULL_DIR})
                file(MAKE_DIRECTORY ${GH_FULL_DIR})
            endif ()

            # write PC_CONTENTS to a temp file
            set(PC_TEMP_NAME ${CMAKE_CURRENT_BINARY_DIR}/${PC_FILENAME})
            file(WRITE ${PC_TEMP_NAME})
            foreach (line ${PC_CONTENTS})
                string(STRIP ${line} line)
                file(APPEND ${PC_TEMP_NAME} ${line} "\n")
                #message("${line}")
            endforeach (line)

            # check if the installed is different and give warning when it is
            set(PC_TARGET_NAME ${GH_FULL_DIR}/${PC_FILENAME})
            execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${PC_TEMP_NAME} ${PC_TARGET_NAME}
                    RESULT_VARIABLE compare_result
            )
            if (compare_result EQUAL 1)
                if (EXISTS ${PC_TARGET_NAME})
                    message(WARNING "Overwritting ${PC_FILENAME} in ${GH_FULL_DIR}")
                endif ()
            elseif (NOT (compare_result EQUAL 0))
                # not (0 or 1), we have an error whilst comparing
                message("Error while comparing ${PC_TEMP_NAME} with ${PC_TARGET_NAME}")
            endif ()

            # install pre-commit hook
            if (NOT (compare_result EQUAL 0))
                # not 0, so also install when compare error
                file(INSTALL ${PC_TEMP_NAME}
                        DESTINATION ${GH_FULL_DIR}
                        FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                        GROUP_READ GROUP_EXECUTE
                        WORLD_READ WORLD_EXECUTE)
            endif ()

            # add pre-commit hook to repo
            # exec_program("git"
            #              ${RP_CURRENT_SOURCE_DIR}
            #              ARGS "add ${GIT_HOOKS_DIR}/${PC_FILENAME}"
            #              OUTPUT_VARIABLE DUMMY
            #              RETURN_VALUE RV_GIT_ADD)

            execute_process(
                    COMMAND git add ${GIT_HOOKS_DIR}/${PC_FILENAME}
                    WORKING_DIRECTORY ${RP_CURRENT_SOURCE_DIR}
                    OUTPUT_VARIABLE DUMMY
                    RESULT_VARIABLE RV_GIT_ADD
            )

            # point hooksPath of local repo to .githooks dir
            # exec_program("git"
            #              ${RP_CURRENT_SOURCE_DIR}
            #              ARGS "config core.hooksPath ${GIT_HOOKS_DIR}"
            #              OUTPUT_VARIABLE DUMMY
            #              RETURN_VALUE RV_GIT_HD)

            execute_process(
                    COMMAND git config core.hooksPath ${GIT_HOOKS_DIR}
                    WORKING_DIRECTORY ${RP_CURRENT_SOURCE_DIR}
                    OUTPUT_VARIABLE DUMMY
                    RESULT_VARIABLE RV_GIT_HD
            )

        endif ()
    endif ()

    ####################### Motorcortex Versions #######################

    # split version into major minor patch and
    string(REGEX MATCH "^[a-zA-Z0-9\\.]+" MCX_VERSION_LITE ${MCX_VERSION})
    string(REGEX MATCHALL "[a-zA-Z0-9]+" MCX_VERSIONS ${MCX_VERSION_LITE})
    list(LENGTH MCX_VERSIONS MCX_VERSIONS_LEN)
    if (${MCX_VERSIONS_LEN} GREATER 2)
        list(GET MCX_VERSIONS 0 MCX_VERSION_MAJOR)
        list(GET MCX_VERSIONS 1 MCX_VERSION_MINOR)
        list(GET MCX_VERSIONS 2 MCX_VERSION_PATCH)
    else ()
        message(WARNING "Failed to get project version.")
    endif ()

    # when not sing lkcpv, prefix and suffix are empty
    set(MCX_VERSION_FULL "${MCX_LKPCV_PREFIX}${GIT_VERSION}${MCX_LKPCV_SUFFIX}")


endmacro()
