if(CONFIG_CHERRYECAT)

    list(APPEND cherryec_incs
        ${CMAKE_CURRENT_LIST_DIR}/include
    )

    list(APPEND cherryec_srcs
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_cmd.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_coe.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_common.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_datagram.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_eoe.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_foe.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_mailbox.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_master.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_netdev.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_perf.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_sii.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_slave.c
        ${CMAKE_CURRENT_LIST_DIR}/src/ec_timestamp.c
        ${CMAKE_CURRENT_LIST_DIR}/src/phy/chry_phy.c
    )

    if(DEFINED CONFIG_CHERRYECAT_OSAL)
        if("${CONFIG_CHERRYECAT_OSAL}" STREQUAL "freertos")
            list(APPEND cherryec_srcs ${CMAKE_CURRENT_LIST_DIR}/osal/ec_osal_freertos.c)
        elseif("${CONFIG_CHERRYECAT_OSAL}" STREQUAL "rtthread")
            list(APPEND cherryec_srcs ${CMAKE_CURRENT_LIST_DIR}/osal/ec_osal_rtthread.c)
        elseif("${CONFIG_CHERRYECAT_OSAL}" STREQUAL "threadx")
            list(APPEND cherryec_srcs ${CMAKE_CURRENT_LIST_DIR}/osal/ec_osal_threadx.c)
        endif()
    endif()

    if(HPM_SDK_BASE)
        list(APPEND cherryec_srcs port/netdev_hpmicro.c)
        sdk_inc(${cherryec_incs})
        sdk_src(${cherryec_srcs})
    endif()

endif()