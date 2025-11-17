# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(HPM_SDK_LD_NO_NANO_SPECS 1)

set(CONFIG_FREERTOS 1)
# set(CONFIG_CHERRYRB 1)
set(CONFIG_CHERRYSH 1)
set(CONFIG_CHERRYSH_INTERFACE "uart")

set(CONFIG_ENET_PHY 1)
set(APP_USE_ENET_PORT_COUNT 1)
#set(APP_USE_ENET_ITF_RGMII 1)
#set(APP_USE_ENET_ITF_RMII 1)
#set(APP_USE_ENET_PHY_DP83867 1)
#set(APP_USE_ENET_PHY_RTL8211 1)
#set(APP_USE_ENET_PHY_DP83848 1)
set(APP_USE_ENET_PHY_RTL8201 1)

set(CONFIG_CHERRYECAT 1)
set(CONFIG_CHERRYECAT_OSAL "freertos")

# if(NOT (HPM_BUILD_TYPE STREQUAL "ram"))
# message(FATAL_ERROR "Only support ram build for demo")
# endif()

#Set CONFIG_FREERTOS_TIMER_RESOURCE_GPTMR to use GPTMR as system's tick source
#set(CONFIG_FREERTOS_TIMER_RESOURCE_GPTMR 1)

# set(CONFIG_LWIP 1)
# set(CONFIG_LWIP_STRERR 1)
# set(CONFIG_LWIP_NETDB 1)
# set(CONFIG_LWIP_SOCKET_API 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(cherryecat)

sdk_compile_definitions(-D__freertos_irq_stack_top=_stack)
sdk_compile_definitions(-DCONFIG_FREERTOS=1)
sdk_compile_definitions(-DUSE_NONVECTOR_MODE=1)
sdk_compile_definitions(-DDISABLE_IRQ_PREEMPTIVE=1)

sdk_compile_options("-O2")

sdk_inc(.)
sdk_inc(inc)
sdk_app_src(main.c)
# sdk_inc(iperf)
# sdk_app_src(arch/sys_arch.c tcp_client.c utils_getopt.c ping.c iperf/iperf.c iperf/iperf_cli.c)

add_subdirectory(../.. cherryecat)
generate_ses_project()

