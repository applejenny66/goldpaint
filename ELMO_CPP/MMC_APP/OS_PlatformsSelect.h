////////////////////////////////////////////////////////////////////////////////
/// Name   		: OS_PlatformsSelect.h
/// Author      : Haim Hillel
/// Created on  : 29Jan2013
/// Version     : 0.0.1,
///               0.2.0 Updated 20Jan2015 Haim H. native Data Types (names), for supporting 64B OS.
///               0.3.0 Updated 03Aug2015 Haim H. support Littlendian & Bigendian GMAs.
///				  0.4.0 Updated 11Sep2017 Haim H.
/// Copyright   : Your copyright notice
/// Description : This file contain GMAS global definitions for:
///               Selecting target OS (OS Platform) as well as Cpu Endian type (CPU Platform).
///                   The file contain 3 parts, user Should EDIT ONLY TWO SYMBOLS IN 'PART 2':
///               Selecting/set CPU type according to:
///                 IPC: The GMAS (proccessing user commands) and the CPU running user program
///                                 are same => same Endian => no needs consider CPUs Endian.
///                 RPC: the GMAS (proccessing user commands) and the CPU running user program
///                                 are differents CPU => same or different Endian...
///                         Decision for Different or Same Endian Cpu is automatic (no user action)
///                         and done ON RUN TIME.
///                         ===========================================
///                         Edit PART 2 for:
///                         1. Set for Select CPU Endian => set one of:
///                                     RPC_CPU
///                                     IPC_CPU
///                         2. Select OS => set OS_PLATFORM to one of:
///                                     LINUXIPC32_PLATFORM  
///                                     WIN32_PLATFORM     
///                                     WIN64_PLATFORM     
///                                     LINUXRPC32_PLATFORM
///                                     LINUXRPC64_PLATFORM
///                                     VXWORKS32_PLATFORM   
///                         ===========================================
///                         E.g1: 
///                            /* Select for GMAS CPU ENDIAN set automatic on run time (RPC)*/
///                            /* Select target OS to Windows 32 bit                        */
///                         #define ENDIAN_SELECT   RPC_ENDIAN_AUTO_SET
///                         #define OS_PLATFORM     WIN32_PLATFORM
/// 
///                         E.g2:
///                            /* Select GMAS is IPC - transparent Endian   */
///                            /* Select target OS to Linux IPC             */
///                         #define ENDIAN_SELECT   IPC_CPU
///                         #define OS_PLATFORM     LINUXIPC32_PLATFORM
////////////////////////////////////////////////////////////////////////////////
#ifndef OS_PLATFORMSSELECT_H_
#define OS_PLATFORMSSELECT_H_
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */


/* --------------------------------   PART 1: Endian & Platform definitions  ---------------------------------------------- */
                                    /*---------------------*\
                                    |* SHOULD NOT BE EDIT  *|
                                    \*---------------------*/
        /* CPU ENDIAN TYPE: */
                                    /* The targer Endian define in run time (same or diff from host)*/
#define RPC_ENDIAN_AUTO_SET 123
                                    /* TRANSPARENT Endian, user program and GMAS are on same CPU so */
                                    /* they are SAME ENDIAN - ignore Endian type.                   */
#define IPC_ENDIAN_IS_TRNSP 999


        /* SUPPORTINT OS PLATFORM: */
#define LINUXIPC32_PLATFORM 333
#define WIN32_PLATFORM      444
#define WIN64_PLATFORM      555
#define LINUXRPC32_PLATFORM 666
#define LINUXRPC64_PLATFORM 777
#define VXWORKS32_PLATFORM  888
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */


/* --------------------------------   PART 2: OS Platform Select    ------------------------------------------------------- */
                        /*--------------------------------------------------*\
                        |* Two below symbols (ENDIAN_SELECT & OS_PLATFORM)  *|
                        |* SHOULD BE SETTING (DEFINE) ACCORDING USING ENVI.*|
                        |* Differents possibility exist for setting (E.g.): *|
                        |*  1. Edit and assign in this file (below),        *|
                        |*  2. Setting in compiler setup (Visual & Eclips), *|
                        |*  3. Setting in builds make file (VxWorsk).       *|
                        \*--------------------------------------------------*/
// Below setting (define) moved (can be move) to COMPILER SETUP (on Eclipse See above E.g. #2)
//     On Eclipse, <project> / Settings / Tool Settings / Symbols:
//			ENDIAN_SELECT=123
//          OS_PLATFORM=77
//  Do not delete below setting (even they appearing as comments):
// ================================================================================================================
//                            /*  target CPU Endian   */
#define ENDIAN_SELECT   RPC_ENDIAN_AUTO_SET
// or:
//#define ENDIAN_SELECT   IPC_ENDIAN_IS_TRNSP
//                            /* Assign target OS.    */
#define OS_PLATFORM     LINUXRPC64_PLATFORM
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */


/* --------------------------------   PART 3: Validation    --------------------------------------------------------------- */
                                    /*---------------------*\
                                    |* SHOULD NOT BE EDIT  *|
                                    \*---------------------*/
                            /* Validation assignments of CPU Endian type */
#if   ((ENDIAN_SELECT == RPC_ENDIAN_AUTO_SET) || (ENDIAN_SELECT == IPC_ENDIAN_IS_TRNSP))
    /* Do Nothing - its OK */
#else
    #error "***OS_PlatformsSelect.h: Symbol 'ENDIAN_SELECT' should set to one of: 'RPC_ENDIAN_AUTO_SET' or 'IPC_ENDIAN_IS_TRNSP' "
#endif
                            
                            /* Confirmation (check) for well assignment of OS Platform */
#if (                                              \
        (OS_PLATFORM == LINUXIPC32_PLATFORM)||     \
        (OS_PLATFORM == WIN32_PLATFORM)     ||     \
        (OS_PLATFORM == WIN64_PLATFORM)     ||     \
        (OS_PLATFORM == LINUXRPC32_PLATFORM)||     \
        (OS_PLATFORM == LINUXRPC64_PLATFORM)||     \
        (OS_PLATFORM == VXWORKS32_PLATFORM)        \
    )
#else
    #error "***OS_PlatformsSelect.h: Symbol 'OS_PLATFORM' should set to one of: 'LINUXIPC32_PLATFORM', 'WIN32_PLATFORM', Etc... (See OS_PlatformSelect.h) "
#endif
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

        /* OS_PLATFORMSSELECT_H_ */
#endif

