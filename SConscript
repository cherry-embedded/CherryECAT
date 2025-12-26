from building import *

cwd = GetCurrentDir()
path = [cwd + '/include']
src = []

LIBS    = []
LIBPATH = []
CPPDEFINES = []

src += Glob('src/ec_cmd.c')
src += Glob('src/ec_coe.c')
src += Glob('src/ec_common.c')
src += Glob('src/ec_datagram.c')
src += Glob('src/ec_eoe.c')
src += Glob('src/ec_foe.c')
src += Glob('src/ec_mailbox.c')
src += Glob('src/ec_master.c')
src += Glob('src/ec_netdev.c')
src += Glob('src/ec_sii.c')
src += Glob('src/ec_slave.c')
src += Glob('src/ec_timestamp.c')
src += Glob('src/phy/chry_phy.c')
src += Glob('osal/ec_osal_rtthread.c')

if GetDepend(['PKG_CHERRYECAT_NETDEV_HPMICRO']):
    src += Glob('port/netdev_hpmicro.c')

if GetDepend(['PKG_CHERRYECAT_NETDEV_RENESAS']):
    src += Glob('port/netdev_renesas.c')

group = DefineGroup('CherryECAT', src, depend = ['PKG_USING_CHERRYECAT'], LIBS = LIBS, LIBPATH=LIBPATH, CPPPATH = path, CPPDEFINES = CPPDEFINES)

Return('group')

