Import('RTT_ROOT')
Import('rtconfig')
from building import *

cwd     = GetCurrentDir()

include_path = Split("""
lib/include
apps/system/generic/machine/rv64_virt
lib/rpmsg
lib/include/openamp
""")

src = []
src += ['apps/examples/rpmsg_sample_echo/rpmsg-sample-echo.c']
src += ['apps/tests/msg/rpmsg-update.c']
src += ['apps/examples/matrix_multiply/matrix_multiplyd.c']
src += ['apps/system/generic/machine/rv64_virt/virt_rv64_rproc.c']
src += ['apps/system/generic/machine/rv64_virt/helper.c']
src += ['apps/system/generic/machine/rv64_virt/platform_info.c']
src += ['apps/system/generic/machine/rv64_virt/rsc_table.c']
src += ['lib/rpmsg/rpmsg.c']
src += ['lib/rpmsg/rpmsg_virtio.c']
src += ['lib/proxy/rpmsg_retarget.c']
src += ['lib/remoteproc/rsc_table_parser.c']
src += ['lib/remoteproc/remoteproc_virtio.c']
src += ['lib/remoteproc/elf_loader.c']
src += ['lib/remoteproc/remoteproc.c']
src += ['lib/virtio/virtqueue.c']
src += ['lib/virtio/virtio.c']

CPPDEFINES = []
CPPDEFINES += ['RPMSG_NO_IPI']
group = DefineGroup('OpenAMP', src, depend = [''], CPPPATH = include_path, CPPDEFINES = CPPDEFINES)

Return('group')
