# Compatibility entry point. Keep all QNX/Python ABI configuration in one
# authoritative toolchain so these two historical paths cannot drift again.
include("${CMAKE_CURRENT_LIST_DIR}/../platform/qnx.nto.toolchain.cmake")
