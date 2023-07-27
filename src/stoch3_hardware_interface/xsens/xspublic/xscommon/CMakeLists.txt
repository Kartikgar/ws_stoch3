

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include xscommon/xscommon_config.h -include xstypes/xsens_compat.h")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include xscommon/xscommon_config.h -include xstypes/xsens_compat.h")



include_directories(../)

add_library(xscommon STATIC
  additionalloggerbase.cpp
  consolelogger.cpp
  enumexpandersbase.cpp
  journalexception.cpp
  journalfile.cpp
  journaller.cpp
  journalthreader.cpp
  stackdumper.cpp
  stackwalker.cpp
  stackwalker_linux.cpp
  threading.cpp
  xprintf.cpp
  xsens_math_throw.cpp
  xsens_threadpool.cpp
  fwupdate.c
  xbus.c
  xbusparser.c  
)

install(TARGETS xscommon
  DESTINATION lib)
