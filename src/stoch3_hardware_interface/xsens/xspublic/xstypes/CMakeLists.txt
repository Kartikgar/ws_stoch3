set(CMAKE_CXX_STANDARD 11)

include_directories(../)

add_library(xstypes STATIC
  datapacket_p.cpp
  xsdatapacket.cpp
  xsdebugcounters.cpp
  xssensorranges.cpp
  xsarray.c
  xsbaud.c
  xsbytearray.c
  xscalibrateddata.c
  xscanoutputconfigurationarray.c
  xscanoutputconfiguration.c
  xsdatapacketptrarray.c
  xsdevicecapabilities.c
  xsdeviceidarray.c
  xsdeviceid.c
  xseuler.c
  xsfile.c
  xsfilterprofilearray.c
  xsfilterprofile.c
  xsglovedata.c
  xsint64array.c
  xsintarray.c
  xslibraryloader.c
  xsmalloc.c
  xsmath.c
  xsmatrix3x3.c
  xsmatrix.c
  xsmessagearray.c
  xsmessage.c
  xsoutputconfigurationarray.c
  xsoutputconfiguration.c
  xsportinfoarray.c
  xsportinfo.c
  xsquaternionarray.c
  xsquaternion.c
  xsrange.c
  xsresultvalue.c
  xsrssi.c
  xssdidata.c
  xssimpleversion.c
  xssnapshot.c
  xssocket.c
  xsstringarray.c
  xsstring.c
  xsstringoutputtypearray.c
  xssyncsettingarray.c
  xssyncsetting.c
  xsthread.c
  xstime.c
  xstimeinfo.c
  xstimestamp.c
  xstriggerindicationdata.c
  xstypedefs.c
  xsutctime.c
  xsvector3.c
  xsvector.c
  xsversion.c
)
