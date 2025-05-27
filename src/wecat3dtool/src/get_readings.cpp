#include <iostream>
#include <cstring>
#include <unistd.h>
#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"

int main()
{
  // 1) connect
  char ip[]   = "192.168.100.1";
  char port[] = "32001";
  void* h = EthernetScanner_Connect(ip, port, 1000);
  if (!h) { std::cerr<<"Connect failed\n"; return 1; }

  // 2) wait for ready
  int st=0, tries=0;
  while (st!=3 && tries++<20) {
    usleep(100000);
    EthernetScanner_GetConnectStatus(h,&st);
  }
  if (st!=3) { std::cerr<<"Not ready ("<<st<<")\n"; return 1; }

  // 3) grab ONE profile (we only care about populating scan data)
  const int W=1280, Z=100, BUFSZ=ETHERNETSCANNER_BUFFERSIZEMAX;
  double xb[W*Z], zb[W*Z]; int ib[W*Z], wb[W*Z];
  unsigned int enc; unsigned char io; int pc;
  int pts = EthernetScanner_GetXZIExtended(
    h, xb, zb, ib, wb, W*Z, &enc, &io, 1000, nullptr,0,&pc
  );
  std::cout<<"Pulled one profile, pts="<<pts<<"\n";

  // 4) now read in scan-mode (iCacheTime=-1)
  const char* props[] = {
    "GetExposureTime",
    "GetExposureTime2",
    "GetAutoExposureMode",
    "GetAutoExposureTimeMin",
    "GetAutoExposureTimeMax",
    "GetAutoExposureIntensityRangeMin",
    "GetAutoExposureIntensityRangeMax",
    "GetAutoExposureRangeXMin",
    "GetAutoExposureRangeXMax"
  };
  char buf[BUFSZ];
  std::cout<<"\n=== Exposure (scan-mode) ===\n";
  for (auto &c: props) {
    memset(buf,0,BUFSZ);
    int r = EthernetScanner_ReadData(h, (char*)c, buf, BUFSZ, /*scan=*/-1);
    if (r==ETHERNETSCANNER_READDATAOK)
      std::cout<<" "<<c<<" = "<<buf<<"\n";
    else
      std::cout<<" FAILED "<<c<<" (code="<<r<<")\n";
  }

  EthernetScanner_Disconnect(h);
  return 0;
}

