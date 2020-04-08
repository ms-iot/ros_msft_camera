// Copyright (C) Microsoft Corporation. All rights reserved.
#include <Windows.h>
#include <mfidl.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <winrt\base.h>
#include <Shlwapi.h>
#include <winrt\Windows.Foundation.Collections.h>
#include <winrt\Windows.Devices.Enumeration.h>
#include <winrt\Windows.System.Threading.h>

#ifdef VIDEOSTREAMER_EXPORTS
#define VIDEOSTREAMER_API __declspec(dllexport)
#else
#define VIDEOSTREAMER_API __declspec(dllimport)
#endif

//EXTERN_C const IID IID_IVideoStreamer;
MIDL_INTERFACE("022C6CB9-64D5-472F-8753-76382CC5F4DA")
IVideoStreamer : public IUnknown
{
public:
    virtual void ConfigEncoder(uint32_t width, uint32_t height, float framerate, GUID inVideoFormat, GUID outVideoFormat, uint32_t bitrate) = 0;
    virtual void AddDestination(std::string destination, std::string protocol = "rtp") = 0;
    virtual void RemoveDestination(std::string destination) = 0;
    virtual void WritePacket(IMFSample* pSample) = 0;
    virtual void GenerateSDP(char* buf, size_t maxSize, std::string destination) = 0;

};

VIDEOSTREAMER_API IVideoStreamer* CreateFFVideoStreamer();

