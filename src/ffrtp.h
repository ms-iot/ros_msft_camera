// Copyright (C) Microsoft Corporation. All rights reserved.

#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <winrt\base.h>
#include <Shlwapi.h>
#include <winrt\Windows.Foundation.Collections.h>
#include <winrt\Windows.Devices.Enumeration.h>
#include <winrt\Windows.System.Threading.h>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>

#include <libavutil/opt.h>
#include <libavutil/channel_layout.h>
#include <libavutil/common.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
}

//IMFSinkWriter* ConfigRTP(uint32_t width, uint32_t height, float framerate, GUID outVideoFormat, GUID inVideoFormat, uint32_t bitrate, std::string destination);

// The class that implements the callback interface.
class SampleGrabberCB : public IMFSampleGrabberSinkCallback
{
    long m_cRef;
    std::vector<std::pair<std::string, AVFormatContext*>> m_aAvfctx;
    SampleGrabberCB() : m_cRef(1) { }
    ~SampleGrabberCB() 
    {
        for(auto &avc : m_aAvfctx)
        {
            avformat_free_context(avc.second);
        }
        m_aAvfctx.clear();
    }
public:
    void InitFFrtp(uint32_t width, uint32_t height, float frameRate, std::string destination);
    static HRESULT CreateInstance(uint32_t width, uint32_t height, SampleGrabberCB** ppCB);

    // IUnknown methods
    STDMETHODIMP QueryInterface(REFIID iid, void** ppv);
    STDMETHODIMP_(ULONG) AddRef();
    STDMETHODIMP_(ULONG) Release();

    // IMFClockStateSink methods
    STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
    STDMETHODIMP OnClockStop(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockPause(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate);

    // IMFSampleGrabberSinkCallback methods
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock* pClock);
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
        DWORD dwSampleSize);
    STDMETHODIMP OnShutdown();
};
class RTPStreamer
{
    winrt::com_ptr<IMFSinkWriter> spSinkWriter;
    winrt::com_ptr<SampleGrabberCB> spSampleGrabberCB;
public:
    RTPStreamer(uint32_t width, uint32_t height, float framerate, GUID outVideoFormat, GUID inVideoFormat, uint32_t bitrate, std::string destination);
    void AddDestination(std::string destination);
    void RemoveDestination(std::string destination) {}
    void WritePacket(IMFSample* pSample) { check_hresult( spSinkWriter->WriteSample(0,pSample)); }
};
