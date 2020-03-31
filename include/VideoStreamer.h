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

// The class that implements the callback interface.
class VideoStreamer : public IMFSampleGrabberSinkCallback
{
    long m_cRef;
    std::map<std::string, AVFormatContext*> m_aAvfctx;
    static bool s_FFmpegInitDone;
    winrt::com_ptr<IMFSinkWriter> m_spSinkWriter;
    uint32_t m_width;
    uint32_t m_height;
    float m_frameRate;
    uint32_t m_bitrate;
    GUID m_outVideoFormat;

    VideoStreamer() : m_cRef(1) { }
    ~VideoStreamer() 
    {
        for(auto &avc : m_aAvfctx)
        {
            avformat_free_context(avc.second);
        }
        m_aAvfctx.clear();
    }
public:
    
    static HRESULT CreateInstance( VideoStreamer** ppCB);

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
    void ConfigEncoder(uint32_t width, uint32_t height, float framerate, GUID inVideoFormat, GUID outVideoFormat, uint32_t bitrate);
    void AddDestination(std::string destination, std::string protocol = "rtp");
    void RemoveDestination(std::string destination);
    void WritePacket(IMFSample* pSample) { winrt::check_hresult( m_spSinkWriter->WriteSample(0,pSample)); }
    void GenerateSDP(char* buf, size_t maxSize, std::string destination);
};
