// Copyright (C) Microsoft Corporation. All rights reserved.
#include "VideoStreamer.h"
extern "C" {
#include <libavformat/avformat.h>
}

// The class that implements the callback interface.
class VideoStreamerBase : public IMFSampleGrabberSinkCallback, public IVideoStreamer
{
protected:
    long m_cRef;
    uint32_t m_height;
    float m_frameRate;
    uint32_t m_width;
    uint32_t m_bitrate;
    GUID m_outVideoFormat;

    winrt::com_ptr<IMFSinkWriter> m_spSinkWriter;

    VideoStreamerBase() :
        m_cRef(1),
        m_width(0),
        m_bitrate(0),
        m_height(0),
        m_frameRate(0),
        m_outVideoFormat(GUID_NULL)
    { }

    virtual ~VideoStreamerBase() = default;

public:

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
    STDMETHODIMP OnShutdown();

    virtual void WritePacket(IMFSample* pSample);

    //IVideoStreamer
    virtual void ConfigEncoder(uint32_t width, uint32_t height, float framerate, GUID inVideoFormat, GUID outVideoFormat, uint32_t bitrate);

};


class VideoStreamerFFmpeg sealed : public VideoStreamerBase
{
    std::map<std::string, AVFormatContext*> m_aAvfctx;
    VideoStreamerFFmpeg() :VideoStreamerBase() {}
    virtual ~VideoStreamerFFmpeg();
    AVFormatContext* CreateAVformatCtxt(std::string destination, std::string protocol);
public:
    static HRESULT CreateInstance(IVideoStreamer** ppVideoStreamer);
 
    void AddDestination(std::string destination, std::string protocol = "rtp") override;
    void RemoveDestination(std::string destination) override;
    void GenerateSDP(char* buf, size_t maxSize, std::string destination) override;

    //IMFSampleGrabberCallback
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
        DWORD dwSampleSize);
};

