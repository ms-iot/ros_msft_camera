// Copyright (C) Microsoft Corporation. All rights reserved.
#include "VideoStreamer.h"
#include <iostream>
extern "C" {
#include <libavformat/avformat.h>
}

class SinkGrabberCBWrapper : public IMFSampleGrabberSinkCallback
{
    IMFSampleGrabberSinkCallback* m_pParent;
    long m_cRef;
    SinkGrabberCBWrapper(IMFSampleGrabberSinkCallback* pParent)
        :m_cRef(1)
    {
        m_pParent = pParent;
    }
    virtual ~SinkGrabberCBWrapper() = default;

public:
    static SinkGrabberCBWrapper* CreateWrapper(IMFSampleGrabberSinkCallback* pParent)
    {
        return new SinkGrabberCBWrapper(pParent);
    }
    // IUnknown methods
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv)
    {
        static const QITAB qit[] =
        {
            QITABENT(SinkGrabberCBWrapper, IMFSampleGrabberSinkCallback),
            QITABENT(SinkGrabberCBWrapper, IMFClockStateSink),
            { 0 }
        };
        return QISearch(this, qit, riid, ppv);
    }

    STDMETHODIMP_(ULONG) AddRef()
    {
        return InterlockedIncrement(&m_cRef);
    }

    STDMETHODIMP_(ULONG) Release()
    {
        ULONG cRef = InterlockedDecrement(&m_cRef);
        if (cRef == 0)
        {
            delete this;
        }
        return cRef;

    }

    // IMFClockStateSink methods
    STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
    {
        return m_pParent->OnClockStart(hnsSystemTime, llClockStartOffset);
    }
    STDMETHODIMP OnClockStop(MFTIME hnsSystemTime)
    {
        return m_pParent->OnClockStop(hnsSystemTime);
    }
    STDMETHODIMP OnClockPause(MFTIME hnsSystemTime)
    {
        return m_pParent->OnClockPause(hnsSystemTime);
    }
    STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime)
    {
        return m_pParent->OnClockRestart(hnsSystemTime);
    }
    STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate)
    {
        return m_pParent->OnClockSetRate(hnsSystemTime, flRate);
    }

    // IMFSampleGrabberSinkCallback methods
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock* pClock)
    {
        return m_pParent->OnSetPresentationClock(pClock);
    }
    STDMETHODIMP OnShutdown()
    {
        return m_pParent->OnShutdown();
    }
    //IMFSampleGrabberCallback
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
        DWORD dwSampleSize)
    {
        return m_pParent->OnProcessSample(guidMajorMediaType, dwSampleFlags, llSampleTime, llSampleDuration, pSampleBuffer, dwSampleSize);
    }
};

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
    static IVideoStreamer* CreateInstance();

    void AddDestination(std::string destination, std::string protocol = "rtp") override;
    void RemoveDestination(std::string destination) override;
    void GenerateSDP(char* buf, size_t maxSize, std::string destination) override;

    //IMFSampleGrabberCallback
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
        DWORD dwSampleSize);
};

