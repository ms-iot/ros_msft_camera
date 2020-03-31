// Copyright (C) Microsoft Corporation. All rights reserved.
#include<Windows.h>
#include <synchapi.h>
#include "wincapture.h"
#include "winrospublisher.h"
#include "VideoStreamer.h"
#include <strmif.h>
#include <Codecapi.h>
#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "avcodec.lib")
#ifdef TIGHT_LATENCY_CONTROL
    uint32_t g_dropCount;
#endif
struct GUIDComparer
{
    bool operator()(const GUID& Left, const GUID& Right) const
    {
        // comparison logic goes here
        return memcmp(&Left, &Right, sizeof(Right)) < 0;
    }
};
std::map <GUID, AVCodecID, GUIDComparer> g_CodecMapMFtoFF =
{ {MFVideoFormat_H264, AV_CODEC_ID_H264},
  {MFVideoFormat_MPEG2, AV_CODEC_ID_MPEG2VIDEO}
};

bool VideoStreamer::s_FFmpegInitDone = false;

// Create a new instance of the object.
HRESULT VideoStreamer::CreateInstance(VideoStreamer** ppCB)
{
    if (!s_FFmpegInitDone)
    {
        avcodec_register_all();
        av_register_all();
        avformat_network_init();
        s_FFmpegInitDone = true;
    }
    *ppCB = new (std::nothrow) VideoStreamer();

    if (ppCB == NULL)
    {
        return E_OUTOFMEMORY;
    }

    return S_OK;
}

STDMETHODIMP VideoStreamer::QueryInterface(REFIID riid, void** ppv)
{
    static const QITAB qit[] =
    {
        QITABENT(VideoStreamer, IMFSampleGrabberSinkCallback),
        QITABENT(VideoStreamer, IMFClockStateSink),
        { 0 }
    };
    return QISearch(this, qit, riid, ppv);
}

STDMETHODIMP_(ULONG) VideoStreamer::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) VideoStreamer::Release()
{
    ULONG cRef = InterlockedDecrement(&m_cRef);
    if (cRef == 0)
    {
        delete this;
    }
    return cRef;

}

// IMFClockStateSink methods.

// In these example, the IMFClockStateSink methods do not perform any actions. 
// You can use these methods to track the state of the sample grabber sink.

STDMETHODIMP VideoStreamer::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    return S_OK;
}

STDMETHODIMP VideoStreamer::OnClockStop(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP VideoStreamer::OnClockPause(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP VideoStreamer::OnClockRestart(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP VideoStreamer::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    return S_OK;
}

// IMFSampleGrabberSink methods.

STDMETHODIMP VideoStreamer::OnSetPresentationClock(IMFPresentationClock* pClock)
{
    return S_OK;
}

STDMETHODIMP VideoStreamer::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
    LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
    DWORD dwSampleSize)
{
#ifdef TIGHT_LATENCY_CONTROL
    auto tm = MFGetSystemTime();
    //std::cout << "\nDelay:" << (tm - llSampleTime)/10000;
    auto delay = (tm - llSampleTime) / 10000;
    if (delay > MAX_SOURCE_LATENCY)
    {
        std::cout << "\ndrop Sample :" << delay;
        g_dropCount++;
    }
#endif
    for (auto& avc : m_aAvfctx)
    {
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = (uint8_t*)pSampleBuffer;
        pkt.size = dwSampleSize;
        pkt.duration = llSampleDuration / 10000;
        pkt.pts = llSampleTime / 10000;
        pkt.dts = pkt.pts;
        av_interleaved_write_frame(avc.second, &pkt);
        av_packet_unref(&pkt);
    }

    return S_OK;
}

STDMETHODIMP VideoStreamer::OnShutdown()
{
    return S_OK;
}


void VideoStreamer::ConfigEncoder(uint32_t width, uint32_t height, float framerate, GUID inVideoFormat, GUID outVideoFormat, uint32_t bitrate)
{
    winrt::com_ptr<IMFMediaType> spOutType, spInType;
    
    winrt::com_ptr<IMFActivate>  spSinkActivate;
    winrt::com_ptr<IMFMediaSink> spSink;

    m_width = width;
    m_height = height;
    m_frameRate = framerate;
    m_bitrate = bitrate;
    m_outVideoFormat = outVideoFormat;

    check_hresult(MFCreateMediaType(spOutType.put()));
    check_hresult(MFCreateMediaType(spInType.put()));

    check_hresult(spOutType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spOutType->SetGUID(MF_MT_SUBTYPE, outVideoFormat));
    check_hresult(spOutType->SetUINT32(MF_MT_AVG_BITRATE, bitrate));

    check_hresult(spOutType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
    check_hresult(MFSetAttributeSize(spOutType.get(), MF_MT_FRAME_SIZE, width, height));
    check_hresult(MFSetAttributeRatio(spOutType.get(), MF_MT_FRAME_RATE, (int)(framerate * 100), 100));

    check_hresult(spOutType->CopyAllItems(spInType.get()));
    check_hresult(spInType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spInType->SetGUID(MF_MT_SUBTYPE, inVideoFormat));
    check_hresult(spInType->DeleteItem(MF_MT_AVG_BITRATE));

    // Create the sample grabber sink.
    check_hresult(MFCreateSampleGrabberSinkActivate(spOutType.get(), this, spSinkActivate.put()));

    // To run as fast as possible, set this attribute
    check_hresult(spSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE));
    check_hresult(spSinkActivate->ActivateObject(__uuidof(IMFMediaSink), spSink.put_void()));
    winrt::com_ptr<IMFAttributes> spSWAttributes;
    check_hresult(MFCreateAttributes(spSWAttributes.put(), 1));
    check_hresult(spSWAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
    check_hresult(MFCreateSinkWriterFromMediaSink(spSink.get(), spSWAttributes.get(), m_spSinkWriter.put()));

    check_hresult(m_spSinkWriter->SetInputMediaType(0, spInType.get(), nullptr));

    check_hresult(m_spSinkWriter->BeginWriting());

}

void VideoStreamer::AddDestination(std::string destination, std::string protocol)
{
    AVFormatContext* pAvfctx;
    if (m_aAvfctx.find(destination) != m_aAvfctx.end())
    {
        //destination already exists
        return;
    }
    AVCodecID codec_id = g_CodecMapMFtoFF[m_outVideoFormat];
    AVOutputFormat* fmt = av_guess_format(protocol.c_str(), NULL, NULL);
    std::string destString = protocol + std::string("://") + destination;
    avformat_alloc_output_context2(&pAvfctx, fmt, fmt->name,
        destString.c_str());

    avio_open(&pAvfctx->pb, pAvfctx->filename, AVIO_FLAG_WRITE);

    struct AVStream* stream = avformat_new_stream(pAvfctx, /*codec*/nullptr);
    stream->codecpar->bit_rate = m_bitrate;
    stream->codecpar->width = m_width;
    stream->codecpar->height = m_height;
    stream->codecpar->codec_id = g_CodecMapMFtoFF[m_outVideoFormat];;
    stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    stream->time_base.num = 100;
    stream->time_base.den = (int)(m_frameRate * 100);

    avformat_write_header(pAvfctx, NULL);
    m_aAvfctx.insert({ destination, pAvfctx });
}

void VideoStreamer::RemoveDestination(std::string destination)
{
    m_aAvfctx.erase(destination);
}

void VideoStreamer::GenerateSDP(char *buf, size_t maxSize, std::string destination)
{
    AVFormatContext* pAvfctx;
    AVCodecID codec_id = g_CodecMapMFtoFF[m_outVideoFormat];

    AVOutputFormat* fmt = av_guess_format("rtp", NULL, NULL);
    std::string destString = std::string("rtp://") + destination;
    avformat_alloc_output_context2(&pAvfctx, fmt, fmt->name,
        destString.c_str());

    printf("Writing to %s\n", pAvfctx->filename);

    avio_open(&pAvfctx->pb, pAvfctx->filename, AVIO_FLAG_WRITE);

    struct AVStream* stream = avformat_new_stream(pAvfctx, /*codec*/nullptr);
    stream->codecpar->bit_rate = m_bitrate;
    stream->codecpar->width = m_width;
    stream->codecpar->height = m_height;
    stream->codecpar->codec_id = g_CodecMapMFtoFF[m_outVideoFormat];;
    stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    stream->time_base.num = 100;
    stream->time_base.den = (int)(m_frameRate * 100);

    AVFormatContext* ac[] = { pAvfctx };
    av_sdp_create(ac, 1, (char*)buf, maxSize);

    avformat_free_context(pAvfctx);
}
