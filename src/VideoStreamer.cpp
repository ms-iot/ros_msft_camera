// Copyright (C) Microsoft Corporation. All rights reserved.
#include "VideoStreamerInternal.h"
#include <iostream>
#pragma comment(lib, "avformat.lib")
using namespace winrt;
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

bool VideoStreamerFFmpeg::s_FFmpegInitDone = false;

void VideoStreamerBase::WritePacket(IMFSample* pSample)
{
    winrt::check_hresult(m_spSinkWriter->WriteSample(0, pSample));
}

VideoStreamerFFmpeg::~VideoStreamerFFmpeg()
{
    for (auto& avc : m_aAvfctx)
    {
        avformat_free_context(avc.second);
    }
    m_aAvfctx.clear();
}

void VideoStreamerFFmpeg::InitFFmpeg()
{
    if (!s_FFmpegInitDone)
    {
        //avcodec_register_all();
        //av_register_all();
        avformat_network_init();
        s_FFmpegInitDone = true;
    }

}

HRESULT VideoStreamerFFmpeg::CreateInstance(IVideoStreamer** ppVideoStreamer)
{
    VideoStreamerFFmpeg::InitFFmpeg();
    //*ppVideoStreamer = new (std::nothrow) VideoStreamerFFmpeg();
    auto vs = new (std::nothrow) VideoStreamerFFmpeg();
    //*ppVideoStreamer = new (std::nothrow) VideoStreamerFFmpeg();
    vs->QueryInterface(__uuidof(IVideoStreamer), (void**)ppVideoStreamer);
    if (ppVideoStreamer == NULL)
    {
        return E_OUTOFMEMORY;
    }
    return S_OK;
}

// Create a new instance of the object.
HRESULT CreateFFVideoStreamer(IVideoStreamer** ppVideoStreamer)
{
    return VideoStreamerFFmpeg::CreateInstance(ppVideoStreamer);
}

STDMETHODIMP VideoStreamerBase::QueryInterface(REFIID riid, void** ppv)
{
    static const QITAB qit[] =
    {
        QITABENT(VideoStreamerBase, IMFSampleGrabberSinkCallback),
        QITABENT(VideoStreamerBase, IMFClockStateSink),
        QITABENT(VideoStreamerBase, IVideoStreamer),
        { 0 }
    };
    return QISearch(this, qit, riid, ppv);
}

STDMETHODIMP_(ULONG) VideoStreamerBase::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) VideoStreamerBase::Release()
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

STDMETHODIMP VideoStreamerBase::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    return S_OK;
}

STDMETHODIMP VideoStreamerBase::OnClockStop(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP VideoStreamerBase::OnClockPause(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP VideoStreamerBase::OnClockRestart(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP VideoStreamerBase::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    return S_OK;
}

// IMFSampleGrabberSink methods.

STDMETHODIMP VideoStreamerBase::OnSetPresentationClock(IMFPresentationClock* pClock)
{
    return S_OK;
}

STDMETHODIMP VideoStreamerFFmpeg::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
    LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
    DWORD dwSampleSize)
{
    auto tm = MFGetSystemTime();
    std::cout << "\t SW Delay:" << (tm - llSampleTime)/10000;
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

STDMETHODIMP VideoStreamerBase::OnShutdown()
{
    return S_OK;
}


void VideoStreamerBase::ConfigEncoder(uint32_t width, uint32_t height, float framerate, GUID inVideoFormat, GUID outVideoFormat, uint32_t bitrate)
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
    check_hresult(MFCreateAttributes(spSWAttributes.put(), 2));
    check_hresult(spSWAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
    check_hresult(spSWAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, FALSE));
    check_hresult(MFCreateSinkWriterFromMediaSink(spSink.get(), spSWAttributes.get(), m_spSinkWriter.put()));
    check_hresult(m_spSinkWriter->SetInputMediaType(0, spInType.get(), nullptr));

    check_hresult(m_spSinkWriter->BeginWriting());
    winrt::com_ptr<IMFSinkWriterEx> spSinkWriterEx;
    int i = 0;
    GUID cagtegory;
    winrt::com_ptr<IMFTransform> spTranform;
    while (SUCCEEDED(m_spSinkWriter.as<IMFSinkWriterEx>()->GetTransformForStream(0, i++, &cagtegory, spTranform.put())))
    {
        printf("\nTranform %d: %x-%x-%x-%x%x%x%x", i - 1, cagtegory.Data1, cagtegory.Data2, cagtegory.Data3, cagtegory.Data4[0], cagtegory.Data4[1], cagtegory.Data4[2], cagtegory.Data4[3]);
    }
    printf("\nTotal: %d",i);
}

void VideoStreamerFFmpeg::AddDestination(std::string destination, std::string protocol)
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

    avio_open(&pAvfctx->pb, pAvfctx->url, AVIO_FLAG_WRITE);

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

void VideoStreamerFFmpeg::RemoveDestination(std::string destination)
{
    m_aAvfctx.erase(destination);
}

void VideoStreamerFFmpeg::GenerateSDP(char *buf, size_t maxSize, std::string destination)
{
    AVFormatContext* pAvfctx;
    AVCodecID codec_id = g_CodecMapMFtoFF[m_outVideoFormat];

    AVOutputFormat* fmt = av_guess_format("rtp", NULL, NULL);
    std::string destString = std::string("rtp://") + destination;
    avformat_alloc_output_context2(&pAvfctx, fmt, fmt->name,
        destString.c_str());

    printf("Writing to %s\n", pAvfctx->url);

    avio_open(&pAvfctx->pb, pAvfctx->url, AVIO_FLAG_WRITE);

    struct AVStream* stream = avformat_new_stream(pAvfctx, /*codec*/nullptr);
    stream->codecpar->bit_rate = m_bitrate;
    stream->codecpar->width = m_width;
    stream->codecpar->height = m_height;
    stream->codecpar->codec_id = g_CodecMapMFtoFF[m_outVideoFormat];;
    stream->codecpar->codec_type = AVMediaType::AVMEDIA_TYPE_VIDEO;
    stream->time_base.num = 100;
    stream->time_base.den = (int)(m_frameRate * 100);

    AVFormatContext* ac[] = { pAvfctx };
    av_sdp_create(ac, 1, (char*)buf, maxSize);

    avformat_free_context(pAvfctx);
}
