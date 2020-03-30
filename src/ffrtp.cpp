// Copyright (C) Microsoft Corporation. All rights reserved.
#include<Windows.h>
#include <synchapi.h>
#include "wincapture.h"
#include "winrospublisher.h"
#include "ffrtp.h"
#include <strmif.h>
#include <Codecapi.h>
#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "avcodec.lib")
uint32_t g_dropCount;
void SampleGrabberCB::InitFFrtp(uint32_t width, uint32_t height,float frameRate, std::string destination)
{
        avcodec_register_all();
        av_register_all();
        avformat_network_init();
        AVFormatContext* pAvfctx;
        AVCodecID codec_id = AV_CODEC_ID_H264;
        AVCodec* codec;

        codec = avcodec_find_encoder(codec_id);
        AVOutputFormat* fmt = av_guess_format("rtp", NULL, NULL);
        std::string destString = std::string("rtp://") + destination;
        avformat_alloc_output_context2(&pAvfctx, fmt, fmt->name,
            destString.c_str());

        printf("Writing to %s\n", pAvfctx->filename);

        avio_open(&pAvfctx->pb, pAvfctx->filename, AVIO_FLAG_WRITE);

        struct AVStream* stream = avformat_new_stream(pAvfctx, codec);
        stream->codecpar->bit_rate = 1000000;
        stream->codecpar->width = width;
        stream->codecpar->height = height;
        stream->codecpar->codec_id = AV_CODEC_ID_H264;
        stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        stream->time_base.num = 100;
        stream->time_base.den = (int)(frameRate*100);       

        char buf[20000];
        AVFormatContext* ac[] = { pAvfctx };
        av_sdp_create(ac, 1, buf, 20000);
        printf("sdp:\n%s\n", buf);
        FILE* fsdp;
        fopen_s(&fsdp, "test.sdp", "w");
        fprintf(fsdp, "%s", buf);
        fclose(fsdp);
        
        system("start cmd /c C:\\Tools\\ffmpeg-20200324-e5d25d1-win64-static\\bin\\ffplay.exe -protocol_whitelist file,udp,rtp test.sdp");

        avformat_write_header(pAvfctx, NULL);
        m_aAvfctx.push_back({ destination,pAvfctx });
}
//void SampleGrabberCB::SendPacket(AVPacket pkt)
//{
//        av_interleaved_write_frame(avfctx, &pkt);
//        av_packet_unref(&pkt);
//}
// SampleGrabberCB implementation

// Create a new instance of the object.
HRESULT SampleGrabberCB::CreateInstance(uint32_t width, uint32_t height, SampleGrabberCB** ppCB)
{
    *ppCB = new (std::nothrow) SampleGrabberCB();

    if (ppCB == NULL)
    {
        return E_OUTOFMEMORY;
    }

    return S_OK;
}

STDMETHODIMP SampleGrabberCB::QueryInterface(REFIID riid, void** ppv)
{
    static const QITAB qit[] =
    {
        QITABENT(SampleGrabberCB, IMFSampleGrabberSinkCallback),
        QITABENT(SampleGrabberCB, IMFClockStateSink),
        { 0 }
    };
    return QISearch(this, qit, riid, ppv);
}

STDMETHODIMP_(ULONG) SampleGrabberCB::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) SampleGrabberCB::Release()
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

STDMETHODIMP SampleGrabberCB::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnClockStop(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnClockPause(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnClockRestart(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    return S_OK;
}

// IMFSampleGrabberSink methods.

STDMETHODIMP SampleGrabberCB::OnSetPresentationClock(IMFPresentationClock* pClock)
{
    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
    LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE* pSampleBuffer,
    DWORD dwSampleSize)
{
    // Display information about the sample.
    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = (uint8_t*)pSampleBuffer;
    pkt.size = dwSampleSize;
    pkt.duration = llSampleDuration / 10000;
    pkt.pts = llSampleTime / 10000;
    pkt.dts = pkt.pts;
    //auto tm = MFGetSystemTime();
    //std::cout << "\nDelay:" << (tm - llSampleTime)/10000;
    /*auto delay = (tm - llSampleTime) / 10000;
    if (delay > 200)
    {
        std::cout << "\ndrop Sample :" << delay;
        g_dropCount++;
    }*/
    //else
    for(auto &avc : m_aAvfctx)
    {
        av_interleaved_write_frame(avc.second, &pkt);
    }
        av_packet_unref(&pkt);
    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnShutdown()
{
    return S_OK;
}


RTPStreamer::RTPStreamer(uint32_t width, uint32_t height, float framerate, GUID outVideoFormat, GUID inVideoFormat, uint32_t bitrate, std::string destination)
{
    winrt::com_ptr<IMFMediaType> spOutType, spInType;
    //winrt::com_ptr<SampleGrabberCB> spCallback;
    winrt::com_ptr<IMFActivate>  spSinkActivate;
    winrt::com_ptr<IMFMediaSink> spSink;
    //winrt::com_ptr<IMFSinkWriter> spSinkWriter;

    check_hresult(MFCreateMediaType(spOutType.put()));
    check_hresult(MFCreateMediaType(spInType.put()));

    check_hresult(spOutType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spOutType->SetGUID(MF_MT_SUBTYPE, outVideoFormat));
    check_hresult(spOutType->SetUINT32(MF_MT_AVG_BITRATE, bitrate));
    //check_hresult(spOutType->SetUINT32(MF_MT_MPEG2_PROFILE, eAVEncH264VProfile_Base));
    check_hresult(spOutType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
    check_hresult(MFSetAttributeSize(spOutType.get(), MF_MT_FRAME_SIZE, width, height));
    check_hresult(MFSetAttributeRatio(spOutType.get(), MF_MT_FRAME_RATE, (int)(framerate * 100), 100));

    check_hresult(spOutType->CopyAllItems(spInType.get()));
    check_hresult(spInType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spInType->SetGUID(MF_MT_SUBTYPE, inVideoFormat));
    check_hresult(spInType->DeleteItem(MF_MT_AVG_BITRATE));

    // Create the sample grabber sink.
    check_hresult(SampleGrabberCB::CreateInstance(width, height, spSampleGrabberCB.put()));
    spSampleGrabberCB->InitFFrtp(width, height, framerate, destination);
    check_hresult(MFCreateSampleGrabberSinkActivate(spOutType.get(), spSampleGrabberCB.get(), spSinkActivate.put()));
    // To run as fast as possible, set this attribute (requires Windows 7):
    check_hresult(spSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE));
    check_hresult(spSinkActivate->ActivateObject(__uuidof(IMFMediaSink), spSink.put_void()));
    winrt::com_ptr<IMFAttributes> spSWAttributes;
    check_hresult(MFCreateAttributes(spSWAttributes.put(), 1));
    check_hresult(spSWAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
    check_hresult(MFCreateSinkWriterFromMediaSink(spSink.get(), spSWAttributes.get(), spSinkWriter.put()));

    check_hresult(spSinkWriter->SetInputMediaType(0, spInType.get(), nullptr));
    //winrt::com_ptr<ICodecAPI> spCodecAPI;
    //check_hresult(spSinkWriter->GetServiceForStream(0, GUID_NULL, __uuidof(ICodecAPI), spCodecAPI.put_void()));
    /*VARIANT vt;
    vt.boolVal = VARIANT_TRUE;
    GUID prop = CODECAPI_AVLowLatencyMode;
    spCodecAPI->SetValue(&prop, &vt);*/

    check_hresult(spSinkWriter->BeginWriting());

}
void RTPStreamer::AddDestination(std::string destination)
{
    //spSampleGrabberCB->InitFFrtp();
}