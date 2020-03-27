// Copyright (C) Microsoft Corporation. All rights reserved.
#include<Windows.h>
#include <synchapi.h>
#include "wincapture.h"
#include "winrospublisher.h"
#include "ffrtp.h"
#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "avcodec.lib")

void SampleGrabberCB::InitFFrtp(uint32_t width, uint32_t height)
{
        avcodec_register_all();
        av_register_all();
        avformat_network_init();

        AVCodecID codec_id = AV_CODEC_ID_H264;
        AVCodec* codec;
        //AVCodecContext* c = NULL;
        int i, ret, x, y, got_output;
        //AVFrame* frame;
        //AVPacket pkt;

        codec = avcodec_find_encoder(codec_id);
        //c = avcodec_alloc_context3(codec);

//c->bit_rate = 1000000;
//c->width = width;
//c->height = height;
//c->time_base.num = 1;
//c->time_base.den = 30;
//c->gop_size = 30;
//c->max_b_frames = 0;
//c->pix_fmt = AV_PIX_FMT_BGR24;
//c->codec_type = AVMEDIA_TYPE_VIDEO;

////c->flags = CODEC_FLAG_GLOBAL_HEADER;

//if (codec_id == AV_CODEC_ID_H264) {
  //  ret = av_opt_set(c->priv_data, "preset", "ultrafast", 0);
   // ret = av_opt_set(c->priv_data, "tune", "zerolatency", 0);
//}

//avcodec_open2(c, codec, NULL);

//frame = av_frame_alloc();
//frame->format = c->pix_fmt;
//frame->width = c->width;
//frame->height = c->height;
//ret = av_image_alloc(frame->data, frame->linesize, c->width, c->height,
 //   c->pix_fmt, 32);
        AVOutputFormat* fmt = av_guess_format("rtp", NULL, NULL);

        ret = avformat_alloc_output_context2(&avfctx, fmt, fmt->name,
            "rtp://127.0.0.1:49990");

        printf("Writing to %s\n", avfctx->filename);

        avio_open(&avfctx->pb, avfctx->filename, AVIO_FLAG_WRITE);

        struct AVStream* stream = avformat_new_stream(avfctx, codec);
        stream->codecpar->bit_rate = 1000000;
        stream->codecpar->width = width;
        stream->codecpar->height = height;
        stream->codecpar->codec_id = AV_CODEC_ID_H264;
        stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        stream->time_base.num = 1;
        stream->time_base.den = 30;

        char buf[200000];
        AVFormatContext* ac[] = { avfctx };
        av_sdp_create(ac, 1, buf, 20000);
        printf("sdp:\n%s\n", buf);
        FILE* fsdp;
        fopen_s(&fsdp, "test.sdp", "w");
        fprintf(fsdp, "%s", buf);
        fclose(fsdp);
        system("PAUSE");
        system("start cmd /c C:\\Tools\\ffmpeg-20200324-e5d25d1-win64-static\\bin\\ffplay.exe -protocol_whitelist file,udp,rtp test.sdp");
        //winrt::slim_mutex m;
        //int j = 0;
        //m.lock();
        //for (i = 0; i < 10000; i++)
        avformat_write_header(avfctx, NULL);
#if 0
        evt.add(
            [&](AVPacket pkt)
            {
                //av_init_packet(&pkt);
                //pkt.data = NULL;    // packet data will be allocated by the encoder
                //pkt.size = 0;
                //fflush(stdout);
                /* prepare a dummy image */
                /* Y */
                //for (y = 0; y < c->height; y++) {
                //    for (x = 0; x < c->width; x++) {
                //        frame->data[0][y * frame->linesize[0] + x] = x + y + i * 3;
                //    }
                //}
                ///* Cb and Cr */
                //for (y = 0; y < c->height / 2; y++) {
                //    for (x = 0; x < c->width / 2; x++) {
                //        frame->data[1][y * frame->linesize[1] + x] = 128 + y + i * 2;
                //        frame->data[2][y * frame->linesize[2] + x] = 64 + x + i * 5;
                //    }
                //}
                //AV_PKT_FLAG_KEY
#if 0
                winrt::com_ptr<IMFMediaBuffer> spMediaBuf;
                winrt::com_ptr<IMF2DBuffer2> spMediaBuf2d;
                BYTE* pix;
                LONG Stride;
                check_hresult(pSample->GetBufferByIndex(0, spMediaBuf.put()));
                spMediaBuf2d = spMediaBuf.as<IMF2DBuffer2>();
                check_hresult(spMediaBuf2d->Lock2D(&pix, &Stride));

                /*for (y = 0; y < c->height; y++)
                    for(x=0; x < c->width; x++)
                {
                    frame->data[0][y * frame->linesize[0] + x*3] = pix[y*Stride + 3*x];
                    frame->data[0][y * frame->linesize[0] + x * 3 +1] = pix[y*Stride + 3*x+1];
                    frame->data[0][y * frame->linesize[0] + x * 3 +2] = pix[y*Stride + 3*x+2];
                }*/
                frame->data[0] = pix;
                frame->linesize[0] = Stride;

                frame->pts = i;
                /* encode the image */
                ret = avcodec_send_frame(c, frame);
                ret = avcodec_receive_packet(c, &pkt);
                check_hresult(spMediaBuf2d->Unlock2D());

                if (ret == AVERROR_EOF) {
                    got_output = false;
                    printf("Stream EOF\n");
                    m.unlock();
                }
                else if (ret == AVERROR(EAGAIN)) {
                    got_output = false;
                    printf("Stream EAGAIN\n");
                }
                else {
                    got_output = true;
                }

                if (got_output)
#else
                if (1)
#endif
                {
                    printf("Write frame %3d (size=%5d)\r", j++, pkt.size);
                    av_interleaved_write_frame(avfctx, &pkt);
                    av_packet_unref(&pkt);
                }

                //Sleep(40);
            });
        //m.lock();
        //while (1);

        // end
#if 0
        ret = avcodec_send_frame(c, NULL);

        /* get the delayed frames */
        for (; ; i++) {
            fflush(stdout);
            ret = avcodec_receive_packet(c, &pkt);
            if (ret == AVERROR_EOF) {
                printf("Stream EOF\n");
                break;
            }
            else if (ret == AVERROR(EAGAIN)) {
                printf("Stream EAGAIN\n");
                got_output = false;
            }
            else {
                got_output = true;
            }

            if (got_output) {
                printf("Write frame %3d (size=%5d)\n", j++, pkt.size);
                av_interleaved_write_frame(avfctx, &pkt);
                av_packet_unref(&pkt);
            }
        }
#endif
        //avcodec_close(c);
        //av_free(c);
        //av_freep(&frame->data[0]);
        //av_frame_free(&frame);
        printf("\n");
        system("pause");
#endif
        //return 0;
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
    //m_evt(pkt);
    av_interleaved_write_frame(avfctx, &pkt);
    av_packet_unref(&pkt);

    return S_OK;
}

STDMETHODIMP SampleGrabberCB::OnShutdown()
{
    return S_OK;
}

IMFSinkWriter *ConfigRTP(uint32_t width, uint32_t height, float framerate, GUID outVideoFormat, GUID inVideoFormat, uint32_t bitrate)
{
    //winrt::event<winrt::delegate<AVPacket>> ffevt;
    winrt::com_ptr<IMFMediaType> spOutType, spInType;
    winrt::com_ptr<SampleGrabberCB> spCallback;
    winrt::com_ptr<IMFActivate>  spSinkActivate;
    winrt::com_ptr<IMFMediaSink> spSink;
    winrt::com_ptr<IMFSinkWriter> spSinkWriter;

    check_hresult(MFCreateMediaType(spOutType.put()));
    check_hresult(MFCreateMediaType(spInType.put()));

    check_hresult(spOutType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spOutType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264));
    check_hresult(spOutType->SetUINT32(MF_MT_AVG_BITRATE, 1000000));
    check_hresult(spOutType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
    check_hresult(MFSetAttributeSize(spOutType.get(), MF_MT_FRAME_SIZE, width, height));
    check_hresult(MFSetAttributeRatio(spOutType.get(), MF_MT_FRAME_RATE, (int)(framerate * 100), 100));

    check_hresult(spOutType->CopyAllItems(spInType.get()));
    check_hresult(spInType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spInType->SetGUID(MF_MT_SUBTYPE, inVideoFormat));
    check_hresult(spInType->DeleteItem(MF_MT_AVG_BITRATE));

    // Create the sample grabber sink.
    check_hresult(SampleGrabberCB::CreateInstance(width, height, spCallback.put()));
    spCallback->InitFFrtp(width, height);
    check_hresult(MFCreateSampleGrabberSinkActivate(spOutType.get(), spCallback.get(), spSinkActivate.put()));
    // To run as fast as possible, set this attribute (requires Windows 7):
    check_hresult(spSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE));
    check_hresult(spSinkActivate->ActivateObject(__uuidof(IMFMediaSink), spSink.put_void()));

    check_hresult(MFCreateSinkWriterFromMediaSink(spSink.get(), nullptr, spSinkWriter.put()));

    check_hresult(spSinkWriter->SetInputMediaType(0, spInType.get(), nullptr));
    check_hresult(spSinkWriter->BeginWriting());

    //winrt::slim_mutex m;
    //m.lock();

    //evt.add([&](IMFSample* pSample)
    //    {
    //        check_hresult(spSinkWriter->WriteSample(0, pSample));
    //        if (!pSample)
    //        {
    //            //m.unlock();
    //        }
    //    });
    //ffrtp(width, height, spCallback->m_evt);
    //auto rtp = ffrtp(width, height);
    //m.lock();
    return spSinkWriter.detach();
}