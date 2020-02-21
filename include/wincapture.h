#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <winrt\base.h>
#include <Shlwapi.h>
#include <winrt\Windows.Foundation.Collections.h>
#include <winrt\Windows.Devices.Enumeration.h>
#include <winrt\Windows.System.Threading.h>
namespace ros_win_camera
{
    class WindowsMFCapture : private IMFSourceReaderCallback
    {
    public:
        static winrt::Windows::Foundation::Collections::IVectorView<winrt::hstring> EnumerateCameraLinks(bool bEnumerateSensorCamera);

        WindowsMFCapture(bool isDevice, const winrt::hstring& link, bool isController = true);

        bool ChangeCaptureConfig(int32_t width, int32_t height, float frameRate, GUID preferredVideoSubType, bool bForceConversion = false);

        // IUnknown methods
        STDMETHODIMP QueryInterface(REFIID iid, void** ppv)
        {
            static const QITAB qit[] =
            {
                QITABENT(WindowsMFCapture, IMFSourceReaderCallback),
                { 0 },
            };
            return QISearch(this, qit, iid, ppv);
        }
        STDMETHODIMP_(ULONG) AddRef()
        {
            return InterlockedIncrement(&m_nRefCount);
        }
        STDMETHODIMP_(ULONG) Release()
        {
            ULONG uCount = InterlockedDecrement(&m_nRefCount);
            if (uCount == 0)
            {
                delete this;
            }
            return uCount;
        }
        winrt::event_token  AddSampleHandler(winrt::delegate<winrt::hresult_error, winrt::hstring, IMFSample*> handler);
        void RemoveSampleHandler(winrt::event_token token);
        void StartStreaming();
        void StopStreaming();
    private:
        // private destructor so that destruction is controlled by Release() as we inherit from IUnknown
        virtual ~WindowsMFCapture()
        {
        }

        void InitCaptureWithDevice(const winrt::hstring& cameraSymbolicLink);
        void InitCaptureWithUrl(const winrt::hstring& url);
        bool FindMatchingMediaType(IMFMediaType** ppMediaType, int32_t width = 0, int32_t height = 0, float frameRate = 0, GUID preferredVideoSubType = GUID_NULL);

        winrt::com_ptr<IMFMediaSource> spMediaSource;
        winrt::com_ptr <IMFSourceReader> spSourceReader;
        UINT32 m_u32Width, m_u32Height;
        UINT32 m_u32SourceReaderFlags;

        winrt::event<winrt::delegate<>> m_configEvent;
        winrt::Windows::Foundation::Collections::IVector<winrt::event_token> m_configEventTokenList;

        winrt::event<winrt::delegate<winrt::hresult_error, winrt::hstring, IMFSample*>> m_captureCallbackEvent;

        // IMFSourceReaderCallback methods
        STDMETHODIMP OnEvent(DWORD dwStreamIndex, IMFMediaEvent* mediaEvt)
        {
            MediaEventType evt;
            mediaEvt->GetType(&evt);
            
            return S_OK;
        }

        STDMETHODIMP OnFlush(DWORD)
        {
            EnterCriticalSection(&m_critsec);
            m_configEvent();
            for (auto token : m_configEventTokenList)
            {
                m_configEvent.remove(token);
            }
            LeaveCriticalSection(&m_critsec);
            return S_OK;
        }

        STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD dwStreamIndex,
            DWORD dwStreamFlags, LONGLONG llTimestamp, IMFSample* pSample);

        long                m_nRefCount;
        CRITICAL_SECTION    m_critsec;
        bool m_bStreamingStarted;
        bool m_bIsController;
        winrt::slim_mutex m_apiGuardMutex;
        winrt::slim_mutex m_sampleHandlerMutex;
    };
}