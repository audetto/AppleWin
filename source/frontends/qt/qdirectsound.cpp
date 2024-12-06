#include "qdirectsound.h"

#include "loggingcategory.h"
#include "windows.h"
#include <unordered_map>
#include <memory>

#include <QAudioOutput>

namespace
{
    qint64 defaultDuration = 0;

    class DirectSoundGenerator : public SoundBuffer, public QIODevice
    {
    public:
        DirectSoundGenerator();
        virtual ~DirectSoundGenerator() override;

        virtual HRESULT Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName) override;
        virtual HRESULT Release() override;
        virtual HRESULT Stop() override;
        virtual HRESULT Play( DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags ) override;
        virtual HRESULT SetVolume( LONG lVolume ) override;

        void setOptions(const qint64 duration);  // in ms
        QDirectSound::SoundInfo getInfo();

    protected:
        virtual qint64 readData(char *data, qint64 maxlen) override;
        virtual qint64 writeData(const char *data, qint64 len) override;

    private:
        std::shared_ptr<QAudioOutput> myAudioOutput;
    };

    std::unordered_map<SoundBuffer *, std::shared_ptr<DirectSoundGenerator> > activeSoundGenerators;

    DirectSoundGenerator::DirectSoundGenerator()
    : SoundBuffer()
    {
    }

    HRESULT DirectSoundGenerator::Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName)
    {
        // only initialise here to skip all the buffers which are not in DSBSTATUS_PLAYING mode
        QAudioFormat audioFormat;
        audioFormat.setSampleRate(nSampleRate);
        audioFormat.setChannelCount(nChannels);
        audioFormat.setSampleSize(16);
        audioFormat.setCodec(QString::fromUtf8("audio/pcm"));
        audioFormat.setByteOrder(QAudioFormat::LittleEndian);
        audioFormat.setSampleType(QAudioFormat::SignedInt);

        myAudioOutput = std::make_shared<QAudioOutput>(audioFormat);
        return SoundBuffer::Init(dwFlags, dwBufferSize, nSampleRate, nChannels, pDevName);
    }

    DirectSoundGenerator::~DirectSoundGenerator()
    {
        myAudioOutput->stop();
    }

    HRESULT DirectSoundGenerator::Release()
    {
        activeSoundGenerators.erase(this);
        return DS_OK;
    }

    void DirectSoundGenerator::setOptions(const qint64 duration)  // in ms
    {
        const qint64 buffer = myAudioOutput->format().bytesForDuration(duration * 1000);
        if (buffer == myAudioOutput->bufferSize())
        {
            return;
        }

        const bool running = QIODevice::isOpen();
        if (running)
        {
            myAudioOutput->stop();
        }

        myAudioOutput->setBufferSize(buffer);

        if (running)
        {
            myAudioOutput->start(this);
        }
    }

    HRESULT DirectSoundGenerator::SetVolume( LONG lVolume )
    {
        const HRESULT res = SoundBuffer::SetVolume(lVolume);
        const qreal logVolume = GetLogarithmicVolume();
        const qreal linVolume = QAudio::convertVolume(logVolume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
        myAudioOutput->setVolume(linVolume);
        return res;
    }

    HRESULT DirectSoundGenerator::Stop()
    {
        const HRESULT res = SoundBuffer::Stop();
        myAudioOutput->stop();
        QIODevice::close();
        return res;
    }

    HRESULT DirectSoundGenerator::Play( DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags )
    {
        const HRESULT res = SoundBuffer::Play(dwReserved1, dwReserved2, dwFlags);
        QIODevice::open(ReadOnly);
        myAudioOutput->start(this);
        return res;
    }

    qint64 DirectSoundGenerator::readData(char *data, qint64 maxlen)
    {
        LPVOID lpvAudioPtr1, lpvAudioPtr2;
        DWORD dwAudioBytes1, dwAudioBytes2;

        const size_t bytesRead = Read(maxlen, &lpvAudioPtr1, &dwAudioBytes1, &lpvAudioPtr2, &dwAudioBytes2);

        char * dest = data;
        if (lpvAudioPtr1 && dwAudioBytes1)
        {
            memcpy(dest, lpvAudioPtr1, dwAudioBytes1);
            dest += dwAudioBytes1;
        }
        if (lpvAudioPtr2 && dwAudioBytes2)
        {
            memcpy(dest, lpvAudioPtr2, dwAudioBytes2);
            dest += dwAudioBytes2;
        }

        return bytesRead;
    }

    QDirectSound::SoundInfo DirectSoundGenerator::getInfo()
    {
        QDirectSound::SoundInfo info;
        info.running = QIODevice::isOpen();
        info.channels = myChannels;
        info.numberOfUnderruns = GetBufferUnderruns();

        if (info.running)
        {
            const DWORD bytesInBuffer = GetBytesInBuffer();
            const auto & format = myAudioOutput->format();
            info.buffer = format.durationForBytes(bytesInBuffer) / 1000;
            info.size = format.durationForBytes(myBufferSize) / 1000;
        }

        return info;
    }

    qint64 DirectSoundGenerator::writeData(const char *data, qint64 len)
    {
        // cannot write
        return 0;
    }

}

static SoundBufferBase* CreateSoundBuffer(void)
{
    try
    {
        DirectSoundGenerator * generator = new DirectSoundGenerator();
        activeSoundGenerators[generator].reset(generator);
        return generator;
    }
    catch (const std::exception & e)
    {
        qDebug(appleAudio) << "SoundBuffer: " << e.what();
        return nullptr;
    }
}

extern bool g_bDSAvailable;

bool DSInit()
{
    SoundBufferBase::Create = CreateSoundBuffer;
    g_bDSAvailable = true;
    return true;
}

void DSUninit()
{

}

namespace QDirectSound
{

    void setOptions(const qint64 duration)
    {
        // this is necessary for the first initialisation
        // which happens before any buffer is created
        defaultDuration = duration;
        for (const auto & it : activeSoundGenerators)
        {
            const auto & generator = it.second;
            generator->setOptions(duration);
        }
    }

    std::vector<SoundInfo> getAudioInfo()
    {
        std::vector<SoundInfo> info;
        info.reserve(activeSoundGenerators.size());

        for (const auto & it : activeSoundGenerators)
        {
            const auto & generator = it.second;
            info.push_back(generator->getInfo());
        }

        return info;
    }

}
