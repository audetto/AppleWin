#include "StdAfx.h"

#include "qdirectsound.h"

#include "loggingcategory.h"
#include "linux/linuxsoundbuffer.h"
#include <unordered_set>
#include <memory>

#include <QIODevice>
#include <QAudioFormat>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
#include <QAudioOutput>
typedef QAudioOutput QAudioSink;
#else
#include <QAudioSink>
#endif

namespace
{
    qint64 defaultDuration = 0;

    class DirectSoundGenerator : public LinuxSoundBuffer, public QIODevice
    {
    public:
        DirectSoundGenerator(DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pStreamName);
        virtual ~DirectSoundGenerator() override;

        virtual HRESULT Stop() override;
        virtual HRESULT Play( DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags ) override;
        virtual HRESULT SetVolume( LONG lVolume ) override;

        void setOptions(const qint64 duration);  // in ms
        QDirectSound::SoundInfo getInfo();

    protected:
        virtual qint64 readData(char *data, qint64 maxlen) override;
        virtual qint64 writeData(const char *data, qint64 len) override;

    private:
        std::shared_ptr<QAudioSink> myAudioOutput;
    };

    std::unordered_set<DirectSoundGenerator *> activeSoundGenerators;

    DirectSoundGenerator::DirectSoundGenerator(DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pStreamName) 
    : LinuxSoundBuffer(dwBufferSize, nSampleRate, nChannels, pStreamName)
    {
        // only initialise here to skip all the buffers which are not in DSBSTATUS_PLAYING mode
        QAudioFormat audioFormat;
        audioFormat.setSampleRate(mySampleRate);
        audioFormat.setChannelCount(myChannels);

        Q_ASSERT(myBitsPerSample == 16);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
        audioFormat.setSampleSize(myBitsPerSample);
        audioFormat.setCodec(QString::fromUtf8("audio/pcm"));
        audioFormat.setByteOrder(QAudioFormat::LittleEndian);
        audioFormat.setSampleType(QAudioFormat::SignedInt);
        myAudioOutput = std::make_shared<QAudioSink>(audioFormat);
#else
        audioFormat.setSampleFormat(QAudioFormat::Int16);
        myAudioOutput = std::make_shared<QAudioSink>(audioFormat);
#endif
    }

    DirectSoundGenerator::~DirectSoundGenerator()
    {
        activeSoundGenerators.erase(this);
        myAudioOutput->stop();
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
        const HRESULT res = LinuxSoundBuffer::SetVolume(lVolume);
        const qreal logVolume = GetLogarithmicVolume();
        const qreal linVolume = QAudio::convertVolume(logVolume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
        myAudioOutput->setVolume(linVolume);
        return res;
    }

    HRESULT DirectSoundGenerator::Stop()
    {
        const HRESULT res = LinuxSoundBuffer::Stop();
        myAudioOutput->stop();
        QIODevice::close();
        return res;
    }

    HRESULT DirectSoundGenerator::Play( DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags )
    {
        const HRESULT res = LinuxSoundBuffer::Play(dwReserved1, dwReserved2, dwFlags);
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
        info.streamName = myStreamName;
        info.running = QIODevice::isOpen();
        info.channels = myChannels;
        info.numberOfUnderruns = GetBufferUnderruns();

        if (info.running)
        {
            const uint32_t bytesInBuffer = GetBytesInBuffer();
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

namespace QDirectSound
{

    std::shared_ptr<SoundBuffer> iCreateDirectSoundBuffer(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pStreamName)
    {
        try
        {
            std::shared_ptr<DirectSoundGenerator> generator = std::make_shared<DirectSoundGenerator>(dwBufferSize, nSampleRate, nChannels, pStreamName);
            generator->setOptions(defaultDuration);
            DirectSoundGenerator * ptr = generator.get();
            activeSoundGenerators.insert(ptr);
            return generator;
        }
        catch (const std::exception & e)
        {
            qDebug(appleAudio) << "SoundBuffer: " << e.what();
            return nullptr;
        }
    }

    void setOptions(const qint64 duration)
    {
        // this is necessary for the first initialisation
        // which happens before any buffer is created
        defaultDuration = duration;
        for (const auto & it : activeSoundGenerators)
        {
            const auto & generator = it;
            generator->setOptions(duration);
        }
    }

    std::vector<SoundInfo> getAudioInfo()
    {
        std::vector<SoundInfo> info;
        info.reserve(activeSoundGenerators.size());

        for (const auto & it : activeSoundGenerators)
        {
            const auto & generator = it;
            info.push_back(generator->getInfo());
        }

        return info;
    }

}
