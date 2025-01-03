#ifndef QTFRAME_H
#define QTFRAME_H

#include "linux/linuxframe.h"
#include <memory>
#include <QByteArray>
#include <QString>

class Emulator;
class QMdiSubWindow;

class QtFrame : public LinuxFrame
{
public:
    QtFrame(Emulator * emulator, QMdiSubWindow * window);

    void VideoPresentScreen() override;
    void FrameRefreshStatus(int drawflags) override;
    void Initialize(bool resetVideoState) override;
    void Destroy() override;

    int FrameMessageBox(LPCSTR lpText, LPCSTR lpCaption, UINT uType) override;
    void GetBitmap(WORD id, LONG cb, LPVOID lpvBits) override;
    BYTE* GetResource(WORD id, LPCSTR lpType, DWORD expectedSize) override;
    std::string Video_GetScreenShotFolder() const override;

    std::shared_ptr<SoundBuffer> CreateSoundBuffer(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pStreamName) override;

    void SetForceRepaint(const bool force);
    void SetZoom(const int x);
    void Set43Ratio();
    bool saveScreen(const QString & filename) const;

private:
    Emulator * myEmulator;
    QMdiSubWindow * myWindow;
    bool myForceRepaint;

    QByteArray myResource;
};

#endif // QTFRAME_H
