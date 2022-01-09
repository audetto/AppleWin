//
//  EmulatorViewController.mm
//  Mariani
//
//  Copyright © 2021 Apple Inc.
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//

#import "EmulatorViewController.h"
#import <MetalKit/MetalKit.h>
#import <AVFoundation/AVFoundation.h>

#import "windows.h"

#import "context.h"

#import <SDL.h>
#import "benchmark.h"
#import "fileregistry.h"
#import "gamepad.h"
#import "programoptions.h"
#import "sdirectsound.h"
#import "MarianiFrame.h"
#import "utils.h"

#import "CommonTypes.h"
#import "MarianiFrame.h"
#import "EmulatorRenderer.h"
#import "UserDefaults.h"

#define SCREEN_RECORDING_FILE_NAME  NSLocalizedString(@"Mariani Recording", @"default name for new screen recording")
#define SCREENSHOT_FILE_NAME        NSLocalizedString(@"Mariani Screen Shot", @"default name for new screenshot")

@interface EmulatorViewController ()

@property (strong) EmulatorRenderer *renderer;

@property LoggerContext *logger;
@property RegistryContext *registryContext;
@property Initialisation *initialisation;

@property NSDate *samplePeriodBeginClockTime;
@property uint64_t samplePeriodBeginCumulativeCycles;
@property NSInteger frameCount;
@property NSTimer *timer;

@property AVAssetWriter *videoWriter;
@property AVAssetWriterInput *videoWriterInput;
@property AVAssetWriterInputPixelBufferAdaptor *videoWriterAdaptor;
@property int64_t videoWriterFrameNumber;
@property (getter=isRecordingScreen) BOOL recordingScreen;

@end

@implementation EmulatorViewController {
    MTKView *_view;
    FrameBuffer frameBuffer;
}

std::shared_ptr<sa2::SDLFrame> frame;

- (void)awakeFromNib {
    [super awakeFromNib];
    
    const Uint32 flags = SDL_INIT_GAMECONTROLLER | SDL_INIT_AUDIO;
    if (SDL_Init(flags) != 0) {
        NSLog(@"SDL_Init error %s", SDL_GetError());
    }
    
    common2::EmulatorOptions options;
    self.logger = new LoggerContext(options.log);
    self.registryContext = new RegistryContext(CreateFileRegistry(options));

    frame.reset(new mariani::MarianiFrame(options));

    std::shared_ptr<Paddle> paddle(new sa2::Gamepad(0));
    self.initialisation = new Initialisation(frame, paddle);
    applyOptions(options);
    frame->Begin();
}

- (void)viewDidLoad {
    [super viewDidLoad];

    _view = (MTKView *)self.view;
    _view.enableSetNeedsDisplay = NO;
    _view.device = MTLCreateSystemDefaultDevice();
#ifdef DEBUG
    //  useful for debugging quad sizing issues.
    _view.clearColor = MTLClearColorMake(0.0, 0.15, 0.3, 0.3);
#endif
    
    Video &video = GetVideo();
    frameBuffer.borderWidth = video.GetFrameBufferBorderWidth();
    frameBuffer.borderHeight = video.GetFrameBufferBorderHeight();
    frameBuffer.bufferWidth = video.GetFrameBufferWidth();
    frameBuffer.bufferHeight = video.GetFrameBufferHeight();
    frameBuffer.pixelWidth = video.GetFrameBufferBorderlessWidth();
    frameBuffer.pixelHeight = video.GetFrameBufferBorderlessHeight();
    
    _renderer = [[EmulatorRenderer alloc] initWithMetalKitView:_view frameBuffer:&frameBuffer];
    if (self.renderer == nil) {
        NSLog(@"Renderer initialization failed");
        return;
    }
    self.renderer.delegate = self;
    
    // Initialize the renderer with the view size.
    [self.renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];
    _view.delegate = self.renderer;
    
    [self.renderer createTexture];
}

- (void)start {
    // reset the effective CPU clock speed meters
    self.samplePeriodBeginClockTime = [NSDate now];
    self.samplePeriodBeginCumulativeCycles = g_nCumulativeCycles;
    self.frameCount = 0;
    
    self.timer = [NSTimer scheduledTimerWithTimeInterval:1.0 / TARGET_FPS target:self selector:@selector(timerFired) userInfo:nil repeats:YES];
}

- (void)timerFired {
#ifdef DEBUG
    NSDate *start = [NSDate now];
#endif
    
    if (self.delegate == nil || [self.delegate shouldPlayAudio]) {
        sa2::writeAudio();
    }
#ifdef DEBUG
    NSTimeInterval audioWriteTimeOffset = -[start timeIntervalSinceNow];
#endif

    bool quit = false;
    frame->ProcessEvents(quit);
    if (quit) {
        [self.delegate terminateWithReason:@"requested by frame"];
    }
#ifdef DEBUG
    NSTimeInterval eventProcessingTimeOffset = -[start timeIntervalSinceNow];
#endif

    frame->ExecuteOneFrame(1000.0 / TARGET_FPS);
#ifdef DEBUG
    NSTimeInterval executionTimeOffset = -[start timeIntervalSinceNow];
#endif

    frame->VideoPresentScreen();

    frameBuffer.data = frame->FrameBufferData();
    [self.renderer updateTextureWithData:frameBuffer.data];
#ifdef DEBUG
    NSTimeInterval screenPresentationTimeOffset = -[start timeIntervalSinceNow];
#endif

    if (self.videoWriterInput.readyForMoreMediaData) {
        if (!self.isRecordingScreen) {
            self.recordingScreen = YES;
        }
        
        // make a CVPixelBuffer and point the frame buffer to it
        CVPixelBufferRef pixelBuffer = NULL;
        CVReturn status = CVPixelBufferCreateWithBytes(kCFAllocatorDefault,
                                                       frameBuffer.bufferWidth,
                                                       frameBuffer.bufferHeight,
                                                       kCVPixelFormatType_32BGRA,
                                                       frameBuffer.data,
                                                       frameBuffer.bufferWidth * 4,
                                                       NULL,
                                                       NULL,
                                                       NULL,
                                                       &pixelBuffer);
        if (status == kCVReturnSuccess && pixelBuffer != NULL) {
            // append the CVPixelBuffer into the output stream
            [self.videoWriterAdaptor appendPixelBuffer:pixelBuffer
                                  withPresentationTime:CMTimeMake(self.videoWriterFrameNumber * (CMTIME_BASE / TARGET_FPS), CMTIME_BASE)];
            CVPixelBufferRelease(pixelBuffer);
            
            // if we realize that we've skipped a frame (i.e., videoWriter is
            // not nil but readyForMoreMediaData is false) should we also
            // increment videoWriterFrameNumber?
            self.videoWriterFrameNumber++;
        }
    }
#ifdef DEBUG
    else if (self.videoWriter != nil) {
        NSLog(@"Screen Recording: skipped a frame");
    }
#endif
    if (self.isRecordingScreen) {
        // blink the screen recording button
        if (self.videoWriterFrameNumber % TARGET_FPS == 0) {
            [self.delegate screenRecordingDidTick];
        }
        else if (self.videoWriterFrameNumber % TARGET_FPS == TARGET_FPS / 2) {
            [self.delegate screenRecordingDidTock];
        }
    }
#ifdef DEBUG
    NSTimeInterval screenRecordingTimeOffset = -[start timeIntervalSinceNow];
#endif

    if (++self.frameCount > TARGET_FPS) {
        NSArray *cpus = @[ @"", @"6502", @"65C02", @"Z80" ];
        double clockSpeed =
            (double)(g_nCumulativeCycles - self.samplePeriodBeginCumulativeCycles) /
            -[self.samplePeriodBeginClockTime timeIntervalSinceNow];
        [self.delegate updateStatus:[NSString stringWithFormat:@"%@@%.3f MHz", cpus[GetActiveCpu()], clockSpeed / 1000000]];

        self.samplePeriodBeginClockTime = [NSDate now];
        self.samplePeriodBeginCumulativeCycles = g_nCumulativeCycles;
        self.frameCount = 0;
    }
    
#ifdef DEBUG
    NSTimeInterval duration = -[start timeIntervalSinceNow];
    if (duration > 1.0 / TARGET_FPS) {
        // oops, took too long
        NSLog(@"Frame time exceeded: %f ms", duration * 1000);
        NSLog(@"    Write audio:                %f ms", (audioWriteTimeOffset * 1000));
        NSLog(@"    Process events:             %f ms", (eventProcessingTimeOffset - audioWriteTimeOffset) * 1000);
        NSLog(@"    Execute:                    %f ms", (executionTimeOffset - eventProcessingTimeOffset) * 1000);
        NSLog(@"    Present screen:             %f ms", (screenPresentationTimeOffset - executionTimeOffset) * 1000);
        NSLog(@"    Record screen:              %f ms", (screenRecordingTimeOffset - screenPresentationTimeOffset) * 1000);
    }
#endif // DEBUG
}

- (void)pause {
    [self.timer invalidate];
}

- (void)reboot {
    frame->Restart();
}

- (void)reinitialize {
    frame->Destroy();
    frame->Initialize(true);
}

- (void)stop {
    [self pause];
    if (frame != NULL) {
        frame->End();
    }
    SDL_Quit();
}

- (void)displayTypeDidChange {
    [self.renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];
}

- (void)videoModeDidChange {
    frame->ApplyVideoModeChange();
    
    // we need to recompute the overscan when video mode is changed
    [self displayTypeDidChange];
}

- (BOOL)emulationHardwareChanged {
    return frame->HardwareChanged();
}

- (void)toggleScreenRecording {
    if (self.videoWriter == nil) {
        [self.delegate screenRecordingDidStart];
        NSURL *url = [self.delegate unusedURLForFilename:SCREEN_RECORDING_FILE_NAME
                                               extension:@"mov"
                                                inFolder:[[UserDefaults sharedInstance] recordingsFolder]];
        NSLog(@"Starting screen recording to %@", url);

        NSError *error = nil;
        self.videoWriter = [[AVAssetWriter alloc] initWithURL:url
                                                     fileType:AVFileTypeAppleM4V
                                                        error:&error];
        const NSInteger shouldOverscan = [self shouldOverscan];
        const NSInteger overscanWidth = shouldOverscan ? frameBuffer.borderWidth * OVERSCAN * 2 : 0;
        const NSInteger overscanHeight = shouldOverscan ? frameBuffer.borderHeight * OVERSCAN * 2 : 0;
        NSDictionary *videoSettings = @{
            AVVideoCodecKey: AVVideoCodecTypeHEVC,
            AVVideoWidthKey: @(frameBuffer.bufferWidth),
            AVVideoHeightKey: @(frameBuffer.bufferHeight),
            // just like the emulated display, overscan a little bit so that we
            // don't clip off portions of the pixels on the edge
            AVVideoCleanApertureKey: @{
                AVVideoCleanApertureWidthKey: @(frameBuffer.pixelWidth + overscanWidth),
                AVVideoCleanApertureHeightKey: @(frameBuffer.pixelHeight + overscanHeight),
                AVVideoCleanApertureHorizontalOffsetKey: @(0),
                AVVideoCleanApertureVerticalOffsetKey: @(0),
            },
        };
        self.videoWriterInput = [AVAssetWriterInput assetWriterInputWithMediaType:AVMediaTypeVideo
                                                                   outputSettings:videoSettings];
        self.videoWriterInput.transform = CGAffineTransformMakeScale(1, -1);
        self.videoWriterAdaptor =
            [AVAssetWriterInputPixelBufferAdaptor assetWriterInputPixelBufferAdaptorWithAssetWriterInput:self.videoWriterInput
                                                                             sourcePixelBufferAttributes:nil];
        [self.videoWriter addInput:self.videoWriterInput];
        [self.videoWriter startWriting];
        [self.videoWriter startSessionAtSourceTime:kCMTimeZero];
        self.videoWriterFrameNumber = 0;
    }
    else {
        // stop recording
        NSLog(@"Ending screen recording");
        self.recordingScreen = NO;
        [self.videoWriterInput markAsFinished];
        [self.videoWriter finishWritingWithCompletionHandler:^(void) {
            if (self.videoWriter.status != AVAssetWriterStatusCompleted) {
                NSLog(@"Failed to write screen recording: %@", self.videoWriter.error);
            }
            
            // clean up
            self.videoWriter = nil;
            self.videoWriterInput = nil;
            self.videoWriterAdaptor = nil;
            self.videoWriterFrameNumber = 0;
            
            self.recordingScreen = NO;

            NSLog(@"Ended screen recording");
            
            dispatch_async(dispatch_get_main_queue(), ^(void) {
                [self.delegate screenRecordingDidStop];
            });
        }];
    }
}

- (void)takeScreenshot {
#ifdef DEBUG
    NSDate *start = [NSDate now];
#endif
    
    // have the BMP screenshot written to a memory stream instead of a file...
    char *buffer;
    size_t bufferSize;
    FILE *memStream = open_memstream(&buffer, &bufferSize);
    GetVideo().Video_MakeScreenShot(memStream, Video::SCREENSHOT_560x384);
    fclose(memStream);

#ifdef DEBUG
    NSTimeInterval duration = -[start timeIntervalSinceNow];
    NSLog(@"Screenshot took: %f ms", duration * 1000);
#endif // DEBUG
    
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_LOW, 0), ^{
#ifdef DEBUG
        NSDate *start = [NSDate now];
#endif
        // ...and then convert it to PNG for saving
        NSImage *image = [[NSImage alloc] initWithData:[NSData dataWithBytes:buffer length:bufferSize]];
        CGImageRef cgRef = [image CGImageForProposedRect:NULL context:nil hints:nil];
        NSBitmapImageRep *newRep = [[NSBitmapImageRep alloc] initWithCGImage:cgRef];
        [newRep setSize:[image size]];
        NSData *pngData = [newRep representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
        NSURL *url = [self.delegate unusedURLForFilename:SCREENSHOT_FILE_NAME
                                               extension:@"png"
                                                inFolder:[[UserDefaults sharedInstance] screenshotsFolder]];
        [pngData writeToURL:url atomically:YES];
        free(buffer);
#ifdef DEBUG
        NSTimeInterval duration = -[start timeIntervalSinceNow];
        NSLog(@"Screenshot saved to %@, PNG conversion took %f ms", url, duration * 1000);
#endif // DEBUG
        [[NSSound soundNamed:@"Blow"] play];
    });
}

#pragma mark - EmulatorRendererProtocol

- (BOOL)shouldOverscan {
    // the idealized display seems to show weird artifacts in the overscan
    // area, so we crop tightly
    Video &video = GetVideo();
    return (video.GetVideoType() != VT_COLOR_IDEALIZED);
}

@end