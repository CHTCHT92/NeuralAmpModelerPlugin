#pragma once

#include <atomic>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "../AudioDSPTools/dsp/ImpulseResponse.h"
#include "LongImpulseResponse.h"
#include "../AudioDSPTools/dsp/NoiseGate.h"
#include "../AudioDSPTools/dsp/dsp.h"
#include "../AudioDSPTools/dsp/wav.h"
#include "../AudioDSPTools/dsp/ResamplingContainer/ResamplingContainer.h"
#include "../NeuralAmpModelerCore/NAM/dsp.h"
#include "../NeuralAmpModelerCore/NAM/slimmable.h"

#include "Colors.h"
#include "ToneStack.h"

// ===== NOUVEAUX INCLUDES =====
#include "Drive.h"
#include "AnalogDelay.h"
#include "Reverb3.h"
// =============================

#include "IPlug_include_in_plug_hdr.h"
#include "ISender.h"


const int kNumPresets = 1;
constexpr size_t kNumChannelsInternal = 1;

class NAMSender : public iplug::IPeakAvgSender<>
{
public:
  NAMSender()
  : iplug::IPeakAvgSender<>(-90.0, true, 5.0f, 1.0f, 300.0f, 500.0f)
  {
  }
};

enum EParams
{
  kInputLevel = 0,
  kNoiseGateThreshold,
  kToneBass,
  kToneMid,
  kToneTreble,
  kOutputLevel,
  // ===== NOUVEAUX PARAMS =====
  kDriveActive,
  kDriveGain,
  kDriveVolume,
  kDelayActive,
  kDelayTime,
  kDelayFeedback,
  kDelayMix,
  kReverbActive,
  kReverbTime,
  kReverbType,
  kReverbMix,
  // ===========================
  kNoiseGateActive,
  kEQActive,
  kIRToggle,
  kCalibrateInput,
  kInputCalibrationLevel,
  kOutputMode,
  kSlim,
  kModelIndex,
  kIRIndex,
  kBankIndex,
  kIRBankIndex,
  kNumParams
};

const int kMaxModels = 32;
const int kMaxIRs = 16;
const int kMaxBanks = 16;

const int numKnobs = 6;

enum ECtrlTags
{
  kCtrlTagModelFileBrowser = 0,
  kCtrlTagIRFileBrowser,
  kCtrlTagInputMeter,
  kCtrlTagOutputMeter,
  kCtrlTagSettingsBox,
  kCtrlTagOutputMode,
  kCtrlTagCalibrateInput,
  kCtrlTagInputCalibrationLevel,
  kCtrlTagSlimmableIcon,
  kCtrlTagSlimOverlayBackdrop,
  kCtrlTagSlimKnob,
  kNumCtrlTags
};

enum EMsgTags
{
  kMsgTagClearModel = 0,
  kMsgTagClearIR,
  kMsgTagHighlightColor,
  kMsgTagLoadFailed,
  kMsgTagLoadedModel,
  kMsgTagLoadedIR,
  kNumMsgTags
};

double GetNAMSampleRate(const std::unique_ptr<nam::DSP>& model)
{
  const double assumedSampleRate = 48000.0;
  const double reportedEncapsulatedSampleRate = model->GetExpectedSampleRate();
  const double encapsulatedSampleRate =
    reportedEncapsulatedSampleRate <= 0.0 ? assumedSampleRate : reportedEncapsulatedSampleRate;
  return encapsulatedSampleRate;
};

class ResamplingNAM : public nam::DSP
{
public:
  ResamplingNAM(std::unique_ptr<nam::DSP> encapsulated, const double expected_sample_rate)
  : nam::DSP(encapsulated->NumInputChannels(), encapsulated->NumOutputChannels(), expected_sample_rate)
  , mEncapsulated(std::move(encapsulated))
  , mResampler(GetNAMSampleRate(mEncapsulated))
  {
    auto ProcessBlockFunc = [&](NAM_SAMPLE** input, NAM_SAMPLE** output, int numFrames) {
      mEncapsulated->process(input, output, numFrames);
    };
    mBlockProcessFunc = ProcessBlockFunc;

    if (mEncapsulated->HasLoudness())
      SetLoudness(mEncapsulated->GetLoudness());
    if (mEncapsulated->HasInputLevel())
      SetInputLevel(mEncapsulated->GetInputLevel());
    if (mEncapsulated->HasOutputLevel())
      SetOutputLevel(mEncapsulated->GetOutputLevel());

    int maxBlockSize = 2048;
    Reset(expected_sample_rate, maxBlockSize);
  };

  ~ResamplingNAM() = default;

  void prewarm() override { mEncapsulated->prewarm(); };

  void process(NAM_SAMPLE** input, NAM_SAMPLE** output, const int num_frames) override
  {
    if (num_frames > mMaxExternalBlockSize)
      throw std::runtime_error("More frames were provided than the max expected!");

    if (!NeedToResample())
      mEncapsulated->process(input, output, num_frames);
    else
      mResampler.ProcessBlock(input, output, num_frames, mBlockProcessFunc);
  };

  int GetLatency() const { return NeedToResample() ? mResampler.GetLatency() : 0; };

  void Reset(const double sampleRate, const int maxBlockSize) override
  {
    mExpectedSampleRate = sampleRate;
    mMaxExternalBlockSize = maxBlockSize;
    mResampler.Reset(sampleRate, maxBlockSize);

    const double mUpRatio = sampleRate / GetEncapsulatedSampleRate();
    const auto maxEncapsulatedBlockSize = static_cast<int>(std::ceil(static_cast<double>(maxBlockSize) / mUpRatio));
    mEncapsulated->ResetAndPrewarm(sampleRate, maxEncapsulatedBlockSize);
  };

  double GetEncapsulatedSampleRate() const { return GetNAMSampleRate(mEncapsulated); };

  nam::SlimmableModel* GetSlimmableModel() { return dynamic_cast<nam::SlimmableModel*>(mEncapsulated.get()); }
  const nam::SlimmableModel* GetSlimmableModel() const
  {
    return dynamic_cast<const nam::SlimmableModel*>(mEncapsulated.get());
  }

private:
  bool NeedToResample() const { return GetExpectedSampleRate() != GetEncapsulatedSampleRate(); };
  std::unique_ptr<nam::DSP> mEncapsulated;
  dsp::ResamplingContainer<NAM_SAMPLE, 1, 12> mResampler;
  int mMaxExternalBlockSize = 0;
  std::function<void(NAM_SAMPLE**, NAM_SAMPLE**, int)> mBlockProcessFunc;
};

class NeuralAmpModeler final : public iplug::Plugin
{
public:
  NeuralAmpModeler(const iplug::InstanceInfo& info);
  ~NeuralAmpModeler();

  void ProcessBlock(iplug::sample** inputs, iplug::sample** outputs, int nFrames) override;
  void OnReset() override;
  void OnIdle() override;

  bool SerializeState(iplug::IByteChunk& chunk) const override;
  int UnserializeState(const iplug::IByteChunk& chunk, int startPos) override;
  void OnUIOpen() override;
  bool OnHostRequestingSupportedViewConfiguration(int width, int height) override { return true; }

  void OnParamChange(int paramIdx) override;
  void OnParamChangeUI(int paramIdx, iplug::EParamSource source) override;
  bool OnMessage(int msgTag, int ctrlTag, int dataSize, const void* pData) override;

private:
  void _AllocateIOPointers(const size_t nChans);
  void _ApplyDSPStaging();
  void _DeallocateIOPointers();
  void _FallbackDSP(iplug::sample** inputs, iplug::sample** outputs, const size_t numChannels, const size_t numFrames);
  size_t _GetBufferNumChannels() const;
  size_t _GetBufferNumFrames() const;
  void _InitToneStack();
  std::string _StageModel(const WDL_String& dspFile);
  dsp::wav::LoadReturnCode _StageIR(const WDL_String& irPath);

  bool _HaveModel() const { return this->mModel != nullptr; };
  void _PrepareBuffers(const size_t numChannels, const size_t numFrames);
  void _PrepareIOPointers(const size_t nChans);
  void _ProcessInput(iplug::sample** inputs, const size_t nFrames, const size_t nChansIn, const size_t nChansOut);
  void _ProcessOutput(iplug::sample** inputs, iplug::sample** outputs, const size_t nFrames, const size_t nChansIn,
                      const size_t nChansOut);
  void _ResetModelAndIR(const double sampleRate, const int maxBlockSize);

  void _SetInputGain();
  void _SetOutputGain();
  void _ApplySlimParamToLoadedNAMs();

  // Model list
  void _ScanModelFolder(const std::filesystem::path& folder);
  void _SetModelFolderFromFile(const WDL_String& filePath, bool notifyHost);
  void _ModelWorkerLoop();
  std::string _StageModelInBackground(const std::filesystem::path& path);

  // IR list
  void _ScanIRFolder(const std::filesystem::path& folder);
  void _SetIRFolderFromFile(const WDL_String& filePath, bool notifyHost);
  void _IRWorkerLoop();
  std::string _StageIRInBackground(const std::filesystem::path& path);

  // Banks
  void _RefreshModelBanks(const std::filesystem::path& currentFolder);
  void _RefreshIRBanks(const std::filesystem::path& currentFolder);
  bool _ApplyModelBank(int idx);
  bool _ApplyIRBank(int idx);

  // Unserialization
  void _UnserializeApplyConfig(nlohmann::json& config);
  int _UnserializeStateWithKnownVersion(const iplug::IByteChunk& chunk, int startPos);
  int _UnserializeStateWithUnknownVersion(const iplug::IByteChunk& chunk, int startPos);

  void _UpdateControlsFromModel();
  void _UpdateLatency();
  void _UpdateMeters(iplug::sample** inputPointer, iplug::sample** outputPointer, const size_t nFrames,
                     const size_t nChansIn, const size_t nChansOut);

  // ===== BUFFERS EXISTANTS =====
  std::vector<std::vector<iplug::sample>> mInputArray;
  std::vector<std::vector<iplug::sample>> mOutputArray;
  iplug::sample** mInputPointers = nullptr;
  iplug::sample** mOutputPointers = nullptr;

  // ===== NOUVEAUX BUFFERS =====
  // Drive (mono)
  std::vector<std::vector<iplug::sample>> mDriveArray;
  iplug::sample** mDrivePointers = nullptr;

  // Stéréo (delay + reverb)
  std::vector<std::vector<iplug::sample>> mStereoArray;   // 2 canaux : entrée du delay
  iplug::sample** mStereoPointers = nullptr;

  std::vector<std::vector<iplug::sample>> mDelayArray;    // 2 canaux : sortie du delay
  iplug::sample** mDelayPointers = nullptr;

  std::vector<std::vector<iplug::sample>> mReverbArray;   // 2 canaux : sortie de la reverb
  iplug::sample** mReverbPointers = nullptr;
  // ============================

  double mInputGain = 1.0;
  double mOutputGain = 1.0;

  dsp::noise_gate::Trigger mNoiseGateTrigger;
  dsp::noise_gate::Gain mNoiseGateGain;

  std::unique_ptr<ResamplingNAM> mModel;
  std::unique_ptr<LongImpulseResponse> mIR;
  std::unique_ptr<ResamplingNAM> mStagedModel;
  std::unique_ptr<LongImpulseResponse> mStagedIR;
  std::atomic<bool> mShouldRemoveModel = false;
  std::atomic<bool> mShouldRemoveIR = false;

  std::atomic<bool> mNewModelLoadedInDSP = false;
  std::atomic<bool> mModelCleared = false;

  std::unique_ptr<dsp::tone_stack::AbstractToneStack> mToneStack;

  recursive_linear_filter::HighPass mHighPass;

  // ===== NOUVEAUX DSP =====
  dsp::drive::TubeScreamer mDrive;
  dsp::delay::AnalogDelay mDelay;
  dsp::reverb::Reverb3 mReverb;
  // ========================

  WDL_String mNAMPath;
  WDL_String mIRPath;

  WDL_String mHighLightColor{PluginColors::NAM_THEMECOLOR.ToColorCode()};

  std::unordered_map<std::string, double> mNAMParams = {{"Input", 0.0}, {"Output", 0.0}};

  NAMSender mInputSender, mOutputSender;

  // Model list state
  std::vector<std::filesystem::path> mModelFiles;
  std::vector<std::string> mModelNames;
  mutable std::mutex mModelListMutex;
  std::atomic<int> mWantedModelIndex{-1};
  std::atomic<int> mLoadedModelIndex{-1};
  std::atomic<bool> mModelWorkerQuit{false};
  std::atomic<bool> mModelPathNeedsUIRefresh{false};
  std::thread mModelWorker;

  // IR list state
  std::vector<std::filesystem::path> mIRFiles;
  std::vector<std::string> mIRNames;
  mutable std::mutex mIRListMutex;
  std::atomic<int> mWantedIRIndex{-1};
  std::atomic<int> mLoadedIRIndex{-1};
  std::atomic<bool> mIRWorkerQuit{false};
  std::atomic<bool> mIRPathNeedsUIRefresh{false};
  std::thread mIRWorker;

  // Bank state
  std::vector<std::filesystem::path> mBankFolders;
  std::vector<std::string> mBankNames;
  std::atomic<int> mCurrentBankIndex{-1};
  std::atomic<int> mWantedBankIndex{-1};
  std::vector<std::filesystem::path> mIRBankFolders;
  std::vector<std::string> mIRBankNames;
  std::atomic<int> mCurrentIRBankIndex{-1};
  std::atomic<int> mWantedIRBankIndex{-1};
};
