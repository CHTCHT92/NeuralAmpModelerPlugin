#include <algorithm> // std::clamp, std::min
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <system_error>
#include <cmath> // pow
#include <filesystem>
#include <iostream>
#include <utility>

#include "Colors.h"
#include "../NeuralAmpModelerCore/NAM/activations.h"
#include "../NeuralAmpModelerCore/NAM/get_dsp.h"
// clang-format off
// These includes need to happen in this order or else the latter won't know
// a bunch of stuff.
#include "NeuralAmpModeler.h"
#include "IPlug_include_in_plug_src.h"
// clang-format on
#include "architecture.hpp"

#include "NeuralAmpModelerControls.h"

using namespace iplug;
using namespace igraphics;

const double kDCBlockerFrequency = 5.0;

// Styles
const IVColorSpec colorSpec{
  DEFAULT_BGCOLOR, // Background
  PluginColors::NAM_THEMECOLOR, // Foreground
  PluginColors::NAM_THEMECOLOR.WithOpacity(0.3f), // Pressed
  PluginColors::NAM_THEMECOLOR.WithOpacity(0.4f), // Frame
  PluginColors::MOUSEOVER, // Highlight
  DEFAULT_SHCOLOR, // Shadow
  PluginColors::NAM_THEMECOLOR, // Extra 1
  COLOR_RED, // Extra 2 --> color for clipping in meters
  PluginColors::NAM_THEMECOLOR.WithContrast(0.1f), // Extra 3
};

const IVStyle style =
  IVStyle{true, // Show label
          true, // Show value
          colorSpec,
          {DEFAULT_TEXT_SIZE + 3.f, EVAlign::Middle, PluginColors::NAM_THEMEFONTCOLOR}, // Knob label text5
          {DEFAULT_TEXT_SIZE + 3.f, EVAlign::Bottom, PluginColors::NAM_THEMEFONTCOLOR}, // Knob value text
          DEFAULT_HIDE_CURSOR,
          DEFAULT_DRAW_FRAME,
          false,
          DEFAULT_EMBOSS,
          0.2f,
          2.f,
          DEFAULT_SHADOW_OFFSET,
          DEFAULT_WIDGET_FRAC,
          DEFAULT_WIDGET_ANGLE};
const IVStyle titleStyle =
  DEFAULT_STYLE.WithValueText(IText(30, COLOR_WHITE, "Michroma-Regular")).WithDrawFrame(false).WithShadowOffset(2.f);
const IVStyle radioButtonStyle =
  style
    .WithColor(EVColor::kON, PluginColors::NAM_THEMECOLOR) // Pressed buttons and their labels
    .WithColor(EVColor::kOFF, PluginColors::NAM_THEMECOLOR.WithOpacity(0.1f)) // Unpressed buttons
    .WithColor(EVColor::kX1, PluginColors::NAM_THEMECOLOR.WithOpacity(0.6f)); // Unpressed buttons' labels

EMsgBoxResult _ShowMessageBox(iplug::igraphics::IGraphics* pGraphics, const char* str, const char* caption,
                              EMsgBoxType type)
{
#ifdef OS_MAC
  // macOS is backwards?
  return pGraphics->ShowMessageBox(caption, str, type);
#else
  return pGraphics->ShowMessageBox(str, caption, type);
#endif
}

const std::string kCalibrateInputParamName = "CalibrateInput";
const bool kDefaultCalibrateInput = false;
const std::string kInputCalibrationLevelParamName = "InputCalibrationLevel";
const double kDefaultInputCalibrationLevel = 12.0;


NeuralAmpModeler::NeuralAmpModeler(const InstanceInfo& info)
: Plugin(info, MakeConfig(kNumParams, kNumPresets))
{
  _InitToneStack();
  nam::activations::Activation::enable_fast_tanh();
  GetParam(kInputLevel)->InitGain("Input", 0.0, -20.0, 20.0, 0.1);
  GetParam(kToneBass)->InitDouble("Bass", 5.0, 0.0, 10.0, 0.1);
  GetParam(kToneMid)->InitDouble("Middle", 5.0, 0.0, 10.0, 0.1);
  GetParam(kToneTreble)->InitDouble("Treble", 5.0, 0.0, 10.0, 0.1);
  GetParam(kOutputLevel)->InitGain("Output", 0.0, -40.0, 40.0, 0.1);
  GetParam(kNoiseGateThreshold)->InitGain("Threshold", -80.0, -100.0, 0.0, 0.1);
  GetParam(kNoiseGateActive)->InitBool("NoiseGateActive", true);
  GetParam(kEQActive)->InitBool("ToneStack", true);
  GetParam(kOutputMode)->InitEnum("OutputMode", 1, {"Raw", "Normalized", "Calibrated"}); // TODO DRY w/ control
  GetParam(kIRToggle)->InitBool("IRToggle", true);
  GetParam(kCalibrateInput)->InitBool(kCalibrateInputParamName.c_str(), kDefaultCalibrateInput);
  GetParam(kInputCalibrationLevel)
    ->InitDouble(kInputCalibrationLevelParamName.c_str(), kDefaultInputCalibrationLevel, -60.0, 60.0, 0.1, "dBu");
  GetParam(kSlim)->InitDouble("Slim", 0.0, 0.0, 1.0, 0.01);
  // Model list parameter: kMaxModels steps, displayed as the file name of the selected .nam
  GetParam(kModelIndex)->InitEnum("Model", 0, kMaxModels);
  GetParam(kModelIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    if (mModelNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mModelNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mModelNames[static_cast<size_t>(idx)].c_str());
  });
  // Cab list parameter: kMaxIRs steps, displayed as the file name of the selected IR
  GetParam(kIRIndex)->InitEnum("Cab", 0, kMaxIRs);
  GetParam(kIRIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    if (mIRNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mIRNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mIRNames[static_cast<size_t>(idx)].c_str());
  });
  // Bank parameters: kMaxBanks steps, displayed as the name of the selected folder
  GetParam(kBankIndex)->InitEnum("Bank", 0, kMaxBanks);
  GetParam(kBankIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    if (mBankNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mBankNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mBankNames[static_cast<size_t>(idx)].c_str());
  });
  GetParam(kIRBankIndex)->InitEnum("Cab Bank", 0, kMaxBanks);
  GetParam(kIRBankIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    if (mIRBankNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mIRBankNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mIRBankNames[static_cast<size_t>(idx)].c_str());
  });

  mNoiseGateTrigger.AddListener(&mNoiseGateGain);

  mMakeGraphicsFunc = [&]() {

#ifdef OS_IOS
    auto scaleFactor = GetScaleForScreen(PLUG_WIDTH, PLUG_HEIGHT) * 0.85f;
#else
    auto scaleFactor = 1.0f;
#endif

    return MakeGraphics(*this, PLUG_WIDTH, PLUG_HEIGHT, PLUG_FPS, scaleFactor);
  };

  mLayoutFunc = [&](IGraphics* pGraphics) {
    pGraphics->AttachCornerResizer(EUIResizerMode::Scale, false);
    pGraphics->AttachTextEntryControl();
    pGraphics->EnableMouseOver(true);
    pGraphics->EnableTooltips(true);
    pGraphics->EnableMultiTouch(true);

    pGraphics->LoadFont("Roboto-Regular", ROBOTO_FN);
    pGraphics->LoadFont("Michroma-Regular", MICHROMA_FN);

    const auto gearSVG = pGraphics->LoadSVG(GEAR_FN);
    const auto fileSVG = pGraphics->LoadSVG(FILE_FN);
    const auto globeSVG = pGraphics->LoadSVG(GLOBE_ICON_FN);
    const auto crossSVG = pGraphics->LoadSVG(CLOSE_BUTTON_FN);
    const auto rightArrowSVG = pGraphics->LoadSVG(RIGHT_ARROW_FN);
    const auto leftArrowSVG = pGraphics->LoadSVG(LEFT_ARROW_FN);
    const auto modelIconSVG = pGraphics->LoadSVG(MODEL_ICON_FN);
    const auto irIconOnSVG = pGraphics->LoadSVG(IR_ICON_ON_FN);
    const auto irIconOffSVG = pGraphics->LoadSVG(IR_ICON_OFF_FN);
    const auto slimIconSVG = pGraphics->LoadSVG(SLIMMABLE_ICON_FN);

    const auto backgroundBitmap = pGraphics->LoadBitmap(BACKGROUND_FN);
    const auto fileBackgroundBitmap = pGraphics->LoadBitmap(FILEBACKGROUND_FN);
    const auto inputLevelBackgroundBitmap = pGraphics->LoadBitmap(INPUTLEVELBACKGROUND_FN);
    const auto linesBitmap = pGraphics->LoadBitmap(LINES_FN);
    const auto knobBackgroundBitmap = pGraphics->LoadBitmap(KNOBBACKGROUND_FN);
    const auto switchHandleBitmap = pGraphics->LoadBitmap(SLIDESWITCHHANDLE_FN);
    const auto meterBackgroundBitmap = pGraphics->LoadBitmap(METERBACKGROUND_FN);

    const auto b = pGraphics->GetBounds();
    const auto mainArea = b.GetPadded(-20);
    const auto contentArea = mainArea.GetPadded(-10);
    const auto titleHeight = 50.0f;
    const auto titleArea = contentArea.GetFromTop(titleHeight);

    // Areas for knobs
    const auto knobsPad = 20.0f;
    const auto knobsExtraSpaceBelowTitle = 25.0f;
    const auto singleKnobPad = -2.0f;
    const auto knobsArea = contentArea.GetFromTop(NAM_KNOB_HEIGHT)
                             .GetReducedFromLeft(knobsPad)
                             .GetReducedFromRight(knobsPad)
                             .GetVShifted(titleHeight + knobsExtraSpaceBelowTitle);
    const auto inputKnobArea = knobsArea.GetGridCell(0, kInputLevel, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto noiseGateArea = knobsArea.GetGridCell(0, kNoiseGateThreshold, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto bassKnobArea = knobsArea.GetGridCell(0, kToneBass, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto midKnobArea = knobsArea.GetGridCell(0, kToneMid, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto trebleKnobArea = knobsArea.GetGridCell(0, kToneTreble, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto outputKnobArea = knobsArea.GetGridCell(0, kOutputLevel, 1, numKnobs).GetPadded(-singleKnobPad);

    const auto ngToggleArea =
      noiseGateArea.GetVShifted(noiseGateArea.H()).SubRectVertical(2, 0).GetReducedFromTop(10.0f);
    const auto eqToggleArea = midKnobArea.GetVShifted(midKnobArea.H()).SubRectVertical(2, 0).GetReducedFromTop(10.0f);

    // Areas for model and IR
    const auto fileWidth = 200.0f;
    const auto fileHeight = 30.0f;
    const auto irYOffset = 38.0f;
    const auto modelArea =
      contentArea.GetFromBottom((2.0f * fileHeight)).GetFromTop(fileHeight).GetMidHPadded(fileWidth).GetVShifted(-1);
    const auto slimIconArea =
      IRECT(modelArea.R + 6.f, modelArea.MH() - 14.f, modelArea.R + 6.f + 2.f * 28.f, modelArea.MH() + 14.f);
    const auto modelIconArea = modelArea.GetFromLeft(30).GetTranslated(-40, 10);
    const auto irArea = modelArea.GetVShifted(irYOffset);
    const auto irSwitchArea = irArea.GetFromLeft(30.0f).GetHShifted(-40.0f).GetScaledAboutCentre(0.6f);

    // Areas for meters
    const auto inputMeterArea = contentArea.GetFromLeft(30).GetHShifted(-20).GetMidVPadded(100).GetVShifted(-25);
    const auto outputMeterArea = contentArea.GetFromRight(30).GetHShifted(20).GetMidVPadded(100).GetVShifted(-25);

    // Misc Areas
    const auto settingsButtonArea = CornerButtonArea(b);

    // Model loader button
    auto loadModelCompletionHandler = [&](const WDL_String& fileName, const WDL_String& path) {
      if (fileName.GetLength())
      {
        // Sets mNAMPath and mStagedNAM
        const std::string msg = _StageModel(fileName);
        // TODO error messages like the IR loader.
        if (msg.empty())
        {
          // The folder of the picked file becomes the list driven by the Model parameter
          _SetModelFolderFromFile(fileName, true);
        }
        if (msg.size())
        {
          std::stringstream ss;
          ss << "Failed to load NAM model. Message:\n\n" << msg;
          _ShowMessageBox(GetUI(), ss.str().c_str(), "Failed to load model!", kMB_OK);
        }
        std::cout << "Loaded: " << fileName.Get() << std::endl;
      }
    };

    // IR loader button
    auto loadIRCompletionHandler = [&](const WDL_String& fileName, const WDL_String& path) {
      if (fileName.GetLength())
      {
        mIRPath = fileName;
        const dsp::wav::LoadReturnCode retCode = _StageIR(fileName);
        if (retCode != dsp::wav::LoadReturnCode::SUCCESS)
        {
          std::stringstream message;
          message << "Failed to load IR file " << fileName.Get() << ":\n";
          message << dsp::wav::GetMsgForLoadReturnCode(retCode);

          _ShowMessageBox(GetUI(), message.str().c_str(), "Failed to load IR!", kMB_OK);
        }
        else
        {
          // The folder of the picked IR becomes the list driven by the Cab parameter
          _SetIRFolderFromFile(fileName, true);
        }
      }
    };

    pGraphics->AttachBackground(BACKGROUND_FN);
    pGraphics->AttachControl(new IBitmapControl(b, linesBitmap));
    pGraphics->AttachControl(new IVLabelControl(titleArea, "NEURAL AMP MODELER", titleStyle));
    pGraphics->AttachControl(new ISVGControl(modelIconArea, modelIconSVG));

#ifdef NAM_PICK_DIRECTORY
    const std::string defaultNamFileString = "Select model directory...";
    const std::string defaultIRString = "Select IR directory...";
#else
    const std::string defaultNamFileString = "Select model...";
    const std::string defaultIRString = "Select IR...";
#endif
    // Getting started page listing additional resources
    const char* const getUrl = "https://www.neuralampmodeler.com/users#comp-marb84o5";
    pGraphics->AttachControl(
      new NAMFileBrowserControl(modelArea, kMsgTagClearModel, defaultNamFileString.c_str(), "nam",
                                loadModelCompletionHandler, style, fileSVG, crossSVG, leftArrowSVG, rightArrowSVG,
                                fileBackgroundBitmap, globeSVG, "Get NAM Models", getUrl),
      kCtrlTagModelFileBrowser);

    auto hideSlimOverlay = [](IControl* pCaller) {
      IGraphics* ui = pCaller->GetUI();
      if (auto* backdrop = ui->GetControlWithTag(kCtrlTagSlimOverlayBackdrop))
        backdrop->Hide(true);
      if (auto* knob = ui->GetControlWithTag(kCtrlTagSlimKnob))
        knob->Hide(true);
      ui->SetAllControlsDirty();
    };
    auto showSlimOverlay = [](IControl* pCaller) {
      IGraphics* ui = pCaller->GetUI();
      if (auto* backdrop = ui->GetControlWithTag(kCtrlTagSlimOverlayBackdrop))
        backdrop->Hide(false);
      if (auto* knob = ui->GetControlWithTag(kCtrlTagSlimKnob))
        knob->Hide(false);
      ui->SetAllControlsDirty();
    };

    pGraphics
      ->AttachControl(
        new NAMSquareButtonControl(slimIconArea, DefaultClickActionFunc, slimIconSVG), kCtrlTagSlimmableIcon)
      ->SetAnimationEndActionFunction(showSlimOverlay)
      ->Hide(true);

    pGraphics->AttachControl(new ISVGSwitchControl(irSwitchArea, {irIconOffSVG, irIconOnSVG}, kIRToggle));
    pGraphics->AttachControl(
      new NAMFileBrowserControl(irArea, kMsgTagClearIR, defaultIRString.c_str(), "wav", loadIRCompletionHandler, style,
                                fileSVG, crossSVG, leftArrowSVG, rightArrowSVG, fileBackgroundBitmap, globeSVG,
                                "Get IRs", getUrl),
      kCtrlTagIRFileBrowser);
    pGraphics->AttachControl(
      new NAMSwitchControl(ngToggleArea, kNoiseGateActive, "Noise Gate", style, switchHandleBitmap));
    pGraphics->AttachControl(new NAMSwitchControl(eqToggleArea, kEQActive, "EQ", style, switchHandleBitmap));

    // The knobs
    pGraphics->AttachControl(new NAMKnobControl(inputKnobArea, kInputLevel, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(noiseGateArea, kNoiseGateThreshold, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(
      new NAMKnobControl(bassKnobArea, kToneBass, "", style, knobBackgroundBitmap), -1, "EQ_KNOBS");
    pGraphics->AttachControl(
      new NAMKnobControl(midKnobArea, kToneMid, "", style, knobBackgroundBitmap), -1, "EQ_KNOBS");
    pGraphics->AttachControl(
      new NAMKnobControl(trebleKnobArea, kToneTreble, "", style, knobBackgroundBitmap), -1, "EQ_KNOBS");
    pGraphics->AttachControl(new NAMKnobControl(outputKnobArea, kOutputLevel, "", style, knobBackgroundBitmap));

    // The meters
    pGraphics->AttachControl(new NAMMeterControl(inputMeterArea, meterBackgroundBitmap, style), kCtrlTagInputMeter);
    pGraphics->AttachControl(new NAMMeterControl(outputMeterArea, meterBackgroundBitmap, style), kCtrlTagOutputMeter);

    // Settings/help/about box
    pGraphics->AttachControl(new NAMCircleButtonControl(
      settingsButtonArea,
      [pGraphics](IControl* pCaller) {
        pGraphics->GetControlWithTag(kCtrlTagSettingsBox)->As<NAMSettingsPageControl>()->HideAnimated(false);
      },
      gearSVG));

    pGraphics
      ->AttachControl(new NAMSettingsPageControl(b, backgroundBitmap, inputLevelBackgroundBitmap, switchHandleBitmap,
                                                 crossSVG, style, radioButtonStyle),
                      kCtrlTagSettingsBox)
      ->Hide(true);

    const auto slimKnobArea = b.GetCentredInside(100.f, NAM_KNOB_HEIGHT + 24.f);
    pGraphics->AttachControl(new NAMSlimOverlayBackdropControl(b, hideSlimOverlay), kCtrlTagSlimOverlayBackdrop)
      ->Hide(true);
    pGraphics
      ->AttachControl(new NAMKnobControl(slimKnobArea, kSlim, "Slim", style, knobBackgroundBitmap), kCtrlTagSlimKnob)
      ->Hide(true);

    pGraphics->ForAllControlsFunc([](IControl* pControl) {
      pControl->SetMouseEventsWhenDisabled(true);
      pControl->SetMouseOverWhenDisabled(true);
    });

    // pGraphics->GetControlWithTag(kCtrlTagOutNorm)->SetMouseEventsWhenDisabled(false);
    // pGraphics->GetControlWithTag(kCtrlTagCalibrateInput)->SetMouseEventsWhenDisabled(false);
  };

  mModelWorker = std::thread([this]() { _ModelWorkerLoop(); });
  mIRWorker = std::thread([this]() { _IRWorkerLoop(); });
}

NeuralAmpModeler::~NeuralAmpModeler()
{
  mModelWorkerQuit = true;
  mIRWorkerQuit = true;
  if (mModelWorker.joinable())
    mModelWorker.join();
  if (mIRWorker.joinable())
    mIRWorker.join();
  _DeallocateIOPointers();
}

void NeuralAmpModeler::ProcessBlock(iplug::sample** inputs, iplug::sample** outputs, int nFrames)
{
  const size_t numChannelsExternalIn = (size_t)NInChansConnected();
  const size_t numChannelsExternalOut = (size_t)NOutChansConnected();
  const size_t numChannelsInternal = kNumChannelsInternal;
  const size_t numFrames = (size_t)nFrames;
  const double sampleRate = GetSampleRate();

  // Disable floating point denormals
  std::fenv_t fe_state;
  std::feholdexcept(&fe_state);
  disable_denormals();

  _PrepareBuffers(numChannelsInternal, numFrames);
  // Input is collapsed to mono in preparation for the NAM.
  _ProcessInput(inputs, numFrames, numChannelsExternalIn, numChannelsInternal);
  _ApplyDSPStaging();
  const bool noiseGateActive = GetParam(kNoiseGateActive)->Value();
  const bool toneStackActive = GetParam(kEQActive)->Value();

  // Noise gate trigger
  sample** triggerOutput = mInputPointers;
  if (noiseGateActive)
  {
    const double time = 0.01;
    const double threshold = GetParam(kNoiseGateThreshold)->Value(); // GetParam...
    const double ratio = 0.1; // Quadratic...
    const double openTime = 0.005;
    const double holdTime = 0.01;
    const double closeTime = 0.05;
    const dsp::noise_gate::TriggerParams triggerParams(time, threshold, ratio, openTime, holdTime, closeTime);
    mNoiseGateTrigger.SetParams(triggerParams);
    mNoiseGateTrigger.SetSampleRate(sampleRate);
    triggerOutput = mNoiseGateTrigger.Process(mInputPointers, numChannelsInternal, numFrames);
  }

  if (mModel != nullptr)
  {
    mModel->process(triggerOutput, mOutputPointers, nFrames);
  }
  else
  {
    _FallbackDSP(triggerOutput, mOutputPointers, numChannelsInternal, numFrames);
  }
  // Apply the noise gate after the NAM
  sample** gateGainOutput =
    noiseGateActive ? mNoiseGateGain.Process(mOutputPointers, numChannelsInternal, numFrames) : mOutputPointers;

  sample** toneStackOutPointers = (toneStackActive && mToneStack != nullptr)
                                    ? mToneStack->Process(gateGainOutput, numChannelsInternal, nFrames)
                                    : gateGainOutput;

  sample** irPointers = toneStackOutPointers;
  if (mIR != nullptr && GetParam(kIRToggle)->Value())
    irPointers = mIR->Process(toneStackOutPointers, numChannelsInternal, numFrames);

  // And the HPF for DC offset (Issue 271)
  const double highPassCutoffFreq = kDCBlockerFrequency;
  // const double lowPassCutoffFreq = 20000.0;
  const recursive_linear_filter::HighPassParams highPassParams(sampleRate, highPassCutoffFreq);
  // const recursive_linear_filter::LowPassParams lowPassParams(sampleRate, lowPassCutoffFreq);
  mHighPass.SetParams(highPassParams);
  // mLowPass.SetParams(lowPassParams);
  sample** hpfPointers = mHighPass.Process(irPointers, numChannelsInternal, numFrames);
  // sample** lpfPointers = mLowPass.Process(hpfPointers, numChannelsInternal, numFrames);

  // restore previous floating point state
  std::feupdateenv(&fe_state);

  // Let's get outta here
  // This is where we exit mono for whatever the output requires.
  _ProcessOutput(hpfPointers, outputs, numFrames, numChannelsInternal, numChannelsExternalOut);
  // _ProcessOutput(lpfPointers, outputs, numFrames, numChannelsInternal, numChannelsExternalOut);
  // * Output of input leveling (inputs -> mInputPointers),
  // * Output of output leveling (mOutputPointers -> outputs)
  _UpdateMeters(mInputPointers, outputs, numFrames, numChannelsInternal, numChannelsExternalOut);
}

void NeuralAmpModeler::OnReset()
{
  const auto sampleRate = GetSampleRate();
  const int maxBlockSize = GetBlockSize();

  // Tail is because the HPF DC blocker has a decay.
  // 10 cycles should be enough to pass the VST3 tests checking tail behavior.
  // I'm ignoring the model & IR, but it's not the end of the world.
  const int tailCycles = 10;
  SetTailSize(tailCycles * (int)(sampleRate / kDCBlockerFrequency));
  mInputSender.Reset(sampleRate);
  mOutputSender.Reset(sampleRate);
  // If there is a model or IR loaded, they need to be checked for resampling.
  _ResetModelAndIR(sampleRate, GetBlockSize());
  mToneStack->Reset(sampleRate, maxBlockSize);
  _UpdateLatency();
}

void NeuralAmpModeler::OnIdle()
{
  mInputSender.TransmitData(*this);
  mOutputSender.TransmitData(*this);

  if (mModelPathNeedsUIRefresh.exchange(false))
  {
    if (GetUI())
    {
      WDL_String path;
      {
        std::lock_guard<std::mutex> lock(mModelListMutex);
        path = mNAMPath;
      }
      if (path.GetLength())
        SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadedModel, path.GetLength(), path.Get());
    }
  }

  if (mIRPathNeedsUIRefresh.exchange(false))
  {
    if (GetUI())
    {
      WDL_String path;
      {
        std::lock_guard<std::mutex> lock(mIRListMutex);
        path = mIRPath;
      }
      if (path.GetLength())
        SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, path.GetLength(), path.Get());
    }
  }

  if (mNewModelLoadedInDSP)
  {
    if (auto* pGraphics = GetUI())
    {
      _UpdateControlsFromModel();
      mNewModelLoadedInDSP = false;
    }
  }
  if (mModelCleared)
  {
    if (auto* pGraphics = GetUI())
    {
      // FIXME -- need to disable only the "normalized" model
      // pGraphics->GetControlWithTag(kCtrlTagOutputMode)->SetDisabled(false);
      static_cast<NAMSettingsPageControl*>(pGraphics->GetControlWithTag(kCtrlTagSettingsBox))->ClearModelInfo();
      if (auto* p = pGraphics->GetControlWithTag(kCtrlTagSlimmableIcon))
        p->Hide(true);
      if (auto* p = pGraphics->GetControlWithTag(kCtrlTagSlimOverlayBackdrop))
        p->Hide(true);
      if (auto* p = pGraphics->GetControlWithTag(kCtrlTagSlimKnob))
        p->Hide(true);
      pGraphics->SetAllControlsDirty();
      mModelCleared = false;
    }
  }
}

bool NeuralAmpModeler::SerializeState(IByteChunk& chunk) const
{
  // If this isn't here when unserializing, then we know we're dealing with something before v0.8.0.
  WDL_String header("###NeuralAmpModeler###"); // Don't change this!
  chunk.PutStr(header.Get());
  // Plugin version, so we can load legacy serialized states in the future!
  WDL_String version(PLUG_VERSION_STR);
  chunk.PutStr(version.Get());
  // Model directory (don't serialize the model itself; we'll just load it again
  // when we unserialize)
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    chunk.PutStr(mNAMPath.Get());
  }
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    chunk.PutStr(mIRPath.Get());
  }
  return SerializeParams(chunk);
}

int NeuralAmpModeler::UnserializeState(const IByteChunk& chunk, int startPos)
{
  // Look for the expected header. If it's there, then we'll know what to do.
  WDL_String header;
  int pos = startPos;
  pos = chunk.GetStr(header, pos);

  const char* kExpectedHeader = "###NeuralAmpModeler###";
  if (strcmp(header.Get(), kExpectedHeader) == 0)
  {
    return _UnserializeStateWithKnownVersion(chunk, pos);
  }
  else
  {
    return _UnserializeStateWithUnknownVersion(chunk, startPos);
  }
}

void NeuralAmpModeler::OnUIOpen()
{
  Plugin::OnUIOpen();

  if (mNAMPath.GetLength())
  {
    SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadedModel, mNAMPath.GetLength(), mNAMPath.Get());
    // If it's not loaded yet, then mark as failed.
    // If it's yet to be loaded, then the completion handler will set us straight once it runs.
    if (mModel == nullptr && mStagedModel == nullptr)
      SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadFailed);
  }

  if (mIRPath.GetLength())
  {
    SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, mIRPath.GetLength(), mIRPath.Get());
    if (mIR == nullptr && mStagedIR == nullptr)
      SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadFailed);
  }

  if (mModel != nullptr)
  {
    _UpdateControlsFromModel();
  }
}

void NeuralAmpModeler::OnParamChange(int paramIdx)
{
  switch (paramIdx)
  {
    // Changes to the input gain
    case kCalibrateInput:
    case kInputCalibrationLevel:
    case kInputLevel: _SetInputGain(); break;
    // Changes to the output gain
    case kOutputLevel:
    case kOutputMode: _SetOutputGain(); break;
    // Tone stack:
    case kToneBass: mToneStack->SetParam("bass", GetParam(paramIdx)->Value()); break;
    case kToneMid: mToneStack->SetParam("middle", GetParam(paramIdx)->Value()); break;
    case kToneTreble: mToneStack->SetParam("treble", GetParam(paramIdx)->Value()); break;
    case kSlim: _ApplySlimParamToLoadedNAMs(); break;
    // Only records the wish; the background thread does the actual (slow) loading.
    case kModelIndex: mWantedModelIndex.store(GetParam(kModelIndex)->Int()); break;
    case kIRIndex: mWantedIRIndex.store(GetParam(kIRIndex)->Int()); break;
    case kBankIndex: mWantedBankIndex.store(GetParam(kBankIndex)->Int()); break;
    case kIRBankIndex: mWantedIRBankIndex.store(GetParam(kIRBankIndex)->Int()); break;
    default: break;
  }
}

void NeuralAmpModeler::OnParamChangeUI(int paramIdx, EParamSource source)
{
  // A new Bank value means the Model / Cab list comes from another folder: swap the list, then tell the host
  // that the displayed file names changed.
  if (paramIdx == kBankIndex)
  {
    if (_ApplyModelBank(GetParam(kBankIndex)->Int()))
      InformHostOfParameterDetailsChange();
    return;
  }
  if (paramIdx == kIRBankIndex)
  {
    if (_ApplyIRBank(GetParam(kIRBankIndex)->Int()))
      InformHostOfParameterDetailsChange();
    return;
  }

  if (auto pGraphics = GetUI())
  {
    bool active = GetParam(paramIdx)->Bool();

    switch (paramIdx)
    {
      case kNoiseGateActive: pGraphics->GetControlWithParamIdx(kNoiseGateThreshold)->SetDisabled(!active); break;
      case kEQActive:
        pGraphics->ForControlInGroup("EQ_KNOBS", [active](IControl* pControl) { pControl->SetDisabled(!active); });
        break;
      case kIRToggle: pGraphics->GetControlWithTag(kCtrlTagIRFileBrowser)->SetDisabled(!active); break;
      default: break;
    }
  }
}

bool NeuralAmpModeler::OnMessage(int msgTag, int ctrlTag, int dataSize, const void* pData)
{
  switch (msgTag)
  {
    case kMsgTagClearModel: mShouldRemoveModel = true; return true;
    case kMsgTagClearIR: mShouldRemoveIR = true; return true;
    case kMsgTagHighlightColor:
    {
      mHighLightColor.Set((const char*)pData);

      if (GetUI())
      {
        GetUI()->ForStandardControlsFunc([&](IControl* pControl) {
          if (auto* pVectorBase = pControl->As<IVectorBase>())
          {
            IColor color = IColor::FromColorCodeStr(mHighLightColor.Get());

            pVectorBase->SetColor(kX1, color);
            pVectorBase->SetColor(kPR, color.WithOpacity(0.3f));
            pVectorBase->SetColor(kFR, color.WithOpacity(0.4f));
            pVectorBase->SetColor(kX3, color.WithContrast(0.1f));
          }
          pControl->GetUI()->SetAllControlsDirty();
        });
      }

      return true;
    }
    default: return false;
  }
}

// Private methods ============================================================

void NeuralAmpModeler::_AllocateIOPointers(const size_t nChans)
{
  if (mInputPointers != nullptr)
    throw std::runtime_error("Tried to re-allocate mInputPointers without freeing");
  mInputPointers = new sample*[nChans];
  if (mInputPointers == nullptr)
    throw std::runtime_error("Failed to allocate pointer to input buffer!\n");
  if (mOutputPointers != nullptr)
    throw std::runtime_error("Tried to re-allocate mOutputPointers without freeing");
  mOutputPointers = new sample*[nChans];
  if (mOutputPointers == nullptr)
    throw std::runtime_error("Failed to allocate pointer to output buffer!\n");
}

void NeuralAmpModeler::_ApplyDSPStaging()
{
  // Remove marked modules
  if (mShouldRemoveModel)
  {
    mModel = nullptr;
    mNAMPath.Set("");
    mShouldRemoveModel = false;
    mModelCleared = true;
    _UpdateLatency();
    _SetInputGain();
    _SetOutputGain();
  }
  if (mShouldRemoveIR)
  {
    mIR = nullptr;
    mIRPath.Set("");
    mShouldRemoveIR = false;
  }
  // Move things from staged to live
  if (mStagedModel != nullptr)
  {
    mModel = std::move(mStagedModel);
    mStagedModel = nullptr;
    mNewModelLoadedInDSP = true;
    _UpdateLatency();
    _SetInputGain();
    _SetOutputGain();
  }
  if (mStagedIR != nullptr)
  {
    mIR = std::move(mStagedIR);
    mStagedIR = nullptr;
  }
}

void NeuralAmpModeler::_DeallocateIOPointers()
{
  if (mInputPointers != nullptr)
  {
    delete[] mInputPointers;
    mInputPointers = nullptr;
  }
  if (mInputPointers != nullptr)
    throw std::runtime_error("Failed to deallocate pointer to input buffer!\n");
  if (mOutputPointers != nullptr)
  {
    delete[] mOutputPointers;
    mOutputPointers = nullptr;
  }
  if (mOutputPointers != nullptr)
    throw std::runtime_error("Failed to deallocate pointer to output buffer!\n");
}

void NeuralAmpModeler::_FallbackDSP(iplug::sample** inputs, iplug::sample** outputs, const size_t numChannels,
                                    const size_t numFrames)
{
  for (auto c = 0; c < numChannels; c++)
    for (auto s = 0; s < numFrames; s++)
      mOutputArray[c][s] = mInputArray[c][s];
}

void NeuralAmpModeler::_ResetModelAndIR(const double sampleRate, const int maxBlockSize)
{
  // Model
  if (mStagedModel != nullptr)
  {
    mStagedModel->Reset(sampleRate, maxBlockSize);
  }
  else if (mModel != nullptr)
  {
    mModel->Reset(sampleRate, maxBlockSize);
  }

  // IR
  if (mStagedIR != nullptr)
  {
    const double irSampleRate = mStagedIR->GetSampleRate();
    if (irSampleRate != sampleRate)
    {
      const auto irData = mStagedIR->GetData();
      mStagedIR = std::make_unique<LongImpulseResponse>(irData, sampleRate);
    }
  }
  else if (mIR != nullptr)
  {
    const double irSampleRate = mIR->GetSampleRate();
    if (irSampleRate != sampleRate)
    {
      const auto irData = mIR->GetData();
      mStagedIR = std::make_unique<LongImpulseResponse>(irData, sampleRate);
    }
  }
}

void NeuralAmpModeler::_SetInputGain()
{
  iplug::sample inputGainDB = GetParam(kInputLevel)->Value();
  // Input calibration
  if ((mModel != nullptr) && (mModel->HasInputLevel()) && GetParam(kCalibrateInput)->Bool())
  {
    inputGainDB += GetParam(kInputCalibrationLevel)->Value() - mModel->GetInputLevel();
  }
  mInputGain = DBToAmp(inputGainDB);
}

void NeuralAmpModeler::_SetOutputGain()
{
  double gainDB = GetParam(kOutputLevel)->Value();
  if (mModel != nullptr)
  {
    const int outputMode = GetParam(kOutputMode)->Int();
    switch (outputMode)
    {
      case 1: // Normalized
        if (mModel->HasLoudness())
        {
          const double loudness = mModel->GetLoudness();
          const double targetLoudness = -18.0;
          gainDB += (targetLoudness - loudness);
        }
        break;
      case 2: // Calibrated
        if (mModel->HasOutputLevel())
        {
          const double inputLevel = GetParam(kInputCalibrationLevel)->Value();
          const double outputLevel = mModel->GetOutputLevel();
          gainDB += (outputLevel - inputLevel);
        }
        break;
      case 0: // Raw
      default: break;
    }
  }
  mOutputGain = DBToAmp(gainDB);
}

void NeuralAmpModeler::_ApplySlimParamToLoadedNAMs()
{
  const double v = GetParam(kSlim)->Value();
  auto apply = [v](ResamplingNAM* p) {
    if (p == nullptr)
      return;
    if (nam::SlimmableModel* s = p->GetSlimmableModel())
      s->SetSlimmableSize(v);
  };
  apply(mModel.get());
  apply(mStagedModel.get());
}

std::string NeuralAmpModeler::_StageModel(const WDL_String& modelPath)
{
  WDL_String previousNAMPath = mNAMPath;
  try
  {
    auto dspPath = std::filesystem::u8path(modelPath.Get());
    std::unique_ptr<nam::DSP> model = nam::get_dsp(dspPath);

    // Check that the model has 1 input and 1 output channel
    if (model->NumInputChannels() != 1)
    {
      throw std::runtime_error("Model must have 1 input channel, but has " + std::to_string(model->NumInputChannels()));
    }
    if (model->NumOutputChannels() != 1)
    {
      throw std::runtime_error("Model must have 1 output channel, but has "
                               + std::to_string(model->NumOutputChannels()));
    }

    std::unique_ptr<ResamplingNAM> temp = std::make_unique<ResamplingNAM>(std::move(model), GetSampleRate());
    temp->Reset(GetSampleRate(), GetBlockSize());
    if (nam::SlimmableModel* slimmable = temp->GetSlimmableModel())
    {
      slimmable->SetSlimmableSize(GetParam(kSlim)->Value());
    }
    mStagedModel = std::move(temp);
    mNAMPath = modelPath;
    SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadedModel, mNAMPath.GetLength(), mNAMPath.Get());
  }
  catch (std::runtime_error& e)
  {
    SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadFailed);

    if (mStagedModel != nullptr)
    {
      mStagedModel = nullptr;
    }
    mNAMPath = previousNAMPath;
    std::cerr << "Failed to read DSP module" << std::endl;
    std::cerr << e.what() << std::endl;
    return e.what();
  }
  return "";
}

// Lists the "banks" around a folder: the sibling folders (same parent) that hold at least one file with the given
// extension. The current folder is always part of the list. When the parent is the user's home folder, the banks are
// not searched (that would walk through Documents, Desktop, ... and trigger macOS privacy prompts).
static void ListBankFolders(const std::filesystem::path& currentFolder, const std::string& extension, size_t maxCount,
                            std::vector<std::filesystem::path>& folders, std::vector<std::string>& names)
{
  folders.clear();
  names.clear();
  const std::filesystem::path root = currentFolder.parent_path();

  auto isHomeFolder = [&]() {
    for (const char* var : {"HOME", "USERPROFILE"})
    {
      const char* home = std::getenv(var);
      if (home != nullptr && *home != '\0')
      {
        std::error_code ec;
        const bool same = std::filesystem::equivalent(root, std::filesystem::path(home), ec);
        if (!ec && same)
          return true;
      }
    }
    return false;
  };
  auto hasFileWithExtension = [&](const std::filesystem::path& dir) {
    std::error_code ec;
    for (std::filesystem::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec))
    {
      std::error_code ec2;
      if (!it->is_regular_file(ec2))
        continue;
      const std::string fileName = it->path().filename().string();
      if (fileName.empty() || fileName[0] == '.')
        continue;
      std::string ext = it->path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
      if (ext == extension)
        return true;
    }
    return false;
  };

  std::vector<std::filesystem::path> found;
  if (root.empty() || isHomeFolder())
  {
    found.push_back(currentFolder);
  }
  else
  {
    std::error_code ec;
    for (std::filesystem::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec))
    {
      std::error_code ec2;
      if (!it->is_directory(ec2))
        continue;
      const std::string dirName = it->path().filename().string();
      if (dirName.empty() || dirName[0] == '.')
        continue;
      if (it->path().filename() == currentFolder.filename() || hasFileWithExtension(it->path()))
        found.push_back(it->path());
    }
  }

  auto lowerName = [](const std::filesystem::path& p) {
    std::string s = p.filename().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return s;
  };
  std::sort(found.begin(), found.end(),
            [&](const std::filesystem::path& a, const std::filesystem::path& b) { return lowerName(a) < lowerName(b); });

  if (found.size() > maxCount)
  {
    // Too many folders: keep the first ones, but never lose the current folder.
    const auto current = std::find_if(found.begin(), found.end(), [&](const std::filesystem::path& p) {
      return p.filename() == currentFolder.filename();
    });
    const bool currentIsKept = current != found.end() && static_cast<size_t>(current - found.begin()) < maxCount;
    const std::filesystem::path currentPath = current != found.end() ? *current : currentFolder;
    found.resize(maxCount);
    if (!currentIsKept)
      found.back() = currentPath;
  }

  for (const auto& p : found)
  {
    folders.push_back(p);
    names.push_back(p.filename().string());
  }
}

void NeuralAmpModeler::_RefreshModelBanks(const std::filesystem::path& currentFolder)
{
  std::vector<std::filesystem::path> folders;
  std::vector<std::string> names;
  ListBankFolders(currentFolder, ".nam", static_cast<size_t>(kMaxBanks), folders, names);

  int bankIndex = -1;
  for (size_t i = 0; i < folders.size(); i++)
  {
    if (folders[i].filename() == currentFolder.filename())
    {
      bankIndex = static_cast<int>(i);
      break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    mBankFolders = std::move(folders);
    mBankNames = std::move(names);
  }
  if (bankIndex >= 0)
  {
    mCurrentBankIndex.store(bankIndex);
    mWantedBankIndex.store(bankIndex);
    GetParam(kBankIndex)->Set(bankIndex);
  }
}

void NeuralAmpModeler::_RefreshIRBanks(const std::filesystem::path& currentFolder)
{
  std::vector<std::filesystem::path> folders;
  std::vector<std::string> names;
  ListBankFolders(currentFolder, ".wav", static_cast<size_t>(kMaxBanks), folders, names);

  int bankIndex = -1;
  for (size_t i = 0; i < folders.size(); i++)
  {
    if (folders[i].filename() == currentFolder.filename())
    {
      bankIndex = static_cast<int>(i);
      break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    mIRBankFolders = std::move(folders);
    mIRBankNames = std::move(names);
  }
  if (bankIndex >= 0)
  {
    mCurrentIRBankIndex.store(bankIndex);
    mWantedIRBankIndex.store(bankIndex);
    GetParam(kIRBankIndex)->Set(bankIndex);
  }
}

bool NeuralAmpModeler::_ApplyModelBank(int idx)
{
  std::filesystem::path folder;
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    if (mBankFolders.empty())
      return false;
    idx = std::clamp(idx, 0, static_cast<int>(mBankFolders.size()) - 1);
    if (idx == mCurrentBankIndex.load())
      return false;
    folder = mBankFolders[static_cast<size_t>(idx)];
  }
  _ScanModelFolder(folder);
  mCurrentBankIndex.store(idx);
  // Forces the worker to load the file at the current Model position, in the new folder.
  mLoadedModelIndex.store(-1);
  return true;
}

bool NeuralAmpModeler::_ApplyIRBank(int idx)
{
  std::filesystem::path folder;
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    if (mIRBankFolders.empty())
      return false;
    idx = std::clamp(idx, 0, static_cast<int>(mIRBankFolders.size()) - 1);
    if (idx == mCurrentIRBankIndex.load())
      return false;
    folder = mIRBankFolders[static_cast<size_t>(idx)];
  }
  _ScanIRFolder(folder);
  mCurrentIRBankIndex.store(idx);
  mLoadedIRIndex.store(-1);
  return true;
}

void NeuralAmpModeler::_ScanModelFolder(const std::filesystem::path& folder)
{
  std::vector<std::filesystem::path> files;
  std::error_code ec;
  for (std::filesystem::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec))
  {
    std::error_code ec2;
    if (!it->is_regular_file(ec2))
      continue;
    const std::string fileName = it->path().filename().string();
    if (fileName.empty() || fileName[0] == '.') // skips hidden files, e.g. macOS "._xxx.nam"
      continue;
    std::string ext = it->path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
    if (ext == ".nam")
      files.push_back(it->path());
  }

  auto lowerName = [](const std::filesystem::path& p) {
    std::string s = p.filename().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return s;
  };
  std::sort(files.begin(), files.end(),
            [&](const std::filesystem::path& a, const std::filesystem::path& b) { return lowerName(a) < lowerName(b); });
  if (files.size() > static_cast<size_t>(kMaxModels))
    files.resize(static_cast<size_t>(kMaxModels));

  std::vector<std::string> names;
  names.reserve(files.size());
  for (const auto& f : files)
    names.push_back(f.stem().string());

  std::lock_guard<std::mutex> lock(mModelListMutex);
  mModelFiles = std::move(files);
  mModelNames = std::move(names);
}

void NeuralAmpModeler::_SetModelFolderFromFile(const WDL_String& filePath, bool notifyHost)
{
  const std::filesystem::path file = std::filesystem::u8path(filePath.Get());
  _ScanModelFolder(file.parent_path());
  _RefreshModelBanks(file.parent_path());

  int index = -1;
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    for (size_t i = 0; i < mModelFiles.size(); i++)
    {
      if (mModelFiles[i].filename() == file.filename())
      {
        index = static_cast<int>(i);
        break;
      }
    }
  }
  if (index < 0)
    return; // The file isn't among the first kMaxModels files of its folder

  // The file is already being loaded by the caller: don't let the worker load it again.
  mLoadedModelIndex.store(index);
  mWantedModelIndex.store(index);
  GetParam(kModelIndex)->Set(index);

  if (notifyHost)
  {
    // The file names changed, and so did the position of the parameter.
    InformHostOfParameterDetailsChange();
    const double normalized = GetParam(kModelIndex)->ToNormalized(index);
    BeginInformHostOfParamChangeFromUI(kModelIndex);
    SendParameterValueFromUI(kModelIndex, normalized);
    EndInformHostOfParamChangeFromUI(kModelIndex);
    const double bankNormalized = GetParam(kBankIndex)->ToNormalized(GetParam(kBankIndex)->Int());
    BeginInformHostOfParamChangeFromUI(kBankIndex);
    SendParameterValueFromUI(kBankIndex, bankNormalized);
    EndInformHostOfParamChangeFromUI(kBankIndex);
  }
}

void NeuralAmpModeler::_ModelWorkerLoop()
{
  int lastSeen = -1;
  int stableCount = 0;
  while (!mModelWorkerQuit.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // A Bank change that arrived through the audio thread (the main-thread path normally handles it already)
    const int wantedBank = mWantedBankIndex.load();
    if (wantedBank >= 0 && wantedBank != mCurrentBankIndex.load())
      _ApplyModelBank(wantedBank);

    const int wanted = mWantedModelIndex.load();
    if (wanted < 0)
      continue;

    std::filesystem::path file;
    int effective = -1;
    {
      std::lock_guard<std::mutex> lock(mModelListMutex);
      if (!mModelFiles.empty())
      {
        // Steps beyond the last file simply stay on the last file.
        effective = std::min(wanted, static_cast<int>(mModelFiles.size()) - 1);
        file = mModelFiles[static_cast<size_t>(effective)];
      }
    }
    if (effective < 0 || effective == mLoadedModelIndex.load())
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }

    // Debounce: only load once the slider has stopped moving for ~100 ms.
    if (effective != lastSeen)
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }
    if (++stableCount < 3)
      continue;

    const std::string msg = _StageModelInBackground(file);
    if (msg.size())
      std::cerr << "Failed to load model " << file.string() << ": " << msg << std::endl;
    // Even on failure, so that we don't retry in a loop.
    mLoadedModelIndex.store(effective);
    stableCount = 0;
  }
}

std::string NeuralAmpModeler::_StageModelInBackground(const std::filesystem::path& path)
{
  try
  {
    std::unique_ptr<nam::DSP> model = nam::get_dsp(path);
    if (model->NumInputChannels() != 1)
      throw std::runtime_error("Model must have 1 input channel, but has " + std::to_string(model->NumInputChannels()));
    if (model->NumOutputChannels() != 1)
      throw std::runtime_error("Model must have 1 output channel, but has "
                               + std::to_string(model->NumOutputChannels()));

    std::unique_ptr<ResamplingNAM> temp = std::make_unique<ResamplingNAM>(std::move(model), GetSampleRate());
    temp->Reset(GetSampleRate(), GetBlockSize());
    if (nam::SlimmableModel* slimmable = temp->GetSlimmableModel())
      slimmable->SetSlimmableSize(GetParam(kSlim)->Value());
    mStagedModel = std::move(temp);
    {
      std::lock_guard<std::mutex> lock(mModelListMutex);
      mNAMPath.Set(path.string().c_str());
    }
    mModelPathNeedsUIRefresh = true; // the UI thread will update the file browser label
  }
  catch (std::exception& e)
  {
    return e.what();
  }
  return "";
}

void NeuralAmpModeler::_ScanIRFolder(const std::filesystem::path& folder)
{
  std::vector<std::filesystem::path> files;
  std::error_code ec;
  for (std::filesystem::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec))
  {
    std::error_code ec2;
    if (!it->is_regular_file(ec2))
      continue;
    const std::string fileName = it->path().filename().string();
    if (fileName.empty() || fileName[0] == '.') // skips hidden files, e.g. macOS "._xxx.wav"
      continue;
    std::string ext = it->path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
    if (ext == ".wav")
      files.push_back(it->path());
  }

  auto lowerName = [](const std::filesystem::path& p) {
    std::string s = p.filename().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return s;
  };
  std::sort(files.begin(), files.end(),
            [&](const std::filesystem::path& a, const std::filesystem::path& b) { return lowerName(a) < lowerName(b); });
  if (files.size() > static_cast<size_t>(kMaxIRs))
    files.resize(static_cast<size_t>(kMaxIRs));

  std::vector<std::string> names;
  names.reserve(files.size());
  for (const auto& f : files)
    names.push_back(f.stem().string());

  std::lock_guard<std::mutex> lock(mIRListMutex);
  mIRFiles = std::move(files);
  mIRNames = std::move(names);
}

void NeuralAmpModeler::_SetIRFolderFromFile(const WDL_String& filePath, bool notifyHost)
{
  const std::filesystem::path file = std::filesystem::u8path(filePath.Get());
  _ScanIRFolder(file.parent_path());
  _RefreshIRBanks(file.parent_path());

  int index = -1;
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    for (size_t i = 0; i < mIRFiles.size(); i++)
    {
      if (mIRFiles[i].filename() == file.filename())
      {
        index = static_cast<int>(i);
        break;
      }
    }
  }
  if (index < 0)
    return; // The file isn't among the first kMaxIRs files of its folder

  // The file is already being loaded by the caller: don't let the worker load it again.
  mLoadedIRIndex.store(index);
  mWantedIRIndex.store(index);
  GetParam(kIRIndex)->Set(index);

  if (notifyHost)
  {
    InformHostOfParameterDetailsChange();
    const double normalized = GetParam(kIRIndex)->ToNormalized(index);
    BeginInformHostOfParamChangeFromUI(kIRIndex);
    SendParameterValueFromUI(kIRIndex, normalized);
    EndInformHostOfParamChangeFromUI(kIRIndex);
    const double bankNormalized = GetParam(kIRBankIndex)->ToNormalized(GetParam(kIRBankIndex)->Int());
    BeginInformHostOfParamChangeFromUI(kIRBankIndex);
    SendParameterValueFromUI(kIRBankIndex, bankNormalized);
    EndInformHostOfParamChangeFromUI(kIRBankIndex);
  }
}

void NeuralAmpModeler::_IRWorkerLoop()
{
  int lastSeen = -1;
  int stableCount = 0;
  while (!mIRWorkerQuit.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    const int wantedBank = mWantedIRBankIndex.load();
    if (wantedBank >= 0 && wantedBank != mCurrentIRBankIndex.load())
      _ApplyIRBank(wantedBank);

    const int wanted = mWantedIRIndex.load();
    if (wanted < 0)
      continue;

    std::filesystem::path file;
    int effective = -1;
    {
      std::lock_guard<std::mutex> lock(mIRListMutex);
      if (!mIRFiles.empty())
      {
        // Steps beyond the last file simply stay on the last file.
        effective = std::min(wanted, static_cast<int>(mIRFiles.size()) - 1);
        file = mIRFiles[static_cast<size_t>(effective)];
      }
    }
    if (effective < 0 || effective == mLoadedIRIndex.load())
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }

    // Debounce: only load once the slider has stopped moving for ~100 ms.
    if (effective != lastSeen)
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }
    if (++stableCount < 3)
      continue;

    const std::string msg = _StageIRInBackground(file);
    if (msg.size())
      std::cerr << "Failed to load IR " << file.string() << ": " << msg << std::endl;
    // Even on failure, so that we don't retry in a loop.
    mLoadedIRIndex.store(effective);
    stableCount = 0;
  }
}

std::string NeuralAmpModeler::_StageIRInBackground(const std::filesystem::path& path)
{
  try
  {
    std::unique_ptr<LongImpulseResponse> temp = std::make_unique<LongImpulseResponse>(path.string().c_str(), GetSampleRate());
    const dsp::wav::LoadReturnCode wavState = temp->GetWavState();
    if (wavState != dsp::wav::LoadReturnCode::SUCCESS)
      return dsp::wav::GetMsgForLoadReturnCode(wavState);
    mStagedIR = std::move(temp);
    {
      std::lock_guard<std::mutex> lock(mIRListMutex);
      mIRPath.Set(path.string().c_str());
    }
    mIRPathNeedsUIRefresh = true; // the UI thread will update the file browser label
  }
  catch (std::exception& e)
  {
    return e.what();
  }
  return "";
}

dsp::wav::LoadReturnCode NeuralAmpModeler::_StageIR(const WDL_String& irPath)
{
  // FIXME it'd be better for the path to be "staged" as well. Just in case the
  // path and the model got caught on opposite sides of the fence...
  WDL_String previousIRPath = mIRPath;
  const double sampleRate = GetSampleRate();
  dsp::wav::LoadReturnCode wavState = dsp::wav::LoadReturnCode::ERROR_OTHER;
  try
  {
    mStagedIR = std::make_unique<LongImpulseResponse>(irPath.Get(), sampleRate);
    wavState = mStagedIR->GetWavState();
  }
  catch (std::runtime_error& e)
  {
    wavState = dsp::wav::LoadReturnCode::ERROR_OTHER;
    std::cerr << "Caught unhandled exception while attempting to load IR:" << std::endl;
    std::cerr << e.what() << std::endl;
  }

  if (wavState == dsp::wav::LoadReturnCode::SUCCESS)
  {
    mIRPath = irPath;
    SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, mIRPath.GetLength(), mIRPath.Get());
  }
  else
  {
    if (mStagedIR != nullptr)
    {
      mStagedIR = nullptr;
    }
    mIRPath = previousIRPath;
    SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadFailed);
  }

  return wavState;
}

size_t NeuralAmpModeler::_GetBufferNumChannels() const
{
  // Assumes input=output (no mono->stereo effects)
  return mInputArray.size();
}

size_t NeuralAmpModeler::_GetBufferNumFrames() const
{
  if (_GetBufferNumChannels() == 0)
    return 0;
  return mInputArray[0].size();
}

void NeuralAmpModeler::_InitToneStack()
{
  // If you want to customize the tone stack, then put it here!
  mToneStack = std::make_unique<dsp::tone_stack::BasicNamToneStack>();
}
void NeuralAmpModeler::_PrepareBuffers(const size_t numChannels, const size_t numFrames)
{
  const bool updateChannels = numChannels != _GetBufferNumChannels();
  const bool updateFrames = updateChannels || (_GetBufferNumFrames() != numFrames);
  //  if (!updateChannels && !updateFrames)  // Could we do this?
  //    return;

  if (updateChannels)
  {
    _PrepareIOPointers(numChannels);
    mInputArray.resize(numChannels);
    mOutputArray.resize(numChannels);
  }
  if (updateFrames)
  {
    for (auto c = 0; c < mInputArray.size(); c++)
    {
      mInputArray[c].resize(numFrames);
      std::fill(mInputArray[c].begin(), mInputArray[c].end(), 0.0);
    }
    for (auto c = 0; c < mOutputArray.size(); c++)
    {
      mOutputArray[c].resize(numFrames);
      std::fill(mOutputArray[c].begin(), mOutputArray[c].end(), 0.0);
    }
  }
  // Would these ever get changed by something?
  for (auto c = 0; c < mInputArray.size(); c++)
    mInputPointers[c] = mInputArray[c].data();
  for (auto c = 0; c < mOutputArray.size(); c++)
    mOutputPointers[c] = mOutputArray[c].data();
}

void NeuralAmpModeler::_PrepareIOPointers(const size_t numChannels)
{
  _DeallocateIOPointers();
  _AllocateIOPointers(numChannels);
}

void NeuralAmpModeler::_ProcessInput(iplug::sample** inputs, const size_t nFrames, const size_t nChansIn,
                                     const size_t nChansOut)
{
  // We'll assume that the main processing is mono for now. We'll handle dual amps later.
  if (nChansOut != 1)
  {
    std::stringstream ss;
    ss << "Expected mono output, but " << nChansOut << " output channels are requested!";
    throw std::runtime_error(ss.str());
  }

  // On the standalone, we can probably assume that the user has plugged into only one input and they expect it to be
  // carried straight through. Don't apply any division over nChansIn because we're just "catching anything out there."
  // However, in a DAW, it's probably something providing stereo, and we want to take the average in order to avoid
  // doubling the loudness. (This would change w/ double mono processing)
  double gain = mInputGain;
#ifndef APP_API
  gain /= (float)nChansIn;
#endif
  // Assume _PrepareBuffers() was already called
  for (size_t c = 0; c < nChansIn; c++)
    for (size_t s = 0; s < nFrames; s++)
      if (c == 0)
        mInputArray[0][s] = gain * inputs[c][s];
      else
        mInputArray[0][s] += gain * inputs[c][s];
}

void NeuralAmpModeler::_ProcessOutput(iplug::sample** inputs, iplug::sample** outputs, const size_t nFrames,
                                      const size_t nChansIn, const size_t nChansOut)
{
  const double gain = mOutputGain;
  // Assume _PrepareBuffers() was already called
  if (nChansIn != 1)
    throw std::runtime_error("Plugin is supposed to process in mono.");
  // Broadcast the internal mono stream to all output channels.
  const size_t cin = 0;
  for (auto cout = 0; cout < nChansOut; cout++)
    for (auto s = 0; s < nFrames; s++)
#ifdef APP_API // Ensure valid output to interface
      outputs[cout][s] = std::clamp(gain * inputs[cin][s], -1.0, 1.0);
#else // In a DAW, other things may come next and should be able to handle large
      // values.
      outputs[cout][s] = gain * inputs[cin][s];
#endif
}

void NeuralAmpModeler::_UpdateControlsFromModel()
{
  if (mModel == nullptr)
  {
    return;
  }
  if (auto* pGraphics = GetUI())
  {
    ModelInfo modelInfo;
    modelInfo.sampleRate.known = true;
    modelInfo.sampleRate.value = mModel->GetEncapsulatedSampleRate();
    modelInfo.inputCalibrationLevel.known = mModel->HasInputLevel();
    modelInfo.inputCalibrationLevel.value = mModel->HasInputLevel() ? mModel->GetInputLevel() : 0.0;
    modelInfo.outputCalibrationLevel.known = mModel->HasOutputLevel();
    modelInfo.outputCalibrationLevel.value = mModel->HasOutputLevel() ? mModel->GetOutputLevel() : 0.0;

    static_cast<NAMSettingsPageControl*>(pGraphics->GetControlWithTag(kCtrlTagSettingsBox))->SetModelInfo(modelInfo);

    const bool disableInputCalibrationControls = !mModel->HasInputLevel();
    pGraphics->GetControlWithTag(kCtrlTagCalibrateInput)->SetDisabled(disableInputCalibrationControls);
    pGraphics->GetControlWithTag(kCtrlTagInputCalibrationLevel)->SetDisabled(disableInputCalibrationControls);
    {
      auto* c = static_cast<OutputModeControl*>(pGraphics->GetControlWithTag(kCtrlTagOutputMode));
      c->SetNormalizedDisable(!mModel->HasLoudness());
      c->SetCalibratedDisable(!mModel->HasOutputLevel());
    }

    if (auto* pSlimIcon = pGraphics->GetControlWithTag(kCtrlTagSlimmableIcon))
    {
      const bool show = mModel->GetSlimmableModel() != nullptr;
      pSlimIcon->Hide(!show);
    }
  }
}

void NeuralAmpModeler::_UpdateLatency()
{
  int latency = 0;
  if (mModel)
  {
    latency += mModel->GetLatency();
  }
  // Other things that add latency here...

  // Feels weird to have to do this.
  if (GetLatency() != latency)
  {
    SetLatency(latency);
  }
}

void NeuralAmpModeler::_UpdateMeters(sample** inputPointer, sample** outputPointer, const size_t nFrames,
                                     const size_t nChansIn, const size_t nChansOut)
{
  // Right now, we didn't specify MAXNC when we initialized these, so it's 1.
  const int nChansHack = 1;
  mInputSender.ProcessBlock(inputPointer, (int)nFrames, kCtrlTagInputMeter, nChansHack);
  mOutputSender.ProcessBlock(outputPointer, (int)nFrames, kCtrlTagOutputMeter, nChansHack);
}

// HACK
#include "Unserialization.cpp"
