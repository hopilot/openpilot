#pragma once

#include <QPushButton>
#include <QLineEdit>
#include <QSoundEffect>


#include <QComboBox>
#include <QAbstractItemView>
#include <QMessageBox>
#include <QProcess>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/groupWidget.h"
#include "selfdrive/ui/ui.h"

class CPandaGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CPandaGroup();

};

class CGitGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CGitGroup( void *parent=0);
};


class CUtilWidget : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CUtilWidget( void *parent );
};


class CPresetWidget : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CPresetWidget();

public slots:  
  virtual void refresh(int nID = 0);

  
};



class SwitchOpenpilot : public ButtonControl {
  Q_OBJECT

public:
  SwitchOpenpilot();
  void executeProgram(const QString &tcmd);

private slots:
  void printMsg();
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
  Params params;

  QString githubid;
  QString githubrepo;
  QString githubbranch;

  QProcess *textMsgProcess;
  QMessageBox *outbox;
  QString outdata;

  void refresh();
  void getUserID(const QString &userid);
  void getRepoID(const QString &repoid);
  void getBranchID(const QString &branchid);
};

class SshLegacyToggle : public ToggleControl {
  Q_OBJECT

public:
  SshLegacyToggle() : ToggleControl("Use Legacy SSH Key", "Public KEY (0.8.2 or less) is used when accessing SSH.", "", Params().getBool("OpkrSSHLegacy")) {
    QObject::connect(this, &SshLegacyToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrSSHLegacy", status);
    });
  }
};

class AutoResumeToggle : public ToggleControl {
  Q_OBJECT

public:
  AutoResumeToggle() : ToggleControl("Use Auto Resume at Stop", "It uses the automatic departure function when stopping while using SCC.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrAutoResume")) {
    QObject::connect(this, &AutoResumeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrAutoResume", status);
    });
  }
};

class VariableCruiseToggle : public ToggleControl {
  Q_OBJECT

public:
  VariableCruiseToggle() : ToggleControl("Use Cruise Button Spamming", "Use the cruise button while using SCC to assist in acceleration and deceleration.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrVariableCruise")) {
    QObject::connect(this, &VariableCruiseToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrVariableCruise", status);
    });
  }
};

class CruiseGapAdjustToggle : public ToggleControl {
  Q_OBJECT

public:
  CruiseGapAdjustToggle() : ToggleControl("Change Cruise Gap at Stop", "For a quick start when stopping, the cruise gap will be changed to 1 step, and after departure, it will return to the original cruise gap according to certain conditions.", "../assets/offroad/icon_shell.png", Params().getBool("CruiseGapAdjust")) {
    QObject::connect(this, &CruiseGapAdjustToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseGapAdjust", status);
    });
  }
};

class AutoEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  AutoEnabledToggle() : ToggleControl("Use Auto Engagement", "If the cruise button status is standby (CRUISE indication only and speed is not specified) in the Disengagement state, activate the automatic Engagement.", "../assets/offroad/icon_shell.png", Params().getBool("AutoEnable")) {
    QObject::connect(this, &AutoEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AutoEnable", status);
    });
  }
};

class CruiseAutoResToggle : public ToggleControl {
  Q_OBJECT

public:
  CruiseAutoResToggle() : ToggleControl("Use Auto RES while Driving", "If the brake is applied while using the SCC and the standby mode is changed (CANCEL is not applicable), set it back to the previous speed when the brake pedal is released/accelerated pedal is operated. It operates when the cruise speed is set and the vehicle speed is more than 30 km/h or the car in front is recognized.", "../assets/offroad/icon_shell.png", Params().getBool("CruiseAutoRes")) {
    QObject::connect(this, &CruiseAutoResToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseAutoRes", status);
    });
  }
};

class BatteryChargingControlToggle : public ToggleControl {
  Q_OBJECT

public:
  BatteryChargingControlToggle() : ToggleControl("Enable Battery Charging Control", "It uses the battery charge control function.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrBatteryChargingControl")) {
    QObject::connect(this, &BatteryChargingControlToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrBatteryChargingControl", status);
    });
  }
};

class BlindSpotDetectToggle : public ToggleControl {
  Q_OBJECT

public:
  BlindSpotDetectToggle() : ToggleControl("Show BSM Status", "If a car is detected in the rear, it will be displayed on the screen.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrBlindSpotDetect")) {
    QObject::connect(this, &BlindSpotDetectToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrBlindSpotDetect", status);
      if (state) {
        QUIState::ui_state.scene.nOpkrBlindSpotDetect = true;
      } else {
        QUIState::ui_state.scene.nOpkrBlindSpotDetect = false;
      }
    });
  }
};

class UFCModeEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  UFCModeEnabledToggle() : ToggleControl("User-Friendly Control(UFC) Mode", "OP activates with Main Cruise Switch, AutoRES while driving, Seperate Lat/Long and etc", "../assets/offroad/icon_shell.png", Params().getBool("UFCModeEnabled")) {
    QObject::connect(this, &UFCModeEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UFCModeEnabled", status);
    });
  }
};

class WhitePandaSupportToggle : public ToggleControl {
  Q_OBJECT

public:
  WhitePandaSupportToggle() : ToggleControl("Support WhitePanda", "Turn on this function if you use WhitePanda.", "../assets/offroad/icon_shell.png", Params().getBool("WhitePandaSupport")) {
    QObject::connect(this, &WhitePandaSupportToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("WhitePandaSupport", status);
    });
  }
};

class SteerWarningFixToggle : public ToggleControl {
  Q_OBJECT

public:
  SteerWarningFixToggle() : ToggleControl("Ignore of Steering Warning", "Turn on the function when a steering error occurs in the vehicle and the open pilot cannot be executed (some vehicles only). Do not turn on the function if it occurs in a normal error environment while driving.", "../assets/offroad/icon_shell.png", Params().getBool("SteerWarningFix")) {
    QObject::connect(this, &SteerWarningFixToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("SteerWarningFix", status);
    });
  }
};

class LiveSteerRatioToggle : public ToggleControl {
  Q_OBJECT

public:
  LiveSteerRatioToggle() : ToggleControl("Use Live SteerRatio", "Live SteerRatio is used instead of variable/fixed SteerRatio.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrLiveSteerRatio")) {
    QObject::connect(this, &LiveSteerRatioToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrLiveSteerRatio", status);
    });
  }
};

class VariableSteerMaxToggle : public ToggleControl {
  Q_OBJECT

public:
  VariableSteerMaxToggle() : ToggleControl("SteerMax/Variable SteerMax Toggle", "Use the variable SteerMax by curvature. If this is off, runs only with base value below. STBase: SteerMax Default value. STMax: SteerMax Maximum value.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrVariableSteerMax")) {
    QObject::connect(this, &VariableSteerMaxToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrVariableSteerMax", status);
    });
  }
};

class VariableSteerDeltaToggle : public ToggleControl {
  Q_OBJECT

public:
  VariableSteerDeltaToggle() : ToggleControl("DeltaUpDown/Variable Delta Toggle", "Use variable SteerDelta by curvature. If this is off, runs only with base value below. DUBase: Steer DeltaUp Default value. DUMax: Steer DeltaUp Maximum value. DDBase: Steer DeltaDown Default value. DDMax: Steer DeltaDown Maximum value.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrVariableSteerDelta")) {
    QObject::connect(this, &VariableSteerDeltaToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrVariableSteerDelta", status);
    });
  }
};

class DrivingRecordToggle : public ToggleControl {
  Q_OBJECT

public:
  DrivingRecordToggle() : ToggleControl("Use Auto Screen Record", "Automatically record/stop the screen while driving. Recording begins after departure, and recording ends when the vehicle stops.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrDrivingRecord")) {
    QObject::connect(this, &DrivingRecordToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrDrivingRecord", status);
      if (state) {
        QUIState::ui_state.scene.driving_record = true;
      } else {
        QUIState::ui_state.scene.driving_record = false;
      }
    });
  }
};

class TurnSteeringDisableToggle : public ToggleControl {
  Q_OBJECT

public:
  TurnSteeringDisableToggle() : ToggleControl("Stop Steer Assist on Turn Signals", "When driving below the lane change speed, the automatic steering is temporarily paused while the turn signals on.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrTurnSteeringDisable")) {
    QObject::connect(this, &TurnSteeringDisableToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrTurnSteeringDisable", status);
    });
  }
};

class HotspotOnBootToggle : public ToggleControl {
  Q_OBJECT

public:
  HotspotOnBootToggle() : ToggleControl("HotSpot on Boot", "It automatically runs a hotspot when booting.", "", Params().getBool("OpkrHotspotOnBoot")) {
    QObject::connect(this, &HotspotOnBootToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrHotspotOnBoot", status);
    });
  }
};

class CruiseOverMaxSpeedToggle : public ToggleControl {
  Q_OBJECT

public:
  CruiseOverMaxSpeedToggle() : ToggleControl("Reset MaxSpeed Over CurrentSpeed", "If the current speed exceeds the set speed, synchronize the set speed with the current speed.", "../assets/offroad/icon_shell.png", Params().getBool("CruiseOverMaxSpeed")) {
    QObject::connect(this, &CruiseOverMaxSpeedToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseOverMaxSpeed", status);
    });
  }
};

class DebugUiOneToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiOneToggle() : ToggleControl("DEBUG UI 1", "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi1")) {
    QObject::connect(this, &DebugUiOneToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi1", status);
      if (state) {
        QUIState::ui_state.scene.nDebugUi1 = true;
      } else {
        QUIState::ui_state.scene.nDebugUi1 = false;
      }
    });
  }
};

class DebugUiTwoToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiTwoToggle() : ToggleControl("DEBUG UI 2", "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi2")) {
    QObject::connect(this, &DebugUiTwoToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi2", status);
      if (state) {
        QUIState::ui_state.scene.nDebugUi2 = true;
      } else {
        QUIState::ui_state.scene.nDebugUi2 = false;
      }
    });
  }
};

class DebugUiThreeToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiThreeToggle() : ToggleControl("DEBUG UI 3", "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi3")) {
    QObject::connect(this, &DebugUiThreeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi3", status);
      if (state) {
        QUIState::ui_state.scene.nDebugUi3 = true;
      } else {
        QUIState::ui_state.scene.nDebugUi3 = false;
      }
    });
  }
};

class LongLogToggle : public ToggleControl {
  Q_OBJECT

public:
  LongLogToggle() : ToggleControl("Show LongControl LOG", "Display logs for long tuning debugs instead of variable cruise logs on the screen.", "../assets/offroad/icon_shell.png", Params().getBool("LongLogDisplay")) {
    QObject::connect(this, &LongLogToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("LongLogDisplay", status);
    });
  }
};

class PrebuiltToggle : public ToggleControl {
  Q_OBJECT

public:
  PrebuiltToggle() : ToggleControl("Use Smart Prebuilt", "Create a Prebuilt file and speed up booting. When this function is turned on, the booting speed is accelerated using the cache, and if you press the update button in the menu after modifying the code, or if you rebooted with the 'gi' command in the command window, remove it automatically and compile it.", "../assets/offroad/icon_shell.png", Params().getBool("PutPrebuiltOn")) {
    QObject::connect(this, &PrebuiltToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("PutPrebuiltOn", status);
    });
  }
};

class LDWSToggle : public ToggleControl {
  Q_OBJECT

public:
  LDWSToggle() : ToggleControl("Set LDWS Vehicles", "", "../assets/offroad/icon_shell.png", Params().getBool("LdwsCarFix")) {
    QObject::connect(this, &LDWSToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("LdwsCarFix", status);
    });
  }
};

class FPTwoToggle : public ToggleControl {
  Q_OBJECT

public:
  FPTwoToggle() : ToggleControl("Use FingerPrint 2.0", "Activate Fingerprint 2.0. Activate the vehicle with ECU recognition.", "../assets/offroad/icon_shell.png", Params().getBool("FingerprintTwoSet")) {
    QObject::connect(this, &FPTwoToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("FingerprintTwoSet", status);
    });
  }
};

class GearDToggle : public ToggleControl {
  Q_OBJECT

public:
  GearDToggle() : ToggleControl("Set DriverGear by Force", "It is used when the gear recognition problem. Basically, CABANA data should be analyzed, but it is temporarily resolved.", "../assets/offroad/icon_shell.png", Params().getBool("JustDoGearD")) {
    QObject::connect(this, &GearDToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("JustDoGearD", status);
    });
  }
};

class ComIssueToggle : public ToggleControl {
  Q_OBJECT

public:
  ComIssueToggle() : ToggleControl("Turn Off Communication Issue Alarm", "Turn this option on to disable the communication issue alarm.", "../assets/offroad/icon_shell.png", Params().getBool("ComIssueGone")) {
    QObject::connect(this, &ComIssueToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("ComIssueGone", status);
    });
  }
};

class RunNaviOnBootToggle : public ToggleControl {
  Q_OBJECT

public:
  RunNaviOnBootToggle() : ToggleControl("Run Navigation on Boot", "Automatically execute the navigation (waze) when switching to the driving screen after booting.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrRunNaviOnBoot")) {
    QObject::connect(this, &RunNaviOnBootToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrRunNaviOnBoot", status);
    });
  }
};

class BattLessToggle : public ToggleControl {
  Q_OBJECT

public:
  BattLessToggle() : ToggleControl("Set BatteryLess Eon", "This is a toggle for batteryless EON. Related settings apply.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrBattLess")) {
    QObject::connect(this, &BattLessToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrBattLess", status);
    });
  }
};

class LiveTunePanelToggle : public ToggleControl {
  Q_OBJECT

public:
  LiveTunePanelToggle() : ToggleControl("Use LiveTune and Show UI", "Display the UI related to live tuning on the screen. Various tuning values can be adjusted live on the driving screen. It is reflected in the parameter when adjusting, and the value is maintained even after turning off the toggle and rebooting.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrLiveTunePanelEnable")) {
    QObject::connect(this, &LiveTunePanelToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrLiveTunePanelEnable", status);
      if (state) {
        QUIState::ui_state.scene.live_tune_panel_enable = true;
        QUIState::ui_state.scene.opkr_livetune_ui = true;
      } else {
        QUIState::ui_state.scene.live_tune_panel_enable = false;
        QUIState::ui_state.scene.opkr_livetune_ui = false;
      }
    });
  }
};

class GitPullOnBootToggle : public ToggleControl {
  Q_OBJECT

public:
  GitPullOnBootToggle() : ToggleControl("Git Pull On Boot", "If there is an update after the boot, run Git Pull automatically and reboot.", "", Params().getBool("GitPullOnBoot")) {
    QObject::connect(this, &GitPullOnBootToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("GitPullOnBoot", status);
    });
  }
};

class StoppingDistAdjToggle : public ToggleControl {
  Q_OBJECT

public:
  StoppingDistAdjToggle() : ToggleControl("Adjust Stopping Distance", "Stop a little further ahead than the radar stop distance. If you approach the car in front of you at a high speed, it may sometimes be difficult to stop enough, so if you are uncomfortable, turn off the function.", "../assets/offroad/icon_shell.png", Params().getBool("StoppingDistAdj")) {
    QObject::connect(this, &StoppingDistAdjToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StoppingDistAdj", status);
    });
  }
};

class ShowErrorToggle : public ToggleControl {
  Q_OBJECT

public:
  ShowErrorToggle() : ToggleControl("Show TMUX Error", "Display the error on the EON screen when a process error occurs while driving or off-road.", "../assets/offroad/icon_shell.png", Params().getBool("ShowError")) {
    QObject::connect(this, &ShowErrorToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("ShowError", status);
      if (state) {
        QUIState::ui_state.scene.show_error = true;
      } else {
        QUIState::ui_state.scene.show_error = false;
      }
    });
  }
};

class StockNaviSpeedToggle : public ToggleControl {
  Q_OBJECT

public:
  StockNaviSpeedToggle() : ToggleControl("Use Stock SafetyCAM Speed", "When decelerating the safety section, use the safety speed from the vehicle navigation system (limited to some vehicles with the corresponding data).", "../assets/offroad/icon_shell.png", Params().getBool("StockNaviSpeedEnabled")) {
    QObject::connect(this, &StockNaviSpeedToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StockNaviSpeedEnabled", status);
    });
  }
};

class E2ELongToggle : public ToggleControl {
  Q_OBJECT

public:
  E2ELongToggle() : ToggleControl("Enable E2E Long", "Activate E2E Long. It may work unexpectedly. Be careful.", "../assets/offroad/icon_shell.png", Params().getBool("E2ELong")) {
    QObject::connect(this, &E2ELongToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("E2ELong", status);
    });
  }
};

class StopAtStopSignToggle : public ToggleControl {
  Q_OBJECT

public:
  StopAtStopSignToggle() : ToggleControl("Stop at Stop Sign", "Openpilot tries to stop at stop sign depends on Model.", "../assets/offroad/icon_shell.png", Params().getBool("StopAtStopSign")) {
    QObject::connect(this, &StopAtStopSignToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StopAtStopSign", status);
    });
  }
};

class GoogleMapEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  GoogleMapEnabledToggle() : ToggleControl("Use GoogleMap for Mapbox", "Use GoogleMap when you search a destination.", "../assets/offroad/icon_shell.png", Params().getBool("GoogleMapEnabled")) {
    QObject::connect(this, &GoogleMapEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("GoogleMapEnabled", status);
    });
  }
};

class OSMEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  OSMEnabledToggle() : ToggleControl("Enable OSM", "This enables OSM.", "../assets/offroad/icon_shell.png", Params().getBool("OSMEnable")) {
    QObject::connect(this, &OSMEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OSMEnable", status);
    });
  }
};

class OSMSpeedLimitEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  OSMSpeedLimitEnabledToggle() : ToggleControl("Enable OSM SpeedLimit", "This enables OSM speedlimit.", "../assets/offroad/icon_shell.png", Params().getBool("OSMSpeedLimitEnable")) {
    QObject::connect(this, &OSMSpeedLimitEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OSMSpeedLimitEnable", status);
    });
  }
};

class IgnoreCanErroronISGToggle : public ToggleControl {
  Q_OBJECT

public:
  IgnoreCanErroronISGToggle() : ToggleControl("Ignore Can Error on ISG", "Turn this on, if can error occurs on ISG operation.", "../assets/offroad/icon_shell.png", Params().getBool("IgnoreCANErroronISG")) {
    QObject::connect(this, &IgnoreCanErroronISGToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("IgnoreCANErroronISG", status);
    });
  }
};

class StockLKASEnabledatDisenagedStatusToggle : public ToggleControl {
  Q_OBJECT

public:
  StockLKASEnabledatDisenagedStatusToggle() : ToggleControl("StockLKAS Enabled at Disengagement", "Turn this on, if you want to use Stock LKAS at OP disengaged status. Seems this related to cluster error when OP active because Stock CAN messages over PANDA or not.", "../assets/offroad/icon_shell.png", Params().getBool("StockLKASEnabled")) {
    QObject::connect(this, &StockLKASEnabledatDisenagedStatusToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StockLKASEnabled", status);
    });
  }
};

class FCA11MessageToggle : public ToggleControl {
  Q_OBJECT

public:
  FCA11MessageToggle() : ToggleControl("Enable FCA11 Message", "Turn this on, if you get an error with forward collision warning.(reboot required)", "../assets/offroad/icon_shell.png", Params().getBool("FCA11Message")) {
    QObject::connect(this, &FCA11MessageToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("FCA11Message", status);
    });
  }
};

class StandstillResumeAltToggle : public ToggleControl {
  Q_OBJECT

public:
  StandstillResumeAltToggle() : ToggleControl("Standstill Resume Alternative", "Turn this on, if auto resume doesn't work at standstill. some cars only(ex. GENESIS). before enable, try to adjust RES message counts above.(reboot required)", "../assets/offroad/icon_shell.png", Params().getBool("StandstillResumeAlt")) {
    QObject::connect(this, &StandstillResumeAltToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StandstillResumeAlt", status);
    });
  }
};

class MapboxEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  MapboxEnabledToggle() : ToggleControl("Enable Mapbox", "If you want to use Mapbox, turn on and then connect to device using web browser http://<device ip>:8082  Mapbox setting will show up and type mapbox pk and sk token(you can created this on mapbox.com website). If you want to search destinations with googlemap, first, you should create google api key and enable Enable GoogleMap for Mapbox", "../assets/offroad/icon_shell.png", Params().getBool("MapboxEnabled")) {
    QObject::connect(this, &MapboxEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("MapboxEnabled", status);
    });
  }
};

class UseRadarTrackToggle : public ToggleControl {
  Q_OBJECT

public:
  UseRadarTrackToggle() : ToggleControl("Use Radar Track", "Some cars have known radar tracks(from comma) for long control. This uses radar track directly instead of scc can message. Before you go, you must need to run hyundai_enable_radar_points.py in /data/openpilot/selfdrive/debug dir to enable your radar track. (Reboot required)", "../assets/offroad/icon_shell.png", Params().getBool("UseRadarTrack")) {
    QObject::connect(this, &UseRadarTrackToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UseRadarTrack", status);
    });
  }
};

class RadarDisableToggle : public ToggleControl {
  Q_OBJECT

public:
  RadarDisableToggle() : ToggleControl("Disable Radar", "This is pre-requisite for LongControl of HKG. It seems that this affects AEB. So do not use this if you have any concern.", "../assets/offroad/icon_shell.png", Params().getBool("RadarDisable")) {
    QObject::connect(this, &RadarDisableToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RadarDisable", status);
    });
  }
};

class C2WithCommaPowerToggle : public ToggleControl {
  Q_OBJECT

public:
  C2WithCommaPowerToggle() : ToggleControl("C2 with CommaPower", "This is for C2 users with Comma Power.", "../assets/offroad/icon_shell.png", Params().getBool("C2WithCommaPower")) {
    QObject::connect(this, &C2WithCommaPowerToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("C2WithCommaPower", status);
    });
  }
};

class CustomTRToggle : public ToggleControl {
  Q_OBJECT

public:
  CustomTRToggle() : ToggleControl("Custom TR Enable", "to use Custom TR not 1.45(comma default).", "../assets/offroad/icon_shell.png", Params().getBool("CustomTREnabled")) {
    QObject::connect(this, &CustomTRToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CustomTREnabled", status);
    });
  }
};

class RoutineDriveOnToggle : public ToggleControl {
  Q_OBJECT

public:
  RoutineDriveOnToggle() : ToggleControl("Routine Drive by RoadName", "This will adjust useful things by roadname. If you want to use, edit the file, /data/params/d/RoadList. modify like this RoadName1,offset1(ex:+0.05),RoadName2,offset2(ex:-0.05),... and the second line RoadName3,speedlimit(ex:30),RoadName4,speedlimit(ex:60),...", "../assets/offroad/icon_shell.png", Params().getBool("RoutineDriveOn")) {
    QObject::connect(this, &RoutineDriveOnToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RoutineDriveOn", status);
    });
  }
};

class CloseToRoadEdgeToggle : public ToggleControl {
  Q_OBJECT

public:
  CloseToRoadEdgeToggle() : ToggleControl("Driving Close to RoadEdge", "This will adjust the camera offset to get close to road edge if the car is on the first or last lane.", "../assets/offroad/icon_shell.png", Params().getBool("CloseToRoadEdge")) {
    QObject::connect(this, &CloseToRoadEdgeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CloseToRoadEdge", status);
    });
  }
};

class ToAvoidLKASFaultToggle : public ToggleControl {
  Q_OBJECT

public:
  ToAvoidLKASFaultToggle() : ToggleControl("To Avoid LKAS Fault", "to avoid LKAS fault above max angle limit(car specific). This is live value. Find out your maxframe while driving.", "../assets/offroad/icon_shell.png", Params().getBool("AvoidLKASFaultEnabled")) {
    QObject::connect(this, &ToAvoidLKASFaultToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AvoidLKASFaultEnabled", status);
    });
  }
};

class ToAvoidLKASFaultBeyondToggle : public ToggleControl {
  Q_OBJECT

public:
  ToAvoidLKASFaultBeyondToggle() : ToggleControl("To Avoid LKAS Fault with More Steer", "This is just in case you are using other panda setting.(delta up&down, maxsteer, rtdelta and etc).", "../assets/offroad/icon_shell.png", Params().getBool("AvoidLKASFaultBeyond")) {
    QObject::connect(this, &ToAvoidLKASFaultBeyondToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AvoidLKASFaultBeyond", status);
    });
  }
};

class StockDecelonCamToggle : public ToggleControl {
  Q_OBJECT

public:
  StockDecelonCamToggle() : ToggleControl("Use Stock Decel on SaftySection", "Use stock deceleration on safety section.(the vehicle equipped with Stock Navigation)", "../assets/offroad/icon_shell.png", Params().getBool("UseStockDecelOnSS")) {
    QObject::connect(this, &StockDecelonCamToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UseStockDecelOnSS", status);
    });
  }
};

class JoystickModeToggle : public ToggleControl {
  Q_OBJECT

public:
  JoystickModeToggle() : ToggleControl("JoyStick Debug Mode", "How to use: https://github.com/commaai/openpilot/tree/master/tools/joystick", "../assets/offroad/icon_shell.png", Params().getBool("JoystickDebugMode")) {
    QObject::connect(this, &JoystickModeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("JoystickDebugMode", status);
    });
  }
};

class RPMAnimatedToggle : public ToggleControl {
  Q_OBJECT

public:
  RPMAnimatedToggle() : ToggleControl("RPM Animated", "Animated RPM", "../assets/offroad/icon_shell.png", Params().getBool("AnimatedRPM")) {
    QObject::connect(this, &RPMAnimatedToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AnimatedRPM", status);
    });
  }
};

class ShowStopLineToggle : public ToggleControl {
  Q_OBJECT

public:
  ShowStopLineToggle() : ToggleControl("Show Stop Line", "Show stop line on the screen.", "../assets/offroad/icon_shell.png", Params().getBool("ShowStopLine")) {
    QObject::connect(this, &ShowStopLineToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("ShowStopLine", status);
    });
  }
};

class NoSmartMDPSToggle : public ToggleControl {
  Q_OBJECT

public:
  NoSmartMDPSToggle() : ToggleControl("No Smart MDPS", "Turn on, if you have no smartmdps or no mdps harness to avoid sending can under certain speed that is not able to use lane keeping.", "../assets/offroad/icon_shell.png", Params().getBool("NoSmartMDPS")) {
    QObject::connect(this, &NoSmartMDPSToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("NoSmartMDPS", status);
    });
  }
};

class SpeedCameraOffsetToggle : public ToggleControl {
  Q_OBJECT

public:
  SpeedCameraOffsetToggle() : ToggleControl("Speed CameraOffset", "This increase offset at low speed and decrease offset at low speed. If you feel car moves to right at low speed.", "../assets/offroad/icon_shell.png", Params().getBool("SpeedCameraOffset")) {
    QObject::connect(this, &SpeedCameraOffsetToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("SpeedCameraOffset", status);
    });
  }
};

class HoldForSettingToggle : public ToggleControl {
  Q_OBJECT

public:
  HoldForSettingToggle() : ToggleControl("Hold Button for Setting Menu", "Use 0.5 sec delay to enter setting menu. This is to prevent being touched in setting menu when you use external program(Mixplorer, etc)", "../assets/offroad/icon_shell.png", Params().getBool("HoldForSetting")) {
    QObject::connect(this, &HoldForSettingToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("HoldForSetting", status);
    });
  }
};

class RTShieldToggle : public ToggleControl {
  Q_OBJECT

public:
  RTShieldToggle() : ToggleControl("Enable RTShield Process", "Seems regarding process stability. This uses resource.", "../assets/offroad/icon_shell.png", Params().getBool("RTShield")) {
    QObject::connect(this, &RTShieldToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RTShield", status);
    });
  }
};

class OSMOfflineUseToggle : public ToggleControl {
  Q_OBJECT

public:
  OSMOfflineUseToggle() : ToggleControl("Offline OSM(64G storage only)", "Local only OSM. This will download DBs from online.", "../assets/offroad/icon_shell.png", Params().getBool("OSMOfflineUse")) {
    QObject::connect(this, &OSMOfflineUseToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OSMOfflineUse", status);
    });
  }
};

// openpilot preview
class OpenpilotView : public AbstractControl {
  Q_OBJECT

public:
  OpenpilotView();

private:
  QPushButton btn;
  QPushButton btnc;
  Params params;
  
  void refresh();
};

class CarSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  CarSelectCombo();

private:
  QPushButton btn;
  QComboBox combobox;
  Params params;

  void refresh();
};

class BranchSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  BranchSelectCombo();
  void executeProgram1(const QString &tcmd1);

private slots:
  void printMsg1();
  void processFinished1(int exitCode, QProcess::ExitStatus exitStatus);

private:
  QPushButton btn;
  QComboBox combobox;
  Params params;

  QProcess *textMsgProcess1;
  QMessageBox *outbox1;
  QString outdata1;
  QString branch_name1;

  void refresh();
};

class TimeZoneSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  TimeZoneSelectCombo();

private:
  QPushButton btn;
  QComboBox combobox;
  Params params;

  void refresh();
};


// UI
class AutoShutdown : public AbstractControl {
  Q_OBJECT

public:
  AutoShutdown();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class ForceShutdown : public AbstractControl {
  Q_OBJECT

public:
  ForceShutdown();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoScreenOff : public AbstractControl {
  Q_OBJECT

public:
  AutoScreenOff();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VolumeControl : public AbstractControl {
  Q_OBJECT

public:
  VolumeControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  QSoundEffect effect;
  
  void refresh();
  void playsound();
};

class BrightnessControl : public AbstractControl {
  Q_OBJECT

public:
  BrightnessControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class BrightnessOffControl : public AbstractControl {
  Q_OBJECT

public:
  BrightnessOffControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};


class ChargingMin : public AbstractControl {
  Q_OBJECT

public:
  ChargingMin();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};
class ChargingMax : public AbstractControl {
  Q_OBJECT

public:
  ChargingMax();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};


// Driving
class CruisemodeSelInit : public AbstractControl {
  Q_OBJECT

public:
  CruisemodeSelInit();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VariableCruiseProfile : public AbstractControl {
  Q_OBJECT

public:
  VariableCruiseProfile();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LaneChangeSpeed : public AbstractControl {
  Q_OBJECT

public:
  LaneChangeSpeed();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LaneChangeDelay : public AbstractControl {
  Q_OBJECT

public:
  LaneChangeDelay();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LeftCurvOffset : public AbstractControl {
  Q_OBJECT

public:
  LeftCurvOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};
class RightCurvOffset : public AbstractControl {
  Q_OBJECT

public:
  RightCurvOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};



class SpeedLimitOffset : public AbstractControl {
  Q_OBJECT

public:
  SpeedLimitOffset();

private:
  QPushButton btn;
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

// 튜닝 설정
class CameraOffset : public AbstractControl {
  Q_OBJECT

public:
  CameraOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PathOffset : public AbstractControl {
  Q_OBJECT

public:
  PathOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SRBaseControl : public AbstractControl {
  Q_OBJECT

public:
  SRBaseControl();

private:
  QPushButton btndigit;
  QPushButton btnminus;
  QPushButton btnplus;
  QLabel label;
  Params params;
  float digit = 0.01;
  
  void refresh();
};

class SRMaxControl : public AbstractControl {
  Q_OBJECT

public:
  SRMaxControl();

private:
  QPushButton btndigit;
  QPushButton btnminus;
  QPushButton btnplus;
  QLabel label;
  Params params;
  float digit = 0.01;
  
  void refresh();
};

class SteerActuatorDelay : public AbstractControl {
  Q_OBJECT

public:
  SteerActuatorDelay();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerRateCost : public AbstractControl {
  Q_OBJECT

public:
  SteerRateCost();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerLimitTimer : public AbstractControl {
  Q_OBJECT

public:
  SteerLimitTimer();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TireStiffnessFactor : public AbstractControl {
  Q_OBJECT

public:
  TireStiffnessFactor();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerMax : public AbstractControl {
  Q_OBJECT

public:
  SteerMax();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

class SteerDeltaUp : public AbstractControl {
  Q_OBJECT

public:
  SteerDeltaUp();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

class SteerDeltaDown : public AbstractControl {
  Q_OBJECT

public:
  SteerDeltaDown();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

// control
class LateralControl : public AbstractControl {
  Q_OBJECT

public:
  LateralControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  int latcontrol;


  void refresh();
};

class PidKp : public AbstractControl {
  Q_OBJECT

public:
  PidKp();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PidKi : public AbstractControl {
  Q_OBJECT

public:
  PidKi();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PidKd : public AbstractControl {
  Q_OBJECT

public:
  PidKd();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PidKf : public AbstractControl {
  Q_OBJECT

public:
  PidKf();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class IgnoreZone : public AbstractControl {
  Q_OBJECT

public:
  IgnoreZone();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OuterLoopGain : public AbstractControl {
  Q_OBJECT

public:
  OuterLoopGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class InnerLoopGain : public AbstractControl {
  Q_OBJECT

public:
  InnerLoopGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TimeConstant : public AbstractControl {
  Q_OBJECT

public:
  TimeConstant();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class ActuatorEffectiveness : public AbstractControl {
  Q_OBJECT

public:
  ActuatorEffectiveness();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class Scale : public AbstractControl {
  Q_OBJECT

public:
  Scale();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LqrKi : public AbstractControl {
  Q_OBJECT

public:
  LqrKi();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class DcGain : public AbstractControl {
  Q_OBJECT

public:
  DcGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueKp : public AbstractControl {
  Q_OBJECT

public:
  TorqueKp();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueKf : public AbstractControl {
  Q_OBJECT

public:
  TorqueKf();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueKi : public AbstractControl {
  Q_OBJECT

public:
  TorqueKi();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueFriction : public AbstractControl {
  Q_OBJECT

public:
  TorqueFriction();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueUseAngle : public ToggleControl {
  Q_OBJECT

public:
  TorqueUseAngle() : ToggleControl("UseAngle", "Use Steer Angle On/Off", "../assets/offroad/icon_shell.png", Params().getBool("TorqueUseAngle")) {
    QObject::connect(this, &TorqueUseAngle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("TorqueUseAngle", status);
    });
  }
};

class TorqueMaxLatAccel : public AbstractControl {
  Q_OBJECT

public:
  TorqueMaxLatAccel();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueAngDeadZone : public AbstractControl {
  Q_OBJECT

public:
  TorqueAngDeadZone();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerAngleCorrection : public AbstractControl {
  Q_OBJECT

public:
  SteerAngleCorrection();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MaxSteer : public AbstractControl {
  Q_OBJECT

public:
  MaxSteer();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MaxRTDelta : public AbstractControl {
  Q_OBJECT

public:
  MaxRTDelta();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MaxRateUp : public AbstractControl {
  Q_OBJECT

public:
  MaxRateUp();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MaxRateDown : public AbstractControl {
  Q_OBJECT

public:
  MaxRateDown();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerThreshold : public AbstractControl {
  Q_OBJECT

public:
  SteerThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class RecordCount : public AbstractControl {
  Q_OBJECT

public:
  RecordCount();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class RecordQuality : public AbstractControl {
  Q_OBJECT

public:
  RecordQuality();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class GitHash : public AbstractControl {
  Q_OBJECT

public:
  GitHash();

private:
  QLabel local_hash;
  QLabel remote_hash;
  Params params;
};

class RESChoice : public AbstractControl {
  Q_OBJECT

public:
  RESChoice();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MonitoringMode : public AbstractControl {
  Q_OBJECT

public:
  MonitoringMode();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MonitorEyesThreshold : public AbstractControl {
  Q_OBJECT

public:
  MonitorEyesThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class NormalEyesThreshold : public AbstractControl {
  Q_OBJECT

public:
  NormalEyesThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class BlinkThreshold : public AbstractControl {
  Q_OBJECT

public:
  BlinkThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class FanSpeedGain : public AbstractControl {
  Q_OBJECT

public:
  FanSpeedGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CruiseGapTR : public AbstractControl {
  Q_OBJECT

public:
  CruiseGapTR();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  QLabel label1;
  QLabel label2;
  QLabel label3;
  QLabel label4;
  QLabel label1a;
  QLabel label2a;
  QLabel label3a;
  QLabel label4a;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refresh4();
};

class DynamicTRGap : public AbstractControl {
  Q_OBJECT

public:
  DynamicTRGap();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class DynamicTRUD : public AbstractControl {
  Q_OBJECT

public:
  DynamicTRUD();
};

class LCTimingFactor : public AbstractControl {
  Q_OBJECT

public:
  LCTimingFactor();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  QLabel label1;
  QLabel label2;
  QLabel label3;
  QLabel label4;
  QLabel label1a;
  QLabel label2a;
  QLabel label3a;
  QLabel label4a;
  Params params;

  void refresh1();
  void refresh2();
  void refresh3();
  void refresh4();
};

class LCTimingFactorUD : public AbstractControl {
  Q_OBJECT

public:
  LCTimingFactorUD();

private:
  QPushButton btn;
  QPushButton btn2;
  Params params;
  
  void refresh();
  void refresh2();
};

class AutoResCondition : public AbstractControl {
  Q_OBJECT

public:
  AutoResCondition();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoResLimitTime : public AbstractControl {
  Q_OBJECT

public:
  AutoResLimitTime();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoEnableSpeed : public AbstractControl {
  Q_OBJECT

public:
  AutoEnableSpeed();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CamDecelDistAdd : public AbstractControl {
  Q_OBJECT

public:
  CamDecelDistAdd();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LiveSRPercent : public AbstractControl {
  Q_OBJECT

public:
  LiveSRPercent();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VCurvSpeedUD : public AbstractControl {
  Q_OBJECT

public:
  VCurvSpeedUD();
};

class VCurvSpeed : public AbstractControl {
  Q_OBJECT

public:
  VCurvSpeed();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class OCurvSpeedUD : public AbstractControl {
  Q_OBJECT

public:
  OCurvSpeedUD();
};

class OCurvSpeed : public AbstractControl {
  Q_OBJECT

public:
  OCurvSpeed();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class GetOffAlert : public AbstractControl {
  Q_OBJECT

public:
  GetOffAlert();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OPKRNaviSelect : public AbstractControl {
  Q_OBJECT

public:
  OPKRNaviSelect();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OPKRServerSelect : public AbstractControl {
  Q_OBJECT

public:
  OPKRServerSelect();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh();
};

class OPKRServerAPI : public AbstractControl {
  Q_OBJECT

public:
  OPKRServerAPI();

private:
  QLabel label;
  QPushButton btn;
  Params params;

  void refresh();
};

class OPKRMapboxStyle : public AbstractControl {
  Q_OBJECT

public:
  OPKRMapboxStyle();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh();
};

class RESCountatStandstill : public AbstractControl {
  Q_OBJECT

public:
  RESCountatStandstill();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SpeedLimitSignType : public AbstractControl {
  Q_OBJECT

public:
  SpeedLimitSignType();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class RadarLongHelperOption : public AbstractControl {
  Q_OBJECT

public:
  RadarLongHelperOption();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CurvDecelSelect : public AbstractControl {
  Q_OBJECT

public:
  CurvDecelSelect();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoRESDelay : public AbstractControl {
  Q_OBJECT

public:
  AutoRESDelay();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OSMCustomSpeedLimitUD : public AbstractControl {
  Q_OBJECT

public:
  OSMCustomSpeedLimitUD();
};

class OSMCustomSpeedLimit : public AbstractControl {
  Q_OBJECT

public:
  OSMCustomSpeedLimit();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class DesiredCurvatureLimit : public AbstractControl {
  Q_OBJECT

public:
  DesiredCurvatureLimit();

private:
  QPushButton btndigit;
  QPushButton btnminus;
  QPushButton btnplus;
  QLabel label;
  Params params;
  float digit = 0.01;
  
  void refresh();
};

class DynamicTRBySpeed : public AbstractControl {
  Q_OBJECT

public:
  DynamicTRBySpeed();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class LaneWidth : public AbstractControl {
  Q_OBJECT

public:
  LaneWidth();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SpeedLaneWidthUD : public AbstractControl {
  Q_OBJECT

public:
  SpeedLaneWidthUD();
};

class SpeedLaneWidth : public AbstractControl {
  Q_OBJECT

public:
  SpeedLaneWidth();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class OPKRTopTextView : public AbstractControl {
  Q_OBJECT

public:
  OPKRTopTextView();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OPKREdgeOffset : public AbstractControl {
  Q_OBJECT

public:
  OPKREdgeOffset();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

class ToAvoidLKASFault : public AbstractControl {
  Q_OBJECT

public:
  ToAvoidLKASFault();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

class RoutineDriveOption : public AbstractControl {
  Q_OBJECT

public:
  RoutineDriveOption();

private:
  QPushButton btn0;
  QPushButton btn1;
  Params params;
  
  void refresh();
};

class RPMAnimatedMaxValue : public AbstractControl {
  Q_OBJECT

public:
  RPMAnimatedMaxValue();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class UserSpecificFeature : public AbstractControl {
  Q_OBJECT

public:
  UserSpecificFeature();

private:
  QPushButton btn;
  QLineEdit edit;
  Params params;

  void refresh();
};

class MultipleLatSelect : public AbstractControl {
  Q_OBJECT

public:
  MultipleLatSelect();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  int    m_nMethod;


  void refresh();
};

class MultipleLateralSpeed : public AbstractControl {
  Q_OBJECT

public:
  MultipleLateralSpeed();

private:
  QLabel label1;
  QPushButton btnplusl;
  QLabel labell;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QLabel labelr;
  QPushButton btnminusr;
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refreshl();
  void refreshr();
};

class MultipleLateralAngle : public AbstractControl {
  Q_OBJECT

public:
  MultipleLateralAngle();

private:
  QLabel label1;
  QPushButton btnplusl;
  QLabel labell;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QLabel labelr;
  QPushButton btnminusr;
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refreshl();
  void refreshr();
};

class StoppingDist : public AbstractControl {
  Q_OBJECT

public:
  StoppingDist();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};