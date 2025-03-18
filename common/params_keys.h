#pragma once

#include <string>
#include <unordered_map>

inline static std::unordered_map<std::string, uint32_t> keys = {
    {"AccessToken", CLEAR_ON_MANAGER_START | DONT_LOG},
    {"AdbEnabled", PERSISTENT},
    {"AlwaysOnDM", PERSISTENT},
    {"ApiCache_Device", PERSISTENT},
    {"ApiCache_FirehoseStats", PERSISTENT},
    {"AssistNowToken", PERSISTENT},
    {"AthenadPid", PERSISTENT},
    {"AthenadUploadQueue", PERSISTENT},
    {"AthenadRecentlyViewedRoutes", PERSISTENT},
    {"BootCount", PERSISTENT},
    {"CalibrationParams", PERSISTENT},
    {"CameraDebugExpGain", CLEAR_ON_MANAGER_START},
    {"CameraDebugExpTime", CLEAR_ON_MANAGER_START},
    {"CarBatteryCapacity", PERSISTENT},
    {"CarParams", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CarParamsCache", CLEAR_ON_MANAGER_START},
    {"CarParamsPersistent", PERSISTENT},
    {"CarParamsPrevRoute", PERSISTENT},
    {"CompletedTrainingVersion", PERSISTENT},
    {"ControlsReady", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CurrentBootlog", PERSISTENT},
    {"CurrentRoute", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"DisableLogging", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"DisablePowerDown", PERSISTENT | BACKUP},
    {"DisableUpdates", PERSISTENT | BACKUP},
    {"DisengageOnAccelerator", PERSISTENT | BACKUP},
    {"DongleId", PERSISTENT},
    {"DoReboot", CLEAR_ON_MANAGER_START},
    {"DoShutdown", CLEAR_ON_MANAGER_START},
    {"DoUninstall", CLEAR_ON_MANAGER_START},
    {"ExperimentalLongitudinalEnabled", PERSISTENT | DEVELOPMENT_ONLY | BACKUP},
    {"ExperimentalMode", PERSISTENT | BACKUP},
    {"ExperimentalModeConfirmed", PERSISTENT | BACKUP},
    {"FirmwareQueryDone", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"ForcePowerDown", PERSISTENT},
    {"GitBranch", PERSISTENT},
    {"GitCommit", PERSISTENT},
    {"GitCommitDate", PERSISTENT},
    {"GitDiff", PERSISTENT},
    {"GithubSshKeys", PERSISTENT | BACKUP},
    {"GithubUsername", PERSISTENT | BACKUP},
    {"GitRemote", PERSISTENT},
    {"GsmApn", PERSISTENT | BACKUP},
    {"GsmMetered", PERSISTENT | BACKUP},
    {"GsmRoaming", PERSISTENT | BACKUP},
    {"HardwareSerial", PERSISTENT},
    {"HasAcceptedTerms", PERSISTENT},
    {"InstallDate", PERSISTENT},
    {"IsDriverViewEnabled", CLEAR_ON_MANAGER_START},
    {"IsEngaged", PERSISTENT},
    {"IsLdwEnabled", PERSISTENT | BACKUP},
    {"IsMetric", PERSISTENT | BACKUP},
    {"IsOffroad", CLEAR_ON_MANAGER_START},
    {"IsOnroad", PERSISTENT},
    {"IsRhdDetected", PERSISTENT},
    {"IsReleaseBranch", CLEAR_ON_MANAGER_START},
    {"IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"IsTestedBranch", CLEAR_ON_MANAGER_START},
    {"JoystickDebugMode", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LanguageSetting", PERSISTENT | BACKUP},
    {"LastAthenaPingTime", CLEAR_ON_MANAGER_START},
    {"LastGPSPosition", PERSISTENT},
    {"LastManagerExitReason", CLEAR_ON_MANAGER_START},
    {"LastOffroadStatusPacket", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LastPowerDropDetected", CLEAR_ON_MANAGER_START},
    {"LastUpdateException", CLEAR_ON_MANAGER_START},
    {"LastUpdateTime", PERSISTENT},
    {"LiveParameters", PERSISTENT},
    {"LiveTorqueParameters", PERSISTENT | DONT_LOG},
    {"LocationFilterInitialState", PERSISTENT},
    {"LongitudinalManeuverMode", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LongitudinalPersonality", PERSISTENT | BACKUP},
    {"NetworkMetered", PERSISTENT},
    {"ObdMultiplexingChanged", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"ObdMultiplexingEnabled", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_BadNvme", CLEAR_ON_MANAGER_START},
    {"Offroad_CarUnrecognized", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_ConnectivityNeeded", CLEAR_ON_MANAGER_START},
    {"Offroad_ConnectivityNeededPrompt", CLEAR_ON_MANAGER_START},
    {"Offroad_IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"Offroad_NeosUpdate", CLEAR_ON_MANAGER_START},
    {"Offroad_NoFirmware", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_Recalibration", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_StorageMissing", CLEAR_ON_MANAGER_START},
    {"Offroad_TemperatureTooHigh", CLEAR_ON_MANAGER_START},
    {"Offroad_UnofficialHardware", CLEAR_ON_MANAGER_START},
    {"Offroad_UpdateFailed", CLEAR_ON_MANAGER_START},
    {"OpenpilotEnabledToggle", PERSISTENT | BACKUP},
    {"PandaHeartbeatLost", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"PandaSomResetTriggered", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"PandaSignatures", CLEAR_ON_MANAGER_START},
    {"PrimeType", PERSISTENT},
    {"RecordFront", PERSISTENT | BACKUP},
    {"RecordFrontLock", PERSISTENT},  // for the internal fleet
    {"SecOCKey", PERSISTENT | DONT_LOG},  // Candidate for | BACKUP
    {"RouteCount", PERSISTENT},
    {"SnoozeUpdate", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"SshEnabled", PERSISTENT | BACKUP},
    {"TermsVersion", PERSISTENT},
    {"TrainingVersion", PERSISTENT},
    {"UbloxAvailable", PERSISTENT},
    {"UpdateAvailable", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"UpdateFailedCount", CLEAR_ON_MANAGER_START},
    {"UpdaterAvailableBranches", PERSISTENT},
    {"UpdaterCurrentDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterCurrentReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterFetchAvailable", CLEAR_ON_MANAGER_START},
    {"UpdaterNewDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterNewReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterState", CLEAR_ON_MANAGER_START},
    {"UpdaterTargetBranch", CLEAR_ON_MANAGER_START},
    {"UpdaterLastFetchTime", PERSISTENT},
    {"Version", PERSISTENT},

    // --- sunnypilot params --- //
    {"ApiCache_DriveStats", PERSISTENT},
    {"CarParamsSP", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CarParamsSPCache", CLEAR_ON_MANAGER_START},
    {"CarParamsSPPersistent", PERSISTENT},
    {"CarPlatformBundle", PERSISTENT},
    {"EnableGithubRunner", PERSISTENT | BACKUP},
    {"LateralTorqueControlLateralJerk", PERSISTENT | BACKUP},
    {"ModelRunnerTypeCache", CLEAR_ON_ONROAD_TRANSITION},
    {"OffroadMode", CLEAR_ON_MANAGER_START},
    {"OffroadMode_Status", CLEAR_ON_MANAGER_START},

    // MADS params
    {"Mads", PERSISTENT | BACKUP},
    {"MadsMainCruiseAllowed", PERSISTENT | BACKUP},
    {"MadsPauseLateralOnBrake", PERSISTENT | BACKUP},
    {"MadsUnifiedEngagementMode", PERSISTENT | BACKUP},

    // Model Manager params
    {"ModelManager_ActiveBundle", PERSISTENT},
    {"ModelManager_DownloadIndex", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"ModelManager_LastSyncTime", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"ModelManager_ModelsCache", PERSISTENT | BACKUP},

    // Neural Network Lateral Control
    {"NeuralNetworkLateralControl", PERSISTENT | BACKUP},

    // sunnylink params
    {"EnableSunnylinkUploader", PERSISTENT | BACKUP},
    {"LastSunnylinkPingTime", CLEAR_ON_MANAGER_START},
    {"SunnylinkCache_Roles", PERSISTENT},
    {"SunnylinkCache_Users", PERSISTENT},
    {"SunnylinkDongleId", PERSISTENT},
    {"SunnylinkdPid", PERSISTENT},
    {"SunnylinkEnabled", PERSISTENT},

    // sunnypilot car specific params
    {"HyundaiRadarTracks", PERSISTENT},
    {"HyundaiRadarTracksConfirmed", PERSISTENT},
    {"HyundaiRadarTracksPersistent", PERSISTENT},
    {"HyundaiRadarTracksToggle", PERSISTENT},

    {"DynamicExperimentalControl", PERSISTENT},
};
