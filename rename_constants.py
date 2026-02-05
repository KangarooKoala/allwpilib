#!/usr/bin/env python3

import re
import os
import os.path
import sys
import time

from pathlib import Path


IGNORE_NAMES: frozenset[str] = frozenset(
    (
        # wpimath
        "k180deg",
        "kAAngular",
        "kALinear",
        "kVAngular",
        "kVLinear",
        # wpinet
        "kDNSServiceErr_NoError",
        "kDNSServiceFlagsAdd",
        "kDNSServiceProtocol_IPv4",
        # cscore
        "kCFAllocatorDefault",
        "kCFRunLoopDefaultMode",
        "kCVPixelBufferPixelFormatTypeKey",
        "kCVPixelFormatType_32BGRA",
        "kCVPixelFormatType_422YpCbCr8FullRange",
        "kCVPixelFormatType_422YpCbCr8_yuvs",
        "kIOCFPlugInInterfaceID",
        "kIOMainPortDefault",
        "kIOReturnSuccess",
        "kIOUSBConfigNotFound",
        "kIOUSBDeviceClassName",
        "kIOUSBDeviceInterfaceID",
        "kIOUSBDeviceUserClientTypeID",
        "kIOUSBEndpointNotFound",
        "kIOUSBFindInterfaceDontCare",
        "kIOUSBInterfaceInterfaceID",
        "kIOUSBInterfaceNotFound",
        "kIOUSBInterfaceUserClientTypeID",
        "kIOUSBPipeStalled",
        "kIOUSBTooManyPipesErr",
        "kIOUSBUnknownPipeErr",
        "kPixelRGBA", # TODO Ignore only gui::kPixelRGBA (wpigui defines a kPixelRGBA)
        "kUSBClass",
        "kUSBIn",
        "kUSBInterface",
        "kUSBOut",
    )
)


def convert_name(name: str) -> tuple[str, bool]:
    new_name: str = re.sub(r"([a-z])([A-Z])", r"\1_\2", name[1:]).upper()
    is_not_simple: bool = bool(re.search(r"[A-Z][A-Z]", name) or re.search("[0-9]", name))
    return (new_name, is_not_simple)


def make_substitution_table() -> dict[str, str]:
    def normal_substitutions(*old_constant_names: tuple[str, ...]) -> dict[str, str]:
        ret: dict[str, str] = {}
        for name in old_constant_names:
            new_name, is_not_simple = convert_name(name)
            assert not is_not_simple
            ret[name] = new_name
        return ret

    base_dict: dict[str, str] = {
        # wpiutil
        'kBGR': 'BGR',
        'kBGRA': 'BGRA',
        'kHandleTypeCSBase': 'HANDLE_TYPE_CS_BASE',
        'kHandleTypeHALBase': 'HANDLE_TYPE_HAL_BASE',
        'kHandleTypeNTBase': 'HANDLE_TYPE_NT_BASE',
        'kInt16': 'INT16',
        'kInt32': 'INT32',
        'kInt64': 'INT64',
        'kInt8': 'INT8',
        'kMJPEG': 'MJPEG',
        'kRGB565': 'RGB565',
        'kSizeInt16': 'SIZE_INT16',
        'kSizeInt32': 'SIZE_INT32',
        'kSizeInt64': 'SIZE_INT64',
        'kSizeInt8': 'SIZE_INT8',
        'kUYVY': 'UYVY',
        'kUint16': 'UINT16',
        'kUint32': 'UINT32',
        'kUint64': 'UINT64',
        'kUint8': 'UINT8',
        'kV4LEOF': 'V4L_EOF',
        'kV4LSOE': 'V4L_SOE',
        'kY16': 'Y16',
        'kYUYV': 'YUYV',
        # fields
        'k2018PowerUp': 'POWER_UP_2018',
        'k2019DeepSpace': 'DEEP_SPACE_2019',
        'k2020InfiniteRecharge': 'INFINITE_RECHARGE_2020',
        'k2021Barrel': 'BARREL_2021',
        'k2021Bounce': 'BOUNCE_2021',
        'k2021GalacticSearchA': 'GALACTIC_SEARCH_A_2021',
        'k2021GalacticSearchB': 'GALACTIC_SEARCH_B_2021',
        'k2021InfiniteRecharge': 'INFINITE_RECHARGE_2021',
        'k2021Slalom': 'SLALOM_2021',
        'k2022RapidReact': 'RAPID_REACT_2022',
        'k2023ChargedUp': 'CHARGED_UP_2023',
        'k2024Crescendo': 'CRESCENDO_2024',
        'k2025Reefscape': 'REEFSCAPE_2025',
        # cscore
        'kCSCore': 'CSCORE',
        'kCmdSetFPS': 'CMD_SET_FPS',
        'kFixedFlourescent2': 'FIXED_FLOURESCENT2',
        'kFixedFluorescent1': 'FIXED_FLUORESCENT1',
        'kFixedOutdoor1': 'FIXED_OUTDOOR1',
        'kFixedOutdoor2': 'FIXED_OUTDOOR2',
        'kMJPGStreamer': 'MJPG_STREAMER',
        # wpimath
        'kAOff': 'A_OFF',
        'kBOff': 'B_OFF',
        'kCCW_90deg': 'CCW_90_DEG',
        'kCCW_Pi_2': 'CCW_PI_2',
        'kCOff': 'C_OFF',
        'kCW_90deg': 'CW_90_DEG',
        'kCW_Pi_2': 'CW_PI_2',
        'kDOff': 'D_OFF',
        'kQOff': 'Q_OFF',
        'kSinglePoleIIRExpectedOutput': 'SINGLE_POLE_IIR_EXPECTED_OUTPUT',
        'kSinglePoleIIRTimeConstant': 'SINGLE_POLE_IIR_TIME_CONSTANT',
        'kTestSinglePoleIIR': 'TEST_SINGLE_POLE_IIR',
        'kWOff': 'W_OFF',
        'kXFinalOff': 'X_FINAL_OFF',
        'kXInitialOff': 'X_INITIAL_OFF',
        'kXOff': 'X_OFF',
        'kXSemiAxisOff': 'X_SEMI_AXIS_OFF',
        'kXWidthOff': 'X_WIDTH_OFF',
        'kYFinalOff': 'Y_FINAL_OFF',
        'kYInitialOff': 'Y_INITIAL_OFF',
        'kYOff': 'Y_OFF',
        'kYSemiAxisOff': 'Y_SEMI_AXIS_OFF',
        'kYWidthOff': 'Y_WIDTH_OFF',
        'kZOff': 'Z_OFF',
        # apriltag (lots of overlap with fields)
        'k2025ReefscapeAndyMark': 'REEFSCAPE_ANDYMARK_2025',
        'k2025ReefscapeWelded': 'REEFSCAPE_WELDED_2025',
        # ntcore
        'kDebug1': 'DEBUG1', # TODO DEBUG1 or DEBUG_1?
        'kDebug2': 'DEBUG2',
        'kDebug3': 'DEBUG3',
        'kDebug4': 'DEBUG4',
        'kFlagDebug1': 'FLAG_DEBUG1',
        'kFlagDebug2': 'FLAG_DEBUG2',
        'kFlagDebug3': 'FLAG_DEBUG3',
        'kFlagDebug4': 'FLAG_DEBUG4',
        'kLogDebug1': 'LOG_DEBUG1',
        'kLogDebug2': 'LOG_DEBUG2',
        'kLogDebug3': 'LOG_DEBUG3',
        'kLogDebug4': 'LOG_DEBUG4',
        'kMTU': 'MTU',

        **normal_substitutions(
            # wpiutil color
            'kAliceBlue',
            'kAntiqueWhite',
            'kAqua',
            'kAquamarine',
            'kAzure',
            'kBeige',
            'kBisque',
            'kBlack',
            'kBlanchedAlmond',
            'kBlue',
            'kBlueViolet',
            'kBrown',
            'kBurlywood',
            'kCadetBlue',
            'kChartreuse',
            'kChocolate',
            'kCoral',
            'kCornflowerBlue',
            'kCornsilk',
            'kCrimson',
            'kCyan',
            'kDarkBlue',
            'kDarkCyan',
            'kDarkGoldenrod',
            'kDarkGray',
            'kDarkGreen',
            'kDarkKhaki',
            'kDarkMagenta',
            'kDarkOliveGreen',
            'kDarkOrange',
            'kDarkOrchid',
            'kDarkRed',
            'kDarkSalmon',
            'kDarkSeaGreen',
            'kDarkSlateBlue',
            'kDarkSlateGray',
            'kDarkTurquoise',
            'kDarkViolet',
            'kDeepPink',
            'kDeepSkyBlue',
            'kDenim',
            'kDimGray',
            'kDodgerBlue',
            'kFirebrick',
            'kFirstBlue',
            'kFirstRed',
            'kFloralWhite',
            'kForestGreen',
            'kFuchsia',
            'kGainsboro',
            'kGhostWhite',
            'kGold',
            'kGoldenrod',
            'kGray',
            'kGreen',
            'kGreenYellow',
            'kHoneydew',
            'kHotPink',
            'kIndianRed',
            'kIndigo',
            'kIvory',
            'kKhaki',
            'kLavender',
            'kLavenderBlush',
            'kLawnGreen',
            'kLemonChiffon',
            'kLightBlue',
            'kLightCoral',
            'kLightCyan',
            'kLightGoldenrodYellow',
            'kLightGray',
            'kLightGreen',
            'kLightPink',
            'kLightSalmon',
            'kLightSeaGreen',
            'kLightSkyBlue',
            'kLightSlateGray',
            'kLightSteelBlue',
            'kLightYellow',
            'kLime',
            'kLimeGreen',
            'kLinen',
            'kMagenta',
            'kMaroon',
            'kMediumAquamarine',
            'kMediumBlue',
            'kMediumOrchid',
            'kMediumPurple',
            'kMediumSeaGreen',
            'kMediumSlateBlue',
            'kMediumSpringGreen',
            'kMediumTurquoise',
            'kMediumVioletRed',
            'kMidnightBlue',
            'kMintcream',
            'kMistyRose',
            'kMoccasin',
            'kNavajoWhite',
            'kNavy',
            'kOldLace',
            'kOlive',
            'kOliveDrab',
            'kOrange',
            'kOrangeRed',
            'kOrchid',
            'kPaleGoldenrod',
            'kPaleGreen',
            'kPaleTurquoise',
            'kPaleVioletRed',
            'kPapayaWhip',
            'kPeachPuff',
            'kPeru',
            'kPink',
            'kPlum',
            'kPowderBlue',
            'kPurple',
            'kRed',
            'kRosyBrown',
            'kRoyalBlue',
            'kSaddleBrown',
            'kSalmon',
            'kSandyBrown',
            'kSeaGreen',
            'kSeashell',
            'kSienna',
            'kSilver',
            'kSkyBlue',
            'kSlateBlue',
            'kSlateGray',
            'kSnow',
            'kSpringGreen',
            'kSteelBlue',
            'kTan',
            'kTeal',
            'kThistle',
            'kTomato',
            'kTurquoise',
            'kViolet',
            'kWheat',
            'kWhite',
            'kWhiteSmoke',
            'kYellow',
            'kYellowGreen',
            # wpiutil
            'kBool',
            'kChar',
            'kColon',
            'kComma',
            'kDouble',
            'kEndOfInput',
            'kEquals',
            'kExpectedData',
            'kFloat',
            'kFrameDequeue',
            'kHandleTypeEvent',
            'kHandleTypeSemaphore',
            'kHandleTypeUserBase',
            'kIdentifier',
            'kInteger',
            'kInvalidHandle',
            'kLeftBrace',
            'kLeftBracket',
            'kMapMode',
            'kNetworkTables',
            'kPriv',
            'kReadOnly',
            'kReadWrite',
            'kRightBrace',
            'kRightBracket',
            'kSemicolon',
            'kSizeBool',
            'kSizeDouble',
            'kSizeFloat',
            'kStruct',
            'kUnknown',
            # fields
            'kBaseResourceDir',
            'kDefaultField',
            'kFields',
            # datalog
            'kActive',
            'kBlockSize',
            'kBufferSize',
            'kControlFinish',
            'kControlSetMetadata',
            'kControlStart',
            'kDataType',
            'kGiB', # TODO Check: GI_B or GIB?
            'kKiB',
            'kMaxBufferCount',
            'kMaxFreeCount',
            'kMiB',
            'kMinFreeSpace',
            'kPaused',
            'kRecordMaxHeaderSize',
            'kStart',
            'kStopped',
            # wpinet
            'kArg',
            'kBinary',
            'kBinaryFragment',
            'kBoth',
            'kBoundary',
            'kClearFlags',
            'kConnectionClosed',
            'kConnectionReset',
            'kConnectionTimedOut',
            'kCwd',
            'kDefault',
            'kDefaultBacklog',
            'kDone',
            'kEnv',
            'kField',
            'kFinalFragment',
            'kFlagControl',
            'kFlagFin',
            'kFlagMasking',
            'kFragment',
            'kGid', # TODO Check
            'kLenMask',
            'kNoWait',
            'kNone',
            'kOnce',
            'kOpBinary',
            'kOpClose',
            'kOpCont',
            'kOpMask',
            'kOpPing',
            'kOpPong',
            'kOpText',
            'kPadding',
            'kPing',
            'kPong',
            'kReconnectTime',
            'kRequest',
            'kResponse',
            'kSetFlags',
            'kStatus',
            'kStdioCreatePipe',
            'kStdioIgnore',
            'kStdioInheritFd',
            'kStdioInheritPipe',
            'kText',
            'kTextFragment',
            'kUid',
            'kUrl',
            'kValue',
            'kWith',
            'kWithout',
            'kWouldBlock',
            'kWriteAllocSize',
            # cscore
            'kAutoManage',
            'kAxis',
            'kBoolean',
            'kCmdSetMode',
            'kCmdSetPath',
            'kCmdSetPixelFormat',
            'kCmdSetProperty',
            'kCmdSetPropertyStr',
            'kCmdSetResolution',
            'kCommand',
            'kConnectionAutoManage',
            'kConnectionForceClose',
            'kConnectionKeepOpen',
            'kCv',
            'kEnum',
            'kError',
            'kFixedIndoor',
            'kForceClose',
            'kGetSettings',
            'kGetSourceConfig',
            'kHttp',
            'kIndexMax',
            'kKeepOpen',
            'kListener',
            'kListenerPoller',
            'kMaxImagesAvail',
            'kMjpeg',
            'kNetworkInterfacesChanged',
            'kNumBuffers',
            'kNumSinksChanged',
            'kNumSinksEnabledChanged',
            'kOk',
            'kPropBrValue',
            'kPropConnectVerbose',
            'kPropConnectVerboseId',
            'kPropExAuto',
            'kPropExValue',
            'kPropWbAuto',
            'kPropWbValue',
            'kProperty',
            'kPropertyAutoExposure',
            'kPropertyAutoExposureOff',
            'kPropertyAutoExposureOn',
            'kPropertyAutoFocus',
            'kPropertyAutoWhiteBalance',
            'kPropertyBackLightCompensation',
            'kPropertyBrightness',
            'kPropertyContrast',
            'kPropertyExposure',
            'kPropertyFocus',
            'kPropertyGain',
            'kPropertyGamma',
            'kPropertyHue',
            'kPropertyPowerLineFrequency',
            'kPropertySaturation',
            'kPropertySharpness',
            'kPropertyWhiteBalance',
            'kPropertyZoom',
            'kRaw',
            'kRootPage',
            'kSink',
            'kSinkCreated',
            'kSinkDestroyed',
            'kSinkDisabled',
            'kSinkEnabled',
            'kSinkProperty',
            'kSinkPropertyChoicesUpdated',
            'kSinkPropertyCreated',
            'kSinkPropertyValueUpdated',
            'kSinkSourceChanged',
            'kSource',
            'kSourceBytesReceived',
            'kSourceConnected',
            'kSourceCreated',
            'kSourceDestroyed',
            'kSourceDisconnected',
            'kSourceFramesReceived',
            'kSourceProperty',
            'kSourcePropertyChoicesUpdated',
            'kSourcePropertyCreated',
            'kSourcePropertyValueUpdated',
            'kSourceVideoModeChanged',
            'kSourceVideoModesUpdated',
            'kStream',
            'kString',
            'kTelemetryUpdated',
            'kUndefined',
            'kUsb',
            'kUsbCamerasChanged',
            # wpimath
            'kA_t',
            'kAccelerationOff',
            'kAlphaOff',
            'kAngleOff',
            'kAngularA',
            'kAngularTolerance',
            'kAngularV',
            'kAxOff',
            'kAyOff',
            'kBufferDuration',
            'kCenterOff',
            'kDataOff',
            'kDim',
            'kDistanceOff',
            'kDoNothingTrajectory',
            'kDt',
            'kDtOff',
            'kDthetaOff',
            'kDxOff',
            'kDyOff',
            'kDzOff',
            'kEpsilon',
            'kFactor',
            'kFalling',
            'kFilterStep',
            'kFilterTime',
            'kFlip',
            'kFreeCurrentOff',
            'kFreeSpeedOff',
            'kFrontLeftOff',
            'kFrontRightOff',
            'kGoal',
            'kHeading',
            'kHighPassExpectedOutput',
            'kHighPassTimeConstant',
            'kInchesPerFoot',
            'kInputs',
            'kKaAngularOff',
            'kKaLinearOff',
            'kKaOff',
            'kKgOff',
            'kKilogramsPerLb',
            'kKsOff',
            'kKvAngularOff',
            'kKvLinearOff',
            'kKvOff',
            'kLeftOff',
            'kLeftVelocity',
            'kLinearA',
            'kLinearV',
            'kMalformedSplineExceptionMsg',
            'kMaxA',
            'kMaxDtheta',
            'kMaxDx',
            'kMaxDy',
            'kMaxIterations',
            'kMaxPastObserverStates',
            'kMaxV',
            'kMeasurement',
            'kMetersPerInch',
            'kMetersPerMile',
            'kMillisecondsPerSecond',
            'kMinutesPerHour',
            'kModulesOff',
            'kMovAvgExpectedOutput',
            'kMovAvgTaps',
            'kNominalVoltageOff',
            'kOmegaOff',
            'kOutputs',
            'kPi',
            'kPositionStddev',
            'kRange',
            'kRearLeftOff',
            'kRearRightOff',
            'kRightOff',
            'kRightVelocity',
            'kRising',
            'kRotationOff',
            'kRxOff',
            'kRyOff',
            'kRzOff',
            'kSchema',
            'kSecondsPerMinute',
            'kSetpoint',
            'kSpeedOff',
            'kStallCurrentOff',
            'kStallTorqueOff',
            'kStates',
            'kStdDev',
            'kTestData',
            'kTestHighPass',
            'kTestMovAvg',
            'kTestPulse',
            'kTolerance',
            'kTrackwidth',
            'kTrackwidthOff',
            'kTranslationOff',
            'kTypeName',
            'kV_t',
            'kValueOff',
            'kVisionUpdateDelay',
            'kVisionUpdateRate',
            'kVxOff',
            'kVyOff',
            'kWh', # TODO Check
            'kZero',
            # apriltag
            'kBlueAllianceWallRightSide',
            'kNumFields',
            'kRedAllianceWallRightSide',
            # ntcore
            'kAll',
            'kAllocSize',
            'kBooleanArray',
            'kClient',
            'kClientProcessMessageCountMax',
            'kConnected',
            'kConnection',
            'kConnectionDataLogger',
            'kCritical',
            'kDataLogger',
            'kDebug',
            'kDefaultPeriodic',
            'kDefaultPeriodicMs',
            'kDefaultPort',
            'kDefaultPubSubOptions',
            'kDefaultPubSubOptionsImpl',
            'kDisabled',
            'kDisconnected',
            'kDoubleArray',
            'kEmpty',
            'kEntry',
            'kFlagCritical',
            'kFlagDebug',
            'kFlagError',
            'kFlagInfo',
            'kFlagWarning',
            'kFloatArray',
            'kFlushThresholdBytes',
            'kFlushThresholdFrames',
            'kImmediate',
            'kInfo',
            'kInstance',
            'kIntegerArray',
            'kLocal',
            'kLogCritical',
            'kLogDebug',
            'kLogError',
            'kLogInfo',
            'kLogMessage',
            'kLogWarning',
            'kMaxImmProcessing',
            'kMaxListeners',
            'kMaxMessageSize',
            'kMaxMultiSubscribers',
            'kMaxPeriodMs',
            'kMaxPoolSize',
            'kMaxPublishers',
            'kMaxSubscribers',
            'kMethodStr',
            'kMinPeriodMs',
            'kMultiSubscriber',
            'kNetModeClient',
            'kNetModeLocal',
            'kNetModeNone',
            'kNetModeServer',
            'kNetModeStarting',
            'kNewFrameThresholdBytes',
            'kNormal',
            'kNumInstances',
            'kOutgoingLimit',
            'kPingIntervalMs',
            'kPingTimeoutMs',
            'kProperties',
            'kPublish',
            'kPublisher',
            'kReconnectRate',
            'kRttIntervalMs',
            'kServer',
            'kStarting',
            'kStringArray',
            'kSubscriber',
            'kTimeSync',
            'kTopic',
            'kType',
            'kTypeMax',
            'kTypeString',
            'kUnassigned',
            'kUnpublish',
            'kValueAll',
            'kValueLocal',
            'kValueRemote',
            'kWarning',
            'kWebsocketHandshakeTimeout',
        )
    }

    sorted_dict: dict[str, str] = {}
    for k, v in reversed(sorted(base_dict.items())):
        assert re.escape(k) == k
        assert re.escape(v) == v
        assert k not in IGNORE_NAMES
        sorted_dict[k] = v
    return sorted_dict


SUBSTITUTIONS: dict[str, str] = make_substitution_table()


old_constant_names: set[str] = set()


def perform_substitutions(content: str) -> str:
    for constant_name in frozenset(re.findall(r"\bk[A-Z0-9]\w+\b", content)):
        if constant_name in IGNORE_NAMES:
            continue
        if constant_name in SUBSTITUTIONS:
            new_constant_name: str = SUBSTITUTIONS[constant_name]
            content = re.sub(rf"\b{constant_name}\b", new_constant_name, content)
            continue
        old_constant_names.add(constant_name)

    return content


def files_in_directory(dirpath: Path, filenames: list[str], *, verbose: bool = True):
    if not filenames:
        return
    # Detect directory type
    valid_exts: tuple[str, ...] = ()
    kind: str
    if dirpath.parts[:2] == ("cscore", "examples"):
        valid_exts = (".cpp",)
        kind = "C++ (cscore example)"
    for i, part in enumerate(reversed(dirpath.parts)):
        i = len(dirpath.parts) - 1 - i
        if part == "java":
            valid_exts = (".java",)
            kind = "Java"
            break
        elif part == "semiwrap":
            valid_exts = (".yml",)
            kind = "Semiwrap"
            break
        elif part == "python":
            valid_exts = (".py")
            kind = "Python"
            break
        elif part == "objcpp":
            valid_exts = (".mm", ".hpp")
            kind = "Objective-C++"
            break
        elif part in ("cpp", "native") or part == "src" and "python" in dirpath.parts[:i]:
            # cpp is included to handle .../python/cpp/... paths
            # We also handle .../python/.../src/... paths
            valid_exts = (".c", ".cpp", ".cpp.inl", ".h", ".hpp", ".inc")
            kind = "C++"
            break
        elif part == "generate" and dirpath.parts[i - 1] == "src":
            valid_exts = (".json",)
            kind = "Generate"
            break
    if not valid_exts:
        quiet: bool = False
        if "src" not in dirpath.parts:
            quiet = True
        if len(dirpath.parts) == 4 and dirpath.parts[1] == "src" and dirpath.parts[-1] == "proto":
            # Something like wpiutil/src/main/proto
            quiet = True
        if verbose or not quiet:
            skipped_files_str: str = "1 file" if len(filenames) == 1 else f"{len(filenames)} files"
            print(f"Could not detect directory type for {dirpath}! Skipping {skipped_files_str}")
        return
    for f in filenames:
        if f.startswith("."):
            continue
        ending_check_f: str = f.removesuffix(".jinja") if "generate" in dirpath.parts else f
        if not any(ending_check_f.endswith(ext) for ext in valid_exts):
            quiet: bool = False
            quiet_exts: tuple[str, ...] = (".jpg", ".md", ".png")
            if any(f.endswith(quiet_ext) for quiet_ext in quiet_exts):
                quiet = True
            if kind == "Python" and (f.endswith(".toml") or f == "py.typed"):
                quiet = True
            if verbose or not quiet:
                print(f"Skipping non-{kind} file {dirpath / f}")
            continue
        if f in (
            "StringExtras.hpp",
            "StringExtras.cpp",
            "MemoryBuffer.cpp",
            "MemoryBuffer.hpp",
            "SmallVectorMemoryBuffer.hpp",
        ):
            print(f"Skipping LLVM file {dirpath / f}")
            continue
        yield dirpath / f


def files(*paths: tuple[str], verbose: bool = True):
    for path in paths:
        if os.path.isfile(path):
            # Explicitly specified file
            yield Path(path)
            continue
        for dp, dn, fn in os.walk(path):
            dp = Path(dp)
            # Report files
            yield from files_in_directory(dp, fn, verbose=verbose)
            # Control which directories we recurse into
            dn.sort()
            for bad_dir in ("resources", "thirdparty"):
                if bad_dir in dn:
                    if verbose:
                        print(f"Skipping bad directory {dp / bad_dir}")
                    dn.remove(bad_dir)


def main():
    VERBOSE: bool = False
    start: float = time.monotonic()

    total_file_count: int = 0
    changed_file_count: int = 0
    for file in files(*sys.argv[1:], verbose=VERBOSE):
        content: str
        try:
            with open(file, "r") as f:
                content = f.read()
        except:
            print(f"Error processing {file}!")
            raise

        old_content: str = content
        content = perform_substitutions(old_content)

        if content != old_content:
            changed_file_count += 1

        with open(file, "w") as f:
            f.write(content)

        total_file_count += 1

    end: float = time.monotonic()

    # Produce suggestions

    old_constant_names.difference_update(IGNORE_NAMES)

    complex_names: list[tuple[str, str]] = []
    simple_names: list[tuple[str, str]] = []
    for name in old_constant_names:
        new_name, is_not_simple = convert_name(name)
        (simple_names, complex_names)[is_not_simple].append((name, new_name))
    complex_names.sort()
    simple_names.sort()
    if complex_names:
        print("Complex names:")
        for old_name, new_name in complex_names:
            print(f"        {old_name!r}: {new_name!r},")
    if simple_names:
        print("Simple names:")
        for old_name, new_name in simple_names:
            print(f"            {old_name!r},")

    print(f"Visited {total_file_count} files ({changed_file_count} changed) in {end - start:.2f} s")
    # print(f"{chr(0x1b)}[43mTODO{chr(0x1b)}[m ...")


if __name__ == "__main__":
    main()
