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
        'kTestSinglePoleIIR': 'TEST_SINGLE_POLE_IIR',
        'kSinglePoleIIRTimeConstant': 'SINGLE_POLE_IIR_TIME_CONSTANT',
        'kSinglePoleIIRExpectedOutput': 'SINGLE_POLE_IIR_EXPECTED_OUTPUT',
        'kYInitialOff': 'Y_INITIAL_OFF',
        'kZOff': 'Z_OFF',
        'kYOff': 'Y_OFF',
        'kDOff': 'D_OFF',
        'kAOff': 'A_OFF',
        'kCOff': 'C_OFF',
        'kXSemiAxisOff': 'X_SEMI_AXIS_OFF',
        'kCW_90deg': 'CW_90_DEG',
        'kXFinalOff': 'X_FINAL_OFF',
        'kCCW_Pi_2': 'CCW_PI_2',
        'kXInitialOff': 'X_INITIAL_OFF',
        'kWOff': 'W_OFF',
        'kYFinalOff': 'Y_FINAL_OFF',
        'kXOff': 'X_OFF',
        'kXWidthOff': 'X_WIDTH_OFF',
        'kBOff': 'B_OFF',
        'kYWidthOff': 'Y_WIDTH_OFF',
        'kCCW_90deg': 'CCW_90_DEG',
        'kQOff': 'Q_OFF',
        'kYSemiAxisOff': 'Y_SEMI_AXIS_OFF',
        'kCW_Pi_2': 'CW_PI_2',
        'kInt8': 'INT8',
        'kV4LEOF': 'V4L_EOF',
        'kUint64': 'UINT64',
        'kSizeInt16': 'SIZE_INT16',
        'kHandleTypeNTBase': 'HANDLE_TYPE_NT_BASE',
        'kMJPEG': 'MJPEG',
        'kSizeInt8': 'SIZE_INT8',
        'kInt64': 'INT64',
        'kSizeInt32': 'SIZE_INT32',
        'kYUYV': 'YUYV',
        'kHandleTypeHALBase': 'HANDLE_TYPE_HAL_BASE',
        'kUint32': 'UINT32',
        'kSizeInt64': 'SIZE_INT64',
        'kUYVY': 'UYVY',
        'kInt32': 'INT32',
        'kUint8': 'UINT8',
        'kV4LSOE': 'V4L_SOE',
        'kBGR': 'BGR',
        'kRGB565': 'RGB565',
        'kY16': 'Y16',
        'kBGRA': 'BGRA',
        'kInt16': 'INT16',
        'kUint16': 'UINT16',
        'kHandleTypeCSBase': 'HANDLE_TYPE_CS_BASE',
        'k2022RapidReact': 'RAPID_REACT_2022',
        'k2023ChargedUp': 'CHARGED_UP_2023',
        'k2024Crescendo': 'CRESCENDO_2024',
        'k2025ReefscapeAndyMark': 'REEFSCAPE_ANDYMARK_2025',
        'k2025ReefscapeWelded': 'REEFSCAPE_WELDED_2025',
        # ntcore
        'kDebug1': 'DEBUG_1',
        'kDebug2': 'DEBUG_2',
        'kDebug3': 'DEBUG_3',
        'kDebug4': 'DEBUG_4',
        'kFlagDebug1': 'FLAG_DEBUG_1',
        'kFlagDebug2': 'FLAG_DEBUG_2',
        'kFlagDebug3': 'FLAG_DEBUG_3',
        'kFlagDebug4': 'FLAG_DEBUG_4',
        'kLogDebug1': 'LOG_DEBUG_1',
        'kLogDebug2': 'LOG_DEBUG_2',
        'kLogDebug3': 'LOG_DEBUG_3',
        'kLogDebug4': 'LOG_DEBUG_4',
        'kMTU': 'MTU',
        # cscore
        'kCFAllocatorDefault': 'CF_ALLOCATOR_DEFAULT',
        'kCFRunLoopDefaultMode': 'CF_RUN_LOOP_DEFAULT_MODE',
        'kCSCore': 'CSCORE',
        'kCVPixelBufferPixelFormatTypeKey': 'CVPIXEL_BUFFER_PIXEL_FORMAT_TYPE_KEY', # TODO ??
        'kCVPixelFormatType_32BGRA': 'CVPIXEL_FORMAT_TYPE_32BGRA',
        'kCVPixelFormatType_422YpCbCr8FullRange': 'CVPIXEL_FORMAT_TYPE_422YP_CB_CR8FULL_RANGE',
        'kCVPixelFormatType_422YpCbCr8_yuvs': 'CVPIXEL_FORMAT_TYPE_422YP_CB_CR8_YUVS',
        'kCmdSetFPS': 'CMD_SET_FPS',
        'kFixedFlourescent2': 'FIXED_FLOURESCENT2',
        'kFixedFluorescent1': 'FIXED_FLUORESCENT1',
        'kFixedOutdoor1': 'FIXED_OUTDOOR1',
        'kFixedOutdoor2': 'FIXED_OUTDOOR2',
        'kIOCFPlugInInterfaceID': 'IOCF_PLUG_IN_INTERFACE_ID',
        'kIOMainPortDefault': 'IO_MAIN_PORT_DEFAULT',
        'kIOReturnSuccess': 'IO_RETURN_SUCCESS',
        'kIOUSBConfigNotFound': 'IO_USB_CONFIG_NOT_FOUND',
        'kIOUSBDeviceClassName': 'IO_USB_DEVICE_CLASS_NAME',
        'kIOUSBDeviceInterfaceID': 'IO_USB_DEVICE_INTERFACE_ID',
        'kIOUSBDeviceUserClientTypeID': 'IO_USB_DEVICE_USER_CLIENT_TYPE_ID',
        'kIOUSBEndpointNotFound': 'IO_USB_ENDPOINT_NOT_FOUND',
        'kIOUSBFindInterfaceDontCare': 'IO_USB_FIND_INTERFACE_DONT_CARE',
        'kIOUSBInterfaceInterfaceID': 'IO_USB_INTERFACE_INTERFACE_ID',
        'kIOUSBInterfaceNotFound': 'IO_USB_INTERFACE_NOT_FOUND',
        'kIOUSBInterfaceUserClientTypeID': 'IO_USB_INTERFACE_USER_CLIENT_TYPE_ID',
        'kIOUSBPipeStalled': 'IO_USB_PIPE_STALLED',
        'kIOUSBTooManyPipesErr': 'IO_USB_TOO_MANY_PIPES_ERR',
        'kIOUSBUnknownPipeErr': 'IO_USB_UNKNOWN_PIPE_ERR',
        'kMJPGStreamer': 'MJPG_STREAMER',
        'kUSBClass': 'USB_CLASS',
        'kUSBIn': 'USB_IN',
        'kUSBInterface': 'USB_INTERFACE',
        'kUSBOut': 'USB_OUT',

        **normal_substitutions(
            # cscore
            'kAutoManage',
            'kAxis',
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
            'kFixedIndoor',
            'kForceClose',
            'kGetSettings',
            'kGetSourceConfig',
            'kHttp',
            'kKeepOpen',
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
            'kTelemetryUpdated',
            'kUndefined',
            'kUsb',
            'kUsbCamerasChanged',
            # ntcore
            'kAll',
            'kAllocSize',
            'kBoolean',
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
            'kError',
            'kFlagCritical',
            'kFlagDebug',
            'kFlagError',
            'kFlagInfo',
            'kFlagWarning',
            'kFloatArray',
            'kFlushThresholdBytes',
            'kFlushThresholdFrames',
            'kImmediate',
            'kIndexMax',
            'kInfo',
            'kInstance',
            'kIntegerArray',
            'kListener',
            'kListenerPoller',
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
            'kRaw',
            'kReconnectRate',
            'kRttIntervalMs',
            'kServer',
            'kStarting',
            'kString',
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
            # datalog
            'kActive',
            'kBlockSize',
            'kBufferSize',
            'kControlFinish',
            'kControlSetMetadata',
            'kControlStart',
            'kDataType',
            'kGiB', # TODO Check
            'kKiB',
            'kMaxBufferCount',
            'kMaxFreeCount',
            'kMiB',
            'kMinFreeSpace',
            'kPaused',
            'kRecordMaxHeaderSize',
            'kStopped',
            # wpinet
            'kArg',
            'kBinary',
            'kBinaryFragment',
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
            'kGid',
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
            'kStart',
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
            # other stuff
            'kBlueAllianceWallRightSide',
            'kRedAllianceWallRightSide',
            'kBaseResourceDir',
            'kNumFields',
            'kDefaultField',
            'kStdDev',
            'kHighPassTimeConstant',
            'kTolerance',
            'kFilterStep',
            'kSetpoint',
            'kFilterTime',
            'kDt',
            'kVisionUpdateRate',
            'kMeasurement',
            'kFactor',
            'kVisionUpdateDelay',
            'kAngularV',
            'kAngularTolerance',
            'kTestMovAvg',
            'kRange',
            'kRightVelocity',
            'kPositionStddev',
            'kGoal',
            'kTestHighPass',
            'kTrackwidth',
            'kMovAvgTaps',
            'kMovAvgExpectedOutput',
            'kLeftVelocity',
            'kLinearA', # TODO Check
            'kAngularA',
            'kTestData',
            'kHeading',
            'kLinearV',
            'kHighPassExpectedOutput',
            'kTestPulse',
            'kFrontLeftOff',
            'kRearRightOff',
            'kKgOff',
            'kFreeSpeedOff',
            'kRearLeftOff',
            'kKvAngularOff',
            'kBoth',
            'kSecondsPerMinute',
            'kInputs',
            'kKaOff',
            'kTypeName',
            'kInchesPerFoot',
            'kSpeedOff',
            'kDoNothingTrajectory',
            'kModulesOff',
            'kValueOff',
            'kAlphaOff',
            'kMaxDtheta',
            'kMinutesPerHour',
            'kOmegaOff',
            'kFlip',
            'kKvLinearOff',
            'kTrackwidthOff',
            'kMaxV',
            'kWh',
            'kVxOff',
            'kDataOff',
            'kV_t',
            'kRyOff',
            'kAccelerationOff',
            'kVyOff',
            'kOutputs',
            'kKaAngularOff',
            'kTranslationOff',
            'kEpsilon',
            'kDthetaOff',
            'kMillisecondsPerSecond',
            'kAngleOff',
            'kDtOff',
            'kRightOff',
            'kMaxA',
            'kBufferDuration',
            'kKilogramsPerLb',
            'kSchema',
            'kZero',
            'kMaxDy',
            'kMetersPerInch',
            'kRising',
            'kDyOff',
            'kStallTorqueOff',
            'kNominalVoltageOff',
            'kKaLinearOff',
            'kMaxPastObserverStates',
            'kAyOff',
            'kStates',
            'kAxOff',
            'kA_t',
            'kMaxIterations',
            'kFreeCurrentOff',
            'kLeftOff',
            'kDistanceOff',
            'kCenterOff',
            'kFrontRightOff',
            'kDzOff',
            'kKsOff',
            'kDxOff',
            'kRxOff',
            'kMalformedSplineExceptionMsg',
            'kDim',
            'kPi',
            'kMetersPerMile',
            'kFalling',
            'kKvOff',
            'kRzOff',
            'kMaxDx',
            'kStallCurrentOff',
            'kRotationOff',
            'kExpectedData',
            'kMapMode',
            'kDarkKhaki',
            'kGhostWhite',
            'kDarkGreen',
            'kLightBlue',
            'kWheat',
            'kInvalidHandle',
            'kSpringGreen',
            'kDarkOrchid',
            'kLightSteelBlue',
            'kEquals',
            'kLemonChiffon',
            'kBlanchedAlmond',
            'kBool',
            'kMediumAquamarine',
            'kEndOfInput',
            'kMediumVioletRed',
            'kPlum',
            'kDarkViolet',
            'kPapayaWhip',
            'kSeaGreen',
            'kBisque',
            'kDarkSeaGreen',
            'kMediumOrchid',
            'kStruct',
            'kMediumSlateBlue',
            'kSizeFloat',
            'kUnknown',
            'kFirebrick',
            'kPink',
            'kIvory',
            'kBlue',
            'kDenim',
            'kGainsboro',
            'kLightGray',
            'kLightYellow',
            'kSnow',
            'kCoral',
            'kRosyBrown',
            'kViolet',
            'kMediumBlue',
            'kMediumSpringGreen',
            'kNavajoWhite',
            'kFirstRed',
            'kBurlywood',
            'kDarkGray',
            'kLinen',
            'kAliceBlue',
            'kSaddleBrown',
            'kAzure',
            'kPaleVioletRed',
            'kLimeGreen',
            'kComma',
            'kSienna',
            'kForestGreen',
            'kHotPink',
            'kAntiqueWhite',
            'kCyan',
            'kMoccasin',
            'kInteger',
            'kDarkTurquoise',
            'kIdentifier',
            'kTeal',
            'kDeepSkyBlue',
            'kLeftBrace',
            'kLavenderBlush',
            'kCadetBlue',
            'kSizeBool',
            'kCornflowerBlue',
            'kLime',
            'kIndigo',
            'kDarkOliveGreen',
            'kSlateBlue',
            'kFloat',
            'kGreen',
            'kAqua',
            'kPaleGreen',
            'kMagenta',
            'kPaleTurquoise',
            'kYellowGreen',
            'kFuchsia',
            'kBeige',
            'kKhaki',
            'kDimGray',
            'kDarkSlateGray',
            'kLightSeaGreen',
            'kBlueViolet',
            'kRoyalBlue',
            'kColon',
            'kDarkMagenta',
            'kOrchid',
            'kMediumSeaGreen',
            'kIndianRed',
            'kSilver',
            'kLightSlateGray',
            'kPeachPuff',
            'kDarkBlue',
            'kLightPink',
            'kThistle',
            'kWhiteSmoke',
            'kBlack',
            'kReadWrite',
            'kGray',
            'kMediumTurquoise',
            'kGreenYellow',
            'kSteelBlue',
            'kHandleTypeEvent',
            'kYellow',
            'kHandleTypeSemaphore',
            'kLeftBracket',
            'kLightGoldenrodYellow',
            'kSkyBlue',
            'kDarkCyan',
            'kCornsilk',
            'kLawnGreen',
            'kSizeDouble',
            'kFloralWhite',
            'kLightSalmon',
            'kPaleGoldenrod',
            'kTomato',
            'kWhite',
            'kBrown',
            'kDarkSlateBlue',
            'kLightGreen',
            'kPeru',
            'kMistyRose',
            'kPowderBlue',
            'kGold',
            'kSlateGray',
            'kLightCyan',
            'kCrimson',
            'kOrange',
            'kSeashell',
            'kDouble',
            'kFirstBlue',
            'kRightBracket',
            'kDarkGoldenrod',
            'kDeepPink',
            'kOrangeRed',
            'kOliveDrab',
            'kNavy',
            'kRightBrace',
            'kOlive',
            'kGoldenrod',
            'kDarkRed',
            'kDarkSalmon',
            'kSalmon',
            'kNetworkTables',
            'kDarkOrange',
            'kHoneydew',
            'kChartreuse',
            'kSandyBrown',
            'kLavender',
            'kReadOnly',
            'kDodgerBlue',
            'kSemicolon',
            'kFrameDequeue',
            'kLightCoral',
            'kOldLace',
            'kRed',
            'kHandleTypeUserBase',
            'kAquamarine',
            'kMintcream',
            'kPriv',
            'kLightSkyBlue',
            'kTurquoise',
            'kMidnightBlue',
            'kMaroon',
            'kPurple',
            'kMediumPurple',
            'kChocolate',
            'kChar',
            'kTan',
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
