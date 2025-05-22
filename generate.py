#!/usr/bin/env python3

import re
import time
from collections.abc import Callable
from pathlib import Path
from typing import TypeAlias


# Producing substitution dictionaries

def add_prefix(prefix: str, substitution_strs: dict[str, str]) -> dict[str, str]:
    return {prefix + key: prefix + value for key, value in substitution_strs.items()}


Substitution: TypeAlias = str | tuple[str, str] | dict[str, str]
PrintFunction: TypeAlias = Callable[[str], None]


def make_substitution_strs(*substitutions: tuple[Substitution], convert: Callable[[str], str] = lambda s: s, output: PrintFunction = print) -> dict[str, str]:
    substitution_strs: dict[str, str] = {}
    for substitution in substitutions:
        if isinstance(substitution, str):
            name: str = substitution
            value: str = convert(name)
            substitution_strs[name] = value
        elif isinstance(substitution, tuple):
            name, value = substitution
            if value == convert(name):
                output(f"Warning! {name} has redundant explicit value {value}!")
            substitution_strs[name] = value
        elif isinstance(substitution, dict):
            substitution_strs.update(substitution)
        else:
            output(f"Error! Unknown type of substitution {substitution}!")
    return substitution_strs


def remove_k(*substitutions: tuple[Substitution], output: PrintFunction = print) -> dict[str, str]:
    def convert(name: str) -> str:
        return name.replace("_k", "_", 1)
    return make_substitution_strs(*substitutions, convert=convert, output=output)


def normal_substitute(*substitutions: tuple[Substitution], output: PrintFunction = print) -> dict[str, str]:
    word_start = re.compile("((?<=[a-z])[A-Z]|(?<=[A-Z])[A-Z](?=[a-z]))")
    def convert(name: str) -> str:
        value: str = name
        value = value.removeprefix("k")
        value = re.sub(word_start, r"_\1", value)
        value = value.upper()
        return value
    return make_substitution_strs(*substitutions, convert=convert, output=output)

# Compiling substitution list

def compile_substitutions(substitution_strs: dict[str, str]):
    substitutions: list[tuple[str, str]] = []
    for key in reversed(sorted(substitution_strs.keys(), key=len)):
        pattern: str = re.escape(key)
        if key[0].isalnum() or key[0] == "_":
            pattern = r"\b" + pattern
        if key[-1].isalnum() or key[-1] == "_":
            pattern = pattern + r"\b"
        substitutions.append((re.compile(pattern), substitution_strs[key]))
    return substitutions

# Performing substitutions

def substitute_file(
    file_path: Path, substitutions: list[tuple[re.Pattern, str]]
) -> bool:
    dirty: bool = False
    lines: list[str] = []
    with open(file_path, "r") as reader:
        lines = reader.readlines()
    for i in range(len(lines)):
        for pattern, replacement in substitutions:
            new_line: str = pattern.sub(replacement, lines[i])
            if new_line != lines[i]:
                dirty = True
                lines[i] = new_line
    if dirty:
        with open(file_path, "w") as reader:
            reader.writelines(lines)
    return dirty


class Substitutor:
    def __init__(self, rootdir: Path):
        self.rootdir = rootdir
        self.__changed_files: set[str] = set()
        self.__num_tasks: int = 0
        self.__start_time: float | None = None
        self.__last_task: str = "<None>"
        self.__on_status_line: bool = False
        self.__preprocess_info: list[str] = []

    def __enter__(self):
        self.__start_time = time.monotonic()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        end_time = time.monotonic()
        duration: float = end_time - self.__start_time
        if exc_type is None:
            self.set_status(
                f"  Done processing {self.__num_tasks} tasks in {duration:.2f} s"
            )
        else:
            self.set_status(
                f"  Failed in {duration:.2f} s after processing task #{self.__num_tasks}: {self.__last_task}"
            )
        print()
        print(f"  {len(self.__changed_files)} changed files")
        return False

    def set_status(self, msg: str):
        print(f"\x1b[2K\x1b[G{msg}", end="", flush=True)

    def set_current_task(self, task: str):
        self.set_status(f"  Processing task {task}")
        self.__num_tasks += 1
        self.__last_task = task
        self.__on_status_line = True

    def add_preprocess_info(self, msg: str):
        self.__preprocess_info.append(msg)

    def add_task_info(self, msg: str):
        if self.__on_status_line:
            print()
            self.__on_status_line = False
        print(f"    {msg}")

    def sub(self, name: str, rel_paths: list[str], substitution_strs: dict[str, str]):
        self.set_current_task(name)
        for msg in self.__preprocess_info:
            self.add_task_info(msg)
        self.__preprocess_info.clear()
        for i in range(1, len(rel_paths)):
            prev_path: str = rel_paths[i - 1]
            curr_path: str = rel_paths[i]
            if prev_path >= curr_path:
                self.add_task_info(f"Unsorted paths {prev_path} and {curr_path}!")
        substitutions: list[tuple[str, str]] = compile_substitutions(substitution_strs)
        for rel_path in rel_paths:
            dirty: bool = substitute_file(self.rootdir / rel_path, substitutions)
            if dirty:
                self.__changed_files.add(rel_path)
            else:
                self.add_task_info(f"Unchanged file {rel_path}!")


def make_substitutions(s: Substitutor):
    def make_substitution_strs(*substitutions: tuple[Substitution]):
        global make_substitution_strs
        return make_substitution_strs(*substitutions, output=s.add_preprocess_info)

    def normal_substitute(*substitutions: tuple[Substitution]):
        global normal_substitute
        return normal_substitute(*substitutions, output=s.add_preprocess_info)

    def remove_k(*substitutions: tuple[Substitution]):
        global remove_k
        return remove_k(*substitutions, output=s.add_preprocess_info)

    # apriltag

    s.sub(
        "apriltag AprilTagFields",
        [
            "apriltag/README.md",
            "apriltag/src/main/java/edu/wpi/first/apriltag/AprilTagFields.java",
            "apriltag/src/main/native/cpp/AprilTagFieldLayout.cpp",
            "apriltag/src/main/native/include/frc/apriltag/AprilTagFields.h",
            "apriltag/src/test/java/edu/wpi/first/apriltag/LoadConfigTest.java",
            "apriltag/src/test/native/cpp/LoadConfigTest.cpp",
            "wpilibcExamples/src/main/cpp/examples/DifferentialDrivePoseEstimator/include/Drivetrain.h",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdriveposeestimator/Drivetrain.java",
        ],
        normal_substitute(
            ("k2022RapidReact", "RAPID_REACT_2022"),
            ("k2023ChargedUp", "CHARGED_UP_2023"),
            ("k2024Crescendo", "CRESCENDO_2024"),
            ("k2025ReefscapeWelded", "REEFSCAPE_2025_WELDED"),
            ("k2025ReefscapeAndyMark", "REEFSCAPE_2025_ANDYMARK"),
            "kBaseResourceDir",
            "kDefaultField",
            "kNumFields",
        ),
    )

    s.sub(
        "apriltag origin",
        [
            "apriltag/src/main/java/edu/wpi/first/apriltag/AprilTagFieldLayout.java",
            "apriltag/src/main/native/cpp/AprilTagFieldLayout.cpp",
            "apriltag/src/main/native/include/frc/apriltag/AprilTagFieldLayout.h",
            "apriltag/src/test/java/edu/wpi/first/apriltag/AprilTagPoseSetOriginTest.java",
            "apriltag/src/test/native/cpp/AprilTagPoseSetOriginTest.cpp",
        ],
        normal_substitute(
            "kBlueAllianceWallRightSide",
            "kRedAllianceWallRightSide",
        ),
    )

    # cameraserver

    s.sub(
        "cameraserver",
        [
            "cameraserver/src/main/java/edu/wpi/first/cameraserver/CameraServer.java",
            "cameraserver/src/main/native/cpp/cameraserver/CameraServer.cpp",
            "cameraserver/src/main/native/include/cameraserver/CameraServer.h",
        ],
        normal_substitute(
            "kBasePort",
            "kPublishName",
        ),
    )

    # cscore

    s.sub(
        "cscore CameraServerJNI TelemetryKind",
        [
            "cscore/src/main/java/edu/wpi/first/cscore/CameraServerJNI.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSource.java",
        ],
        normal_substitute(
            "kSourceBytesReceived",
            "kSourceFramesReceived",
        ),
    )

    s.sub(
        "cscore VideoEvent.java",
        ["cscore/src/main/java/edu/wpi/first/cscore/VideoEvent.java"],
        normal_substitute(
            "kSource*",
            "kSink*",
            "kSourceProperty*",
        ),
    )

    s.sub(
        "cscore VideoMode PixelFormat",
        [
            "apriltag/src/test/java/edu/wpi/first/apriltag/AprilTagGenerationTest.java",
            "cameraserver/src/main/java/edu/wpi/first/cameraserver/CameraServer.java",
            "cameraserver/src/main/native/cpp/cameraserver/CameraServer.cpp",
            "cscore/examples/enum_usb/enum_usb.cpp",
            "cscore/examples/httpcvstream/httpcvstream.cpp",
            "cscore/examples/usbcvstream/usbcvstream.cpp",
            "cscore/examples/usbstream/usbstream.cpp",
            "cscore/examples/usbviewer/usbviewer.cpp",
            "cscore/java-examples/RawCVMatSink.java",
            "cscore/java-examples/RawCVMatSource.java",
            "cscore/src/main/java/edu/wpi/first/cscore/CvSink.java",
            "cscore/src/main/java/edu/wpi/first/cscore/CvSource.java",
            "cscore/src/main/native/cpp/Frame.cpp",
            "cscore/src/main/native/cpp/Frame.h",
            "cscore/src/main/native/cpp/HttpCameraImpl.cpp",
            "cscore/src/main/native/cpp/Image.h",
            "cscore/src/main/native/cpp/MjpegServerImpl.cpp",
            "cscore/src/main/native/cpp/SourceImpl.cpp",
            "cscore/src/main/native/include/cscore_cpp.h",
            "cscore/src/main/native/include/cscore_cv.h",
            "cscore/src/main/native/linux/UsbCameraImpl.cpp",
            "cscore/src/main/native/objcpp/UsbCameraImplObjc.mm",
            "cscore/src/main/native/windows/UsbCameraImpl.cpp",
            "cscore/src/test/java/edu/wpi/first/cscore/VideoModeTest.java",
        ],
        normal_substitute(
            "kUnknown",
            "kMJPEG",
            "kYUYV",
            "kRGB565",
            "kBGR",
            "kGray",
            "kY16",
            "kUYVY",
            "kBGRA",
        ),
    )
    s.sub(
        "cscore RawEvent Kind",
        [
            "cameraserver/src/main/java/edu/wpi/first/cameraserver/CameraServer.java",
            "cameraserver/src/main/native/cpp/cameraserver/CameraServer.cpp",
            "cscore/examples/usbstream/usbstream.cpp",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoEvent.java",
            "cscore/src/main/native/cpp/Notifier.cpp",
            "cscore/src/main/native/include/cscore_cpp.h",
        ],
        normal_substitute(
            "kSourceCreated",
            "kSourceDestroyed",
            "kSourceConnected",
            "kSourceDisconnected",
            "kSourceVideoModesUpdated",
            "kSourceVideoModeChanged",
            "kSourcePropertyCreated",
            "kSourcePropertyValueUpdated",
            "kSourcePropertyChoicesUpdated",
            "kSinkSourceChanged",
            "kSinkCreated",
            "kSinkDestroyed",
            "kSinkEnabled",
            "kSinkDisabled",
            "kNetworkInterfacesChanged",
            "kTelemetryUpdated",
            "kSinkPropertyCreated",
            "kSinkPropertyValueUpdated",
            "kSinkPropertyChoicesUpdated",
            "kUsbCamerasChanged",
        ),
    )
    s.sub(
        "cscore VideoProperty Kind",
        [
            "cameraserver/src/main/java/edu/wpi/first/cameraserver/CameraServer.java",
            "cscore/examples/enum_usb/enum_usb.cpp",
            "cscore/examples/settings/settings.cpp",
            "cscore/src/main/java/edu/wpi/first/cscore/ImageSource.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoProperty.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSink.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSource.java",
            "cscore/src/main/native/include/cscore_oo.h",
        ],
        normal_substitute(
            "kNone",
            "kBoolean",
            "kInteger",
            "kString",
            "kEnum",
        ),
    )
    s.sub(
        "cscore VideoSource Kind",
        [
            "cameraserver/src/main/java/edu/wpi/first/cameraserver/CameraServer.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSource.java",
            "cscore/src/main/native/include/cscore_oo.h",
        ],
        normal_substitute(
            "kUnknown",
            "kUsb",
            "kHttp",
            "kCv",
            "kRaw",
        ),
    )
    s.sub(
        "cscore c++ VideoSource ConnectionStrategy",
        [
            "cscore/src/main/native/include/cscore_oo.h",
        ],
        normal_substitute(
            "kConnectionAutoManage",
            "kConnectionKeepOpen",
            "kConnectionForceClose",
        ),
    )
    s.sub(
        "cscore java VideoSource ConnectionStrategy",
        [
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSource.java",
        ],
        normal_substitute(
            "kAutoManage",
            "kKeepOpen",
            "kForceClose",
        ),
    )
    s.sub(
        "cscore VideoCamera WhiteBalance",
        [
            "cscore/src/main/java/edu/wpi/first/cscore/VideoCamera.java",
            "cscore/src/main/native/include/cscore_oo.h",
        ],
        normal_substitute(
            "kFixedIndoor",
            "kFixedOutdoor1",
            "kFixedOutdoor2",
            "kFixedFluorescent1",
            # Fix typo
            ("kFixedFlourescent2", "FIXED_FLUORESCENT2"),
        ),
    )
    s.sub(
        "cscore HttpCamera HttpCameraKind",
        [
            "cscore/src/main/java/edu/wpi/first/cscore/AxisCamera.java",
            "cscore/src/main/java/edu/wpi/first/cscore/HttpCamera.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoEvent.java",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSink.java",
            "cscore/src/main/native/include/cscore_oo.h",
        ],
        normal_substitute(
            "kUnknown",
            # NOTE Seems like a mistake, plus Java-only
            ("kMJPGStreamer", "MJPEG_STREAMER"),
            "kCSCore",
            "kAxis",
        ),
    )
    s.sub(
        "cscore VideoSink Kind",
        [
            "cameraserver/src/main/java/edu/wpi/first/cameraserver/CameraServer.java",
            "cameraserver/src/main/native/cpp/cameraserver/CameraServer.cpp",
            "cscore/src/main/java/edu/wpi/first/cscore/VideoSink.java",
            "cscore/src/main/native/include/cscore_oo.h",
        ],
        normal_substitute(
            "kMjpeg",
            "kCv",
            "kRaw",
        ),
    )
    s.sub(
        "cscore Handle",
        [
            "cscore/src/main/native/cpp/ConfigurableSourceImpl.cpp",
            "cscore/src/main/native/cpp/Handle.h",
            "cscore/src/main/native/cpp/Instance.h",
            "cscore/src/main/native/cpp/Notifier.cpp",
            "cscore/src/main/native/cpp/Notifier.h",
            "cscore/src/main/native/cpp/Telemetry.cpp",
            "cscore/src/main/native/cpp/UnlimitedHandleResource.h",
            "cscore/src/main/native/cpp/cscore_cpp.cpp",
            "cscore/src/main/native/include/cscore_cpp.h",
        ],
        normal_substitute(
            "kUndefined",
            "kProperty",
            "kSource",
            "kSink",
            "kListener",
            "kSinkProperty",
            "kListenerPoller",
            "kIndexMax",
        ),
    )

    s.sub(
        "cscore cscore_cpp.h",
        ["cscore/src/main/native/include/cscore_cpp.h"],
        normal_substitute(
            "kSourceProperty*",
        ),
    )
    s.sub(
        "cscore MjpegServerImpl.cpp",
        ["cscore/src/main/native/cpp/MjpegServerImpl.cpp"],
        normal_substitute(
            "kCommand",
            "kStream",
            "kGetSettings",
            "kGetSourceConfig",
            "kRootPage",
            "kSinkSourceChanged",
            "kNetworkInterfacesChanged",
            "kTelemetryUpdated",
            "kUsbCamerasChanged",
        ),
    )
    s.sub(
        "cscore SourceImpl.cpp",
        ["cscore/src/main/native/cpp/SourceImpl.cpp"],
        normal_substitute(
            "kMaxImagesAvail",
        ),
    )

    s.sub(
        "cscore linux UsbCameraImpl.cpp",
        ["cscore/src/main/native/linux/UsbCameraImpl.cpp"],
        normal_substitute(
            "kPropWbAuto",
            "kPropWbValue",
            "kPropExAuto",
            "kPropExValue",
            "kPropBrValue",
            "kPropConnectVerbose",
            "kPropConnectVerboseId",
        ),
    )
    s.sub(
        "cscore linux Message",
        [
            "cscore/src/main/native/linux/UsbCameraImpl.cpp",
            "cscore/src/main/native/linux/UsbCameraImpl.h",
        ],
        normal_substitute(
            "kNone",
            "kCmdSetPath",
            "kCmdSetMode",
            "kCmdSetPixelFormat",
            "kCmdSetResolution",
            "kCmdSetFPS",
            "kCmdSetProperty",
            "kCmdSetPropertyStr",
            "kNumSinksChanged",
            "kNumSinksEnabledChanged",
            # Avoid conflict with Windows header macro: https://github.com/wpilibsuite/allwpilib/pull/7954#discussion_r2072501900
            ("kOk", "MSG_OK"),
            ("kError", "MSG_ERROR"),
            "kNumBuffers",
        ),
    )

    s.sub(
        "cscore windows UsbCameraImpl.cpp",
        ["cscore/src/main/native/windows/UsbCameraImpl.cpp"],
        normal_substitute(
            "kPropWbValue",
            "kPropExValue",
            "kPropBrValue",
            "kPropConnectVerbose",
            "kPropConnectVerboseId",
        ),
    )
    s.sub(
        "cscore windows Message",
        [
            "cscore/src/main/native/windows/UsbCameraImpl.cpp",
            "cscore/src/main/native/windows/UsbCameraImpl.h",
        ],
        normal_substitute(
            "kNone",
            "kCmdSetPath",
            "kCmdSetMode",
            "kCmdSetPixelFormat",
            "kCmdSetResolution",
            "kCmdSetFPS",
            "kCmdSetProperty",
            "kCmdSetPropertyStr",
            "kNumSinksChanged",
            "kNumSinksEnabledChanged",
            # Avoid conflict with Windows header macro: https://github.com/wpilibsuite/allwpilib/pull/7954#discussion_r2072501900
            ("kOk", "MSG_OK"),
            ("kError", "MSG_ERROR"),
        ),
    )

    # datalog

    s.sub(
        "printlog example",
        ["datalog/examples/printlog/datalog.py"],
        normal_substitute(
            "kControlStart",
            "kControlFinish",
            "kControlSetMetadata",
        ),
    )
    s.sub(
        "writelog example",
        ["datalog/examples/writelog/writelog.cpp"],
        normal_substitute(
            ("kNumRuns", "numRuns"),
        ),
    )

    s.sub(
        "datalog DATA_TYPE",
        [
            "datalog/src/main/java/edu/wpi/first/datalog/BooleanArrayLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/BooleanLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/DoubleArrayLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/DoubleLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/FloatArrayLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/FloatLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/IntegerArrayLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/IntegerLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/RawLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/StringArrayLogEntry.java",
            "datalog/src/main/java/edu/wpi/first/datalog/StringLogEntry.java",
            "datalog/src/main/native/include/wpi/datalog/DataLog.h",
        ],
        normal_substitute(
            "kDataType",
        ),
    )
    s.sub(
        "datalog DataLogRecord",
        [
            "datalog/src/main/java/edu/wpi/first/datalog/DataLogRecord.java",
            "datalog/src/main/native/cpp/DataLog.cpp",
            "datalog/src/main/native/cpp/DataLogReader.cpp",
            "datalog/src/main/native/include/wpi/datalog/DataLog.h",
        ],
        normal_substitute(
            "kControlStart",
            "kControlFinish",
            "kControlSetMetadata",
            "kBlockSize",
            "kMaxBufferCount",
            "kMaxFreeCount",
        ),
    )

    s.sub(
        "datalog DataLogWriter.java",
        [
            "datalog/src/main/java/edu/wpi/first/datalog/DataLogWriter.java",
        ],
        normal_substitute(
            "kBufferSize",
        ),
    )
    s.sub(
        "datalog DataLog.cpp",
        ["datalog/src/main/native/cpp/DataLog.cpp"],
        normal_substitute(
            "kRecordMaxHeaderSize",
        ),
    )

    s.sub(
        "datalog DataLogBackgroundWriter",
        [
            "datalog/src/main/native/cpp/DataLogBackgroundWriter.cpp",
            "datalog/src/main/native/include/wpi/datalog/DataLogBackgroundWriter.h",
        ],
        normal_substitute(
            "kPaused",
            "kActive",
            "kStopped",
            "kStart",
        ),
    )
    s.sub(
        "datalog DataLogBackgroundWriter.cpp",
        ["datalog/src/main/native/cpp/DataLogBackgroundWriter.cpp"],
        normal_substitute(
            "kMinFreeSpace",
            ("kKiB", "KIB"),
            ("kMiB", "MIB"),
            ("kGiB", "GIB"),
        ),
    )

    # datalogtool

    s.sub(
        "datalogtool",
        [
            "datalogtool/src/main/native/cpp/Downloader.cpp",
            "datalogtool/src/main/native/cpp/Downloader.h",
        ],
        normal_substitute(
            "kDisconnected",
            "kConnecting",
            "kConnected",
            "kDisconnecting",
            "kGetFiles",
            ("kDownload", "DOWNLOADING"),
            ("kDownloadDone", "DOWNLOADED"),
            ("kDelete", "DELETING"),
            ("kDeleteDone", "DELETED"),
            ("kExit", "EXITING"),
            "kBufSize",
        ),
    )

    # epilogue

    s.sub(
        "epilogue AnnotationProcessor.java",
        [
            "epilogue-processor/src/main/java/edu/wpi/first/epilogue/processor/AnnotationProcessor.java"
        ],
        normal_substitute(
            "kCustomLoggerFqn",
            "kClassSpecificLoggerFqn",
            "kLoggedFqn",
        ),
    )
    s.sub(
        "epilogue LoggerGenerator.java",
        [
            "epilogue-processor/src/main/java/edu/wpi/first/epilogue/processor/LoggerGenerator.java"
        ],
        normal_substitute(
            "kIsBuiltInJavaMethod",
        ),
    )
    s.sub(
        "epilogue java version",
        [
            "epilogue-processor/src/test/java/edu/wpi/first/epilogue/processor/AnnotationProcessorTest.java",
            "epilogue-processor/src/test/java/edu/wpi/first/epilogue/processor/CompileTestOptions.java",
            "epilogue-processor/src/test/java/edu/wpi/first/epilogue/processor/EpilogueGeneratorTest.java",
        ],
        normal_substitute(
            "kJavaVersionOptions",
            "kJavaVersion",
        ),
    )
    s.sub(
        "epilogue AnnotationProcessorTest.java",
        [
            "epilogue-processor/src/test/java/edu/wpi/first/epilogue/processor/AnnotationProcessorTest.java",
        ],
        make_substitution_strs(
            ("kConstantPrefix", "CONSTANT_PREFIX"),
            ("k_otherConstantPrefix", "OTHER_CONSTANT_PREFIX"),
        ),
    )

    # fieldImages

    s.sub(
        "fieldImages Fields",
        [
            "fieldImages/src/main/java/edu/wpi/first/fields/FieldConfig.java",
            "fieldImages/src/main/java/edu/wpi/first/fields/Fields.java",
        ],
        normal_substitute(
            ("k2018PowerUp", "POWER_UP_2018"),
            ("k2019DeepSpace", "DEEP_SPACE_2019"),
            ("k2020InfiniteRecharge", "INFINITE_RECHARGE_2020"),
            ("k2021InfiniteRecharge", "INFINITE_RECHARGE_2021"),
            ("k2021Barrel", "BARREL_2021"),
            ("k2021Bounce", "BOUNCE_2021"),
            ("k2021GalacticSearchA", "GALACTIC_SEARCHA_2021"),
            ("k2021GalacticSearchB", "GALACTIC_SEARCHB_2021"),
            ("k2021Slalom", "SLALOM_2021"),
            ("k2022RapidReact", "RAPID_REACT_2022"),
            ("k2023ChargedUp", "CHARGED_UP_2023"),
            ("k2024Crescendo", "CRESCENDO_2024"),
            ("k2025Reefscape", "REEFSCAPE_2025"),
            "kBaseResourceDir",
            "kDefaultField",
        ),
    )
    s.sub(
        "fieldImages fields.cpp",
        [
            "fieldImages/src/main/native/cpp/fields.cpp",
        ],
        normal_substitute(
            "kFields",
        ),
    )

    # glass

    s.sub(
        "glass Window",
        [
            "glass/src/app/native/cpp/main.cpp",
            "glass/src/lib/native/cpp/Window.cpp",
            "glass/src/lib/native/include/glass/Window.h",
            "glass/src/lib/native/include/glass/WindowManager.h",
            "glass/src/libnt/native/cpp/NetworkTablesProvider.cpp",
            "simulation/halsim_gui/src/main/native/cpp/DriverStationGui.cpp",
            "simulation/halsim_gui/src/main/native/cpp/HALProvider.cpp",
            "simulation/halsim_gui/src/main/native/cpp/NetworkTablesSimGui.cpp",
        ],
        normal_substitute(
            "kHide",
            "kShow",
            "kDisabled",
        ),
    )

    s.sub(
        "glass DataSource",
        [
            "glass/src/lib/native/cpp/DataSource.cpp",
            "glass/src/lib/native/include/glass/DataSource.h",
        ],
        normal_substitute(
            "kType",
        ),
    )

    s.sub(
        "glass libnt Model TYPE",
        [
            "glass/src/libnt/native/cpp/StandardNetworkTables.cpp",
            "glass/src/libnt/native/include/glass/networktables/NTAlerts.h",
            "glass/src/libnt/native/include/glass/networktables/NTCommandScheduler.h",
            "glass/src/libnt/native/include/glass/networktables/NTCommandSelector.h",
            "glass/src/libnt/native/include/glass/networktables/NTDifferentialDrive.h",
            "glass/src/libnt/native/include/glass/networktables/NTDigitalInput.h",
            "glass/src/libnt/native/include/glass/networktables/NTDigitalOutput.h",
            "glass/src/libnt/native/include/glass/networktables/NTFMS.h",
            "glass/src/libnt/native/include/glass/networktables/NTField2D.h",
            "glass/src/libnt/native/include/glass/networktables/NTGyro.h",
            "glass/src/libnt/native/include/glass/networktables/NTMecanumDrive.h",
            "glass/src/libnt/native/include/glass/networktables/NTMechanism2D.h",
            "glass/src/libnt/native/include/glass/networktables/NTMotorController.h",
            "glass/src/libnt/native/include/glass/networktables/NTPIDController.h",
            "glass/src/libnt/native/include/glass/networktables/NTProfiledPIDController.h",
            "glass/src/libnt/native/include/glass/networktables/NTStringChooser.h",
            "glass/src/libnt/native/include/glass/networktables/NTSubsystem.h",
        ],
        normal_substitute(
            "kType",
        ),
    )

    s.sub(
        "glass Alerts.cpp",
        [
            "glass/src/lib/native/cpp/other/Alerts.cpp",
        ],
        normal_substitute(
            "kWarningColor",
            "kErrorColor",
        ),
    )
    s.sub(
        "glass Field2D.cpp",
        [
            "glass/src/lib/native/cpp/other/Field2D.cpp",
        ],
        normal_substitute(
            "kDisplayMeters",
            "kDisplayFeet",
            "kDisplayInches",
            "kBoxImage",
            "kLine",
            "kLineClosed",
            "kTrack",
            "kHidden",
            "kDefaultStyle",
            "kDefaultWeight",
            "kDefaultColorFloat",
            "kDefaultColor",
            "kDefaultWidth",
            "kDefaultLength",
            "kDefaultArrows",
            "kDefaultArrowSize",
            "kDefaultArrowWeight",
            "kDefaultArrowColorFloat",
            "kDefaultArrowColor",
            "kDefaultSelectable",
            "kDefaultWidth",
            "kDefaultHeight",
        ),
    )
    s.sub(
        "glass Plot.cpp",
        [
            "glass/src/lib/native/cpp/other/Plot.cpp",
        ],
        normal_substitute(
            "kAxisCount",
            "kNone",
            "kMoveUp",
            "kMoveDown",
            "kDelete",
            "kDefaultColor",
            "kAuto",
            "kDigital",
            "kAnalog",
            "kMaxSize",
            "kTimeGap",
            "kDefaultBackgroundColor",
            ("  IMPLOT_AUTO};", "   IMPLOT_AUTO};"),
        ),
    )
    s.sub(
        "glass ExtraGuiWidgets.cpp",
        [
            "glass/src/lib/native/cpp/support/ExtraGuiWidgets.cpp",
        ],
        normal_substitute(
            "kBufferSize",
        ),
    )

    # Make sure to do these substitutions before the Storage.Value.Type ones
    s.sub(
        "glass Storage.cpp",
        [
            "glass/src/lib/native/cpp/Storage.cpp",
        ],
        normal_substitute(
            ("k##CapsName", "AllCapsName"),
            ("k##CapsName##Array", "AllCapsName##_ARRAY"),
            ("CONVERT(CapsName, LowerName, CType)", "CONVERT(CapsName, LowerName, AllCapsName, CType)"),
            ("CONVERT(Int, int, int)", "CONVERT(Int, int, INT, int)"),
            ("CONVERT(Int64, int64, int64_t)", "CONVERT(Int64, int64, INT64, int64_t)"),
            ("CONVERT(Float, float, float)", "CONVERT(Float, float, FLOAT, float)"),
            ("CONVERT(Double, double, double)", "CONVERT(Double, double, DOUBLE, double)"),
            ("CONVERT(Bool, bool, bool)", "CONVERT(Bool, bool, BOOL, bool)"),
            ("CONVERT_ARRAY(CapsName, LowerName)", "CONVERT_ARRAY(CapsName, LowerName, AllCapsName)"),
            ("CONVERT_ARRAY(Int, int)", "CONVERT_ARRAY(Int, int, INT)"),
            ("CONVERT_ARRAY(Int64, int64)", "CONVERT_ARRAY(Int64, int64, INT64)"),
            ("CONVERT_ARRAY(Float, float)", "CONVERT_ARRAY(Float, float, FLOAT)"),
            ("CONVERT_ARRAY(Double, double)", "CONVERT_ARRAY(Double, double, DOUBLE)"),
            ("DEFUN(CapsName, LowerName, CType, CParamType, ArrCType)", "DEFUN(CapsName, LowerName, AllCapsName, CType, CParamType, ArrCType)"),
            ("DEFUN(Int, int, int, int, int)", "DEFUN(Int, int, INT, int, int, int)"),
            ("DEFUN(Int64, int64, int64_t, int64_t, int64_t)", "DEFUN(Int64, int64, INT64, int64_t, int64_t, int64_t)"),
            ("DEFUN(Bool, bool, bool, bool, int)", "DEFUN(Bool, bool, BOOL, bool, bool, int)"),
            ("DEFUN(Float, float, float, float, float)", "DEFUN(Float, float, FLOAT, float, float, float)"),
            ("DEFUN(Double, double, double, double, double)", "DEFUN(Double, double, DOUBLE, double, double, double)"),
            ("DEFUN(String, string, std::string, std::string_view, std::string)", "DEFUN(String, string, STRING, std::string, std::string_view, std::string)"),
            ("CASE(CapsName, LowerName)", "CASE(CapsName, LowerName, AllCapsName)"),
            ("CASE(Int, int)", "CASE(Int, int, INT)"),
            ("CASE(Int64, int64)", "CASE(Int64, int64, INT64)"),
            ("CASE(Bool, bool)", "CASE(Bool, bool, BOOL)"),
            ("CASE(Float, float)", "CASE(Float, float, FLOAT)"),
            ("CASE(Double, double)", "CASE(Double, double, DOUBLE)"),
            ("CASE(String, string)", "CASE(String, string, STRING)"),
        ),
    )

    s.sub(
        "glass Storage Value Type",
        [
            "glass/src/lib/native/cpp/Storage.cpp",
            "glass/src/lib/native/include/glass/Storage.h",
            "glass/src/libnt/native/cpp/NetworkTablesProvider.cpp",
        ],
        normal_substitute(
            "kNone",
            "kInt",
            "kInt64",
            "kBool",
            "kFloat",
            "kDouble",
            "kString",
            "kChild",
            "kIntArray",
            ("kInt64Array", "INT64_ARRAY"),
            "kBoolArray",
            "kFloatArray",
            "kDoubleArray",
            "kStringArray",
            "kChildArray",
        ),
    )

    s.sub(
        "glass NetworkTablesFlags",
        [
            "glass/src/libnt/native/cpp/NetworkTables.cpp",
            "glass/src/libnt/native/include/glass/networktables/NetworkTables.h",
        ],
        normal_substitute(
            ("kNetworkTablesFlags_PrecisionBitShift", "NETWORK_TABLES_FLAGS__PRECISION_BIT_SHIFT"),
            ("NetworkTablesFlags_TreeView", "NETWORK_TABLES_FLAGS__TREE_VIEW"),
            ("NetworkTablesFlags_CombinedView", "NETWORK_TABLES_FLAGS__COMBINED_VIEW"),
            ("NetworkTablesFlags_ReadOnly", "NETWORK_TABLES_FLAGS__READ_ONLY"),
            ("NetworkTablesFlags_ShowSpecial", "NETWORK_TABLES_FLAGS__SHOW_SPECIAL"),
            ("NetworkTablesFlags_ShowProperties", "NETWORK_TABLES_FLAGS__SHOW_PROPERTIES"),
            ("NetworkTablesFlags_ShowTimestamp", "NETWORK_TABLES_FLAGS__SHOW_TIMESTAMP"),
            ("NetworkTablesFlags_ShowServerTimestamp", "NETWORK_TABLES_FLAGS__SHOW_SERVER_TIMESTAMP"),
            ("NetworkTablesFlags_CreateNoncanonicalKeys", "NETWORK_TABLES_FLAGS__CREATE_NONCANONICAL_KEYS"),
            ("NetworkTablesFlags_Precision", "NETWORK_TABLES_FLAGS__PRECISION"),
            ("NetworkTablesFlags_Default", "NETWORK_TABLES_FLAGS__DEFAULT"),
        ),
    )

    s.sub(
        "glass NetworkTables.cpp",
        [
            "glass/src/libnt/native/cpp/NetworkTables.cpp",
        ],
        normal_substitute(
            "kTextBufferSize",
        ),
    )

    # hal

    s.sub(
        "hal java CANAPITypes CANDeviceType",
        [
            "hal/src/main/java/edu/wpi/first/hal/CANAPITypes.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/CAN.java",
        ],
        normal_substitute(
            "kBroadcast",
            "kRobotController",
            "kMotorController",
            "kRelayController",
            "kGyroSensor",
            "kAccelerometer",
            "kUltrasonicSensor",
            "kGearToothSensor",
            "kPowerDistribution",
            "kPneumatics",
            "kMiscellaneous",
            "kIOBreakout",
            "kServoController",
            "kFirmwareUpdate",
        ),
    )
    s.sub(
        "hal c++ CANDeviceType",
        [
            "hal/src/main/native/include/hal/CANAPITypes.h",
            "hal/src/main/native/sim/PowerDistribution.cpp",
            "hal/src/main/native/systemcore/CTREPCM.cpp",
            "hal/src/main/native/systemcore/CTREPDP.cpp",
            "hal/src/main/native/systemcore/REVPDH.cpp",
            "hal/src/main/native/systemcore/REVPH.cpp",
            "hal/src/test/native/cpp/can/CANTest.cpp",
            "wpilibc/src/main/native/include/frc/CAN.h",
        ],
        remove_k(
            "HAL_CAN_Dev_kBroadcast",
            "HAL_CAN_Dev_kRobotController",
            "HAL_CAN_Dev_kMotorController",
            "HAL_CAN_Dev_kRelayController",
            "HAL_CAN_Dev_kGyroSensor",
            "HAL_CAN_Dev_kAccelerometer",
            "HAL_CAN_Dev_kUltrasonicSensor",
            "HAL_CAN_Dev_kGearToothSensor",
            "HAL_CAN_Dev_kPowerDistribution",
            "HAL_CAN_Dev_kPneumatics",
            "HAL_CAN_Dev_kMiscellaneous",
            "HAL_CAN_Dev_kIOBreakout",
            "HAL_CAN_Dev_kServoController",
            "HAL_CAN_Dev_kFirmwareUpdate",
        ),
    )

    s.sub(
        "hal java CANAPITypes CANManufacturer",
        [
            "hal/src/main/java/edu/wpi/first/hal/CANAPITypes.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/CAN.java",
        ],
        normal_substitute(
            "kBroadcast",
            "kNI",
            "kLM",
            "kDEKA",
            "kCTRE",
            "kREV",
            "kGrapple",
            "kMS",
            "kTeamUse",
            "kKauaiLabs",
            ("kCopperforge", "COPPER_FORGE"),
            "kPWF",
            "kStudica",
            "kTheThriftyBot",
            "kReduxRobotics",
            ("kAndyMark", "ANDYMARK"),
            "kVividHosting",
        ),
    )
    s.sub(
        "hal c++ CANManufacturer",
        [
            "hal/src/main/native/include/hal/CANAPITypes.h",
            "hal/src/main/native/sim/PowerDistribution.cpp",
            "hal/src/main/native/systemcore/CTREPCM.cpp",
            "hal/src/main/native/systemcore/CTREPDP.cpp",
            "hal/src/main/native/systemcore/REVPDH.cpp",
            "hal/src/main/native/systemcore/REVPH.cpp",
            "hal/src/test/native/cpp/can/CANTest.cpp",
            "wpilibc/src/main/native/include/frc/CAN.h",
        ],
        remove_k(
            "HAL_CAN_Man_kBroadcast",
            "HAL_CAN_Man_kNI",
            "HAL_CAN_Man_kLM",
            "HAL_CAN_Man_kDEKA",
            "HAL_CAN_Man_kCTRE",
            "HAL_CAN_Man_kREV",
            "HAL_CAN_Man_kGrapple",
            "HAL_CAN_Man_kMS",
            "HAL_CAN_Man_kTeamUse",
            "HAL_CAN_Man_kKauaiLabs",
            "HAL_CAN_Man_kCopperforge",
            "HAL_CAN_Man_kPWF",
            "HAL_CAN_Man_kStudica",
            "HAL_CAN_Man_kTheThriftyBot",
            "HAL_CAN_Man_kReduxRobotics",
            "HAL_CAN_Man_kAndyMark",
            "HAL_CAN_Man_kVividHosting",
        ),
    )

    s.sub(
        "hal HALValue",
        [
            "hal/src/main/java/edu/wpi/first/hal/HALValue.java",
            "hal/src/main/java/edu/wpi/first/hal/SimDeviceJNI.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/testutils/BooleanCallback.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/testutils/DoubleCallback.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/testutils/EnumCallback.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/testutils/IntCallback.java",
        ],
        normal_substitute(
            "kUnassigned",
            "kBoolean",
            "kDouble",
            "kEnum",
            "kInt",
            "kLong",
        ),
    )
    s.sub(
        "hal SimDevice Direction",
        [
            "hal/src/main/java/edu/wpi/first/hal/SimDevice.java",
            "hal/src/main/java/edu/wpi/first/hal/SimDeviceJNI.java",
            "hal/src/main/native/include/hal/SimDevice.h",
            "romiVendordep/src/main/java/edu/wpi/first/wpilibj/romi/RomiGyro.java",
            "romiVendordep/src/main/native/cpp/romi/RomiGyro.cpp",
            "wpilibc/src/main/native/cpp/ADXL345_I2C.cpp",
            "wpilibc/src/main/native/cpp/DutyCycleEncoder.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/ADXL345_I2C.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/AnalogEncoder.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DutyCycleEncoder.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Servo.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/SharpIR.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/motorcontrol/PWMMotorController.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/SimDeviceSimTest.java",
            "xrpVendordep/src/main/java/edu/wpi/first/wpilibj/xrp/XRPGyro.java",
            "xrpVendordep/src/main/java/edu/wpi/first/wpilibj/xrp/XRPMotor.java",
            "xrpVendordep/src/main/java/edu/wpi/first/wpilibj/xrp/XRPServo.java",
            "xrpVendordep/src/main/native/cpp/xrp/XRPGyro.cpp",
            "xrpVendordep/src/main/native/cpp/xrp/XRPMotor.cpp",
            "xrpVendordep/src/main/native/cpp/xrp/XRPServo.cpp",
        ],
        normal_substitute(
            "kInput",
            "kOutput",
            "kBidir",
        ),
    )

    s.sub(
        "hal NIRioStatus",
        [
            "hal/src/main/java/edu/wpi/first/hal/communication/NIRioStatus.java",
            "hal/src/main/native/cpp/jni/HALUtil.cpp",
        ],
        normal_substitute(
            "kRioStatusOffset",
            "kRioStatusSuccess",
            "kRIOStatusBufferInvalidSize",
            "kRIOStatusOperationTimedOut",
            "kRIOStatusFeatureNotSupported",
            "kRIOStatusResourceNotInitialized",
        ),
    )

    driverstation_substitutions: dict[str, str] = normal_substitute(
        "kUnknownAllianceStation",
        ("kRed1AllianceStation", "RED1_ALLIANCE_STATION"),
        ("kRed2AllianceStation", "RED2_ALLIANCE_STATION"),
        ("kRed3AllianceStation", "RED3_ALLIANCE_STATION"),
        ("kBlue1AllianceStation", "BLUE1_ALLIANCE_STATION"),
        ("kBlue2AllianceStation", "BLUE2_ALLIANCE_STATION"),
        ("kBlue3AllianceStation", "BLUE3_ALLIANCE_STATION"),
        "kMaxJoystickAxes",
        ("kMaxJoystickPOVs", "MAX_JOYSTICK_POVS"),
        "kMaxJoysticks",
    )
    s.sub(
        "hal DriverStationJNI.java",
        [
            "hal/src/main/java/edu/wpi/first/hal/DriverStationJNI.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DriverStation.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/DriverStationSim.java",
        ],
        driverstation_substitutions,
    )
    s.sub(
        "hal DriverStationJNI.cpp",
        [
            "hal/src/main/native/cpp/jni/DriverStationJNI.cpp",
        ],
        add_prefix("edu_wpi_first_hal_DriverStationJNI_", driverstation_substitutions),
    )
    s.sub(
        "hal DriverStation",
        [
            "hal/src/main/native/cpp/jni/DriverStationJNI.cpp",
            "hal/src/main/native/cpp/jni/simulation/DriverStationDataJNI.cpp",
            "hal/src/main/native/include/hal/DriverStationTypes.h",
            "hal/src/main/native/sim/DriverStation.cpp",
            "hal/src/main/native/sim/mockdata/DriverStationData.cpp",
            "hal/src/main/native/systemcore/FRCDriverStation.cpp",
            "hal/src/main/native/systemcore/mockdata/DriverStationData.cpp",
            "hal/src/test/native/cpp/mockdata/DriverStationDataTest.cpp",
            "simulation/halsim_ds_socket/src/main/native/cpp/DSCommPacket.cpp",
            "simulation/halsim_ds_socket/src/main/native/include/DSCommPacket.h",
            "simulation/halsim_ds_socket/src/test/native/cpp/DSCommPacketTest.cpp",
            "simulation/halsim_gui/src/main/native/cpp/DriverStationGui.cpp",
            "simulation/halsim_ws_core/src/main/native/cpp/WSProvider_DriverStation.cpp",
            "simulation/halsim_ws_core/src/main/native/cpp/WSProvider_Joystick.cpp",
            "wpilibc/src/main/native/cpp/DriverStation.cpp",
            "wpilibc/src/test/native/cpp/simulation/DriverStationSimTest.cpp",
            "wpilibcExamples/src/test/cpp/examples/DigitalCommunication/cpp/DigitalCommunicationTest.cpp",
            "wpilibcExamples/src/test/cpp/examples/I2CCommunication/cpp/I2CCommunicationTest.cpp",
        ],
        remove_k(
            "HAL_AllianceStationID_kUnknown",
            "HAL_AllianceStationID_kRed1",
            "HAL_AllianceStationID_kRed2",
            "HAL_AllianceStationID_kRed3",
            "HAL_AllianceStationID_kBlue1",
            "HAL_AllianceStationID_kBlue2",
            "HAL_AllianceStationID_kBlue3",
            "HAL_kMatchType_none",
            "HAL_kMatchType_practice",
            "HAL_kMatchType_qualification",
            "HAL_kMatchType_elimination",
            "HAL_kMaxJoystickAxes",
            "HAL_kMaxJoystickPOVs",
            "HAL_kMaxJoysticks",
        ),
    )

    s.sub(
        "hal InvalidHandle",
        [
            "hal/src/main/native/cpp/handles/HandlesInternal.cpp",
            "hal/src/main/native/cpp/jni/AddressableLEDJNI.cpp",
            "hal/src/main/native/cpp/jni/AnalogJNI.cpp",
            "hal/src/main/native/cpp/jni/CANAPIJNI.cpp",
            "hal/src/main/native/cpp/jni/CTREPCMJNI.cpp",
            "hal/src/main/native/cpp/jni/CounterJNI.cpp",
            "hal/src/main/native/cpp/jni/DIOJNI.cpp",
            "hal/src/main/native/cpp/jni/DutyCycleJNI.cpp",
            "hal/src/main/native/cpp/jni/EncoderJNI.cpp",
            "hal/src/main/native/cpp/jni/NotifierJNI.cpp",
            "hal/src/main/native/cpp/jni/PWMJNI.cpp",
            "hal/src/main/native/cpp/jni/PowerDistributionJNI.cpp",
            "hal/src/main/native/cpp/jni/SerialPortJNI.cpp",
            "hal/src/main/native/cpp/jni/SimDeviceJNI.cpp",
            "hal/src/main/native/cpp/jni/simulation/BufferCallbackStore.cpp",
            "hal/src/main/native/cpp/jni/simulation/CallbackStore.cpp",
            "hal/src/main/native/cpp/jni/simulation/ConstBufferCallbackStore.cpp",
            "hal/src/main/native/cpp/jni/simulation/SimDeviceDataJNI.cpp",
            "hal/src/main/native/include/hal/SimDevice.h",
            "hal/src/main/native/include/hal/Types.h",
            "hal/src/main/native/include/hal/handles/DigitalHandleResource.h",
            "hal/src/main/native/include/hal/handles/IndexedClassedHandleResource.h",
            "hal/src/main/native/include/hal/handles/IndexedHandleResource.h",
            "hal/src/main/native/include/hal/handles/LimitedClassedHandleResource.h",
            "hal/src/main/native/include/hal/handles/LimitedHandleResource.h",
            "hal/src/main/native/include/hal/handles/UnlimitedHandleResource.h",
            "hal/src/main/native/sim/AddressableLED.cpp",
            "hal/src/main/native/sim/AnalogInput.cpp",
            "hal/src/main/native/sim/CANAPI.cpp",
            "hal/src/main/native/sim/CTREPCM.cpp",
            "hal/src/main/native/sim/DIO.cpp",
            "hal/src/main/native/sim/DutyCycle.cpp",
            "hal/src/main/native/sim/Encoder.cpp",
            "hal/src/main/native/sim/Notifier.cpp",
            "hal/src/main/native/sim/PWM.cpp",
            "hal/src/main/native/sim/PowerDistribution.cpp",
            "hal/src/main/native/sim/REVPH.cpp",
            "hal/src/main/native/sim/SerialPort.cpp",
            "hal/src/main/native/systemcore/AddressableLED.cpp",
            "hal/src/main/native/systemcore/AnalogInput.cpp",
            "hal/src/main/native/systemcore/CANAPI.cpp",
            "hal/src/main/native/systemcore/CTREPCM.cpp",
            "hal/src/main/native/systemcore/CTREPDP.cpp",
            "hal/src/main/native/systemcore/Counter.cpp",
            "hal/src/main/native/systemcore/DIO.cpp",
            "hal/src/main/native/systemcore/DutyCycle.cpp",
            "hal/src/main/native/systemcore/Encoder.cpp",
            "hal/src/main/native/systemcore/Notifier.cpp",
            "hal/src/main/native/systemcore/PWM.cpp",
            "hal/src/main/native/systemcore/PowerDistribution.cpp",
            "hal/src/main/native/systemcore/REVPDH.cpp",
            "hal/src/main/native/systemcore/REVPH.cpp",
            "hal/src/main/native/systemcore/SerialPort.cpp",
            "hal/src/test/native/cpp/can/CANTest.cpp",
            "hal/src/test/native/cpp/mockdata/AnalogInDataTest.cpp",
            "hal/src/test/native/cpp/mockdata/DIODataTest.cpp",
            "hal/src/test/native/cpp/mockdata/PCMDataTest.cpp",
            "hal/src/test/native/cpp/mockdata/PWMDataTest.cpp",
            "wpilibc/src/main/native/cpp/AddressableLED.cpp",
            "wpilibc/src/main/native/cpp/DigitalOutput.cpp",
            "wpilibc/src/main/native/cpp/Notifier.cpp",
            "wpilibc/src/main/native/cpp/PWM.cpp",
            "wpilibc/src/main/native/cpp/PneumaticHub.cpp",
            "wpilibc/src/main/native/cpp/PneumaticsControlModule.cpp",
            "wpilibc/src/main/native/cpp/TimedRobot.cpp",
        ],
        remove_k("HAL_kInvalidHandle"),
    )

    s.sub(
        "hal PowerDistributionType",
        [
            "hal/src/main/native/cpp/jni/PowerDistributionJNI.cpp",
            "hal/src/main/native/include/hal/PowerDistribution.h",
            "hal/src/main/native/sim/PowerDistribution.cpp",
            "hal/src/main/native/systemcore/PowerDistribution.cpp",
            "hal/src/test/native/cpp/mockdata/PDPDataTest.cpp",
            "wpilibc/src/main/native/cpp/PowerDistribution.cpp",
        ],
        remove_k(
            "HAL_PowerDistributionType_kAutomatic",
            "HAL_PowerDistributionType_kCTRE",
            # Rev -> REV
            ("HAL_PowerDistributionType_kRev", "HAL_PowerDistributionType_REV"),
        ),
    )
    s.sub(
        "hal REVPHCompressorConfigType",
        [
            "hal/src/main/native/cpp/jni/REVPHJNI.cpp",
            "hal/src/main/native/include/hal/REVPH.h",
            "hal/src/main/native/sim/REVPH.cpp",
            "hal/src/main/native/sim/mockdata/REVPHData.cpp",
            "hal/src/main/native/sim/mockdata/REVPHDataInternal.h",
            "hal/src/main/native/systemcore/REVPH.cpp",
            "hal/src/main/native/systemcore/mockdata/REVPHData.cpp",
            "wpilibc/src/main/native/cpp/PneumaticsBase.cpp",
        ],
        remove_k(
            "HAL_REVPHCompressorConfigType_kDisabled",
            "HAL_REVPHCompressorConfigType_kDigital",
            "HAL_REVPHCompressorConfigType_kAnalog",
            "HAL_REVPHCompressorConfigType_kHybrid",
        ),
    )
    s.sub(
        "hal AddressableLED",
        [
            "hal/src/main/native/cpp/jni/simulation/AddressableLEDDataJNI.cpp",
            "hal/src/main/native/include/hal/AddressableLEDTypes.h",
            "hal/src/main/native/sim/AddressableLED.cpp",
            "hal/src/main/native/sim/mockdata/AddressableLEDData.cpp",
            "hal/src/main/native/sim/mockdata/AddressableLEDDataInternal.h",
            "simulation/halsim_gui/src/main/native/cpp/AddressableLEDGui.cpp",
        ],
        remove_k("HAL_kAddressableLEDMaxLength"),
    )
    s.sub(
        "hal Encoder",
        [
            "hal/src/main/native/include/hal/Encoder.h",
            "hal/src/main/native/sim/Encoder.cpp",
            "hal/src/main/native/systemcore/Encoder.cpp",
            "wpilibc/src/main/native/cpp/Encoder.cpp",
        ],
        remove_k(
            "HAL_kResetWhileHigh",
            "HAL_kResetWhileLow",
            "HAL_kResetOnFallingEdge",
            "HAL_kResetOnRisingEdge",
            "HAL_Encoder_k1X",
            "HAL_Encoder_k2X",
            "HAL_Encoder_k4X",
        ),
    )
    s.sub(
        "hal I2CPort",
        [
            "hal/src/main/native/include/hal/I2CTypes.h",
            "hal/src/test/native/cpp/mockdata/I2CDataTest.cpp",
            "wpilibc/src/main/native/include/frc/I2C.h",
        ],
        remove_k(
            "HAL_I2C_kInvalid",
            "HAL_I2C_kPort0",
            "HAL_I2C_kPort1",
        ),
    )

    s.sub(
        "hal sim AnalogInternal.h",
        [
            "hal/src/main/native/sim/AnalogInput.cpp",
            "hal/src/main/native/sim/AnalogInternal.h",
        ],
        normal_substitute(
            ("kTimebase", "TIME_BASE"),
            "kDefaultOversampleBits",
            "kDefaultAverageBits",
            "kDefaultSampleRate",
            "kAccumulatorChannels",
        ),
    )
    s.sub(
        "hal sim+systemcore Constants",
        [
            "hal/src/main/native/sim/Constants.cpp",
            "hal/src/main/native/sim/ConstantsInternal.h",
            "hal/src/main/native/systemcore/Constants.cpp",
            "hal/src/main/native/systemcore/ConstantsInternal.h",
        ],
        normal_substitute(
            "kSystemClockTicksPerMicrosecond",
        ),
    )
    s.sub(
        "hal sim DigitalInternal.h",
        [
            "hal/src/main/native/sim/DIO.cpp",
            "hal/src/main/native/sim/DigitalInternal.h",
        ],
        normal_substitute(
            "kPwmDisabled",
            "kMXPDigitalPWMOffset",
            "kExpectedLoopTiming",
            "kDefaultPwmPeriod",
        ),
    )
    s.sub(
        "hal sim DriverStation.cpp",
        [
            "hal/src/main/native/sim/DriverStation.cpp",
        ],
        normal_substitute(
            "kJoystickPorts",
        ),
    )
    s.sub(
        "hal sim DriverStationData",
        [
            "hal/src/main/native/sim/mockdata/DriverStationData.cpp",
            "hal/src/main/native/sim/mockdata/DriverStationDataInternal.h",
        ],
        normal_substitute(
            "kNumJoysticks",
        ),
    )

    s.sub(
        "hal sim PortsInternal.h",
        [
            "hal/src/main/native/sim/AddressableLED.cpp",
            "hal/src/main/native/sim/AnalogInput.cpp",
            "hal/src/main/native/sim/AnalogInternal.cpp",
            "hal/src/main/native/sim/AnalogInternal.h",
            "hal/src/main/native/sim/CANAPI.cpp",
            "hal/src/main/native/sim/CTREPCM.cpp",
            "hal/src/main/native/sim/Counter.cpp",
            "hal/src/main/native/sim/CounterInternal.h",
            "hal/src/main/native/sim/DIO.cpp",
            "hal/src/main/native/sim/DigitalInternal.cpp",
            "hal/src/main/native/sim/DigitalInternal.h",
            "hal/src/main/native/sim/DutyCycle.cpp",
            "hal/src/main/native/sim/Encoder.cpp",
            "hal/src/main/native/sim/PWM.cpp",
            "hal/src/main/native/sim/Ports.cpp",
            "hal/src/main/native/sim/PortsInternal.h",
            "hal/src/main/native/sim/PowerDistribution.cpp",
            "hal/src/main/native/sim/REVPH.cpp",
            "hal/src/main/native/sim/mockdata/AddressableLEDData.cpp",
            "hal/src/main/native/sim/mockdata/AnalogInData.cpp",
            "hal/src/main/native/sim/mockdata/CTREPCMData.cpp",
            "hal/src/main/native/sim/mockdata/CTREPCMDataInternal.h",
            "hal/src/main/native/sim/mockdata/DIOData.cpp",
            "hal/src/main/native/sim/mockdata/DigitalPWMData.cpp",
            "hal/src/main/native/sim/mockdata/DutyCycleData.cpp",
            "hal/src/main/native/sim/mockdata/EncoderData.cpp",
            "hal/src/main/native/sim/mockdata/I2CData.cpp",
            "hal/src/main/native/sim/mockdata/PWMData.cpp",
            "hal/src/main/native/sim/mockdata/PowerDistributionData.cpp",
            "hal/src/main/native/sim/mockdata/PowerDistributionDataInternal.h",
            "hal/src/main/native/sim/mockdata/REVPHData.cpp",
            "hal/src/main/native/sim/mockdata/REVPHDataInternal.h",
            "hal/src/main/native/sim/mockdata/Reset.cpp",
        ],
        normal_substitute(
            "kNumCanBuses",
            "kAccelerometers",
            "kNumAccumulators",
            "kNumAnalogInputs",
            "kNumAnalogOutputs",
            "kNumCounters",
            "kNumDigitalHeaders",
            "kNumPWMHeaders",
            "kNumDigitalChannels",
            "kNumPWMChannels",
            "kNumDigitalPWMOutputs",
            "kNumEncoders",
            "kI2CPorts",
            "kNumInterrupts",
            "kNumRelayChannels",
            "kNumRelayHeaders",
            ("kNumCTREPCMModules", "NUM_CTRE_PCM_MODULES"),
            "kNumCTRESolenoidChannels",
            ("kNumCTREPDPModules", "NUM_CTRE_PDP_MODULES"),
            ("kNumCTREPDPChannels", "NUM_CTRE_PDP_CHANNELS"),
            ("kNumREVPDHModules", "NUM_REV_PDH_MODULES"),
            ("kNumREVPDHChannels", "NUM_REV_PDH_CHANNELS"),
            "kNumPDSimModules",
            "kNumPDSimChannels",
            "kNumDutyCycles",
            ("kNumAddressableLEDs", "NUM_ADDRESSABLE_LEDS"),
            ("kNumREVPHModules", "NUM_REV_PH_MODULES"),
            ("kNumREVPHChannels", "NUM_REV_PH_CHANNELS"),
            "kSPIAccelerometers",
            "kSPIPorts",
        ),
    )

    s.sub(
        "hal systemcore REVPDH",
        [
            "hal/src/main/native/systemcore/REVPDH.cpp",
        ],
        normal_substitute(
            "kDefaultControlPeriod",
            ("kPDHFrameStatus0Timeout", "PDH_FRAME_STATUS_0_TIMEOUT"),
            ("kPDHFrameStatus1Timeout", "PDH_FRAME_STATUS_1_TIMEOUT"),
            ("kPDHFrameStatus2Timeout", "PDH_FRAME_STATUS_2_TIMEOUT"),
            ("kPDHFrameStatus3Timeout", "PDH_FRAME_STATUS_3_TIMEOUT"),
            ("kPDHFrameStatus4Timeout", "PDH_FRAME_STATUS_4_TIMEOUT"),
        ),
    )
    s.sub(
        "hal systemcore REVPH",
        [
            "hal/src/main/native/systemcore/REVPH.cpp",
        ],
        normal_substitute(
            "kDefaultControlPeriod",
            "kDefaultCompressorDuty",
            "kDefaultPressureTarget",
            "kDefaultPressureHysteresis",
            ("kPHFrameStatus0Timeout", "PH_FRAME_STATUS_0_TIMEOUT"),
            ("kPHFrameStatus1Timeout", "PH_FRAME_STATUS_1_TIMEOUT"),
            "kSolenoidDisabled",
            "kSolenoidEnabled",
            "kSolenoidControlledViaPulse",
        ),
    )
    s.sub(
        "hal systemcore SmartIo.h",
        [
            "hal/src/main/native/systemcore/SmartIo.h",
        ],
        normal_substitute(
            "kPwmDisabled",
            "kPwmAlwaysHigh",
        ),
    )

    s.sub(
        "hal systemcore PortsInternal",
        [
            "hal/src/main/native/systemcore/AnalogInput.cpp",
            "hal/src/main/native/systemcore/CAN.cpp",
            "hal/src/main/native/systemcore/CANAPI.cpp",
            "hal/src/main/native/systemcore/CTREPCM.cpp",
            "hal/src/main/native/systemcore/CTREPDP.cpp",
            "hal/src/main/native/systemcore/Counter.cpp",
            "hal/src/main/native/systemcore/DIO.cpp",
            "hal/src/main/native/systemcore/DutyCycle.cpp",
            "hal/src/main/native/systemcore/I2C.cpp",
            "hal/src/main/native/systemcore/PWM.cpp",
            "hal/src/main/native/systemcore/Ports.cpp",
            "hal/src/main/native/systemcore/PortsInternal.h",
            "hal/src/main/native/systemcore/PowerDistribution.cpp",
            "hal/src/main/native/systemcore/REVPDH.cpp",
            "hal/src/main/native/systemcore/REVPDH.h",
            "hal/src/main/native/systemcore/REVPH.cpp",
            "hal/src/main/native/systemcore/SmartIo.cpp",
            "hal/src/main/native/systemcore/SmartIo.h",
        ],
        normal_substitute(
            "kNumCanBuses",
            "kNumSmartIo",
            "kNumI2cBuses",
            "kNumAccumulators",
            "kNumAnalogInputs",
            "kNumAnalogOutputs",
            "kNumCounters",
            "kNumDigitalHeaders",
            "kNumDigitalMXPChannels",
            "kNumDigitalSPIPortChannels",
            "kNumPWMHeaders",
            "kNumDigitalChannels",
            "kNumPWMChannels",
            "kNumDigitalPWMOutputs",
            "kNumEncoders",
            "kNumInterrupts",
            "kNumRelayChannels",
            "kNumRelayHeaders",
            ("kNumCTREPCMModules", "NUM_CTRE_PCM_MODULES"),
            "kNumCTRESolenoidChannels",
            ("kNumCTREPDPModules", "NUM_CTRE_PDP_MODULES"),
            ("kNumCTREPDPChannels", "NUM_CTRE_PDP_CHANNELS"),
            ("kNumREVPDHModules", "NUM_REV_PDH_MODULES"),
            ("kNumREVPDHChannels", "NUM_REV_PDH_CHANNELS"),
            "kNumDutyCycles",
            ("kNumAddressableLEDs", "NUM_ADDRESSABLE_LEDS"),
            ("kNumREVPHModules", "NUM_REV_PH_MODULES"),
            ("kNumREVPHChannels", "NUM_REV_PH_CHANNELS"),
        ),
    )

    # ntcore

    s.sub(
        "ntcore jinja type name",
        [
            "ntcore/src/generate/main/java/NetworkTableValue.java.jinja",
            "ntcore/src/generate/main/java/Topic.java.jinja",
            "ntcore/src/generate/main/native/include/networktables/Topic.h.jinja",
        ],
        make_substitution_strs(
            ("k{{ t.TypeName }}", "{{ t.TYPE_NAME }}"),
            ("k{{ TypeName }}", "{{ TYPE_NAME }}"),
            ("NT_{{ cpp.TYPE_NAME }}", "NT_{{ TYPE_NAME }}"),
        ),
    )

    s.sub(
        "ntcore NetworkTableInstance NetworkMode",
        [
            "ntcore/src/main/native/include/networktables/NetworkTableInstance.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotBase.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/PreferencesTest.java",
        ],
        normal_substitute(
            "kNetModeNone",
            "kNetModeServer",
            "kNetModeClient",
            "kNetModeLocal",
            "kStarting",
        ),
    )
    s.sub(
        "ntcore NetworkTableInstance.LogLevel",
        [
            "ntcore/src/main/native/include/networktables/NetworkTableInstance.h",
        ],
        normal_substitute(
            "kLogCritical",
            "kLogError",
            "kLogWarning",
            "kLogInfo",
            "kLogDebug",
            "kLogDebug1",
            "kLogDebug2",
            "kLogDebug3",
            "kLogDebug4",
        ),
    )

    s.sub(
        "ntcore java LogMessage",
        [
            "ntcore/src/main/java/edu/wpi/first/networktables/LogMessage.java",
            "ntcore/src/test/java/edu/wpi/first/networktables/LoggerTest.java",
        ],
        normal_substitute(
            "kCritical",
            "kError",
            "kWarning",
            "kInfo",
            "kDebug",
            "kDebug1",
            "kDebug2",
            "kDebug3",
            "kDebug4",
        ),
    )
    s.sub(
        "ntcore java NetworkTableEvent Kind",
        [
            "ntcore/src/generate/main/java/NetworkTableInstance.java.jinja",
            "ntcore/src/main/java/edu/wpi/first/networktables/NetworkTable.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/NetworkTableEvent.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/NetworkTableListenerPoller.java",
            "ntcore/src/test/java/edu/wpi/first/networktables/ConnectionListenerTest.java",
            "ntcore/src/test/java/edu/wpi/first/networktables/TableListenerTest.java",
            "ntcore/src/test/java/edu/wpi/first/networktables/TopicListenerTest.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Preferences.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotBase.java",
        ],
        normal_substitute(
            "kImmediate",
            "kConnected",
            "kDisconnected",
            "kConnection",
            "kPublish",
            "kUnpublish",
            "kProperties",
            "kTopic",
            "kValueRemote",
            "kValueLocal",
            "kValueAll",
            "kLogMessage",
            "kTimeSync",
        ),
    )
    s.sub(
        "ntcore NetworkTableInstance",
        [
            "ntcore/src/generate/main/java/NetworkTableInstance.java.jinja",
            "ntcore/src/main/native/include/networktables/NetworkTableInstance.h",
            "ntcore/src/test/java/edu/wpi/first/networktables/LoggerTest.java",
        ],
        normal_substitute(
            "kServer",
            "kClient",
            "kStarting",
            "kLocal",
            "kDefaultPort",
        ),
    )
    s.sub(
        "ntcore NetworkTableType",
        [
            "ntcore/src/generate/main/java/GenericEntryImpl.java.jinja",
            "ntcore/src/generate/main/java/GenericSubscriber.java.jinja",
            "ntcore/src/generate/main/java/NetworkTableEntry.java.jinja",
            "ntcore/src/generate/main/java/NetworkTableValue.java.jinja",
            "ntcore/src/main/java/edu/wpi/first/networktables/NetworkTableType.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/ProtobufTopic.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/StructArrayTopic.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/StructTopic.java",
            "ntcore/src/main/native/include/networktables/NetworkTableType.h",
        ],
        normal_substitute(
            "kUnassigned",
            "kBoolean",
            "kDouble",
            "kString",
            "kRaw",
            "kBooleanArray",
            "kDoubleArray",
            "kStringArray",
            "kInteger",
            "kFloat",
            "kIntegerArray",
            "kFloatArray",
        ),
    )
    s.sub(
        "ntcore PubSubOptions",
        [
            "ntcore/src/generate/main/native/include/networktables/Topic.h.jinja",
            "ntcore/src/main/java/edu/wpi/first/networktables/PubSubOptions.java",
            "ntcore/src/main/native/cpp/PubSubOptions.h",
            "ntcore/src/main/native/cpp/net/WireEncoder.cpp",
            "ntcore/src/main/native/cpp/server/ServerSubscriber.cpp",
            "ntcore/src/main/native/include/networktables/MultiSubscriber.h",
            "ntcore/src/main/native/include/networktables/ProtobufTopic.h",
            "ntcore/src/main/native/include/networktables/StructArrayTopic.h",
            "ntcore/src/main/native/include/networktables/StructTopic.h",
            "ntcore/src/main/native/include/networktables/Topic.h",
            "ntcore/src/main/native/include/networktables/UnitTopic.h",
            "ntcore/src/main/native/include/ntcore_cpp.h",
            "ntcore/src/test/native/cpp/LocalStorageTest.cpp",
        ],
        normal_substitute(
            "kDefaultPubSubOptions",
            "kDefaultPeriodicMs",
            "kDefaultPeriodic",
        ),
    )
    s.sub(
        "ntcore Topic",
        [
            "glass/src/libnt/native/cpp/NTStringChooser.cpp",
            "ntcore/src/generate/main/java/Topic.java.jinja",
            "ntcore/src/generate/main/native/include/networktables/Topic.h.jinja",
            "ntcore/src/main/native/include/networktables/UnitTopic.h",
            "wpilibc/src/main/native/cpp/DriverStation.cpp",
            "wpilibc/src/main/native/cpp/Preferences.cpp",
            "wpilibc/src/main/native/cpp/smartdashboard/MechanismLigament2d.cpp",
            "wpilibc/src/main/native/cpp/smartdashboard/SendableBuilderImpl.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DriverStation.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Preferences.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/smartdashboard/MechanismLigament2d.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/smartdashboard/SendableBuilderImpl.java",
        ],
        normal_substitute(
            "kTypeString",
        ),
    )
    s.sub(
        "ntcore c++ net ClientMessageQueue",
        [
            "ntcore/src/main/native/cpp/NetworkClient.h",
            "ntcore/src/main/native/cpp/NetworkServer.h",
            "ntcore/src/main/native/cpp/net/ClientMessageQueue.h",
        ],
        normal_substitute(
            "kBlockSize",
        ),
    )
    s.sub(
        "ntcore c++ net Message",
        [
            "ntcore/src/main/native/cpp/net/Message.h",
            "ntcore/src/main/native/cpp/net/WireDecoder.cpp",
            "ntcore/src/main/native/cpp/net/WireEncoder.cpp",
        ],
        normal_substitute(
            "kMethodStr",
        ),
    )
    s.sub(
        "ntcore c++ net NetworkOutgoingQueue",
        [
            "ntcore/src/main/native/cpp/net/NetworkOutgoingQueue.h",
        ],
        normal_substitute(
            "kMinPeriodMs",
            "kOutgoingLimit",
        ),
    )
    s.sub(
        "ntcore c++ net ValueSendMode",
        [
            "ntcore/src/main/native/cpp/net/ClientImpl.cpp",
            "ntcore/src/main/native/cpp/net/NetworkOutgoingQueue.h",
            "ntcore/src/main/native/cpp/server/ServerClient4Base.cpp",
            "ntcore/src/main/native/cpp/server/ServerStorage.cpp",
            "ntcore/src/main/native/cpp/server/ServerTopic.h",
        ],
        normal_substitute(
            "kDisabled",
            "kAll",
            "kNormal",
            "kImm",
        ),
    )
    s.sub(
        "ntcore c++ net NetworkPing",
        [
            "ntcore/src/main/native/cpp/net/ClientImpl.cpp",
            "ntcore/src/main/native/cpp/net/ClientImpl.h",
            "ntcore/src/main/native/cpp/net/NetworkPing.cpp",
            "ntcore/src/main/native/cpp/net/NetworkPing.h",
        ],
        normal_substitute(
            "kPingIntervalMs",
            "kPingTimeoutMs",
        ),
    )
    s.sub(
        "ntcore c++ server Consants",
        [
            "ntcore/src/main/native/cpp/server/Constants.h",
            "ntcore/src/main/native/cpp/server/ServerSubscriber.h",
        ],
        normal_substitute(
            "kMinPeriodMs",
        ),
    )

    s.sub(
        "ntcore c++ EventFlags",
        [
            "glass/src/libnt/native/cpp/NTField2D.cpp",
            "glass/src/libnt/native/cpp/NTMechanism2D.cpp",
            "glass/src/libnt/native/cpp/NetworkTables.cpp",
            "glass/src/libnt/native/cpp/NetworkTablesProvider.cpp",
            "ntcore/src/main/native/cpp/LoggerImpl.cpp",
            "ntcore/src/main/native/include/ntcore_cpp.h",
            "ntcore/src/test/native/cpp/ConnectionListenerTest.cpp",
            "ntcore/src/test/native/cpp/LoggerTest.cpp",
            "ntcore/src/test/native/cpp/TopicListenerTest.cpp",
            "ntcore/src/test/native/cpp/ValueListenerTest.cpp",
            "ntcore/src/test/native/cpp/server/ServerImplTest.cpp",
            "wpilibc/src/main/native/cppcs/RobotBase.cpp",
        ],
        normal_substitute(
            "kNone",
            "kImmediate",
            "kConnected",
            "kDisconnected",
            "kConnection",
            "kPublish",
            "kUnpublish",
            "kProperties",
            "kTopic",
            "kValueRemote",
            "kValueLocal",
            "kValueAll",
            "kLogMessage",
            "kTimeSync",
        ),
    )
    s.sub(
        "ntcore c++ Handle",
        [
            "ntcore/src/main/native/cpp/ConnectionList.h",
            "ntcore/src/main/native/cpp/Handle.h",
            "ntcore/src/main/native/cpp/ListenerStorage.h",
            "ntcore/src/main/native/cpp/LocalStorage.cpp",
            "ntcore/src/main/native/cpp/local/LocalDataLogger.h",
            "ntcore/src/main/native/cpp/local/LocalEntry.h",
            "ntcore/src/main/native/cpp/local/LocalMultiSubscriber.h",
            "ntcore/src/main/native/cpp/local/LocalPublisher.h",
            "ntcore/src/main/native/cpp/local/LocalStorageImpl.cpp",
            "ntcore/src/main/native/cpp/local/LocalStorageImpl.h",
            "ntcore/src/main/native/cpp/local/LocalSubscriber.h",
            "ntcore/src/main/native/cpp/local/LocalTopic.h",
            "ntcore/src/main/native/cpp/ntcore_cpp.cpp",
            # "ntcore/src/test/native/cpp/LoggerTest.cpp", # Substitutions covered by EventFlags
            "ntcore/src/test/native/cpp/TestPrinters.cpp",
        ],
        normal_substitute(
            "kListener",
            "kListenerPoller",
            "kEntry",
            "kInstance",
            "kDataLogger",
            "kConnectionDataLogger",
            "kMultiSubscriber",
            "kTopic",
            "kSubscriber",
            "kPublisher",
            "kTypeMax",
            "kIndexMax",
        ),
    )
    s.sub(
        "ntcore c++ HandleType",
        [
            "ntcore/src/main/native/cpp/ConnectionList.h",
            "ntcore/src/main/native/cpp/HandleMap.h",
            "ntcore/src/main/native/cpp/ListenerStorage.h",
            "ntcore/src/main/native/cpp/Value_internal.h",
            "ntcore/src/main/native/cpp/local/LocalDataLogger.h",
            "ntcore/src/main/native/cpp/local/LocalEntry.h",
            "ntcore/src/main/native/cpp/local/LocalMultiSubscriber.h",
            "ntcore/src/main/native/cpp/local/LocalPublisher.h",
            "ntcore/src/main/native/cpp/local/LocalSubscriber.h",
            "ntcore/src/main/native/cpp/local/LocalTopic.h",
        ],
        normal_substitute(
            "kType",
        ),
    )

    s.sub(
        "ntcore InstanceImpl.h InstanceImpl.cpp",
        [
            "ntcore/src/main/native/cpp/InstanceImpl.cpp",
            "ntcore/src/main/native/cpp/InstanceImpl.h",
        ],
        normal_substitute(
            "kNumInstances",
        ),
    )
    s.sub(
        "ntcore LoggerImpl.cpp",
        [
            "ntcore/src/main/native/cpp/LoggerImpl.cpp",
        ],
        normal_substitute(
            "kFlagCritical",
            "kFlagError",
            "kFlagWarning",
            "kFlagInfo",
            "kFlagDebug",
            "kFlagDebug1",
            "kFlagDebug2",
            "kFlagDebug3",
            "kFlagDebug4",
        ),
    )
    s.sub(
        "ntcore NetworkClient.cpp",
        [
            "ntcore/src/main/native/cpp/NetworkClient.cpp",
        ],
        normal_substitute(
            "kReconnectRate",
            "kWebsocketHandshakeTimeout",
            "kMaxMessageSize",
        ),
    )
    s.sub(
        "ntcore NetworkServer.cpp",
        [
            "ntcore/src/main/native/cpp/NetworkServer.cpp",
        ],
        normal_substitute(
            "kMaxMessageSize",
            "kClientProcessMessageCountMax",
        ),
    )
    s.sub(
        "ntcore LocalStorageImpl.cpp",
        [
            "ntcore/src/main/native/cpp/local/LocalStorageImpl.cpp",
        ],
        normal_substitute(
            "kMaxPublishers",
            "kMaxSubscribers",
            "kMaxMultiSubscribers",
            "kMaxListeners",
        ),
    )
    s.sub(
        "ntcore net ClientImpl.h ClientImpl.cpp",
        [
            "ntcore/src/main/native/cpp/net/ClientImpl.cpp",
            "ntcore/src/main/native/cpp/net/ClientImpl.h",
        ],
        normal_substitute(
            "kRttIntervalMs",
            "kMinPeriodMs",
            "kMaxPeriodMs",
        ),
    )
    s.sub(
        "ntcore net WebSocketConnection.cpp WebSocketConnection.h",
        [
            "ntcore/src/main/native/cpp/net/WebSocketConnection.cpp",
            "ntcore/src/main/native/cpp/net/WebSocketConnection.h",
        ],
        normal_substitute(
            "kMTU",
            "kAllocSize",
            "kNewFrameThresholdBytes",
            "kFlushThresholdFrames",
            "kFlushThresholdBytes",
            "kMaxPoolSize",
            ("wpi::WebSocket::kFlagFin", "wpi::WebSocket::FLAG_FIN"),  # TODO Move to WebSocket section
            ("wpi::WebSocket::Frame::kFragment", "wpi::WebSocket::Frame::FRAGMENT"),  # TODO
            ("wpi::WebSocket::Frame::kText", "wpi::WebSocket::Frame::TEXT"),  # TODO
            ("wpi::WebSocket::Frame::kBinary", "wpi::WebSocket::Frame::BINARY"),  # TODO
            "kEmpty",
            "kText",
            "kBinary",
        ),
    )
    s.sub(
        "ntcore c++ server ServerClient4.cpp",
        [
            "ntcore/src/main/native/cpp/server/ServerClient4.cpp",
        ],
        normal_substitute(
            "kMaxImmProcessing",
        ),
    )
    s.sub(
        "ntcore c++ test LocalStorageTest.cpp",
        [
            "ntcore/src/test/native/cpp/LocalStorageTest.cpp",
        ],
        normal_substitute(
            "kDefaultPubSubOptionsImpl",
        ),
    )

    # ntcorefii

    s.sub(
        "ntcoreffi DataLogmanager.cpp",
        [
            "ntcoreffi/src/main/native/cpp/DataLogManager.cpp",
        ],
        normal_substitute(
            "kNone",
            "kPractice",
            "kQualification",
            "kElimination",
            "kMatchType_none",
            "kMatchType_practice",
            "kMatchType_qualification",
            "kMatchType_elimination",
            ("kRoboRIO", "ROBORIO"),
            "kFreeSpaceThreshold",
            "kFileCountThreshold",
            ("nLoadOut::kTargetClass_RoboRIO2", "nLoadOut::TARGET_CLASS_ROBORIO2"),
        ),
    )

    # romi

    s.sub(
        "romi OnBoardIO",
        [
            "romiVendordep/src/main/native/cpp/romi/OnBoardIO.cpp",
            "romiVendordep/src/main/native/include/frc/romi/OnBoardIO.h",
        ],
        normal_substitute(
            "kMessageInterval",
        ),
    )

    # simulation

    s.sub(
        "simulation halsim_ds_socket DSCommPacket",
        [
            "simulation/halsim_ds_socket/src/main/native/cpp/DSCommPacket.cpp",
            "simulation/halsim_ds_socket/src/main/native/include/DSCommPacket.h",
        ],
        normal_substitute(
            "kGameDataTag",
            "kJoystickNameTag",
            "kMatchInfoTag",
            "kJoystickDataTag",
            "kMatchTimeTag",
            "kTest",
            "kEnabled",
            "kAutonomous",
            "kFMS_Attached",
            "kEmergencyStop",
            "kRequestNormalMask",
            "kRobotHasCode",
            "kUnknown",
            "kCommVersion",
            "kHIDTag",
        ),
    )
    s.sub(
        "simulation halsim_gui DriverStationGui.cpp",
        [
            "simulation/halsim_gui/src/main/native/cpp/DriverStationGui.cpp",
        ],
        normal_substitute(
            "kMaxButtonCount",
        ),
    )
    s.sub(
        "simulation halsim_ws_client HALSimWS.cpp",
        [
            "simulation/halsim_ws_client/src/main/native/cpp/HALSimWS.cpp",
        ],
        normal_substitute(
            "kTcpConnectAttemptTimeout",
        ),
    )
    s.sub(
        "simulation halsim_ws_server WebServerClientTest.cpp",
        [
            "simulation/halsim_ws_server/src/test/native/cpp/WebServerClientTest.cpp",
        ],
        normal_substitute(
            "kTcpConnectAttemptTimeout",
        ),
    )

    # sysid

    s.sub(
        "sysid gains",
        [
            "sysid/src/main/native/cpp/analysis/AnalysisManager.cpp",
            "sysid/src/main/native/cpp/analysis/ArmSim.cpp",
            "sysid/src/main/native/cpp/analysis/ElevatorSim.cpp",
            "sysid/src/main/native/cpp/analysis/FeedbackAnalysis.cpp",
            "sysid/src/main/native/cpp/analysis/FeedforwardAnalysis.cpp",
            "sysid/src/main/native/cpp/analysis/SimpleMotorSim.cpp",
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/cpp/view/AnalyzerPlot.cpp",
            "sysid/src/main/native/include/sysid/analysis/AnalysisManager.h",
            "sysid/src/main/native/include/sysid/analysis/ArmSim.h",
            "sysid/src/main/native/include/sysid/analysis/ElevatorSim.h",
            "sysid/src/main/native/include/sysid/analysis/FeedbackAnalysis.h",
            "sysid/src/main/native/include/sysid/analysis/SimpleMotorSim.h",
            "sysid/src/main/native/include/sysid/view/AnalyzerPlot.h",
            "sysid/src/test/native/cpp/analysis/FeedbackAnalysisTest.cpp",
            "sysid/src/test/native/cpp/analysis/FeedforwardAnalysisTest.cpp",
        ],
        normal_substitute(
            ("Ks", "S"),
            ("Kv", "V"),
            ("Ka", "A"),
            ("Kg", "G"),
        ),
    )

    s.sub(
        "sysid analysis type",
        [
            "sysid/src/main/native/cpp/analysis/AnalysisManager.cpp",
            "sysid/src/main/native/cpp/analysis/FeedforwardAnalysis.cpp",
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/cpp/view/AnalyzerPlot.cpp",
            "sysid/src/main/native/include/sysid/analysis/AnalysisType.h",
            "sysid/src/test/native/cpp/analysis/AnalysisTypeTest.cpp",
            "sysid/src/test/native/cpp/analysis/FeedforwardAnalysisTest.cpp",
        ],
        normal_substitute(
            "kElevator",
            "kArm",
            "kSimple",
        ),
    )
    s.sub(
        "sysid feedback controller loop type",
        [
            "sysid/src/main/native/cpp/analysis/AnalysisManager.cpp",
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/include/sysid/analysis/AnalysisManager.h",
            "sysid/src/main/native/include/sysid/analysis/FeedbackControllerPreset.h",
        ],
        normal_substitute(
            "kPosition",
            "kVelocity",
        ),
    )
    s.sub(
        "sysid presets",
        [
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/include/sysid/analysis/AnalysisManager.h",
            "sysid/src/main/native/include/sysid/analysis/FeedbackControllerPreset.h",
            "sysid/src/test/native/cpp/analysis/FeedbackAnalysisTest.cpp",
        ],
        normal_substitute(
            "kDefault",
            ("kWPILib", "WPILIB"),
            ("kCTREv5", "CTRE_V5"),
            ("kCTREv6", "CTRE_V6"),
            ("kREVNEOBuiltIn", "REV_NEO_BUILTIN"),
            "kREVNonNEO",
            "kVenom",
        ),
    )
    s.sub(
        "sysid Analyzer",
        [
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/include/sysid/view/Analyzer.h",
        ],
        normal_substitute(
            "kWaitingForData",
            "kNominalDisplay",
            "kVelocityThresholdError",
            "kTestDurationError",
            "kGeneralDataError",
            "kMissingTestsError",
            "kFileError",
            "kPresetNames",
            "kLoopTypes",
            "kDatasets",
            "kHorizontalOffset",
        ),
    )
    s.sub(
        "sysid units",
        [
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/cpp/view/DataSelector.cpp",
            "sysid/src/main/native/include/sysid/Util.h",
        ],
        normal_substitute(
            "kUnits",
        ),
    )
    s.sub(
        "sysid noise mean window",
        [
            "sysid/src/main/native/cpp/analysis/FilteringUtils.cpp",
            "sysid/src/main/native/include/sysid/analysis/FilteringUtils.h",
        ],
        normal_substitute(
            "kNoiseMeanWindow",
        ),
    )
    s.sub(
        "sysid ui layout",
        [
            "sysid/src/main/native/cpp/App.cpp",
            "sysid/src/main/native/cpp/view/Analyzer.cpp",
            "sysid/src/main/native/include/sysid/view/UILayout.h",
        ],
        normal_substitute(
            "kAppWindowSize",
            "kMenubarHeight",
            "kWindowGap",
            "kLeftColPos",
            "kLeftColSize",
            "kLogLoaderWindowPos",
            "kLogLoaderWindowSize",
            "kDataSelectorWindowPos",
            "kDataSelectorWindowSize",
            "kCenterColPos",
            "kCenterColSize",
            "kAnalyzerWindowPos",
            "kAnalyzerWindowSize",
            "kProgramLogWindowPos",
            "kProgramLogWindowSize",
            "kRightColPos",
            "kRightColSize",
            "kDiagnosticPlotWindowPos",
            "kDiagnosticPlotWindowSize",
            "kTextBoxWidthMultiple",
        ),
    )

    s.sub(
        "sysid FilteringUtils.cpp",
        [
            "sysid/src/main/native/cpp/analysis/FilteringUtils.cpp",
        ],
        normal_substitute(
            "kWindow",
        ),
    )
    s.sub(
        "sysid AnalyzerPlot",
        [
            "sysid/src/main/native/cpp/view/AnalyzerPlot.cpp",
            "sysid/src/main/native/include/sysid/view/AnalyzerPlot.h",
        ],
        normal_substitute(
            "kMaxSize",
        ),
    )
    s.sub(
        "sysid DataSelector",
        [
            "sysid/src/main/native/cpp/view/DataSelector.cpp",
            "sysid/src/main/native/include/sysid/view/DataSelector.h",
        ],
        normal_substitute(
            "kAnalysisTypes",
            "kValidTests",
        ),
    )
    s.sub(
        "sysid AnalysisManager",
        [
            "sysid/src/main/native/include/sysid/analysis/AnalysisManager.h",
        ],
        normal_substitute(
            "kJsonDataKeys",
        ),
    )
    s.sub(
        "sysid FeedforwardAnalysisTest.cpp",
        [
            "sysid/src/test/native/cpp/analysis/FeedforwardAnalysisTest.cpp",
        ],
        normal_substitute(
            "kSlowForward",
            "kSlowBackward",
            "kFastForward",
            "kFastBackward",
            "kMovementCombinations",
            ("kUstep", "U_STEP"),
            ("kUmax", "U_MAX"),
            "kTestDuration",
        ),
    )

    # wpigui

    s.sub(
        "wpigui Style",
        [
            "wpigui/src/main/native/cpp/wpigui.cpp",
            "wpigui/src/main/native/include/wpigui.h",
        ],
        normal_substitute(
            "kStyleClassic",
            "kStyleDark",
            "kStyleLight",
            "kStyleDeepDark",
        ),
    )
    s.sub(
        "wpigui PixelFormat",
        [
            "cscore/examples/usbviewer/usbviewer.cpp",
            "wpigui/src/main/native/cpp/wpigui.cpp",
            "wpigui/src/main/native/directx11/wpigui_directx11.cpp",
            "wpigui/src/main/native/include/wpigui.h",
            "wpigui/src/main/native/metal/wpigui_metal.mm",
            "wpigui/src/main/native/opengl2/wpigui_opengl2.cpp",
            "wpigui/src/main/native/opengl3/wpigui_opengl3.cpp",
        ],
        normal_substitute(
            "kPixelRGBA",
            "kPixelBGRA",
        ),
    )
    s.sub(
        "wpigui FONT_SCALED_LEVELS",
        [
            "wpigui/src/main/native/cpp/wpigui.cpp",
            "wpigui/src/main/native/include/wpigui_internal.h",
        ],
        normal_substitute(
            "kFontScaledLevels",
        ),
    )

    # wpilibNewCommands

    False and s.sub(  # TODO Undisable after changing generate.sh
        "wpilibNewCommands jinja button and trigger names",
        [
            "wpilibNewCommands/src/generate/main/java/commandhid.java.jinja",
            "wpilibNewCommands/src/generate/main/native/cpp/frc2/command/button/commandhid.cpp.jinja",
        ],
        make_substitution_strs(
            ("k{{ capitalize_first(button.name) }}", "{{ snakecase(button.name) }}"),
            ("k{{ capitalize_first(trigger.name) }}", "{{ snakecase(trigger.name) }}"),
        ),
    )

    s.sub(
        "wpilibNewCommands Command InterruptionBehavior",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/Command.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/CommandScheduler.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/ConditionalCommand.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/ParallelCommandGroup.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/ParallelDeadlineGroup.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/ParallelRaceGroup.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/SelectCommand.java",
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/SequentialCommandGroup.java",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/Command.cpp",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/CommandScheduler.cpp",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/ConditionalCommand.cpp",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/ParallelCommandGroup.cpp",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/ParallelDeadlineGroup.cpp",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/ParallelRaceGroup.cpp",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/SequentialCommandGroup.cpp",
            "wpilibNewCommands/src/main/native/include/frc2/command/Command.h",
            "wpilibNewCommands/src/main/native/include/frc2/command/ParallelCommandGroup.h",
            "wpilibNewCommands/src/main/native/include/frc2/command/ParallelDeadlineGroup.h",
            "wpilibNewCommands/src/main/native/include/frc2/command/ParallelRaceGroup.h",
            "wpilibNewCommands/src/main/native/include/frc2/command/SelectCommand.h",
            "wpilibNewCommands/src/main/native/include/frc2/command/SequentialCommandGroup.h",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/CommandRequirementsTest.java",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/CommandTestBase.java",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/ConditionalCommandTest.java",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/MultiCompositionTestBase.java",
            "wpilibNewCommands/src/test/native/cpp/frc2/command/CommandRequirementsTest.cpp",
            "wpilibNewCommands/src/test/native/cpp/frc2/command/CompositionTestBase.h",
            "wpilibNewCommands/src/test/native/cpp/frc2/command/ConditionalCommandTest.cpp",
            "wpilibNewCommands/src/test/native/cpp/frc2/command/SchedulingRecursionTest.cpp",
        ],
        normal_substitute(
            "kCancelSelf",
            "kCancelIncoming",
        ),
    )

    s.sub(
        "wpilibNewCommands CommandScheduler NO_INTERRUPTOR",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/CommandScheduler.java",
        ],
        normal_substitute(
            "kNoInterruptor",
        ),
    )

    s.sub(
        "wpilibNewCommands sysid SysIdRoutine Direction",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutine.java",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/sysid/SysIdRoutine.cpp",
            "wpilibNewCommands/src/main/native/include/frc2/command/sysid/SysIdRoutine.h",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutineTest.java",
            "wpilibNewCommands/src/test/native/cpp/frc2/command/sysid/SysIdRoutineTest.cpp",
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/cpp/SysIdRoutineBot.cpp",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysidroutine/SysIdRoutineBot.java",
        ],
        normal_substitute(
            "kForward",
            "kReverse",
        ),
    )

    s.sub(
        "wpilibNewCommands MecanumControllerCommandTest.java",
        [
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/MecanumControllerCommandTest.java",
        ],
        normal_substitute(
            "kAngularTolerance",
            "kWheelBase",
            # NOTE This is now split into two words
            ("kTrackwidth", "TRACK_WIDTH"),
        ),
    )
    s.sub(
        "wpilibNewCommands SwerveControllerCommandTest.java",
        [
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommandTest.java",
        ],
        normal_substitute(
            "kAngularTolerance",
            "kWheelBase",
            # NOTE This is now split into two words
            ("kTrackwidth", "TRACK_WIDTH"),
        ),
    )
    s.sub(
        "wpilibNewCommands MecanumControllerCommandTest.cpp",
        [
            "wpilibNewCommands/src/test/native/cpp/frc2/command/MecanumControllerCommandTest.cpp",
        ],
        normal_substitute(
            "kxTolerance",
            "kyTolerance",
            "kAngularTolerance",
            "kWheelBase",
            # NOTE This is now split into two words
            ("kTrackwidth", "TRACK_WIDTH"),
        ),
    )
    s.sub(
        "wpilibNewCommands SwerveControllerCommandTest.cpp",
        [
            "wpilibNewCommands/src/test/native/cpp/frc2/command/SwerveControllerCommandTest.cpp",
        ],
        normal_substitute(
            "kxTolerance",
            "kyTolerance",
            "kAngularTolerance",
            "kWheelBase",
            ("kTrackwidth", "TRACK_WIDTH"),
        ),
    )

    # wpilib

    False and s.sub(  # TODO Reenable
        "wpilib jinja hid names",
        [
            "wpilibc/src/generate/main/native/cpp/hid.cpp.jinja",
            "wpilibc/src/generate/main/native/cpp/simulation/hidsim.cpp.jinja",
            "wpilibc/src/generate/main/native/include/frc/hid.h.jinja",
        ],
        normal_substitute(
            ('k{{ stick.NameParts|map("capitalize")|join }}', "{{ snakecase(stick.NameParts|join) }}"),
            ("k{{ capitalize_first(trigger.name) }}", "{{ snakecase(trigger.name) }}"),
            ("k{{ capitalize_first(button.name) }}", "{{ snakecase(button.name) }}"),
            "kLeftBumper",
            "kRightBumper",
            "kTouchpad",
        ),
    )
    s.sub(
        "wpilib jinja pwm motor controllers",
        [
            "wpilibc/src/generate/main/native/cpp/motorcontroller/pwm_motor_controller.cpp.jinja",
        ],
        normal_substitute(
            "kOutputPeriod_",
            "Ms",
        ),
    )

    s.sub(
        "wpilib ADXL345_I2C",
        [
            "wpilibc/src/main/native/cpp/ADXL345_I2C.cpp",
            "wpilibc/src/main/native/include/frc/ADXL345_I2C.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/ADXL345_I2C.java",
        ],
        normal_substitute(
            # Range (C++)
            "kRange_2G",
            "kRange_4G",
            "kRange_8G",
            "kRange_16G",
            # Range (Java)
            ("k2G", "TWO_G"),
            ("k4G", "FOUR_G"),
            ("k8G", "EIGHT_G"),
            ("k16G", "SIXTEEN_G"),
            # Axes (C++)
            "kAxis_X",
            "kAxis_Y",
            "kAxis_Z",
            # Axes (Java)
            "kX",
            "kY",
            "kZ",
            # Class
            "kAddress",
            "kPowerCtlRegister",
            "kDataFormatRegister",
            "kDataRegister",
            "kGsPerLSB",
            # PowerCtlFields
            "kPowerCtl_Link",
            "kPowerCtl_AutoSleep",
            "kPowerCtl_Measure",
            "kPowerCtl_Sleep",
            # DataFormatFields
            "kDataFormat_SelfTest",
            "kDataFormat_SPI",
            "kDataFormat_IntInvert",
            "kDataFormat_FullRes",
            "kDataFormat_Justify",
        ),
    )
    s.sub(
        "wpilib AddressableLED ColorOrder",
        [
            "wpilibc/src/main/native/include/frc/AddressableLED.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/AddressableLED.java",
        ],
        normal_substitute(
            "kRGB",
            "kRBG",
            "kBGR",
            "kBRG",
            "kGBR",
            "kGRB",
        ),
    )
    s.sub(
        "wpilib Alert AlertType",
        [
            "wpilibc/src/main/native/cpp/Alert.cpp",
            "wpilibc/src/main/native/include/frc/Alert.h",
            "wpilibc/src/test/native/cpp/AlertTest.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Alert.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/AlertTest.java",
        ],
        normal_substitute(
            "kError",
            "kWarning",
            "kInfo",
        ),
    )
    s.sub(
        "wpilib CAN",
        [
            "wpilibc/src/main/native/cpp/CAN.cpp",
            "wpilibc/src/main/native/include/frc/CAN.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/CAN.java",
        ],
        normal_substitute(
            "kTeamManufacturer",
            "kTeamDeviceType",
        ),
    )
    s.sub(
        "wpilib DoubleSolenoid Value",
        [
            "wpilibc/src/main/native/cpp/DoubleSolenoid.cpp",
            "wpilibc/src/main/native/cpp/simulation/DoubleSolenoidSim.cpp",
            "wpilibc/src/main/native/include/frc/DoubleSolenoid.h",
            "wpilibc/src/test/native/cpp/DoubleSolenoidTestCTRE.cpp",
            "wpilibc/src/test/native/cpp/DoubleSolenoidTestREV.cpp",
            "wpilibc/src/test/native/cpp/simulation/CTREPCMSimTest.cpp",
            "wpilibc/src/test/native/cpp/simulation/REVPHSimTest.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotInlined/cpp/subsystems/HatchSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/cpp/subsystems/HatchSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/cpp/subsystems/Intake.cpp",
            "wpilibcExamples/src/main/cpp/examples/Solenoid/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/UnitTest/cpp/subsystems/Intake.cpp",
            "wpilibcExamples/src/test/cpp/examples/UnitTest/cpp/subsystems/IntakeTest.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DoubleSolenoid.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/DoubleSolenoidSim.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/DoubleSolenoidTestCTRE.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/DoubleSolenoidTestREV.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/CTREPCMSimTest.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/REVPHSimTest.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/subsystems/HatchSubsystem.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/subsystems/HatchSubsystem.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Intake.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/solenoid/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/unittest/subsystems/Intake.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/unittest/subsystems/IntakeTest.java",
        ],
        normal_substitute(
            "kOff",
            "kForward",
            "kReverse",
        ),
    )
    s.sub(
        "wpilib DriverStation",
        [
            "wpilibc/src/main/native/cpp/DataLogManager.cpp",
            "wpilibc/src/main/native/cpp/DriverStation.cpp",
            "wpilibc/src/main/native/cpp/GenericHID.cpp",
            "wpilibc/src/main/native/include/frc/DriverStation.h",
            "wpilibc/src/test/native/cpp/simulation/DriverStationSimTest.cpp",
            "wpilibcExamples/src/main/cpp/examples/DigitalCommunication/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/I2CCommunication/cpp/Robot.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DriverStation.java",
        ],
        normal_substitute(
            "kRed",
            "kBlue",
            "kNone",
            "kPractice",
            "kQualification",
            "kElimination",
            "kJoystickPorts",
        ),
    )
    s.sub(
        "wpilib GenericHID RumbleType",
        [
            "wpilibc/src/main/native/cpp/GenericHID.cpp",
            "wpilibc/src/main/native/cpp/simulation/GenericHIDSim.cpp",
            "wpilibc/src/main/native/include/frc/GenericHID.h",
            "wpilibc/src/test/native/cpp/GenericHIDTest.cpp",
            "wpilibcExamples/src/main/cpp/examples/HidRumble/cpp/Robot.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/GenericHID.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/GenericHIDSim.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/GenericHIDTest.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hidrumble/Robot.java",
        ],
        normal_substitute(
            "kLeftRumble",
            "kRightRumble",
            "kBothRumble",
        ),
    )
    s.sub(
        "wpilib GenericHID HIDType",
        [
            "wpilibc/src/main/native/include/frc/GenericHID.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/GenericHID.java",
        ],
        normal_substitute(
            "kUnknown",
            ("kXInputUnknown", "XINPUT_UNKNOWN"),
            ("kXInputGamepad", "XINPUT_GAMEPAD"),
            ("kXInputWheel", "XINPUT_WHEEL"),
            ("kXInputArcadeStick", "XINPUT_ARCADE_STICK"),
            ("kXInputFlightStick", "XINPUT_FLIGHT_STICK"),
            ("kXInputDancePad", "XINPUT_DANCE_PAD"),
            ("kXInputGuitar", "XINPUT_GUITAR"),
            ("kXInputGuitar2", "XINPUT_GUITAR2"),
            # NOTE Java uses XINPUT_DRUMKIT
            ("kXInputDrumKit", "XINPUT_DRUM_KIT"),
            ("kXInputGuitar3", "XINPUT_GUITAR3"),
            ("kXInputArcadePad", "XINPUT_ARCADE_PAD"),
            "kHIDJoystick",
            "kHIDGamepad",
            "kHIDDriving",
            "kHIDFlight",
            ("kHID1stPerson", "HID_1ST_PERSON"),
        ),
    )
    s.sub(
        "wpilib I2C",
        [
            "wpilibc/src/main/native/include/frc/I2C.h",
            "wpilibcExamples/src/main/cpp/examples/I2CCommunication/include/Robot.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/I2C.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/ADXL345SimTest.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/i2ccommunication/Robot.java",
        ],
        normal_substitute(
            "kPort0",
            "kPort1",
        ),
    )
    s.sub(
        "wpilib IterativeRobotBase",
        [
            "wpilibc/src/main/native/cpp/IterativeRobotBase.cpp",
            "wpilibc/src/main/native/include/frc/IterativeRobotBase.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/IterativeRobotBase.java",
        ],
        normal_substitute(
            "kNone",
            "kDisabled",
            "kAutonomous",
            "kTeleop",
            "kTest",
        ),
    )
    s.sub(
        "wpilib Joystick",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/button/CommandJoystick.java",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/button/CommandJoystick.cpp",
            "wpilibc/src/main/native/cpp/Joystick.cpp",
            "wpilibc/src/main/native/cpp/simulation/JoystickSim.cpp",
            "wpilibc/src/main/native/include/frc/Joystick.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Joystick.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/JoystickSim.java",
        ],
        normal_substitute(
            "kDefaultXChannel",
            "kDefaultYChannel",
            "kDefaultZChannel",
            "kDefaultTwistChannel",
            "kDefaultThrottleChannel",
            # AxisType (C++)
            "kXAxis",
            "kYAxis",
            "kZAxis",
            "kTwistAxis",
            "kThrottleAxis",
            # ButtonType (C++)
            "kTriggerButton",
            "kTopButton",
            # AxisType (Java) / Axis (C++)
            "kX",
            "kY",
            "kZ",
            "kTwist",
            "kThrottle",
            "kNumAxes",
            # ButtonType (Java) / Button (C++)
            "kTrigger",
            "kTop",
        ),
    )
    s.sub(
        "wpilib LEDPattern",
        [
            "wpilibc/src/main/native/cpp/LEDPattern.cpp",
            "wpilibc/src/main/native/include/frc/LEDPattern.h",
            "wpilibc/src/test/native/cpp/LEDPatternTest.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/LEDPattern.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/LEDPatternTest.java",
        ],
        normal_substitute(
            # GradientType
            "kContinuous",
            "kDiscontinuous",
            # Java only
            "kOff",
        ),
    )
    s.sub(
        "wpilib MotorSafety",
        [
            "wpilibc/src/main/native/include/frc/MotorSafety.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/MotorSafety.java",
        ],
        normal_substitute(
            "kDefaultSafetyExpiration",
        ),
    )
    s.sub(
        "wpilib PWM OutputPeriod",
        [
            "romiVendordep/src/main/native/cpp/romi/RomiMotor.cpp",
            "wpilibc/src/main/native/cpp/PWM.cpp",
            "wpilibc/src/main/native/cpp/Servo.cpp",
            "wpilibc/src/main/native/include/frc/PWM.h",
        ],
        normal_substitute(
            "kOutputPeriod_20Ms",
            "kOutputPeriod_10Ms",
            "kOutputPeriod_5Ms",
        ),
    )
    s.sub(
        "wpilib PowerDistribution",
        [
            "wpilibc/src/main/native/cpp/PowerDistribution.cpp",
            "wpilibc/src/main/native/include/frc/PowerDistribution.h",
            "wpilibc/src/test/native/cpp/simulation/PDPSimTest.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/PowerDistribution.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/PowerDistributionTest.java",
        ],
        normal_substitute(
            "kDefaultModule",
            # ModuleType
            "kCTRE",
            "kRev",
        ),
    )
    s.sub(
        "wpilib RuntimeType",
        [
            "wpilibc/src/main/native/cpp/DataLogManager.cpp",
            "wpilibc/src/main/native/cppcs/RobotBase.cpp",
            "wpilibc/src/main/native/include/frc/RuntimeType.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DataLogManager.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotBase.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/RuntimeType.java",
        ],
        normal_substitute(
            ("kRoboRIO", "ROBORIO"),
            ("kRoboRIO2", "ROBORIO2"),
            "kSimulation",
            # Only Java has RuntimeType.SYSTEM_CORE, apparently
            "kSystemCore",
        ),
    )
    s.sub(
        "wpilib java SensorUtil",
        [
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DigitalOutput.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/SensorUtil.java",
        ],
        normal_substitute(
            "kSystemClockTicksPerMicrosecond",
            "kDigitalChannels",
            "kAnalogInputChannels",
            "kCTRESolenoidChannels",
            "kPwmChannels",
            ("kCTREPDPChannels", "CTRE_PDP_CHANNELS"),
            ("kCTREPDPModules", "CTRE_PDP_MODULES"),
            ("kCTREPCMModules", "CTRE_PCM_MODULES"),
            ("kREVPHChannels", "REV_PH_CHANNELS"),
            ("kREVPHModules", "REV_PH_MODULES"),
        ),
    )
    s.sub(
        "wpilib SerialPort",
        [
            "wpilibc/src/main/native/cpp/SerialPort.cpp",
            "wpilibc/src/main/native/include/frc/SerialPort.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/SerialPort.java",
        ],
        normal_substitute(
            "kFlushOnAccess",
            # Port
            "kOnboard",
            "kMXP",
            "kUSB",
            "kUSB1",
            "kUSB2",
            # Parity (C++)
            "kParity_None",
            "kParity_Odd",
            "kParity_Even",
            "kParity_Mark",
            "kParity_Space",
            # Parity (Java)
            "kNone",
            "kOdd",
            "kEven",
            "kMark",
            "kSpace",
            # StopBits (C++)
            "kStopBits_One",
            "kStopBits_OnePointFive",
            "kStopBits_Two",
            # StopBits (Java)
            "kOne",
            "kOnePointFive",
            "kTwo",
            # FlowControl (C++)
            "kFlowControl_None",
            ("kFlowControl_XonXoff", "FLOW_CONTROL_XONXOFF"),
            ("kFlowControl_RtsCts", "FLOW_CONTROL_RTSCTS"),
            ("kFlowControl_DtrDsr", "FLOW_CONTROL_DTRDSR"),
            # FlowControl (Java)
            "kNone",
            "kXonXoff",
            "kRtsCts",
            "kDtsDsr",
            # WriteBufferMode
            "kFlushOnAccess",
            "kFlushWhenFull",
        ),
    )
    s.sub(
        "wpilib Servo",
        [
            "wpilibc/src/main/native/cpp/Servo.cpp",
            "wpilibc/src/main/native/cpp/simulation/ServoSim.cpp",
            "wpilibc/src/main/native/include/frc/Servo.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Servo.java",
        ],
        normal_substitute(
            "kMinServoAngle",
            "kMaxServoAngle",
            "kDefaultMaxServoPWM",
            "kDefaultMinServoPWM",
        ),
    )
    s.sub(
        "wpilib TimedRobot DEFAULT_PERIOD",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/CommandScheduler.java",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/CommandScheduler.cpp",
            "wpilibc/src/main/native/include/frc/TimedRobot.h",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/include/Constants.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/TimedRobot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/Constants.java",
        ],
        normal_substitute(
            "kDefaultPeriod",
        ),
    )
    s.sub(
        "wpilib Tracer",
        [
            "wpilibc/src/main/native/cpp/Tracer.cpp",
            "wpilibc/src/main/native/include/frc/Tracer.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Tracer.java",
        ],
        normal_substitute(
            "kMinPrintPeriod",
        ),
    )
    s.sub(
        "wpilib Watchdog",
        [
            "wpilibc/src/main/native/cpp/Watchdog.cpp",
            "wpilibc/src/main/native/include/frc/Watchdog.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Watchdog.java",
        ],
        normal_substitute(
            "kMinPrintPeriod",
        ),
    )

    # For usages- The class itself is part of jinja
    s.sub(
        "wpilib XboxController Button",
        [
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/cpp/RobotContainer.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumControllerCommand/cpp/RobotContainer.cpp",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand/RobotContainer.java",
        ],
        normal_substitute(
            "kA",
            "kB",
            "kRightBumper",
        ),
    )

    s.sub(
        "wpilib counter EdgeConfiguration",
        [
            "wpilibc/src/main/native/cpp/counter/Tachometer.cpp",
            "wpilibc/src/main/native/cpp/counter/UpDownCounter.cpp",
            "wpilibc/src/main/native/include/frc/counter/EdgeConfiguration.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/counter/EdgeConfiguration.java",
        ],
        normal_substitute(
            "kRisingEdge",
            "kFallingEdge",
        ),
    )

    s.sub(
        "wpilib drive RobotDriveBase",
        [
            "wpilibc/src/main/native/cpp/drive/MecanumDrive.cpp",
            "wpilibc/src/main/native/include/frc/drive/RobotDriveBase.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/MecanumDrive.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/RobotDriveBase.java",
        ],
        normal_substitute(
            # MotorType
            "kFrontLeft",
            "kFrontRight",
            "kRearLeft",
            "kRearRight",
            "kLeft",
            "kRight",
            "kBack",
            # Class
            "kDefaultDeadband",
            "kDefaultMaxOutput",
        ),
    )

    s.sub(
        "wpilib simulation DifferentialDrivetrainSim State",
        [
            "wpilibc/src/main/native/cpp/simulation/DifferentialDrivetrainSim.cpp",
            "wpilibc/src/main/native/include/frc/simulation/DifferentialDrivetrainSim.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/DifferentialDrivetrainSim.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/DifferentialDrivetrainSimTest.java",
        ],
        normal_substitute(
            # State
            "kX",
            "kY",
            "kHeading",
            "kLeftVelocity",
            "kRightVelocity",
            "kLeftPosition",
            "kRightPosition",
            # KitbotGearing
            ("k12p75", "RATIO_12_75"),
            ("k10p71", "RATIO_10_71"),
            ("k8p45", "RATIO_8_45"),
            ("k7p31", "RATIO_7_31"),
            ("k5p95", "RATIO_5_95"),
            # KitbotMotor (Java)
            "kSingleCIMPerSide",
            "kDualCIMPerSide",
            "kSingleMiniCIMPerSide",
            ("kDualMiniCIMPerSide", "DUAL_MINICIM_PER_SIDE"),
            ("kSingleFalcon500PerSide", "SINGLE_FALCON500_PER_SIDE"),
            ("kDoubleFalcon500PerSide", "DUAL_FALCON500_PER_SIDE"),
            "kSingleNEOPerSide",
            ("kDoubleNEOPerSide", "DUAL_NEO_PER_SIDE"),  # NOTE
            # KitbotWheelSize
            "kSixInch",
            "kEightInch",
            "kTenInch",
        ),
    )

    s.sub(
        "wpilib smartdashboard Mechanism2d",
        [
            "wpilibc/src/main/native/cpp/smartdashboard/Mechanism2d.cpp",
        ],
        normal_substitute(
            "kBackgroundColor",
            "kDims",
        ),
    )
    s.sub(
        "wpilib smartdashboard MechanismLigament2d",
        [
            "wpilibc/src/main/native/cpp/smartdashboard/MechanismLigament2d.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/smartdashboard/MechanismLigament2d.java",
        ],
        normal_substitute(
            "kSmartDashboardType",
        ),
    )
    s.sub(
        "wpilib smartdashboard SendableChooserBase",
        [
            "wpilibc/src/main/native/include/frc/smartdashboard/SendableChooser.h",
            "wpilibc/src/main/native/include/frc/smartdashboard/SendableChooserBase.h",
        ],
        normal_substitute(
            "kDefault",
            "kOptions",
            "kSelected",
            "kActive",
            "kInstance",
        ),
    )

    s.sub(
        "wpilib sysid SysIdRoutineLog State",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutine.java",
            "wpilibNewCommands/src/main/native/cpp/frc2/command/sysid/SysIdRoutine.cpp",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutineTest.java",
            "wpilibNewCommands/src/test/native/cpp/frc2/command/sysid/SysIdRoutineTest.cpp",
            "wpilibc/src/main/native/cpp/sysid/SysIdRoutineLog.cpp",
            "wpilibc/src/main/native/include/frc/sysid/SysIdRoutineLog.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/sysid/SysIdRoutineLog.java",
        ],
        normal_substitute(
            "kQuasistaticForward",
            "kQuasistaticReverse",
            "kDynamicForward",
            "kDynamicReverse",
            "kNone",
        ),
    )

    s.sub(
        "wpilib java util Color RGBChannel",
        [
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/LEDWriter.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/util/Color.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/util/ColorTest.java",
        ],
        normal_substitute(
            "kRed",
            "kGreen",
            "kBlue",
        ),
    )
    s.sub(
        "wpilib util Color",
        [
            "wpilibc/src/main/native/cpp/LEDPattern.cpp",
            "wpilibc/src/main/native/include/frc/LEDPattern.h",
            "wpilibc/src/main/native/include/frc/util/Color.h",
            "wpilibc/src/test/native/cpp/LEDPatternTest.cpp",
            "wpilibcExamples/src/main/cpp/examples/ArmSimulation/include/subsystems/Arm.h",
            "wpilibcExamples/src/main/cpp/examples/Mechanism2d/cpp/Robot.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/LEDPattern.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/util/Color.java",
            "wpilibj/src/test/java/edu/wpi/first/math/util/ColorTest.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/AddressableLEDBufferTest.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/AddressableLEDBufferViewTest.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/LEDPatternTest.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mechanism2d/Robot.java",
        ],
        normal_substitute(
            "kDenim",
            "kFirstBlue",
            "kFirstRed",
            "kAliceBlue",
            "kAntiqueWhite",
            "kAqua",
            "kAquamarine",
            "kAzure",
            "kBeige",
            "kBisque",
            "kBlack",
            "kBlanchedAlmond",
            "kBlue",
            "kBlueViolet",
            "kBrown",
            "kBurlywood",
            "kCadetBlue",
            "kChartreuse",
            "kChocolate",
            "kCoral",
            "kCornflowerBlue",
            "kCornsilk",
            "kCrimson",
            "kCyan",
            "kDarkBlue",
            "kDarkCyan",
            "kDarkGoldenrod",
            "kDarkGray",
            "kDarkGreen",
            "kDarkKhaki",
            "kDarkMagenta",
            "kDarkOliveGreen",
            "kDarkOrange",
            "kDarkOrchid",
            "kDarkRed",
            "kDarkSalmon",
            "kDarkSeaGreen",
            "kDarkSlateBlue",
            "kDarkSlateGray",
            "kDarkTurquoise",
            "kDarkViolet",
            "kDeepPink",
            "kDeepSkyBlue",
            "kDimGray",
            "kDodgerBlue",
            "kFirebrick",
            "kFloralWhite",
            "kForestGreen",
            "kFuchsia",
            "kGainsboro",
            "kGhostWhite",
            "kGold",
            "kGoldenrod",
            "kGray",
            "kGreen",
            "kGreenYellow",
            "kHoneydew",
            "kHotPink",
            "kIndianRed",
            "kIndigo",
            "kIvory",
            "kKhaki",
            "kLavender",
            "kLavenderBlush",
            "kLawnGreen",
            "kLemonChiffon",
            "kLightBlue",
            "kLightCoral",
            "kLightCyan",
            "kLightGoldenrodYellow",
            "kLightGray",
            "kLightGreen",
            "kLightPink",
            "kLightSalmon",
            "kLightSeaGreen",
            "kLightSkyBlue",
            "kLightSlateGray",
            "kLightSteelBlue",
            "kLightYellow",
            "kLime",
            "kLimeGreen",
            "kLinen",
            "kMagenta",
            "kMaroon",
            "kMediumAquamarine",
            "kMediumBlue",
            "kMediumOrchid",
            "kMediumPurple",
            "kMediumSeaGreen",
            "kMediumSlateBlue",
            "kMediumSpringGreen",
            "kMediumTurquoise",
            "kMediumVioletRed",
            "kMidnightBlue",
            "kMintcream",
            "kMistyRose",
            "kMoccasin",
            "kNavajoWhite",
            "kNavy",
            "kOldLace",
            "kOlive",
            "kOliveDrab",
            "kOrange",
            "kOrangeRed",
            "kOrchid",
            "kPaleGoldenrod",
            "kPaleGreen",
            "kPaleTurquoise",
            "kPaleVioletRed",
            "kPapayaWhip",
            "kPeachPuff",
            "kPeru",
            "kPink",
            "kPlum",
            "kPowderBlue",
            "kPurple",
            "kRed",
            "kRosyBrown",
            "kRoyalBlue",
            "kSaddleBrown",
            "kSalmon",
            "kSandyBrown",
            "kSeaGreen",
            "kSeashell",
            "kSienna",
            "kSilver",
            "kSkyBlue",
            "kSlateBlue",
            "kSlateGray",
            "kSnow",
            "kSpringGreen",
            "kSteelBlue",
            "kTan",
            "kTeal",
            "kThistle",
            "kTomato",
            "kTurquoise",
            "kViolet",
            "kWheat",
            "kWhite",
            "kWhiteSmoke",
            "kYellow",
            "kYellowGreen",
        ),
    )

    s.sub(
        "wpilib DataLogManager.cpp DataLogManager.java",
        [
            "wpilibc/src/main/native/cpp/DataLogManager.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DataLogManager.java",
        ],
        normal_substitute(
            "kFreeSpaceThreshold",
            "kFileCountThreshold",
        ),
    )
    s.sub(
        "wpilib DriverStation.cpp DriverStation.java",
        [
            "wpilibc/src/main/native/cpp/DriverStation.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/DriverStation.java",
        ],
        normal_substitute(
            "kSmartDashboardType",
            "kJoystickUnpluggedMessageInterval",
        ),
    )
    s.sub(
        "wpilib Preferences.cpp Preferences.java",
        [
            "wpilibc/src/main/native/cpp/Preferences.cpp",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/Preferences.java",
        ],
        normal_substitute(
            "kTableName",
            "kSmartDashboardType",
        ),
    )
    s.sub(
        "wpilib GenericHIDTest.cpp GenericHIDTest.java",
        [
            "wpilibc/src/test/native/cpp/GenericHIDTest.cpp",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/GenericHIDTest.java",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpilib TimedRobotTest.cpp TimedRobotTest.java",
        [
            "wpilibc/src/test/native/cpp/TimedRobotTest.cpp",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/TimedRobotTest.java",
        ],
        normal_substitute(
            "kPeriod",
        ),
    )
    s.sub(
        "wpilib CallbackStore.java",
        [
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/CallbackStore.java",
        ],
        normal_substitute(
            "kAlreadyCancelled",
            "kNormalCancel",
            "kChannelCancel",
            "kNoIndexCancel",
        ),
    )
    s.sub(
        "wpilib DigitalPWMSimTest.cpp DigitalPWMSimTest.java",
        [
            "wpilibc/src/test/native/cpp/simulation/DigitalPWMSimTest.cpp",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/DigitalPWMSimTest.java",
        ],
        normal_substitute(
            "kTestDutyCycle",
        ),
    )
    s.sub(
        "wpilib DriverStationSimTest.cpp",
        [
            "wpilibc/src/test/native/cpp/simulation/DriverStationSimTest.cpp",
        ],
        normal_substitute(
            "kTestTime",
        ),
    )
    s.sub(
        "wpilib DutyCycleEncoderSimTest.cpp",
        [
            "wpilibc/src/test/native/cpp/simulation/DutyCycleEncoderSimTest.cpp",
        ],
        normal_substitute(
            "kTestValue",
        ),
    )
    s.sub(
        "wpilib ElevatorSim.cpp",
        [
            "wpilibc/src/main/native/cpp/simulation/ElevatorSim.cpp",
        ],
        normal_substitute(
            ("Kv", "V"),
        ),
    )
    s.sub(
        "wpilib EncoderSimTest.cpp",
        [
            "wpilibc/src/test/native/cpp/simulation/EncoderSimTest.cpp",
        ],
        normal_substitute(
            "kDefaultDistancePerPulse",
        ),
    )
    s.sub(
        "wpilib PDPSimTest.cpp",
        [
            "wpilibc/src/test/native/cpp/simulation/PDPSimTest.cpp",
        ],
        normal_substitute(
            "kTestCurrent",
        ),
    )
    s.sub(
        "wpilib RoboRioSimTest.cpp RoboRioSimTest.java",
        [
            "wpilibc/src/test/native/cpp/simulation/RoboRioSimTest.cpp",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/RoboRioSimTest.java",
        ],
        normal_substitute(
            "kTestVoltage",
            "kTestCurrent",
            "kTestFaults",
            "kCPUTemp",
            "kTeamNumber",
            "kSerialNum", # C++
            "kSerialNumber", # Java
            "kSerialNumberOverflow",
            "kSerialNumberTruncated",
            "kComments",
            "kCommentsOverflow",
            "kCommentsTruncated",
        ),
    )
    s.sub(
        "wpilib ColorTest.java",
        [
            "wpilibj/src/test/java/edu/wpi/first/math/util/ColorTest.java",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpilib PreferencesTest.java",
        [
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/PreferencesTest.java",
        ],
        normal_substitute(
            "kFilename",
        ),
    )

    # wpilibcExamples

    s.sub(
        "wpilibcExamples AddressableLED",
        [
            "wpilibcExamples/src/main/cpp/examples/AddressableLED/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/AddressableLED/include/Robot.h",
        ],
        normal_substitute(
            "kLength",
            "kLedSpacing",
        ),
    )
    s.sub(
        "wpilibcExamples ArmSimulation",
        [
            "wpilibcExamples/src/main/cpp/examples/ArmSimulation/cpp/subsystems/Arm.cpp",
            "wpilibcExamples/src/main/cpp/examples/ArmSimulation/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/ArmSimulation/include/Robot.h",
            "wpilibcExamples/src/main/cpp/examples/ArmSimulation/include/subsystems/Arm.h",
            "wpilibcExamples/src/test/cpp/examples/ArmSimulation/cpp/ArmSimulationTest.cpp",
        ],
        normal_substitute(
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kArmPositionKey",
            "kArmPKey",
            ("kDefaultArmKp", "DEFAULT_ARM_P"),  # NOTE
            "kDefaultArmSetpoint",
            "kMinAngle",
            "kMaxAngle",
            "kArmReduction",
            "kArmMass",
            "kArmLength",
            "kArmEncoderDistPerPulse",
            ("P", "kP"),  # NOTE
        ),
    )

    s.sub(
        "wpilibcExamples DifferentialDriveBot",
        [
            "wpilibcExamples/src/main/cpp/examples/DifferentialDriveBot/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/DifferentialDriveBot/include/Drivetrain.h",
        ],
        normal_substitute(
            "kMaxSpeed",
            "kMaxAngularSpeed",
            ("kTrackwidth", "TRACK_WIDTH"),  # NOTE
            "kEncoderResolution",
            "kWheelRadius",
        ),
    )
    s.sub(
        "wpilibcExamples DifferentialDrivePoseEstimator",
        [
            "wpilibcExamples/src/main/cpp/examples/DifferentialDrivePoseEstimator/cpp/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/DifferentialDrivePoseEstimator/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/DifferentialDrivePoseEstimator/include/Drivetrain.h",
        ],
        normal_substitute(
            "kMaxSpeed",
            "kMaxAngularSpeed",
            ("kTrackwidth", "TRACK_WIDTH"),  # NOTE
            "kWheelRadius",
            "kEncoderResolution",
            "kDefaultVal",
        ),
    )
    s.sub(
        "wpilibcExamples DigitalCommunication",
        [
            "wpilibcExamples/src/main/cpp/examples/DigitalCommunication/include/Robot.h",
            "wpilibcExamples/src/test/cpp/examples/DigitalCommunication/cpp/DigitalCommunicationTest.cpp",
        ],
        normal_substitute(
            "kAlliancePort",
            "kEnabledPort",
            "kAutonomousPort",
            "kAlertPort",
        ),
    )
    s.sub(
        "wpilibcExamples DriveDistanceOffboard",
        [
            "wpilibcExamples/src/main/cpp/examples/DriveDistanceOffboard/cpp/subsystems/DriveSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/DriveDistanceOffboard/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/DriveDistanceOffboard/include/ExampleSmartMotorController.h",
            "wpilibcExamples/src/main/cpp/examples/DriveDistanceOffboard/include/RobotContainer.h",
            "wpilibcExamples/src/main/cpp/examples/DriveDistanceOffboard/include/subsystems/DriveSubsystem.h",
        ],
        normal_substitute(
            "kDt",
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kMaxSpeed",
            "kMaxAcceleration",
            "kDriverControllerPort",
            "kPosition",
            "kVelocity",
            "kMovementWitchcraft",
        ),
    )

    s.sub(
        "wpilibcExamples ElevatorExponentialProfile",
        [
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialProfile/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialProfile/include/ExampleSmartMotorController.h",
        ],
        normal_substitute(
            "kDt",
            "kPosition",
            "kVelocity",
            "kMovementWitchcraft",
        ),
    )
    s.sub(
        "wpilibcExamples ElevatorExponentialSimulation",
        [
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialSimulation/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialSimulation/cpp/subsystems/Elevator.cpp",
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialSimulation/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialSimulation/include/Robot.h",
            "wpilibcExamples/src/main/cpp/examples/ElevatorExponentialSimulation/include/subsystems/Elevator.h",
        ],
        normal_substitute(
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            ("kElevatorKp", "ELEVATOR_P"),
            ("kElevatorKi", "ELEVATOR_I"),
            ("kElevatorKd", "ELEVATOR_D"),
            "kElevatorMaxV",
            ("kElevatorkS", "ELEVATOR_S"),
            ("kElevatorkG", "ELEVATOR_G"),
            ("kElevatorkV", "ELEVATOR_V"),
            ("kElevatorkA", "ELEVATOR_A"),
            "kElevatorGearing",
            "kElevatorDrumRadius",
            "kCarriageMass",
            "kSetpoint",
            "kLowerSetpoint",
            "kMinElevatorHeight",
            "kMaxElevatorHeight",
            "kArmEncoderDistPerPulse",
        ),
    )
    s.sub(
        "wpilibcExamples ElevatorProfiledPID",
        [
            "wpilibcExamples/src/main/cpp/examples/ElevatorProfiledPID/cpp/Robot.cpp",
        ],
        normal_substitute(
            "kDt",
            "kMaxVelocity",
            "kMaxAcceleration",
            "kS",  # NOTE
        ),
    )
    s.sub(
        "wpilibcExamples ElevatorSimulation",
        [
            "wpilibcExamples/src/main/cpp/examples/ElevatorSimulation/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/ElevatorSimulation/cpp/subsystems/Elevator.cpp",
            "wpilibcExamples/src/main/cpp/examples/ElevatorSimulation/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/ElevatorSimulation/include/Robot.h",
            "wpilibcExamples/src/main/cpp/examples/ElevatorSimulation/include/subsystems/Elevator.h",
            "wpilibcExamples/src/test/cpp/examples/ElevatorSimulation/cpp/ElevatorSimulationTest.cpp",
        ],
        normal_substitute(
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            ("kElevatorKp", "ELEVATOR_P"),
            ("kElevatorKi", "ELEVATOR_I"),
            ("kElevatorKd", "ELEVATOR_D"),
            ("kElevatorkS", "ELEVATOR_S"),
            ("kElevatorkG", "ELEVATOR_G"),
            ("kElevatorkV", "ELEVATOR_V"),
            ("kElevatorkA", "ELEVATOR_A"),
            "kElevatorGearing",
            "kElevatorDrumRadius",
            "kCarriageMass",
            "kSetpoint",
            "kMinElevatorHeight",
            "kMaxElevatorHeight",
            "kArmEncoderDistPerPulse",
        ),
    )
    s.sub(
        "wpilibcExamples ElevatorTrapezoidProfile",
        [
            "wpilibcExamples/src/main/cpp/examples/ElevatorTrapezoidProfile/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/ElevatorTrapezoidProfile/include/ExampleSmartMotorController.h",
        ],
        normal_substitute(
            "kDt",
            "kPosition",
            "kVelocity",
            "kMovementWitchcraft",
        ),
    )

    s.sub(
        "wpilibcExamples FlywheelBangBangController",
        [
            "wpilibcExamples/src/main/cpp/examples/FlywheelBangBangController/cpp/Robot.cpp",
        ],
        normal_substitute(
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kMaxSetpointValue",
            ("kFlywheelKs", "FLYWHEEL_S"),
            ("kFlywheelKv", "FLYWHEEL_V"),
            ("kFlywheelKa", "FLYWHEEL_A"),
            "kFlywheelGearing",
            "kFlywheelMomentOfInertia",
        ),
    )

    s.sub(
        "wpilibcExamples Gyro",
        [
            "wpilibcExamples/src/main/cpp/examples/Gyro/cpp/Robot.cpp",
        ],
        normal_substitute(
            "kAngleSetpoint",
            "kVoltsPerDegreePerSecond",
            "kLeftMotorPort",
            "kRightMotorPort",
            "kGyroPort",
            "kJoystickPort",
        ),
    )
    s.sub(
        "wpilibcExamples GyroMecanum",
        [
            "wpilibcExamples/src/main/cpp/examples/GyroMecanum/cpp/Robot.cpp",
        ],
        normal_substitute(
            "kVoltsPerDegreePerSecond",
            "kFrontLeftMotorPort",
            ("kRearLeftMotorPort", "READ_LEFT_MOTOR_PORT"),  # NOTE Mistake?
            "kFrontRightMotorPort",
            "kRearRightMotorPort",
            "kGyroPort",
            "kJoystickPort",
        ),
    )

    s.sub(
        "wpilibcExamples HatchbotInlined",
        [
            "wpilibcExamples/src/main/cpp/examples/HatchbotInlined/cpp/commands/Autos.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotInlined/cpp/subsystems/DriveSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotInlined/cpp/subsystems/HatchSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotInlined/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/HatchbotInlined/include/RobotContainer.h",
        ],
        normal_substitute(
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameterInches",
            "kEncoderDistancePerPulse",
            "kHatchSolenoidModule",
            "kHatchSolenoidPorts",
            "kAutoDriveDistanceInches",
            "kAutoBackupDistanceInches",
            "kAutoDriveSpeed",
            "kDriverControllerPort",
        ),
    )
    s.sub(
        "wpilibcExamples HatchbotTraditional",
        [
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/cpp/commands/ComplexAuto.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/cpp/subsystems/DriveSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/cpp/subsystems/HatchSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/HatchbotTraditional/include/RobotContainer.h",
        ],
        normal_substitute(
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameterInches",
            "kEncoderDistancePerPulse",
            "kHatchSolenoidModule",
            "kHatchSolenoidPorts",
            "kAutoDriveDistanceInches",
            "kAutoBackupDistanceInches",
            "kAutoDriveSpeed",
            "kDriverControllerPort",
        ),
    )

    s.sub(
        "wpilibcExamples I2CCommunication",
        [
            "wpilibcExamples/src/main/cpp/examples/I2CCommunication/include/Robot.h",
            "wpilibcExamples/src/test/cpp/examples/I2CCommunication/cpp/I2CCommunicationTest.cpp",
        ],
        normal_substitute(
            "kPort",
        ),
    )

    s.sub(
        "wpilibcExamples MecanumBot",
        [
            "wpilibcExamples/src/main/cpp/examples/MecanumBot/cpp/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumBot/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumBot/include/Drivetrain.h",
        ],
        normal_substitute(
            "kMaxSpeed",
            "kMaxAngularSpeed",
        ),
    )
    s.sub(
        "wpilibcExamples MecanumControllerCommand",
        [
            "wpilibcExamples/src/main/cpp/examples/MecanumControllerCommand/cpp/Constants.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumControllerCommand/cpp/RobotContainer.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumControllerCommand/cpp/subsystems/DriveSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumControllerCommand/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/MecanumControllerCommand/include/RobotContainer.h",
        ],
        normal_substitute(
            "kFrontLeftMotorPort",
            "kRearLeftMotorPort",
            "kFrontRightMotorPort",
            "kRearRightMotorPort",
            "kFrontLeftEncoderPorts",
            "kRearLeftEncoderPorts",
            "kFrontRightEncoderPorts",
            "kRearRightEncoderPorts",
            "kFrontLeftEncoderReversed",
            "kRearLeftEncoderReversed",
            "kFrontRightEncoderReversed",
            "kRearRightEncoderReversed",
            # NOTE Split words
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelBase",
            "kDriveKinematics",
            "kEncoderCPR",
            "kWheelDiameter",
            "kEncoderDistancePerPulse",
            # NOTE P instead of kP
            ("kPFrontLeftVel", "FRONT_LEFT_VEL_P"),
            ("kPRearLeftVel", "REAR_LEFT_VEL_P"),
            ("kPFrontRightVel", "FRONT_RIGHT_VEL_P"),
            ("kPRearRightVel", "REAR_RIGHT_VEL_P"),
            "kMaxSpeed",
            "kMaxAcceleration",
            "kMaxAngularSpeed",
            "kMaxAngularAcceleration",
            # NOTE P instead of kP
            ("kPXController", "X_CONTROLLER_P"),
            ("kPYController", "Y_CONTROLLER_P"),
            ("kPThetaController", "THETA_CONTROLLER_P"),
            "kThetaControllerConstraints",
            "kDriverControllerPort",
        ),
    )
    s.sub(
        "wpilibcExamples MecanumDrive",
        [
            "wpilibcExamples/src/main/cpp/examples/MecanumDrive/cpp/Robot.cpp",
        ],
        normal_substitute(
            "kFrontLeftChannel",
            "kRearLeftChannel",
            "kFrontRightChannel",
            "kRearRightChannel",
            "kJoystickChannel",
        ),
    )
    s.sub(
        "wpilibcExamples MecanumDrivePoseEstimator",
        [
            "wpilibcExamples/src/main/cpp/examples/MecanumDrivePoseEstimator/cpp/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumDrivePoseEstimator/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/MecanumDrivePoseEstimator/include/Drivetrain.h",
        ],
        normal_substitute(
            "kMaxSpeed",
            "kMaxAngularSpeed",
        ),
    )
    s.sub(
        "wpilibcExamples Mechanism2d",
        [
            "wpilibcExamples/src/main/cpp/examples/Mechanism2d/cpp/Robot.cpp",
        ],
        normal_substitute(
            "kMetersPerPulse",
            "kElevatorMinimumLength",
        ),
    )

    s.sub(
        "wpilibcExamples PotentiometerPID",
        [
            "wpilibcExamples/src/main/cpp/examples/PotentiometerPID/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/PotentiometerPID/include/Robot.h",
            "wpilibcExamples/src/test/cpp/examples/PotentiometerPID/cpp/PotentiometerPIDTest.cpp",
        ],
        normal_substitute(
            "kPotChannel",
            "kMotorChannel",
            "kJoystickChannel",
            "kFullHeight",
            "kSetpoints",
            # PotentiometerPIDTest
            "kElevatorGearing",
            "kElevatorDrumRadius",
            "kCarriageMass",
        ),
    )

    s.sub(
        "wpilibcExamples RapidReactCommandBot",
        [
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/cpp/RapidReactCommandBot.cpp",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/cpp/subsystems/Drive.cpp",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/cpp/subsystems/Shooter.cpp",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/RapidReactCommandBot.h",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/subsystems/Drive.h",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/subsystems/Intake.h",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/subsystems/Pneumatics.h",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/subsystems/Shooter.h",
            "wpilibcExamples/src/main/cpp/examples/RapidReactCommandBot/include/subsystems/Storage.h",
        ],
        normal_substitute(
            # DriveConstants
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameter",
            "kEncoderDistancePerPulse",
            "kTurnP",
            "kTurnI",
            "kTurnD",
            "kTurnTolerance",
            "kTurnRateTolerance",
            "kMaxTurnRate",
            "kMaxTurnAcceleration",
            # IntakeConstants
            "kMotorPort",
            "kSolenoidPorts",
            # StorageConstants
            "kMotorPort",
            "kBallSensorPort",
            # ShooterConstants
            "kEncoderPorts",
            "kEncoderReversed",
            "kEncoderCPR",
            "kEncoderDistancePerPulse",
            "kShooterMotorPort",
            "kFeederMotorPort",
            "kShooterFree",
            "kShooterTarget",
            "kShooterTolerance",
            "kS",
            # "kV",  # NOTE This isn't changed
            "kFeederSpeed",
            # OIConstants
            "kDriverControllerPort",
            # AutoConstants
            "kTimeout",
            "kDriveDistance",
            "kDriveSpeed",
            # Pneumatics
            "kScale",
            "kOffset",
        ),
    )
    s.sub(
        "wpilibcExamples RomiReference",
        [
            "wpilibcExamples/src/main/cpp/examples/RomiReference/cpp/subsystems/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/RomiReference/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/RomiReference/include/subsystems/Drivetrain.h",
        ],
        normal_substitute(
            # DriveConstants
            "kCountsPerRevolution",
            "kWheelDiameterInch",
            # Drivetrain
            "kCountsPerRevolution",
            "kWheelDiameter",
        ),
    )

    s.sub(
        "wpilibcExamples SelectCommand",
        [
            "wpilibcExamples/src/main/cpp/examples/SelectCommand/include/Constants.h",
        ],
        normal_substitute(
            "kDriverControllerPort",
        ),
    )
    s.sub(
        "wpilibcExamples SimpleDifferentialDriveSimulation",
        [
            "wpilibcExamples/src/main/cpp/examples/SimpleDifferentialDriveSimulation/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/SimpleDifferentialDriveSimulation/include/Drivetrain.h",
            "wpilibcExamples/src/main/cpp/examples/Solenoid/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/Solenoid/include/Robot.h",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelRadius",
            "kEncoderResolution",
            # Robot
            "kSolenoidButton",
            "kDoubleSolenoidForward",
            "kDoubleSolenoidReverse",
            "kCompressorButton",
        ),
    )
    s.sub(
        "wpilibcExamples StateSpaceArm",
        [
            "wpilibcExamples/src/main/cpp/examples/StateSpaceArm/cpp/Robot.cpp",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kRaisedPosition",
            "kLoweredPosition",
            "kArmMOI",
            "kArmGearing",
        ),
    )
    s.sub(
        "wpilibcExamples StateSpaceElevator",
        [
            "wpilibcExamples/src/main/cpp/examples/StateSpaceElevator/cpp/Robot.cpp",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kRaisedPosition",
            "kLoweredPosition",
            "kDrumRadius",
            "kCarriageMass",
            "kGearRatio",
        ),
    )
    s.sub(
        "wpilibcExamples StateSpaceFlywheel",
        [
            "wpilibcExamples/src/main/cpp/examples/StateSpaceFlywheel/cpp/Robot.cpp",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kSpinup",
            "kFlywheelMomentOfInertia",
            "kFlywheelGearing",
        ),
    )
    s.sub(
        "wpilibcExamples StateSpaceFlywheelSysId",
        [
            "wpilibcExamples/src/main/cpp/examples/StateSpaceFlywheelSysId/cpp/Robot.cpp",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kSpinup",
            ("kFlywheelKv", "FLYWHEEL_V"),
            ("kFlywheelKa", "FLYWHEEL_A"),
            ("Kv", "V"), # NOTE This and Ka
            ("Ka", "A"),
        ),
    )
    s.sub(
        "wpilibcExamples SwerveBot",
        [
            "wpilibcExamples/src/main/cpp/examples/SwerveBot/cpp/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveBot/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveBot/cpp/SwerveModule.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveBot/include/Drivetrain.h",
            "wpilibcExamples/src/main/cpp/examples/SwerveBot/include/SwerveModule.h",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            # SwerveModule
            "kWheelRadius",
            "kEncoderResolution",
            ("kModuleMaxAngularVelocity", "MODEULE_MAX_ANGULAR_VELOCITY"), # NOTE Typos
            ("kModuleMaxAngularAcceleration", "MODEULE_MAX_ANGULAR_ACCELERATION"),
        ),
    )
    s.sub(
        "wpilibcExamples SwerveControllerCommand",
        [
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/Constants.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/RobotContainer.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/DriveSubsystem.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/SwerveModule.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/include/RobotContainer.h",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/include/subsystems/DriveSubsystem.h",
            "wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/include/subsystems/SwerveModule.h",
        ],
        normal_substitute(
            # DriveConstants
            "kFrontLeftDriveMotorPort",
            "kRearLeftDriveMotorPort",
            "kFrontRightDriveMotorPort",
            "kRearRightDriveMotorPort",
            "kFrontLeftTurningMotorPort",
            "kRearLeftTurningMotorPort",
            "kFrontRightTurningMotorPort",
            "kRearRightTurningMotorPort",
            "kFrontLeftTurningEncoderPorts",
            "kRearLeftTurningEncoderPorts",
            "kFrontRightTurningEncoderPorts",
            "kRearRightTurningEncoderPorts",
            "kFrontLeftTurningEncoderReversed",
            "kRearLeftTurningEncoderReversed",
            "kFrontRightTurningEncoderReversed",
            "kRearRightTurningEncoderReversed",
            "kFrontLeftDriveEncoderPorts",
            "kRearLeftDriveEncoderPorts",
            "kFrontRightDriveEncoderPorts",
            "kRearRightDriveEncoderPorts",
            "kFrontLeftDriveEncoderReversed",
            "kRearLeftDriveEncoderReversed",
            "kFrontRightDriveEncoderReversed",
            "kRearRightDriveEncoderReversed",
            "kDrivePeriod",
            ("kPFrontLeftVel", "FRONT_LEFT_VEL_P"),
            ("kPRearLeftVel", "REAR_LEFT_VEL_P"),
            ("kPFrontRightVel", "FRONT_RIGHT_VEL_P"),
            ("kPRearRightVel", "REAR_RIGHT_VEL_P"),
            # ModuleConstants
            "kEncoderCPR",
            "kWheelDiameter",
            "kDriveEncoderDistancePerPulse",
            "kTurningEncoderDistancePerPulse",
            ("kPModuleTurningController", "MODULE_TURNING_CONTROLLER_P"),
            ("kPModuleDriveController", "MODULE_DRIVE_CONTROLLER_P"),
            # AutoConstants
            "kMaxSpeed",
            "kMaxAcceleration",
            "kMaxAngularSpeed",
            "kMaxAngularAcceleration",
            ("kPXController", "X_CONTROLLER_P"),
            ("kPYController", "Y_CONTROLLER_P"),
            ("kPThetaController", "THETA_CONTROLLER_P"),
            "kThetaControllerConstraints",
            # OIConstants
            "kDriverControllerPort",
            # Drivetrain
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelBase",
            "kDriveKinematics",
            # SwerveModule
            "kModuleMaxAngularVelocity",
            "kModuleMaxAngularAcceleration",
        ),
    )
    s.sub(
        "wpilibcExamples SwerveDrivePoseEstimator",
        [
            "wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/cpp/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/cpp/SwerveModule.cpp",
            "wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/include/Drivetrain.h",
            "wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/include/SwerveModule.h",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            # SwerveModule
            "kWheelRadius",
            "kEncoderResolution",
            "kModuleMaxAngularVelocity",
            "kModuleMaxAngularAcceleration",
        ),
    )
    s.sub(
        "wpilibcExamples SysIdRoutine",
        [
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/cpp/subsystems/Drive.cpp",
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/cpp/subsystems/Shooter.cpp",
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/include/SysIdRoutineBot.h",
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/include/subsystems/Drive.h",
            "wpilibcExamples/src/main/cpp/examples/SysIdRoutine/include/subsystems/Shooter.h",
        ],
        normal_substitute(
            # constants::drive
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCpr",
            "kWheelDiameter",
            "kEncoderDistancePerPulse",
            # constants::shooter
            "kEncoderPorts",
            "kEncoderReversed",
            "kEncoderCpr",
            "kEncoderDistancePerPulse",
            "kShooterMotorPort",
            "kFeederMotorPort",
            "kShooterFreeSpeed",
            "kShooterTargetSpeed",
            "kShooterTolerance",
            "kS", # NOTE
            "kFeederSpeed",
            # constants::intake
            "kMotorPort",
            "kSolenoidPorts",
            # constants::storage
            "kMotorPort",
            "kBallSensorPort",
            # constants::autonomous
            "kTimeout",
            "kDriveDistance",
            "kDriveSpeed",
            # constants::oi
            "kDriverControllerPort",
        ),
    )

    s.sub(
        "wpilibcExamples UnitTest",
        [
            "wpilibcExamples/src/main/cpp/examples/UnitTest/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/examples/UnitTest/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/UnitTest/include/Robot.h",
            "wpilibcExamples/src/main/cpp/examples/UnitTest/include/subsystems/Intake.h",
            "wpilibcExamples/src/test/cpp/examples/UnitTest/cpp/subsystems/IntakeTest.cpp",
        ],
        normal_substitute(
            # IntakeConstants
            "kMotorPort",
            "kPistonFwdChannel",
            "kPistonRevChannel",
            "kIntakeSpeed",
            # OperatorConstants
            "kJoystickIndex",
        ),
    )

    s.sub(
        "wpilibcExamples XRPReference",
        [
            "wpilibcExamples/src/main/cpp/examples/XRPReference/cpp/subsystems/Drivetrain.cpp",
            "wpilibcExamples/src/main/cpp/examples/XRPReference/include/Constants.h",
            "wpilibcExamples/src/main/cpp/examples/XRPReference/include/subsystems/Drivetrain.h",
        ],
        normal_substitute(
            # DriveConstants
            "kCountsPerRevolution",
            "kWheelDiameterInch",
            # Drivetrain
            "kGearRatio",
            "kCountsPerMotorShaftRev",
            "kCountsPerRevolution",
            "kWheelDiameter",
        ),
    )

    s.sub(
        "wpilibcExamples commandbased",
        [
            "wpilibcExamples/src/main/cpp/templates/commandbased/include/Constants.h",
            "wpilibcExamples/src/main/cpp/templates/commandbased/include/RobotContainer.h",
        ],
        normal_substitute(
            # OperatorConstants
            "kDriverControllerPort",
        ),
    )
    s.sub(
        "wpilibcExamples timed",
        [
            "wpilibcExamples/src/main/cpp/templates/timed/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/templates/timed/include/Robot.h",
        ],
        normal_substitute(
            # Robot
            "kAutoNameDefault",
            "kAutoNameCustom",
        ),
    )
    s.sub(
        "wpilibcExamples timeslice",
        [
            "wpilibcExamples/src/main/cpp/templates/timeslice/cpp/Robot.cpp",
            "wpilibcExamples/src/main/cpp/templates/timeslice/include/Robot.h",
        ],
        normal_substitute(
            # Robot
            "kAutoNameDefault",
            "kAutoNameCustom",
        ),
    )

    # wpilibjExamples

    s.sub(
        "wpilibjExamples addressableled",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/addressableled/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kLedSpacing",
        ),
    )
    s.sub(
        "wpilibjExamples armsimulation",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/armsimulation/ArmSimulationTest.java",
        ],
        normal_substitute(
            # Constants
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            ("kArmPositionKey", "ARMPOSITION_KEY"), # NOTE typo?
            ("kArmPKey", "ARMP_KEY"), # NOTE typo?
            ("kDefaultArmKp", "DEFAULT_ARM_P"),
            "kDefaultArmSetpointDegrees",
            "kArmEncoderDistPerPulse",
            "kArmReduction",
            "kArmMass",
            "kArmLength",
            "kMinAngleRads",
            "kMaxAngleRads",
        ),
    )

    s.sub(
        "wpilibjExamples differentialdrivebot",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdrivebot/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdrivebot/Robot.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelRadius",
            "kEncoderResolution",
        ),
    )
    s.sub(
        "wpilibjExamples differentialdriveposeestimator",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdriveposeestimator/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdriveposeestimator/Robot.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelRadius",
            "kEncoderResolution",
        ),
    )
    s.sub(
        "wpilibjExamples digitalcommunication",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/digitalcommunication/Robot.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/digitalcommunication/DigitalCommunicationTest.java",
        ],
        normal_substitute(
            # Robot
            "kAlliancePort",
            "kEnabledPort",
            "kAutonomousPort",
            "kAlertPort",
        ),
    )
    s.sub(
        "wpilibjExamples drivedistanceoffboard",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/drivedistanceoffboard/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/drivedistanceoffboard/ExampleSmartMotorController.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/drivedistanceoffboard/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/drivedistanceoffboard/subsystems/DriveSubsystem.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            "kDt",
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "ks", # NOTE ks -> S, etc.
            "kv",
            "ka",
            "kMaxSpeed",
            "kMaxAcceleration",
            # Constants.OIConstants
            "kDriverControllerPort",
            # ExampleSmartModeController.PIDMode
            "kPosition",
            "kVelocity",
            "kMovementWitchcraft",
        ),
    )

    s.sub(
        "wpilibjExamples elevatorexponentialprofile",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorexponentialprofile/ExampleSmartMotorController.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorexponentialprofile/Robot.java",
        ],
        normal_substitute(
            # ExampleSmartMotorController.PIDMode
            "kPosition",
            "kVelocity",
            "kMovementWitchcraft",
            # Robot
            "kDt",
        ),
    )
    s.sub(
        "wpilibjExamples elevatorexponentialsimulation",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorexponentialsimulation/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorexponentialsimulation/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorexponentialsimulation/subsystems/Elevator.java",
        ],
        normal_substitute(
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            ("kElevatorKp", "ELEVATOR_P"), # NOTE Kp->P
            ("kElevatorKi", "ELEVATOR_I"),
            ("kElevatorKd", "ELEVATOR_D"),
            "kElevatorMaxV",
            ("kElevatorkS", "ELEVATOR_S"),
            ("kElevatorkG", "ELEVATOR_G"),
            ("kElevatorkV", "ELEVATOR_V"),
            ("kElevatorkA", "ELEVATOR_A"),
            "kElevatorGearing",
            "kElevatorDrumRadius",
            "kCarriageMass",
            "kSetpoint",
            ("kLowerkSetpoint", "LOWER_SETPOINT"), # Fix typo
            "kMinElevatorHeight",
            "kMaxElevatorHeight",
            "kElevatorEncoderDistPerPulse",
        ),
    )
    s.sub(
        "wpilibjExamples elevatorprofiledpid",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorprofiledpid/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kDt",
            "kMaxVelocity",
            "kMaxAcceleration",
            "kP", # NOTE kP -> P
            "kI",
            "kD",
            "kS",
            "kG",
            "kV",
        ),
    )
    s.sub(
        "wpilibjExamples elevatorsimulation",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/ElevatorSimulationTest.java",
        ],
        normal_substitute(
            # Constants
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            ("kElevatorKp", "ELEVATOR_P"),
            ("kElevatorKi", "ELEVATOR_I"),
            ("kElevatorKd", "ELEVATOR_D"),
            ("kElevatorkS", "ELEVATOR_S"),
            ("kElevatorkG", "ELEVATOR_G"),
            ("kElevatorkV", "ELEVATOR_V"),
            ("kElevatorkA", "ELEVATOR_A"),
            "kElevatorGearing",
            "kElevatorDrumRadius",
            "kCarriageMass",
            "kSetpoint",
            "kMinElevatorHeight",
            "kMaxElevatorHeight",
            "kElevatorEncoderDistPerPulse",
        ),
    )
    s.sub(
        "wpilibjExamples elevatortrapezoidprofile",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatortrapezoidprofile/ExampleSmartMotorController.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatortrapezoidprofile/Robot.java",
        ],
        normal_substitute(
            # ExampleSmartMotorController.PIDMode
            "kPosition",
            "kVelocity",
            "kMovementWitchcraft",
            # Robot
            "kDt",
        ),
    )

    s.sub(
        "wpilibjExamples flywheelbangbangcontroller",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/flywheelbangbangcontroller/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kMaxSetpointValue",
            ("kFlywheelKs", "FLYWHEEL_S"),
            ("kFlywheelKv", "FLYWHEEL_V"),
            ("kFlywheelKa", "FLYWHEEL_A"),
            "kFlywheelGearing",
            "kFlywheelMomentOfInertia",
        ),
    )

    s.sub(
        "wpilibjExamples gyro",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/gyro/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kAngleSetpoint",
            "kP", # NOTE
            "kVoltsPerDegreePerSecond",
            "kLeftMotorPort",
            "kRightMotorPort",
            "kGyroPort",
            "kJoystickPort",
        ),
    )
    s.sub(
        "wpilibjExamples gyromecanum",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/gyromecanum/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kVoltsPerDegreePerSecond",
            "kFrontLeftChannel",
            "kRearLeftChannel",
            "kFrontRightChannel",
            "kRearRightChannel",
            "kGyroPort",
            "kJoystickPort",
        ),
    )

    s.sub(
        "wpilibjExamples hatchbotinlined",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/commands/Autos.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/subsystems/DriveSubsystem.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/subsystems/HatchSubsystem.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameterInches",
            "kEncoderDistancePerPulse",
            # Constants.HatchConstants
            "kHatchSolenoidModule",
            "kHatchSolenoidPorts",
            # Constants.AutoConstants
            "kAutoDriveDistanceInches",
            "kAutoBackupDistanceInches",
            "kAutoDriveSpeed",
            # Constants.OIConstants
            "kDriverControllerPort",
        ),
    )
    s.sub(
        "wpilibjExamples hatchbottraditional",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/commands/ComplexAuto.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/subsystems/DriveSubsystem.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/subsystems/HatchSubsystem.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameterInches",
            "kEncoderDistancePerPulse",
            # Constants.HatchConstants
            "kHatchSolenoidModule",
            "kHatchSolenoidPorts",
            # Constants.AutoConstants
            ("kAutoDriveDistanceInches", "AUTO_DRIVE_DISTACE_INCHES"),
            "kAutoBackupDistanceInches",
            "kAutoDriveSpeed",
            # Constants.OICOnstants
            "kDriverControllerPort",
        ),
    )

    s.sub(
        "wpilibjExamples i2ccommunication",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/i2ccommunication/Robot.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/i2ccommunication/I2CCommunicationTest.java",
        ],
        normal_substitute(
            # Robot
            "kPort",
            "kDeviceAddress",
        ),
    )

    s.sub(
        "wpilibjExamples mecanumbot",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumbot/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumbot/Robot.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
        ),
    )
    s.sub(
        "wpilibjExamples mecanumcontrollercommand",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand/subsystems/DriveSubsystem.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            "kFrontLeftMotorPort",
            "kRearLeftMotorPort",
            "kFrontRightMotorPort",
            "kRearRightMotorPort",
            "kFrontLeftEncoderPorts",
            "kRearLeftEncoderPorts",
            "kFrontRightEncoderPorts",
            "kRearRightEncoderPorts",
            "kFrontLeftEncoderReversed",
            "kRearLeftEncoderReversed",
            "kFrontRightEncoderReversed",
            "kRearRightEncoderReversed",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelBase",
            "kDriveKinematics",
            "kEncoderCPR",
            "kWheelDiameter",
            "kEncoderDistancePerPulse",
            ("kFeedforward", "FEED_FORWARD"),
            ("kPFrontLeftVel", "FRONT_LEFT_VEL_P"),
            ("kPRearLeftVel", "REAR_LEFT_VEL_P"),
            ("kPFrontRightVel", "FRONT_RIGHT_VEL_P"),
            ("kPRearRightVel", "REAR_RIGHT_VEL_P"),
            # Constants.OIConstants
            "kDriverControllerPort",
            # Constants.AutoConstants
            "kMaxSpeed",
            "kMaxAcceleration",
            "kMaxAngularSpeed",
            "kMaxAngularAcceleration",
            ("kPXController", "X_CONTROLLER_P"),
            ("kPYController", "Y_CONTROLLER_P"),
            ("kPThetaController", "THETA_CONTROLLER_P"),
            "kThetaControllerConstraints",
        ),
    )
    s.sub(
        "wpilibjExamples mecanumdrive",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumdrive/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kFrontLeftChannel",
            "kRearLeftChannel",
            "kFrontRightChannel",
            "kRearRightChannel",
            "kJoystickChannel",
        ),
    )
    s.sub(
        "wpilibjExamples mecanumdriveposeestimator",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumdriveposeestimator/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumdriveposeestimator/Robot.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
        ),
    )
    s.sub(
        "wpilibjExamples mechanism2d",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mechanism2d/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMetersPerPulse",
            "kElevatorMinimumLength",
        ),
    )
    s.sub(
        "wpilibjExamples motorcontrol",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/motorcontrol/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kJoystickPort",
            "kEncoderPortA",
            "kEncoderPortB",
        ),
    )

    s.sub(
        "wpilibjExamples potentiometerpid",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/potentiometerpid/Robot.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/potentiometerpid/PotentiometerPIDTest.java",
        ],
        normal_substitute(
            # Robot
            "kPotChannel",
            "kMotorChannel",
            "kJoystickChannel",
            "kFullHeight",
            "kSetpoints",
            # PotentiometerPIDTest
            "kElevatorGearing",
            "kElevatorDrumRadius",
            "kCarriageMassKg",
        ),
    )

    s.sub(
        "wpilibjExamples rapidreactcommandbot",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/RapidReactCommandBot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Drive.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Intake.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Pneumatics.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Shooter.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Storage.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameter",
            "kEncoderDistancePerPulse",
            "kTurnP",
            "kTurnI",
            "kTurnD",
            "kTurnToleranceDeg",
            "kTurnRateToleranceDegPerS",
            "kMaxTurnRateDegPerS",
            "kMaxTurnAccelerationDegPerSSquared",
            # Constants.ShooterConstants
            "kEncoderPorts",
            "kEncoderReversed",
            "kEncoderCPR",
            "kEncoderDistancePerPulse",
            "kShooterMotorPort",
            "kFeederMotorPort",
            "kShooterFreeRPS",
            "kShooterTargetRPS",
            "kShooterToleranceRPS",
            "kP", # NOTE
            "kS",
            "kV",
            "kFeederSpeed",
            # Constants.IntakeConstants
            "kMotorPort",
            "kSolenoidPorts",
            # Constants.StorageConstants
            "kMotorPort",
            "kBallSensorPort",
            # Constants.AutoConstants
            "kTimeout",
            "kDriveDistance",
            "kDriveSpeed",
            # Constants.OIConstants
            "kDriverControllerPort",
            # Pneumatics
            "kScale",
            "kOffset",
        ),
    )
    s.sub(
        "wpilibjExamples romireference",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/romireference/subsystems/Drivetrain.java",
        ],
        normal_substitute(
            # Drivetrain
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )
    s.sub(
        "wpilibjExamples selectcommand",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/selectcommand/Constants.java",
        ],
        normal_substitute(
            # Constants.OIConstants
            "kDriverControllerPort",
        ),
    )
    s.sub(
        "wpilibjExamples simpledifferentialdrivesimulation",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/simpledifferentialdrivesimulation/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/simpledifferentialdrivesimulation/Robot.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelRadius",
            "kEncoderResolution",
        ),
    )
    s.sub(
        "wpilibjExamples solenoid",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/solenoid/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kSolenoidButton",
            "kDoubleSolenoidForwardButton",
            "kDoubleSolenoidReverseButton",
            "kCompressorButton",
        ),
    )
    s.sub(
        "wpilibjExamples statespacearm",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/statespacearm/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kRaisedPosition",
            "kLoweredPosition",
            "kArmMOI",
            "kArmGearing",
        ),
    )
    s.sub(
        "wpilibjExamples statespaceelevator",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/statespaceelevator/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kHighGoalPosition",
            "kLowGoalPosition",
            "kCarriageMass",
            "kDrumRadius",
            "kElevatorGearing",
        ),
    )
    s.sub(
        "wpilibjExamples statespaceflywheel",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/statespaceflywheel/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kSpinupRadPerSec",
            "kFlywheelMomentOfInertia",
            "kFlywheelGearing",
        ),
    )
    s.sub(
        "wpilibjExamples statespaceflywheelsysid",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/statespaceflywheelsysid/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kMotorPort",
            "kEncoderAChannel",
            "kEncoderBChannel",
            "kJoystickPort",
            "kSpinupRadPerSec",
            ("kFlywheelKv", "FLYWHEEL_V"), # NOTE
            ("kFlywheelKa", "FLYWHEEL_A"),
        ),
    )
    s.sub(
        "wpilibjExamples swervebot",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/SwerveModule.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            # SwerveModule
            "kWheelRadius",
            "kEncoderResolution",
            "kModuleMaxAngularVelocity",
            "kModuleMaxAngularAcceleration",
        ),
    )
    s.sub(
        "wpilibjExamples swervecontrollercommand",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/SwerveModule.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            "kFrontLeftDriveMotorPort",
            "kRearLeftDriveMotorPort",
            "kFrontRightDriveMotorPort",
            "kRearRightDriveMotorPort",
            "kFrontLeftTurningMotorPort",
            "kRearLeftTurningMotorPort",
            "kFrontRightTurningMotorPort",
            "kRearRightTurningMotorPort",
            "kFrontLeftTurningEncoderPorts",
            "kRearLeftTurningEncoderPorts",
            "kFrontRightTurningEncoderPorts",
            "kRearRightTurningEncoderPorts",
            "kFrontLeftTurningEncoderReversed",
            "kRearLeftTurningEncoderReversed",
            "kFrontRightTurningEncoderReversed",
            "kRearRightTurningEncoderReversed",
            "kFrontLeftDriveEncoderPorts",
            "kRearLeftDriveEncoderPorts",
            "kFrontRightDriveEncoderPorts",
            "kRearRightDriveEncoderPorts",
            "kFrontLeftDriveEncoderReversed",
            "kRearLeftDriveEncoderReversed",
            "kFrontRightDriveEncoderReversed",
            "kRearRightDriveEncoderReversed",
            "kDrivePeriod",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kWheelBase",
            "kDriveKinematics",
            "kGyroReversed",
            "ks", # NOTE
            "kv",
            "ka",
            "kMaxSpeed",
            # Constants.ModuleConstants
            "kMaxModuleAngularSpeed",
            "kMaxModuleAngularAcceleration",
            "kEncoderCPR",
            "kWheelDiameter",
            "kDriveEncoderDistancePerPulse",
            "kTurningEncoderDistancePerPulse",
            ("kPModuleTurningController", "MODULE_TURNING_CONTROLLER_P"),
            ("kPModuleDriveController", "MODULE_DRIVE_CONTROLLER_P"),
            # Constants.OIConstants
            "kDriverControllerPort",
            # Constants.AutoConstants
            "kMaxSpeed",
            "kMaxAcceleration",
            "kMaxAngularSpeed",
            "kMaxAngularAcceleration",
            ("kPXController", "X_CONTROLLER_P"),
            ("kPYController", "Y_CONTROLLER_P"),
            ("kPThetaController", "THETA_CONTROLLER_P"),
            "kThetaControllerConstraints",
        ),
    )
    s.sub(
        "wpilibjExamples swervedriveposeestimator",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervedriveposeestimator/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervedriveposeestimator/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervedriveposeestimator/SwerveModule.java",
        ],
        normal_substitute(
            # Drivetrain
            "kMaxSpeed",
            "kMaxAngularSpeed",
            # SwerveModule
            "kWheelRadius",
            "kEncoderResolution",
            "kModuleMaxAngularVelocity",
            "kModuleMaxAngularAcceleration",
        ),
    )
    s.sub(
        "wpilibjExamples sysidroutine",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysidroutine/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysidroutine/SysIdRoutineBot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysidroutine/subsystems/Drive.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysidroutine/subsystems/Shooter.java",
        ],
        normal_substitute(
            # Constants.DriveConstants
            ("kLeftMotor1Port", "LEFT_MOTOR_1_PORT"),
            ("kLeftMotor2Port", "LEFT_MOTOR_2_PORT"),
            ("kRightMotor1Port", "RIGHT_MOTOR_1_PORT"),
            ("kRightMotor2Port", "RIGHT_MOTOR_2_PORT"),
            "kLeftEncoderPorts",
            "kRightEncoderPorts",
            "kLeftEncoderReversed",
            "kRightEncoderReversed",
            "kEncoderCPR",
            "kWheelDiameter",
            "kEncoderDistancePerPulse",
            # Constants.ShooterConstants
            "kEncoderPorts",
            "kEncoderReversed",
            "kEncoderCPR",
            "kEncoderDistancePerPulse",
            "kShooterMotorPort",
            "kFeederMotorPort",
            "kShooterFreeRPS",
            "kShooterTargetRPS",
            "kShooterToleranceRPS",
            "kP", # NOTE
            "kS",
            "kV",
            "kA",
            "kFeederSpeed",
            # Constants.IntakeConstants
            "kMotorPort",
            "kSolenoidPorts",
            # Constants.StorageConstants
            "kMotorPort",
            "kBallSensorPort",
            # Constants.AutoConstants
            "kTimeout",
            "kDriveDistance",
            "kDriveSpeed",
            # Constants.OIConstants
            "kDriverControllerPort",
        ),
    )

    s.sub(
        "wpilibjExamples unittest",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/unittest/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/unittest/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/unittest/subsystems/Intake.java",
            "wpilibjExamples/src/test/java/edu/wpi/first/wpilibj/examples/unittest/subsystems/IntakeTest.java",
        ],
        normal_substitute(
            # Constants
            "kJoystickIndex",
            # Constants.IntakeConstants
            "kMotorPort",
            "kPistonFwdChannel",
            "kPistonRevChannel",
            "kIntakeSpeed",
        ),
    )

    s.sub(
        "wpilibjExamples xrpreference",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/xrpreference/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/xrpreference/subsystems/Drivetrain.java",
        ],
        normal_substitute(
            # Constants.OperatorConstants
            "kDriverControllerPort",
            # Drivetrain
            "kGearRatio",
            "kCountsPerMotorShaftRev",
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )

    s.sub(
        "wpilibjExamples commandbased",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/commandbased/Constants.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/commandbased/RobotContainer.java",
        ],
        normal_substitute(
            # Constants.OperatorConstants
            "kDriverControllerPort",
        ),
    )

    s.sub(
        "wpilibjExamples romicommandbased",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/romicommandbased/subsystems/RomiDrivetrain.java",
        ],
        normal_substitute(
            # RomiDrivetrain
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )
    s.sub(
        "wpilibjExamples romieducational",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/romieducational/RomiDrivetrain.java",
        ],
        normal_substitute(
            # RomiDrivetrain
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )
    s.sub(
        "wpilibjExamples romitimed",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/romitimed/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/romitimed/RomiDrivetrain.java",
        ],
        normal_substitute(
            # Robot
            "kDefaultAuto",
            "kCustomAuto",
            # RomiDrivetrain
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )

    s.sub(
        "wpilibjExamples timed",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/timed/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kDefaultAuto",
            "kCustomAuto",
        ),
    )
    s.sub(
        "wpilibjExamples timeslice",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/timeslice/Robot.java",
        ],
        normal_substitute(
            # Robot
            "kDefaultAuto",
            "kCustomAuto",
        ),
    )

    s.sub(
        "wpilibjExamples xrpcommandbased",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/xrpcommandbased/subsystems/XRPDrivetrain.java",
        ],
        normal_substitute(
            # XRPDrivetrain
            "kGearRatio",
            "kCountsPerMotorShaftRev",
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )
    s.sub(
        "wpilibjExamples xrpeducational",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/xrpeducational/XRPDrivetrain.java",
        ],
        normal_substitute(
            # XRPDrivetrain
            "kGearRatio",
            "kCountsPerMotorShaftRev",
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )
    s.sub(
        "wpilibjExamples xrptimed",
        [
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/xrptimed/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/xrptimed/XRPDrivetrain.java",
        ],
        normal_substitute(
            # Robot
            "kDefaultAuto",
            "kCustomAuto",
            # XRPDrivetrain
            "kGearRatio",
            "kCountsPerMotorShaftRev",
            "kCountsPerRevolution",
            "kWheelDiameterInch",
        ),
    )

    # wpimath

    s.sub(
        "wpimath geometry ZERO",
        [
            "apriltag/src/main/java/edu/wpi/first/apriltag/AprilTagFieldLayout.java",
            "apriltag/src/test/java/edu/wpi/first/apriltag/AprilTagDetectorTest.java",
            "apriltag/src/test/java/edu/wpi/first/apriltag/AprilTagPoseSetOriginTest.java",
            "apriltag/src/test/java/edu/wpi/first/apriltag/AprilTagSerializationTest.java",
            "apriltag/src/test/java/edu/wpi/first/apriltag/LoadConfigTest.java",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/MecanumControllerCommandTest.java",
            "wpilibNewCommands/src/test/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommandTest.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/MecanumDrive.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/smartdashboard/Field2d.java",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/smartdashboard/FieldObject2d.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/drive/MecanumDriveTest.java",
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/simulation/DifferentialDrivetrainSimTest.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdriveposeestimator/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumdriveposeestimator/Drivetrain.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/simpledifferentialdrivesimulation/Robot.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java",
            "wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervedriveposeestimator/Drivetrain.java",
            "wpimath/src/main/java/edu/wpi/first/math/ComputerVisionUtil.java",
            "wpimath/src/main/java/edu/wpi/first/math/controller/HolonomicDriveController.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Ellipse2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Pose2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Pose3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Rectangle2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Rotation2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Rotation3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Transform2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Transform3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Translation2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Translation3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/ChassisSpeeds.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometry.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometry3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/MecanumDriveKinematics.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/MecanumDriveOdometry.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/MecanumDriveOdometry3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveDriveKinematics.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry3d.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveModulePosition.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveModuleState.java",
            "wpimath/src/main/java/edu/wpi/first/math/spline/PoseWithCurvature.java",
            "wpimath/src/main/java/edu/wpi/first/math/trajectory/Trajectory.java",
            "wpimath/src/main/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.java",
            "wpimath/src/test/java/edu/wpi/first/math/controller/HolonomicDriveControllerTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/controller/LTVDifferentialDriveControllerTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/controller/LTVUnicycleControllerTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/ExtendedKalmanFilterTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/KalmanFilterTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/MecanumDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/MecanumDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/UnscentedKalmanFilterTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/CoordinateSystemTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Pose2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Pose3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Rotation2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Rotation3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Translation2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Twist2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Twist3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/interpolation/TimeInterpolatableBufferTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/ChassisSpeedsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometry3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometryTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/MecanumDriveOdometry3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/MecanumDriveOdometryTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveKinematicsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveOdometryTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveModuleStateTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/path/TravelingSalesmanTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/spline/CubicHermiteSplineTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/spline/QuinticHermiteSplineTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/TrajectoryConcatenateTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/TrajectoryGeneratorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/TrajectoryTransformTest.java",
        ],
        normal_substitute(
            "kZero",
        ),
    )

    s.sub(
        "wpimath DCMotor",
        [
            "wpilibc/src/main/native/cpp/simulation/DCMotorSim.cpp",
            "wpilibc/src/main/native/cpp/simulation/FlywheelSim.cpp",
            "wpilibc/src/test/native/cpp/simulation/DCMotorSimTest.cpp",
            "wpimath/src/main/native/include/frc/system/plant/DCMotor.h",
            "wpimath/src/main/native/include/frc/system/plant/LinearSystemId.h",
            "wpimath/src/test/native/cpp/estimator/ExtendedKalmanFilterTest.cpp",
            "wpimath/src/test/native/cpp/estimator/UnscentedKalmanFilterTest.cpp",
            "wpimath/src/test/native/cpp/system/plant/proto/DCMotorProtoTest.cpp",
            "wpimath/src/test/native/cpp/system/plant/struct/DCMotorStructTest.cpp",
        ],
        normal_substitute(
            ("Kv", "V"),
        ),
    )
    s.sub(
        "wpimath Debouncer DebounceType",
        [
            "wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/button/Trigger.java",
            "wpilibNewCommands/src/main/native/include/frc2/command/button/Trigger.h",
            "wpilibc/src/main/native/include/frc/event/BooleanEvent.h",
            "wpilibj/src/main/java/edu/wpi/first/wpilibj/event/BooleanEvent.java",
            "wpimath/src/main/java/edu/wpi/first/math/filter/Debouncer.java",
            "wpimath/src/main/native/cpp/filter/Debouncer.cpp",
            "wpimath/src/main/native/include/frc/filter/Debouncer.h",
            "wpimath/src/test/java/edu/wpi/first/math/filter/DebouncerTest.java",
            "wpimath/src/test/native/cpp/filter/DebouncerTest.cpp",
        ],
        normal_substitute(
            "kRising",
            "kFalling",
            "kBoth",
        ),
    )
    s.sub(
        "wpimath Rotation2d",
        [
            "wpilibj/src/test/java/edu/wpi/first/wpilibj/drive/MecanumDriveTest.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/Rotation2d.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveModuleState.java",
            "wpimath/src/main/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/MecanumDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/MecanumDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Pose2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Rotation2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Translation2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Twist2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/interpolation/TimeInterpolatableBufferTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/ChassisSpeedsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/MecanumDriveOdometry3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/MecanumDriveOdometryTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveKinematicsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveOdometryTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveModuleStateTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/spline/CubicHermiteSplineTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/spline/QuinticHermiteSplineTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/DifferentialDriveVoltageConstraintTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/EllipticalRegionConstraintTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/TrajectoryGeneratorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/TrajectoryTransformTest.java",
        ],
        normal_substitute(
            # "kZero", # Included in geometry ZERO
            "kCW_Pi_2",
            "kCW_90deg",
            "kCCW_Pi_2",
            "kCCW_90deg",
            "kPi",
            # "k180deg",  # NOTE This isn't included?
        ),
    )

    s.sub(
        "wpimath estimator KalmanFilterLatencyCompensator.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/estimator/KalmanFilterLatencyCompensator.java",
        ],
        normal_substitute(
            "kMaxPastObserverStates",
        ),
    )
    s.sub(
        "wpimath estimator PoseEstimator BUFFER_DURATION",
        [
            "wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java",
            "wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator3d.java",
        ],
        normal_substitute(
            "kBufferDuration",
        ),
    )
    s.sub(
        "wpimath spline SplineParameterizer.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/spline/SplineParameterizer.java",
        ],
        normal_substitute(
            ("kMaxDx", "MAX_D_X"),
            ("kMaxDy", "MAX_D_Y"),
            ("kMaxDtheta", "MAX_D_THETA"),
            "kMalformedSplineExceptionMsg",
            "kMaxIterations",
        ),
    )
    s.sub(
        "wpimath system NumericalJacobian.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/system/NumericalJacobian.java",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpimath trajectory ExponentialProfile.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/trajectory/ExponentialProfile.java",
        ],
        normal_substitute(
            ("fromCharacteristics(kMaxV, kV, kA);", "fromCharacteristics(MAX_V, V, A);"),
            ("kV", "v"),
            ("kA", "a"),
        ),
    )
    s.sub(
        "wpimath trajectory TrajectoryGenerator.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.java",
        ],
        normal_substitute(
            "kFlip",
            "kDoNothingTrajectory",
        ),
    )
    s.sub(
        "wpimath trajectory TrapezoidProfile.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/trajectory/TrapezoidProfile.java",
        ],
        normal_substitute(
            "kMaxV",
            "kMaxA",
        ),
    )
    s.sub(
        "wpimath util Units.java",
        [
            "wpimath/src/main/java/edu/wpi/first/math/util/Units.java",
        ],
        normal_substitute(
            "kInchesPerFoot",
            "kMetersPerInch",
            "kSecondsPerMinute",
            "kMillisecondsPerSecond",
            "kKilogramsPerLb",
        ),
    )

    s.sub(
        "wpimath controller struct ArmFeedforwardStruct.cpp",
        [
            "wpimath/src/main/native/cpp/controller/struct/ArmFeedforwardStruct.cpp",
        ],
        normal_substitute(
            "kKsOff",
            "kKgOff",
            "kKvOff",
            "kKaOff",
        ),
    )
    s.sub(
        "wpimath controller struct DifferentialDriveFeedforwardStruct.cpp",
        [
            "wpimath/src/main/native/cpp/controller/struct/DifferentialDriveFeedforwardStruct.cpp",
        ],
        normal_substitute(
            "kKvLinearOff",
            "kKaLinearOff",
            "kKvAngularOff",
            "kKaAngularOff",
        ),
    )
    s.sub(
        "wpimath controller struct DifferentialDriveWheelVoltagesStruct.cpp",
        [
            "wpimath/src/main/native/cpp/controller/struct/DifferentialDriveWheelVoltagesStruct.cpp",
        ],
        normal_substitute(
            "kLeftOff",
            "kRightOff",
        ),
    )
    s.sub(
        "wpimath controller struct ElevatorFeedforwardStruct.cpp",
        [
            "wpimath/src/main/native/cpp/controller/struct/ElevatorFeedforwardStruct.cpp",
        ],
        normal_substitute(
            "kKsOff",
            "kKgOff",
            "kKvOff",
            "kKaOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Ellipse2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Ellipse2dStruct.cpp",
        ],
        normal_substitute(
            "kCenterOff",
            "kXSemiAxisOff",
            "kYSemiAxisOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Pose2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Pose2dStruct.cpp",
        ],
        normal_substitute(
            "kTranslationOff",
            "kRotationOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Pose3dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Pose3dStruct.cpp",
        ],
        normal_substitute(
            "kTranslationOff",
            "kRotationOff",
        ),
    )
    s.sub(
        "wpimath geometry struct QuaternionStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/QuaternionStruct.cpp",
        ],
        normal_substitute(
            "kWOff",
            "kXOff",
            "kYOff",
            "kZOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Rectangle2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Rectangle2dStruct.cpp",
        ],
        normal_substitute(
            "kCenterOff",
            "kXWidthOff",
            "kYWidthOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Rotation2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Rotation2dStruct.cpp",
        ],
        normal_substitute(
            "kValueOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Rotation3dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Rotation3dStruct.cpp",
        ],
        normal_substitute(
            "kQOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Transform2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Transform2dStruct.cpp",
        ],
        normal_substitute(
            "kTranslationOff",
            "kRotationOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Transform3dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Transform3dStruct.cpp",
        ],
        normal_substitute(
            "kTranslationOff",
            "kRotationOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Translation2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Translation2dStruct.cpp",
        ],
        normal_substitute(
            "kXOff",
            "kYOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Translation3dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Translation3dStruct.cpp",
        ],
        normal_substitute(
            "kXOff",
            "kYOff",
            "kZOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Twist2dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Twist2dStruct.cpp",
        ],
        normal_substitute(
            "kDxOff",
            "kDyOff",
            "kDthetaOff",
        ),
    )
    s.sub(
        "wpimath geometry struct Twist3dStruct.cpp",
        [
            "wpimath/src/main/native/cpp/geometry/struct/Twist3dStruct.cpp",
        ],
        normal_substitute(
            "kDxOff",
            "kDyOff",
            "kDzOff",
            "kRxOff",
            "kRyOff",
            "kRzOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct ChassisSpeedsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/ChassisSpeedsStruct.cpp",
        ],
        normal_substitute(
            "kVxOff",
            "kVyOff",
            "kOmegaOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct DifferentialDriveKinematicsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/DifferentialDriveKinematicsStruct.cpp",
        ],
        normal_substitute(
            # NOTE This separates track and width
            ("kTrackwidthOff", "TRACK_WIDTH_OFF"),
        ),
    )
    s.sub(
        "wpimath kinematics struct DifferentialDriveWheelPositionsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/DifferentialDriveWheelPositionsStruct.cpp",
        ],
        normal_substitute(
            "kLeftOff",
            "kRightOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct DifferentialDriveWheelSpeedsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/DifferentialDriveWheelSpeedsStruct.cpp",
        ],
        normal_substitute(
            "kLeftOff",
            "kRightOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct MecanumDriveKinematicsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/MecanumDriveKinematicsStruct.cpp",
        ],
        normal_substitute(
            "kFrontLeftOff",
            "kFrontRightOff",
            "kRearLeftOff",
            "kRearRightOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct MecanumDriveWheelPositionsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/MecanumDriveWheelPositionsStruct.cpp",
        ],
        normal_substitute(
            "kFrontLeftOff",
            "kFrontRightOff",
            "kRearLeftOff",
            "kRearRightOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct MecanumDriveWheelSpeedsStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/MecanumDriveWheelSpeedsStruct.cpp",
        ],
        normal_substitute(
            "kFrontLeftOff",
            "kFrontRightOff",
            "kRearLeftOff",
            "kRearRightOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct SwerveModulePositionStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/SwerveModulePositionStruct.cpp",
        ],
        normal_substitute(
            "kDistanceOff",
            "kAngleOff",
        ),
    )
    s.sub(
        "wpimath kinematics struct SwerveModuleStateStruct.cpp",
        [
            "wpimath/src/main/native/cpp/kinematics/struct/SwerveModuleStateStruct.cpp",
        ],
        normal_substitute(
            "kSpeedOff",
            "kAngleOff",
        ),
    )
    s.sub(
        "wpimath spline struct CubicHermiteSplineStruct.cpp",
        [
            "wpimath/src/main/native/cpp/spline/struct/CubicHermiteSplineStruct.cpp",
        ],
        normal_substitute(
            "kXInitialOff",
            "kXFinalOff",
            "kYInitialOff",
            "kYFinalOff",
        ),
    )
    s.sub(
        "wpimath spline struct QuinticHermiteSplineStruct.cpp",
        [
            "wpimath/src/main/native/cpp/spline/struct/QuinticHermiteSplineStruct.cpp",
        ],
        normal_substitute(
            "kXInitialOff",
            "kXFinalOff",
            "kYInitialOff",
            "kYFinalOff",
        ),
    )
    s.sub(
        "wpimath system plant struct DCMotorStruct.cpp",
        [
            "wpimath/src/main/native/cpp/system/plant/struct/DCMotorStruct.cpp",
        ],
        normal_substitute(
            "kNominalVoltageOff",
            "kStallTorqueOff",
            "kStallCurrentOff",
            "kFreeCurrentOff",
            "kFreeSpeedOff",
        ),
    )
    s.sub(
        "wpimath trajectory TrajectoryGenerator.cpp TrajectoryGenerator.h",
        [
            "wpimath/src/main/native/cpp/trajectory/TrajectoryGenerator.cpp",
            "wpimath/src/main/native/include/frc/trajectory/TrajectoryGenerator.h",
        ],
        normal_substitute(
            "kDoNothingTrajectory",
        ),
    )
    s.sub(
        "wpimath trajectory TrajectoryParameterizer.cpp/.h",
        [
            "wpimath/src/main/native/cpp/trajectory/TrajectoryParameterizer.cpp",
            "wpimath/src/main/native/include/frc/trajectory/TrajectoryParameterizer.h",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpimath controller struct SimpleMotorFeedforwardStruct.h",
        [
            "wpimath/src/main/native/include/frc/controller/struct/SimpleMotorFeedforwardStruct.h",
        ],
        normal_substitute(
            "kKsOff",
            "kKvOff",
            "kKaOff",
            "kDtOff",
        ),
    )
    s.sub(
        "wpimath estimator KalmanFilterLatencyCompensator.h",
        [
            "wpimath/src/main/native/include/frc/estimator/KalmanFilterLatencyCompensator.h",
        ],
        normal_substitute(
            "kMaxPastObserverStates",
        ),
    )
    s.sub(
        "wpimath estimator PoseEstimator.h PoseEstimator3d.h",
        [
            "wpimath/src/main/native/include/frc/estimator/PoseEstimator.h",
            "wpimath/src/main/native/include/frc/estimator/PoseEstimator3d.h",
        ],
        normal_substitute(
            "kBufferDuration",
        ),
    )
    s.sub(
        "wpimath kinematics struct SwerveDriveKinematicsStruct.h",
        [
            "wpimath/src/main/native/include/frc/kinematics/struct/SwerveDriveKinematicsStruct.h",
        ],
        normal_substitute(
            "kTypeName",
            "kSchema",
            "kModulesOff",
        ),
    )
    s.sub(
        "wpimath spline SplineParameterizer.h",
        [
            "wpimath/src/main/native/include/frc/spline/SplineParameterizer.h",
            "wpimath/src/test/native/cpp/spline/CubicHermiteSplineTest.cpp",
            "wpimath/src/test/native/cpp/spline/QuinticHermiteSplineTest.cpp",
        ],
        normal_substitute(
            "kMalformedSplineExceptionMsg",
            "kMaxDx",
            "kMaxDy",
            "kMaxDtheta",
            "kMaxIterations",
        ),
    )
    s.sub(
        "wpimath struct MatrixStruct.h",
        [
            "wpimath/src/main/native/include/frc/struct/MatrixStruct.h",
        ],
        normal_substitute(
            "kTypeName",
            "kSchema",
            "kDataOff",
        ),
    )
    s.sub(
        "wpimath struct VectorStruct.h",
        [
            "wpimath/src/main/native/include/frc/struct/VectorStruct.h",
        ],
        normal_substitute(
            "kTypeName",
            "kSchema",
            "kDataOff",
        ),
    )
    s.sub(
        "wpimath system LinearSystemLoop.h",
        [
            "wpimath/src/main/native/include/frc/system/LinearSystemLoop.h",
        ],
        normal_substitute(
            "kStates",
            "kInputs",
            "kOutputs",
        ),
    )
    s.sub(
        "wpimath system NumericalIntegration.h",
        [
            "wpimath/src/main/native/include/frc/system/NumericalIntegration.h",
        ],
        normal_substitute(
            "kDim",
        ),
    )
    s.sub(
        "wpimath system NumericalJacobian.h",
        [
            "wpimath/src/main/native/include/frc/system/NumericalJacobian.h",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpimath system plant LinearSystemId.h",
        [
            "wpimath/src/main/native/include/frc/system/plant/LinearSystemId.h",
        ],
        normal_substitute(
            # ("Kv", "V"),  # Included in wpimath DCMotor
            ("Ka", "A"),
        ),
    )
    s.sub(
        "wpimath system struct LinearSystemStruct.h",
        [
            "wpimath/src/main/native/include/frc/system/struct/LinearSystemStruct.h",
        ],
        normal_substitute(
            "kTypeName",
            "kSchema",
            "kAOff",
            "kBOff",
            "kCOff",
            "kDOff",
        ),
    )

    s.sub(
        "wpimath test controller DifferentialDriveFeedforwardTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/DifferentialDriveFeedforwardTest.java",
        ],
        normal_substitute(
            "kVLinear",
            "kALinear",
            "kVAngular",
            "kAAngular",
        ),
    )
    s.sub(
        "wpimath test controller HolonomicDriveControllerTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/HolonomicDriveControllerTest.java",
        ],
        normal_substitute(
            "kTolerance",
            "kAngularTolerance",
            "kDt",
            "kTotalTime",
        ),
    )
    s.sub(
        "wpimath test controller LTVDifferentialDriveControllerTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/LTVDifferentialDriveControllerTest.java",
        ],
        normal_substitute(
            "kTolerance",
            "kAngularTolerance",
            "kX",
            "kY",
            "kHeading",
            "kLeftVelocity",
            "kRightVelocity",
            "kLinearV",
            "kLinearA",
            "kAngularV",
            "kAngularA",
            ("kTrackwidth", "TRACK_WIDTH"),
            "kDt",
        ),
    )
    s.sub(
        "wpimath test controller LTVUnicycleControllerTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/LTVUnicycleControllerTest.java",
        ],
        normal_substitute(
            "kTolerance",
            "kAngularTolerance",
            "kDt",
        ),
    )
    s.sub(
        "wpimath test controller LinearSystemLoopTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/LinearSystemLoopTest.java",
        ],
        normal_substitute(
            "kDt",
            "kPositionStddev",
        ),
    )
    s.sub(
        "wpimath test controller PIDToleranceTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/PIDToleranceTest.java",
        ],
        normal_substitute(
            "kSetpoint",
            "kTolerance",
            "kRange",
        ),
    )
    s.sub(
        "wpimath test controller ProfiledPIDInputOutputTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/controller/ProfiledPIDInputOutputTest.java",
        ],
        normal_substitute(
            "kSetpoint",
            "kMeasurement",
            "kGoal",
        ),
    )

    s.sub(
        "wpimath test estimator kEpsilon",
        [
            "wpimath/src/test/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/MecanumDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/MecanumDrivePoseEstimatorTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimatorTest.java",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpimath test estimator KalmanFilterTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/estimator/KalmanFilterTest.java",
        ],
        normal_substitute(
            "kDt",
        ),
    )

    s.sub(
        "wpimath test filter LinearFilterTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/filter/LinearFilterTest.java",
        ],
        normal_substitute(
            "kFilterStep",
            "kFilterTime",
            "kSinglePoleIIRTimeConstant",
            "kHighPassTimeConstant",
            "kMovAvgTaps",
            "kSinglePoleIIRExpectedOutput",
            "kHighPassExpectedOutput",
            "kMovAvgExpectedOutput",
            ("kStdDev", "STDDEV"),
        ),
    )

    s.sub(
        "wpimath test geometry kEpsilon",
        [
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Ellipse2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Pose2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Pose3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Rectangle2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Rotation2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Rotation3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Transform2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Translation2dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/geometry/Translation3dTest.java",
            "wpimath/src/test/native/cpp/geometry/Ellipse2dTest.cpp",
            "wpimath/src/test/native/cpp/geometry/Pose3dTest.cpp",
            "wpimath/src/test/native/cpp/geometry/Rectangle2dTest.cpp",
            "wpimath/src/test/native/cpp/geometry/Translation3dTest.cpp",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )

    s.sub(
        "wpimath test kinematics kEpsilon",
        [
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/ChassisSpeedsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/DifferentialDriveKinematicsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometry3dTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometryTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/MecanumDriveKinematicsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveDriveKinematicsTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/SwerveModuleStateTest.java",
            "wpimath/src/test/native/cpp/kinematics/ChassisSpeedsTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/DifferentialDriveKinematicsTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/DifferentialDriveOdometry3dTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/DifferentialDriveOdometryTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/SwerveDriveKinematicsTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/SwerveDriveOdometry3dTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/SwerveDriveOdometryTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/SwerveModuleStateTest.cpp",
        ],
        normal_substitute(
            "kEpsilon",
        ),
    )
    s.sub(
        "wpimath test kinematics MecanumDriveKinematicsTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/kinematics/MecanumDriveKinematicsTest.java",
        ],
        normal_substitute(
            "kFactor",
        ),
    )

    s.sub(
        "wpimath test spline *HermiteSplineTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/spline/CubicHermiteSplineTest.java",
            "wpimath/src/test/java/edu/wpi/first/math/spline/QuinticHermiteSplineTest.java",
        ],
        normal_substitute(
            "kMaxDx",
            "kMaxDy",
            "kMaxDtheta",
        ),
    )

    s.sub(
        "wpimath test trajectory ExponentialProfileTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/ExponentialProfileTest.java",
        ],
        normal_substitute(
            "kDt",
        ),
    )
    s.sub(
        "wpimath test trajectory TrapezoidProfileTest.java",
        [
            "wpimath/src/test/java/edu/wpi/first/math/trajectory/TrapezoidProfileTest.java",
        ],
        normal_substitute(
            "kDt",
        ),
    )

    s.sub(
        "wpimath test ProtoTest TEST_DATA",
        [
            "wpimath/src/test/native/cpp/ProtoTestBase.h",
            "wpimath/src/test/native/cpp/controller/proto/DifferentialDriveFeedforwardProtoTest.cpp",
            "wpimath/src/test/native/cpp/controller/proto/SimpleMotorFeedforwardProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/SwerveDriveKinematicsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/SwerveDriveKinematicsStructTest.cpp",
            "wpimath/src/test/native/cpp/proto/MatrixProtoTest.cpp",
            "wpimath/src/test/native/cpp/proto/VectorProtoTest.cpp",
            "wpimath/src/test/native/cpp/spline/proto/CubicHermiteSplineProtoTest.cpp",
            "wpimath/src/test/native/cpp/spline/proto/QuinticHermiteSplineProtoTest.cpp",
            "wpimath/src/test/native/cpp/spline/struct/CubicHermiteSplineStructTest.cpp",
            "wpimath/src/test/native/cpp/spline/struct/QuinticHermiteSplineStructTest.cpp",
            "wpimath/src/test/native/cpp/struct/MatrixStructTest.cpp",
            "wpimath/src/test/native/cpp/struct/VectorStructTest.cpp",
            "wpimath/src/test/native/cpp/system/proto/LinearSystemProtoTest.cpp",
            "wpimath/src/test/native/cpp/system/struct/LinearSystemStructTest.cpp",
        ],
        normal_substitute(
            "kTestData",
        ),
    )
    s.sub(
        "wpimath test StructTest TEST_DATA",
        [
            "wpimath/src/test/native/cpp/StructTestBase.h",
            "wpimath/src/test/native/cpp/controller/struct/DifferentialDriveFeedforwardStructTest.cpp",
            "wpimath/src/test/native/cpp/controller/struct/SimpleMotorFeedforwardStructTest.cpp",
        ],
        normal_substitute(
            "kTestData",
        ),
    )

    s.sub(
        "wpimath StateSpaceTest.cpp",
        [
            "wpimath/src/test/native/cpp/StateSpaceTest.cpp",
        ],
        normal_substitute(
            ("kPositionStddev", "POSITION_STD_DEV"),
            "kDt",
        ),
    )
    s.sub(
        "wpimath controller ArmFeedforwardTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/ArmFeedforwardTest.cpp",
        ],
        normal_substitute(
            # NOTE
            ("Ks", "s"),
            ("Kv", "v"),
            ("Ka", "a"),
            ("Kg", "g"),
            # These aren't even done consistently within the file...
        ),
    )
    s.sub(
        "wpimath controller DifferentialDriveFeedforwardTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/DifferentialDriveFeedforwardTest.cpp",
        ],
        normal_substitute(
            "kVLinear",
            "kALinear",
            "kVAngular",
            "kAAngular",
            ("trackwidth", "TRACK_WIDTH"),
            "dt",
        ),
    )
    s.sub(
        "wpimath controller HolonomicDriveControllerTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/HolonomicDriveControllerTest.cpp",
        ],
        normal_substitute(
            "kTolerance",
            "kAngularTolerance",
            "kDt",
        ),
    )
    s.sub(
        "wpimath controller LTVDifferentialDriveControllerTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/LTVDifferentialDriveControllerTest.cpp",
        ],
        normal_substitute(
            "kTolerance",
            "kAngularTolerance",
            "kX",
            "kY",
            "kHeading",
            "kLeftVelocity",
            "kRightVelocity",
            "kLinearV",
            "kLinearA",
            "kAngularV",
            "kAngularA",
            ("kTrackwidth", "TRACK_WIDTH"),  # NOTE
            "kDt",
        ),
    )
    s.sub(
        "wpimath controller LTVUnicycleControllerTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/LTVUnicycleControllerTest.cpp",
        ],
        normal_substitute(
            "kTolerance",
            "kAngularTolerance",
            "kDt",
        ),
    )
    s.sub(
        "wpimath controller PIDToleranceTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/PIDToleranceTest.cpp",
        ],
        normal_substitute(
            "kSetpoint",
            "kRange",
            "kTolerance",
        ),
    )
    s.sub(
        "wpimath controller ProfiledPIDInputOutputTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/ProfiledPIDInputOutputTest.cpp",
        ],
        normal_substitute(
            "kSetpoint",
            "kMeasurement",
            "kGoal",
        ),
    )
    s.sub(
        "wpimath controller SimpleMotorFeedforwardTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/SimpleMotorFeedforwardTest.cpp",
        ],
        normal_substitute(
            # NOTE
            ("Ks", "kS"),
            ("Kv", "kV"),
            ("Ka", "kA"),
        ),
    )

    s.sub(
        "wpimath controller proto ArmFeedforwardProtoTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/proto/ArmFeedforwardProtoTest.cpp",
        ],
        normal_substitute(
            # NOTE
            ("Ks", "S"),
            ("Kg", "G"),
            ("Kv", "V"),
            ("Ka", "A"),
            "kExpectedData",
        ),
    )
    s.sub(
        "wpimath controller proto DifferentialDriveWheelVoltagesProtoTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/proto/DifferentialDriveWheelVoltagesProtoTest.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )
    s.sub(
        "wpimath controller proto ElevatorFeedforwardProtoTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/proto/ElevatorFeedforwardProtoTest.cpp",
        ],
        normal_substitute(
            # NOTE
            ("Ks", "S"),
            ("Kg", "G"),
            ("Kv", "V"),
            ("Ka", "A"),
            "kExpectedData",
        ),
    )
    s.sub(
        "wpimath controller struct ArmFeedforwardStructTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/struct/ArmFeedforwardStructTest.cpp",
        ],
        normal_substitute(
            # NOTE
            ("Ks", "S"),
            ("Kg", "G"),
            ("Kv", "V"),
            ("Ka", "A"),
            "kExpectedData",
        ),
    )
    s.sub(
        "wpimath controller struct DifferentialDriveWheelVoltagesStructTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/struct/DifferentialDriveWheelVoltagesStructTest.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )
    s.sub(
        "wpimath controller struct ElevatorFeedforwardStructTest.cpp",
        [
            "wpimath/src/test/native/cpp/controller/struct/ElevatorFeedforwardStructTest.cpp",
        ],
        normal_substitute(
            # NOTE
            ("Ks", "S"),
            ("Kg", "G"),
            ("Kv", "V"),
            ("Ka", "A"),
            "kExpectedData",
        ),
    )

    s.sub(
        "wpimath estimator DifferentialDrivePoseEstimator3dTest.cpp",
        [
            "wpimath/src/test/native/cpp/estimator/DifferentialDrivePoseEstimator3dTest.cpp",
        ],
        normal_substitute(
            "kVisionUpdateRate",
            "kVisionUpdateDelay",
        ),
    )
    s.sub(
        "wpimath estimator *PoseEstimatorTest visionUpdateRate visionUpdateDelay",
        [
            "wpimath/src/test/native/cpp/estimator/DifferentialDrivePoseEstimatorTest.cpp",
            "wpimath/src/test/native/cpp/estimator/MecanumDrivePoseEstimator3dTest.cpp",
            "wpimath/src/test/native/cpp/estimator/MecanumDrivePoseEstimatorTest.cpp",
            "wpimath/src/test/native/cpp/estimator/SwerveDrivePoseEstimator3dTest.cpp",
            "wpimath/src/test/native/cpp/estimator/SwerveDrivePoseEstimatorTest.cpp",
        ],
        make_substitution_strs(
            ("kVisionUpdateRate", "visionUpdateRate"),
            ("kVisionUpdateDelay", "visionUpdateDelay"),
        ),
    )

    s.sub(
        "wpimath filter LinearFilterNoiseTest.cpp",
        [
            "wpimath/src/test/native/cpp/filter/LinearFilterNoiseTest.cpp",
        ],
        normal_substitute(
            "kFilterStep",
            "kFilterTime",
            "kSinglePoleIIRTimeConstant",
            "kMovAvgTaps",
            "kTestSinglePoleIIR",
            "kTestMovAvg",
        ),
    )
    s.sub(
        "wpimath filter LinearFilterOutputTest.cpp",
        [
            "wpimath/src/test/native/cpp/filter/LinearFilterOutputTest.cpp",
        ],
        normal_substitute(
            "kFilterStep",
            "kFilterTime",
            "kSinglePoleIIRTimeConstant",
            "kSinglePoleIIRExpectedOutput",
            "kHighPassTimeConstant",
            "kHighPassExpectedOutput",
            "kMovAvgTaps",
            "kMovAvgExpectedOutput",
            "kTestSinglePoleIIR",
            "kTestHighPass",
            "kTestMovAvg",
            "kTestPulse",
        ),
    )

    s.sub(
        "wpimath test geometry proto/struct EXPECTED_DATA",
        [
            "wpimath/src/test/native/cpp/geometry/proto/Ellipse2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Pose2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Pose3dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/QuaternionProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Rectangle2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Rotation2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Rotation3dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Transform2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Transform3dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Translation2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Translation3dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Twist2dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/proto/Twist3dProtoTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Ellipse2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Pose2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Pose3dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/QuaternionStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Rectangle2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Rotation2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Rotation3dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Transform2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Transform3dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Translation2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Translation3dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Twist2dStructTest.cpp",
            "wpimath/src/test/native/cpp/geometry/struct/Twist3dStructTest.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )

    s.sub(
        "wpimath test kinematics MecanumDriveKinematicsTest.cpp",
        [
            "wpimath/src/test/native/cpp/kinematics/MecanumDriveKinematicsTest.cpp",
        ],
        normal_substitute(
            ("kFactor", "factor"),
        ),
    )
    s.sub(
        "wpimath test kinematics SwerveDriveKinematicsTest.cpp",
        [
            "wpimath/src/test/native/cpp/kinematics/SwerveDriveKinematicsTest.cpp",
        ],
        normal_substitute(
            ("kFactor", "factor"),
        ),
    )

    s.sub(
        "wpimath test kinematics proto struct EXPECTED_DATA",
        [
            "wpimath/src/test/native/cpp/kinematics/proto/ChassisSpeedsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/DifferentialDriveKinematicsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/DifferentialDriveWheelSpeedsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/MecanumDriveKinematicsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/MecanumDriveWheelPositionsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/MecanumDriveWheelSpeedsProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/SwerveModulePositionProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/proto/SwerveModuleStateProtoTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/ChassisSpeedsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/DifferentialDriveKinematicsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/DifferentialDriveWheelPositionsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/DifferentialDriveWheelSpeedsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/MecanumDriveKinematicsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/MecanumDriveWheelPositionsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/MecanumDriveWheelSpeedsStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/SwerveModulePositionStructTest.cpp",
            "wpimath/src/test/native/cpp/kinematics/struct/SwerveModuleStateStructTest.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )

    s.sub(
        "wpimath test system plant proto struct EXPECTED_DATA",
        [
            "wpimath/src/test/native/cpp/system/plant/proto/DCMotorProtoTest.cpp",
            "wpimath/src/test/native/cpp/system/plant/struct/DCMotorStructTest.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )

    s.sub(
        "wpimath test trajectory DT",
        [
            "wpimath/src/test/native/cpp/trajectory/ExponentialProfileTest.cpp",
            "wpimath/src/test/native/cpp/trajectory/TrapezoidProfileTest.cpp",
        ],
        normal_substitute(
            "kDt",
        ),
    )

    s.sub(
        "wpimath test trajectory proto struct EXPECTED_DATA",
        [
            "wpimath/src/test/native/cpp/trajectory/proto/TrajectoryProtoTest.cpp",
            "wpimath/src/test/native/cpp/trajectory/proto/TrajectoryStateProtoTest.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )

    # wpinet

    s.sub(
        "wpinet HttpParser Type",
        [
            "wpinet/src/main/native/cpp/WebSocket.cpp",
            "wpinet/src/main/native/include/wpinet/HttpParser.h",
            "wpinet/src/main/native/include/wpinet/HttpServerConnection.h",
            "wpinet/src/main/native/include/wpinet/WebSocketServer.h",
            "wpinet/src/test/native/cpp/HttpParserTest.cpp",
            "wpinet/src/test/native/cpp/WebSocketClientTest.cpp",
            "wpinet/src/test/native/cpp/WebSocketServerTest.cpp",
            "wpinet/src/test/native/cpp/WebSocketTest.cpp",
        ],
        normal_substitute(
            "kRequest",
            "kResponse",
            "kBoth",
        ),
    )
    s.sub(
        "wpinet HttpParser state",
        [
            "wpinet/src/main/native/cpp/HttpParser.cpp",
            "wpinet/src/main/native/include/wpinet/HttpParser.h",
        ],
        normal_substitute(
            "kStart",
            "kUrl",
            "kStatus",
            "kField",
            "kValue",
        ),
    )
    s.sub(
        "wpinet HttpMultipartScanner State",
        [
            "wpinet/src/main/native/cpp/HttpUtil.cpp",
            "wpinet/src/main/native/include/wpinet/HttpUtil.h",
        ],
        normal_substitute(
            "kBoundary",
            "kPadding",
            "kDone",
        ),
    )
    s.sub(
        "wpinet HttpMultipartScanner Dashes",
        [
            "wpinet/src/main/native/cpp/HttpUtil.cpp",
            "wpinet/src/main/native/include/wpinet/HttpUtil.h",
        ],
        normal_substitute(
            "kUnknown",
            "kWith",
            "kWithout",
        ),
    )
    s.sub(
        "wpinet NetworkStream Error",
        [
            "wpinet/src/main/native/include/wpinet/NetworkStream.h",
            "wpinet/src/main/native/thirdparty/tcpsockets/cpp/TCPStream.cpp",
        ],
        normal_substitute(
            "kConnectionClosed",
            "kConnectionReset",
            "kConnectionTimedOut",
            "kWouldBlock",
        ),
    )
    s.sub(
        "wpinet WebSocket",
        [
            "wpinet/src/main/native/cpp/WebSocket.cpp",
            "wpinet/src/main/native/cpp/WebSocketSerializer.h",
            "wpinet/src/main/native/include/wpinet/WebSocket.h",
            "wpinet/src/test/native/cpp/WebSocketSerializerTest.cpp",
        ],
        normal_substitute(
            "kOpCont",
            "kOpText",
            "kOpBinary",
            "kOpClose",
            "kOpPing",
            "kOpPong",
            "kOpMask",
            "kFlagFin",
            "kFlagControl",
        ),
    )
    s.sub(
        "wpinet WebSocket Frame",
        [
            "wpinet/src/main/native/include/wpinet/WebSocket.h",
            "wpinet/src/test/native/cpp/WebSocketSerializerTest.cpp",
        ],
        normal_substitute(
            "kText",
            "kBinary",
            "kTextFragment",
            "kBinaryFragment",
            "kFragment",
            "kFinalFragment",
            "kPing",
            "kPong",
        ),
    )

    s.sub(
        "wpinet uv Loop Mode",
        [
            "wpinet/src/main/native/include/wpinet/uv/Loop.h",
        ],
        normal_substitute(
            "kDefault",
            "kOnce",
            "kNoWait",
        ),
    )
    s.sub(
        "wpinet uv NetworkStream",
        [
            "wpinet/src/main/native/include/wpinet/uv/NetworkStream.h",
        ],
        normal_substitute(
            "kDefaultBacklog",
        ),
    )
    s.sub(
        "wpinet uv Process Option Type",
        [
            "wpinet/src/main/native/cpp/uv/Process.cpp",
            "wpinet/src/main/native/include/wpinet/uv/Process.h",
        ],
        normal_substitute(
            "kNone",
            "kArg",
            "kEnv",
            "kCwd",
            "kUid",
            "kGid",
            "kSetFlags",
            "kClearFlags",
            "kStdioIgnore",
            "kStdioInheritFd",
            "kStdioInheritPipe",
            "kStdioCreatePipe",
        ),
    )

    s.sub(
        "wpinet DsClient.cpp",
        [
            "wpinet/src/main/native/cpp/DsClient.cpp",
        ],
        normal_substitute(
            "kReconnectTime",
        ),
    )
    s.sub(
        "wpinet WebSocket.cpp",
        [
            "wpinet/src/main/native/cpp/WebSocket.cpp",
        ],
        normal_substitute(
            "kFlagMasking",
            "kLenMask",
            "kWriteAllocSize",
        ),
    )
    s.sub(
        "wpinet WebSocketSerializer.cpp",
        [
            "wpinet/src/main/native/cpp/WebSocketSerializer.cpp",
        ],
        normal_substitute(
            "kFlagMasking",
            "kWriteAllocSize",
        ),
    )

    # wpiunits

    s.sub(
        "wpiunits LongToObjectHashMap",
        [
            "wpiunits/src/main/java/edu/wpi/first/units/collections/LongToObjectHashMap.java",
        ],
        normal_substitute(
            "kInitialSize",
            "kInitialCapacity",
            "kLoadFactor",
        ),
    )

    # wpiutil

    s.sub(
        "wpiutil c++ MappedFileRegion MapMode",
        [
            "wpiutil/src/main/native/cpp/MappedFileRegion.cpp",
            "wpiutil/src/main/native/include/wpi/MappedFileRegion.h",
            "wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/MemoryBuffer.cpp",
        ],
        normal_substitute(
            "kReadOnly",
            "kReadWrite",
            "kPriv",
        ),
    )
    s.sub(
        "wpiutil PixelFormat",
        [
            "wpiutil/src/main/java/edu/wpi/first/util/PixelFormat.java",
            "wpiutil/src/main/java/edu/wpi/first/util/RawFrame.java",
        ],
        normal_substitute(
            "kUnknown",
            "kMJPEG",
            "kYUYV",
            "kRGB565",
            "kBGR",
            "kGray",
            "kY16",
            "kUYVY",
            "kBGRA",
        ),
    )
    s.sub(
        "wpiutil Synchronization",
        [
            "cscore/src/main/native/cpp/Handle.h",
            "hal/src/main/native/include/hal/handles/HandlesInternal.h",
            "ntcore/src/main/native/cpp/Handle.h",
            "wpiutil/src/main/native/cpp/Synchronization.cpp",
            "wpiutil/src/main/native/include/wpi/Synchronization.h",
        ],
        normal_substitute(
            "kInvalidHandle",
            "kHandleTypeEvent",
            "kHandleTypeSemaphore",
            "kHandleTypeCSBase",
            "kHandleTypeNTBase",
            "kHandleTypeHALBase",
            "kHandleTypeUserBase",
        )
    )
    s.sub(
        "wpiutil TimestampSource",
        [
            # "wpiutil/src/main/java/edu/wpi/first/util/RawFrame.java", # kUnknown -> UNKNOWN is handled by wpiutil PixelFormat
            "wpiutil/src/main/java/edu/wpi/first/util/TimestampSource.java",
        ],
        normal_substitute(
            "kUnknown",
            "kFrameDequeue",
            # NOTE This override seems wrong
            ("kV4LEOF", "VALUE_OF"),
            "kV4LSOE",
        ),
    )

    s.sub(
        "wpiutil sendable SendableBuilder BackendKind",
        [
            "epilogue-runtime/src/main/java/edu/wpi/first/epilogue/logging/LogBackedSendableBuilder.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/NTSendable.java",
            "ntcore/src/main/java/edu/wpi/first/networktables/NTSendableBuilder.java",
            "ntcore/src/main/native/cpp/networktables/NTSendable.cpp",
            "ntcore/src/main/native/cpp/networktables/NTSendableBuilder.cpp",
            "wpiutil/src/main/java/edu/wpi/first/util/sendable/SendableBuilder.java",
            "wpiutil/src/main/native/include/wpi/sendable/SendableBuilder.h",
        ],
        normal_substitute(
            "kUnknown",
            "kNetworkTables",
        ),
    )

    s.sub(
        "wpiutil java struct Struct",
        [
            "epilogue-runtime/src/test/java/edu/wpi/first/epilogue/logging/CustomStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/controller/struct/ArmFeedforwardStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/controller/struct/DifferentialDriveFeedforwardStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/controller/struct/DifferentialDriveWheelVoltagesStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/controller/struct/ElevatorFeedforwardStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/controller/struct/SimpleMotorFeedforwardStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Ellipse2dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/QuaternionStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Rectangle2dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Rotation2dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Translation2dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Translation3dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Twist2dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/geometry/struct/Twist3dStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/ChassisSpeedsStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/DifferentialDriveKinematicsStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/DifferentialDriveWheelPositionsStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/DifferentialDriveWheelSpeedsStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/MecanumDriveWheelPositionsStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/MecanumDriveWheelSpeedsStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/SwerveModulePositionStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/kinematics/struct/SwerveModuleStateStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/spline/struct/CubicHermiteSplineStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/spline/struct/QuinticHermiteSplineStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/struct/MatrixStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/struct/VectorStruct.java",
            "wpimath/src/main/java/edu/wpi/first/math/system/plant/struct/DCMotorStruct.java",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/Struct.java",
        ],
        normal_substitute(
            "kSizeBool",
            "kSizeInt8",
            "kSizeInt16",
            "kSizeInt32",
            "kSizeInt64",
            "kSizeFloat",
            "kSizeDouble",
        ),
    )
    s.sub(
        "wpiutil struct StructFieldType",
        [
            "glass/src/libnt/native/cpp/NetworkTables.cpp",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/DynamicStruct.java",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/StructDescriptor.java",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/StructDescriptorDatabase.java",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/StructFieldType.java",
            "wpiutil/src/main/native/cpp/struct/DynamicStruct.cpp",
            "wpiutil/src/main/native/include/wpi/struct/DynamicStruct.h",
            "wpiutil/src/test/java/edu/wpi/first/util/struct/DynamicStructTest.java",
            "wpiutil/src/test/native/cpp/struct/DynamicStructTest.cpp",
        ],
        normal_substitute(
            "kBool",
            "kChar",
            "kInt8",
            "kInt16",
            "kInt32",
            "kInt64",
            "kUint8",
            "kUint16",
            "kUint32",
            "kUint64",
            "kFloat",
            "kDouble",
            "kStruct",
        ),
    )

    s.sub(
        "wpiutil struct parser Token/TokenKind",
        [
            "wpiutil/src/main/java/edu/wpi/first/util/struct/parser/Lexer.java",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/parser/Parser.java",
            "wpiutil/src/main/java/edu/wpi/first/util/struct/parser/TokenKind.java",
            "wpiutil/src/main/native/cpp/struct/SchemaParser.cpp",
            "wpiutil/src/main/native/include/wpi/struct/SchemaParser.h",
        ],
        normal_substitute(
            "kUnknown",
            "kInteger",
            "kIdentifier",
            "kLeftBracket",
            "kRightBracket",
            "kLeftBrace",
            "kRightBrace",
            "kColon",
            "kSemicolon",
            "kComma",
            "kEquals",
            "kEndOfInput",
        ),
    )

    s.sub(
        "wpiutil proto EXPECTED_DATA",
        [
            "wpiutil/src/test/native/cpp/proto/TestProto.cpp",
            "wpiutil/src/test/native/cpp/proto/TestProtoInner.cpp",
            "wpiutil/src/test/native/cpp/proto/TestProtoRepeated.cpp",
        ],
        normal_substitute(
            "kExpectedData",
        ),
    )

    # xrpVendordep

    s.sub(
        "xrpVendordep XRPOnBoardIO.h",
        [
            "xrpVendordep/src/main/native/include/frc/xrp/XRPOnBoardIO.h",
        ],
        normal_substitute(
            "kMessageInterval",
        ),
    )


def main():
    script_path: Path = Path(__file__).resolve()
    rootdir: Path = script_path.parent

    with Substitutor(rootdir) as substitutor:
        make_substitutions(substitutor)


if __name__ == "__main__":
    main()
