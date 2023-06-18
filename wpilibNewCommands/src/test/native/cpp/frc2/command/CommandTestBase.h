// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>

#include <frc/simulation/DriverStationSim.h>
#include <wpi/DenseMap.h>
#include <wpi/SmallVector.h>

#include "frc2/command/CommandHelper.h"
#include "frc2/command/CommandScheduler.h"
#include "frc2/command/SetUtilities.h"
#include "frc2/command/SubsystemBase.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "make_vector.h"

namespace frc2 {

class CommandScheduler::Impl {
 public:
  // A set of the currently-running commands.
  wpi::SmallSet<Command*, 12> scheduledCommands;

  // A map from required subsystems to their requiring commands.  Also used as a
  // set of the currently-required subsystems.
  wpi::DenseMap<Subsystem*, Command*> requirements;

  // A map from subsystems registered with the scheduler to their default
  // commands.  Also used as a list of currently-registered subsystems.
  wpi::DenseMap<Subsystem*, std::unique_ptr<Command>> subsystems;

  frc::EventLoop defaultButtonLoop;
  // The set of currently-registered buttons that will be polled every
  // iteration.
  frc::EventLoop* activeButtonLoop{&defaultButtonLoop};

  bool disabled{false};

  // Lists of user-supplied actions to be executed on scheduling events for
  // every command.
  wpi::SmallVector<Action, 4> initActions;
  wpi::SmallVector<Action, 4> executeActions;
  wpi::SmallVector<Action, 4> interruptActions;
  wpi::SmallVector<Action, 4> finishActions;

  // Flag and queues for avoiding concurrent modification if commands are
  // scheduled/canceled during run

  bool inRunLoop = false;
  wpi::SmallVector<Command*, 4> toSchedule;
  wpi::SmallVector<Command*, 4> toCancel;
};

class TestSubsystem : public SubsystemBase {};

/**
 * NOTE: Moving mock objects causes EXPECT_CALL to not work correctly!
 */
class MockCommand : public CommandHelper<CommandBase, MockCommand> {
 public:
  MOCK_CONST_METHOD0(GetRequirements, wpi::SmallSet<Subsystem*, 4>());
  MOCK_METHOD0(IsFinished, bool());
  MOCK_CONST_METHOD0(RunsWhenDisabled, bool());
  MOCK_METHOD0(Initialize, void());
  MOCK_METHOD0(Execute, void());
  MOCK_METHOD1(End, void(bool interrupted));

  MockCommand() {
    m_requirements = {};
    EXPECT_CALL(*this, GetRequirements())
        .WillRepeatedly(::testing::Return(m_requirements));
    EXPECT_CALL(*this, IsFinished()).WillRepeatedly(::testing::Return(false));
    EXPECT_CALL(*this, RunsWhenDisabled())
        .WillRepeatedly(::testing::Return(true));
  }

  MockCommand(std::initializer_list<Subsystem*> requirements,
              bool finished = false, bool runWhenDisabled = true) {
    m_requirements.insert(requirements.begin(), requirements.end());
    EXPECT_CALL(*this, GetRequirements())
        .WillRepeatedly(::testing::Return(m_requirements));
    EXPECT_CALL(*this, IsFinished())
        .WillRepeatedly(::testing::Return(finished));
    EXPECT_CALL(*this, RunsWhenDisabled())
        .WillRepeatedly(::testing::Return(runWhenDisabled));
  }

  MockCommand(MockCommand&& other) {
    EXPECT_CALL(*this, IsFinished())
        .WillRepeatedly(::testing::Return(other.IsFinished()));
    EXPECT_CALL(*this, RunsWhenDisabled())
        .WillRepeatedly(::testing::Return(other.RunsWhenDisabled()));
    std::swap(m_requirements, other.m_requirements);
    EXPECT_CALL(*this, GetRequirements())
        .WillRepeatedly(::testing::Return(m_requirements));
  }

  MockCommand(const MockCommand& other) : CommandHelper{other} {}

  void SetFinished(bool finished) {
    EXPECT_CALL(*this, IsFinished())
        .WillRepeatedly(::testing::Return(finished));
  }

  ~MockCommand() {  // NOLINT
    auto& scheduler = CommandScheduler::GetInstance();
    scheduler.Cancel(this);
  }

 private:
  wpi::SmallSet<Subsystem*, 4> m_requirements;
};

class CommandTestBase : public ::testing::Test {
 public:
  CommandTestBase();

 protected:
  CommandScheduler GetScheduler();

  void SetUp() override;

  void TearDown() override;

  void SetDSEnabled(bool enabled);
};

template <typename T>
class CommandTestBaseWithParam : public ::testing::TestWithParam<T> {
 public:
  CommandTestBaseWithParam() {
    auto& scheduler = CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.Enable();
    scheduler.GetActiveButtonLoop()->Clear();
  }

 protected:
  CommandScheduler GetScheduler() { return CommandScheduler(); }

  std::unique_ptr<CommandScheduler::Impl> GetSchedulerImpl(
      CommandScheduler& sched) {
    return std::move(sched.m_impl);
  }

  void SetSchedulerImpl(CommandScheduler& sched,
                        std::unique_ptr<CommandScheduler::Impl>&& impl) {
    sched.m_impl = std::move(impl);
  }

  void SetUp() override { frc::sim::DriverStationSim::SetEnabled(true); }

  void TearDown() override {
    CommandScheduler::GetInstance().GetActiveButtonLoop()->Clear();
  }

  void SetDSEnabled(bool enabled) {
    frc::sim::DriverStationSim::SetEnabled(enabled);
  }
};

}  // namespace frc2
