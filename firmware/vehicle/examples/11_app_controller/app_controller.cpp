/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.cpp
 * @brief See app_controller.hpp for what this class is for.
 *        このクラスの目的は app_controller.hpp を参照。
 *
 * @design controller.hpp — IController interface (12 methods)   [OK]
 */

#include "app_controller.hpp"

namespace sf::app {

void AppController::init()
{
    pid_.init();
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);
    return adjust(output, state, setpoint);
}

// @@snippet: adjust
sf::ControlOutput AppController::adjust(
    sf::ControlOutput output,
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint)
{
    // Named constant, not a magic number: at the default value (1.0) this is
    // the identity — PidController's output passes through unchanged. Change
    // kPitchTorqueScale (or replace this line with real control-law code,
    // e.g. your own pitch-rate P term computed from state/setpoint) and
    // rebuild to see the behavior change.
    // マジックナンバーではなく名前付き定数: 既定値（1.0）では恒等 —
    // PidController の出力がそのまま通る。kPitchTorqueScale を変える
    // （または本行を state/setpoint から自作したピッチレート P 項などの
    // 本物の制御則コードに置き換える）と、再ビルドで挙動が変わる。
    constexpr float kPitchTorqueScale = 1.0f;
    output.torque[1] *= kPitchTorqueScale;  // torque[3] = R, P, Y (data_types.hpp)
    return output;
}
// @@end-snippet: adjust

void AppController::reset()
{
    pid_.reset();
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    pid_.onModeChange(new_mode);
}

void AppController::onLanding()
{
    pid_.onLanding();
}

void AppController::onTakeoff()
{
    pid_.onTakeoff();
}

void AppController::onTakeoffComplete()
{
    pid_.onTakeoffComplete();
}

bool AppController::isTakeoffComplete() const
{
    return pid_.isTakeoffComplete();
}

void AppController::setGuidanceTarget(const sf::GuidanceTarget& target,
                                       const sf::CommandSetpoint& current_sticks)
{
    pid_.setGuidanceTarget(target, current_sticks);
}

bool AppController::isGuidanceActive() const
{
    return pid_.isGuidanceActive();
}

void AppController::startExcitation(const sf::SysidCommand& cmd)
{
    pid_.startExcitation(cmd);
}

bool AppController::fetchSysidResult(sf::SysidFreqResult& out)
{
    return pid_.fetchSysidResult(out);
}

void AppController::reloadParams()
{
    pid_.reloadParams();
}

}  // namespace sf::app
