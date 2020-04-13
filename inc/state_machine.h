/**
 * \file state_machine.h
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief State machine and enums definitions
 */

#pragma once

/**
 * \enum StateMachine_e
 * \brief Functional state machine
 */
enum
{
  INIT_LL=0, 
  INIT_HL, 
  WAITING,
  BALANCING,
  DEBUG,
  DEFAULT,
  N_STATES
} StateMachine_e;

/**
 * \enum ControlModes_e
 * \brief Control modes
 */
enum
{
  MANUAL_CTRL=0,
  AUTOMATIC_CTRL,
  N_CTRLS
} ControlModes_e;

/**
 * \enum BalancingStates_e
 * \brief Balancing states
 */
enum
{
  DOWN=0,
  UP
} BalancingStates_e;