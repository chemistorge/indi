/*
    arduino myRotator Driver
    Copyright (C) 2017-2024 Chemistorge

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#pragma once

#include "indirotator.h"


/***************************** arduinoMyRotator Commands **************************/

#define AMR_CMD_LEN 8

//Get Commands
#define AMR_GET_CURRENT_ROTATOR_ANGLE "00#" //uint
#define AMR_GET_MOTOR_STATUS "01#" //uint
#define AMR_GET_FIRMWARE_VERSION "02#"
#define AMR_GET_PROGRAM_NAME_PLUS_FW_VERSION "03#" //string
#define AMR_GET_COIL_PWR_SETTINGS "04#" //bool
#define AMR_GET_REVERSE_DIRECTION_SET "06#" //int
#define AMR_GET_MOTOR_SPEED "11#" //int
#define AMR_GET_MOTORSPEED_DELAY_VALUE "17#" //long
#define AMR_GET_STEPSPERROTATOR_360 "19#" //long
#define AMR_GET_STEPSPERMOTORSHAFT_360 "24#" //int
#define AMR_GET_STEPSPERDEGREE "25#" //float
#define AMR_GET_POSITION_COUNTER "27#" //long
#define AMR_GET_DIRECTION "29#" //int
#define AMR_GET_BACKLASH_CW_ENABLED "30#" //int
#define AMR_GET_BACKLASH_CCW_ENABLED "32#" //int
#define AMR_GET_BACKLASH_CW_STEPS "34#" //int
#define AMR_GET_BACKLASH_CCW_STEPS "36#" //int
#define AMR_GET_STEP_MODE "38#" //int

//set Commands
#define AMR_SET_CURRENT_ROTATOR_ANGLE "12%u#" //int
#define AMR_SET_COIL_PWR_SETTINGS "05%u#" //bool
#define AMR_SET_REVERSE_DIRECTION_SET "07%u#" //int
#define AMR_SET_MOTOR_SPEED "08%u#" //int
#define AMR_SET_MOTORSPEED_DELAY_VALUE "18%ld#" //long
#define AMR_SET_STEPSPERROTATOR_360 "20%ld#" //long
#define AMR_SET_STEPSPERMOTORSHAFT_360 "26%u#" //int
#define AMR_SET_POSITION_COUNTER "28%ld#" //long
#define AMR_SET_BACKLASH_CW_ENABLED "31%u#" //int
#define AMR_SET_BACKLASH_CCW_ENABLED "33%u#" //int
#define AMR_SET_BACKLASH_CW_STEPS "35%u#" //int
#define AMR_SET_BACKLASH_CCW_STEPS "37%u#" //int
#define AMR_SET_STEP_MODE "39%u#" //int

//misc commands

#define AMR_HALT_MOVE "09#"
#define AMR_RESET_MCU "10#"
#define AMR_MOVE_TO_ANGLE_RELATIVE "13%d#"
#define AMR_MOVE_TO_ANGLE_ABSOLUTE "14%d#"
#define AMR_QUERY_MCU "15#"
#define AMR_FIND_HOMING_SENSOR "16#"
#define AMR_MOVE_A_NUMBER_OF_STEPS "21%d#"
#define AMR_MOVE_ONE_MOTOR_REV_PLUS "22#"
#define AMR_MOVE_ONE_MOTOR_REV_NEG "23#"
#define AMR_SAVE_SET_TO_EEPROM "40#"
#define AMR_RESTORE_DEFAULT_SET "41#"

//Responses
#define AMR_QUERY_MCU_RESPONSE "$EOK#"
#define AMR_INT_RESPONSE "$%d#"
#define AMR_LONG_RESPONSE "$%ld#"
#define AMR_STRING_RESPONSE "$%s#"
#define AMR_FLOAT_RESPONSE "$%.2f#"
#define AMR_PROGRAM_PLUS_FW_RESPONSE "$%30[^,],%d#"

#define AMR_RES_LEN 80


/******************************************************************************/


class arduinoMyRotator : public INDI::Rotator
{
    public:
        arduinoMyRotator();
        virtual ~arduinoMyRotator() = default;
        virtual bool Handshake() override;
        const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual void TimerHit() override;

    protected:
        //rotator overrides
        /**
         * @brief MoveRotator Go to specific angle
         * @param angle Target angle in degrees.
         * @return State of operation: IPS_OK is motion is completed, IPS_BUSY if motion in progress, IPS_ALERT on error.
         */
        virtual IPState MoveRotator(double angle) override;

        /**
         * @brief SyncRotator Set current angle as the supplied angle without moving the rotator.
         * @param ticks Desired new angle.
         * @return True if successful, false otherwise.
         */
        virtual bool SyncRotator(double angle) override;

        /**
         * @brief HomeRotator Go to home position.
         * @return State of operation: IPS_OK is motion is completed, IPS_BUSY if motion in progress, IPS_ALERT on error.
         */
        virtual IPState HomeRotator() override;

        /**
         * @brief ReverseRotator Reverse the direction of the rotator. CW is usually the normal direction, and CCW is the reversed direction.
         * @param enable if True, reverse direction. If false, revert to normal direction.
         * @return True if successful, false otherwise.
         */
        virtual bool ReverseRotator(bool enabled) override;

        /**
         * @brief AbortRotator Abort all motion
         * @return True if successful, false otherwise.
         */
        virtual bool AbortRotator() override;

    private:
        bool sendCommand(const char *cmd, char *response);
        bool Ack();
        bool readMainValues();
        bool readMotorValues();
        bool readBaskLashValues();
        bool readOffsetValues();
        bool setInt(int value, const char* mask, const char* errMessage);
        bool setLong(long value, const char* mask, const char* errMessage);

        INDI::PropertyNumber Motor_Setup_PropertiesNP{3}; //MOTOR_SETTTINGS
        enum {
            STEPS_PER_ROTATOR_360,
            FULL_STEPS_PER_MOTOR_360,
            MOTOR_SPEED_DELAY_VALUE,

        };

        INDI::PropertySwitch Motor_SpeedSP{3}; //MAIN_TAB
        enum {
            MOTORSPEED_SLOW,
            MOTORSPEED_MEDIUM,
            MOTORSPEED_FAST
        }; //done

        INDI::PropertySwitch Step_Mode_DRV8825SP{6}; //MOTOR_SETTTINGS
        enum {
            STEP_MODE_DRV_FULL,
            STEP_MODE_DRV_HALF,
            STEP_MODE_DRV_QUARTER,
            STEP_MODE_DRV_EIGHTH,
            STEP_MODE_DRV_SIXTEENTH,
            STEP_MODE_DRV_THIRTYSECOND
        }; //done

        INDI::PropertySwitch Step_Mode_ULN2003SP{2}; //MOTOR_SETTTINGS
        enum {
            STEP_MODE_ULN_FULL,
            STEP_MODE_ULN_HALF
        }; //done

        INDI::PropertyNumber Motor_Constants_ValuesNP{2}; //MOTOR_SETTTINGS
        enum {
            MOTOR_STEPS_PER_DEGREE,
            TOTAL_STEPS_PER_MOTOR_360
        }; //done

        INDI::PropertyNumber Motor_PositionNP{1}; //MOTOR_SETTTINGS
        //done

        INDI::PropertySwitch Current_DirectionSP{2}; //MAIN_TAB
        enum {
            CURRENT_DIRECTION_CCW,
            CURRENT_DIRECTION_CW
        }; //done

        INDI::PropertyNumber Motor_Move_By_StepsNP{1}; //MOTOR_SETTTINGS
        //done

        INDI::PropertySwitch Motor_Coil_PowerSP{2}; //MAIN_TAB
        enum {
            IDLE_COIL_POWER_OFF,
            IDLE_COIL_POWER_ON
        }; //done

        INDI::PropertySwitch Move_Motor_1_RevSP{2}; //MOTOR_SETTTINGS
        enum {           
            MOVE_MOTOR_ONE_REV_CCW,
            MOVE_MOTOR_ONE_REV_CW
        };

        INDI::PropertySwitch BackLashSP{ 2 }; //BACKLASH_TAB
        enum {
            BACKLASH_CCW_SET,
            BACKLASH_CW_SET
        };

        INDI::PropertyNumber BackLashNP{ 2 }; //BACKLASH_TAB
        enum {
            BACKLASH_CCW_STEPS,
            BACKLASH_CW_STEPS
        };

        INDI::PropertySwitch EEPROMSP{ 2 }; //OPTIONS_TAB
        enum {
            RESET_EEPROM,
            SAVE_TO_EEPROM
        };

        INDI::PropertyText FWNameTP{2}; //OPTIONS_TAB
        enum {
            FIRMWARE_NAME,
            FIRMWARE_VERSION
        };


        INDI::PropertySwitch isMovingSP{1}; //MAIN_TAB

        INDI::PropertyNumber relativeMoveNP{1}; //MOTOR_SETTTINGS


};
