/*
    myDewControllerPro Driver
    Copyright (C) 2017-2023 Chemistorge

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

#include "arduinoMyRotator.h"
#include <string.h>
#include "indicom.h"
#include "connectionplugins/connectionserial.h"
#include <termios.h>

#define MYDEWHEATERPRO_TIMEOUT 3
#define MOTOR_SETTINGS_TAB "Motor Settings"

std::unique_ptr<arduinoMyRotator> arduinomyrotator(new arduinoMyRotator());


arduinoMyRotator::arduinoMyRotator()
{
    setVersion(1, 0);
    
    RI::SetCapability(ROTATOR_CAN_ABORT |
                     ROTATOR_CAN_REVERSE |
                     ROTATOR_CAN_SYNC |
                     ROTATOR_CAN_HOME);
    setRotatorConnection(CONNECTION_SERIAL);
}

bool arduinoMyRotator::initProperties()
{
    INDI::Rotator::initProperties();




    Motor_Setup_PropertiesNP[STEPS_PER_ROTATOR_360].fill("ROTATOR_360", "Steps Per Rotator 360", "%4.0f", 1000., 500000., 1., 0.);
    Motor_Setup_PropertiesNP[FULL_STEPS_PER_MOTOR_360].fill("FULL_STEPS_MOTOR_360", "Full Steps Per Motor 360", "%d", 20, 3000, 1, 0);
    Motor_Setup_PropertiesNP[MOTOR_SPEED_DELAY_VALUE].fill("SPEED_DELAY", "Motor Step Delay", "%4.0f", 3000., 15000., 1., 0.);
    Motor_Setup_PropertiesNP.fill(getDeviceName(), "MOTOR_SETTINGS_VALUES", "Motor Values", MOTOR_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    Motor_Constants_ValuesNP[MOTOR_STEPS_PER_DEGREE].fill("STEPS_PER_DEGREE", "Motor Steps Per Degree", "%4.0f", 0., 500., 1., 0.);
    Motor_Constants_ValuesNP[TOTAL_STEPS_PER_MOTOR_360].fill("STEPS_PER_MOTOR_360", "Steps Per Motor 360", "%4.0f", 50., 10000., 1., 0.);
    Motor_Constants_ValuesNP.fill(getDeviceName(), "Motor Constants", "Motor Constants", MOTOR_SETTINGS_TAB, IP_RO, 0, IPS_IDLE);

    Step_Mode_DRV8825SP[STEP_MODE_DRV_FULL].fill("MODE_FULL", "Full Step", ISS_OFF);
    Step_Mode_DRV8825SP[STEP_MODE_DRV_HALF].fill("MODE_HALF", "1/2 Step", ISS_ON);
    Step_Mode_DRV8825SP[STEP_MODE_DRV_QUARTER].fill("MODE_QUARTER", "1/4 Step", ISS_OFF);
    Step_Mode_DRV8825SP[STEP_MODE_DRV_EIGHTH].fill("MODE_EIGHTH", "1/8 Step", ISS_OFF);
    Step_Mode_DRV8825SP[STEP_MODE_DRV_SIXTEENTH].fill("MODE_SIXTEENTH", "1/16 Step", ISS_OFF);
    Step_Mode_DRV8825SP[STEP_MODE_DRV_THIRTYSECOND].fill("MODE_THIRTYSECOND", "1/32 Step", ISS_OFF);
    Step_Mode_DRV8825SP.fill(getDeviceName(), "Step_Mode", "Step Mode", MOTOR_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    Step_Mode_ULN2003SP[STEP_MODE_ULN_FULL].fill("MODE_FULL", "Full Step", ISS_OFF);
    Step_Mode_ULN2003SP[STEP_MODE_ULN_HALF].fill("MODE_HALF", "1/2 Step", ISS_ON);
    Step_Mode_ULN2003SP.fill(getDeviceName(), "Step_Mode", "Step Mode", MOTOR_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    Motor_SpeedSP[MOTORSPEED_SLOW].fill("SLOW_SPEED", "Slow", ISS_OFF);
    Motor_SpeedSP[MOTORSPEED_MEDIUM].fill("MEDIUM_SPEED", "Medium", ISS_OFF);
    Motor_SpeedSP[MOTORSPEED_FAST].fill("FAST_SPEED", "Fast", ISS_ON);
    Motor_SpeedSP.fill(getDeviceName(), "MOTOR_SPEED", "Motor Speed", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    EEPROMSP[RESET_EEPROM].fill("Reset EEPROM", "Reset EEPROM to Defaults", ISS_OFF);
    EEPROMSP[SAVE_TO_EEPROM].fill("Save to EEPROM", "Save to EEPROM", ISS_OFF);
    EEPROMSP.fill(getDeviceName(), "EEPROM", "EEPROM", OPTIONS_TAB, IP_WO, ISR_ATMOST1, 0, IPS_IDLE);

    Motor_PositionNP[0].fill("MOTOR_POSITION", "Steps", "%4.0f", -100000., 100000., 1., 0.);
    Motor_PositionNP.fill(getDeviceName(), "MOTOR_POSITION_STEPS", "Motor Position", MOTOR_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    Motor_Move_By_StepsNP[0].fill("MOVE_MOTOR_BY_STEPS", "Steps", "%4.0f", -30000., 30000., 1., 0.);
    Motor_Move_By_StepsNP.fill(getDeviceName(), "MOTOR_MOVE_BY_STEPS", "Move Motor", MOTOR_SETTINGS_TAB, IP_WO, 0, IPS_IDLE);

    Current_DirectionSP[CURRENT_DIRECTION_CCW].fill("DIRECTION_CCW", "CCW", ISS_OFF);
    Current_DirectionSP[CURRENT_DIRECTION_CW].fill("DIRECTION_CW", "CW", ISS_OFF);
    Current_DirectionSP.fill(getDeviceName(), "CURRENT TRAVEL DIRECTION", "Travel Direction", MAIN_CONTROL_TAB, IP_WO, ISR_ATMOST1, 0, IPS_IDLE);

    Motor_Coil_PowerSP[IDLE_COIL_POWER_OFF].fill("IDLE_POWER_OFF", "Off", ISS_OFF);
    Motor_Coil_PowerSP[IDLE_COIL_POWER_ON].fill("IDLE_POWER_ON", "On", ISS_OFF);
    Motor_Coil_PowerSP.fill(getDeviceName(), "IDLE_COIL_POWER_SETTING", "Idle Coil Power", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    Move_Motor_1_RevSP[MOVE_MOTOR_ONE_REV_CCW].fill("MOVE_CCW_1_REV", "CCW 1 Revolution", ISS_OFF);
    Move_Motor_1_RevSP[MOVE_MOTOR_ONE_REV_CW].fill("MOVE_CW_1_REV", "CW 1 Revolution", ISS_OFF);
    Move_Motor_1_RevSP.fill(getDeviceName(), "MOVE_MOTOR_1_REVOLUTION", "Move Motor", MOTOR_SETTINGS_TAB, IP_WO, ISR_ATMOST1, 0, IPS_IDLE);

    BackLashSP[BACKLASH_CCW_SET].fill("BACKLASH_SET_CCW", "CCW", ISS_OFF);
    BackLashSP[BACKLASH_CW_SET].fill("BACKLASH_SET_CW", "CW", ISS_OFF);
    BackLashSP.fill(getDeviceName(), "ENABLE_BACKLASH", "Enable Backlash", MOTOR_SETTINGS_TAB, IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    //add backlash number set back lash tab
    BackLashNP[BACKLASH_CCW_STEPS].fill("CCW_STEPS", "CCW Steps", "%4.0f", 0., 400., 1., 0.);
    BackLashNP[BACKLASH_CW_STEPS].fill("CW_STEPS", "CW Steps", "%4.0f", 0., 400., 1., 0.);
    BackLashNP.fill(getDeviceName(), "SET_BACKLASH_STEPS", "Set Backlash Steps", MOTOR_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    //add FW Name
    FWNameTP[FIRMWARE_NAME].fill("FW_NAME", "Name", "");
    FWNameTP[FIRMWARE_VERSION].fill("FW_NAME", "Version", "");
    FWNameTP.fill(getDeviceName(), "Firmware", "", OPTIONS_TAB, IP_RO, 60, IPS_IDLE);

    //add light is moving
    isMovingSP[0].fill("MOTOR_MOVING", "Motor Moving", ISS_OFF);
    isMovingSP.fill(getDeviceName(), "ROTATOR_STATUS", "Rotator Status", MAIN_CONTROL_TAB, IP_RO, ISR_ATMOST1, 0, IPS_IDLE);

    //add backlash number set back lash tab
    relativeMoveNP[BACKLASH_CCW_STEPS].fill("Degrees", "Angle", "%4.0f", -180., 180., 1., 0.);
    relativeMoveNP.fill(getDeviceName(), "Relative Move", "Relative Move", MAIN_CONTROL_TAB, IP_WO, 0, IPS_IDLE);

    setDriverInterface(ROTATOR_INTERFACE);


    setDefaultPollingPeriod(2000);
    serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);


    // No simulation control for now


    return true;
}


bool arduinoMyRotator::updateProperties()
{
     INDI::Rotator::updateProperties();

    if (isConnected())
    {
        
        //MAIN_TAB
        defineProperty(relativeMoveNP);
        defineProperty(Current_DirectionSP);
        defineProperty(isMovingSP);
        defineProperty(Motor_Coil_PowerSP);
        defineProperty(Motor_SpeedSP);

        //MOTOR_SETTINGS_TAB
        defineProperty(Motor_Setup_PropertiesNP);
        char resp[AMR_RES_LEN] = {};
        if (!sendCommand(AMR_GET_PROGRAM_NAME_PLUS_FW_VERSION, resp)) {
            LOG_DEBUG("Error Fetching FW name");

            return false;
        }
        //LOG_INFO(resp);
        std::string result = resp;
        int startofString = result.find("-")+1;
        int endofString = result.find(",");
        std::string name = result.substr(startofString, endofString-startofString);

        if (name == "ULN2003") {
            defineProperty(Step_Mode_ULN2003SP);
        } else if (name == "DRV8825") {
            defineProperty(Step_Mode_DRV8825SP);
        }


        defineProperty(Motor_Constants_ValuesNP);
        defineProperty(Motor_PositionNP);
        defineProperty(Motor_Move_By_StepsNP);
        defineProperty(Move_Motor_1_RevSP);

        //Options Tab
        defineProperty(FWNameTP);
        defineProperty(EEPROMSP);

        //Backlash tab
        defineProperty(BackLashSP);
        defineProperty(BackLashNP);




        loadConfig(true);
        if (!readMainValues()) {
            LOG_INFO("Reading Main Values Error");
        }
//        if (!readLCDDisplayValues()) {
//            LOG_INFO("Reading LCD Display Values Error");
//        }
//        if (!readBoardFanValues()) {
//            LOG_INFO("Reading Board Fan Values Error");
//        }
//        if (!readOffsetValues()) {
//            LOG_INFO("Reading Offset Values Error");
//        }
        LOG_INFO("arduino myRotator parameters updated, device ready for use.");
        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(relativeMoveNP);
        deleteProperty(Motor_Setup_PropertiesNP);
        deleteProperty(Motor_SpeedSP);
        deleteProperty(Step_Mode_DRV8825SP);
        deleteProperty(Step_Mode_ULN2003SP);
        deleteProperty(Motor_Constants_ValuesNP);
        deleteProperty(Motor_PositionNP);
        deleteProperty(Current_DirectionSP);
        deleteProperty(Motor_Move_By_StepsNP);
        deleteProperty(Motor_Coil_PowerSP);
        deleteProperty(Move_Motor_1_RevSP);
        deleteProperty(BackLashSP);
        deleteProperty(BackLashNP);
        deleteProperty(FWNameTP);
        deleteProperty(isMovingSP);
        deleteProperty(EEPROMSP);

    }

    return true;
}

const char *arduinoMyRotator::getDefaultName()
{
    return "arduinoMyRotator";
}

bool arduinoMyRotator::sendCommand(const char *cmd, char *resp)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    char errstr[MAXRBUF];
    LOGF_DEBUG("CMD: %s.", cmd);

    tcflush(PortFD, TCIOFLUSH);
    if ((rc = tty_write(PortFD, cmd, strlen(cmd), &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Error writing command %s: %s. %d", cmd, errstr, rc);
        return false;
    }

    if (resp)
    {
        if ((rc = tty_nread_section(PortFD, resp, AMR_RES_LEN, '#', MYDEWHEATERPRO_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(rc, errstr, MAXRBUF);
            LOGF_ERROR("Error reading response for command %s: %s.", cmd, errstr);
            return false;
        }
    }
    return true;
}



bool arduinoMyRotator::Handshake()
{
    PortFD = serialConnection->getPortFD();
    if (PortFD < 0) {
        LOG_INFO("Invalid Port");
        return false;
    }

    int tries = 3;
    do
    {
        if (Ack())
        {
            LOG_INFO("arduino myRotator is online. Getting device parameters...");
            return true;
        }
        LOG_INFO("Error retrieving data from arduino myRotator, trying resync...");
    }
    while (--tries > 0);

    //LOG_INFO("Error retrieving data from arduino myRotator, please ensure controller "
    //         "is powered and the port is correct.");
    return false;
}

bool arduinoMyRotator::Ack()
{
    char resp[AMR_RES_LEN] = {};
    tcflush(PortFD, TCIOFLUSH);

    if (!sendCommand(AMR_GET_PROGRAM_NAME_PLUS_FW_VERSION, resp))
        return false;

    int firmware = -1;
    char fwname[30] = {};
    char progName[20] = {};
    int ok = sscanf(resp, AMR_PROGRAM_PLUS_FW_RESPONSE, fwname, &firmware);
    snprintf(resp, 40, "Firmware Version: %d", firmware);
    //LOG_INFO(resp);
    //LOG_INFO(fwname);
    strncpy(progName, fwname, 9);

    if (ok != 2)
    {
        LOGF_ERROR("arduino myRotator not properly identified! Answer was: %s.", resp);
        return false;
    }
    if (strcmp(progName, "myRotator") != 0) {
        LOG_INFO(progName);
        LOG_INFO("Name not Correct");
        return false;
    }
    if (firmware < 125) {
        LOG_INFO("Please update arduino myRotator firmware");
        LOG_INFO("https://sourceforge.net/projects/arduino-myrotator/");
        return false;
    }


    FWNameTP[FIRMWARE_NAME].setText(fwname);
    FWNameTP[FIRMWARE_VERSION].setText(std::to_string(firmware));
    FWNameTP.setState(IPS_OK);
    FWNameTP.apply();

    return true;

}

IPState arduinoMyRotator::MoveRotator(double angle) {
    if (rotatorMoving) {
        return IPS_ALERT;
        LOG_INFO("Rotator is movng, please wait");
    }
    char cmd[AMR_CMD_LEN + 1];
    int degree = angle;
    rotatorMoving = true;
    snprintf(cmd, AMR_CMD_LEN + 1, AMR_MOVE_TO_ANGLE_ABSOLUTE, degree);
    //LOG_INFO(cmd);
    
    return sendCommand(cmd, nullptr) ? IPS_BUSY : IPS_ALERT;
    
}


bool arduinoMyRotator::SyncRotator(double angle) {

    if (rotatorMoving) {
        return false;
        LOG_INFO("Rotator is movng, please wait");
    }

    char cmd[AMR_CMD_LEN + 1];
    int degree = angle;

    snprintf(cmd, AMR_CMD_LEN + 1, AMR_SET_CURRENT_ROTATOR_ANGLE, degree);
    if (sendCommand(cmd, nullptr)) {
        readMainValues():
        return true;
    } else {
        LOGF_DEBUG("Command not sent $S", cmd);
        return false;
    }
}

IPState arduinoMyRotator::HomeRotator() {
    if (rotatorMoving) {
        return IPS_ALERT;
        LOG_INFO("Rotator is movng, please wait");
    }
   
    if (sendCommand(AMR_FIND_HOMING_SENSOR, nullptr)) {
        rotatorMoving = true;
        return IPS_BUSY;
    } else {
        LOGF_DEBUG("Command not sent $S", cmd);
        return IPS_ALERT;
    }

    
}

bool arduinoMyRotator::ReverseRotator(bool enabled) {
    int set = enabled? 1 : 0;
    char cmd[AMR_CMD_LEN + 1];
    snprintf(cmd, AMR_CMD_LEN + 1, AMR_SET_REVERSE_DIRECTION_SET, degree);
    if (sendCommand(cmd, nullptr)) {
        readMainValues():
        return true;
    } else {
        LOGF_DEBUG("Command not sent $S", cmd);
        return false;
    }
}

bool arduinoMyRotator::AbortRotator() {
    if (sendCommand(AMR_HALT_MOVE, nullptr)) {
        readMainValues():
        return true;
    } else {
        LOGF_DEBUG("Command not sent $S", cmd);
        return false;
    }
}


//bool myDewControllerPro::setOutputBoost(unsigned int channel)
//{



//    if (channel == 0) {
//        return sendCommand(MDCP_BOOST_CH1, nullptr);
//    } else if (channel == 1) {
//        return sendCommand(MDCP_BOOST_CH2, nullptr);
//    } else {
//        LOG_INFO("No Channel Set");
//        return false;
//    }

//}

//bool myDewControllerPro::setInt(int mode, const char *mask, const char *errMessage)
//{
//    char cmd[MDCP_CMD_LEN + 1];

//    snprintf(cmd, MDCP_CMD_LEN + 1, mask, mode);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO(errMessage);
//        LOG_INFO(cmd);
//        return false;
//    }
//    return true;

//}

//bool myDewControllerPro::setChoice(int testInt, const char *positiveChoice, const char *negativeChoice, const char *errMessage)
//{
//    const char* mask = testInt == 1 ? positiveChoice : negativeChoice;
//    if (!sendCommand(mask, nullptr)) {
//        LOG_INFO(errMessage);

//        return false;
//    }
//    return true;
//}



//bool myDewControllerPro::setTempCalibrations(float ch1, float ch2, float ch3, int ambient)
//{
//    char cmd[MDCP_CMD_LEN + 1];


//    snprintf(cmd, MDCP_CMD_LEN + 1, MDCP_SET_TEMP_CH1_OFFSET, ch1);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO("Failed to set CH1 offset");
//        LOG_INFO(cmd);
//        return false;
//    }
//    snprintf(cmd, MDCP_CMD_LEN + 1, MDCP_SET_TEMP_CH2_OFFSET, ch2);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO("Failed to set CH2 offset");
//        LOG_INFO(cmd);
//        return false;
//    }
//    snprintf(cmd, MDCP_CMD_LEN + 1, MDCP_SET_TEMP_CH3_OFFSET, ch3);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO("Failed to set CH3 offset");
//        LOG_INFO(cmd);
//        return false;
//    }

//    snprintf(cmd, MDCP_CMD_LEN + 1, MDCP_SET_AMB_TEMP_OFFSET, ambient);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO("Failed to set CH3 offset");
//        LOG_INFO(cmd);
//        return false;
//    }
//    return true;



//}

//bool myDewControllerPro::setFanTempTrigger(int tempOn, int tempOff)
//{
//    char cmd[MDCP_CMD_LEN + 1];


//    snprintf(cmd, MDCP_CMD_LEN + 1, MDCP_SET_FAN_ON_TEMP, tempOn);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO("Failed to set temp on");
//        LOG_INFO(cmd);
//        return false;
//    }
//    snprintf(cmd, MDCP_CMD_LEN + 1, MDCP_SET_FAN_OFF_TEMP, tempOff);
//    if (!sendCommand(cmd, nullptr)) {
//        LOG_INFO("Failed to set CH2 offset");
//        LOG_INFO(cmd);
//        return false;
//    }

//    return true;



//}

//bool myDewControllerPro::zeroTempCalibrations() {

//    if (!sendCommand(MDCP_CLEAR_TEMP_OFFSETS, nullptr)) {
//        LOG_INFO("Failed to zero temp offset");

//        return false;
//    }
//    if (!sendCommand("e0#", nullptr)) {
//        LOG_INFO("Failed to zero ambtemp offset");

//        return false;
//    }
//    return true;


//}








bool arduinoMyRotator::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
//    if (!dev || strcmp(dev, getDeviceName()))
//        return false;

//    if (CH1CH2BoostSP.isNameMatch(name))
//    {
//        CH1CH2BoostSP.update( states, names, n);
//        CH1CH2BoostSP.setState(IPS_BUSY);
//        cancelOutputBoost();
//        if (CH1CH2BoostSP[CH1_BOOST_100].getState() == ISS_ON) {
//            setOutputBoost(CH1_BOOST_100);
//        }
//        if (CH1CH2BoostSP[CH2_BOOST_100].getState() == ISS_ON) {
//            setOutputBoost(CH2_BOOST_100);
//        }
//        CH1CH2BoostSP.setState(IPS_OK);
//        CH1CH2BoostSP.apply();
//        readMainValues();
//        return true;

//    }

//    if (CH3_ModeSP.isNameMatch(name))
//    {
//        CH3_ModeSP.update(states, names, n);
//        CH3_ModeSP.setState(IPS_BUSY);
//        int mode = CH3_ModeSP.findOnSwitchIndex();
//        setInt(mode, MDCP_SET_CH3_SETTINGS, "Failed to set CH3 mode");
//        CH3_ModeSP.setState(IPS_OK);
//        CH3_ModeSP.apply();
//        readMainValues();
//        return true;

//    }

//    if (ZeroTempOffsetsSP.isNameMatch(name))
//    {
//        ZeroTempOffsetsSP.update(states, names, n);
//        ZeroTempOffsetsSP.setState(IPS_BUSY);
//        zeroTempCalibrations();
//        ZeroTempOffsetsSP.setState(IPS_OK);
//        ZeroTempOffsetsSP[0].setState(ISS_OFF);
//        ZeroTempOffsetsSP.apply();
//        readOffsetValues();
//        return true;
//    }

//    if (TrackingModeSP.isNameMatch(name))
//    {
//        TrackingModeSP.update(states, names, n);
//        TrackingModeSP.setState(IPS_BUSY);
//        int mode = TrackingModeSP.findOnSwitchIndex();
//        setInt(mode, MDCP_SET_TRACKING_MODE, "Failed to set Tracking Mode");
//        TrackingModeSP.setState(IPS_OK);
//        TrackingModeSP.apply();
//        readOffsetValues();
//        return true;
//    }

//    if (FanModeSP.isNameMatch(name))
//    {
//        FanModeSP.update(states, names, n);
//        FanModeSP.setState(IPS_BUSY);
//        int mode = FanModeSP.findOnSwitchIndex();
//        setInt(mode, MDCP_SET_FAN_MODE, "Failed to set Fan Mode");
//        FanModeSP.setState(IPS_OK);
//        FanModeSP.apply();
//        readBoardFanValues();
//        return true;
//    }

//    if (EnableLCDDisplaySP.isNameMatch(name))
//    {
//        EnableLCDDisplaySP.update(states, names, n);
//        EnableLCDDisplaySP.setState(IPS_BUSY);
//        int mode = EnableLCDDisplaySP.findOnSwitchIndex();
//        setChoice(mode, MDCP_LCD_ENABLE, MDCP_LCD_DISABLE, "Failed to set LCD enable");
//        EnableLCDDisplaySP.setState(IPS_OK);
//        EnableLCDDisplaySP.apply();
//        readLCDDisplayValues();
//        return true;

//    }

//    if (LCDDisplayTempUnitsSP.isNameMatch(name))
//    {
//        LCDDisplayTempUnitsSP.update(states, names, n);
//        LCDDisplayTempUnitsSP.setState(IPS_BUSY);
//        int mode = LCDDisplayTempUnitsSP.findOnSwitchIndex();
//        setChoice(mode, MDCP_LCD_DISPLAY_FAHRENHEIT, MDCP_LCD_DISPLAY_CELSIUS, "Failed to set temp display mode");
//        LCDDisplayTempUnitsSP.setState(IPS_OK);
//        LCDDisplayTempUnitsSP.apply();
//        readLCDDisplayValues();
//        return true;

//    }

//    if (EEPROMSP.isNameMatch(name))
//    {
//        EEPROMSP.update(states, names, n);
//        EEPROMSP.setState(IPS_BUSY);
//        int mode = EEPROMSP.findOnSwitchIndex();
//        if (setChoice(mode, MDCP_SAVE_TO_EEPROM, MDCP_RESET_EEPROM_TO_DEFAULT, "Failed to Save/reset EEPROM"))
//        {
//            const char* message = mode == 1 ? "Saved to EEPPROM Successfully" : "Reset EEPROM to Default";
//            LOG_INFO(message);
//        }
//        readMainValues();
//        readOffsetValues();
//        readBoardFanValues();
//        readLCDDisplayValues();
//        EEPROMSP.setState(IPS_OK);
//        EEPROMSP.apply();
//        return true;

//    }
    return INDI::Rotator::ISNewSwitch(dev, name, states, names, n);
}

bool arduinoMyRotator::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
//    if (!dev || strcmp(dev, getDeviceName()))
//        return false;

//    if (CH3_Manual_PowerNP.isNameMatch(name))
//    {
//        if (CH3_ModeSP.findOnSwitchIndex() == 3) {
//            CH3_Manual_PowerNP.update(values, names, n);
//            CH3_Manual_PowerNP.setState(IPS_BUSY);
//            int power = CH3_Manual_PowerNP[0].getValue();
//            setInt(power, MDCP_SET_CH3_MANUAL_POWER, "Failed to set CH3 Power");
//            CH3_Manual_PowerNP.setState(IPS_OK);
//            CH3_Manual_PowerNP.apply();
//        } else {
//            LOG_INFO("Power can only be manually adjusted in Strap 3 manual mode");
//        }
//        readMainValues();
//        return true;
//    }

//    if (TemperatureOffsetsNP.isNameMatch(name))
//    {
//        TemperatureOffsetsNP.update(values, names, n);
//        TemperatureOffsetsNP.setState(IPS_BUSY);
//        int ch1 = TemperatureOffsetsNP[TEMP_PROBE_ONE_OFFSET].getValue();
//        int ch2 = TemperatureOffsetsNP[TEMP_PROBE_TWO_OFFSET].getValue();
//        int ch3 = TemperatureOffsetsNP[TEMP_PROBE_THREE_OFFSET].getValue();
//        int ambient = TemperatureOffsetsNP[AMBIENT_TEMP_PROBE_OFFSET].getValue();
//        setTempCalibrations(ch1, ch2, ch3, ambient);
//        TemperatureOffsetsNP.setState(IPS_OK);
//        TemperatureOffsetsNP.apply();
//        readOffsetValues();
//        return true;

//    }

//    if (TrackingModeOffsetNP.isNameMatch(name))
//    {
//        TrackingModeOffsetNP.update(values, names, n);
//        TrackingModeOffsetNP.setState(IPS_BUSY);
//        int offset = TrackingModeOffsetNP[0].getValue();
//        setInt(offset, MDCP_SET_TRACKING_MODE_OFFSET, "Failed to set Tracking Mode offsets");
//        TrackingModeOffsetNP.setState(IPS_OK);
//        TrackingModeOffsetNP.apply();
//        readOffsetValues();
//        return true;
//    }

//    if (FanTempTriggerNP.isNameMatch(name))
//    {
//        FanTempTriggerNP.update(values, names, n);
//        FanTempTriggerNP.setState(IPS_BUSY);
//        int tempOn = FanTempTriggerNP[FANTEMPON].getValue();
//        int tempOff = FanTempTriggerNP[FANTEMPOFF].getValue();
//        setFanTempTrigger(tempOn, tempOff);
//        FanTempTriggerNP.setState(IPS_OK);
//        FanTempTriggerNP.apply();
//        readBoardFanValues();
//        return true;

//    }
//    if (FanSpeedNP.isNameMatch(name))
//    {
//        FanSpeedNP.update(values, names, n);
//        FanSpeedNP.setState(IPS_BUSY);
//        int speed = FanSpeedNP[0].getValue();
//        setInt(speed, MDCP_SET_FAN_SPEED, "Failed to set Fan Speed");
//        FanSpeedNP.setState(IPS_OK);
//        FanSpeedNP.apply();
//        readBoardFanValues();
//        return true;

//    }

//    if (LCDPageRefreshNP.isNameMatch(name)) {
//        LCDPageRefreshNP.update(values, names, n);
//        LCDPageRefreshNP.setState(IPS_BUSY);
//        int time = LCDPageRefreshNP[0].getValue();
//        setInt(time, MDCP_SET_LCD_DISPLAY_TIME, "Failed to set LCD Page refressh");
//        LCDPageRefreshNP.setState(IPS_OK);
//        LCDPageRefreshNP.apply();
//        readLCDDisplayValues();
//        return true;

//    }

   return INDI::Rotator::ISNewNumber(dev, name, values, names, n);
}

bool arduinoMyRotator::readMainValues()
{

    char resp[AMR_RES_LEN];
    int temp;


    if (!sendCommand(AMR_GET_CURRENT_ROTATOR_ANGLE, resp)) {
        LOG_INFO(resp);
        return false;
    }

    if (sscanf(resp, AMR_INT_RESPONSE, &temp) == 1) {
        SyncRotatorN[0].value = temp;
        GotoRotatorN[0].value = temp;
        SyncRotatorNP.s = IPS_OK;
        GotoRotatorNP.s = IPS_OK;
        IDSetNumber(&SyncRotatorNP, nullptr);
        IDSetNumber(&GotoRotatorNP, nullptr);
    }
    temp = 0;

    if (!sendCommand(AMR_GET_DIRECTION, resp)) {
        LOG_INFO(resp);
        return false;
    }

    if (sscanf(resp, AMR_INT_RESPONSE, &temp) == 1) {
        Current_DirectionSP.reset();
        Current_DirectionSP[temp].setState(ISS_ON);
        Current_DirectionSP.setState(IPS_OK);
        Current_DirectionSP.apply();
    }

    if (!sendCommand(AMR_GET_MOTOR_STATUS, resp)) {
        LOG_INFO(resp);
        return false;
    }

    if (sscanf(resp, AMR_INT_RESPONSE, &temp) == 1) {
        isMovingSP[0].setState(temp == 1? ISS_ON : ISS_OFF);
        rotatorMoving = temp == 1 ? true : false;
        isMovingSP.setState(IPS_OK);
        isMovingSP.apply();
    }

//    if (!sendCommand(MDCP_GET_REL_HUMIDITY, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_REL_HUMIDITY_REPSONSE, &humidity) == 1) {

//        HumidityNP[0].setValue(humidity);
//        HumidityNP.setState(IPS_OK);
//        HumidityNP.apply();
//    } else {
//        LOG_INFO(resp);
//    }

//    if (!sendCommand(MDCP_GET_DEW_POINT, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_DEW_POINT_RESPONSE, &dewpoint) == 1) {
//        DewpointNP[0].setValue(dewpoint);
//        DewpointNP.setState(IPS_OK);
//        DewpointNP.apply();
//    }

//    int power1, power2, power3;

//    if (!sendCommand(MDCP_GET_CHANNEL_POWER, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_CHANNEL_POWER_RESPONSE, &power1, &power2, &power3) == 3) {
//        OutputsNP[DEW_STRAP_ONE_POWER].setValue(power1);
//        OutputsNP[DEW_STRAP_TWO_POWER].setValue(power2);
//        OutputsNP[DEW_STRAP_THREE_POWER].setValue(power3);
//        OutputsNP.setState(IPS_OK);
//        OutputsNP.apply();
//        CH3_Manual_PowerNP[0].setValue(power3);
//        CH3_Manual_PowerNP.apply();
//    } else {
//        LOG_INFO(resp);
//    }

//    int mode;

//    if (!sendCommand(MDCP_GET_CH3_SETTINGS, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_CH3_SETTINGS_RESPONSE, &mode) == 1) {
//        CH3_ModeSP.reset();
//        CH3_ModeSP[mode].setState(ISS_ON);
//        CH3_ModeSP.setState(IPS_OK);
//        CH3_ModeSP.apply();
//    } else {
//        LOG_INFO(resp);
//    }

//    if (!sendCommand(MDCP_GET_FAN_SPEED, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    int fanSpeed;

//    if (sscanf(resp, "F%d$", &fanSpeed) == 1) {
//        FanSpeedNP[0].setValue(fanSpeed);
//        FanSpeedNP.setState(IPS_OK);
//        FanSpeedNP.apply();
//    }

    return true;
}

//bool myDewControllerPro::readOffsetValues()
//{
//    char resp[MDCP_RES_LEN];
//    float temp1, temp2, temp3;


//    if (!sendCommand(MDCP_GET_TEMP_OFFSETS, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_TEMP_OFFSETS_RESPONSE, &temp1, &temp2, &temp3) == 3) {
//        TemperatureOffsetsNP[TEMP_PROBE_ONE_OFFSET].setValue(temp1);
//        TemperatureOffsetsNP[TEMP_PROBE_TWO_OFFSET].setValue(temp2);
//        TemperatureOffsetsNP[TEMP_PROBE_THREE_OFFSET].setValue(temp3);
//        TemperatureOffsetsNP.setState(IPS_OK);
//        TemperatureOffsetsNP.apply();
//    }

//    if (!sendCommand(MDCP_GET_AMB_TEMP_OFFSET, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    int atBias = 0;
//    if (sscanf(resp, MDCP_GET_AMB_TEMP_OFFSET_RESPONSE, &atBias) == 1) {
//        TemperatureOffsetsNP[AMBIENT_TEMP_PROBE_OFFSET].setValue(atBias);
//        TemperatureOffsetsNP.setState(IPS_OK);
//        TemperatureOffsetsNP.apply();
//    }
//    if (!sendCommand(MDCP_GET_TRACKING_MODE, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    int mode;

//    if (sscanf(resp, MDCP_GET_TRACKING_MODE_RESPONSE, &mode) == 1) {
//        TrackingModeSP.reset();
//        TrackingModeSP[mode].setState(ISS_ON);
//        TrackingModeSP.setState(IPS_OK);
//        TrackingModeSP.apply();
//    }

//    if (!sendCommand(MDCP_GET_TRACKING_MODE_OFFSET, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    int toffset = 0;

//    if (sscanf(resp, "y%d$", &toffset) == 1) {
//        TrackingModeOffsetNP[0].setValue(toffset);
//        TrackingModeOffsetNP.setState(IPS_OK);
//        TrackingModeOffsetNP.apply();
//    }
//    return true;
//}


//bool myDewControllerPro::readBoardFanValues()
//{
//    char resp[MDCP_RES_LEN];

//    if (!sendCommand(MDCP_GET_FAN_SPEED, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    int fanSpeed;

//    if (sscanf(resp, "F%d$", &fanSpeed) == 1) {
//        FanSpeedNP[0].setValue(fanSpeed);
//        FanSpeedNP.setState(IPS_OK);
//        FanSpeedNP.apply();
//    }

//    if (!sendCommand(MDCP_GET_FAN_MODE, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    int mode;
//    if (sscanf(resp, MDCP_GET_FAN_MODE_RESPONSE, &mode) == 1) {
//        FanModeSP.reset();
//        FanModeSP[mode].setState(ISS_ON);
//        FanModeSP.setState(IPS_OK);
//        FanModeSP.apply();
//    }

//    if (!sendCommand(MDCP_GET_FAN_ON_TEMP, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    int fanTemp;

//    if (sscanf(resp, MDCP_GET_FAN_ON_TEMP_RESPONSE, &fanTemp) == 1) {
//        FanTempTriggerNP[FANTEMPON].setValue(fanTemp);
//        FanTempTriggerNP.setState(IPS_OK);
//        FanTempTriggerNP.apply();
//    }

//    if (!sendCommand(MDCP_GET_FAN_OFF_TEMP, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_FAN_OFF_TEMP_RESPONSE, &fanTemp) == 1) {
//        FanTempTriggerNP[FANTEMPOFF].setValue(fanTemp);
//        FanTempTriggerNP.setState(IPS_OK);
//        FanTempTriggerNP.apply();
//    }

//    return true;
//}

//bool myDewControllerPro::readLCDDisplayValues()
//{
//    char resp[MDCP_RES_LEN];
//    int value;

//    if (!sendCommand(MDCP_GET_LCD_DISPLAY_TIME, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }
//    if (sscanf(resp, MDCP_GET_LCD_DISPLAY_TIME_RESPONSE, &value) == 1) {
//        LCDPageRefreshNP[0].setValue(value);
//        LCDPageRefreshNP.setState(IPS_OK);
//        LCDPageRefreshNP.apply();
//    }

//    if (!sendCommand(MDCP_GET_LCD_STATE, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_LCD_STATE_RESPONSE, &value) == 1) {
//        EnableLCDDisplaySP.reset();
//        EnableLCDDisplaySP[value].setState(ISS_ON);
//        EnableLCDDisplaySP.setState(IPS_OK);
//        EnableLCDDisplaySP.apply();
//    }

//    if (!sendCommand(MDCP_GET_TEMP_DISPLAY, resp)) {
//        LOG_INFO(resp);
//        return false;
//    }

//    if (sscanf(resp, MDCP_GET_TEMP_DISPLAY_RESPONSE, &value) == 1) {
//        LCDDisplayTempUnitsSP.reset();
//        LCDDisplayTempUnitsSP[value-1].setState(ISS_ON);
//        LCDDisplayTempUnitsSP.setState(IPS_OK);
//        LCDDisplayTempUnitsSP.apply();
//    }
//        return true;
//}

void arduinoMyRotator::TimerHit()
{
    if (!isConnected())
    {
        return;
    }
    if (rotatorMoving) {
        readMainValues();
    }
    SetTimer(getCurrentPollingPeriod());
}
