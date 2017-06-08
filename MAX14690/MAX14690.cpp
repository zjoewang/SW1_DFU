/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#include "MAX14690.h"

//******************************************************************************
MAX14690::MAX14690(I2C &i2c): _i2c(i2c)
{
    resetToDefaults();
}

MAX14690::MAX14690(PinName sda, PinName scl): _i2c(sda, scl)
{
    resetToDefaults();
}


//******************************************************************************
MAX14690::~MAX14690()
{
}

// extern Serial pc;

//******************************************************************************
void MAX14690::resetToDefaults()
{
    intEnThermStatus = false;
    intEnChgStatus = false;
    intEnILim = false;
    intEnUSBOVP = false;
    intEnUSBOK = false;
    intEnChgThmSD = false;
    intEnThermReg = false;
    intEnChgTimeOut = false;
    intEnThermBuck1 = false;
    intEnThermBuck2 = false;
    intEnThermLDO1 = false;
    intEnThermLDO2 = false;
    intEnThermLDO3 = false;
    iLimCntl = ILIM_100mA;       // Changed
    chgAutoStp = true;
    chgAutoReSta = true;
    batReChg = BAT_RECHG_220mV;  //Changed
    batReg = BAT_REG_4200mV;
    chgEn = true;
    vPChg = VPCHG_3000mV;
    iPChg = IPCHG_10;
    chgDone = CHGDONE_5;        //Changed
    mtChgTmr = MTCHGTMR_0min;
    fChgTmr = FCHGTMR_75min;    //Changed
    pChgTmr = PCHGTMR_30min;    //Changed
    buck1Md = BUCK_BURST;
    buck1Ind = 0;
    buck2Md = BUCK_BURST;
    buck2Ind = 0;
    ldo2Mode = LDO_ENABLED;     //Changed
    ldo2Millivolts = 3200;      //Changed
    ldo3Mode = SW_ENABLED;      //Changed
    // ldo3Millivolts = 3000;    Cannot set LDO 3 Voltage on version "B" 
    thrmCfg = THRM_ENABLED;
    monRatio = MON_DIV4;
    monCfg = MON_PULLDOWN;
    buck2ActDsc = false;
    buck2FFET = false;
    buck1ActDsc = false;
    buck1FFET = false;
    pfnResEna = true;
    stayOn = true;
}

//******************************************************************************
int MAX14690::ldo2SetMode(ldoMode_t mode)
{
    ldo2Mode = mode;
    return writeReg(REG_LDO2_CFG, mode);
}

//******************************************************************************
int MAX14690::ldo2SetVoltage(int mV)
{
    int regBits = mv2bits(mV);
    char data;

    if (regBits < 0) {
        return MAX14690_ERROR;
    } else {
        data = regBits;
    }

    if (ldo2Mode == LDO_ENABLED) {
        if (writeReg(REG_LDO2_CFG, LDO_DISABLED) != MAX14690_NO_ERROR) {
            return MAX14690_ERROR;
        }
    }

    if (writeReg(REG_LDO2_VSET, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }

    if (ldo2Mode == LDO_ENABLED) {
        if (writeReg(REG_LDO2_CFG, LDO_ENABLED) != MAX14690_NO_ERROR) {
            return MAX14690_ERROR;
        }
    }

    return MAX14690_NO_ERROR;
}

//******************************************************************************
int MAX14690::ldo3SetMode(ldoMode_t mode)
{
    ldo3Mode = mode;
    return writeReg(REG_LDO3_CFG, mode);
}

//******************************************************************************
int MAX14690::ldo3SetVoltage(int mV)
{
    int regBits = mv2bits(mV);
    char data;

    if (regBits < 0) {
        return MAX14690_ERROR;
    } else {
        data = regBits;
    }

    if (ldo3Mode == LDO_ENABLED) {
        if (writeReg(REG_LDO3_CFG, LDO_DISABLED) != MAX14690_NO_ERROR) {
            return MAX14690_ERROR;
        }
    }

    if (writeReg(REG_LDO3_VSET, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }

    if (ldo3Mode == LDO_ENABLED) {
        if (writeReg(REG_LDO3_CFG, LDO_ENABLED) != MAX14690_NO_ERROR) {
            return MAX14690_ERROR;
        }
    }

    return MAX14690_NO_ERROR;
}

//******************************************************************************
int MAX14690::init()
{
    int regBits;
    char data;
    
    // Configure buck regulators
    data = (buck1Md << 1) |
           (buck1Ind);
    if (writeReg(REG_BUCK1_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
    wait(0.01);
      
    data = (buck2Md << 1) |
           (buck2Ind);
    if (writeReg(REG_BUCK2_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
      wait(0.01);
       
    data = (buck2ActDsc << 5) |
           (buck2FFET << 4) |
           (buck1ActDsc << 1) |
           (buck1FFET);
    if (writeReg(REG_BUCK_EXTRA, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
      wait(0.01);

    // Configure Charger
    data = (iLimCntl);
    if (writeReg(REG_I_LIM_CNTL, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
        wait(0.01);
      
    data = (vPChg << 4) |
           (iPChg << 2) |
           (chgDone);
    if (writeReg(REG_CHG_CNTL_B, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
    wait(0.01);
      
    data = (mtChgTmr << 4) |
           (fChgTmr << 2) |
           (pChgTmr);
    if (writeReg(REG_CHG_TMR, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
        wait(0.01);
      
    data = (thrmCfg);
    if (writeReg(REG_THRM_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    // Set enable bit after setting other charger bits
    data = (chgAutoStp << 7) |
           (chgAutoReSta << 6) |
           (batReChg << 4) |
           (batReg << 1) |
           (chgEn);
    if (writeReg(REG_CHG_CNTL_A, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
     wait(0.01);

    // Configure monitor multiplexer
    data = (monRatio << 4) |
           (monCfg);
    if (writeReg(REG_MON_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    wait(0.01);
      
    // Configure and enable LDOs
    regBits = mv2bits(ldo2Millivolts);
    if (regBits < 0) {
        return MAX14690_ERROR;
    } else {
        data = regBits;
    }
    if (writeReg(REG_LDO2_VSET, data) != MAX14690_NO_ERROR) {
        //  pc.printf("Error with LDO2\r\n");
        return MAX14690_ERROR;
    }
    data = (ldo2Mode);
    if (writeReg(REG_LDO2_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    regBits = mv2bits(ldo3Millivolts);
    if (regBits < 0) {
        return MAX14690_ERROR;
    } else {
        data = regBits;
    }
    if (writeReg(REG_LDO3_VSET, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    data = (ldo3Mode);
    if (writeReg(REG_LDO3_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
    // Set stayOn bit after other registers to confirm successful boot.
    data = (pfnResEna << 7) |
           (stayOn);
    if (writeReg(REG_PWR_CFG, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }

    // Unmask Interrupts Last
    data = (intEnThermStatus << 7) |
           (intEnChgStatus << 6) |
           (intEnILim << 5) |
           (intEnUSBOVP << 4) |
           (intEnUSBOK << 3) |
           (intEnChgThmSD << 2) |
           (intEnThermReg << 1) |
           (intEnChgTimeOut);
    if (writeReg(REG_INT_MASK_A, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    data = (intEnThermBuck1 << 4) |
           (intEnThermBuck2 << 3) |
           (intEnThermLDO1 << 2) |
           (intEnThermLDO2 << 1) |
           (intEnThermLDO3);
    if (writeReg(REG_INT_MASK_B, data) != MAX14690_NO_ERROR) {
        return MAX14690_ERROR;
    }
    
    return MAX14690_NO_ERROR;
}

//******************************************************************************
int MAX14690::monSet(monCfg_t newMonCfg, monRatio_t newMonRatio)
{
    char data = (newMonRatio << 4) | (newMonCfg);
    monCfg = newMonCfg;
    monRatio = newMonRatio;
    return writeReg(REG_MON_CFG, data);
}

//******************************************************************************
int MAX14690::shutdown()
{
    return writeReg(REG_PWR_OFF, 0xB2);
}


//******************************************************************************
int MAX14690::writeReg(registers_t reg,  char value)
{
    char cmdData[2] = { (char)reg, value };

    if (_i2c.write(MAX14690_I2C_ADDR, cmdData, sizeof(cmdData)) != 0) {
        return MAX14690_ERROR;
    }

    return MAX14690_NO_ERROR;
}

//******************************************************************************
int MAX14690::readReg(registers_t reg, char *value)
{
    char cmdData[1] = { (char)reg };

    if (_i2c.write(MAX14690_I2C_ADDR, cmdData, sizeof(cmdData)) != 0) {
        return MAX14690_ERROR;
    }

    if (_i2c.read(MAX14690_I2C_ADDR, value, 1) != 0) {
        return MAX14690_ERROR;
    }

    return MAX14690_NO_ERROR;
}

//******************************************************************************
int MAX14690::mv2bits(int mV)
{
    int regBits;

    if ((MAX14690_LDO_MIN_MV <= mV) && (mV <= MAX14690_LDO_MAX_MV)) {
        regBits = (mV - MAX14690_LDO_MIN_MV) / MAX14690_LDO_STEP_MV;
    } else {
        return -1;
    }

    return regBits;
}

