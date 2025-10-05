/*
	Copyright 2024-2025 ChipFoundry, a DBA of Umbralogic Technologies LLC.

	Original Copyright 2024 Efabless Corp.
	Author: Efabless Corp. (ip_admin@efabless.com)

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

	    http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.

*/


/*! \file CF_I2C.c
    \brief C file for I2C APIs which contains the function implementations 
    
*/

#ifndef CF_I2C_C
#define CF_I2C_C

/******************************************************************************
* Includes
******************************************************************************/
#include "CF_I2C.h"

/******************************************************************************
* File-Specific Macros and Constants
******************************************************************************/



/******************************************************************************
* Static Variables
******************************************************************************/



/******************************************************************************
* Static Function Prototypes
******************************************************************************/



/******************************************************************************
* Function Definitions
******************************************************************************/

CF_DRIVER_STATUS CF_I2C_setGclkEnable (CF_I2C_TYPE_PTR i2c, uint32_t value){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (value > (uint32_t)0x1) {  
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if value is out of range
    }else {
        i2c->GCLK = value;                      // Set the GCLK enable bit to the given value
    }

    return status;
}




CF_DRIVER_STATUS CF_I2C_setCommandReg(CF_I2C_TYPE_PTR i2c, uint32_t value){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } if ((value & CF_I2C_COMMAND_REG_CMD_CORRECT_MASK) != (uint32_t)0x0) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if value is out of range
    }else{    
        bool is_command_available = false;
        do{
            status = CF_I2C_isCommandFIFOAvailable(i2c, &is_command_available);
        }while ((status==CF_DRIVER_OK)&&(is_command_available==false));
        i2c->COMMAND = value;
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_setCommandRegNonBlocking (CF_I2C_TYPE_PTR i2c, uint32_t value, bool *command_sent){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } if ((value & CF_I2C_COMMAND_REG_CMD_CORRECT_MASK) != (uint32_t)0x0) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if value is out of range
    }else{    
        bool is_command_available = false;
        status = CF_I2C_isCommandFIFOAvailable(i2c, &is_command_available);
        if ((status==CF_DRIVER_OK)&&(is_command_available==true)){
            i2c->COMMAND = value;
            *command_sent = true;
        }else{
            *command_sent = false;
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_getDataValid(CF_I2C_TYPE_PTR i2c, bool *data_valid){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (data_valid == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if data_valid is NULL
    } else{
        if (((i2c->DATA & CF_I2C_DATA_REG_DATA_VALID_MASK) >> CF_I2C_DATA_REG_DATA_VALID_BIT) != (uint32_t)0){
            *data_valid = true;
        }
        else{
            *data_valid = false;
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_setDataLast(CF_I2C_TYPE_PTR i2c){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    }else{
        i2c->DATA |= CF_I2C_DATA_REG_DATA_LAST_MASK;
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_getDataLast(CF_I2C_TYPE_PTR i2c, bool *data_last){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    }else if (data_last == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if data_last is NULL
    }else{    
        if (((i2c->DATA & CF_I2C_DATA_REG_DATA_LAST_MASK) >> CF_I2C_DATA_REG_DATA_LAST_BIT) != (uint32_t)0){
            *data_last = true;
        }else{
            *data_last = false;
        }
    }
    return status;
}



CF_DRIVER_STATUS CF_I2C_setPrescaler(CF_I2C_TYPE_PTR i2c, uint32_t value){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL   
    } else if (value > CF_I2C_PR_MAX_VALUE) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if value is out of range
    }else{
        i2c->PR = value;
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_getPrescaler(CF_I2C_TYPE_PTR i2c, uint32_t* prescaler_value){
    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL   
    } else if (prescaler_value == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if prescaler_value is NULL
    }else{  
        *prescaler_value = i2c->PR;
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_getRIS(CF_I2C_TYPE_PTR i2c, uint32_t* ris_value){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (ris_value == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if ris_value is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *ris_value = i2c->RIS;
        
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_getMIS(CF_I2C_TYPE_PTR i2c, uint32_t* mis_value){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (mis_value == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if mis_value is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *mis_value = i2c->MIS;
        
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_setIM(CF_I2C_TYPE_PTR i2c, uint32_t mask){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (mask > CF_I2C_IM_REG_MAX_VALUE) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if mask is out of range

    } else {
        i2c->IM = mask;
        
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_getIM(CF_I2C_TYPE_PTR i2c, uint32_t* im_value){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (im_value == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if im_value is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *im_value = i2c->IM;
        
    }
    return status;
}



//CF_DRIVER_STATUS CF_I2C_setICR(CF_I2C_TYPE_PTR i2c, uint32_t mask){
    
//    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

//    if (i2c == NULL) {
//        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
//    } else if (mask > CF_I2C_IC_REG_MAX_VALUE) {
//        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if mask is out of range
//    } else {
//        i2c->IC = mask;
        
//    }
//    return status;
//}




CF_DRIVER_STATUS CF_I2C_isBusy(CF_I2C_TYPE_PTR i2c, bool *is_busy){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (is_busy == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if is_busy is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *is_busy = i2c->STATUS & CF_I2C_STATUS_REG_BUSY_MASK;
        
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_isCommandFIFOAvailable(CF_I2C_TYPE_PTR i2c, bool *is_available){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (is_available == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if is_available is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *is_available = !((i2c->STATUS & CF_I2C_STATUS_REG_CMD_FULL_MASK) >> CF_I2C_STATUS_REG_CMD_FULL_BIT);
        
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_isWriteFIFOAvailable(CF_I2C_TYPE_PTR i2c, bool *is_available){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (is_available == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if is_available is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *is_available = !((i2c->STATUS & CF_I2C_STATUS_REG_WR_FULL_MASK) >> CF_I2C_STATUS_REG_WR_FULL_BIT);
        
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_isReadFIFOAvailable(CF_I2C_TYPE_PTR i2c, bool *is_available){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK; 

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (is_available == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;                // Return CF_DRIVER_ERROR_PARAMETER if is_available is NULL, 
                                                        // i.e. there is no memory location to store the value
    } else {
        *is_available = !((i2c->STATUS & CF_I2C_STATUS_REG_RD_FULL_MASK) >> CF_I2C_STATUS_REG_RD_FULL_BIT);
        
    }
    return status;
}


// this is a blocking send write command function
CF_DRIVER_STATUS CF_I2C_sendWriteCommand(CF_I2C_TYPE_PTR i2c, uint8_t addr){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else{
        bool command_FIFO_available = false;
        do{
            status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        }while ((status==CF_DRIVER_OK)&&(command_FIFO_available==false));
        
        i2c->COMMAND = ((addr << CF_I2C_COMMAND_REG_CMD_ADDRESS_BIT) | CF_I2C_COMMAND_REG_CMD_WRITE_MASK);
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_sendWriteCommandNonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t addr, bool *command_sent){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (command_sent == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if command_sent is NULL
    } else{
        bool command_FIFO_available = false;
        status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(command_FIFO_available==true)){
            i2c->COMMAND = ((addr << CF_I2C_COMMAND_REG_CMD_ADDRESS_BIT) | CF_I2C_COMMAND_REG_CMD_WRITE_MASK);
            *command_sent = true;
        }else{
            *command_sent = false;
        }
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_sendReadCommand(CF_I2C_TYPE_PTR i2c, uint8_t addr){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else{
        bool command_FIFO_available = false;
        do{
            status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        }while ((status==CF_DRIVER_OK)&&(command_FIFO_available==false));
        
        i2c->COMMAND = ((addr << CF_I2C_COMMAND_REG_CMD_ADDRESS_BIT) | CF_I2C_COMMAND_REG_CMD_READ_MASK);
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_sendReadCommandNonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t addr, bool *command_sent){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (command_sent == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if command_sent is NULL
    } else{
        bool command_FIFO_available = false;
        status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(command_FIFO_available==true)){
            i2c->COMMAND = ((addr << CF_I2C_COMMAND_REG_CMD_ADDRESS_BIT) | CF_I2C_COMMAND_REG_CMD_READ_MASK);
            *command_sent = true;
        }else{
            *command_sent = false;
        }
    }
    return status;
}  


CF_DRIVER_STATUS CF_I2C_sendStartCommand(CF_I2C_TYPE_PTR i2c){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else{
        bool command_FIFO_available = false;
        do{
            status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        }while ((status==CF_DRIVER_OK)&&(command_FIFO_available==false));
        
        i2c->COMMAND = CF_I2C_COMMAND_REG_CMD_START_MASK;
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_sendStartCommandNonBlocking(CF_I2C_TYPE_PTR i2c, bool *command_sent){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (command_sent == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if command_sent is NULL
    } else{
        bool command_FIFO_available = false;
        status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(command_FIFO_available==true)){
            i2c->COMMAND = CF_I2C_COMMAND_REG_CMD_START_MASK;
            *command_sent = true;
        }else{
            *command_sent = false;
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_sendStopCommand(CF_I2C_TYPE_PTR i2c){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else{
        bool command_FIFO_available = false;
        do{
            status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        }while ((status==CF_DRIVER_OK)&&(command_FIFO_available==false));
        
        i2c->COMMAND = CF_I2C_COMMAND_REG_CMD_STOP_MASK;
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_sendStopCommandNonBlocking(CF_I2C_TYPE_PTR i2c, bool *command_sent){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (command_sent == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if command_sent is NULL
    } else{
        bool command_FIFO_available = false;
        status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(command_FIFO_available==true)){
            i2c->COMMAND = CF_I2C_COMMAND_REG_CMD_STOP_MASK;
            *command_sent = true;
        }else{
            *command_sent = false;
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_sendWriteMultipleCommand(CF_I2C_TYPE_PTR i2c, uint8_t addr){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    bool command_FIFO_available = false;
    do{
        status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
    }while ((status==CF_DRIVER_OK)&&(command_FIFO_available==false));
    
    i2c->COMMAND = ((addr << CF_I2C_COMMAND_REG_CMD_ADDRESS_BIT) & CF_I2C_COMMAND_REG_CMD_WRITE_MULTIPLE_MASK);
    return status;
}

CF_DRIVER_STATUS CF_I2C_sendWriteMultipleCommandNonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t addr, bool *command_sent){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (command_sent == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if command_sent is NULL
    } else{
        bool command_FIFO_available = false;
        status = CF_I2C_isCommandFIFOAvailable(i2c, &command_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(command_FIFO_available==true)){
            i2c->COMMAND = ((addr << CF_I2C_COMMAND_REG_CMD_ADDRESS_BIT) & CF_I2C_COMMAND_REG_CMD_WRITE_MULTIPLE_MASK);
            *command_sent = true;
        }else{
            *command_sent = false;
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_writeDataToWriteFIFO(CF_I2C_TYPE_PTR i2c, uint8_t data){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else{
        bool write_FIFO_available = false;
        do{
            status = CF_I2C_isWriteFIFOAvailable(i2c, &write_FIFO_available);
        }while ((status==CF_DRIVER_OK)&&(write_FIFO_available==false));
        
        i2c->DATA = ((data << CF_I2C_DATA_REG_DATA_BIT) & CF_I2C_DATA_REG_DATA_MASK);
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_writeDataToWriteFIFONonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t data, bool *data_written){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (data_written == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if data_written is NULL
    } else{
        bool write_FIFO_available = false;
        status = CF_I2C_isWriteFIFOAvailable(i2c, &write_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(write_FIFO_available==true)){
            i2c->DATA = ((data << CF_I2C_DATA_REG_DATA_BIT) & CF_I2C_DATA_REG_DATA_MASK);
            *data_written = true;
        }else{
            *data_written = false;
        }
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_readDataFromReadFIFO(CF_I2C_TYPE_PTR i2c, uint8_t *data){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (data == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if data is NULL
    } else{
        bool read_FIFO_available = false;
        do{
            status = CF_I2C_isReadFIFOAvailable(i2c, &read_FIFO_available);
        }while ((status==CF_DRIVER_OK)&&(read_FIFO_available==false));
        
        *data = i2c->DATA;

        bool data_valid = ((*data & CF_I2C_DATA_REG_DATA_VALID_MASK) >> CF_I2C_DATA_REG_DATA_VALID_BIT);
        if (!data_valid){
            status = CF_DRIVER_ERROR_I2C_INVALID_DATA;     // Return CF_DRIVER_ERROR_DATA if data is not valid
        }else{}
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_readDataFromReadFIFONonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t *data, bool *data_read){

    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    } else if (data == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if data is NULL
    } else if (data_read == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if data_read is NULL
    } else{
        bool read_FIFO_available = false;
        status = CF_I2C_isReadFIFOAvailable(i2c, &read_FIFO_available);
        if ((status==CF_DRIVER_OK)&&(read_FIFO_available==true)){
            *data = i2c->DATA;
            bool data_valid = ((*data & CF_I2C_DATA_REG_DATA_VALID_MASK) >> CF_I2C_DATA_REG_DATA_VALID_BIT);
            if (!data_valid){
                status = CF_DRIVER_ERROR_I2C_INVALID_DATA;     // Return CF_DRIVER_ERROR_DATA if data is not valid
            }else{}
            *data_read = true;
        }else{
            *data_read = false;
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_transmitByte(CF_I2C_TYPE_PTR i2c, uint8_t data, uint8_t addr){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK;
    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    }else{
        status = CF_I2C_writeDataToWriteFIFO(i2c, data);
        if (status == CF_DRIVER_OK) {status = CF_I2C_sendWriteCommand(i2c, addr);}else{}
        if (status == CF_DRIVER_OK) {status = CF_I2C_sendStopCommand(i2c);}else{}
    }
    return status;
}



CF_DRIVER_STATUS CF_I2C_transmitByteNonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t data, uint8_t addr, bool *transmitted){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK;
    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    }else if (transmitted == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if transmitted is NULL
    }else{
        bool command_sent = false;
        bool data_written = false;
        bool stop_command_sent = false;
        status = CF_I2C_writeDataToWriteFIFONonBlocking(i2c, data, &data_written);
        if ((status == CF_DRIVER_OK)&&(data_written == true)) {status = CF_I2C_sendWriteCommandNonBlocking(i2c, addr, &command_sent);}else{}
        if ((status == CF_DRIVER_OK)&&(command_sent == true)) {status = CF_I2C_sendStopCommandNonBlocking(i2c, &stop_command_sent);}else{}
        if ((status == CF_DRIVER_OK)&&(stop_command_sent == true)) {*transmitted = true;}else{*transmitted = false;}
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_receiveByte(CF_I2C_TYPE_PTR i2c, uint8_t *data, uint8_t addr){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK;
    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    }else{
        status = CF_I2C_sendReadCommand(i2c, addr);
        if (status == CF_DRIVER_OK) {status = CF_I2C_sendStopCommand(i2c);}else{}
        if (status == CF_DRIVER_OK) {status = CF_I2C_readDataFromReadFIFO(i2c, data);}else{}
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_receiveByteNonBlocking(CF_I2C_TYPE_PTR i2c, uint8_t *data, uint8_t addr, bool *received){
    
    CF_DRIVER_STATUS status = CF_DRIVER_OK;
    if (i2c == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if i2c is NULL
    }else if (received == NULL) {
        status = CF_DRIVER_ERROR_PARAMETER;     // Return CF_DRIVER_ERROR_PARAMETER if received is NULL
    }else{
        bool command_sent = false;
        bool data_read = false;
        bool stop_command_sent = false;
        
        status = CF_I2C_sendReadCommandNonBlocking(i2c, addr, &command_sent);
        if ((status == CF_DRIVER_OK)&&(command_sent == true)) {status = CF_I2C_sendStopCommandNonBlocking(i2c, &stop_command_sent);}else{}
        if ((status == CF_DRIVER_OK)&&(stop_command_sent == true)) {status = CF_I2C_readDataFromReadFIFONonBlocking(i2c, data, &data_read);}else{}
        if ((status == CF_DRIVER_OK)&&(data_read == true)) {*received = true;}else{*received = false;}
    }
    return status;
}


CF_DRIVER_STATUS CF_I2C_transmitCharArr(CF_I2C_TYPE_PTR i2c, uint8_t *data, uint32_t data_length, uint8_t addr) {
    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    // Validate input parameters
    if ((i2c == NULL) || (data == NULL)) {
        status = CF_DRIVER_ERROR_PARAMETER;                                 // Set error status if parameters are invalid
    } else {

        status = CF_I2C_writeDataToWriteFIFO(i2c, data[0]);                 // Write the first byte to the FIFO

        if (status == CF_DRIVER_OK) {
            status = CF_I2C_sendWriteMultipleCommand(i2c, addr);                  // Send the write multiple command
        }else{}

        // Transmit the remaining bytes if no error has occurred
        for (volatile uint32_t i = 1; (i < data_length) && (status == CF_DRIVER_OK); i++) {
            
            if (i == (data_length - (uint32_t)1)) {
                status = CF_I2C_setDataLast(i2c);                     // Mark the last byte appropriately
            }

            if (status == CF_DRIVER_OK) {
                status = CF_I2C_writeDataToWriteFIFO(i2c, data[i]);         // Write the current byte to the FIFO
            }
        }
        
        if (status == CF_DRIVER_OK) {
            status = CF_I2C_sendStopCommand(i2c);                           // Send the stop command if all prior steps succeeded
        }
    }
    return status;
}

CF_DRIVER_STATUS CF_I2C_recieveCharArr(CF_I2C_TYPE_PTR i2c, uint8_t *data, uint32_t data_length, uint8_t addr) {
    CF_DRIVER_STATUS status = CF_DRIVER_OK;

    // Validate input parameters
    if ((i2c == NULL) || (data == NULL) || (data_length == (uint32_t)0)) {
        status = CF_DRIVER_ERROR_PARAMETER;                                 // Set error status if parameters are invalid
    } else {
        // Transmit the remaining bytes if no error has occurred
        for (volatile uint32_t i = 0; (i < data_length) && (status == CF_DRIVER_OK); i++) {
            
            status = CF_I2C_sendReadCommand(i2c, addr);                     // Send the read command

            if (status == CF_DRIVER_OK) {
                status = CF_I2C_readDataFromReadFIFO(i2c, data[i]);                     // read the current byte to the FIFO
            }
        }
        
        if (status == CF_DRIVER_OK) {
            status = CF_I2C_sendStopCommand(i2c);                           // Send the stop command if all prior steps succeeded
        }
    }
    return status;
}
    

/******************************************************************************
* Static Function Definitions
******************************************************************************/



#endif // CF_I2C_C

/******************************************************************************
* End of File
******************************************************************************/
