/******************************************************************************
hardware.cpp
MicroOLED Arduino Library Hardware Interface

Jim Lindblom @ SparkFun Electronics
October 26, 2014
https://github.com/sparkfun/Micro_OLED_Breakout/tree/master/Firmware/Arduino/libraries/SFE_MicroOLED

Modified by:
Emil Varughese @ Edwin Robotics Pvt. Ltd.
July 27, 2015
https://github.com/emil01/SparkFun_Micro_OLED_Arduino_Library/

This file defines the hardware interface(s) for the Micro OLED Breakout. Those
interfaces include SPI, I2C and a parallel bus.

Development environment specifics:
Arduino 1.0.5
Arduino Pro 3.3V
Micro OLED Breakout v1.0

This code was heavily based around the MicroView library, written by GeekAmmo
(https://github.com/geekammo/MicroView-Arduino-Library), and released under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include "SFE_MicroOLED.h"
//#include <SPI.h> - included in SFE_MicroOLED.h
//#include <Wire.h> - included in SFE_MicroOLED.h

/** \brief Initialize the I2C Interface

	This function initializes the I2C peripheral.
**/
void MicroOLED::i2cSetup(uint8_t deviceAddress, TwoWire &wirePort)
{
	_i2cPort = &wirePort;
	if (deviceAddress != I2C_ADDRESS_UNDEFINED)
		moled_i2c_address = deviceAddress;
	moled_interface = MOLED_MODE_I2C; // Just in case moled_interface was undefined
}

/** \brief  Write a byte over I2C

	Write a byte to I2C device _address_. The DC byte determines whether
	the data being sent is a command or display data. Use either I2C_COMMAND
	or I2C_DATA in that parameter. The data byte can be any 8-bit value.
**/
void MicroOLED::i2cWrite(uint8_t address, uint8_t dc, uint8_t data)
{
	_i2cPort->beginTransmission(address);
	_i2cPort->write(dc);
  _i2cPort->write(data); // If data dc = 0, if command dc = 0x40
	_i2cPort->endTransmission();
}

/** \brief  Write multiple data bytes over I2C

	Write multiple bytes to I2C device _address_.
	Returns true if all numDataBytes were written successfully
**/
bool MicroOLED::i2cWriteMultiple(uint8_t address, uint8_t *dataBytes, size_t numDataBytes)
{
  // I2C: split the data up into packets of i2cTransactionSize
  size_t bytesLeftToWrite = numDataBytes;
  size_t bytesWrittenTotal = 0;

  while (bytesLeftToWrite > 0)
  {
    size_t bytesToWrite; // Limit bytesToWrite to i2cTransactionSize
    if (bytesLeftToWrite > ((size_t)i2cTransactionSize - 1))
      bytesToWrite = i2cTransactionSize - 1;
    else
      bytesToWrite = bytesLeftToWrite;

    _i2cPort->beginTransmission(address);
		_i2cPort->write(I2C_DATA);
    size_t bytesWritten = _i2cPort->write(dataBytes, bytesToWrite); // Write the bytes

    bytesWrittenTotal += bytesWritten; // Update the totals
    bytesLeftToWrite -= bytesToWrite;
    dataBytes += bytesToWrite; // Point to fresh data

    if (bytesLeftToWrite > 0)
    {
      if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
        return (false);                          //Sensor did not ACK
    }
    else
    {
      if (_i2cPort->endTransmission() != 0) //We're done. Release bus.
        return (false);                     //Sensor did not ACK
    }
  }

  return (bytesWrittenTotal == numDataBytes);
}