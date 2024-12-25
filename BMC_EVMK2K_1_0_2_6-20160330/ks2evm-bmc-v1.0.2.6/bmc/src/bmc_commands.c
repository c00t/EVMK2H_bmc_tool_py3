/******************************************************************************
 *
 * File	Name:       bmc_commands.c
 *
 * Description: This contains a lookup table used for command parsing.
 * 
 ******************************************************************************/
/****************************************************************************
 * Copyright (c) 2012 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "bmc_commands.h"

const int total_commands = 19;
Command command_list[] = {
    {   "lcd",
        &command_lcd,
        "Adds a string to a specified line and page of the LCD",
"lcd string page line\r\n\r\n\
    -Adds the specified character string to the page and line (given as decimal\r\n\
    integer values).\r\n\
\r\n\
    Example: lcd hello 0 0",
        false, false
    },
    {   "wait",
        &command_wait,
        "Waits for a specified amount of time",
        "wait n\r\n\r\n    -Delays for n milliseconds.\r\n\r\n    Example: wait 10000",
        false, false
    },
    {   "gpio",
        &command_gpio,
        "Used to read and write values to the MCU GPIO ports and GPIO expanders.",
"gpio port [value]\r\n\r\n\
    -Read or write to the specified port. If [value] is not present, a read will be\r\n\
    performed otherwise the operation will be a write. Ports are given as the\r\n\
    letter value (i.e. \'a\') with GPIO expanders containing an \'x\' in front of\r\n\
    the port name. The [value] given is represented as either an 8 or 16 bit (for\r\n\
    MCU and expander GPIO pins respectively) string with each bit being represented\r\n\
    as one of the following values:\r\n\
        -0: output HIGH to this pin.\r\n\
        -1: output LOW to this pin.\r\n\
        -z: configure this pin as Hi-Z (input).\r\n\
        -x: Do not modify this pin.\r\n\r\n\
    The value can be seperated regulary by \'_\' to allow for easier reading\r\n\
    (i.e. 1010_zzxx), as all underscores are ignored by the command.\r\n\
\r\n\
    Example: gpio xa 0000_xxxx_zzzz_1111",
        true, true
    },
    {   "spi",
        &command_spi,
        "Used to communicate with spi devices connected to the MCU.",
"spi port.cs [value...]\r\n\r\n\
    -Performs an SPI operation on the specifed SPI port with the given chip select.\r\n\
    The series of values are given as bytes in hexadecimal and separated by spaces.\r\n\
    Each value is sent to the SPI device and the result is read back and displayed\r\n\
    at the end of the operation.\r\n\
\r\n\
    Example: spi 0.3 41 12 00 00 00",
        true, true
    },
    {   "i2c",
        &command_i2c,
        "Used to communicate with I2C devices connected to the MCU.",
"i2c port.address [value|'r']\r\n\r\n\
    -Performs an I2C operation on the specified i2c port and address. A string of\r\n\
    values and the character \'r\' are used to determine whether to send a value to\r\n\
    the device or read from the device. The address and value are to be given in\r\n\
    hexadecimal. Upon completion The operation will print any errors it encountered,\r\n\
    and the results of any reads it performed.\r\n\
\r\n\
    Example: i2c 1.FA 0F B8 r r r",
        true, true
    },
    {   "clkreg",
        &command_clkreg,
        "Used to set/read registers from the CDCM Clock Generators.",
"clkreg clk.reg [data]\r\n\r\n\
    -Reads or writes to the specified clock register. [data] is an optiona\r\n\
    value given as a 16 bit value in either octal, decimal, or hexadecimal (using\r\n\
    a leading 0 indicates octal, using a leading 0x will indicate hex, and no\r\n\
    leading notation indicates decimal). If data is given, the value is written to\r\n\
    the specified clock registers; otherwise the current value of the register is\r\n\
    read and output on the console. \"reg\" can be listed as octal, decimal, or\r\n\
    hexadecimal in the same manner as [data]. \"clk\" specifies which CDCM Clock\r\n\
    Generator will be read from or written to.\r\n\
\r\n\
    Example: clkreg 1.4 0x020F",
        true, true
    },
    {   "readall",
        &command_readall,
        "",
        "",
        true, true
    },
    {   "eeprom",
        &command_eeprom,
        "Writes or reads from the EEPROM.",
"eeprom addr [bytes...]\r\n\r\n\
    -Performs a read or a write operation on the EEPROM. The start address is\r\n\
    specified through addr, which is given in hexadecimal. [bytes...] is a series of\r\n\
    8-bit values represented in hexadecimal. If given, these values are written to\r\n\
    the EEPROM; otherwise the first 16 bytes starting at the given address are read.\r\n\
\r\n\
    Example: eeprom 10F57 A5 F8 80",
        false, false
    },
    {   "help",
        &command_help,
        "Used to get information on other commands. Type \"help command\" for specific\r\n    information on a command.",
"help [command]\r\n\r\n\
    -Gives information on all BMC commands. If [command] is given, a detailed\r\n\
    description of the command and its use will be displayed. If no command is\r\n\
    given, all currently enabled commands will be listed with a short description.\r\n\
\r\n\
    Example: help reboot",
        false, false
    },
    {   "hwdbg",
        &command_hwdbg,
        "Used to set the debug mode of the BMC.",
"hwdbg mode\r\n\r\n\
    -Changes the current debug mode of the BMC system to mode. The current supported\r\n\
    modes are:\r\n\
        -OFF: default, no debugging occurs, the system will continually try to boot\r\n\
        if any process fails.\r\n\
        -ON: whenever an error is encountered, the system will halt and wait for\r\n\
        commands.\r\n\
        -CONTINUE: whenever an error is encountered, the system will ignore it and\r\n\
        continue with the startup process.\r\n\
\r\n\
    Exmaple: hwdbg on",
        true, false
    },
    {   "wp",
        &command_wp,
        "Used to control the write protect of EEPROM, NOR, and NAND.",
"wp [on|off] [eeprom] [nand] [nor]\r\n\r\n\
    -Enables/disables the Write Protects on the EEPROM, NAND and NOR, and displays\r\n\
    the current state of the WP values. If no options are given the current state of\r\n\
    the WP's is given. On or off will determine whether the WP is enabled or\r\n\
    disabled respectively. If no devices are listed, the command is applied to all\r\n\
    three; otherwise the command is applied to any device listed.\r\n\
 \r\n\
    Example: wp on eeprom nand",
        false, false
    },
    {   "reboot",
        &command_reboot,
        "Performs various reboot functions.",
"reboot [warm|*full*|por|pwr] [force]\r\n\r\n\
    -Places the board in the state provided (full if no state is given), and then\r\n\
    brings the board back to the running state. If the force option is given the BMC\r\n\
    will not request graceful shutdowns from the SOC.\r\n\
\r\n\
    Example: reboot por force",
        false, false
    },
    {   "shutdown",
        &command_shutdown,
        "Performs various shutdown functions.",
"shutdown [warm|*full*|por|pwr] [force]\r\n\r\n\
    -Places the board in the state provided (full if no state is given). If the\r\n\
    force option is given the BMC will not request graceful shutdowns from the SOC.\r\n\
\r\n\
    Example: shutdown pwr force",
        false, false
    },
    {   "run",
        &command_run,
        "Attempts to bring the board from its current state to the running state.",
"run\r\n\r\n\
    -Brings the board from its current state to the running state.\r\n\
\r\n\
    Example: run",
        false, false
    },
    {   "status",
        &command_status,
        "Displays the status of the board in formatted fashion.",
        "",
        false, false
    },
    {   "bootmode",
        &command_bootmode,
        "Used to perform various actions with Bootmodes.",
"bootmode [[#]x|all|read] [mode_hi mode_lo title]\r\n\r\n\
    -Used to alter/display bootmodes. If no options are given, the current bootmode\r\n\
    is displayed. If 'all' is given, All bootmode values are given with the current\r\n\
    bootmode marked in the list. If 'read' is given, the DIP switch is read and the\r\n\
    bootmode set accordingly. If #x with no mode or title is given, the bootmode is\r\n\
    set to the value represented by this index. Note that if mode or title are not\r\n\
    given, the # is not optional. If mode and title are given, the bootmode at the\r\n\
    index x is set to these values (only user programmable bootmodes may be adjusted\r\n\
    in this manner). If the # is present in this case then the new value is stored\r\n\
    in flash. Mode is given in hexadecimal; title is a string.\r\n\
\r\n\
    Example: bootmode #x 0 0100001 No-Boot",
        false, false
    },
    {   "commission",
        &command_commission,
        "Stores board type, version, and S/N, as well as enabling in-field updates.",
"commission [type] [revision] [s/n]\r\n\r\n\
    -Sets the flash registers to the values given. It is not required to give each\r\n\
    value when using the command. However, values must be given for all each\r\n\
    variable to the left of the one being set. That is, to set s/n, a value for\r\n\
    revision and type must be give. The command also sets up in-field updates. The\r\n\
    format of type should be a string of the EVM type. Revision should be given as a\r\n\
    decimal seperated revision value. S/N is the board serial number entered exactly\r\n\
    as listed.\r\n\
\r\n\
    Example: commission XTCIEVMK2X 1.0 123456",
        true, false
    },
    {   "ver",
        &command_ver,
        "Displays BMC version, board type, board version, and board serial number.",
"ver\r\n\r\n\
    -Displays BMC version, board type, board version, and board serial number in a\r\n\
    formatted output.\r\n\
\r\n\
    Example: ver",
        false, false
    },
    {   "pcie",
        &command_pcie,
        "Control the select to the PCIE clock mux.",
"pcie [0|1]\r\n\r\n\
    -Control the select to the PCIE clock mux. If no arguments are given the present select\r\n\
    of the mux will be given.\r\n\
\r\n\
    Example: pcie enable",
        false, false
    }
};
