/**
 * mSerialCommand - A Wiring/Arduino library to tokenize and parse commands
 * received over a serial port.
 *
 * Copyright (C) 2012 Stefan Rado
 * Copyright (C) 2011 Steven Cogswell <steven.cogswell@gmail.com>
 *                    http://husks.wordpress.com
 *
 * Version 20120522
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include "stdlib.h"
#include <ctype.h>

#include "mSerialCommand.h"
#include "source/globals.h"

/**
 * Constructor makes sure some things are set.
 */
mSerialCommand::mSerialCommand()
  : commandList(NULL),
    commandCount(0),
    defaultHandler(NULL),
    term('\r'),           // default terminator for commands, newline character
    last(NULL)
{
  strcpy(delim, " "); // strtok_r needs a null-terminated string
  clearBuffer();
}

/**
 * Adds a "command" and a handler function to the list of available commands.
 * This is used for matching a found token in the buffer, and gives the pointer
 * to the handler function to deal with it.
 */
void mSerialCommand::addCommand(const char *command, void (*function)()) {
  #ifdef SERIALCOMMAND_DEBUG
	serial.writeString("Adding command (");
	serial.writeuint32_t(commandCount, false);
	serial.writeString("): ");
	serial.writeStringNL(command);
  #endif

  commandList = (mSerialCommandCallback *) realloc(commandList, (commandCount + 1) * sizeof(mSerialCommandCallback));
  if(commandList == NULL){
	  serial.println("no mem!");
	  while(1);
  }
  strncpy(commandList[commandCount].command, command, SERIALCOMMAND_MAXCOMMANDLENGTH);
  commandList[commandCount].function = function;
  commandCount++;
}

void mSerialCommand::printCommandList(){  // Print command list
	for (int i = 0; i < commandCount; i++) {
		serial.println(commandList[i].command);
	}

}

void mSerialCommand::printCommandListAdmin(){  // Print command list
	for (int i = 0; i < (commandCount-1); i++) {
		serial.println(commandList[i].command);
	}

}

void mSerialCommand::printCommandListUser(){  // Print command list
	for (int i = 0; i < 12; i++) {
		serial.println(commandList[i].command);
	}

}

/**
 * This sets up a handler to be called in the event that the receveived command string
 * isn't in the list of commands.
 */
void mSerialCommand::setDefaultHandler(void (*function)(const char *)) {
  defaultHandler = function;
}


/**
 * This checks the Serial stream for characters, and assembles them into a buffer.
 * When the terminator character (default '\n') is seen, it starts parsing the
 * buffer for a prefix command, and calls handlers setup by addCommand() member
 */
void mSerialCommand::readSerial() {
  while (serial.getInputBufferCount() > 0) {
    char inChar = serial.readByte();   // Read single available character, there may be more waiting
    #ifdef SERIALCOMMAND_ECHO
      serial.print(inChar);   // Echo back to serial stream
    #endif

    if (inChar == term) {     // Check for the terminator (default '\r') meaning end of command
      #ifdef SERIALCOMMAND_DEBUG
        serial.writeString("Received: ");
        serial.writeStringNL(buffer);
      #endif

      char *command = strtok_r(buffer, delim, &last);   // Search for command at start of buffer
      if (command != NULL) {
        bool matched = false;
        for (int i = 0; i < commandCount; i++) {
          #ifdef SERIALCOMMAND_DEBUG
        	serial.writeString("Comparing [");
        	serial.writeString(command);
        	serial.writeString("] to [");
        	serial.writeString(commandList[i].command);
        	serial.writeStringNL("]");
          #endif

          // Compare the found command against the list of known commands for a match
          if (strncmp(command, commandList[i].command, SERIALCOMMAND_MAXCOMMANDLENGTH) == 0) {
            #ifdef SERIALCOMMAND_DEBUG
        	  serial.writeString("Matched Command: ");
        	  serial.writeStringNL(command);
            #endif

            // Execute the stored handler function for the command
            (*commandList[i].function)();
			#ifdef SERIALCOMMAND_SHOW_PROMPT
			  serial.print("\n>");
			#endif
            matched = true;
            break;
          }
        }
        if (!matched && (defaultHandler != NULL)) {
        	(*defaultHandler)(command);
			#ifdef SERIALCOMMAND_SHOW_PROMPT
			  serial.print("\n>");
			#endif
        }
      }
      clearBuffer();
    }
    else if (isprint(inChar)) {     // Only printable characters into the buffer
      if (bufPos < SERIALCOMMAND_BUFFER) {
        buffer[bufPos++] = inChar;  // Put character into buffer
        buffer[bufPos] = '\0';      // Null terminate
      } else {
        #ifdef SERIALCOMMAND_DEBUG
    	  serial.writeStringNL("Line buffer is full - increase SERIALCOMMAND_BUFFER");
        #endif
      }
    }
  }
}

/*
 * Clear the input buffer.
 */
void mSerialCommand::clearBuffer() {
  buffer[0] = '\0';
  bufPos = 0;
}

/**
 * Retrieve the next token ("word" or "argument") from the command buffer.
 * Returns NULL if no more tokens exist.
 */
char *mSerialCommand::next() {
  return strtok_r(NULL, delim, &last);
}

float mSerialCommand::parseFloatArg() {
    //Serial.println("[WARN]: unsafe method: mSerialCommand::parseDoubleArg() - must be corrected");
    char* arg;
    arg = next();
    return atof(arg);
}

long mSerialCommand::parseLongArg() {
  //Serial.println("[WARN]: unsafe method: mSerialCommand::parseLongArg() - must be corrected");
  char * arg;
  arg = next();
  return atol(arg);
}

double mSerialCommand::parseDoubleArg() {
    //Serial.println("[WARN]: unsafe method: mSerialCommand::parseDoubleArg() - must be corrected");
    char* arg;
    arg = next();
    return atof(arg);
}
