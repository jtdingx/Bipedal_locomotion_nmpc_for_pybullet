/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <XBotInterface/Logger.hpp>

namespace XBot {

/////////////////////////////////////////////////////////////////////////////////////
// LoggerEndl << operator : print the os using the different Logger severity level //
/////////////////////////////////////////////////////////////////////////////////////

void operator<< ( std::ostream& os, LoggerEndl& log ) {
    log.print ( os );
}

std::map<std::string, FileLogger::Ptr> FileLogger::_instance_map;

ConsoleLogger::Ptr ConsoleLogger::_instance;

std::unordered_map<std::string, MatLogger::Ptr> MatLogger::_instances;

// now that SSLogger is completely defined we can implment the print()
void LoggerEndl::print ( std::ostream& ss ) {
    _ss_logger->print ( ss );
}


}