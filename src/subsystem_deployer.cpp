/***************************************************************************
  tag: Peter Soetens  Thu Jul 3 15:30:14 CEST 2008  deployer.cpp

                        deployer.cpp -  description
                           -------------------
    begin                : Thu July 03 2008
    copyright            : (C) 2008 Peter Soetens
    email                : peter.soetens@fmtc.be

 ***************************************************************************
 *   This program is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 ***************************************************************************/

#include "subsystem_deployer/subsystem_deployer.h"

#include <rtt/rtt-config.h>
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <ocl/TaskBrowser.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>

using namespace RTT;
using namespace std;

SubsystemDeployer::SubsystemDeployer(const std::string& name) :
    name_(name)
{
}

void SubsystemDeployer::initializeSubsystem(const std::string& master_service_name) {
    dc_.reset(new OCL::DeploymentComponent(name_));

    // TODO: load master component and other generic stuff
}

void SubsystemDeployer::runScripts(const std::vector<std::string>& scriptFiles) {
    /******************** WARNING ***********************
     *   NO log(...) statements before __os_init() !!!!! 
     ***************************************************/
    int rc = 0;
    bool deploymentOnlyChecked = false;


            /* Only start the scripts after the Orb was created. Processing of
               scripts stops after the first failed script, and -1 is returned.
               Whether a script failed or all scripts succeeded, in non-daemon
               and non-checking mode the TaskBrowser will be run to allow
               inspection if the input is a tty.
             */
            bool result = true;
            for (std::vector<std::string>::const_iterator iter=scriptFiles.begin();
                 iter!=scriptFiles.end() && result;
                 ++iter)
            {
                if ( !(*iter).empty() )
                {
                    if ( (*iter).rfind(".xml",std::string::npos) == (*iter).length() - 4 || (*iter).rfind(".cpf",std::string::npos) == (*iter).length() - 4) {
/*                        if ( deploymentOnlyChecked ) {
                            if (!dc_->loadComponents( (*iter) )) {
                                result = false;
                                log(Error) << "Failed to load file: '"<< (*iter) <<"'." << endlog();
                            } else if (!dc_.configureComponents()) {
                                result = false;
                                log(Error) << "Failed to configure file: '"<< (*iter) <<"'." << endlog();
                            }
                            // else leave result=true and continue
                        } else {*/
                            result = dc_->kickStart( (*iter) );
//                        }
                        continue;
                    }

                    if ( (*iter).rfind(".ops",std::string::npos) == (*iter).length() - 4 ||
                         (*iter).rfind(".osd",std::string::npos) == (*iter).length() - 4 ||
                         (*iter).rfind(".lua",std::string::npos) == (*iter).length() - 4) {
                        result = dc_->runScript( (*iter) ) && result;
                        continue;
                    }
                    log(Error) << "Unknown extension of file: '"<< (*iter) <<"'. Must be xml, cpf for XML files or, ops, osd or lua for script files."<<endlog();
                }
            }
            rc = (result ? 0 : -1);
}

bool SubsystemDeployer::runTaskBrowser() {

//            if ( !deploymentOnlyChecked ) {
                if (isatty(fileno(stdin))) {
                    OCL::TaskBrowser tb( dc_.get() );
                    tb.loop();
                } else {
                    dc_->waitForInterrupt();
                }

                dc_->shutdownDeployment();
//            }
}

boost::shared_ptr<OCL::DeploymentComponent >& SubsystemDeployer::getDc() {
    return dc_;
}

