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
#include <ocl/DeploymentComponent.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>

using namespace RTT;
using namespace std;

int main(int argc, char** argv)
{
	std::string                 siteFile;      // "" means use default in DeploymentComponent.cpp
    std::vector<std::string>    scriptFiles;
	std::string                 name("Deployer");
    bool                        deploymentOnlyChecked = false;
	int							minNumberCPU = 0;

    // were we given non-deployer options? ie find "--"
    int     optIndex    = 0;
    bool    found       = false;

    while(!found && optIndex<argc)
    {
//        std::cout << "arg: \'" << argv[optIndex] << "\'" << std::endl;
        found = (0 == strcmp("--", argv[optIndex]));
        if(!found) optIndex++;
    }

    if (found) {
        argv[optIndex] = argv[0];
    }

    // extract script names from the command line
    std::string master_package_name;
    std::vector<std::string> scripts;
    for (int i = 1; i < optIndex; ++i) {
//        std::cout << "i: " << i << std::endl;
//        std::cout << "arg2: \'" << argv[i] << "\'" << std::endl;

        if (strcmp("-s", argv[i]) == 0) {
            scripts.push_back(argv[i+1]);
        }

        if (strcmp("-m", argv[i]) == 0) {
            master_package_name = argv[i+1];
        }
    }

    if (master_package_name.empty()) {
        std::cerr << "Master package name is missing. Usage argument: \'-m <master_package_name>\'" << std::endl;
        return -1;
    }

    SubsystemDeployer depl(name);

	if (0 == __os_init(argc, argv)) {
        RTT::Logger::log().setStdStream(std::cerr);
        RTT::Logger::log().mayLogStdOut(true);
        RTT::Logger::log().setLogLevel(RTT::Logger::Info);

        if (!depl.initializeSubsystem(master_package_name)) {
            return -2;
        }

        depl.runScripts(scripts);

        if (!depl.configure()) {
            return -3;
        }

        depl.runTaskBrowser();

        __os_exit();
    }

    return 0;
}

