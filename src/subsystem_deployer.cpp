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
#include "common_behavior/master_service_requester.h"
#include "common_behavior/master_service.h"

#include <rtt/rtt-config.h>
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/internal/GlobalService.hpp>

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

bool SubsystemDeployer::import(const std::string& name) {
    if (!ros_import_.ready()) {
        Logger::log() << Logger::Error << "ros.import operation is not ready" << Logger::endl;
        return false;
    }

    if (!ros_import_(name)) {
        Logger::log() << Logger::Error << "could not import: " << name << Logger::endl;
        return false;
    }

    return true;
}

static void printInputBufferInfo(const common_behavior::InputBufferInfo& info) {
    Logger::log()
        << " enable_ipc: " << (info.enable_ipc_?"true":"false")
        << ", ipc_channel_name: \'" << info.ipc_channel_name_ << "\'"
        << ", event_port: " << (info.event_port_?"true":"false")
        << ", always_update_peers: " << (info.always_update_peers_?"true":"false")
        << ", interface_prefix: \'" << info.interface_prefix_ << "\'"
        << ", master_component_port_name: \'" << info.master_component_port_name_ << "\'"
        << Logger::endl;
}

static bool loadROSParam(RTT::TaskContext* tc) {
    tc->loadService("rosparam");

    RTT::Service::shared_ptr tc_rosparam = tc->provides("rosparam");

    RTT::OperationCaller<bool()> tc_rosparam_getAll = tc_rosparam->getOperation("getAll");
    if (!tc_rosparam_getAll.ready()) {
        Logger::log() << Logger::Error << "could not get ROS parameter getAll operation for " << tc->getName() << Logger::endl;
        return false;
    }
    if (!tc_rosparam_getAll()) {
        Logger::log() << Logger::Warning << "could not read ROS parameters for " << tc->getName() << Logger::endl;
//        return false;     // TODO: this IS an error
    }

    return true;
}

bool SubsystemDeployer::initializeSubsystem(const std::string& master_service_name) {
    Logger::In in("SubsystemDeployer::init");

    dc_.reset(new OCL::DeploymentComponent(name_));

    dc_->import("rtt_ros");

    RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->getService("ros");
    if (!ros) {
        Logger::log() << Logger::Error << "rtt_ros: ros service could not be loaded (NULL pointer)" << Logger::endl;
        return false;
    }

    ros_import_ = ros->getOperation("import");

    if (!import("rtt_roscomm") ||
        !import("rtt_rosparam") ||
//        !import("rtt_rosclock") ||
        !import("rtt_rospack") ||
        !import("rtt_actionlib") ||
        !import("common_behavior") ||
        !import("conman") ||
//        !import("conman_ros") ||
        !import("eigen_typekit"))
    {
        Logger::log() << Logger::Error << "could not load some core dependencies" << Logger::endl;
        return false;
    }

    Logger::log() << Logger::Info << "loaded core dependencies" << Logger::endl;

    //
    // load conman scheme
    //
    if (!dc_->loadComponent("scheme","conman::Scheme")) {
        Logger::log() << Logger::Error << "could not load conman::Scheme" << Logger::endl;
        return false;
    }

    RTT::TaskContext* scheme = dc_->getPeer("scheme");

    if (!scheme) {
        Logger::log() << Logger::Error << "scheme in NULL" << Logger::endl;
        return false;
    }
    // scheme.loadService("conman_ros");    // TODO: check if this is needed
    scheme->configure();

    //
    // load master component
    //
    if (!dc_->loadComponent("master_component","MasterComponent")) {
        Logger::log() << Logger::Error << "could not load MasterComponent" << Logger::endl;
        return false;
    }

    RTT::TaskContext* master_component = dc_->getPeer("master_component");

    if (!master_component) {
        Logger::log() << Logger::Error << "master_component in NULL" << Logger::endl;
        return false;
    }

    // TODO: MasterService and its package are arguments
    if (!import("velma_core_ve_body")) {                        // TODO: read this from command line or ROS param
        return false;
    }

    //setActivity("master_component", 0, 6, ORO_SCHED_RT);
    master_component->loadService("VelmaCoreVeBodyMaster");     // TODO: read this from command line or ROS param

    master_component->addPeer(scheme);

    if (!loadROSParam(master_component)) {
        return false;
    }

/*    master_component->loadService("rosparam");

    RTT::Service::shared_ptr master_component_rosparam = master_component->provides("rosparam");

    RTT::OperationCaller<bool()> master_component_rosparam_getAll = master_component_rosparam->getOperation("getAll");
    if (!master_component_rosparam_getAll.ready()) {
        Logger::log() << Logger::Error << "could not get ROS parameter getAll operation for master_component" << Logger::endl;
        return false;
    }
    if (!master_component_rosparam_getAll()) {
        Logger::log() << Logger::Warning << "could not read ROS parameters for master_component" << Logger::endl;
//        return false;     // TODO: this IS an error
    }
*/
    // master component can be configured after all peers are added to scheme
    //master_component->configure();    // TODO

    //
    // manage ports and ipc buffers
    //
    std::vector<std::string > master_port_names = master_component->ports()->getPortNames();
    for (int i = 0; i < master_port_names.size(); ++i) {
        Logger::log() << Logger::Info << "master_component port[" << i << "]: " << master_port_names[i] << Logger::endl;
    }



    boost::shared_ptr<common_behavior::MasterServiceRequester > master_service_ = master_component->getProvider<common_behavior::MasterServiceRequester >("master");
    if (!master_service_) {
        RTT::log(RTT::Error) << "Unable to load common_behavior::MasterService from master_component" << RTT::endlog();
        return false;
    }

    std::vector<common_behavior::InputBufferInfo > lowerInputBuffers;
    std::vector<common_behavior::InputBufferInfo > upperInputBuffers;

    master_service_->getLowerInputBuffers(lowerInputBuffers);
    master_service_->getUpperInputBuffers(upperInputBuffers);

    Logger::log() << Logger::Info << "lowerInputBuffers:" << Logger::endl;
    for (int i = 0; i < lowerInputBuffers.size(); ++i) {
        printInputBufferInfo(lowerInputBuffers[i]);
    }

    Logger::log() << Logger::Info << "upperInputBuffers:" << Logger::endl;
    for (int i = 0; i < upperInputBuffers.size(); ++i) {
        printInputBufferInfo(upperInputBuffers[i]);
    }

    for (int i = 0; i < lowerInputBuffers.size(); ++i) {
        const common_behavior::InputBufferInfo& buf_info = lowerInputBuffers[i];
        if (buf_info.enable_ipc_) {
            std::string type = buf_info.interface_prefix_ + "StatusRx";
            std::string name = type + "_component";
            if (!dc_->loadComponent(name, type)) {
                RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
                return false;
            }
            RTT::TaskContext* buffer_comp = dc_->getPeer(name);

            if (!loadROSParam(buffer_comp)) {
                return false;
            }
            RTT::Property<std::string>* channel_name = dynamic_cast<RTT::Property<std::string>* >(buffer_comp->properties()->getProperty("channel_name"));
            if (!channel_name) {
                RTT::log(RTT::Error) << "component " << buffer_comp->getName() << " does not have property channel_name" << RTT::endlog();
                return false;
            }

            RTT::Property<bool>* event_port = dynamic_cast<RTT::Property<bool>* >(buffer_comp->properties()->getProperty("event_port"));
            if (!event_port) {
                RTT::log(RTT::Error) << "component " << buffer_comp->getName() << " does not have property event_port" << RTT::endlog();
                return false;
            }

            RTT::Property<bool>* always_update_peers = dynamic_cast<RTT::Property<bool>* >(buffer_comp->properties()->getProperty("always_update_peers"));
            if (!always_update_peers) {
                RTT::log(RTT::Error) << "component " << buffer_comp->getName() << " does not have property always_update_peers" << RTT::endlog();
                return false;
            }

            channel_name->set(buf_info.ipc_channel_name_);
            event_port->set(buf_info.event_port_);
            always_update_peers->set(buf_info.always_update_peers_);
            //setActivity("core_ve_body_status_tx", 0, 6, ORO_SCHED_RT);    // TODO
            buffer_comp->configure();
        }
        // TODO: add concate/split
    }

    for (int i = 0; i < upperInputBuffers.size(); ++i) {
        const common_behavior::InputBufferInfo& buf_info = upperInputBuffers[i];
        if (buf_info.enable_ipc_) {
            std::string type = buf_info.interface_prefix_ + "CommandRx";
            std::string name = type + "_component";
            if (!dc_->loadComponent(name, type)) {
                RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
                return false;
            }
            RTT::TaskContext* buffer_comp = dc_->getPeer(name);

            if (!loadROSParam(buffer_comp)) {
                return false;
            }
            RTT::Property<std::string>* channel_name = dynamic_cast<RTT::Property<std::string>* >(buffer_comp->properties()->getProperty("channel_name"));
            if (!channel_name) {
                RTT::log(RTT::Error) << "component " << buffer_comp->getName() << " does not have property channel_name" << RTT::endlog();
                return false;
            }

            RTT::Property<bool>* event_port = dynamic_cast<RTT::Property<bool>* >(buffer_comp->properties()->getProperty("event_port"));
            if (!event_port) {
                RTT::log(RTT::Error) << "component " << buffer_comp->getName() << " does not have property event_port" << RTT::endlog();
                return false;
            }

            RTT::Property<bool>* always_update_peers = dynamic_cast<RTT::Property<bool>* >(buffer_comp->properties()->getProperty("always_update_peers"));
            if (!always_update_peers) {
                RTT::log(RTT::Error) << "component " << buffer_comp->getName() << " does not have property always_update_peers" << RTT::endlog();
                return false;
            }

            channel_name->set(buf_info.ipc_channel_name_);
            event_port->set(buf_info.event_port_);
            always_update_peers->set(buf_info.always_update_peers_);
            //setActivity("core_ve_body_status_tx", 0, 6, ORO_SCHED_RT);    // TODO
            buffer_comp->configure();
        }
        // TODO: add concate/split
    }


    // TODO

/*
ros.import("rtt_velma_core_cs_ve_body_msgs");
ros.import("velma_core_cs_ve_body_interface");
ros.import("velma_core_ve_body");
ros.import("rtt_velma_core_ve_body_re_body_msgs");
ros.import("velma_core_ve_body_re_body_interface");
ros.import("rtt_std_msgs");
ros.import("port_operations");
ros.import("rtt_control_msgs");
ros.import("lwr_fri");
ros.import("controller_common");
ros.import("velma_controller");
ros.import("rtt_cartesian_trajectory_msgs");
ros.import("rtt_std_msgs");
ros.import("rtt_tf");
ros.import("velma_sim_gazebo");
*/



    Logger::log() << Logger::Info << "OK" << Logger::endl;

    // TODO: load master component and other generic stuff

    return true;
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

