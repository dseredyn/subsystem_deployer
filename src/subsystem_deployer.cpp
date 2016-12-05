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

static void printOutputBufferInfo(const common_behavior::OutputBufferInfo& info) {
    Logger::log()
        << " enable_ipc: " << (info.enable_ipc_?"true":"false")
        << ", ipc_channel_name: \'" << info.ipc_channel_name_ << "\'"
        << ", interface_prefix: \'" << info.interface_prefix_ << "\'"
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

template <class T>
static bool setComponentProperty(RTT::TaskContext *tc, const std::string& prop_name, const T& value) {
    RTT::Property<T>* property = dynamic_cast<RTT::Property<T>* >(tc->properties()->getProperty(prop_name));
    if (!property) {
        RTT::log(RTT::Error) << "component " << tc->getName() << " does not have property " << prop_name << RTT::endlog();
        return false;
    }
    property->set(value);

    return true;
}

bool SubsystemDeployer::deployInputBufferIpcComponent(const common_behavior::InputBufferInfo& buf_info) {
    std::string type = buf_info.interface_prefix_ + "Rx";
    if (buf_info.enable_ipc_) {
        std::string name = type;
        if (!dc_->loadComponent(name, type)) {
            RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
            return false;
        }
        RTT::TaskContext* comp = dc_->getPeer(name);
        if (!setTriggerOnStart(comp, false)) {
            return false;
        }

        setComponentProperty<std::string >(comp, "channel_name", buf_info.ipc_channel_name_);
        setComponentProperty<bool >(comp, "event_port", buf_info.event_port_);
        setComponentProperty<bool >(comp, "always_update_peers", buf_info.always_update_peers_);

        //setActivity("core_ve_body_status_tx", 0, 6, ORO_SCHED_RT);    // TODO
        if (!comp->configure()) {
            RTT::log(RTT::Error) << "Unable to configure component " << name << RTT::endlog();
            return false;
        }
        return true;
    }
    RTT::log(RTT::Error) << "component " << type << " should not have IPC buffer" << RTT::endlog();
    return false;
}

bool SubsystemDeployer::deployOutputBufferIpcComponent(const common_behavior::OutputBufferInfo& buf_info) {
    std::string type = buf_info.interface_prefix_ + "Tx";
    if (buf_info.enable_ipc_) {
        std::string name = type;
        if (!dc_->loadComponent(name, type)) {
            RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
            return false;
        }
        RTT::TaskContext* comp = dc_->getPeer(name);
        if (!setTriggerOnStart(comp, false)) {
            return false;
        }

        setComponentProperty<std::string >(comp, "channel_name", buf_info.ipc_channel_name_);

        //setActivity("core_ve_body_status_tx", 0, 6, ORO_SCHED_RT);    // TODO
        if (!comp->configure()) {
            RTT::log(RTT::Error) << "Unable to configure component " << name << RTT::endlog();
            return false;
        }
        return true;
    }
    RTT::log(RTT::Error) << "component " << type << " should not have IPC buffer" << RTT::endlog();
    return false;
}

bool SubsystemDeployer::deployBufferSplitComponent(const common_behavior::BufferInfo& buf_info) {
    std::string type = buf_info.interface_prefix_ + "Split";
    std::string name = type;
    if (!dc_->loadComponent(name, type)) {
        RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
        return false;
    }
    RTT::TaskContext* comp = dc_->getPeer(name);
    if (!setTriggerOnStart(comp, false)) {
            return false;
        }

    //setActivity("core_ve_body_status_tx", 0, 6, ORO_SCHED_RT);    // TODO
    if (!comp->configure()) {
        RTT::log(RTT::Error) << "Unable to configure component " << name << RTT::endlog();
        return false;
    }
    return true;
}

bool SubsystemDeployer::deployBufferConcateComponent(const common_behavior::BufferInfo& buf_info) {
    std::string type = buf_info.interface_prefix_ + "Concate";
    std::string name = type;
    if (!dc_->loadComponent(name, type)) {
        RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
        return false;
    }
    RTT::TaskContext* comp = dc_->getPeer(name);
    if (!setTriggerOnStart(comp, false)) {
            return false;
        }

    //setActivity("core_ve_body_status_tx", 0, 6, ORO_SCHED_RT);    // TODO
    if (!comp->configure()) {
        RTT::log(RTT::Error) << "Unable to configure component " << name << RTT::endlog();
        return false;
    }
    return true;
}

bool SubsystemDeployer::createInputBuffers(const std::vector<common_behavior::InputBufferInfo >& buffers) {
    for (int i = 0; i < buffers.size(); ++i) {
        const common_behavior::InputBufferInfo& buf_info = buffers[i];
        if (buf_info.enable_ipc_) {
            if (!deployInputBufferIpcComponent(buf_info)) {
                return false;
            }
        }

        if (!deployBufferSplitComponent(buf_info)) {
            return false;
        }

        if (buf_info.enable_ipc_) {
            // connect Split-Rx ports
            if (!dc_->connect(buf_info.interface_prefix_ + "Rx.msg_OUTPORT", buf_info.interface_prefix_ + "Split.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Split-Rx: " << buf_info.interface_prefix_ << RTT::endlog();
                return false;
            }

            // connect Rx to master_component
            if (!dc_->connect(buf_info.interface_prefix_ + "Rx.msg_OUTPORT", std::string("master_component.") + buf_info.master_component_port_name_, ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Rx-master_component: " << buf_info.interface_prefix_ << RTT::endlog();
                return false;
            }
        }
        else {
            // there is no Rx component, so create additional Concate component
            if (!deployBufferConcateComponent(buf_info)) {
                return false;
            }

            // connect Split-Concate ports
            if (!dc_->connect(buf_info.interface_prefix_ + "Concate.msg_OUTPORT", buf_info.interface_prefix_ + "Split.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Split-Concate: " << buf_info.interface_prefix_ << RTT::endlog();
                return false;
            }

            // connect Concate to master_component
            if (!dc_->connect(buf_info.interface_prefix_ + "Concate.msg_OUTPORT", std::string("master_component.") + buf_info.master_component_port_name_, ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Concate-master_component: " << buf_info.interface_prefix_ << RTT::endlog();
                return false;
            }
        }
    }
    return true;
}

bool SubsystemDeployer::createOutputBuffers(const std::vector<common_behavior::OutputBufferInfo >& buffers) {
    for (int i = 0; i < buffers.size(); ++i) {
        const common_behavior::OutputBufferInfo& buf_info = buffers[i];
        if (buf_info.enable_ipc_) {
            if (!deployOutputBufferIpcComponent(buf_info)) {
                return false;
            }
        }

        if (!deployBufferConcateComponent(buf_info)) {
            return false;
        }

        if (buf_info.enable_ipc_) {
            if (!dc_->connect(buf_info.interface_prefix_ + "Concate.msg_OUTPORT", buf_info.interface_prefix_ + "Tx.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Concate-Tx: " << buf_info.interface_prefix_ << RTT::endlog();
                return false;
            }
        }
        else {
            // there is no Rx component, so create additional Split component
            if (!deployBufferSplitComponent(buf_info)) {
                return false;
            }

            // connect Split-Concate ports
            if (!dc_->connect(buf_info.interface_prefix_ + "Concate.msg_OUTPORT", buf_info.interface_prefix_ + "Split.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Split-Concate: " << buf_info.interface_prefix_ << RTT::endlog();
                return false;
            }
        }
    }
    return true;
}

bool SubsystemDeployer::setTriggerOnStart(RTT::TaskContext* tc, bool trigger) {
    RTT::base::AttributeBase* base = tc->attributes()->getAttribute("TriggerOnStart");
    if (!base) {
        RTT::log(RTT::Error) << "component " << tc->getName() << " does not have attribute " << "TriggerOnStart" << RTT::endlog();
        return false;
    }
    RTT::Attribute<bool>* triggerOnStart = static_cast<RTT::Attribute<bool>* >(base);
    if (!triggerOnStart) {
        RTT::log(RTT::Error) << "component " << tc->getName() << " does not have attribute " << "TriggerOnStart" << " of type bool" << RTT::endlog();
        return false;
    }
    triggerOnStart->set(trigger);
    return true;
}

bool SubsystemDeployer::initializeSubsystem(const std::string& master_package_name) {
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

    scheme_ = dc_->getPeer("scheme");

    if (!setTriggerOnStart(scheme_, false)) {
        return false;
    }

    if (!scheme_) {
        Logger::log() << Logger::Error << "scheme in NULL" << Logger::endl;
        return false;
    }
    // scheme.loadService("conman_ros");    // TODO: check if this is needed
    scheme_->configure();

    //
    // load master component
    //
    if (!dc_->loadComponent("master_component","MasterComponent")) {
        Logger::log() << Logger::Error << "could not load MasterComponent" << Logger::endl;
        return false;
    }

    master_component_ = dc_->getPeer("master_component");

    if (!setTriggerOnStart(master_component_, false)) {
        return false;
    }

    if (!master_component_) {
        Logger::log() << Logger::Error << "master_component in NULL" << Logger::endl;
        return false;
    }

//master_service_name = "velma_core_ve_body";
    // TODO: MasterService and its package are arguments
    if (!import(master_package_name)) {                        // TODO: read this from command line or ROS param
        return false;
    }

    //setActivity("master_component", 0, 6, ORO_SCHED_RT);
    master_component_->loadService(master_package_name + "_master");     // TODO: read this from command line or ROS param

    master_component_->addPeer(scheme_);

// TODO: this is not needes, as Master Component reads its parameters from Master Service
//    if (!loadROSParam(master_component_)) {
//        return false;
//    }

    //
    // manage ports and ipc buffers
    //
    std::vector<std::string > master_port_names = master_component_->ports()->getPortNames();
    for (int i = 0; i < master_port_names.size(); ++i) {
        Logger::log() << Logger::Info << "master_component port[" << i << "]: " << master_port_names[i] << Logger::endl;
    }

    boost::shared_ptr<common_behavior::MasterServiceRequester > master_service_ = master_component_->getProvider<common_behavior::MasterServiceRequester >("master");
    if (!master_service_) {
        RTT::log(RTT::Error) << "Unable to load common_behavior::MasterService from master_component" << RTT::endlog();
        return false;
    }

    // inputs
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

    if (!createInputBuffers(lowerInputBuffers)) {
        RTT::log(RTT::Error) << "Could not create lower input buffers" << RTT::endlog();
        return false;
    }

    if (!createInputBuffers(upperInputBuffers)) {
        RTT::log(RTT::Error) << "Could not create upper input buffers" << RTT::endlog();
        return false;
    }

    // outputs
    std::vector<common_behavior::OutputBufferInfo > lowerOutputBuffers;
    std::vector<common_behavior::OutputBufferInfo > upperOutputBuffers;

    master_service_->getLowerOutputBuffers(lowerOutputBuffers);
    master_service_->getUpperOutputBuffers(upperOutputBuffers);

    Logger::log() << Logger::Info << "lowerOutputBuffers:" << Logger::endl;
    for (int i = 0; i < lowerOutputBuffers.size(); ++i) {
        printOutputBufferInfo(lowerOutputBuffers[i]);
    }

    Logger::log() << Logger::Info << "upperOutputBuffers:" << Logger::endl;
    for (int i = 0; i < upperOutputBuffers.size(); ++i) {
        printOutputBufferInfo(upperOutputBuffers[i]);
    }

    if (!createOutputBuffers(lowerOutputBuffers)) {
        RTT::log(RTT::Error) << "Could not create lower output buffers" << RTT::endlog();
        return false;
    }

    if (!createOutputBuffers(upperOutputBuffers)) {
        RTT::log(RTT::Error) << "Could not create upper output buffers" << RTT::endlog();
        return false;
    }

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

    core_peers_ = dc_->getPeerList();

    RTT::log(RTT::Info) << "core components:" << RTT::endlog();
    for (int i = 0; i < core_peers_.size(); ++i) {
        RTT::log(RTT::Info) << "  " << core_peers_[i] << RTT::endlog();
    }

    Logger::log() << Logger::Info << "OK" << Logger::endl;

    return true;
}

bool SubsystemDeployer::isCorePeer(const std::string& name) {
    for (int i = 0; i < core_peers_.size(); ++i) {
        if (name == core_peers_[i]) {
            return true;
        }
    }
    return false;
}

bool SubsystemDeployer::configure() {
    std::vector< std::string > peer_names = dc_->getPeerList();

    for (int i = 0; i < peer_names.size(); ++i) {
        const std::string& name = peer_names[i];
        if (!isCorePeer(name)) {
            TaskContext* tc = dc_->getPeer(name);
            if (!setTriggerOnStart(tc, false)) {
                return false;
            }

            scheme_->addPeer(tc);

            RTT::OperationCaller<bool(const std::string&)> scheme_addBlock = scheme_->getOperation("addBlock");
            if (!scheme_addBlock.ready()) {
                Logger::log() << Logger::Error << "Could not get addBlock operation of Conman scheme" << Logger::endl;
                return false;
            }

            if (!scheme_addBlock(name)) {
                Logger::log() << Logger::Warning << "Could not add block to Conman scheme: " << name << Logger::endl;
                return true;
            }
        }
    }
/*
TODO
# set slave tasks execution order (before configure)
core_cs_command_rx.pushBackPeerExecution("master_component");
core_cs_command_rx.pushBackPeerExecution("scheme");
core_cs_command_rx.configure();
*/

    // master component can be configured after all peers are added to scheme
    master_component_->configure();

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

