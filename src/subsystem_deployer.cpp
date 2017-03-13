/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "subsystem_deployer/subsystem_deployer.h"

//#include "common_interfaces/message_concate.h"

#include <rtt/rtt-config.h>
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <ocl/TaskBrowser.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/base/ExecutableInterface.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <rtt_roscomm/rtt_rostopic.h>

#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>

#include "ros/ros.h"
#include <subsystem_msgs/GetSubsystemInfo.h>
#include <tinyxml.h>

using namespace RTT;
using namespace std;

// dummy component for rtt_dot_service
class GraphComponent: public RTT::TaskContext {
public:
    explicit GraphComponent(const std::string &name)
        : RTT::TaskContext(name, PreOperational) {
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
    }
};

class SubsystemDeployerRosService : public SubsystemDeployerRosServiceBase {
public:
    SubsystemDeployerRosService(const SubsystemDeployer& d)
            : d_(d)
            , ss_GetSubsystemInfo_( n_.advertiseService(d.getSubsystemName() + "/getSubsystemInfo", &SubsystemDeployerRosService::getSubsystemInfo, this) ) {
    }

    bool getSubsystemInfo(   subsystem_msgs::GetSubsystemInfo::Request  &req,
                             subsystem_msgs::GetSubsystemInfo::Response &res) {

		res.is_initialized = d_.isInitialized();

        for (int i = 0; i < d_.getLowerInputBuffers().size(); ++i) {
            res.lower_inputs.push_back(d_.getLowerInputBuffers()[i].getChannelName());
            res.lower_inputs_ipc.push_back(d_.getLowerInputBuffers()[i].enable_ipc_);
        }

        for (int i = 0; i < d_.getUpperInputBuffers().size(); ++i) {
            res.upper_inputs.push_back(d_.getUpperInputBuffers()[i].getChannelName());
            res.upper_inputs_ipc.push_back(d_.getUpperInputBuffers()[i].enable_ipc_);
        }

        for (int i = 0; i < d_.getLowerOutputBuffers().size(); ++i) {
            res.lower_outputs.push_back(d_.getLowerOutputBuffers()[i].getChannelName());
            res.lower_outputs_ipc.push_back(d_.getLowerOutputBuffers()[i].enable_ipc_);
        }

        for (int i = 0; i < d_.getUpperOutputBuffers().size(); ++i) {
            res.upper_outputs.push_back(d_.getUpperOutputBuffers()[i].getChannelName());
            res.upper_outputs_ipc.push_back(d_.getUpperOutputBuffers()[i].enable_ipc_);
        }

        std::vector<RTT::TaskContext* > components = d_.getAllComponents();

        for (int i = 0; i < components.size(); ++i) {
            RTT::TaskContext* tc = components[i];
            subsystem_msgs::ComponentInfo cinf;
            cinf.name = tc->getName();
            std::vector<RTT::base::PortInterface* > ports = tc->ports()->getPorts();
            for (int ip = 0; ip < ports.size(); ++ip) {
                subsystem_msgs::PortInfo pinf;

                pinf.name = ports[ip]->getName();

                pinf.is_connected = ports[ip]->connected();

                RTT::base::InputPortInterface* ipi;
                RTT::base::OutputPortInterface* opi;
                if (ipi = dynamic_cast<RTT::base::InputPortInterface* >(ports[ip])) {
                    pinf.is_input = true;
                }
                else if (opi = dynamic_cast<RTT::base::OutputPortInterface* >(ports[ip])) {
                    pinf.is_input = false;
                }

                const RTT::types::TypeInfo* ti = ports[ip]->getTypeInfo();
                std::vector<std::string > type_names = ti->getTypeNames();
                for (int itn = 0; itn < type_names.size(); ++itn) {
                    pinf.type_names.push_back(type_names[itn]);
                }

                cinf.ports.push_back(pinf);
            }
            res.components.push_back(cinf);
        }

//TODO: BehaviorInfo[] behaviors

        return true;
    }

private:
    const SubsystemDeployer& d_;
    ros::NodeHandle n_;
    ros::ServiceServer ss_GetSubsystemInfo_;
};

SubsystemDeployer::SubsystemDeployer(const std::string& name)
    : name_(name)
	, is_initialized_(false)
{
}

bool SubsystemDeployer::isInitialized() const {
	return is_initialized_;
}

const std::vector<common_behavior::InputBufferInfo >& SubsystemDeployer::getLowerInputBuffers() const {
    return lowerInputBuffers_;
}

const std::vector<common_behavior::InputBufferInfo >& SubsystemDeployer::getUpperInputBuffers() const {
    return upperInputBuffers_;
}

const std::vector<common_behavior::OutputBufferInfo >& SubsystemDeployer::getLowerOutputBuffers() const {
    return lowerOutputBuffers_;
}

const std::vector<common_behavior::OutputBufferInfo >& SubsystemDeployer::getUpperOutputBuffers() const {
    return upperOutputBuffers_;
}

const std::string& SubsystemDeployer::getSubsystemName() const {
    return full_name_;
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
//        << ", event_port: " << (info.event_port_?"true":"false")
        << ", interface_prefix: \'" << info.getChannelName() << "\'"
        << Logger::endl;
}

static void printOutputBufferInfo(const common_behavior::OutputBufferInfo& info) {
    Logger::log()
        << " enable_ipc: " << (info.enable_ipc_?"true":"false")
        << ", interface_prefix: \'" << info.getChannelName() << "\'"
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
    std::string suffix = "Rx";
    std::string type = buf_info.interface_type_ + suffix;
    std::string ch_name = buf_info.getChannelName();
    std::string name = ch_name + suffix;

    if (buf_info.enable_ipc_) {
        if (!dc_->loadComponent(name, type)) {
            RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
            return false;
        }
        RTT::TaskContext* comp = dc_->getPeer(name);
        if (!setTriggerOnStart(comp, true)) {
            return false;
        }
        if (!setComponentProperty<std::string >(comp, "channel_name", ch_name)) {
            return false;
        }

        if (!setComponentProperty<bool >(comp, "event", buf_info.event_)) {
            return false;
        }
        if (!setComponentProperty<double >(comp, "period_min", buf_info.period_min_)) {
            return false;
        }
        if (!setComponentProperty<double >(comp, "period_avg", buf_info.period_avg_)) {
            return false;
        }
        if (!setComponentProperty<double >(comp, "period_max", buf_info.period_max_)) {
            return false;
        }

        buffer_rx_components_.push_back(comp);

        return true;
    }
    RTT::log(RTT::Error) << "component " << type << " should not have IPC buffer" << RTT::endlog();
    return false;
}

bool SubsystemDeployer::deployOutputBufferIpcComponent(const common_behavior::OutputBufferInfo& buf_info) {
    std::string suffix = "Tx";
    std::string type = buf_info.interface_type_ + suffix;
    std::string ch_name = buf_info.getChannelName();
    std::string name = ch_name + suffix;

    if (buf_info.enable_ipc_) {
        if (!dc_->loadComponent(name, type)) {
            RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
            return false;
        }
        RTT::TaskContext* comp = dc_->getPeer(name);
        if (!setTriggerOnStart(comp, false)) {
            return false;
        }

        if (!setComponentProperty<std::string >(comp, "channel_name", ch_name)) {
            return false;
        }

        buffer_tx_components_.push_back(comp);

        return true;
    }
    RTT::log(RTT::Error) << "component " << type << " should not have IPC buffer" << RTT::endlog();
    return false;
}

bool SubsystemDeployer::deployBufferSplitComponent(const common_behavior::BufferInfo& buf_info) {
    std::string suffix = "Split";
    std::string type = buf_info.interface_type_ + suffix;
    std::string ch_name = buf_info.getChannelName();
    std::string name = ch_name + suffix;

    if (!dc_->loadComponent(name, type)) {
        RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
        return false;
    }
    RTT::TaskContext* comp = dc_->getPeer(name);
    if (!setTriggerOnStart(comp, false)) {
        return false;
    }

    buffer_split_components_.push_back(comp);

    return true;
}

bool SubsystemDeployer::deployBufferConcateComponent(const common_behavior::BufferInfo& buf_info) {
    std::string suffix = "Concate";
    std::string type = buf_info.interface_type_ + suffix;
    std::string ch_name = buf_info.getChannelName();
    std::string name = ch_name + suffix;

    if (!dc_->loadComponent(name, type)) {
        RTT::log(RTT::Error) << "Unable to load component " << type << RTT::endlog();
        return false;
    }
    RTT::TaskContext* comp = dc_->getPeer(name);
    if (!setTriggerOnStart(comp, false)) {
        return false;
    }

    buffer_concate_components_.push_back(comp);

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
            if (!dc_->connect(std::string("master_component.") + buf_info.getChannelName() + "_OUTPORT", buf_info.getChannelName() + "Split.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Split-Rx: " << buf_info.getChannelName() << RTT::endlog();
                return false;
            }

            // connect Rx to master_component
            if (!dc_->connect(buf_info.getChannelName() + "Rx.msg_OUTPORT", std::string("master_component.") + buf_info.getChannelName() + "_INPORT", ConnPolicy::data(ConnPolicy::LOCKED))) {
                RTT::log(RTT::Error) << "could not connect ports Rx-master_component: " << buf_info.getChannelName() << RTT::endlog();
                return false;
            }

            if (buf_info.event_) {
                // connect Rx no_data to master_component
                if (!dc_->connect(buf_info.getChannelName() + "Rx.no_data_OUTPORT", std::string("master_component.no_data_trigger_INPORT_"), ConnPolicy::data(ConnPolicy::LOCKED))) {
                    RTT::log(RTT::Error) << "could not connect ports Rx-master_component no_data: " << buf_info.getChannelName() << RTT::endlog();
                    return false;
                }
            }
        }
        else {
            // there is no Rx component, so create additional Concate component
            if (!deployBufferConcateComponent(buf_info)) {
                return false;
            }

            // connect Split-Concate ports
            if (!dc_->connect(buf_info.getChannelName() + "Concate.msg_OUTPORT", buf_info.getChannelName() + "Split.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Split-Concate: " << buf_info.getChannelName() << RTT::endlog();
                return false;
            }

            // connect Concate to master_component
            if (!dc_->connect(buf_info.getChannelName() + "Concate.msg_OUTPORT", std::string("master_component.") + buf_info.getChannelName() + "_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Concate-master_component: " << buf_info.getChannelName() << RTT::endlog();
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
            if (!dc_->connect(buf_info.getChannelName() + "Concate.msg_OUTPORT", buf_info.getChannelName() + "Tx.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Concate-Tx: " << buf_info.getChannelName() << RTT::endlog();
                return false;
            }
        }
        else {
            // there is no Tx component, so create additional Split component
            if (!deployBufferSplitComponent(buf_info)) {
                return false;
            }

            // connect Split-Concate ports
            if (!dc_->connect(buf_info.getChannelName() + "Concate.msg_OUTPORT", buf_info.getChannelName() + "Split.msg_INPORT", ConnPolicy())) {
                RTT::log(RTT::Error) << "could not connect ports Split-Concate: " << buf_info.getChannelName() << RTT::endlog();
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

bool SubsystemDeployer::initializeSubsystem(const std::string& master_package_name, const std::string& subsystem_subname) {
    Logger::In in("SubsystemDeployer::init");

    master_package_name_ = master_package_name;
	subname_ = subsystem_subname;
    full_name_ = master_package_name_ + subname_;

    ros_service.reset( new SubsystemDeployerRosService(*this) );

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
        !import("rtt_rosclock") ||
        !import("rtt_rospack") ||
        !import("rtt_actionlib") ||
        !import("common_behavior") ||
        !import("conman") ||
        !import("rtt_diagnostic_msgs") ||
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

    if (!master_component_) {
        Logger::log() << Logger::Error << "master_component in NULL" << Logger::endl;
        return false;
    }

    if (!setTriggerOnStart(master_component_, false)) {
        return false;
    }

    RTT::Property<std::string >* master_subsystem_subname = dynamic_cast<RTT::Property<std::string >* >(master_component_->getProperty("subsystem_subname"));
    if (!master_subsystem_subname) {
        RTT::log(RTT::Error) << "Could not get property \'subsystem_subname\' of master_component" << RTT::endlog();
        return false;
    }
    master_subsystem_subname->set(subsystem_subname);

    //
    // load subsystem-specific master service
    //
    if (!import(master_package_name)) {
        return false;
    }

    if (!master_component_->loadService(master_package_name + "_master")) {
        Logger::log() << Logger::Error << "Could not load service \'" << master_package_name << "_master\'" << Logger::endl;
        return false;
    }

    RTT::OperationCaller<bool(RTT::TaskContext*)> master_component_addConmanScheme = master_component_->getOperation("addConmanScheme");
    if (!master_component_addConmanScheme.ready()) {
        Logger::log() << Logger::Error << "Could not get addConmanScheme operation of Master Component" << Logger::endl;
        return false;
    }

    if (!master_component_addConmanScheme(scheme_)) {
        Logger::log() << Logger::Error << "Could not add Conman Scheme to Master Component" << Logger::endl;
        return false;
    }

    //
    // manage ports and ipc buffers
    //
    std::vector<std::string > master_port_names = master_component_->ports()->getPortNames();
    for (int i = 0; i < master_port_names.size(); ++i) {
        Logger::log() << Logger::Info << "master_component port[" << i << "]: " << master_port_names[i] << Logger::endl;
    }

    master_service_ = master_component_->getProvider<common_behavior::MasterServiceRequester >("master");
    if (!master_service_) {
        RTT::log(RTT::Error) << "Unable to load common_behavior::MasterService from master_component" << RTT::endlog();
        return false;
    }

    // inputs
    master_service_->getLowerInputBuffers(lowerInputBuffers_);
    master_service_->getUpperInputBuffers(upperInputBuffers_);

    Logger::log() << Logger::Info << "lowerInputBuffers:" << Logger::endl;
    for (int i = 0; i < lowerInputBuffers_.size(); ++i) {
        printInputBufferInfo(lowerInputBuffers_[i]);
    }

    Logger::log() << Logger::Info << "upperInputBuffers:" << Logger::endl;
    for (int i = 0; i < upperInputBuffers_.size(); ++i) {
        printInputBufferInfo(upperInputBuffers_[i]);
    }

    if (!createInputBuffers(lowerInputBuffers_)) {
        RTT::log(RTT::Error) << "Could not create lower input buffers" << RTT::endlog();
        return false;
    }

    if (!createInputBuffers(upperInputBuffers_)) {
        RTT::log(RTT::Error) << "Could not create upper input buffers" << RTT::endlog();
        return false;
    }

    // outputs
    master_service_->getLowerOutputBuffers(lowerOutputBuffers_);
    master_service_->getUpperOutputBuffers(upperOutputBuffers_);

    Logger::log() << Logger::Info << "lowerOutputBuffers:" << Logger::endl;
    for (int i = 0; i < lowerOutputBuffers_.size(); ++i) {
        printOutputBufferInfo(lowerOutputBuffers_[i]);
    }

    Logger::log() << Logger::Info << "upperOutputBuffers:" << Logger::endl;
    for (int i = 0; i < upperOutputBuffers_.size(); ++i) {
        printOutputBufferInfo(upperOutputBuffers_[i]);
    }

    if (!createOutputBuffers(lowerOutputBuffers_)) {
        RTT::log(RTT::Error) << "Could not create lower output buffers" << RTT::endlog();
        return false;
    }

    if (!createOutputBuffers(upperOutputBuffers_)) {
        RTT::log(RTT::Error) << "Could not create upper output buffers" << RTT::endlog();
        return false;
    }

    //
    // diagnostics ROS interface
    //
    dc_->loadComponent("diag","DiagnosticComponent");
    diag_component_ = dc_->getPeer("diag");
    if (!diag_component_->setPeriod(0.01)) {
        RTT::log(RTT::Error) << "could not change period of component \'" << diag_component_->getName() << RTT::endlog();
        return false;
    }

//TODO:
//    diag_component_.loadService("sim_clock_activity");

    RTT::base::PortInterface* diag_port_out_ = diag_component_->ports()->getPort("diag_OUTPORT");
    if (diag_port_out_) {
        if (!diag_port_out_->createStream(rtt_roscomm::topic(std::string("/") + getSubsystemName() + "/diag"))) {
            RTT::log(RTT::Error) << "could not create ROS stream for port \'" << diag_component_->getName() << "." << diag_port_out_->getName() << "\'" << RTT::endlog();
            return false;
        }
    }
    else {
        RTT::log(RTT::Error) << "component \'" << diag_component_->getName() << "\' does not have port \'diag_OUTPORT\'" << RTT::endlog();
        return false;
    }

    Logger::log() << Logger::Info << "OK" << Logger::endl;

    RTT::log(RTT::Info) << "Master Component ports:" << RTT::endlog();
    std::vector<std::string > port_names = master_component_->ports()->getPortNames();
    for (int  i = 0; i < port_names.size(); ++i) {
        RTT::log(RTT::Info) << "  " << port_names[i] << RTT::endlog();
    }

    return true;
}

std::vector<RTT::TaskContext* > SubsystemDeployer::getAllComponents() const {
    std::vector<RTT::TaskContext* > result;

    std::vector< std::string > peer_names = dc_->getPeerList();
    for (int i = 0; i < peer_names.size(); ++i) {
        result.push_back(dc_->getPeer(peer_names[i]));
    }

    return result;
}

std::vector<RTT::TaskContext* > SubsystemDeployer::getCoreComponents() const {
    std::vector<RTT::TaskContext* > result;
    result.push_back(master_component_);
    result.push_back(scheme_);
    result.push_back(diag_component_);
    result.insert(result.end(), buffer_rx_components_.begin(), buffer_rx_components_.end());
    result.insert(result.end(), buffer_tx_components_.begin(), buffer_tx_components_.end());
    result.insert(result.end(), buffer_split_components_.begin(), buffer_split_components_.end());
    result.insert(result.end(), buffer_concate_components_.begin(), buffer_concate_components_.end());
    return result;
}

std::vector<RTT::TaskContext* > SubsystemDeployer::getNonCoreComponents() const {
    std::vector<RTT::TaskContext* > result;
    std::vector<RTT::TaskContext* > core = getCoreComponents();
    
    std::vector< std::string > peer_names = dc_->getPeerList();

    for (int i = 0; i < peer_names.size(); ++i) {
        const std::string& name = peer_names[i];
        bool is_core = false;
        for (int j = 0; j < core.size(); ++j) {
            if (core[j]->getName() == name) {
                is_core = true;
                break;
            }
        }
        if (!is_core) {
            TaskContext* tc = dc_->getPeer(name);
            result.push_back(tc);
        }
    }
    return result;
}

bool SubsystemDeployer::isInputPort(const std::string &path) const {
    size_t first_dot = path.find(".");
    if (first_dot == std::string::npos) {
        return false;
    }

    TaskContext* tc = dc_->getPeer( path.substr(0, first_dot) );
    if (!tc) {
        return false;
    }
    RTT::base::PortInterface *pi = tc->ports()->getPort( path.substr(first_dot+1, std::string::npos) );
    if (dynamic_cast<RTT::base::InputPortInterface*>(pi)) {
        return true;
    }

    return false;
}

bool SubsystemDeployer::isOutputPort(const std::string &path) const {
    size_t first_dot = path.find(".");
    if (first_dot == std::string::npos) {
        return false;
    }

    TaskContext* tc = dc_->getPeer( path.substr(0, first_dot) );
    if (!tc) {
        return false;
    }
    RTT::base::PortInterface *pi = tc->ports()->getPort( path.substr(first_dot+1, std::string::npos) );
    if (dynamic_cast<RTT::base::OutputPortInterface*>(pi)) {
        return true;
    }

    return false;
}

bool SubsystemDeployer::configure() {
    Logger::In in("SubsystemDeployer::configure");

    // disable Trigger On Start for all components
    const std::vector<RTT::TaskContext* > all_components = getAllComponents();
    for (int i = 0; i < all_components.size(); ++i) {
        setTriggerOnStart(all_components[i], false);
    }

    // initialize conman scheme
    RTT::OperationCaller<bool(const std::string&)> scheme_addBlock = scheme_->getOperation("addBlock");
    if (!scheme_addBlock.ready()) {
        Logger::log() << Logger::Error << "Could not get addBlock operation of Conman scheme" << Logger::endl;
        return false;
    }
    std::vector<RTT::TaskContext* > conman_peers = getNonCoreComponents();
    conman_peers.insert(conman_peers.end(), buffer_tx_components_.begin(), buffer_tx_components_.end());
    conman_peers.insert(conman_peers.end(), buffer_split_components_.begin(), buffer_split_components_.end());
    conman_peers.insert(conman_peers.end(), buffer_concate_components_.begin(), buffer_concate_components_.end());
    std::vector<bool > conman_peers_running(conman_peers.size(), false);
    for (int i = 0; i < conman_peers.size(); ++i) {
        if (conman_peers[i]->isRunning()) {
            conman_peers[i]->stop();
            conman_peers_running[i] = true;
        }
        scheme_->addPeer(conman_peers[i]);
        if (!scheme_addBlock(conman_peers[i]->getName())) {
            Logger::log() << Logger::Warning << "Could not add block to Conman scheme: " << conman_peers[i]->getName() << Logger::endl;
            return true;
        }
    }

    RTT::OperationCaller<bool(const std::string&, const std::string&, const bool) > scheme_latchConnections = scheme_->getOperation("latchConnections");
    if (!scheme_latchConnections.ready()) {
        Logger::log() << Logger::Error << "Could not get getFlowCycles operation of Conman scheme" << Logger::endl;
        return false;
    }

    const std::vector<std::pair<std::string, std::string > >& latched_connections = master_service_->getLatchedConnections();
    for (int i = 0; i < latched_connections.size(); ++i) {
        scheme_latchConnections(latched_connections[i].first, latched_connections[i].second, true);
    }

    const std::vector<RTT::TaskContext* > core_components = getCoreComponents();
    const std::vector<RTT::TaskContext* > non_core_components = getNonCoreComponents();

    // add all peers to diagnostics component
    for (int i = 0; i < all_components.size(); ++i) {
        if (all_components[i]->getName() != diag_component_->getName()) {
            diag_component_->addPeer(all_components[i]);
        }
    }

    Logger::log() << Logger::Info << "[before master_component configure] scheme_->getActivity(): "
        << (scheme_->getActivity()) << Logger::endl;

    // configure unconfigured core peers
    // master component can be configured after all peers are added to scheme
    for (int i = 0; i < core_components.size(); ++i) {
        if (!core_components[i]->isConfigured()) {
            if (!core_components[i]->configure()) {
                RTT::log(RTT::Error) << "Unable to configure component " << core_components[i]->getName() << RTT::endlog();
                return false;
            }
        }
    }

    // try connecting ports before components configuration
    for (std::list<std::pair<std::string, std::string> >::iterator it = connections_.begin(); it != connections_.end(); ) {
        if (!isInputPort(it->second) || !isOutputPort(it->first)) {
            ++it;
            continue;
        }

        if (dc_->connect(it->first, it->second, ConnPolicy::data(ConnPolicy::LOCKED))) {
            connections_.erase(it++);
        }
        else {
            ++it;
        }
    }

    // load orocos services
    for (int i = 0; i < non_core_components.size(); ++i) {
        RTT::TaskContext *tc = non_core_components[i];
        std::map<std::string, std::vector<std::string> >::iterator it = component_services_.find(tc->getName());
        if (it != component_services_.end()) {
            for (int j = 0; j < it->second.size(); ++j) {
                if (!tc->loadService( it->second[j] )) {
                    RTT::log(RTT::Error) << "Could not load service \'" << it->second[j]
                        << "\' for component \'" << tc->getName() << "\'" << RTT::endlog();
                    return false;
                }
            }
        }
    }

    // configure other unconfigured peers
    for (int i = 0; i < non_core_components.size(); ++i) {
        loadROSParam(non_core_components[i]);
        if (!non_core_components[i]->isConfigured()) {
            RTT::log(RTT::Info) << "loading ROS parameters for \'" << non_core_components[i]->getName() << "\'" << RTT::endlog();
            if (!non_core_components[i]->configure()) {
                RTT::log(RTT::Error) << "Unable to configure component " << non_core_components[i]->getName() << RTT::endlog();
                return false;
            }
        }
        else {
            RTT::log(RTT::Info) << "component \'" << non_core_components[i]->getName() << "\' is already configured" << RTT::endlog();
        }
    }

    // connect remaining ports
    for (std::list<std::pair<std::string, std::string> >::iterator it = connections_.begin(); it != connections_.end(); ++it) {
        if (!isInputPort(it->second)) {
            RTT::log(RTT::Error) << "port \'" << it->second << "\' is not an input port" << RTT::endlog();
            return false;
        }

        if (!isOutputPort(it->first)) {
            RTT::log(RTT::Error) << "port \'" << it->first << "\' is not an output port" << RTT::endlog();
            return false;
        }

        if (!dc_->connect(it->first, it->second, ConnPolicy::data(ConnPolicy::LOCKED))) {
            RTT::log(RTT::Error) << "Unable to connect \'" << it->first << "\' and \'" << it->second << "\'" << RTT::endlog();
            return false;
        }
    }

    // connect ROS topics
    for (std::list<std::pair<std::string, std::string> >::iterator it = ros_streams_.begin(); it != ros_streams_.end(); ++it) {
        // TODO: check if from/to is input/output
        if (!dc_->stream(it->first, rtt_roscomm::topic(it->second))) {
            RTT::log(RTT::Error) << "Unable to connect \'" << it->first << "\' and \'" << it->second << "\'" << RTT::endlog();
            return false;
        }
    }

    // connect ROS actions
    for (int i = 0; i < conman_peers.size(); ++i) {
        std::map<std::string, std::string>::const_iterator it = components_ros_action_.find( conman_peers[i]->getName() );
        if (it != components_ros_action_.end()) {
            if (!conman_peers[i]->loadService("actionlib")) {
                RTT::log(RTT::Error) << "Could not load service \'actionlib\' for component \'"
                    << it->first << "\', action \'" << it->second << "\'" << RTT::endlog();
                return false;
            }

            RTT::Service::shared_ptr actionlib_service = conman_peers[i]->provides("actionlib");
            if (!actionlib_service) {
                RTT::log(RTT::Error) << "Could not get service \'actionlib\' of component \'"
                    << it->first << "\', action \'" << it->second << "\'" << RTT::endlog();
                return false;
            }
            RTT::OperationCaller<bool(const std::string&)> connect_action = actionlib_service->getOperation("connect");
            if (!connect_action.ready()) {
                RTT::log(RTT::Error) << "Could not get operation \'connect\' of action_service of component \'"
                    << it->first << "\', action \'" << it->second << "\'" << RTT::endlog();
                return false;
            }

            if (!connect_action(it->second)) {
                RTT::log(RTT::Error) << "Could not connect action \'" << it->second << "\' to action server" << RTT::endlog();
                return false;
            }
        }
    }

    //
    // load rtt_dot_service
    //
    {
        boost::shared_ptr<GraphComponent > gc( new GraphComponent("graph_component"));

        for (int i = 0; i < all_components.size(); ++i) {
            gc->addPeer( all_components[i] );
        }

        if (!import("rtt_dot_service")) {
            Logger::log() << Logger::Warning << "Could not load rtt_dot_service. Graph generation is disabled." << Logger::endl;
        }
        else if (!gc->loadService("dot")) {
            Logger::log() << Logger::Warning << "Could not load rtt_dot_service \'dot\'. Graph generation is disabled." << Logger::endl;
        }
        else {
            dot_graph_service_ = gc->provides("dot");
            if (!dot_graph_service_) {
                Logger::log() << Logger::Error << "Could not get loaded rtt_dot_service \'dot\'." << Logger::endl;
                return false;
            }
            Logger::log() << Logger::Info << "Loaded rtt_dot_service \'dot\'. Graph generation is enabled." << Logger::endl;
        }

        if (dot_graph_service_) {
            RTT::Property<std::string >* dot_file = dynamic_cast<RTT::Property<std::string >* >(dot_graph_service_->getProperty("dot_file"));
            if (!dot_file) {
                RTT::log(RTT::Error) << "Could not get property \'dot_graph\' of rtt_dot_service" << RTT::endlog();
                return false;
            }
            dot_file->set(std::string("/tmp/") + getSubsystemName() + ".dot");

            RTT::OperationCaller<bool()> generate_graph = dot_graph_service_->getOperation("generate");
            if (!generate_graph.ready()) {
                RTT::log(RTT::Error) << "Could not get operation \'generate\' of rtt_dot_service" << RTT::endlog();
                return false;
            }

            if (!generate_graph()) {
                RTT::log(RTT::Error) << "Could not generate graph with rtt_dot_service.generate()" << RTT::endlog();
                return false;
            }

            if (gc->provides()->hasService("dot")) {
                gc->provides()->removeService("dot");
                if (gc->provides()->hasService("dot")) {
                    RTT::log(RTT::Error) << "Could not remove dot service from master component." << RTT::endlog();
                    return false;
                }
            }
            else {
                RTT::log(RTT::Error) << "Could not find dot service in master component." << RTT::endlog();
                return false;
            }
            RTT::log(RTT::Info) << "Generated graph." << RTT::endlog();
        }
    }

    //
    // remove unused ports from msg concate/split components
    //
/*    for (int i = 0; i < core_components.size(); ++i) {
        RTT::TaskContext* tc = core_components[i];

        RTT::OperationInterfacePart *removeUnconnectedPortsOp;
        if (tc->provides()->hasOperation("removeUnconnectedPorts") && (removeUnconnectedPortsOp = tc->provides()->getOperation("removeUnconnectedPorts")) != NULL) {
            RTT::OperationCaller<bool()> removeUnconnectedPorts = RTT::OperationCaller<bool()>(removeUnconnectedPortsOp, tc->engine());
            if (removeUnconnectedPorts.ready()) {
                size_t before = tc->ports()->getPorts().size();
                removeUnconnectedPorts();
                size_t after = tc->ports()->getPorts().size();
                RTT::log(RTT::Info) << "Removed unconnected ports of " << tc->getName() << ": reduced from " << before << " to " << after << RTT::endlog();
            }
            else {
                RTT::log(RTT::Warning) << "Could not removeUnconnectedPorts() of " << tc->getName() << RTT::endlog();
            }
        }
    }
*/

    // start conman scheme first
    if (!scheme_->start()) {
        RTT::log(RTT::Error) << "Unable to start component: " << scheme_->getName() << RTT::endlog();
        return false;
    }

    if (scheme_->getTaskState() != RTT::TaskContext::Running) {
        RTT::log(RTT::Error) << "Component is not in the running state: " << scheme_->getName() << RTT::endlog();
        return false;
    }

    // start other core peers
    for (int i = 0; i < core_components.size(); ++i) {
        if (!core_components[i]->isRunning()) {
            if (!core_components[i]->start()) {
                RTT::log(RTT::Error) << "Unable to start component " << core_components[i]->getName() << RTT::endlog();
                return false;
            }
        }
    }

    // start non-core components that should always run
    for (int i = 0; i < conman_peers.size(); ++i) {
        std::set<std::string>::const_iterator it = components_initially_running_.find( conman_peers[i]->getName() );
        if (it != components_initially_running_.end() || conman_peers_running[i]) {
            conman_peers[i]->start();
        }
    }

    RTT::Activity* master_activity = dynamic_cast<RTT::Activity* >(master_component_->getActivity());
    if (!master_activity) {
        Logger::log() << Logger::Warning << "Could not set scheduler for Master Component to ORO_SCHED_RT. Could not get Activity." << Logger::endl;
    }
    else if (!master_activity->setScheduler(ORO_SCHED_RT)) {
        Logger::log() << Logger::Warning << "Could not set scheduler for Master Component to ORO_SCHED_RT." << Logger::endl;
    }

    // trigger all input buffers components
    for (int i = 0; i < buffer_rx_components_.size(); ++i) {
        buffer_rx_components_[i]->trigger();
    }

    Logger::log() << Logger::Info << "OK" << Logger::endl;

	is_initialized_ = true;

    return true;
}

bool SubsystemDeployer::runXmls(const std::vector<std::string>& xmlFiles) {
//    using namespace tinyxml2;
    for (std::vector<std::string>::const_iterator iter=xmlFiles.begin();
        iter!=xmlFiles.end();
        ++iter)
    {
        Logger::In in(std::string("SubsystemDeployer::runXmls") + (*iter));
        TiXmlDocument doc;
        doc.LoadFile( iter->c_str() );
        TiXmlElement *root = doc.RootElement();
        if (!root) {
            Logger::log() << Logger::Error << "no root element" << Logger::endl;
            return false;
        }
        if (strcmp(root->Value(), "subsystem") != 0) {
            Logger::log() << Logger::Error << "wrong root element: \'" << std::string(root->Value())
                << "\', should be: \'subsystem\'" << Logger::endl;
            return false;
        }

        const TiXmlElement *import_elem = root->FirstChildElement("import");
        while (import_elem) {
            const char *import_text = import_elem->GetText();
            if (!import_text) {
                Logger::log() << Logger::Error << "wrong value of \'import\' element" << Logger::endl;
                return false;
            }
            if (!import(import_text)) {
                Logger::log() << Logger::Error << "could not import \'" << std::string(import_text) << "\'" << Logger::endl;
                return false;
            }
            import_elem = import_elem->NextSiblingElement("import");
        }

        const TiXmlElement *component_elem = root->FirstChildElement("component");
        while (component_elem) {
            const char *type_attr = component_elem->Attribute("type");
            const char *name_attr = component_elem->Attribute("name");
            const char *running_attr = component_elem->Attribute("running");
            const char *ros_action_attr = component_elem->Attribute("ros_action");

            if (!dc_->loadComponent(name_attr, type_attr)) {
                RTT::log(RTT::Error) << "Unable to load component \'" << std::string(name_attr)
                    << "\' of type \'" << std::string(type_attr) << "\'" << RTT::endlog();
                return false;
            }

            if (running_attr && (strcmp(running_attr, "true") == 0 || strcmp(running_attr, "True") == 0 || strcmp(running_attr, "TRUE") == 0)) {
                components_initially_running_.insert( std::string(name_attr) );
            }

            if (ros_action_attr) {
                components_ros_action_.insert( std::make_pair<std::string, std::string >(std::string(name_attr), std::string(ros_action_attr)) );
            }

            const TiXmlElement *service_elem = root->FirstChildElement("service");
            while (service_elem) {
                const char *service_text = service_elem->GetText();
                if (!service_text) {
                    RTT::log(RTT::Error) << "wrong service definition for component\'" << std::string(name_attr) << "\'" << RTT::endlog();
                    return false;
                }

                std::map<std::string, std::vector<std::string> >::iterator it = component_services_.find(name_attr);
                if (it == component_services_.end()) {
                    it = component_services_.insert( std::make_pair<std::string, std::vector<std::string>>(std::string(name_attr), std::vector<std::string>()) ).first;
                }
                it->second.push_back( std::string(service_text) );
                service_elem = service_elem->NextSiblingElement("service");
            }

            component_elem = component_elem->NextSiblingElement("component");
        }

        const TiXmlElement *connection_elem = root->FirstChildElement("connection");
        while (connection_elem) {
            const char *from_attr = connection_elem->Attribute("from");
            const char *to_attr = connection_elem->Attribute("to");

            if (from_attr && to_attr) {
                connections_.push_back( std::make_pair<std::string, std::string >(std::string(from_attr), std::string(to_attr)) );
            }
            else {
                RTT::log(RTT::Error) << "wrong connection definition: missing \'from\' or \'to\' attribute" << RTT::endlog();
                return false;
            }

            connection_elem = connection_elem->NextSiblingElement("connection");
        }

        const TiXmlElement *ros_stream_elem = root->FirstChildElement("ros_stream");
        while (ros_stream_elem) {
            const char *port_attr = ros_stream_elem->Attribute("port");
            const char *topic_attr = ros_stream_elem->Attribute("topic");

            if (port_attr && topic_attr) {
                ros_streams_.push_back( std::make_pair<std::string, std::string >(std::string(port_attr), std::string(topic_attr)) );
            }
            else {
                RTT::log(RTT::Error) << "wrong ros_stream definition: missing \'port\' or \'topic\' attribute" << RTT::endlog();
                return false;
            }
            ros_stream_elem = ros_stream_elem->NextSiblingElement("ros_stream");
        }

    }
    return true;
}

bool SubsystemDeployer::runScripts(const std::vector<std::string>& scriptFiles) {
    /******************** WARNING ***********************
     *   NO log(...) statements before __os_init() !!!!! 
     ***************************************************/
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
            return result;
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

