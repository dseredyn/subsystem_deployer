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

#ifndef COMMON_BEHAVIOR_SUBSYSTEM_DEPLOYER_H_
#define COMMON_BEHAVIOR_SUBSYSTEM_DEPLOYER_H_

#include <vector>
#include <set>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ocl/DeploymentComponent.hpp>
#include "common_behavior/master_service_requester.h"
#include "common_behavior/master_service.h"

class SubsystemDeployerRosServiceBase {};

class SubsystemDeployer {
public:
    explicit SubsystemDeployer(const std::string& name);

    bool import(const std::string& name);

    bool initializeSubsystem(const std::string& master_package_name, const std::string& subsystem_subname=std::string());

    bool runXmls(const std::vector<std::string>& xmlFiles);
    bool runScripts(const std::vector<std::string>& scriptFiles);

    bool runTaskBrowser();

    boost::shared_ptr<OCL::DeploymentComponent >& getDc();

    bool configure();

    const std::vector<common_behavior::InputBufferInfo >& getLowerInputBuffers() const;
    const std::vector<common_behavior::InputBufferInfo >& getUpperInputBuffers() const;
    const std::vector<common_behavior::OutputBufferInfo >& getLowerOutputBuffers() const;
    const std::vector<common_behavior::OutputBufferInfo >& getUpperOutputBuffers() const;

    const std::string& getChannelName(const std::string& alias) const;

    const std::string& getSubsystemName() const;

    std::vector<RTT::TaskContext* > getAllComponents() const;

    bool isInitialized() const;

    const std::string& getConnectionName(const std::string& from, const std::string& to) const;
    const std::string& getConnectionNameLatex(const std::string& from, const std::string& to) const;
    const std::string& getComponentNameLatex(const std::string& name) const;

    bool isConverter(const std::string& name) const;

private:

    RTT::base::PortInterface* strToPort(const std::string &path) const;
    bool connectPorts(const std::string& from, const std::string& to, const RTT::ConnPolicy& conn);
    bool connectionExists(const std::string& from, const std::string& to) const;

    bool deployInputBufferIpcComponent(const common_behavior::InputBufferInfo& buf_info);
    bool deployOutputBufferIpcComponent(const common_behavior::OutputBufferInfo& buf_info);
    bool deployBufferSplitComponent(const common_behavior::BufferInfo& buf_info);
    bool deployBufferConcateComponent(const common_behavior::BufferInfo& buf_info);
    bool createInputBuffers(const std::vector<common_behavior::InputBufferInfo >& buffers);
    bool createOutputBuffers(const std::vector<common_behavior::OutputBufferInfo >& buffers);
    bool isInputPort(const std::string &path) const;
    bool isOutputPort(const std::string &path) const;
    bool isSubsystemBuffer(const std::string& port_name) const;
    bool isSubsystemOutput(const std::string& port_name) const;

    bool setChannelsNames();

    std::vector<RTT::TaskContext* > getCoreComponents() const;
    std::vector<RTT::TaskContext* > getNonCoreComponents() const;

    bool setTriggerOnStart(RTT::TaskContext* tc, bool trigger);

    RTT::TaskContext* scheme_;
    RTT::TaskContext* master_component_;
    RTT::TaskContext* diag_component_;

    bool use_sim_time_;

    std::string name_;
    std::string master_package_name_;
    std::string subname_;
    std::string full_name_;


    boost::shared_ptr<OCL::DeploymentComponent > dc_;
    RTT::OperationCaller<bool(const std::string&)> ros_import_;
    boost::shared_ptr<common_behavior::MasterServiceRequester > master_service_;

    std::vector<RTT::TaskContext* > converter_components_;

    std::vector<RTT::TaskContext* > buffer_rx_components_;
    std::vector<RTT::TaskContext* > buffer_tx_components_;
    std::vector<RTT::TaskContext* > buffer_split_components_;
    std::vector<RTT::TaskContext* > buffer_concate_components_;

    std::vector<common_behavior::InputBufferInfo > lowerInputBuffers_;
    std::vector<common_behavior::InputBufferInfo > upperInputBuffers_;
    std::vector<common_behavior::OutputBufferInfo > lowerOutputBuffers_;
    std::vector<common_behavior::OutputBufferInfo > upperOutputBuffers_;

    boost::shared_ptr<SubsystemDeployerRosServiceBase > ros_service;

    class Connection {
    public:
        Connection(const std::string& from, const std::string& to, const std::string& name, const std::string& latex) {
            this->from = from;
            this->to = to;
            this->name = name;
            this->latex = latex;
        }
        std::string from, to, name, latex;
    };
    std::set<std::string > components_initially_running_;
    std::map<std::string, std::string > components_ros_action_;
    std::map<std::string, std::string > component_names_latex_;
    std::list<Connection > connections_;
    std::map<std::string, std::string > io_buffers_;
    std::list<std::pair<std::string, std::string> > ros_streams_;
    std::map<std::string, std::vector<std::string> > component_services_;

    std::vector<std::pair<std::string, std::string > > latched_connections_;

    bool is_initialized_;
};

#endif  // COMMON_BEHAVIOR_SUBSYSTEM_DEPLOYER_H_

