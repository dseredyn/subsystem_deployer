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
#include <string>
#include <boost/shared_ptr.hpp>
#include <ocl/DeploymentComponent.hpp>
#include "common_behavior/master_service_requester.h"
#include "common_behavior/master_service.h"

class SubsystemDeployer {
public:
    explicit SubsystemDeployer(const std::string& name);

    bool import(const std::string& name);

    bool initializeSubsystem(const std::string& master_package_name);

    void runScripts(const std::vector<std::string>& scriptFiles);

    bool runTaskBrowser();

    boost::shared_ptr<OCL::DeploymentComponent >& getDc();

    bool configure();

private:

    bool deployInputBufferIpcComponent(const common_behavior::InputBufferInfo& buf_info);
    bool deployOutputBufferIpcComponent(const common_behavior::OutputBufferInfo& buf_info);
    bool deployBufferSplitComponent(const common_behavior::BufferInfo& buf_info);
    bool deployBufferConcateComponent(const common_behavior::BufferInfo& buf_info);
    bool createInputBuffers(const std::vector<common_behavior::InputBufferInfo >& buffers);
    bool createOutputBuffers(const std::vector<common_behavior::OutputBufferInfo >& buffers);

    std::vector<RTT::TaskContext* > getAllComponents() const;
    std::vector<RTT::TaskContext* > getCoreComponents() const;
    std::vector<RTT::TaskContext* > getNonCoreComponents() const;

    bool setTriggerOnStart(RTT::TaskContext* tc, bool trigger);

    RTT::TaskContext* scheme_;
    RTT::TaskContext* master_component_;
    RTT::TaskContext* diag_component_;

    std::string name_;
    boost::shared_ptr<OCL::DeploymentComponent > dc_;
    RTT::OperationCaller<bool(const std::string&)> ros_import_;
    boost::shared_ptr<common_behavior::MasterServiceRequester > master_service_;

    std::vector<RTT::TaskContext* > buffer_rx_components_;
    std::vector<RTT::TaskContext* > buffer_tx_components_;
    std::vector<RTT::TaskContext* > buffer_split_components_;
    std::vector<RTT::TaskContext* > buffer_concate_components_;
};

#endif  // COMMON_BEHAVIOR_SUBSYSTEM_DEPLOYER_H_

