/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: update_timer.h
* Author: 
* Version: 1.0
* Date: 10-05-2017
* Operating Environment: 
* Compiler with Version Number: 
* Description: 
*

* List of functions used: 
* Revisers Name:
* Date:
* Customer Bug No./ CMF No. :
* Brief description of the fix/enhancement: Replacement of constant numbers with macros. Deleting commented code.
* Created by Tata Elxsi Ltd., < Automotive Group >
* Copyright <Year>Tata Elxsi Ltd.
* All rights reserved.
This code contains information that is proprietary to Tata Elxsi Ltd.
No part of this document/code may be reproduced or used in whole or
part in any form or by any means - graphic, electronic or mechanical
without the written permission of Tata Elxsi Ltd
**********************************************************************/

#ifndef GAZEBO_PLUGINS_UPDATE_TIMER_H
#define GAZEBO_PLUGINS_UPDATE_TIMER_H

#include <sdf/sdf.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>

namespace gazebo {

class UpdateTimer {
public:
  UpdateTimer()
    : connection_count_(0)
  {
  }

  virtual ~UpdateTimer()
  {
  }

  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf, const std::string& _prefix = "update")
  {
    this->world_ = _world;

    if (_sdf->HasElement(_prefix + "Rate")) {
      double update_rate = 0.0;
      _sdf->GetElement(_prefix + "Rate")->GetValue()->Get(update_rate);
      update_period_ = update_rate > 0.0 ? 1.0/update_rate : 0.0;
    }

    if (_sdf->HasElement(_prefix + "Period")) {
      _sdf->GetElement(_prefix + "Period")->GetValue()->Get(update_period_);
    }

    if (_sdf->HasElement(_prefix + "Offset")) {
      _sdf->GetElement(_prefix + "Offset")->GetValue()->Get(update_offset_);
    }
  }

  virtual event::ConnectionPtr Connect(const boost::function<void()> &_subscriber, bool connectToWorldUpdateBegin = true)
  {
    if (connectToWorldUpdateBegin && !update_connection_) {
      update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&UpdateTimer::Update, this));
    }
    connection_count_++;
    return update_event_.Connect(_subscriber);
  }

  virtual void Disconnect(event::ConnectionPtr const& _c = event::ConnectionPtr())
  {
    if (_c) update_event_.Disconnect(_c);

    if (update_connection_ && (!_c || --connection_count_ == 0)) {
      event::Events::DisconnectWorldUpdateBegin(update_connection_);
      update_connection_.reset();
    }
  }

  common::Time const& getUpdatePeriod() const {
    return update_period_;
  }

  void setUpdatePeriod(common::Time const& period) {
    update_period_ = period;
  }

  double getUpdateRate() const {
    double period = update_period_.Double();
    return (period > 0.0)  ? (1.0 / period) : 0.0;
  }

  void setUpdateRate(double rate) {
    update_period_ = (rate > 0.0) ? (1.0 / rate) : 0.0;
  }

  common::Time const& getLastUpdate() const {
    return last_update_;
  }

  common::Time getTimeSinceLastUpdate() const {
    if (last_update_ == common::Time()) return common::Time();
    return world_->GetSimTime() - last_update_;
  }

  virtual bool checkUpdate() const
  {
    double period = update_period_.Double();
    double step = world_->GetPhysicsEngine()->GetMaxStepSize();
    if (period == 0) return true;
    double fraction = fmod((world_->GetSimTime() - update_offset_).Double() + (step / 2.0), period);
    return (fraction >= 0.0) && (fraction < step);
  }

  virtual bool update()
  {
    if (!checkUpdate()) return false;
    last_update_ = world_->GetSimTime();
    return true;
  }

  virtual bool update(double& dt)
  {
    dt = getTimeSinceLastUpdate().Double();
    return update();
  }

  virtual void Reset()
  {
    last_update_ = common::Time();
  }

protected:
  virtual bool Update()
  {
    if (!checkUpdate()) {
      return false;
    }
    update_event_();
    last_update_ = world_->GetSimTime();
    return true;
  }

private:
  physics::WorldPtr world_;
  common::Time update_period_;
  common::Time update_offset_;
  common::Time last_update_;

  event::EventT<void()> update_event_;
  unsigned int connection_count_;
  event::ConnectionPtr update_connection_;
};

} // namespace gazebo

#endif // GAZEBO_PLUGINS_UPDATE_TIMER_H
