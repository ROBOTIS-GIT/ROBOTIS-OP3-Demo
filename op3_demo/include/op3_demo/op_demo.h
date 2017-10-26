/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef OP_DEMO_H_
#define OP_DEMO_H_

namespace robotis_op
{

class OPDemo
{
 public:
  enum Motion_Index
  {
    InitPose = 1,
    WalkingReady = 9,
    GetUpFront = 122,
    GetUpBack = 123,
    RightKick = 121,
    LeftKick = 120,
    Ceremony = 27,
    ForGrass = 20,
  };

  OPDemo()
  {
  }
  virtual ~OPDemo()
  {
  }

  virtual void setDemoEnable()
  {
  }
  virtual void setDemoDisable()
  {
  }

 protected:
  bool enable_;
};

} /* namespace robotis_op */

#endif /* OP_DEMO_H_ */
