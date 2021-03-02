/*
 * Copyright (c) 2021, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <gtest/gtest.h>

#include <robot_localization/navsat_conversions.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "robot_localization/srv/set_datum.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

TEST(NavSatTransformUTMJumpTest, UtmTest)
{
  auto node_ = rclcpp::Node::make_shared("test_navsat_transform");
  auto setDatumClient = node_->create_client<robot_localization::srv::SetDatum>("/datum");
  auto fromLLClient = node_->create_client<robot_localization::srv::FromLL>("/fromLL");

  EXPECT_TRUE(setDatumClient->wait_for_service(5s));

  // Initialise the navsat_transform node to a UTM zone
  auto setDatumRequest = std::make_shared<robot_localization::srv::SetDatum::Request>();
  setDatumRequest->geo_pose.position.latitude = 1;
  setDatumRequest->geo_pose.position.longitude = 4;
  setDatumRequest->geo_pose.orientation.w = 1;

  auto setDatumResponse = setDatumClient->async_send_request(setDatumRequest);
  auto ret = rclcpp::spin_until_future_complete(node_, setDatumResponse, 5s);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  // Let the node figure out its transforms
  rclcpp::Rate(0.2).sleep();

  // Request the GPS point of said point:
  auto fromLLRequest = std::make_shared<robot_localization::srv::FromLL::Request>();
  fromLLRequest->ll_point.latitude = 10;
  fromLLRequest->ll_point.longitude = 4.5;
  auto fromLLResponse = fromLLClient->async_send_request(fromLLRequest);
  ret = rclcpp::spin_until_future_complete(node_, fromLLResponse, 5s);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  auto initial_response = fromLLResponse.get();

  // Request GPS point also in that zone
  fromLLRequest->ll_point.longitude = 5.5;

  fromLLResponse = fromLLClient->async_send_request(fromLLRequest);
  ret = rclcpp::spin_until_future_complete(node_, fromLLResponse, 5s);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  auto same_zone_response = fromLLResponse.get();

  // 1° of longitude is about 111 kilometers in length
  EXPECT_NEAR(initial_response->map_point.x, same_zone_response->map_point.x, 120e3);
  EXPECT_NEAR(initial_response->map_point.y, same_zone_response->map_point.y, 120e3);

  // Request GPS point from neighboring zone (zone crossing is at 6 degrees)
  fromLLRequest->ll_point.longitude = 6.5;

  fromLLResponse = fromLLClient->async_send_request(fromLLRequest);
  ret = rclcpp::spin_until_future_complete(node_, fromLLResponse, 5s);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  auto neighbour_zone_response = fromLLResponse.get();

  EXPECT_NEAR(initial_response->map_point.x, neighbour_zone_response->map_point.x, 2 * 120e3);
  EXPECT_NEAR(initial_response->map_point.y, neighbour_zone_response->map_point.y, 2 * 120e3);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
