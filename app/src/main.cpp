#include "QuadActionCmdPubSubTypes.h"
#include "default_participant.h"
#include "default_publisher.h"
#include "quadcopter_msgs/msgs/QuadActionCmd.h"
#include <chrono>
#include <cstdlib>
#include <future>

int main() {
  // Message
  idl_msg::QuadActionCmd action_cmd;

  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "selva");

  // Create publisher with msg type
  DDSPublisher action_cmd_pub(idl_msg::QuadActionCmdPubSubType(), "action_cmd",
                            dp.participant());

  // Initialize publisher with topic name
  if (action_cmd_pub.init()) {
    for (u_int8_t i = 0; i < 10; i++) {
      action_cmd.cmd(i + 1);
      action_cmd_pub.publish(action_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }


}