#include "QuadActionCmdPubSubTypes.h"
#include "default_participant.h"
#include "default_subscriber.h"
#include "quadcopter_msgs/msgs/QuadActionCmd.h"

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char** argv) {
  //mavsdk setup
  if (argc != 2) {
      usage(argv[0]);
      return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
      std::cerr << "Connection failed: " << connection_result << '\n';
      return 1;
  }

  auto system = get_system(mavsdk);
  if (!system) {
      return 1;
  }

  // Instantiate plugins.
  auto action = Action{system};
  auto telemetry = Telemetry{system};
  std::cout << "System is ready\n";


  // QuadActionCmd subscriber setup
  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "selva");

  // Message
  cpp_msg::QuadActionCmd action_msg{};

  // Create subscriber with msg type
  DDSSubscriber action_cmd_sub(idl_msg::QuadActionCmdPubSubType(), &action_msg, "action_cmd",
                          dp.participant());

  action_cmd_sub.init();

  for (;;) {

    // Blocks until new data is available
    action_cmd_sub.listener->wait_for_data();

    // DEBUG
    std::cout << "Received ActionCmd: " << unsigned(action_msg.cmd) << std::endl;

    // process data
    // proposal for action_cmd:
    // 0: disarm
    // 1: arm
    // 2: reboot
    // 3: takeoff
    // 4: land
    switch(action_msg.cmd) {
      case 0: //disarm
      {
        std::cout << "Disarming Vehicle... \n";
        const Action::Result disarm_result = action.disarm();
        if(disarm_result != Action::Result::Success)
          std::cerr << "Disarming failed: " << disarm_result << std::endl;
        break;
      }

      case 1: // arm
      {
        std::cout << "Arming Vehicle... \n";
        if(!telemetry.health_all_ok()) {
          std::cout << "Vehicle not ready to arm! \n";
          break;
        }

        const Action::Result arm_result = action.arm();
        if(arm_result != Action::Result::Success)
          std::cerr << "Arming failed: " << arm_result << std::endl;
        break;
      }

      case 2: // reboot
        // TODO
        break;

      case 3: // takeoff
      {
        std::cout << "Taking off... \n";
        const Action::Result takeoff_result = action.takeoff();
        if(takeoff_result != Action::Result::Success)
          std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        break;
      }

      case 4: // land
      {
        std::cout << "Landing... \n";
        const Action::Result land_result = action.land();
        if(land_result != Action::Result::Success) {
          std::cerr << "Land failed: " << land_result << std::endl;
          break;
        }
        // wait until vehicle is not in the air anymore
        while (telemetry.in_air()) {
          std::cout << "Vehicle is landing...\n";
          sleep_for(seconds(1));
        }
        std::cout << "Landed!\n";
        break;
      }

      default:
        std::cout << "Unknown QuadActionCmd \n" ;
    }

  }

  return 0;
}