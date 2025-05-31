/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "app_info.h"
#include "app_version.h"
#include "ctrl_manipulatorcontrol.h"
#include "instrument/instrument_module.h"
#include "lgc_motion.h"
#include "mcx/control3/ctrl_axescontrol.h"
#include <mcx/core.h>
#include <mcx/mechanics.h>
#include <mcx/udp.h>

// this flag is for debugging purposes, when it is necessary to find math exception
// #define ADD_FE
#ifdef ADD_FE
#include <cfenv>
#endif

using namespace mcx;

void run(const cmd_line::Config& config) {
  // create the Parameter Server
  parameter_server::Parameter paramServer;
  // create the root of the parameter tree
  paramServer.create("root", nullptr);
  // create the req/rep server
  comm::RequestReply reqrep;

  // Defining Tasks
  container::Task loggerTask("Logger_task", &paramServer);
  container::Task fieldbusTask("Fieldbus_task", &paramServer);
  container::Task controlTask("Control_task", &paramServer);
  container::Task commTask("Comm_task", &paramServer);

  // create the logger module
  log::Module logger(config.path("Log"));
  logger.create("logger", &paramServer);
  // create and configure log output task
  loggerTask.add(&logger);
  loggerTask.configure();

  // print system configuration
  log_info("Configuration:\n{}", config.toString());

  // App Info Module
  feature::App appFeatures{.signalGenerators = feature::getAmount(config, "SignalGenerator"),
                           .setpointGenerators = feature::getAmount(config, "SetpointGenerator"),
                           .joysticks = feature::getAmount(config, "Joystick"),
                           .udp = feature::getAmount(config, "UdpComm"),
                           .axes = feature::getAxes(config, "AxesControl"),
                           .instrumentAxes = feature::getAxes(config, "InstrumentAxesControl"),
                           .manipulator = feature::getManipulator(config, "ManipulatorControl"),
                           .safetyChannelSelector = feature::getSafetyChannelSelector(config, "SafetyChannelSelector")};

  AppInfo appInfo(appFeatures, config.mode(), config.license().isValid());
  appInfo.create("AppInfo", &paramServer);

  // Loading mechanism
  auto mechanismModuleFactory = mechanics::MechanismModuleFactorySingleton::get();
  auto mechPath = config.path("Mechanism");
  auto mechanismModule = mechanismModuleFactory.load(config.path("Mechanism"));

  // Creating Manipulator
  control::ManipulatorControl manipulatorControl(std::move(mechanismModule), appFeatures.manipulator);
  manipulatorControl.create("ManipulatorControl", &paramServer);

  // Creating Machine Control
  //  control::MachineControl machineControl(appFeatures.machine);
  //  machineControl.create("MachineControl", &paramServer);

  // Creating SafetyChannelSelector
  control3::ChannelSelector safetyChannelSelector(appFeatures.safetyChannelSelector.numberOfChannels);

  // Create a Watchdog
  watchdog::Module watchdog(config.mode(), watchdog::WatchdogType::EXTERNAL, "/dev/null");
  watchdog.create("Watchdog", &paramServer);

  // Creating Persistence
  parameter_server::Persistence persistence(config.path("Persistence"));
  persistence.create("Persistence", &paramServer);

  // Creating AxesControl
  control3::AxesControl axesControl(appFeatures.axes.numberOfAxes, appFeatures.axes.numberOfActuators);
  axesControl.create("AxesControl", &paramServer);

  // std::unique_ptr<control3::SimpleAxesControl> instrumentAxesControl;
  std::unique_ptr<control3::AxesControl> instrumentAxesControl;
  if (appFeatures.instrumentAxes.enabled) {
    // instrumentAxesControl = std::make_unique<control3::SimpleAxesControl>(appFeatures.instrumentAxes.numberOfAxes,
    //                                                                 appFeatures.instrumentAxes.numberOfActuators);
    instrumentAxesControl = std::make_unique<control3::AxesControl>(appFeatures.instrumentAxes.numberOfAxes,
                                                                    appFeatures.instrumentAxes.numberOfActuators);
    instrumentAxesControl->create("InstrumentAxesControl", &paramServer);
  }

  // Create Field Bus Manager
  fbus::Manager fbusManager;
  // load field bus master configuration from the file and creating a communication module
  auto fbusModule = fbusManager.create(config.mode(), &paramServer, config.path("Fieldbus"));

  const auto totalNumberOfActuators = appFeatures.axes.numberOfActuators +
                                      appFeatures.instrumentAxes.numberOfActuators * appFeatures.instrumentAxes.enabled;

  // Create Hardware Simulator Module
  drive::sim::DriveBasic<int64_t> simulator(totalNumberOfActuators);
  simulator.create("Simulator", &paramServer);

  // Create CIA402 drives
  drive::Module ciadsp402Logic(config.mode(), drive::strToDriveType(config.get<std::string>("/Drive")),
                               totalNumberOfActuators);
  ciadsp402Logic.create("DriveLogic", &paramServer);

  // Create Motion Logic
  logic::MotionLogic motionLogic(config.path("Control"), appFeatures.instrumentAxes.enabled, totalNumberOfActuators,
                                 config.mode());
  motionLogic.create("Logic", &paramServer);

  // Create User Parameters
  user_parameters::Module userParameters(config.path("UserParameters"));
  userParameters.create("UserParameters", &paramServer);

  std::unique_ptr<udp::Module> udpModule;
  if (appFeatures.udp > 0) {
    // create udp communication module
    udpModule = std::make_unique<udp::Module>("UdpComm", config);
    udpModule->create("UdpComm", &paramServer);
  }

  // Feature: SetpointGenerators Modules
  auto [setpoint_generators, setpoint_generators_amount] = feature::create<control3::SetpointGenerator>(
      paramServer, controlTask, config, "SetpointGenerator", "SetpointGenerators");

  // Feature: SignalGenerator Modules
  auto [signal_generators, signal_generators_amount] = feature::create<control3::SignalGenerator>(
      paramServer, controlTask, config, "SignalGenerator", "SignalGenerators");

  // Create Publisher Module
  auto connectionData = config.server("Default");
  comm::Publisher publisher(reqrep, connectionData);
  publisher.create("ParamPub", &paramServer);

  /* Adding modules to the tasks */

  // create and configure field bus task
  fieldbusTask.add(fbusModule);
  fieldbusTask.add({&axesControl, instrumentAxesControl.get()});
  fieldbusTask.configure();

  // adding instruments
  std::unique_ptr<InstrumentModule> instrumentModule;
  if (instrumentsEnabled(config)) {
    auto [instruments, is_ok] = loadInstruments(config);
    if (!is_ok) {
      log_error("Failed to load instruments");
    }
    instrumentModule = std::make_unique<InstrumentModule>(instruments);
    instrumentModule->create("Instruments", &paramServer);
  }

  // Create Logic Task
  container::Task logicTask("Logic_task", &paramServer);
  logicTask.add({&appInfo, &motionLogic, &ciadsp402Logic, &userParameters, &persistence, instrumentModule.get()});
  logicTask.configure();

  // Create Simulator Task
  container::Task simulatorTask("Simulator_task", &paramServer);
  simulatorTask.add({&simulator, &watchdog, udpModule.get()});

  simulatorTask.configure();

  // Create Control Task
  controlTask.add(&manipulatorControl);
  controlTask.configure();

  // Create Communication Task
  commTask.add(&publisher);
  commTask.configure();

  // When all modules are configured req/rep caches the parameter tree
  reqrep.configure(&paramServer, cmd_line::path(config.path("Control")));

  // Linking
  parameter_server::link(config.paths("Linking"), &paramServer, config.systemMode());
  if (instrumentAxesControl) {
    parameter_server::link(config.paths("/InstrumentAxesControl/Linking"), &paramServer, config.systemMode());
  }

  if (setpoint_generators_amount > 0) {
    log_info("Applying SetpointGenerator Links");
    parameter_server::load(config.path("/SetpointGenerator/Control"), &paramServer);
    parameter_server::link(config.path("/SetpointGenerator/Linking"), &paramServer, config.systemMode());
  }
  if (signal_generators_amount > 0) {
    log_info("Applying SignalGenerator Links");
    parameter_server::link(config.path("/SignalGenerator/Linking"), &paramServer, config.systemMode());
  }

  // Loading parameter values from control.xml
  parameter_server::load(config.path("Control"), &paramServer);

  // Start all the tasks
  loggerTask.start(config.task("Logger_task"));
  commTask.start(config.task("Comm_task"));
  logicTask.start(config.task("Logic_task"));
  simulatorTask.start(config.task("Simulator_task"));
  controlTask.start(config.task("Control_task"));
  fieldbusTask.start(config.task("Fieldbus_task"));

  // Start the req/rep server
  bool isConnected = reqrep.start(connectionData);
  log_assert(isConnected, "Failed to start Req/Rep server");

  // req/rep server is used to create the main loop because it is blocking
  while (utils::running()) {
    reqrep.iterate(1000);
  }

  // stop all the tasks
  reqrep.stop();
  fieldbusTask.stop();
  simulatorTask.stop();
  controlTask.stop();
  commTask.stop();
  logicTask.stop();
  loggerTask.stop();

  // clear all allocated resources
  paramServer.destroy();
}

int main(int argc, char** argv) {

#ifdef ADD_FE
  /* FE_DIVBYZERO    pole error occurred in an earlier floating-point operation
   * FE_INEXACT      inexact result: rounding was necessary to store the result of an earlier floating-point operation
   * FE_INVALID      domain error occurred in an earlier floating-point operation
   * FE_OVERFLOW     the result of the earlier floating-point operation was too large to be representable
   * FE_UNDERFLOW    the result of the earlier floating-point operation was subnormal with a loss of precision
   * FE_ALL_EXCEPT   bitwise OR of all supported floating-point exceptions */

  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

  auto config = cmd_line::parse(argc, argv,
                                {{"Meril-robot-template", meril_robot::version()},
                                 {"Motorcortex-core", utils::version()},
                                 {"Motorcortex-math", math::version()},
                                 {"Motorcortex-control", control3::version()},
                                 {"Motorcortex-mechanics", mechanics::version()},
                                 {"Motorcortex-udp", udp::version()}});

  // start low latency, isolate CPU 0 and 1
  utils::startRealTime(config.realtime());
  // setup and run awesome controls
  run(config);
  // stop low latency, removes CPU isolation
  utils::stopRealTime(config.realtime());

  return 0;
}
