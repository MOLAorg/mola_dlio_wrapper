/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   mola-dlio-cli.cpp
 * @brief  main() for the offline, loss-free DLIO CLI.
 *
 * Structurally identical to mola_fast_lio2_wrapper's own
 * `mola-fast-lio2-cli.cpp` (same dataset-source helpers, same TCLAP flags,
 * same `while (isBusy()) sleep(1ms)` no-drop idiom).
 */

#include <mola_dlio_wrapper/DlioOdometry.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

namespace
{

struct Cli
{
  TCLAP::CmdLine cmd{"mola-dlio-cli"};

  TCLAP::ValueArg<std::string> argYAML{
    "c",  "config", "Input DLIO pipeline YAML config file (required)",
    true, "",       "dlio-oxford-spires.yaml",
    cmd};

  TCLAP::ValueArg<std::string> arg_verbosity_level{
    "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}", false, "",
    "INFO", cmd};

  TCLAP::ValueArg<std::string> arg_plugins{
    "l",   "load-plugins", "One or more {comma separated} *.so files to load as plugins",
    false, "foobar.so",    "foobar.so",
    cmd};

  TCLAP::ValueArg<std::string> arg_outPath{
    "",
    "output-tum-path",
    "Save the estimated path as a TXT file using the TUM file format (see evo docs)",
    false,
    "output-trajectory.txt",
    "output-trajectory.txt",
    cmd};

  TCLAP::ValueArg<int> arg_firstN{
    "",
    "only-first-n",
    "Run for the first N steps only (0=default, not used)",
    false,
    0,
    "Number of dataset entries to run",
    cmd};

#if defined(HAVE_MOLA_INPUT_MULRAN)
  TCLAP::ValueArg<std::string> argMulranSeq{
    "",    "input-mulran-seq", "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...",
    false, "KAIST01",          "KAIST01",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  TCLAP::ValueArg<std::string> argRawlog{
    "",    "input-rawlog",   "INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)",
    false, "dataset.rawlog", "dataset.rawlog",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  TCLAP::ValueArg<std::string> argRosbag2{
    "",    "input-rosbag2", "INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)",
    false, "dataset.mcap",  "dataset.mcap",
    cmd};
  TCLAP::ValueArg<std::string> arg_lidarTopic{
    "",    "lidar-topic", "Only for rosbag2 input: the LiDAR point cloud topic name.",
    false, "/lidar",      "/lidar",
    cmd};
  TCLAP::ValueArg<std::string> arg_imuTopic{
    "", "imu-topic", "Only for rosbag2 input: the IMU topic name.", false, "/imu", "/imu", cmd};
#endif
};  // end struct "Cli"

#if defined(HAVE_MOLA_INPUT_MULRAN)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_mulran(
  const std::string & mulranSequence, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::MulranDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${MULRAN_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_ground_truth: true
)"""",
    mulranSequence.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rawlog(
  const std::string & rawlogFile, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::RawlogDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rawlog_filename: '%s'
      read_all_first: true
)"""",
    rawlogFile.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  Cli & cli, const std::string & rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag2Dataset>();
  o->setMinLoggingLevel(logLevel);

  // A comma-separated value becomes a YAML sequence, so that a sequence split
  // across several bag directories (e.g. Oxford Spires keble-college-04, whose
  // two parts share a base timestamp and are two halves of one continuous
  // recording) can be replayed as the single sequence it is. Rosbag2Dataset
  // accepts either a scalar or a sequence for 'rosbag_filename'.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag2file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & p : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(p) + "'";
      }
    }
  }

  // Fixed sensor poses (env vars), for bags with no /tf or /tf_static (e.g.
  // Oxford Spires): same env var names as mola-lidar-odometry-cli's own
  // dataset_from_rosbag2(), so the same override snippet works for all
  // wrappers in this benchmark suite.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
    bagsYaml.c_str(), cli.arg_lidarTopic.getValue().c_str(),
    cli.arg_imuTopic.getValue().c_str())));

  o->initialize(cfg);
  return o;
}
#endif

void mola_signal_handler(int s)
{
  std::cerr << "Caught signal " << s << ". Shutting down...\n";
  exit(0);  // NOLINT
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler
  {
  };
  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, nullptr);
}

int main_odometry(Cli & cli)
{
  auto dlio = mola::DlioOdometry::Create();

  mrpt::system::VerbosityLevel logLevel = dlio->getMinLoggingLevel();
  if (cli.arg_verbosity_level.isSet()) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    dlio->setVerbosityLevel(logLevel);
  }

  // Initialize DLIO (no 'raw_data_source': we feed it directly below):
  const auto cfg = mola::load_yaml_file(cli.argYAML.getValue());
  dlio->initialize(cfg);

  // Select dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  if (cli.argRawlog.isSet()) {
    dataset = dataset_from_rawlog(cli.argRawlog.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_MULRAN)
    if (cli.argMulranSeq.isSet()) {
    dataset = dataset_from_mulran(cli.argMulranSeq.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    if (cli.argRosbag2.isSet()) {
    dataset = dataset_from_rosbag2(cli, cli.argRosbag2.getValue(), logLevel);
  } else
#endif
  {
    THROW_EXCEPTION("At least one of the dataset input CLI flags must be defined. Use --help.");
  }
  ASSERT_(dataset);

  // Save GT, if available:
  if (cli.arg_outPath.isSet() && dataset->hasGroundTruthTrajectory()) {
    using namespace std::string_literals;
    const auto gtPath = dataset->getGroundTruthTrajectory();
    const auto gtOutFile = mrpt::system::fileNameChangeExtension(cli.arg_outPath.getValue(), "") +
                           "_gt."s + mrpt::system::extractFileExtension(cli.arg_outPath.getValue());
    std::cout << "Ground truth available. Saving it to: " << gtOutFile << "\n";
    gtPath.saveToTextFile_TUM(gtOutFile);
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  if (cli.arg_firstN.isSet()) {
    lastDatasetEntry = static_cast<size_t>(cli.arg_firstN.getValue());
  }
  mrpt::keep_min(lastDatasetEntry, dataset->datasetSize());

  size_t nLidarFed = 0;

  std::cout << "\n";  // Needed for the VT100 codes below.

  for (size_t i = 0; i < lastDatasetEntry; i++) {
    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    mrpt::obs::CObservation::Ptr obs =
      sf->getObservationByClass<mrpt::obs::CObservationPointCloud>();
    if (!obs) {
      obs = sf->getObservationByClass<mrpt::obs::CObservationIMU>();
    }
    if (!obs) {
      continue;
    }

    if (obs->GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservationPointCloud)) {
      nLidarFed++;
    }

    dlio->onNewObservation(obs);

    // No-drop guarantee: wait for the worker to fully drain this scan
    // before feeding the next one.
    while (dlio->isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    static int cnt = 0;
    if (cnt++ % 100 == 0) {
      cnt = 0;
      const size_t N = (dataset->datasetSize() - 1);
      const double pc = N > 0 ? static_cast<double>(i) / static_cast<double>(N) : 1.0;
      const double tNow = mrpt::Clock::nowDouble();
      const double ETA = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
      const double totalTime = ETA + (tNow - tStart);

      std::cout << "\033[A\33[2KT\r" << mrpt::system::progress(pc, 30)
                << mrpt::format(
                     " %6zu/%6zu (%.02f%%) ETA=%s/T=%s | scans fed=%zu\n", i, N, 100 * pc,
                     mrpt::system::formatTimeInterval(ETA).c_str(),
                     mrpt::system::formatTimeInterval(totalTime).c_str(), nLidarFed);
      std::cout.flush();
    }
  }

  std::cout << "\nDone. Dataset entries processed: " << lastDatasetEntry
            << ", LiDAR scans fed: " << nLidarFed << "\n";

  if (cli.arg_outPath.isSet()) {
    const auto fil = cli.arg_outPath.getValue();
    std::cout << "Saving estimated path in TUM format to: " << fil << "\n";
    dlio->estimatedTrajectory().saveToTextFile_TUM(fil);
  }

  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;
    if (!cli.cmd.parse(argc, argv)) {
      return 1;
    }

    if (cli.arg_plugins.isSet()) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins.getValue();
      std::cout << "Loading plugin(s): " << plugins << "\n";
      if (!mrpt::system::loadPluginModules(plugins, errMsg)) {
        std::cerr << errMsg << std::endl;
        return 1;
      }
    }

    mola_install_signal_handler();
    return main_odometry(cli);
  } catch (std::exception & e) {
    mola::pretty_print_exception(e, "Exit due to exception:");
    return 1;
  }
}
