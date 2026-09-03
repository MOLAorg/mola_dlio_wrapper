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
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
#include <mola_input_rosbag1/Rosbag1Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#endif

namespace
{

struct Cli
{
  CLI::App cmd{"mola-dlio-cli"};

  std::string argYAML;
  std::string arg_verbosity_level{"INFO"};
  bool arg_verbosity_level_set{false};
  std::string arg_plugins;
  bool arg_plugins_set{false};
  std::string arg_outPath{"output-trajectory.txt"};
  bool arg_outPath_set{false};
  int arg_firstN{0};
  bool arg_firstN_set{false};

#if defined(HAVE_MOLA_INPUT_MULRAN)
  std::string argMulranSeq{"KAIST01"};
  bool argMulranSeq_set{false};
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  std::string argRawlog{"dataset.rawlog"};
  bool argRawlog_set{false};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  std::string argRosbag2{"dataset.mcap"};
  bool argRosbag2_set{false};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  std::string argRosbag1{"dataset.bag"};
  bool argRosbag1_set{false};
#endif

// Shared by both bag formats -- rosbag1 and rosbag2 read the same topic
// names into the same lidar/imu sensor entries, see dataset_from_rosbag1()
// and dataset_from_rosbag2() below.
#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
  std::string arg_lidarTopic{"/lidar"};
  std::string arg_imuTopic{"/imu"};
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
  std::string argKittiSeq{"00"};
  bool argKittiSeq_set{false};
#endif

  CLI::Option * optVerbosity{nullptr};
  CLI::Option * optPlugins{nullptr};
  CLI::Option * optOutPath{nullptr};
  CLI::Option * optFirstN{nullptr};
#if defined(HAVE_MOLA_INPUT_MULRAN)
  CLI::Option * optMulranSeq{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_RAWLOG)
  CLI::Option * optRawlog{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  CLI::Option * optRosbag2{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  CLI::Option * optRosbag1{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
  CLI::Option * optKittiSeq{nullptr};
#endif

  Cli()
  {
    cmd.add_option("-c,--config", argYAML, "Input DLIO pipeline YAML config file (required)")
      ->required();
    optVerbosity = cmd.add_option(
      "-v,--verbosity", arg_verbosity_level,
      "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}");
    optPlugins = cmd.add_option(
      "-l,--load-plugins", arg_plugins,
      "One or more {comma separated} *.so files to load as plugins");
    optOutPath = cmd.add_option(
      "--output-tum-path", arg_outPath,
      "Save the estimated path as a TXT file using the TUM file format (see evo docs)");
    optFirstN = cmd.add_option(
      "--only-first-n", arg_firstN, "Run for the first N steps only (0=default, not used)");

#if defined(HAVE_MOLA_INPUT_MULRAN)
    optMulranSeq = cmd.add_option(
      "--input-mulran-seq", argMulranSeq,
      "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...");
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
    optRawlog = cmd.add_option(
      "--input-rawlog", argRawlog,
      "INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    optRosbag2 = cmd.add_option(
      "--input-rosbag2", argRosbag2,
      "INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    optRosbag1 = cmd.add_option(
      "--input-rosbag1", argRosbag1,
      "INPUT DATASET: rosbag1. Input dataset in ROS 1 bag format (*.bag)");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
    cmd.add_option(
      "--lidar-topic", arg_lidarTopic,
      "Only for rosbag1/rosbag2 input: the LiDAR point cloud topic name.");
    cmd.add_option(
      "--imu-topic", arg_imuTopic, "Only for rosbag1/rosbag2 input: the IMU topic name.");
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
    optKittiSeq = cmd.add_option(
      "--input-kitti-seq", argKittiSeq,
      "INPUT DATASET: Use KITTI dataset sequence number 00|01|...");
#endif
  }

  // Called from main() right after CLI11_PARSE, to fill in the *_set flags
  // (mirrors what TCLAP's own isSet() gave us for free).
  void afterParse()
  {
    arg_verbosity_level_set = (optVerbosity->count() > 0);
    arg_plugins_set = (optPlugins->count() > 0);
    arg_outPath_set = (optOutPath->count() > 0);
    arg_firstN_set = (optFirstN->count() > 0);
#if defined(HAVE_MOLA_INPUT_MULRAN)
    argMulranSeq_set = (optMulranSeq->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_RAWLOG)
    argRawlog_set = (optRawlog->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    argRosbag2_set = (optRosbag2->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    argRosbag1_set = (optRosbag1->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
    argKittiSeq_set = (optKittiSeq->count() > 0);
#endif
  }
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
    bagsYaml.c_str(), cli.arg_lidarTopic.c_str(), cli.arg_imuTopic.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag1(
  Cli & cli, const std::string & rosbag1file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag1Dataset>();
  o->setMinLoggingLevel(logLevel);

  // A comma-separated value becomes a YAML sequence, so a recording split
  // across several bag files (e.g. CitrusFarm) is replayed as the single
  // sequence it is. Rosbag1Dataset accepts a scalar or a sequence.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag1file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & p : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(p) + "'";
      }
    }
  }

  // Same env var names as dataset_from_rosbag2() above and as
  // mola-lidar-odometry-cli's own dataset_from_rosbag1(), so the same
  // override snippet (and the same dataset profiles) work unchanged for
  // this wrapper too. Unlike that CLI's version, this carries only the
  // lidar and imu sensor entries: DLIO has no GNSS/wheel-odometry fusion
  // path (DlioOdometry::initialize_frontend() only ever reads
  // lidar_sensor_label/imu_sensor_label).
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
    bagsYaml.c_str(), cli.arg_lidarTopic.c_str(), cli.arg_imuTopic.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
// KittiOdometryDataset carries no IMU stream at all -- the standard KITTI
// odometry-benchmark sequences never shipped one, unlike KITTI raw/tracking.
// DLIO is IMU-mandatory (its IMU stream is the deskew clock and the
// geometric observer's propagation clock, see DlioOdometry::onImuObservation()),
// so this wraps the real dataset and interleaves one synthetic, constant-
// gravity, zero-angular-rate CObservationIMU immediately before each real
// LiDAR scan. That keeps DLIO's propagation clock ticking and its gravity/
// bias calibration trivially converges to "level, stationary" -- the actual
// motion estimate then comes entirely from LiDAR-only GICP-to-submap
// registration, which is sound here because KITTI's scans are already
// provider-deskewed (no per-point motion compensation from a real IMU is
// needed either).
//
// Doubling datasetSize() (one synthetic-IMU timestep before each real scan
// timestep) rather than merging both into one CSensoryFrame per timestep is
// deliberate: main_odometry()'s loop below only ever takes ONE observation
// per timestep (pointcloud, else IMU), so a merged frame would silently
// drop whichever one it didn't pick.
class KittiWithSyntheticImu : public mola::OfflineDatasetSource
{
public:
  explicit KittiWithSyntheticImu(std::shared_ptr<mola::KittiOdometryDataset> inner)
  : inner_(std::move(inner))
  {
  }

  size_t datasetSize() const override { return 2 * inner_->datasetSize(); }

  mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(size_t timestep) const override
  {
    const size_t realIdx = timestep / 2;
    if (timestep % 2 == 1) {
      return inner_->datasetGetObservations(realIdx);
    }

    // Synthetic IMU tick, timestamped just before the scan it precedes.
    const auto realSf = inner_->datasetGetObservations(realIdx);
    const auto lidarObs = realSf->getObservationByClass<mrpt::obs::CObservationPointCloud>();
    ASSERT_(lidarObs);

    // mrpt::Clock::fromDouble()/TTimeStamp's raw tick count has no epoch
    // offset baked in -- fromDouble(0.0) IS the representable minimum, and
    // KITTI's own sequence-relative timestamps (KittiOdometryDataset reads
    // them straight from times.txt) start at exactly 0.0. Subtracting a
    // fixed offset without this clamp underflows the underlying unsigned
    // tick count into a garbage multi-millennium timestamp for that first
    // scan -- which then poisons DlioCore::first_imu_stamp_ (set from
    // whatever this function returns first) and permanently fails its
    // `t_since_first_imu < imu_calib_time_sec` check, so calibration simply
    // never completes and no output is ever produced (confirmed: this was
    // silent, no error, no log -- the only symptom is an empty output
    // trajectory no matter how many scans are fed). The clamp costs at most
    // the very first scan (tied to its own synthetic IMU tick, dropped by
    // the `scan_stamp_ <= imu_buffer_.back().stamp` check below) --
    // harmless, DLIO's real-IMU startup already drops several seconds of
    // scans during calibration regardless.
    const double lidarT = mrpt::Clock::toDouble(lidarObs->timestamp);
    auto imu = mrpt::obs::CObservationIMU::Create();
    imu->sensorLabel = "imu";
    imu->timestamp = mrpt::Clock::fromDouble(std::max(0.0, lidarT - 0.001));
    imu->set(mrpt::obs::IMU_X_ACC, 0.0);
    imu->set(mrpt::obs::IMU_Y_ACC, 0.0);
    imu->set(mrpt::obs::IMU_Z_ACC, kGravity);
    imu->set(mrpt::obs::IMU_WX, 0.0);
    imu->set(mrpt::obs::IMU_WY, 0.0);
    imu->set(mrpt::obs::IMU_WZ, 0.0);

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(imu);
    return sf;
  }

  bool hasGroundTruthTrajectory() const override { return inner_->hasGroundTruthTrajectory(); }
  mola::trajectory_t getGroundTruthTrajectory() const override
  {
    return inner_->getGroundTruthTrajectory();
  }

private:
  // Standard gravity magnitude (m/s^2); sign matches a stationary, level
  // accelerometer reading +g on its up axis, the same convention as
  // dlio-oxford-spires.yaml's `gravity: 9.80665` -- verify against
  // dlio_core's actual sign convention the first time this runs for real.
  static constexpr double kGravity = 9.80665;

  std::shared_ptr<mola::KittiOdometryDataset> inner_;
};

std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti(
  const std::string & kittiSeqNumber, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::KittiOdometryDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      clouds_as_organized_points: false
      publish_lidar: true
      publish_image_0: false
      publish_image_1: false
      publish_ground_truth: true
)"""",
    kittiSeqNumber.c_str())));

  o->initialize(cfg);

  return std::make_shared<KittiWithSyntheticImu>(o);
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
  if (cli.arg_verbosity_level_set) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level);
    dlio->setVerbosityLevel(logLevel);
  }

  // Initialize DLIO (no 'raw_data_source': we feed it directly below):
  const auto cfg = mola::load_yaml_file(cli.argYAML);
  dlio->initialize(cfg);

  // Select dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  if (cli.argRawlog_set) {
    dataset = dataset_from_rawlog(cli.argRawlog, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_MULRAN)
    if (cli.argMulranSeq_set) {
    dataset = dataset_from_mulran(cli.argMulranSeq, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    if (cli.argRosbag2_set) {
    dataset = dataset_from_rosbag2(cli, cli.argRosbag2, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    if (cli.argRosbag1_set) {
    dataset = dataset_from_rosbag1(cli, cli.argRosbag1, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
    if (cli.argKittiSeq_set) {
    dataset = dataset_from_kitti(cli.argKittiSeq, logLevel);
  } else
#endif
  {
    THROW_EXCEPTION("At least one of the dataset input CLI flags must be defined. Use --help.");
  }
  ASSERT_(dataset);

  // Save GT, if available:
  if (cli.arg_outPath_set && dataset->hasGroundTruthTrajectory()) {
    using namespace std::string_literals;
    const auto gtPath = dataset->getGroundTruthTrajectory();
    const auto gtOutFile = mrpt::system::fileNameChangeExtension(cli.arg_outPath, "") + "_gt."s +
                           mrpt::system::extractFileExtension(cli.arg_outPath);
    std::cout << "Ground truth available. Saving it to: " << gtOutFile << "\n";
    gtPath.saveToTextFile_TUM(gtOutFile);
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  if (cli.arg_firstN_set) {
    lastDatasetEntry = static_cast<size_t>(cli.arg_firstN);
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

  if (cli.arg_outPath_set) {
    const auto fil = cli.arg_outPath;
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
    CLI11_PARSE(cli.cmd, argc, argv);
    cli.afterParse();

    if (cli.arg_plugins_set) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins;
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
