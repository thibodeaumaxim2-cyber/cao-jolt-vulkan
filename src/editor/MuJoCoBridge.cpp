#include "MuJoCoBridge.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr std::array<const char *, kRobotLegCount> kLegNames{{
    "Front Left", "Middle Left", "Rear Left",
    "Front Right", "Middle Right", "Rear Right"}};

constexpr std::array<float, 4> kMinAngles{{-0.20f, -0.40f, -0.70f, -0.30f}};
constexpr std::array<float, 4> kMaxAngles{{ 0.20f,  0.40f,  0.20f,  0.30f}};
// A shallow crouch gives the knees useful travel in both directions. The
// resulting stance keeps all six feet on the ground while avoiding a fully
// straight, mechanically locked leg.
constexpr std::array<float, 4> kStandPose{{0.0f, 0.08f, -0.12f, 0.06f}};

std::string makeModelXml(bool biped) {
  const size_t legCount = biped ? 2u : kRobotLegCount;
  std::ostringstream xml;
  xml << R"(<mujoco model=")" << (biped ? "cao_biped" : "cao_hexapod") << R"(">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="0.0041666667" gravity="0 0 -9.81" iterations="100" solver="Newton" integrator="implicitfast"/>
  <default>
    <joint limited="true" damping="8" armature="0.02"/>
    <geom contype="1" conaffinity="0" condim="4" friction="1.0 0.01 0.001"/>
  </default>
  <worldbody>
    <geom name="Ground" type="plane" size="20 20 0.1" conaffinity="1" friction="4.5 0.02 0.001"/>
    <body name="root" pos="0 0 0.925">
      <freejoint name="root_free"/>
      <geom name="Torso" type="box" size=")" << (biped ? "0.42 0.40 0.14" : "0.775 0.725 0.14")
      << R"(" mass=")" << (biped ? "0.50" : "0.90") << R"("/>
)";
  for (size_t leg = 0; leg < legCount; ++leg) {
    const float side = biped ? (leg == 0 ? -1.0f : 1.0f) : (leg < 3 ? -1.0f : 1.0f);
    const int station = biped ? 0 : static_cast<int>(leg % 3) - 1;
    const float x = side * (biped ? 0.44f : 0.64f);
    const float y = static_cast<float>(station) * 0.62f;
    const std::string prefix = biped ? std::string("Front ") + (leg == 0 ? "Left" : "Right") : kLegNames[leg];
    const size_t base = leg * kRobotJointsPerLeg;
    xml << "      <body name=\"body_" << base << "\" pos=\"" << x << ' ' << y << " 0.055\">\n"
        << "        <joint name=\"joint_" << base << "\" type=\"hinge\" axis=\"0 1 0\" range=\"-0.20 0.20\"/>\n"
        << "        <geom name=\"" << prefix << " Hip Roll\" type=\"box\" size=\"0.11 0.16 0.07\" mass=\"0.03\"/>\n"
        << "        <body name=\"body_" << base + 1 << "\" pos=\"0 0 -0.055\">\n"
        << "          <joint name=\"joint_" << base + 1 << "\" type=\"hinge\" axis=\"1 0 0\" range=\"-0.40 0.40\"/>\n"
        << "          <geom name=\"" << prefix << " Hip\" type=\"box\" pos=\"0 0 -0.1625\" size=\"0.14 0.14 0.1375\" mass=\"0.12\"/>\n"
        << "          <body name=\"body_" << base + 2 << "\" pos=\"0 0 -0.325\">\n"
        << "            <joint name=\"joint_" << base + 2 << "\" type=\"hinge\" axis=\"1 0 0\" range=\"-0.70 0.20\"/>\n"
        << "            <geom name=\"" << prefix << " Shin\" type=\"box\" pos=\"0 0 -0.255\" size=\"0.12 0.12 0.255\" mass=\"0.08\"/>\n"
        << "            <body name=\"body_" << base + 3 << "\" pos=\"0 0 -0.51\">\n"
        << "              <joint name=\"joint_" << base + 3 << "\" type=\"hinge\" axis=\"1 0 0\" range=\"-0.30 0.30\"/>\n"
        << "              <geom name=\"" << prefix << " Foot\" type=\"box\" pos=\"0 0 -0.045\" size=\""
        << (biped ? "0.21 0.46 0.045" : "0.17 0.21 0.045") << "\" mass=\"0.15\"/>\n"
        << "            </body>\n          </body>\n        </body>\n      </body>\n";
  }
  xml << "    </body>\n  </worldbody>\n  <actuator>\n";
  for (size_t leg = 0; leg < legCount; ++leg) {
    for (size_t joint = 0; joint < kRobotJointsPerLeg; ++joint) {
      const size_t index = leg * kRobotJointsPerLeg + joint;
      const float kp = joint == 0 ? 30.0f : joint == 1 ? 80.0f : joint == 2 ? 100.0f : 30.0f;
      xml << "    <position name=\"actuator_" << index << "\" joint=\"joint_" << index
          << "\" kp=\"" << kp << "\" ctrlrange=\"" << kMinAngles[joint] << ' '
          << kMaxAngles[joint] << "\" forcelimited=\"true\" forcerange=\"-" << kp << ' ' << kp << "\"/>\n";
    }
  }
  xml << "  </actuator>\n</mujoco>\n";
  return xml.str();
}

float clampUnit(float value) { return std::clamp(value, -1.0f, 1.0f); }

} // namespace

struct MuJoCoBridge::Impl {
  mjModel *model = nullptr;
  mjData *data = nullptr;
  bool ready = false;
  int script = 0;
  float scriptTime = 0.0f;
  StandingTuning tuning{};
  RobotTelemetry telemetry{};
  size_t legCount = kRobotLegCount;
  bool biped = false;
  bool unitreeH1 = false;
};

MuJoCoBridge::MuJoCoBridge() : impl_(std::make_unique<Impl>()) {}
MuJoCoBridge::~MuJoCoBridge() { shutdown(); }

void MuJoCoBridge::initialize() {
  if (impl_->ready) return;
  impl_->ready = true;
}

void MuJoCoBridge::rebuild(Scene &scene) {
  if (!impl_->ready) return;
  if (impl_->data) { mj_deleteData(impl_->data); impl_->data = nullptr; }
  if (impl_->model) { mj_deleteModel(impl_->model); impl_->model = nullptr; }

  impl_->biped = scene.isBiped();
  impl_->unitreeH1 = scene.isUnitreeH1();
  impl_->legCount = impl_->unitreeH1 ? 0u : impl_->biped ? 2u : kRobotLegCount;
  char error[1024]{};
  if (impl_->unitreeH1) {
    const auto h1Scene = std::filesystem::path(CAO_SOURCE_DIR) / "third_party" / "unitree_mujoco" /
        "unitree_robots" / "h1" / "scene.xml";
    if (!std::filesystem::exists(h1Scene))
      throw std::runtime_error("Unitree H1 assets are missing; run: git submodule update --init --recursive");
    impl_->model = mj_loadXML(h1Scene.string().c_str(), nullptr, error, sizeof(error));
    if (!impl_->model)
      throw std::runtime_error(std::string("Unitree H1 MJCF error: ") + error);
  } else {
    const std::string xml = makeModelXml(impl_->biped);
  mjVFS vfs{};
  mj_defaultVFS(&vfs);
  const int fileResult = mj_addBufferVFS(
      &vfs, "cao_hexapod.xml", xml.data(), static_cast<int>(xml.size()));
  if (fileResult != 0) {
    mj_deleteVFS(&vfs);
    throw std::runtime_error("MuJoCo could not allocate the in-memory MJCF model");
  }
  impl_->model = mj_loadXML("cao_hexapod.xml", &vfs, error, sizeof(error));
  mj_deleteVFS(&vfs);
  if (!impl_->model)
    throw std::runtime_error(std::string("MuJoCo MJCF error: ") + error);
  }
  impl_->data = mj_makeData(impl_->model);
  if (!impl_->data) throw std::runtime_error("MuJoCo could not allocate simulation data");
  mj_forward(impl_->model, impl_->data);
  if (impl_->unitreeH1) {
    // Initialize proxy transforms immediately so a paused import appears as a
    // humanoid, not a stack of boxes waiting for the first simulation frame.
    constexpr std::array<int, 3> axisMap{{0, 2, 1}};
    for (SceneObject &object : scene.objects()) {
      const int body = mj_name2id(impl_->model, mjOBJ_BODY, object.name.c_str());
      if (body < 0) continue;
      const mjtNum *position = impl_->data->xpos + 3 * body;
      const mjtNum *rotation = impl_->data->xmat + 9 * body;
      const auto mapped = [&](int row, int column) {
        return static_cast<float>(rotation[3 * axisMap[row] + axisMap[column]]);
      };
      object.transform.position = {static_cast<float>(position[0]), static_cast<float>(position[2]), static_cast<float>(position[1])};
      object.transform.rotation = {std::atan2(mapped(2, 1), mapped(2, 2)),
          std::asin(clampUnit(-mapped(2, 0))), std::atan2(mapped(1, 0), mapped(0, 0))};
    }
  }
  impl_->scriptTime = 0.0f;
  impl_->telemetry = {};
  impl_->telemetry.linkCount = static_cast<int>(scene.objects().size());
}

void MuJoCoBridge::step(Scene &scene, float seconds) {
  if (!impl_->ready || !impl_->model || !impl_->data || seconds <= 0.0f) return;
  impl_->scriptTime += seconds;
  impl_->model->opt.timestep = seconds;
  auto &telemetry = impl_->telemetry;
  telemetry = {};
  telemetry.motionScript = impl_->script;
  telemetry.linkCount = static_cast<int>(scene.objects().size());

  // MuJoCo is Z-up while CAO's renderer is Y-up: (x, y, z)_CAO maps to
  // (x, z, y)_MuJoCo.
  const float rootY = static_cast<float>(impl_->data->qpos[2]);
  const float speed = std::hypot(static_cast<float>(impl_->data->qvel[0]),
                                 static_cast<float>(impl_->data->qvel[1]));
  const float heightScore = std::clamp((rootY - 0.70f) / 0.20f, 0.0f, 1.0f);
  const float speedScore = std::clamp(1.0f - speed / 0.70f, 0.0f, 1.0f);
  telemetry.equilibriumScore = 0.65f * heightScore + 0.35f * speedScore;
  telemetry.walkingAllowed = telemetry.equilibriumScore >= 0.88f;
  telemetry.torsoSpeedMps = speed;

  std::array<std::array<float, kRobotJointsPerLeg>, kRobotLegCount> desired;
  desired.fill(kStandPose);
  if (!impl_->biped && !impl_->unitreeH1 && impl_->script >= 1 && telemetry.walkingAllowed) {
    const auto commandLeg = [&](size_t leg, float phase, float rearHip,
                                float frontHip, float kneeLift, float ankleLift,
                                int tripod) {
      if (phase < 0.56f) {
        // A planted foot sweeps backwards to move the body forwards.
        const float stance = phase / 0.56f;
        desired[leg][1] = kStandPose[1] + frontHip + (rearHip - frontHip) * stance;
      } else if (phase < 0.66f) {
        desired[leg][1] = kStandPose[1] + rearHip;
        telemetry.legState[leg] = 1;
      } else if (phase < 0.92f) {
        const float swingPhase = (phase - 0.66f) / 0.26f;
        const float lift = std::sin(3.14159265f * swingPhase);
        desired[leg][1] = kStandPose[1] + rearHip + (frontHip - rearHip) * swingPhase;
        desired[leg][2] = kStandPose[2] - kneeLift * lift;
        desired[leg][3] = kStandPose[3] + ankleLift * lift;
        telemetry.legState[leg] = 2;
        if (telemetry.activeSwingLeg < 0) telemetry.activeSwingLeg = static_cast<int>(leg);
        telemetry.activeSwingTripod = tripod;
      } else {
        desired[leg][1] = kStandPose[1] + frontHip;
        telemetry.legState[leg] = 3;
      }
    };

    if (impl_->script == 2) {
      // Alternating-tripod gait: three feet form a broad support triangle
      // while the other three reposition together. The short stride keeps it
      // a stable alternative to the conservative one-foot crawl.
      const float cycle = std::fmod(impl_->scriptTime / 6.0f, 1.0f);
      const int swingTripod = cycle < 0.5f ? 0 : 1;
      const float tripodPhase = std::fmod(cycle * 2.0f, 1.0f);
      telemetry.gaitCycle = cycle;
      // During the short swing window, gently load the planted tripod. This
      // is deliberately an extension into the ground, rather than a lift:
      // it transfers weight onto the support triangle and gives the swing
      // tripod room to travel up and forward without losing contact support.
      const float supportPhase = std::clamp((tripodPhase - 0.66f) / 0.26f, 0.0f, 1.0f);
      const float supportLoad = std::sin(3.14159265f * supportPhase);
      for (size_t leg = 0; leg < kRobotLegCount; ++leg) {
        const int tripod = (leg == 0 || leg == 2 || leg == 4) ? 0 : 1;
        if (tripod == swingTripod) {
          commandLeg(leg, tripodPhase, -0.08f, 0.08f, 0.26f, 0.11f, tripod);
        } else {
          desired[leg][2] = kStandPose[2] + 0.035f * supportLoad;
          desired[leg][3] = kStandPose[3] - 0.015f * supportLoad;
        }
      }
    } else {
      // Scripts 1 and 3 are one-foot crawls; script 1 is deliberately slow.
      const float cycleSeconds = impl_->script == 1 ? 5.4f : 3.2f;
      const float cycle = std::fmod(impl_->scriptTime / cycleSeconds, 1.0f);
      const size_t activeLeg = std::min(static_cast<size_t>(cycle * kRobotLegCount), kRobotLegCount - 1u);
      const float legCycle = cycle * kRobotLegCount - static_cast<float>(activeLeg);
      telemetry.gaitCycle = cycle;
      commandLeg(activeLeg, legCycle, -0.28f, 0.28f, 0.55f, 0.24f,
                 activeLeg == 0 || activeLeg == 2 || activeLeg == 4 ? 0 : 1);
    }
  }

  for (size_t leg = 0; leg < impl_->legCount; ++leg) {
    for (size_t joint = 0; joint < kRobotJointsPerLeg; ++joint) {
      const size_t index = leg * kRobotJointsPerLeg + joint;
      impl_->data->ctrl[index] = desired[leg][joint];
      const int jointId = mj_name2id(impl_->model, mjOBJ_JOINT, ("joint_" + std::to_string(index)).c_str());
      const int qpos = impl_->model->jnt_qposadr[jointId];
      const float actual = static_cast<float>(impl_->data->qpos[qpos]);
      const float error = desired[leg][joint] - actual;
      telemetry.targetAnglesRad[leg][joint] = desired[leg][joint];
      telemetry.measuredAnglesRad[leg][joint] = actual;
      telemetry.angleErrorRad[leg][joint] = error;
      const float limit = telemetry.torqueLimitsNm[joint];
      telemetry.estimatedTorqueDemandNm[index] = std::min(limit, std::abs(error) * limit / 0.35f);
      telemetry.torqueSaturated[index] = std::abs(error) > 0.50f;
    }
    // A lifted foot must not drag the ground, while every support foot gets
    // high tangential grip. MuJoCo combines this with the ground coefficient
    // in its contact solver, avoiding the all-feet-equal slip seen in the
    // recorded gait.
    const bool planted = telemetry.legState[leg] != 2;
    const float friction = planted ? 4.5f : 0.08f;
    const std::string footName = impl_->biped
        ? std::string("Front ") + (leg == 0 ? "Left" : "Right") + " Foot"
        : std::string(kLegNames[leg]) + " Foot";
    const int footGeom = mj_name2id(impl_->model, mjOBJ_GEOM, footName.c_str());
    if (footGeom >= 0) {
      impl_->model->geom_friction[3 * footGeom] = friction;
      impl_->model->geom_friction[3 * footGeom + 1] = 0.02;
      impl_->model->geom_friction[3 * footGeom + 2] = 0.001;
    }
    telemetry.footFriction[leg] = friction;
  }
  mj_step(impl_->model, impl_->data);

  for (SceneObject &object : scene.objects()) {
    const int id = mj_name2id(impl_->model, impl_->unitreeH1 ? mjOBJ_BODY : mjOBJ_GEOM, object.name.c_str());
    if (id < 0) continue;
    const mjtNum *position = impl_->unitreeH1 ? impl_->data->xpos + 3 * id : impl_->data->geom_xpos + 3 * id;
    const mjtNum *rotation = impl_->unitreeH1 ? impl_->data->xmat + 9 * id : impl_->data->geom_xmat + 9 * id;
    object.transform.position = {static_cast<float>(position[0]), static_cast<float>(position[2]), static_cast<float>(position[1])};
    constexpr std::array<int, 3> axisMap{{0, 2, 1}};
    const auto mapped = [&](int row, int column) {
      return static_cast<float>(rotation[3 * axisMap[row] + axisMap[column]]);
    };
    object.transform.rotation = {
        std::atan2(mapped(2, 1), mapped(2, 2)),
        std::asin(clampUnit(-mapped(2, 0))),
        std::atan2(mapped(1, 0), mapped(0, 0))};
  }
}

void MuJoCoBridge::demolish(const Scene &) {
  if (!impl_->data) return;
  impl_->data->qvel[0] = 1.5;
  impl_->data->qvel[1] = -1.0;
  impl_->data->qvel[2] = 2.5;
}

void MuJoCoBridge::setRobotScript(int script) {
  if (!impl_->ready) return;
  // Walking a biped needs an explicit COM-over-foot controller. Until that
  // exists, keep the two-legged configuration in its verified standing pose.
  impl_->script = (impl_->biped || impl_->unitreeH1) ? 0 : std::clamp(script, 0, 3);
  impl_->scriptTime = 0.0f;
}

void MuJoCoBridge::setStandingTuning(const StandingTuning &tuning) { impl_->tuning = tuning; }
int MuJoCoBridge::robotScript() const { return impl_ ? impl_->script : 0; }
const RobotTelemetry &MuJoCoBridge::telemetry() const { return impl_->telemetry; }
bool MuJoCoBridge::initialized() const { return impl_ && impl_->ready; }

void MuJoCoBridge::shutdown() {
  if (!impl_) return;
  if (impl_->data) { mj_deleteData(impl_->data); impl_->data = nullptr; }
  if (impl_->model) { mj_deleteModel(impl_->model); impl_->model = nullptr; }
  impl_->ready = false;
}
