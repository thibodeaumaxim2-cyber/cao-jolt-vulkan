#include "MuJoCoBridge.hpp"
#include "NavigationMacroPolicy.hpp"
#include "FullBodyGoalPolicy.hpp"
#include "UnitreePolicy.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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

// Deliberately close profiles make online exploration safe: the learner can
// refine a step in simulation without trying a radically different gait.
struct H1GaitProfile {
  float cycleSeconds;
  float strideRadians;
  float kneeLiftRadians;
  float ankleLiftRadians;
  float supportShiftMeters;
};
constexpr std::array<H1GaitProfile, 3> kH1GaitProfiles{{
    // Conservative alternating range proven to complete left/right transfers
    // under the 88% balance gate. A 30 cm placement needs a dedicated
    // whole-body/ZMP planner; the learner must not explore that unsafe range.
    {2.60f, 0.045f, 0.260f, 0.070f, 0.015f},
    {2.50f, 0.055f, 0.280f, 0.075f, 0.020f},
    {2.70f, 0.060f, 0.270f, 0.072f, 0.020f},
}};

constexpr std::array<std::array<float, 2>, 16> kNavigationObstacles{{
    {{1.45f, -0.65f}}, {{1.45f, -0.22f}}, {{1.45f, 0.22f}},
    {{2.40f, -0.38f}}, {{2.40f, 0.00f}}, {{2.40f, 0.38f}},
    {{2.78f, -0.19f}}, {{2.78f, 0.19f}}, {{3.16f, 0.00f}},
    {{3.65f, 0.00f}}, {{3.65f, 0.44f}}, {{3.65f, 0.88f}},
    {{4.85f, -0.88f}}, {{4.85f, -0.44f}}, {{4.85f, 0.00f}},
    {{5.70f, 0.45f}}
}};

std::vector<std::array<float, 2>> makeNavigationPath() {
  constexpr float cell = 0.25f, minX = -0.50f, minY = -1.50f;
  constexpr int width = 31, height = 13;
  const auto node = [](int x, int y) { return y * 31 + x; };
  const auto toCell = [](float value, float minimum) { return static_cast<int>(std::round((value - minimum) / cell)); };
  const int start = node(toCell(0.0f, minX), toCell(0.0f, minY));
  const int goal = node(toCell(6.20f, minX), toCell(0.0f, minY));
  std::array<bool, width * height> blocked{};
  for (const auto &obstacle : kNavigationObstacles) {
    const int cx = toCell(obstacle[0], minX), cy = toCell(obstacle[1], minY);
    for (int y = cy - 1; y <= cy + 1; ++y) for (int x = cx - 1; x <= cx + 1; ++x)
      if (x >= 0 && x < width && y >= 0 && y < height) blocked[node(x, y)] = true;
  }
  blocked[start] = false; blocked[goal] = false;
  struct Entry { float score; int value; bool operator<(const Entry &other) const { return score > other.score; } };
  std::priority_queue<Entry> open;
  std::array<float, width * height> cost; cost.fill(std::numeric_limits<float>::infinity());
  std::array<int, width * height> parent; parent.fill(-1);
  cost[start] = 0.0f; open.push({0.0f, start});
  constexpr std::array<std::array<int, 2>, 8> directions{{{{1,0}},{{-1,0}},{{0,1}},{{0,-1}},{{1,1}},{{1,-1}},{{-1,1}},{{-1,-1}}}};
  while (!open.empty()) {
    const int current = open.top().value; open.pop();
    if (current == goal) break;
    const int x = current % width, y = current / width;
    for (const auto &delta : directions) {
      const int nx = x + delta[0], ny = y + delta[1];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height || blocked[node(nx, ny)]) continue;
      const int next = node(nx, ny);
      const float nextCost = cost[current] + (delta[0] && delta[1] ? 1.4142f : 1.0f);
      if (nextCost >= cost[next]) continue;
      cost[next] = nextCost; parent[next] = current;
      const float heuristic = std::hypot(static_cast<float>(nx - goal % width), static_cast<float>(ny - goal / width));
      open.push({nextCost + heuristic, next});
    }
  }
  std::vector<std::array<float, 2>> path;
  for (int current = goal; current >= 0; current = parent[current]) {
    path.push_back({minX + cell * static_cast<float>(current % width), minY + cell * static_cast<float>(current / width)});
    if (current == start) break;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace

struct MuJoCoBridge::Impl {
  UnitreePolicy unitreePolicy;
  NavigationMacroPolicy navigationMacro;
  FullBodyGoalPolicy fullBodyPolicy;
  bool officialPolicy = false;
  bool fullBodyMode = false;
  int fullBodyGoal = 0;
  double policyAccumulator = 0;
  int policySteps = 0;
  std::array<float,10> officialAction{};
  bool jumpPending = false;
  int jumpIndex = 0;
  std::vector<std::array<float, 2>> navigationPath;
  size_t navigationWaypoint = 0;
  struct H1MotionReferenceSample { bool leftSwing; float stride; float knee; float ankle; float arm; };
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
  int h1LearningProfile = 0;
  int h1LearningSamples = 0;
  float h1LearningStartTime = 0.0f;
  float h1LearningStartX = 0.0f;
  float h1GaitTime = 0.0f;
  float h1EquilibriumScore = 1.0f;
  float h1SwingGraceSeconds = 0.0f;
  bool h1PolicyLoaded = false;
  bool feetInitialized = false;
  int swingFoot = 0;
  float footPhase = 0.0f;
  std::array<float, 2> plantedX{};
  float swingStartX = 0.0f;
  float swingEndX = 0.0f;
  bool footWasAirborne = false;
  std::array<std::array<float, 4>, 3> h1PolicyWeight{};
  std::array<float, 3> h1PolicyBias{};
  std::vector<H1MotionReferenceSample> h1MotionReference;
  std::array<float, kH1GaitProfiles.size()> h1RewardTotal{};
  std::array<int, kH1GaitProfiles.size()> h1RewardCount{};
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
  impl_->officialPolicy = impl_->script == 5;
  impl_->jumpPending = false;
  impl_->jumpIndex = 0;
    auto h1Scene = std::filesystem::path(CAO_SOURCE_DIR) / "third_party" / "unitree_mujoco" /
        "unitree_robots" / "h1" / "scene.xml";
    if (impl_->officialPolicy) {
      h1Scene = std::filesystem::path(CAO_SOURCE_DIR) / "assets" / "unitree_h1" /
          (impl_->fullBodyMode ? "scene_full.xml" : "scene.xml");
      impl_->unitreePolicy.load((h1Scene.parent_path() / "weights.json").string());
      impl_->navigationMacro.load((std::filesystem::path(CAO_SOURCE_DIR) / "assets" / "navigation_macro_policy.json").string());
      if (impl_->fullBodyMode) impl_->fullBodyPolicy.load((std::filesystem::path(CAO_SOURCE_DIR) / "assets" / "fullbody_goal_policy.json").string());
      impl_->officialAction.fill(0);
      impl_->policySteps = 0;
      impl_->policyAccumulator = 0;
    }
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
  if (impl_->unitreeH1 && impl_->officialPolicy &&
      (mj_name2id(impl_->model, mjOBJ_ACTUATOR, "left_hip_yaw_joint") < 0 ||
       mj_name2id(impl_->model, mjOBJ_ACTUATOR, "right_ankle_joint") < 0))
    throw std::runtime_error("Unitree H1 policy requires all ten leg actuators (nq=" +
        std::to_string(impl_->model->nq) + ", nv=" + std::to_string(impl_->model->nv) +
        ", nu=" + std::to_string(impl_->model->nu) + ")");
  if (impl_->unitreeH1 && (!impl_->officialPolicy || impl_->fullBodyMode)) mj_resetDataKeyframe(impl_->model, impl_->data, 0);
  if (impl_->unitreeH1 && !impl_->officialPolicy) {
    // The upstream visual/collision assets use the default friction of 1.0.
    // Raise the floor and sole contact friction for a stationary strength test.
    const int floor = mj_name2id(impl_->model, mjOBJ_GEOM, "floor");
    if (floor >= 0) impl_->model->geom_friction[3 * floor] = 3.0;
    for (int geom = 0; geom < impl_->model->ngeom; ++geom)
      if (impl_->model->geom_group[geom] == 3) impl_->model->geom_friction[3 * geom] = 2.2;
  }
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
  impl_->h1LearningProfile = 0;
  impl_->h1LearningSamples = 0;
  impl_->h1LearningStartTime = 0.0f;
  impl_->h1LearningStartX = impl_->data->qpos[0];
  impl_->h1GaitTime = 0.0f;
  impl_->h1EquilibriumScore = 1.0f;
  impl_->h1SwingGraceSeconds = 0.0f;
  impl_->h1PolicyLoaded = false;
  impl_->feetInitialized = false;
  impl_->footPhase = 0.0f;
  impl_->swingFoot = 0;
  impl_->footWasAirborne = false;
  const auto policyPath = std::filesystem::path(CAO_SOURCE_DIR) / "assets" / "h1_locomotion_policy.json";
  if (std::ifstream stream{policyPath}; stream) {
    try {
      nlohmann::json policy; stream >> policy;
      if (policy.value("format", "") == "cao_h1_linear_policy_v1") {
        const auto &weight = policy.at("weight"); const auto &bias = policy.at("bias");
        for (size_t row = 0; row < 3; ++row) for (size_t col = 0; col < 4; ++col)
          impl_->h1PolicyWeight[row][col] = weight.at(row).at(col).get<float>();
        for (size_t row = 0; row < 3; ++row) impl_->h1PolicyBias[row] = bias.at(row).get<float>();
        impl_->h1PolicyLoaded = true;
      }
    } catch (const std::exception &) { impl_->h1PolicyLoaded = false; }
  }
  impl_->h1MotionReference.clear();
  const auto motionPath = std::filesystem::path(CAO_SOURCE_DIR) / "assets" / "h1_motion_reference.json";
  if (std::ifstream stream{motionPath}; stream) {
    try {
      nlohmann::json motion; stream >> motion;
      if (motion.value("format", "") == "cao_h1_motion_reference_v1") {
        for (const auto &sample : motion.at("samples")) impl_->h1MotionReference.push_back({
            sample.at("left_swing").get<bool>(), sample.at("stride_rad").get<float>(),
            sample.at("knee_lift_rad").get<float>(), sample.at("ankle_lift_rad").get<float>(),
            sample.at("arm_drive_rad").get<float>()});
      }
    } catch (const std::exception &) { impl_->h1MotionReference.clear(); }
  }
  impl_->h1RewardTotal.fill(0.0f);
  impl_->h1RewardCount.fill(0);
  impl_->telemetry = {};
  impl_->telemetry.linkCount = static_cast<int>(scene.objects().size());
  if (impl_->unitreeH1 && impl_->officialPolicy) {
    impl_->navigationPath = makeNavigationPath();
    impl_->navigationWaypoint = impl_->navigationPath.size() > 1 ? 1 : 0;
  }
}

void MuJoCoBridge::step(Scene &scene, float seconds) {
  if (!impl_->ready || !impl_->model || !impl_->data || seconds <= 0.0f) return;
  if (impl_->unitreeH1 && (impl_->script == 5) != impl_->officialPolicy) rebuild(scene);
  if (impl_->unitreeH1 && impl_->officialPolicy) {
    constexpr std::array<float,10> home{{0,0,-.1f,.3f,-.2f,0,0,-.1f,.3f,-.2f}};
    constexpr std::array<float,10> kp{{150,150,150,200,40,150,150,150,200,40}};
    constexpr std::array<float,10> kd{{2,2,2,4,2,2,2,2,4,2}};
    auto *d=impl_->data; auto *m=impl_->model;
    if (impl_->jumpPending) {
      const int jumpIndex = impl_->jumpIndex % 4;
      const std::array<float,4> lateral{{0.0f, 0.8f, -0.7f, 0.35f}};
      const std::array<float,4> forward{{0.3f, 0.0f, 0.25f, -0.2f}};
      d->qvel[2] = jumpIndex == 3 ? 1.8 : 2.8;
      d->qvel[0] = forward[jumpIndex]; d->qvel[1] = lateral[jumpIndex];
      d->qvel[3] = jumpIndex == 3 ? 1.4 : 0.35 * lateral[jumpIndex];
      d->qvel[4] = jumpIndex == 3 ? -0.9 : 0.0;
      d->qvel[5] = 0.25 * forward[jumpIndex];
      impl_->jumpPending = false;
      ++impl_->jumpIndex;
    }
    // The pretrained Unitree policy is ordered by these ten leg joints. The
    // full H1 model also has waist and arm motors, so never rely on actuator
    // index ordering here.
    constexpr std::array<const char *, 10> legJointNames{{
        "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint", "left_ankle_joint",
        "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "right_ankle_joint"}};
    std::array<int,10> jointQpos{}, jointDof{}, legActuator{};
    for(int i=0;i<10;++i) {
      const int joint=mj_name2id(m, mjOBJ_JOINT, legJointNames[i]);
      legActuator[i]=mj_name2id(m, mjOBJ_ACTUATOR, legJointNames[i]);
      jointQpos[i]=m->jnt_qposadr[joint]; jointDof[i]=m->jnt_dofadr[joint];
    }
    impl_->policyAccumulator += seconds;
    m->opt.timestep=.002;
    while(impl_->policyAccumulator >= .002) {
      for(int i=0;i<10;++i)
        d->ctrl[legActuator[i]]=kp[i]*(home[i]+.25f*impl_->officialAction[i]-d->qpos[jointQpos[i]])-kd[i]*d->qvel[jointDof[i]];
      const auto holdJoint = [&](const char *name, float target, float stiffness, float damping) {
        const int actuator=mj_name2id(m,mjOBJ_ACTUATOR,name);
        const int joint=mj_name2id(m,mjOBJ_JOINT,name);
        if(actuator<0 || joint<0) return;
        const int qpos=m->jnt_qposadr[joint], dof=m->jnt_dofadr[joint];
        const float torque=stiffness*(target-static_cast<float>(d->qpos[qpos]))-damping*static_cast<float>(d->qvel[dof]);
        d->ctrl[actuator]=std::clamp(torque, static_cast<float>(m->actuator_ctrlrange[2*actuator]),
            static_cast<float>(m->actuator_ctrlrange[2*actuator+1]));
      };
      // Full-body neutral hold: active torso and arms without changing the
      // ten-joint locomotion policy's observation/action contract.
      holdJoint("torso_joint", 0.0f, 75.0f, 6.0f);
      holdJoint("left_shoulder_pitch_joint", 0.18f, 18.0f, 2.0f);
      holdJoint("right_shoulder_pitch_joint", 0.18f, 18.0f, 2.0f);
      holdJoint("left_shoulder_roll_joint", 0.05f, 12.0f, 1.5f);
      holdJoint("right_shoulder_roll_joint", -0.05f, 12.0f, 1.5f);
      holdJoint("left_shoulder_yaw_joint", 0.0f, 8.0f, 1.0f);
      holdJoint("right_shoulder_yaw_joint", 0.0f, 8.0f, 1.0f);
      holdJoint("left_elbow_joint", 0.45f, 8.0f, 1.0f);
      holdJoint("right_elbow_joint", 0.45f, 8.0f, 1.0f);
      if (impl_->fullBodyMode) {
        constexpr std::array<const char *,19> names{{"torso_joint","left_hip_yaw_joint","left_hip_roll_joint","left_hip_pitch_joint","left_knee_joint","left_ankle_joint","right_hip_yaw_joint","right_hip_roll_joint","right_hip_pitch_joint","right_knee_joint","right_ankle_joint","left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint"}};
        constexpr std::array<float,19> scales{{1.2f,.43f,.43f,1.57f,2.05f,.87f,.43f,.43f,1.57f,2.05f,.87f,2.2f,1.5f,2.2f,2.0f,2.2f,1.5f,2.2f,2.0f}};
        std::array<float,42> input{};
        for(size_t i=0;i<names.size();++i) { const int joint=mj_name2id(m,mjOBJ_JOINT,names[i]); input[i]=static_cast<float>(d->qpos[m->jnt_qposadr[joint]])/scales[i]; input[19+i]=.08f*static_cast<float>(d->qvel[m->jnt_dofadr[joint]]); }
        input[38+std::clamp(impl_->fullBodyGoal,0,3)]=1.0f;
        const auto pose=impl_->fullBodyPolicy.infer(input);
        for(size_t i=0;i<names.size();++i) holdJoint(names[i], pose[i]*scales[i], i<11 ? 90.0f : 22.0f, i<11 ? 8.0f : 2.0f);
        // The pose network controls joints, not the floating base. Keep its
        // correction task physically meaningful by applying only bounded IMU
        // stabilization torques to the free pelvis while contacts settle.
        const float w=static_cast<float>(d->qpos[3]), x=static_cast<float>(d->qpos[4]);
        const float y=static_cast<float>(d->qpos[5]), z=static_cast<float>(d->qpos[6]);
        const float roll=std::atan2(2.0f*(w*x+y*z),1.0f-2.0f*(x*x+y*y));
        const float pitch=std::asin(std::clamp(2.0f*(w*y-z*x),-1.0f,1.0f));
        d->qfrc_applied[3]=std::clamp(-420.0f*roll-75.0f*static_cast<float>(d->qvel[3]),-220.0f,220.0f);
        d->qfrc_applied[4]=std::clamp(-480.0f*pitch-85.0f*static_cast<float>(d->qvel[4]),-240.0f,240.0f);
      }
      mj_step(m,d);
      ++impl_->policySteps;
      if(impl_->policySteps%10==0) {
        std::array<float,41> obs{};
        for(int i=0;i<3;++i) obs[i]=.25f*d->qvel[3+i];
        const double w=d->qpos[3],x=d->qpos[4],y=d->qpos[5],z=d->qpos[6];
        obs[3]=2*(-z*x+w*y); obs[4]=-2*(z*y+w*x); obs[5]=1-2*(w*w+z*z);
        // A* supplies the next collision-free waypoint. Convert the world
        // vector into H1's body frame and use Unitree's velocity/yaw command.
        const float robotX = static_cast<float>(d->qpos[0]);
        const float robotY = static_cast<float>(d->qpos[1]);
        if (impl_->navigationWaypoint < impl_->navigationPath.size()) {
          const auto target = impl_->navigationPath[impl_->navigationWaypoint];
          if (std::hypot(target[0] - robotX, target[1] - robotY) < 0.30f &&
              impl_->navigationWaypoint + 1 < impl_->navigationPath.size()) ++impl_->navigationWaypoint;
        }
        const auto target = impl_->navigationPath.empty() ? std::array<float, 2>{{robotX, robotY}} :
            impl_->navigationPath[impl_->navigationWaypoint];
        const float dx = target[0] - robotX, dy = target[1] - robotY;
        const float distance = std::hypot(dx, dy);
        const float yaw = std::atan2(2.0f * static_cast<float>(w * z + x * y),
                                     1.0f - 2.0f * static_cast<float>(y * y + z * z));
        const float desiredYaw = distance > 0.05f ? std::atan2(dy, dx) : yaw;
        const float yawError = std::atan2(std::sin(desiredYaw - yaw), std::cos(desiredYaw - yaw));
        const float localX = std::cos(yaw) * dx + std::sin(yaw) * dy;
        const float localY = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        const auto macro = impl_->navigationMacro.infer({{
            std::clamp(localX / 1.5f, -1.0f, 1.0f), std::clamp(localY / 1.5f, -1.0f, 1.0f),
            std::clamp(yawError / 1.2f, -1.0f, 1.0f), std::clamp(distance / 2.0f, 0.0f, 1.0f)}});
        // A* supplies the collision-free nominal direction. The local neural
        // macro is the command gate: if it cannot produce a meaningful
        // locomotion intent, no walking command reaches the low-level policy.
        // Direction stays geometric because a tiny learned steering drift can
        // accumulate into a collision over this long obstacle course.
        const float macroPlanar = std::hypot(macro[0], macro[1]);
        const bool macroWantsToNavigate = macroPlanar > 0.01f;
        const float cruise = std::min(0.50f, distance * 2.0f);
        const float nominalX = distance > 0.01f ? cruise * localX / distance : 0.0f;
        const float nominalY = distance > 0.01f ? cruise * localY / distance : 0.0f;
        obs[6] = macroWantsToNavigate ? std::clamp(nominalX, -0.50f, 0.50f) : 0.0f;
        obs[7] = macroWantsToNavigate ? std::clamp(nominalY, -0.50f, 0.50f) : 0.0f;
        const float nominalYaw = std::clamp(2.0f * yawError, -0.20f, 0.20f);
        obs[8] = macroWantsToNavigate ? nominalYaw : 0.0f;
        for(int i=0;i<10;++i) {
          obs[9+i]=d->qpos[jointQpos[i]]-home[i]; obs[19+i]=.05f*d->qvel[jointDof[i]];
          obs[29+i]=impl_->officialAction[i];
        }
        const float phase=std::fmod(impl_->policySteps*.002/.8,1.0);
        obs[39]=std::sin(6.283185307f*phase); obs[40]=std::cos(6.283185307f*phase);
        impl_->officialAction=impl_->unitreePolicy.infer(obs);
      }
      impl_->policyAccumulator-=.002;
    }
    mj_forward(m,d);
    auto &t=impl_->telemetry; t={}; t.motionScript=5;
    t.linkCount=scene.objects().size();
    t.torsoSpeedMps=std::hypot(d->qvel[0],d->qvel[1]);
    t.gaitCycle=std::fmod(d->time/.8,1.0);
    t.activeSwingLeg=t.gaitCycle<.5f ? 0:1;
    t.walkingAllowed=true;
    t.equilibriumScore=std::clamp(static_cast<float>((d->qpos[2]-.65)/.35),0.f,1.f);
    t.navigationGoalDistanceM = std::hypot(static_cast<float>(d->qpos[0] - 6.20), static_cast<float>(d->qpos[1]));
    t.navigationGoalReached = t.navigationGoalDistanceM <= 0.35f;
    for(int leg=0;leg<2;++leg) {
      const int foot=mj_name2id(m,mjOBJ_BODY,leg==0 ? "left_ankle_link":"right_ankle_link");
      for(int i=0;i<d->ncon;++i) {
        const auto &hit=d->contact[i];
        const int a=m->geom_bodyid[hit.geom[0]], b=m->geom_bodyid[hit.geom[1]];
        if((a==foot && b==0)||(b==foot && a==0)) {
          mjtNum force[6]{}; mj_contactForce(m,d,i,force);
          t.footNormalForceN[leg]+=std::max(0.0,force[0]);
        }
      }
      t.footContact[leg]=t.footNormalForceN[leg]>5.0f;
    }
    t.activeSwingLeg = t.footContact[0] && !t.footContact[1] ? 1 :
        t.footContact[1] && !t.footContact[0] ? 0 : -1;
  } else {
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
  if (impl_->unitreeH1) {
    // Unitree's home keyframe is a compact, bent-leg stance. Each motor is
    // torque controlled, so retain that pose with PD strength and correct
    // torso tilt through the high-torque hip/knee joints and ankle pitch.
    const int pelvis = mj_name2id(impl_->model, mjOBJ_BODY, "pelvis");
    const mjtNum *rotation = pelvis >= 0 ? impl_->data->xmat + 9 * pelvis : nullptr;
    const float roll = rotation ? std::atan2(static_cast<float>(rotation[7]), static_cast<float>(rotation[8])) : 0.0f;
    const float pitch = rotation ? std::asin(clampUnit(-static_cast<float>(rotation[6]))) : 0.0f;
    const float rollRate = static_cast<float>(impl_->data->qvel[3]);
    const float pitchRate = static_cast<float>(impl_->data->qvel[4]);
    const float pitchCorrection = std::clamp(-48.0f * pitch - 8.0f * pitchRate, -28.0f, 28.0f);
    // A bounded virtual IMU balance assist prevents the unconstrained torso
    // from collapsing while the stance controller establishes contact. This
    // is a temporary whole-body stabilizer, not a substitute for locomotion.
    impl_->data->qfrc_applied[3] = std::clamp(-1250.0f * roll - 180.0f * rollRate, -500.0f, 500.0f);
    impl_->data->qfrc_applied[4] = std::clamp(-1350.0f * pitch - 200.0f * pitchRate, -550.0f, 550.0f);
    const auto torquePd = [&](const char *actuatorName, float target, float kp, float kd, float balanceTorque = 0.0f) {
      const int actuator = mj_name2id(impl_->model, mjOBJ_ACTUATOR, actuatorName);
      if (actuator < 0) return;
      const int joint = impl_->model->actuator_trnid[2 * actuator];
      const int qpos = impl_->model->jnt_qposadr[joint];
      const int dof = impl_->model->jnt_dofadr[joint];
      const float torque = kp * (target - static_cast<float>(impl_->data->qpos[qpos])) -
          kd * static_cast<float>(impl_->data->qvel[dof]) + balanceTorque;
      const float minimum = static_cast<float>(impl_->model->actuator_ctrlrange[2 * actuator]);
      const float maximum = static_cast<float>(impl_->model->actuator_ctrlrange[2 * actuator + 1]);
      impl_->data->ctrl[actuator] = std::clamp(torque, minimum, maximum);
    };
    float leftHipPitch = -0.40f, rightHipPitch = -0.40f;
    float leftKnee = 0.80f, rightKnee = 0.80f;
    float leftAnkle = -0.40f, rightAnkle = -0.40f;
    float leftShoulderPitch = 0.20f, rightShoulderPitch = 0.20f;
    float leftShoulderRoll = 0.0f, rightShoulderRoll = 0.0f;
    // First let the torque controller settle the imported model in double
    // support. A gait phase must never advance while the balance gate is
    // closed; otherwise it resumes halfway through a swing and kicks the
    // support foot.
    // Use the previous H1-specific IMU/pose score, not the generic robot
    // metric calculated before the H1 controller has updated its telemetry.
    // Let a lifted foot finish its forward arc through a brief, bounded score
    // dip. Outside the middle of a swing, 88% remains mandatory; below 72%
    // it freezes immediately. This prevents the gate from locking a foot in
    // place just before touchdown while retaining a hard recovery boundary.
    const float gateCycle = std::fmod(impl_->h1GaitTime / 2.60f, 1.0f);
    const float gatePhase = std::fmod(gateCycle * 2.0f, 1.0f);
    const bool middleOfSwing = gatePhase > 0.18f && gatePhase < 0.82f;
    const bool graceEligible = middleOfSwing && impl_->h1EquilibriumScore >= 0.72f &&
                               impl_->h1EquilibriumScore < 0.88f;
    impl_->h1SwingGraceSeconds = graceEligible ? impl_->h1SwingGraceSeconds + seconds : 0.0f;
    const bool h1CanStep = impl_->scriptTime >= 1.25f &&
        (impl_->h1EquilibriumScore >= 0.88f || (graceEligible && impl_->h1SwingGraceSeconds <= 0.55f));
    if (impl_->script >= 1 && h1CanStep)
      impl_->h1GaitTime += seconds;
    // When balance is temporarily below 88%, hold the current pose and
    // freeze the phase. Do not erase the phase: that caused the left leg to
    // be selected repeatedly and the right leg never got a turn.
    if (impl_->script >= 1 && impl_->scriptTime >= 1.25f) {
      // Quasi-static assisted stepping: first move the pelvis toward the
      // planted foot, then flex and advance exactly one swing leg. The short
      // cycle puts the foot down before the support margin is exhausted.
      H1GaitProfile profile = kH1GaitProfiles[1];
      if (impl_->script == 3)
        profile = {3.80f, 0.380f, 0.420f, 0.120f, 0.055f};
      if (impl_->script == 2) {
        // One full left/right cycle is one training sample. Reward forward
        // distance, but penalize low-equilibrium trajectories. The first
        // three cycles explore each conservative profile; thereafter the
        // highest average-reward profile is used, with periodic re-checks.
        const float elapsed = impl_->h1GaitTime - impl_->h1LearningStartTime;
        const H1GaitProfile current = kH1GaitProfiles[impl_->h1LearningProfile];
        if (elapsed >= current.cycleSeconds) {
          const float distance = static_cast<float>(impl_->data->qpos[0]) - impl_->h1LearningStartX;
          const float reward = distance * 8.0f - 2.0f * (1.0f - telemetry.equilibriumScore);
          const int completed = impl_->h1LearningProfile;
          impl_->h1RewardTotal[completed] += reward;
          ++impl_->h1RewardCount[completed];
          ++impl_->h1LearningSamples;
          impl_->h1LearningStartTime = impl_->h1GaitTime;
          impl_->h1LearningStartX = impl_->data->qpos[0];
          if (impl_->h1LearningSamples < static_cast<int>(kH1GaitProfiles.size())) {
            impl_->h1LearningProfile = impl_->h1LearningSamples;
          } else if (impl_->h1LearningSamples % 6 == 0) {
            impl_->h1LearningProfile = (impl_->h1LearningProfile + 1) % static_cast<int>(kH1GaitProfiles.size());
          } else {
            float bestReward = -std::numeric_limits<float>::infinity();
            for (size_t i = 0; i < kH1GaitProfiles.size(); ++i) {
              const float average = impl_->h1RewardTotal[i] / std::max(1, impl_->h1RewardCount[i]);
              if (average > bestReward) { bestReward = average; impl_->h1LearningProfile = static_cast<int>(i); }
            }
          }
        }
        profile = kH1GaitProfiles[impl_->h1LearningProfile];
        telemetry.learningActive = true;
        telemetry.learningProfile = impl_->h1LearningProfile;
        telemetry.learningSamples = impl_->h1LearningSamples;
        const int count = impl_->h1RewardCount[impl_->h1LearningProfile];
        telemetry.learningReward = count == 0 ? 0.0f : impl_->h1RewardTotal[impl_->h1LearningProfile] / count;
      }
      const float cycle = std::fmod((impl_->h1GaitTime - impl_->h1LearningStartTime) / profile.cycleSeconds, 1.0f);
      bool leftSwing = cycle < 0.5f;
      const float phase = std::fmod(cycle * 2.0f, 1.0f);
      const float lift = std::sin(3.14159265f * phase);
      // Do not start a new swing by snapping the hip rearward: that was
      // effectively a shove on the pelvis. Advance from the home pose with a
      // smooth, deliberately short arc and let the planted foot supply drive.
      float stride = profile.strideRadians * (1.0f - std::cos(3.14159265f * phase));
      float kneeLift = profile.kneeLiftRadians * lift;
      float ankleLift = profile.ankleLiftRadians * lift;
      if (impl_->script == 2 && impl_->h1PolicyLoaded) {
        const std::array<float, 4> input{{std::sin(6.2831853f * cycle), std::cos(6.2831853f * cycle), telemetry.equilibriumScore, leftSwing ? 1.0f : -1.0f}};
        std::array<float, 3> output{};
        for (size_t row = 0; row < output.size(); ++row) {
          output[row] = impl_->h1PolicyBias[row];
          for (size_t col = 0; col < input.size(); ++col) output[row] += impl_->h1PolicyWeight[row][col] * input[col];
        }
        stride = std::clamp(output[0], 0.0f, 0.12f);
        kneeLift = std::clamp(output[1], 0.0f, 0.34f);
        ankleLift = std::clamp(output[2], -0.12f, 0.0f);
      }
      // The motion-reference clip is a direct retarget of the imported CC0
      // walk. It replaces only swing offsets; MuJoCo remains authoritative
      // for contact, torque limits, and the balance gate.
      float referenceArmDrive = 0.0f;
      if (impl_->script == 2 && !impl_->h1MotionReference.empty()) {
        const size_t index = std::min(static_cast<size_t>(cycle * impl_->h1MotionReference.size()),
                                      impl_->h1MotionReference.size() - 1u);
        const auto &reference = impl_->h1MotionReference[index];
        leftSwing = reference.leftSwing;
        stride = std::clamp(reference.stride, 0.0f, 0.12f);
        kneeLift = std::clamp(reference.knee, 0.0f, 0.34f);
        ankleLift = std::clamp(reference.ankle, -0.12f, 0.0f);
        referenceArmDrive = std::clamp(reference.arm, 0.0f, 0.34f);
      }
      if (impl_->script == 3) {
        // ZMP-inspired alternating gait. The planted leg sweeps from front to
        // rear while the other folds and travels rear to front, retaining the
        // foot placement instead of resetting it at the next half-cycle.
        const float plantedHip = profile.strideRadians * (1.0f - 2.0f * phase);
        const float swingHip = profile.strideRadians * (-1.0f + 2.0f * phase);
        if (leftSwing) {
          leftHipPitch += swingHip; leftKnee += profile.kneeLiftRadians * lift; leftAnkle -= profile.ankleLiftRadians * lift;
          rightHipPitch += plantedHip;
        } else {
          rightHipPitch += swingHip; rightKnee += profile.kneeLiftRadians * lift; rightAnkle -= profile.ankleLiftRadians * lift;
          leftHipPitch += plantedHip;
        }
      } else if (leftSwing) {
        leftHipPitch += stride; leftKnee += kneeLift; leftAnkle += ankleLift;
        rightHipPitch -= 0.015f;
      } else {
        rightHipPitch += stride; rightKnee += kneeLift; rightAnkle += ankleLift;
        leftHipPitch -= 0.015f;
      }
      // Arms provide counter-momentum while the opposite leg is unloaded.
      // Roll biases move arm mass toward the planted side without pushing the
      // shoulder joints near their asymmetric mechanical limits.
      const float armDrive = referenceArmDrive > 0.0f ? referenceArmDrive : 0.34f * lift;
      if (leftSwing) {
        rightShoulderPitch += armDrive; leftShoulderPitch -= 0.12f * lift;
        rightShoulderRoll = -0.10f * lift; leftShoulderRoll = 0.08f * lift;
      } else {
        leftShoulderPitch += armDrive; rightShoulderPitch -= 0.12f * lift;
        leftShoulderRoll = 0.10f * lift; rightShoulderRoll = -0.08f * lift;
      }
      const float supportY = leftSwing ? -profile.supportShiftMeters : profile.supportShiftMeters;
      const float lateralForce = std::clamp(300.0f * (supportY - static_cast<float>(impl_->data->qpos[1])) -
          50.0f * static_cast<float>(impl_->data->qvel[1]), -60.0f, 60.0f);
      impl_->data->qfrc_applied[1] = lateralForce;
      if (impl_->script == 3 && pelvis >= 0) {
        const int supportFoot = mj_name2id(impl_->model, mjOBJ_BODY,
                                           leftSwing ? "right_ankle_link" : "left_ankle_link");
        if (supportFoot >= 0) {
          const float comX = static_cast<float>(impl_->data->subtree_com[3 * pelvis]);
          const float footX = static_cast<float>(impl_->data->xpos[3 * supportFoot]);
          // Keep the COM projection near the planted ankle before allowing a
          // large 30 cm placement; this is the bounded ZMP proxy layer.
          impl_->data->qfrc_applied[0] = std::clamp(220.0f * (footX - comX) -
              45.0f * static_cast<float>(impl_->data->qvel[0]), -70.0f, 70.0f);
        }
      }
      telemetry.gaitCycle = cycle;
      telemetry.activeSwingLeg = leftSwing ? 0 : 1;
      telemetry.legState[leftSwing ? 0 : 1] = 2;
    }
    if (impl_->script == 4 && impl_->scriptTime >= 1.25f) {
      // Hold stance targets in world space. Advance to the next leg only
      // after the current swing has left the ground and made contact again.
      const std::array<int, 2> feet{{
          mj_name2id(impl_->model, mjOBJ_BODY, "left_ankle_link"),
          mj_name2id(impl_->model, mjOBJ_BODY, "right_ankle_link")}};
      std::array<bool, 2> contact{};
      for (int c = 0; c < impl_->data->ncon; ++c) {
        const auto &hit = impl_->data->contact[c];
        for (int leg = 0; leg < 2; ++leg) {
          const int a = impl_->model->geom_bodyid[hit.geom[0]];
          const int b = impl_->model->geom_bodyid[hit.geom[1]];
          if ((a == feet[leg] && b == 0) || (b == feet[leg] && a == 0))
            contact[leg] = true;
        }
      }
      if (!impl_->feetInitialized) {
        for (int leg = 0; leg < 2; ++leg)
          impl_->plantedX[leg] = impl_->data->xpos[3 * feet[leg]];
        impl_->swingStartX = impl_->plantedX[0];
        impl_->swingEndX = impl_->plantedX[1] + 0.12f;
        impl_->feetInitialized = true;
      }
      const int swing = impl_->swingFoot;
      const int support = 1 - swing;
      if (contact[support] && (impl_->footPhase > 0.0f || h1CanStep))
        impl_->footPhase = std::min(1.0f, impl_->footPhase + seconds / 1.1f);
      const float u = impl_->footPhase;
      if (u > 0.15f && !contact[swing]) impl_->footWasAirborne = true;
      const float smooth = u * u * (3.0f - 2.0f * u);
      for (int leg = 0; leg < 2; ++leg) {
        const float targetX = leg == swing ? impl_->swingStartX +
            (impl_->swingEndX - impl_->swingStartX) * smooth : impl_->plantedX[leg];
        const float targetZ = 0.035f + (leg == swing ? 0.09f * std::sin(3.14159265f * u) : 0.0f);
        const int hip = mj_name2id(impl_->model, mjOBJ_BODY,
            leg == 0 ? "left_hip_pitch_link" : "right_hip_pitch_link");
        const float dx = targetX - impl_->data->xpos[3 * hip];
        const float dz = impl_->data->xpos[3 * hip + 2] - targetZ;
        // H1 has two 0.4 m sagittal links; positive knee angle folds
        // forward while negative hip pitch advances the ankle.
        const float knee = std::acos(std::clamp((dx * dx + dz * dz - 0.32f) / 0.32f, -0.98f, 0.98f));
        const float hipAngle = -std::atan2(dx, dz) - 0.5f * knee - pitch;
        const float ankle = -hipAngle - knee - pitch;
        if (leg == 0) { leftHipPitch = hipAngle; leftKnee = knee; leftAnkle = ankle; }
        else { rightHipPitch = hipAngle; rightKnee = knee; rightAnkle = ankle; }
      }
      telemetry.activeSwingLeg = swing;
      telemetry.gaitCycle = (swing + u) * 0.5f;
      if (u >= 1.0f && impl_->footWasAirborne && contact[swing]) {
        impl_->plantedX[swing] = impl_->data->xpos[3 * feet[swing]];
        impl_->swingFoot = support;
        impl_->swingStartX = impl_->plantedX[support];
        impl_->swingEndX = impl_->plantedX[swing] + 0.12f;
        impl_->footPhase = 0.0f;
        impl_->footWasAirborne = false;
      }
    }
    // Legs: a strong crouched pose, with opposite roll torque on each side
    // and shared pitch torque to keep the COM over the feet.
    torquePd("left_hip_yaw_joint", 0.0f, 55.0f, 5.0f);
    torquePd("right_hip_yaw_joint", 0.0f, 55.0f, 5.0f);
    // Keep both roll joints at their home width. Feeding the pelvis roll
    // correction into opposite hips was abducting the legs under load; the
    // bounded virtual IMU torque above now handles torso roll alone.
    torquePd("left_hip_roll_joint", 0.0f, 180.0f, 12.0f);
    torquePd("right_hip_roll_joint", 0.0f, 180.0f, 12.0f);
    torquePd("left_hip_pitch_joint", leftHipPitch, 240.0f, 18.0f, pitchCorrection);
    torquePd("right_hip_pitch_joint", rightHipPitch, 240.0f, 18.0f, pitchCorrection);
    torquePd("left_knee_joint", leftKnee, 300.0f, 20.0f);
    torquePd("right_knee_joint", rightKnee, 300.0f, 20.0f);
    torquePd("left_ankle_joint", leftAnkle, 70.0f, 7.0f, -0.35f * pitchCorrection);
    torquePd("right_ankle_joint", rightAnkle, 70.0f, 7.0f, -0.35f * pitchCorrection);
    torquePd("torso_joint", 0.0f, 100.0f, 10.0f);
    torquePd("left_shoulder_pitch_joint", leftShoulderPitch, 32.0f, 4.0f);
    torquePd("right_shoulder_pitch_joint", rightShoulderPitch, 32.0f, 4.0f);
    torquePd("left_shoulder_roll_joint", leftShoulderRoll, 20.0f, 3.0f);
    torquePd("right_shoulder_roll_joint", rightShoulderRoll, 20.0f, 3.0f);
    // The prior velocity penalty was tuned for a static stand and classified
    // a small, controlled pelvis translation as unsafe. Keep the 88% gate,
    // but score walking velocity against a realistic H1 assisted-step range.
    telemetry.equilibriumScore = std::clamp(1.0f - 0.75f * std::abs(roll) - 0.75f * std::abs(pitch) - speed / 1.45f, 0.0f, 1.0f);
    telemetry.walkingAllowed = telemetry.equilibriumScore >= 0.88f;
    impl_->h1EquilibriumScore = telemetry.equilibriumScore;
  }
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
  }

  for (SceneObject &object : scene.objects()) {
    const int id = mj_name2id(impl_->model, impl_->unitreeH1 ? mjOBJ_BODY : mjOBJ_GEOM, object.name.c_str());
    int fixedGeom = -1;
    if(id < 0 && impl_->officialPolicy) {
      std::string meshName=object.name;
      const auto suffix=meshName.find("_ball_hand");
      if(suffix!=std::string::npos) meshName.resize(suffix);
      const int mesh=mj_name2id(impl_->model,mjOBJ_MESH,meshName.c_str());
      for(int g=0;g<impl_->model->ngeom;++g)
        if(mesh>=0 && impl_->model->geom_type[g]==mjGEOM_MESH && impl_->model->geom_dataid[g]==mesh) { fixedGeom=g; break; }
    }
    if (id < 0 && fixedGeom < 0) continue;
    const mjtNum *position = fixedGeom>=0 ? impl_->data->geom_xpos+3*fixedGeom : impl_->unitreeH1 ? impl_->data->xpos + 3 * id : impl_->data->geom_xpos + 3 * id;
    const mjtNum *rotation = fixedGeom>=0 ? impl_->data->geom_xmat+9*fixedGeom : impl_->unitreeH1 ? impl_->data->xmat + 9 * id : impl_->data->geom_xmat + 9 * id;
    mjtNum rawPosition[3]{}, rawRotation[9]{};
    if(fixedGeom>=0) {
      // MuJoCo recenters mesh assets. Undo that asset transform when drawing
      // the original STL vertices used by Vulkan.
      const int mesh=impl_->model->geom_dataid[fixedGeom];
      mjtNum meshRotation[9]{};
      mju_quat2Mat(meshRotation,impl_->model->mesh_quat+4*mesh);
      for(int r=0;r<3;++r) for(int c=0;c<3;++c)
        for(int k=0;k<3;++k) rawRotation[3*r+c]+=rotation[3*r+k]*meshRotation[3*c+k];
      for(int r=0;r<3;++r) {
        rawPosition[r]=position[r];
        for(int k=0;k<3;++k) rawPosition[r]-=rawRotation[3*r+k]*impl_->model->mesh_pos[3*mesh+k];
      }
      position=rawPosition; rotation=rawRotation;
    }
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

void MuJoCoBridge::startJumpTest() {
  if (!impl_ || !impl_->unitreeH1) return;
  impl_->jumpPending = true;
}

void MuJoCoBridge::startNavigation() {
  if (!impl_ || !impl_->officialPolicy) return;
  impl_->navigationPath = makeNavigationPath();
  impl_->navigationWaypoint = impl_->navigationPath.size() > 1 ? 1 : 0;
}

void MuJoCoBridge::enableFullBodyMode(bool enabled) {
  if (!impl_) return;
  impl_->fullBodyMode = enabled;
}

bool MuJoCoBridge::fullBodyMode() const { return impl_ && impl_->fullBodyMode; }
void MuJoCoBridge::setFullBodyGoal(int goal) { if (impl_) impl_->fullBodyGoal=std::clamp(goal,0,3); }
int MuJoCoBridge::fullBodyGoal() const { return impl_ ? impl_->fullBodyGoal : 0; }

void MuJoCoBridge::setRobotScript(int script) {
  if (!impl_->ready) return;
  // Walking a biped needs an explicit COM-over-foot controller. Until that
  // exists, keep the two-legged configuration in its verified standing pose.
  impl_->script = impl_->biped ? 0 : impl_->unitreeH1 ? std::clamp(script, 0, 5) : std::clamp(script, 0, 3);
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
