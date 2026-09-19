#pragma once
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <string>
#include <stdexcept>
#include <vector>

// Counterpart of python/envs/valkyrie_balance.py. Estimated sole wrenches
// affect motor feedforward only; the free base receives no applied forces.
class ValkyrieBalance {
  std::vector<mjtNum> home, kp, kd, jac;
  std::array<int, 2> feet{};
  int root = 0, base = 0;
public:
  void reset(mjModel* m, mjData* d) {
    m->opt.timestep = .001;
    home.assign(m->nu, 0); kp.assign(m->nu, 80); kd.assign(m->nu, 8);
    jac.resize(12*m->nv);
    const int joint = mj_name2id(m, mjOBJ_JOINT, "valkyrie_root");
    if (joint < 0) throw std::runtime_error("Missing Valkyrie root");
    root = m->jnt_qposadr[joint]; base = m->jnt_dofadr[joint];
    feet = {mj_name2id(m, mjOBJ_SITE, "left_sole"), mj_name2id(m, mjOBJ_SITE, "right_sole")};
    if (feet[0] < 0 || feet[1] < 0) throw std::runtime_error("Missing Valkyrie soles");
    for (int i=0; i<m->nu; ++i) {
      const std::string n = mj_id2name(m, mjOBJ_ACTUATOR, i);
      const auto has = [&](const char* s) { return n.find(s) != std::string::npos; };
      if (has("Hip") || has("Knee")) kp[i]=800, kd[i]=50;
      if (has("Ankle")) kp[i]=600, kd[i]=35;
      if (has("torso")) kp[i]=500, kd[i]=35;
      if (has("Finger") || has("Thumb") || has("Pinky")) kp[i]=5, kd[i]=.5;
      if (has("HipPitch") || has("AnklePitch")) home[i]=-.2;
      if (has("KneePitch")) home[i]=.4;
      const int j=m->actuator_trnid[2*i];
      home[i]=std::clamp(home[i], m->jnt_range[2*j], m->jnt_range[2*j+1]);
      d->qpos[m->jnt_qposadr[j]]=home[i];
    }
    mj_forward(m,d);
    // Initialization only. Runtime control never edits root position/velocity.
    d->qpos[root+2]-=std::min(d->site_xpos[3*feet[0]+2],d->site_xpos[3*feet[1]+2]);
    mj_forward(m,d);
  }
  void control(const mjModel* m, mjData* d) {
    mj_forward(m,d);
    for (int i=0;i<2;++i)
      mj_jacSite(m,d,jac.data()+6*i*m->nv,jac.data()+(6*i+3)*m->nv,feet[i]);
    mjtNum gram[36]{}, dual[6]{}, wrench[12]{};
    for (int a=0;a<6;++a) for(int b=0;b<6;++b)
      for(int r=0;r<12;++r) gram[6*a+b]+=jac[r*m->nv+base+a]*jac[r*m->nv+base+b];
    if(mju_cholFactor(gram,6,1e-12)!=6) throw std::runtime_error("Singular Valkyrie support geometry");
    mju_cholSolve(dual,gram,d->qfrc_bias+base,6);
    for(int r=0;r<12;++r) for(int a=0;a<6;++a) wrench[r]+=jac[r*m->nv+base+a]*dual[a];
    for(int i=0;i<m->nu;++i) {
      const int j=m->actuator_trnid[2*i], v=m->jnt_dofadr[j], q=m->jnt_qposadr[j];
      mjtNum ff=d->qfrc_bias[v];
      for(int r=0;r<12;++r) ff-=jac[r*m->nv+v]*wrench[r];
      d->ctrl[i]=std::clamp(kp[i]*(home[i]-d->qpos[q])-kd[i]*d->qvel[v]+ff,
                            m->actuator_ctrlrange[2*i],m->actuator_ctrlrange[2*i+1]);
    }
    mju_zero(d->qfrc_applied,m->nv);
    mju_zero(d->xfrc_applied,6*m->nbody);
  }
};
