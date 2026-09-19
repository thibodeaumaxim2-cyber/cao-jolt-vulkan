#pragma once
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// Slow level-ground gait. Sole Cartesian acceleration and pelvis feedback are
// converted to motor torque by inverse dynamics. All contact forces are real
// MuJoCo contacts: the computed support wrench is never applied to the base.
class ValkyrieWalk {
  using V3=std::array<mjtNum,3>;
  enum Phase { Crouch, Transfer, Swing, Stop, Hold } phase=Crouch;
  std::array<V3,2> feet{}, fs{};
  V3 p{}, begin{}, end{}, destination{};
  std::array<int,2> sites{}, bodies{};
  std::array<std::array<int,6>,2> legDofs{};
  std::vector<mjtNum> home, jac, oldJac, acceleration, rhs;
  std::array<mjtNum,12> weights{}, initialWeights{};
  mjtNum height=0, elapsed=0;
  int side=0;
  bool hasOld=false;

  static void solve(mjtNum* a, mjtNum* b, mjtNum* x) {
    for(int k=0;k<6;++k) {
      int pivot=k;
      for(int i=k+1;i<6;++i) if(std::abs(a[6*i+k])>std::abs(a[6*pivot+k])) pivot=i;
      if(std::abs(a[6*pivot+k])<1e-12) throw std::runtime_error("Singular Valkyrie gait solve");
      for(int j=k;j<6;++j) std::swap(a[6*k+j],a[6*pivot+j]);
      std::swap(b[k],b[pivot]);
      for(int i=k+1;i<6;++i) {
        const mjtNum scale=a[6*i+k]/a[6*k+k];
        for(int j=k;j<6;++j) a[6*i+j]-=scale*a[6*k+j];
        b[i]-=scale*b[k];
      }
    }
    for(int i=5;i>=0;--i) {
      x[i]=b[i];
      for(int j=i+1;j<6;++j) x[i]-=a[6*i+j]*x[j];
      x[i]/=a[6*i+i];
    }
  }
  void transition(Phase next) {
    phase=next; elapsed=0; begin=p; initialWeights=weights;
    const V3 center{(feet[0][0]+feet[1][0])*.5,(feet[0][1]+feet[1][1])*.5,0};
    end={center[0]-.04,center[1],height};
    if(next==Transfer) end={feet[1-side][0]-.04,center[1]+.8*(feet[1-side][1]-center[1]),height};
    if(next==Swing) { destination=feet[side]; destination[0]=feet[1-side][0]+.08; }
  }
public:
  bool initialized=false;
  int steps=0, activeSwing=-1;
  float cycle() const { return static_cast<float>(elapsed/(phase==Swing ? 1.4 : 2.0)); }
  void reset(const mjModel* m, mjData* d, const std::vector<mjtNum>& nominal) {
    mj_forward(m,d); home=nominal;
    sites={mj_name2id(m,mjOBJ_SITE,"left_sole"),mj_name2id(m,mjOBJ_SITE,"right_sole")};
    bodies={mj_name2id(m,mjOBJ_BODY,"leftFoot"),mj_name2id(m,mjOBJ_BODY,"rightFoot")};
    const std::array<const char*,6> names{"HipYaw","HipRoll","HipPitch","KneePitch","AnklePitch","AnkleRoll"};
    for(int i=0;i<2;++i) {
      for(int a=0;a<3;++a) feet[i][a]=d->site_xpos[3*sites[i]+a];
      feet[i][2]=0;
      for(int a=0;a<6;++a) {
        const auto name=std::string(i==0 ? "left" : "right")+names[a];
        const int j=mj_name2id(m,mjOBJ_JOINT,name.c_str());
        if(j<0) throw std::runtime_error("Missing Valkyrie leg joint");
        legDofs[i][a]=m->jnt_dofadr[j];
      }
    }
    fs=feet; for(int a=0;a<3;++a) p[a]=d->qpos[a];
    height=p[2]-.035; begin=p; end=p; end[2]=height;
    weights.fill(1); initialWeights=weights;
    jac.resize(12*m->nv); oldJac.resize(12*m->nv);
    acceleration.resize(m->nv); rhs.resize(m->nv);
    phase=Crouch; elapsed=0; side=0; steps=0; activeSwing=-1; hasOld=false; initialized=true;
  }
  void control(const mjModel* m, mjData* d, bool walking) {
    mj_forward(m,d);
    mjtNum forces[2]{};
    for(int c=0;c<d->ncon;++c) {
      const int a=m->geom_bodyid[d->contact[c].geom[0]], b=m->geom_bodyid[d->contact[c].geom[1]];
      if(a!=0 && b!=0) continue;
      mjtNum force[6]{}; mj_contactForce(m,d,c,force);
      for(int i=0;i<2;++i) if(a==bodies[i] || b==bodies[i]) forces[i]+=force[0];
    }
    elapsed+=m->opt.timestep;
    const mjtNum u=std::min(mjtNum(1),elapsed/(phase==Swing ? 1.4 : 2.0));
    const mjtNum smooth=u*u*(3-2*u);
    activeSwing=phase==Swing ? side : -1;
    if(phase==Crouch || phase==Transfer || phase==Stop) {
      for(int a=0;a<3;++a) p[a]=begin[a]+(end[a]-begin[a])*smooth;
      for(int a=0;a<12;++a) {
        const mjtNum final=phase==Transfer && a/6==side ? .001 : 1;
        weights[a]=initialWeights[a]+(final-initialWeights[a])*smooth;
      }
      if(u>=1) {
        if(phase==Transfer && walking && forces[1-side]>100) transition(Swing);
        else if(phase!=Transfer && walking) transition(Transfer);
        else if(!walking) transition(phase==Stop ? Hold : Stop);
      }
    } else if(phase==Swing) {
      for(int a=0;a<3;++a) fs[side][a]=feet[side][a]+(destination[a]-feet[side][a])*smooth;
      fs[side][2]+=.035*std::pow(std::sin(3.141592653589793*u),2);
      if(u>=1 && forces[side]>1) {
        feet[side]=destination; fs=feet; ++steps; side=1-side;
        transition(walking ? Transfer : Stop);
      }
    } else if(walking) transition(Transfer);

    for(int i=0;i<2;++i) mj_jacSite(m,d,jac.data()+6*i*m->nv,jac.data()+(6*i+3)*m->nv,sites[i]);
    std::fill(acceleration.begin(),acceleration.end(),0);
    for(int i=0;i<m->nu;++i) {
      const int j=m->actuator_trnid[2*i],v=m->jnt_dofadr[j];
      acceleration[v]=100*(home[i]-d->qpos[m->jnt_qposadr[j]])-20*d->qvel[v];
    }
    for(int a=0;a<3;++a) {
      acceleration[a]=100*(p[a]-d->qpos[a])-20*d->qvel[a];
      acceleration[3+a]=-200*d->qpos[4+a]-20*d->qvel[3+a];
    }
    for(int i=0;i<2;++i) {
      const auto* R=d->site_xmat+9*sites[i];
      mjtNum error[6]{fs[i][0]-d->site_xpos[3*sites[i]],fs[i][1]-d->site_xpos[3*sites[i]+1],
        fs[i][2]-d->site_xpos[3*sites[i]+2],.5*(R[5]-R[7]),.5*(R[6]-R[2]),.5*(R[1]-R[3])};
      mjtNum matrix[36]{}, desired[6]{}, solution[6]{};
      for(int r=0;r<6;++r) {
        const int offset=(6*i+r)*m->nv;
        desired[r]=200*error[r];
        for(int v=0;v<m->nv;++v) {
          desired[r]-=30*jac[offset+v]*d->qvel[v];
          if(hasOld) desired[r]-=(jac[offset+v]-oldJac[offset+v])*d->qvel[v]/m->opt.timestep;
        }
        for(int a=0;a<6;++a) {
          desired[r]-=jac[offset+a]*acceleration[a];
          matrix[6*r+a]=jac[offset+legDofs[i][a]]+(r==a ? 1e-8 : 0);
        }
      }
      solve(matrix,desired,solution);
      for(int a=0;a<6;++a) acceleration[legDofs[i][a]]=solution[a];
    }
    oldJac=jac; hasOld=true;
    mj_mulM(m,d,rhs.data(),acceleration.data());
    for(int v=0;v<m->nv;++v) rhs[v]+=d->qfrc_bias[v];
    mjtNum gram[36]{}, dual[6]{}, load[6]{}, wrench[12]{};
    for(int a=0;a<6;++a) {
      load[a]=rhs[a];
      for(int b=0;b<6;++b) for(int r=0;r<12;++r)
        gram[6*a+b]+=jac[r*m->nv+a]*weights[r]*jac[r*m->nv+b];
    }
    solve(gram,load,dual);
    for(int r=0;r<12;++r) for(int a=0;a<6;++a) wrench[r]+=weights[r]*jac[r*m->nv+a]*dual[a];
    for(int i=0;i<m->nu;++i) {
      const int v=m->jnt_dofadr[m->actuator_trnid[2*i]];
      mjtNum torque=rhs[v];
      for(int r=0;r<12;++r) torque-=jac[r*m->nv+v]*wrench[r];
      d->ctrl[i]=std::clamp(torque,m->actuator_ctrlrange[2*i],m->actuator_ctrlrange[2*i+1]);
    }
    mju_zero(d->qfrc_applied,m->nv); mju_zero(d->xfrc_applied,6*m->nbody);
  }
};
