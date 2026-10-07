#pragma once
#include "MathUtils.h"
#include <vector>

namespace sim{
  using Math::vec2;
  
  struct Body;

  struct Particle{
    public:
      double mass = 1.0;
      vec2 pos, vel;

      Particle() {}
      Particle (vec2 Pos) : pos(Pos) {}
      Particle (vec2 Pos, vec2 Vel) : pos(Pos), vel(Vel) {}
      Particle (vec2 Pos, vec2 Vel, double Mass) : pos(Pos), vel(Vel), mass(Mass) {}
  };
  
  struct Spring{
  public:
    Body* shape;
    int iidx, eidx;
    double kc = 100.0, dc = 0.9; // for compression
    double ke = 15.0, de = 0.2; // for expansion
    double l0, l;

    Spring() {}
    Spring(int _iidx, int _eidx, Body* _shape);

    void update();
    vec2 rebound(); // gives rebound force against initial point
  };

  struct Body{
  public:
    int N;
    double nRT = 1000.0;
    std::vector<Particle> body;
    std::vector<Spring> springs;

    Body (std::vector<Particle> _body) : body(_body) {
      N = static_cast<int>(body.size());

      for (int idx = 0; idx < N; idx++) {
        springs.push_back(Spring(idx, (idx + 1) % N, this));
      }
    };

    double Area() {
      double signedArea = 0.0;
      for (int idx = 0; idx < N; idx++) {
        signedArea += body[idx].pos.cross(body[(idx + 1) % N].pos);
      } return std::abs(signedArea) / 2.0;
    }

    void update (int dt) {
      return;
    }
  };

  inline Spring::Spring (int _iidx, int _eidx, Body* _shape) : iidx(_iidx), eidx(_eidx), shape(_shape) {
    vec2 spos = shape -> body[iidx].pos;
    vec2 epos = shape -> body[eidx].pos;
    l0 = l = (epos - spos).magnitude();
  }

  inline void Spring::update() {
    vec2 spos = shape -> body[iidx].pos;
    vec2 epos = shape -> body[eidx].pos;
    l = (epos - spos).magnitude();
  }
};
