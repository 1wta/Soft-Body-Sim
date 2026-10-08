#pragma once
#include "MathUtils.h"
#include <vector>

namespace sim{
  using Math::vec2;
  
  double extraDampening = 0.999;
  vec2 gravity = vec2(0.0, 0.0);
  // inline const vec2 gravity = vec2(0.0, 98.1);

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
    double kc = 10000.0, dc = 0.99; // for compression
    double ke = 1500.0, de = 0.9; // for expansion
    double l0, l;

    Spring() {}
    Spring(int _iidx, int _eidx, Body* _shape);

    void update();
    vec2 rebound(); // gives rebound force against initial point
  };

  struct Derivative{
    vec2 v, a;
  };

  struct Body{
  public:
    int N;
    double nRT = 670000.0;
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

    std::vector<Derivative> getDerivative () {
      double A = Area();
      
      if (A < Math::eps) A = Math::eps;
      double P = nRT / A;

      std::vector<vec2> F(N);

      for (int idx = 0; idx < N; idx++) {
        int oth = (idx + 1) % N;

        springs[idx].update();
        vec2 Frebound = springs[idx].rebound();
        F[idx] = F[idx] + Frebound;
        F[oth] = F[oth] - Frebound;

        vec2 edge = body[oth].pos - body[idx].pos;
        vec2 Normal = vec2(edge.y, -edge.x);

        vec2 Fp = Normal * P;
        F[idx] = F[idx] + Fp * 0.5;
        F[oth] = F[oth] + Fp * 0.5;

        F[idx] = F[idx] + gravity * body[idx].mass;
      }

      std::vector<Derivative> dYdt(N);
      for (int idx = 0; idx < N; idx++) {
        dYdt[idx].a = F[idx] / body[idx].mass;
        dYdt[idx].v = body[idx].vel;
      }

      return dYdt;
    }

    void update (double dt) { // RK4
      std::vector<Particle> cur = body;
      
      std::vector<Derivative> k1 = getDerivative();
      for (int idx = 0; idx < N; idx++) {
        body[idx].pos = cur[idx].pos + k1[idx].v * dt * 0.5;
        body[idx].vel = cur[idx].vel + k1[idx].a * dt * 0.5;
      }

      std::vector<Derivative> k2 = getDerivative();

      for (int idx = 0; idx < N; idx++) {
        body[idx].pos = cur[idx].pos + k2[idx].v * dt * 0.5;
        body[idx].vel = cur[idx].vel + k2[idx].a * dt * 0.5;
      }

      std::vector<Derivative> k3 = getDerivative();

      for (int idx = 0; idx < N; idx++) {
        body[idx].pos = cur[idx].pos + k3[idx].v * dt;
        body[idx].vel = cur[idx].vel + k3[idx].a * dt;
      }

      std::vector<Derivative> k4 = getDerivative();

      body = cur;
      for (int idx = 0; idx < N; idx++) {
        vec2 avgAcc = (k1[idx].a + 2 * k2[idx].a + 2 * k3[idx].a + k4[idx].a) / 6.0;
        vec2 avgVel = (k1[idx].v + 2 * k2[idx].v + 2 * k3[idx].v + k4[idx].v) / 6.0;

        body[idx].pos = body[idx].pos + avgVel * dt;
        body[idx].vel = body[idx].vel + avgAcc * dt;

        body[idx].vel = body[idx].vel * extraDampening; // extra dampening
      }
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

  inline vec2 Spring::rebound() {
    vec2 spos = shape -> body[iidx].pos;
    vec2 epos = shape -> body[eidx].pos;

    vec2 svel = shape -> body[iidx].vel;
    vec2 evel = shape -> body[eidx].vel;

    vec2 dirn = (epos - spos).normalize();
    vec2 vrel = (evel - svel);
    double x = l - l0;

    double k = (x > 0) ? ke : kc;
    double d = (x > 0) ? de : dc;
  
    double Fspring = k * x;
    double Fdamp = d * dirn.dot(vrel);

    return (Fspring + Fdamp) * dirn;
  }
};
