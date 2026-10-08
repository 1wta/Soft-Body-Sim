#include <SDL2/SDL.h>
#include <utility>
#include <vector>
#include <cmath>
#include <array>
#include <iostream>

#include "MathUtils.h"
#include "SimUtils.h"

using namespace Math;
using namespace sim;

int Nsb = 50;
double Rsb = 6.0;
std::array<int, 2> wdim{1200, 800};

void DrawPolygon (SDL_Renderer* renderer, const std::vector<Particle>& point) {
  int Np = static_cast<int>(point.size());
  if (Np < 2) return;
  for (int idx = 0; idx < Np; idx++) {
    std::array<float, 2> u = point[idx].pos.floatify(), v = point[(idx + 1) % Np].pos.floatify();
    SDL_SetRenderDrawColor(renderer, 255, 100, 100, 255);
    SDL_RenderDrawLine(renderer, u[0], u[1], v[0], v[1]);
  }
}

int main(int argc, char* argv[]) {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "SDL Init Error: " << SDL_GetError() << std::endl;
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow(
    "SDL2 Wayland Window",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    wdim[0], wdim[1],
    SDL_WINDOW_SHOWN
  );

  if (!window) {
    std::cerr << "Window Error: " << SDL_GetError() << std::endl;
    SDL_Quit(); return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  vec2 center = vec2(wdim) / 2;

  std::vector<Particle> shape;
  for (int idx = 0; idx < Nsb; idx++) {
    double theta = ((2.0 * M_PI) / Nsb) * idx;
    shape.push_back(Particle(Rsb * vec2(cos(theta), sin(theta)) + center/2));
  }

  Body softBody(shape);

  bool running = true;
  SDL_Event event;

  Uint64 Frequency = SDL_GetPerformanceFrequency();
  Uint64 prevTime = SDL_GetPerformanceCounter();

  double dt = 0.0;

  auto clamp = [&](Particle& p){
    std::array<double, 2> e{0.8, 0.9};
    if (p.pos.x < 0) {
      p.pos.x = 0; p.vel.x = e[0] * std::abs(p.vel.x);
    } else if (wdim[0] < p.pos.x) {
      p.pos.x = wdim[0]; p.vel.x = - e[0] * std::abs(p.vel.x);
    }

    if (p.pos.y < 0) {
      p.pos.y = 0; p.vel.y = e[1] * std::abs(p.vel.y);
    } else if (wdim[1] < p.pos.y) {
      p.pos.y = wdim[1]; p.vel.y = - e[1] * std::abs(p.vel.y);
    }
  };

  while (running) {
    Uint64 curTime = SDL_GetPerformanceCounter();
    dt = static_cast<double>(curTime - prevTime) / static_cast<double>(Frequency);
    prevTime = curTime;

    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) running = false;
      if (event.type == SDL_MOUSEBUTTONDOWN) gravity = vec2(0.0, 981.0), extraDampening = 1;
    }

    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); 
    SDL_RenderClear(renderer); // screen.fill()

    softBody.update(dt);
    for (auto& p : softBody.body) clamp(p);

    DrawPolygon(renderer, softBody.body);

    SDL_RenderPresent(renderer); // pygame.display.update()
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
