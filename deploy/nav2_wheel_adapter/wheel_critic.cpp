#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>

namespace mppi::critics {
// Penalize each physical wheel's deadband, allowing both STOP and pure spin.
// No trajectory generation, motor authority or per-rollout allocations here.
class R2B4WheelCritic : public CriticFunction {
 public:
  void initialize() override {
    auto get=parameters_handler_->getParamGetter(name_);
    get(weight_,"cost_weight",10.0f);
    auto physical=parameters_handler_->getParamGetter(parent_name_);
    physical(track_,"track_width_m",0.3557f);
    physical(minimum_,"wheel_minimum_mps",0.15f);
  }
  void score(CriticData &data) override {
    if (!enabled_ || minimum_<=0.) return;
    const auto rows=data.state.vx.shape()[0], cols=data.state.vx.shape()[1];
    auto penalty=[this](float speed) {
      const float u=std::abs(speed);
      return u>=minimum_?0.f:2.f*std::min(u,minimum_-u)/minimum_;
    };
    for (size_t i=0;i<rows;++i) {
      float cost=0.f;
      for (size_t j=0;j<cols;++j) {
        const float v=data.state.vx(i,j), turn=data.state.wz(i,j)*track_*.5f;
        cost+=.5f*(penalty(v-turn)+penalty(v+turn));
      }
      data.costs(i)+=weight_*cost/std::max<size_t>(1,cols);
    }
  }
 private:
  float weight_=10.f,track_=.3557f,minimum_=.15f;
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::R2B4WheelCritic,mppi::critics::CriticFunction)
