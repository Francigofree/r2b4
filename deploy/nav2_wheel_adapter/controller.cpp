#include "wheel_projection.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace r2b4_nav2 {
struct CollisionEvidence {
  const char *reason = "clear";
  int steps = 0, sample = -1, cell_x = -1, cell_y = -1, cost = -1;
  double x = 0., y = 0., yaw = 0.;
  double origin_x = 0., origin_y = 0., resolution = 0.;
  double sampling_guard = 0.;
};

// Check the complete configured convex polygon against complete occupied cells.
// The generated polygon already includes the physical Euclidean body margin.
// Inflation is a preference; lethal, unknown and map exit are hard constraints.
CollisionEvidence commandEvidence(nav2_costmap_2d::Costmap2D *map,
    const nav2_costmap_2d::Footprint &footprint, double x, double y, double yaw,
    double v, double omega) {
  CollisionEvidence e;
  e.origin_x=map->getOriginX(); e.origin_y=map->getOriginY(); e.resolution=map->getResolution();
  const double res=map->getResolution(), r=res*.5;
  double radius=0.;
  if (footprint.size()<3 || footprint.size()>32 || !std::isfinite(res) || res<=0. ||
      !std::isfinite(x+y+yaw+v+omega)) {e.reason="invalid_geometry"; return e;}
  struct Axis {double x,y,low,high;};
  std::vector<Axis> axes;
  for (const auto &p : footprint) {
    if (!std::isfinite(p.x+p.y)) {e.reason="invalid_geometry"; return e;}
    radius=std::max(radius,std::hypot(p.x,p.y));
  }
  const double travel=std::abs(v)+radius*std::abs(omega); // unchanged one-second horizon
  if (travel/(res*.25)>128.) {e.reason="trajectory_bound_exceeded"; return e;}
  e.steps=std::max(1,static_cast<int>(std::ceil(travel/(res*.25))));
  if (e.steps>128) {e.reason="trajectory_bound_exceeded"; return e;}
  e.sampling_guard=travel/(2*e.steps);
  for (size_t i=0;i<footprint.size();++i) {
    const auto &p=footprint[i], &q=footprint[(i+1)%footprint.size()];
    double nx=q.y-p.y,ny=p.x-q.x, length=std::hypot(nx,ny);
    if (length<1e-12) {e.reason="invalid_geometry"; return e;}
    nx/=length; ny/=length;
    double low=1e100,high=-1e100;
    for (const auto &vertex:footprint) {
      const double d=nx*vertex.x+ny*vertex.y;
      low=std::min(low,d); high=std::max(high,d);
    }
    // CCW convex footprint only; fail closed on an invalid configuration.
    if (high-(nx*p.x+ny*p.y)>1e-9) {e.reason="invalid_geometry"; return e;}
    axes.push_back({nx,ny,low-e.sampling_guard,high+e.sampling_guard});
  }
  nav2_costmap_2d::Footprint guarded(footprint.size());
  for (size_t i=0;i<axes.size();++i) {
    const auto &a=axes[(i+axes.size()-1)%axes.size()], &b=axes[i];
    const double det=a.x*b.y-a.y*b.x;
    if (det<1e-9) {e.reason="invalid_geometry"; return e;}
    guarded[i].x=(a.high*b.y-a.y*b.high)/det;
    guarded[i].y=(a.x*b.high-a.high*b.x)/det;
  }
  for (auto &a:axes) {
    a.low=1e100; a.high=-1e100;
    for (const auto &p:guarded) {
      const double d=a.x*p.x+a.y*p.y;
      a.low=std::min(a.low,d); a.high=std::max(a.high,d);
    }
  }
  for (int i=0; i<=e.steps; ++i) {
    const double t=static_cast<double>(i)/e.steps, angle=yaw+omega*t;
    const double px=x+(std::abs(omega)<1e-9 ? v*t*std::cos(yaw)
        : v/omega*(std::sin(angle)-std::sin(yaw)));
    const double py=y+(std::abs(omega)<1e-9 ? v*t*std::sin(yaw)
        : v/omega*(std::cos(yaw)-std::cos(angle)));
    const double c=std::cos(angle), s=std::sin(angle);
    double xmin=1e100,xmax=-1e100,ymin=1e100,ymax=-1e100;
    for (const auto &p:guarded) {
      const double wx=px+c*p.x-s*p.y, wy=py+s*p.x+c*p.y;
      xmin=std::min(xmin,wx); xmax=std::max(xmax,wx);
      ymin=std::min(ymin,wy); ymax=std::max(ymax,wy);
    }
    const int minx=static_cast<int>(std::floor((xmin-map->getOriginX()-1e-10)/res));
    const int maxx=static_cast<int>(std::floor((xmax-map->getOriginX()+1e-10)/res));
    const int miny=static_cast<int>(std::floor((ymin-map->getOriginY()-1e-10)/res));
    const int maxy=static_cast<int>(std::floor((ymax-map->getOriginY()+1e-10)/res));
    e.sample=i; e.x=px; e.y=py; e.yaw=angle;
    if (minx<0 || miny<0 || maxx>=static_cast<int>(map->getSizeInCellsX()) ||
        maxy>=static_cast<int>(map->getSizeInCellsY())) {
      e.reason="out_of_map"; e.cell_x=minx<0?minx:maxx; e.cell_y=miny<0?miny:maxy; return e;
    }
    for (int my=miny; my<=maxy; ++my) for (int mx=minx; mx<=maxx; ++mx) {
      const int cost=map->getCost(mx,my);
      if (cost<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      double wx,wy; map->mapToWorld(mx,my,wx,wy);
      const double dx=wx-px,dy=wy-py;
      if (wx+r<xmin-1e-10 || wx-r>xmax+1e-10 ||
          wy+r<ymin-1e-10 || wy-r>ymax+1e-10) continue;
      bool separated=false;
      for (const auto &a:axes) {
        const double nx=c*a.x-s*a.y, ny=s*a.x+c*a.y;
        const double d=nx*dx+ny*dy, extent=r*(std::abs(nx)+std::abs(ny));
        if (d<a.low-extent-1e-10 || d>a.high+extent+1e-10) {separated=true; break;}
      }
      if (separated) continue;
      e.reason=cost==nav2_costmap_2d::NO_INFORMATION?"unknown_cell":"lethal_cell";
      e.cell_x=mx; e.cell_y=my; e.cost=cost; return e;
    }
  }
  e.sample=-1; return e;
}
bool commandCollides(nav2_costmap_2d::Costmap2D *map,
    const nav2_costmap_2d::Footprint &footprint, double x, double y, double yaw,
    double v, double omega) {
  return std::string(commandEvidence(map,footprint,x,y,yaw,v,omega).reason)!="clear";
}

struct CommandProjection {
  std::pair<double,double> command;
  CollisionEvidence collision;
  bool collision_projected = false;
};

CommandProjection collisionFreeCommand(nav2_costmap_2d::Costmap2D *map,
    const nav2_costmap_2d::Footprint &footprint, double x, double y, double yaw,
    const WheelLimits &limits, double v, double omega, std::pair<double,double> initial) {
  CommandProjection result{initial, {}};
  auto check = [&](std::pair<double,double> command) {
    return commandEvidence(map,footprint,x,y,yaw,command.first,command.second);
  };
  result.collision=check(result.command);
  if (std::string(result.collision.reason)=="clear") return result;
  // A one-second constant command can collide even when MPPI's time-varying
  // rollout is free. Try bounded, slower realizable commands and the existing
  // wheel families before aborting the entire navigation action. Every option
  // retains the same full horizon, padded footprint and hard costmap checks.
  std::vector<std::pair<double,double>> candidates;
  candidates.push_back(limits.project(v,omega));
  for (int branch=0;branch<4;++branch) candidates.push_back(limits.project(v,omega,branch));
  for (double scale : {.75,.5,.25}) candidates.push_back(limits.projectMppi(v*scale,omega*scale));
  auto distance = [&](std::pair<double,double> c) {
    return std::pow(c.first-v,2)+std::pow((c.second-omega)*limits.track*.5,2);
  };
  std::sort(candidates.begin(),candidates.end(),[&](auto a,auto b) {
    return distance(a)==distance(b)?a<b:distance(a)<distance(b);
  });
  candidates.erase(std::unique(candidates.begin(),candidates.end()),candidates.end());
  for (auto candidate : candidates) {
    if (candidate==result.command || (candidate.first==0. && candidate.second==0.)) continue;
    auto collision=check(candidate);
    if (std::string(collision.reason)=="clear") return {candidate,collision,true};
  }
  return result;
}

class WheelAwareMPPIController : public nav2_mppi_controller::MPPIController {
  struct Timing {
    uint64_t count=0, wall_max_ns=0, wall_max_cpu_ns=0, cpu_max_ns=0, wall_sum_ns=0, cpu_sum_ns=0;
  };
  static uint64_t wallNow() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static uint64_t cpuNow() {
    timespec value{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID,&value);
    return static_cast<uint64_t>(value.tv_sec)*1000000000ULL+value.tv_nsec;
  }
  // Scalar observation only: distinguish actual compute from waits/preemption.
  struct Phase {
    WheelAwareMPPIController &owner; size_t index;
    uint64_t wall=wallNow(), cpu=cpuNow();
    ~Phase() {
      const auto elapsed=wallNow()-wall, used=cpuNow()-cpu;
      std::lock_guard<std::mutex> lock(owner.timing_mutex_);
      auto &t=owner.timings_[index];
      ++t.count; t.wall_sum_ns+=elapsed; t.cpu_sum_ns+=used;
      if (elapsed>t.wall_max_ns) {t.wall_max_ns=elapsed; t.wall_max_cpu_ns=used;}
      t.cpu_max_ns=std::max(t.cpu_max_ns,used);
    }
  };
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    MPPIController::configure(parent,name,tf,costmap);
    const auto node=parent.lock();
    auto value=[&](const char *key) {
      const auto parameter=name+"."+key;
      nav2_util::declare_parameter_if_not_declared(node,parameter,rclcpp::ParameterValue(0.));
      const double result=node->get_parameter(parameter).as_double();
      if (!std::isfinite(result) || result<=0.)
        throw std::runtime_error("Missing/invalid wheel adapter parameter: "+parameter);
      return result;
    };
    limits_={value("track_width_m"),value("wheel_minimum_mps"),value("wheel_maximum_mps"),
        value("platform_max_v_mps"),value("platform_max_omega_rad_s"),value("max_curvature_rad_per_m")};
    if (limits_.minimum>limits_.maximum) throw std::runtime_error("Wheel minimum exceeds maximum");
    auto getParam=parameters_handler_->getParamGetter(name);
    getParam(mission_max_v_,"mission_max_v_mps",limits_.max_v);
    getParam(mission_max_omega_,"mission_max_omega_rad_s",limits_.max_omega);
    raw_publisher_=node->create_publisher<geometry_msgs::msg::TwistStamped>("/r2b4/nav2/raw_cmd_vel",1);
    evidence_publisher_=node->create_publisher<std_msgs::msg::String>("/r2b4/nav2/motion_evidence",4);
    evidence_timer_=node->create_wall_timer(std::chrono::milliseconds(200),[this] {publishEvidence();});
  }
  void activate() override {
    MPPIController::activate(); raw_publisher_->on_activate(); evidence_publisher_->on_activate();
  }
  void deactivate() override {
    publishEvidence(); raw_publisher_->on_deactivate(); evidence_publisher_->on_deactivate(); MPPIController::deactivate();
  }
  void cleanup() override {
    evidence_timer_.reset(); evidence_publisher_.reset(); raw_publisher_.reset(); MPPIController::cleanup();
  }
  void setSpeedLimit(const double &speed_limit,const bool &percentage) override {
    std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
    const double nominal=parent_.lock()->get_parameter(name_+".vx_max").as_double();
    double ratio=0.;
    if (std::isfinite(speed_limit) && nominal>0.)
      ratio=speed_limit==nav2_costmap_2d::NO_SPEED_LIMIT?100.:
        (percentage?std::clamp(speed_limit,0.,100.):std::clamp(speed_limit/nominal,0.,1.)*100.);
    optimizer_.setSpeedLimit(ratio,true);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *goal_checker) override {
    Phase total{*this,0};
    measured_v_=speed.linear.x; measured_w_=speed.angular.z;
    geometry_msgs::msg::TwistStamped command;
    try { Phase phase{*this,1}; command=MPPIController::computeVelocityCommands(pose,speed,goal_checker); }
    catch (const std::exception &) {record(pose,0.,0.,0.,0.,"mppi_exception",{},false); throw;}
    { Phase phase{*this,2}; raw_publisher_->publish(command); }
    const double raw_v=command.twist.linear.x,raw_w=command.twist.angular.z;
    auto limits=limits_;
    const auto node=parent_.lock();
    const double max_v=node->get_parameter(name_+".mission_max_v_mps").as_double();
    const double max_w=node->get_parameter(name_+".mission_max_omega_rad_s").as_double();
    if (!std::isfinite(max_v) || !std::isfinite(max_w) || max_v<=0. || max_w<=0.) {
      record(pose,raw_v,raw_w,0.,0.,"invalid_mission_envelope",{});
      throw nav2_core::NoValidControl("Invalid mission envelope");
    }
    limits.max_v=std::min(limits.max_v,max_v); limits.max_omega=std::min(limits.max_omega,max_w);
    std::pair<double,double> initial;
    { Phase phase{*this,3}; initial=limits.projectMppi(raw_v,raw_w); }
    auto *map=costmap_ros_->getCostmap();
    const auto &q=pose.pose.orientation;
    const double yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
    CommandProjection projected;
    {
      Phase phase{*this,4};
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      projected=collisionFreeCommand(map,costmap_ros_->getRobotFootprint(),pose.pose.position.x,
          pose.pose.position.y,yaw,limits,raw_v,raw_w,initial);
    }
    const auto feasible=projected.command;
    const auto collision=projected.collision;
    command.twist.linear.x=feasible.first; command.twist.angular.z=feasible.second;
    if (std::string(collision.reason)!="clear") {
      record(pose,raw_v,raw_w,feasible.first,feasible.second,collision.reason,collision);
      throw nav2_core::NoValidControl(std::string("R2B4 projected trajectory: ")+collision.reason);
    }
    const bool changed=std::abs(raw_v-feasible.first)>1e-8 || std::abs(raw_w-feasible.second)>1e-8;
    if (changed) record(pose,raw_v,raw_w,feasible.first,feasible.second,
        projected.collision_projected?"collision_velocity_projection":
        feasible.first==0. && feasible.second==0.?"no_nonzero_wheel_feasible_command":"wheel_projection",collision);
    return command;
  }
 private:
  struct Event {
    uint64_t sequence=0; int64_t stamp=0,pose_stamp=0;
    double x=0.,y=0.,yaw=0.,v=0.,w=0.,pv=0.,pw=0.,measured_v=0.,measured_w=0.;
    const char *reason=""; CollisionEvidence collision; bool raw_available=true;
  };
  void record(const geometry_msgs::msg::PoseStamped &pose,double v,double w,double pv,double pw,
      const char *reason,CollisionEvidence collision,bool raw_available=true) {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    Event e; e.sequence=++event_count_; e.stamp=parent_.lock()->now().nanoseconds();
    e.pose_stamp=rclcpp::Time(pose.header.stamp).nanoseconds();
    e.x=pose.pose.position.x; e.y=pose.pose.position.y;
    const auto &q=pose.pose.orientation;
    e.yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
    e.v=v; e.w=w; e.pv=pv; e.pw=pw; e.reason=reason; e.collision=collision; e.raw_available=raw_available;
    e.measured_v=measured_v_; e.measured_w=measured_w_;
    if (size_==events_.size()) {
      if (events_[head_].sequence>last_published_sequence_) ++overflow_;
      head_=(head_+1)%events_.size(); --size_;
    }
    events_[(head_+size_)%events_.size()]=e; ++size_;
    if (std::string(reason)=="wheel_projection" || std::string(reason)=="collision_velocity_projection") ++projected_;
    else if (std::string(reason)=="no_nonzero_wheel_feasible_command") ++zeroed_;
    else ++rejected_;
  }
  void publishEvidence() {
    if (!evidence_publisher_ || !evidence_publisher_->is_activated()) return;
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    std::ostringstream out; out<<std::setprecision(10);
    out<<"{\"schema\":\"r2b4.motion_evidence.v1\",\"component\":\"nav2_wheel_adapter\",\"frame_id\":\""
       <<costmap_ros_->getGlobalFrameID()<<"\",\"event_count\":"<<event_count_
       <<",\"projected_count\":"<<projected_<<",\"zeroed_count\":"<<zeroed_<<",\"rejected_count\":"<<rejected_
       <<",\"overflow_count\":"<<overflow_<<",\"events\":[";
    for (size_t i=0;i<size_;++i) {
      const auto &e=events_[(head_+i)%events_.size()]; const auto &c=e.collision;
      const bool forwarded=std::string(e.reason)=="wheel_projection" ||
          std::string(e.reason)=="collision_velocity_projection" ||
          std::string(e.reason)=="no_nonzero_wheel_feasible_command";
      if (i) out<<',';
      out<<"{\"sequence\":"<<e.sequence<<",\"measurement_ros_ns\":"<<e.stamp<<",\"pose_ros_ns\":"<<e.pose_stamp<<",\"reason\":\""<<e.reason
         <<"\",\"raw_available\":"<<(e.raw_available?"true":"false")<<",\"pose\":["<<e.x<<','<<e.y<<','<<e.yaw
         <<"],\"raw\":["<<e.v<<','<<e.w<<"],\"projected\":["<<e.pv<<','<<e.pw
         <<"],\"forwarded\":["<<(forwarded?e.pv:0.)<<','<<(forwarded?e.pw:0.)
         <<"],\"measured_velocity\":["<<e.measured_v<<','<<e.measured_w
         <<"],\"raw_wheels\":["<<e.v-e.w*limits_.track*.5<<','<<e.v+e.w*limits_.track*.5
         <<"],\"projected_wheels\":["<<e.pv-e.pw*limits_.track*.5<<','<<e.pv+e.pw*limits_.track*.5
         <<"],\"trajectory\":{\"model\":\"constant_twist_arc\",\"duration_s\":1.0,\"steps\":"<<c.steps
         <<",\"sampling_guard_m\":"<<c.sampling_guard
         <<"},\"costmap_geometry\":{\"origin\":["<<c.origin_x<<','<<c.origin_y<<"],\"resolution_m\":"<<c.resolution
         <<"},\"first_blocking\":";
      if (c.sample<0) out<<"null";
      else out<<"{\"sample\":"<<c.sample<<",\"t_s\":"<<static_cast<double>(c.sample)/std::max(1,c.steps)
         <<",\"pose\":["<<c.x<<','<<c.y<<','<<c.yaw<<"],\"cell\":["<<c.cell_x<<','<<c.cell_y<<"],\"cost\":"<<c.cost<<'}';
      out<<'}';
    }
    out<<"],\"compute_timing\":{";
    {
      std::lock_guard<std::mutex> timing_lock(timing_mutex_);
      const char *names[]={"total","mppi","raw_publish","projection","collision"};
      for (size_t i=0;i<timings_.size();++i) {
        if (i) out<<',';
        const auto &t=timings_[i];
        out<<'"'<<names[i]<<"\":{\"count\":"<<t.count<<",\"wall_max_ns\":"<<t.wall_max_ns
           <<",\"wall_max_cpu_ns\":"<<t.wall_max_cpu_ns
           <<",\"cpu_max_ns\":"<<t.cpu_max_ns<<",\"wall_sum_ns\":"<<t.wall_sum_ns
           <<",\"cpu_sum_ns\":"<<t.cpu_sum_ns<<'}';
      }
    }
    out<<"}}";
    std_msgs::msg::String message; message.data=out.str(); evidence_publisher_->publish(message);
    last_published_sequence_=event_count_;
    // Retain the last 16 across publications: a superseded observation still
    // exposes sequence gaps and cumulative totals, without a control payload.
  }
  WheelLimits limits_{};
  double mission_max_v_{},mission_max_omega_{};
  double measured_v_=0.,measured_w_=0.;
  std::mutex evidence_mutex_;
  std::mutex timing_mutex_;
  std::array<Timing,5> timings_{};
  std::array<Event,16> events_{};
  size_t head_=0,size_=0;
  uint64_t event_count_=0,projected_=0,zeroed_=0,rejected_=0,overflow_=0;
  uint64_t last_published_sequence_=0;
  rclcpp::TimerBase::SharedPtr evidence_timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr evidence_publisher_;
};
} // namespace r2b4_nav2
PLUGINLIB_EXPORT_CLASS(r2b4_nav2::WheelAwareMPPIController,nav2_core::Controller)
