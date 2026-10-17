#include <memory>
#include <mutex>
#include <vector>
#include <cmath>
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "interfaces/msg/predictions.hpp"
#include "interfaces/msg/perception_health.hpp"
#include "control_mppi_critic/proximity.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi::critics {
class PredictionCritic : public CriticFunction {
  using Predictions=interfaces::msg::Predictions;
  using Health=interfaces::msg::PerceptionHealth;
  std::mutex mutex_;
  Predictions::ConstSharedPtr predictions_;
  Health::ConstSharedPtr health_;
  rclcpp::Subscription<Predictions>::SharedPtr subscription_;
  rclcpp::Subscription<Health>::SharedPtr health_subscription_;
  double weight_, radius_, margin_, stale_, epsilon_;
  bool required_;
  static double seconds(const builtin_interfaces::msg::Time &t) {return t.sec+t.nanosec*1e-9;}
public:
  void initialize() override {
    auto get=parameters_handler_->getParamGetter(name_);
    get(weight_,"cost_weight",8.0); get(radius_,"robot_radius",0.168);
    get(margin_,"safety_margin",0.20); get(stale_,"stale_timeout",1.2);
    get(epsilon_,"epsilon",1e-6); get(required_,"require_predictions",true);
    auto node=parent_.lock();
    subscription_=node->create_subscription<Predictions>("/perception/predictions",10,
      [this](Predictions::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);predictions_=m;});
    health_subscription_=node->create_subscription<Health>("/perception/health",10,
      [this](Health::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);health_=m;});
  }
  void score(CriticData &data) override {
    if(!enabled_) return;
    Predictions::ConstSharedPtr predictions;Health::ConstSharedPtr health;
    {std::lock_guard<std::mutex> lock(mutex_);predictions=predictions_;health=health_;}
    const double now=parent_.lock()->now().seconds();
    if(!predictions || !health || health->status==Health::FAILED ||
       now-seconds(predictions->header.stamp)>stale_ || now-seconds(health->header.stamp)>stale_ ||
       predictions->map_revision!=health->map_revision) {if(required_)data.fail_flag=true;return;}
    double tx=0.,ty=0.,angle=0.;
    try {
      auto tf=costmap_ros_->getTfBuffer()->lookupTransform(predictions->header.frame_id,
        costmap_ros_->getGlobalFrameID(),tf2::TimePointZero).transform;
      tx=tf.translation.x;ty=tf.translation.y;angle=tf2::getYaw(tf.rotation);
    } catch(const tf2::TransformException &) {data.fail_flag=true;return;}
    const double c=std::cos(angle),s=std::sin(angle);
    struct Sample {double probability,x,y,xx,xy,yy;};
    const size_t steps=data.trajectories.x.shape()[1];
    std::vector<std::vector<Sample>> samples(steps);
    for(const auto &robot:predictions->robots) {
      double mass=robot.leftover.probability;for(const auto &g:robot.groups)mass+=g.probability;
      if(!std::isfinite(mass)||std::abs(mass-1.)>epsilon_) {data.fail_flag=true;return;}
      std::vector<const interfaces::msg::PredictedTrajectory*> groups;
      for(const auto &g:robot.groups)groups.push_back(&g);groups.push_back(&robot.leftover);
      for(const auto *g:groups) {
        if(!std::isfinite(g->probability)||g->probability<0){data.fail_flag=true;return;}
        if(g->probability==0)continue;
        if(g->points.size()<2){data.fail_flag=true;return;}
        for(size_t t=0;t<steps;++t) {
          const double when=now+(t+1)*data.model_dt;
          auto b=std::lower_bound(g->points.begin(),g->points.end(),when,
            [](const auto &p,double time){return seconds(p.stamp)<time;});
          if(b==g->points.begin()||b==g->points.end()){data.fail_flag=true;return;}
          auto a=b-1;const double duration=seconds(b->stamp)-seconds(a->stamp);
          if(duration<=0){data.fail_flag=true;return;}
          const double f=(when-seconds(a->stamp))/duration;
          auto lerp=[f](double a,double b){return a+(b-a)*f;};
          Sample sample{g->probability,lerp(a->x,b->x),lerp(a->y,b->y),
            lerp(a->covariance[0],b->covariance[0]),lerp(a->covariance[1],b->covariance[1]),lerp(a->covariance[3],b->covariance[3])};
          if(!std::isfinite(sample.x)||!std::isfinite(sample.y)||
             std::abs(a->covariance[1]-a->covariance[2])>epsilon_||std::abs(b->covariance[1]-b->covariance[2])>epsilon_||
             !std::isfinite(sample.xx)||!std::isfinite(sample.xy)||!std::isfinite(sample.yy)||
             sample.xx<0||sample.yy<0||sample.xx*sample.yy<sample.xy*sample.xy-epsilon_) {data.fail_flag=true;return;}
          samples[t].push_back(sample);
        }
      }
    }
    for(size_t i=0;i<data.trajectories.x.shape()[0];++i) {
      double cost=0.;
      for(size_t t=0;t<steps;++t) {
        const double x=c*data.trajectories.x(i,t)-s*data.trajectories.y(i,t)+tx;
        const double y=s*data.trajectories.x(i,t)+c*data.trajectories.y(i,t)+ty;
        for(const auto &p:samples[t])cost+=p.probability*control::proximity(x-p.x,y-p.y,p.xx,p.xy,p.yy,2*radius_+margin_,epsilon_);
      }
      data.costs(i)+=static_cast<float>(weight_*cost/steps);
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::PredictionCritic,mppi::critics::CriticFunction)
