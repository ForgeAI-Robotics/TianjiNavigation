#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <array>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <set>
#include <unordered_map>
#include <vector>

namespace tianji_nav3d {
using Json = nlohmann::json;
using V3 = Eigen::Vector3d;
using Pose = std::array<double, 3>;
inline double wall_now() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
inline double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}
inline void require(bool valid, const char *reason) {if (!valid) throw std::runtime_error(reason);}
inline double scalar(const Json &j) {
  require(j.is_number(), "NON_NUMERIC_INPUT");
  double x = j.get<double>(); require(std::isfinite(x), "NONFINITE_INPUT"); return x;
}
inline V3 vector3(const Json &j) {
  require(j.is_array() && j.size() == 3, "VECTOR3_REQUIRED");
  return V3(scalar(j[0]), scalar(j[1]), scalar(j[2]));
}
struct Cell {
  int x,y,z;
  bool operator==(const Cell &b) const {return x == b.x && y == b.y && z == b.z;}
};
struct CellHash {
  size_t operator()(const Cell &k) const {
    return size_t(uint64_t(int64_t(k.x)*73856093) ^ uint64_t(int64_t(k.y)*19349663) ^ uint64_t(int64_t(k.z)*83492791));
  }
};
struct Shape {
  std::string link;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  V3 translation = V3::Zero(), low = V3::Zero(), high = V3::Zero();
};
struct Contact {bool collision = false; std::string link;};
struct World {
  double stamp = 0., pose_stamp = 0., resolution = .03, margin = .05, radius = 0.;
  std::string id, model, frame, source;
  bool complete = false, alignment = false, snapshot_preview = false;
  bool refine_voxel_boxes=false;
  Eigen::Matrix3d voxel_rotation=Eigen::Matrix3d::Identity();
  struct LiveBounds {V3 low,high;};
  std::map<std::array<double,3>,LiveBounds> live_bounds;
  std::vector<Shape> shapes;
  std::vector<V3> points;
  std::unordered_map<Cell, std::vector<V3>, CellHash> grid;
  static constexpr double bucket = .20;
  Cell key(const V3 &p) const {
    return {int(std::floor(p.x()/bucket)), int(std::floor(p.y()/bucket)), int(std::floor(p.z()/bucket))};
  }
  bool fresh(double now) const {
    return !snapshot_preview && complete && alignment && now >= stamp-.05 && now-stamp <= .65 && now >= pose_stamp-.05 && now-pose_stamp <= .25;
  }
  void add_point(const V3 &p) {grid[key(p)].push_back(p);points.push_back(p);}
  void add_shape(const Shape &s) {
    shapes.push_back(s);
    for (int i=0;i<8;++i) {
      V3 p=s.rotation*V3(i&1?s.high.x():s.low.x(),i&2?s.high.y():s.low.y(),i&4?s.high.z():s.low.z())+s.translation;
      radius=std::max(radius,p.head<2>().norm());
    }
  }
  // Certify separation of the full oriented voxel box from the URDF OBB
  // plus its original Euclidean safety/motion margin. Any inconclusive case
  // remains a collision; no occupied points or voxel extent are removed.
  bool voxel_separated(const V3&p,const V3&center,const Eigen::Matrix3d&rotation,const V3&half,double clearance) const {
    if(!refine_voxel_boxes)return false;
    require(clearance>=0.,"NEGATIVE_VOXEL_BOX_CLEARANCE");
    V3 cell_center=p,cell_half=V3::Constant(resolution*.5);
    auto found=live_bounds.find({p.x(),p.y(),p.z()});
    if(found!=live_bounds.end()) {
      cell_center+=voxel_rotation*((found->second.low+found->second.high)*.5);
      cell_half=(found->second.high-found->second.low)*.5;
    }
    V3 delta=cell_center-center;
    auto separates=[&](const V3&axis){
      double length=axis.norm();if(length<1e-10)return false;
      double body=half.dot((rotation.transpose()*axis).cwiseAbs());
      double cell=cell_half.dot((voxel_rotation.transpose()*axis).cwiseAbs());
      return std::abs(delta.dot(axis))>body+cell+clearance*length+1e-10;
    };
    for(int i=0;i<3;++i)if(separates(rotation.col(i))||separates(voxel_rotation.col(i)))return true;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)if(separates(rotation.col(i).cross(voxel_rotation.col(j))))return true;
    return false;
  }
  Contact at(const Pose &pose, double interpolation_padding=0.) const {
    require(std::isfinite(pose[0]) && std::isfinite(pose[1]) && std::isfinite(pose[2]),"INVALID_QUERY_POSE");
    if (points.empty()) return {};
    const double padding=margin+std::sqrt(3.)*resolution/2.+interpolation_padding;
    Eigen::Matrix3d yaw=Eigen::AngleAxisd(pose[2],V3::UnitZ()).toRotationMatrix();
    for (const auto &s:shapes) {
      auto rotation=(yaw*s.rotation).eval(); V3 t=yaw*s.translation+V3(pose[0],pose[1],0.);
      V3 center=rotation*((s.low+s.high)/2.)+t;
      V3 extent=rotation.cwiseAbs()*((s.high-s.low)/2.)+V3::Constant(padding);
      if(points.size()<128) {
        for(const auto &p:points) {
          if(((p-center).cwiseAbs().array()>extent.array()).any()) continue;
          V3 local=rotation.transpose()*(p-t);
          V3 gap=(s.low-local).cwiseMax(local-s.high).cwiseMax(0.);
          if(gap.squaredNorm()<=padding*padding && !voxel_separated(p,center,rotation,(s.high-s.low)/2.,margin+interpolation_padding)) return {true,s.link};
        }
        continue;
      }
      Cell a=key(center-extent),b=key(center+extent);
      require(int64_t(b.x-a.x+1)*(b.y-a.y+1)*(b.z-a.z+1)<200000,"SHAPE_QUERY_BUDGET");
      for (int x=a.x;x<=b.x;++x) for(int y=a.y;y<=b.y;++y) for(int z=a.z;z<=b.z;++z) {
        auto it=grid.find({x,y,z}); if(it==grid.end()) continue;
        for (const auto &p:it->second) {
          V3 local=rotation.transpose()*(p-t);
          V3 gap=(s.low-local).cwiseMax(local-s.high).cwiseMax(0.);
          if(gap.squaredNorm() <= padding*padding && !voxel_separated(p,center,rotation,(s.high-s.low)/2.,margin+interpolation_padding)) return {true,s.link};
        }
      }
    }
    return {};
  }
  Contact segment(const Pose &a, const Pose &b, double step=.025) const {
    double dyaw=angle(b[2]-a[2]);
    double travel=std::hypot(b[0]-a[0],b[1]-a[1])+radius*std::abs(dyaw);
    require(std::isfinite(travel) && travel<=20.,"SEGMENT_QUERY_BUDGET");
    int n=std::max(1,int(std::ceil(travel/step))); require(n<=1200,"SEGMENT_SAMPLE_BUDGET");
    for(int i=0;i<=n;++i) {
      double f=double(i)/n;
      auto c=at({a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*dyaw},travel/n/2.);
      if(c.collision) return c;
    }
    return {};
  }
  static std::shared_ptr<World> parse(const std::string &text, const std::string &expected_model) {
    return parse_impl(text,expected_model,false);
  }
  static std::shared_ptr<World> parse_snapshot_preview(const std::string &text, const std::string &expected_model) {
    return parse_impl(text,expected_model,true);
  }
private:
  static std::shared_ptr<World> parse_impl(const std::string &text, const std::string &expected_model, bool preview) {
    require(text.size()<=6000000,"SNAPSHOT_SIZE_BUDGET");
    auto j=Json::parse(text); auto w=std::make_shared<World>();
    require(j.at("schema_version")==1,"SNAPSHOT_SCHEMA");
    w->stamp=scalar(j.at("observation_time")); w->pose_stamp=scalar(j.at("pose_time"));
    require(w->stamp>0. && w->pose_stamp>0.,"ORIGINAL_TIMESTAMPS_REQUIRED");
    w->id=j.at("snapshot_id").get<std::string>(); require(!w->id.empty() && w->id.size()<=128,"SNAPSHOT_ID");
    w->model=j.at("model_sha256").get<std::string>(); require(w->model==expected_model && expected_model.size()==64,"MODEL_HASH_MISMATCH");
    w->frame=j.at("frame_id").get<std::string>(); require(w->frame=="nav3d_map","WORLD_FRAME_MISMATCH");
    w->source=j.at("source").get<std::string>();
    require(w->source=="synthetic" || w->source=="recorded" || w->source=="measured","SOURCE_INVALID");
    w->alignment=j.at("alignment_verified").get<bool>();
    if(preview) {
      require(j.at("planning_mode")=="immutable_snapshot_replay" && j.at("execution_permission")==false,"SNAPSHOT_PREVIEW_CONTRACT_REQUIRED");
      require(j.at("clearance_assumed")==true && w->source=="recorded","EXPLICIT_RECORDED_CLEARANCE_ASSUMPTION_REQUIRED");
      require(j.at("geometry_owner")=="verified_local_urdf" && j.at("alignment_assumption")=="vendor_base_to_model_identity","LOCAL_REPLAY_GEOMETRY_CONTRACT_REQUIRED");
      w->snapshot_preview=true;
    } else {
      require(!j.contains("planning_mode"),"SNAPSHOT_REPLAY_REJECTED_ON_LIVE_INPUT");
      require(w->alignment,"ALIGNMENT_UNVERIFIED");
    }
    const auto &q=j.at("joint_positions_rad");
    require(q.is_object() && (preview ? q.size()>=14 && q.size()<=17 : q.size()==17),"FULL_POSTURE_REQUIRED");
    std::set<std::string> allowed,missing;
    for (const auto &side:{"L","R"}) for(int i=1;i<=7;++i) {std::string name="Joint"+std::to_string(i)+"_"+side;scalar(q.at(name));allowed.insert(name);}
    for (const auto &name:{"head_pitch_joint","left_robotiq_85_left_knuckle_joint","right_robotiq_85_left_knuckle_joint"}) {
      allowed.insert(name);
      if(q.contains(name)) scalar(q.at(name));
      else {require(preview,"FULL_POSTURE_REQUIRED");missing.insert(name);}
    }
    for(auto it=q.begin();it!=q.end();++it) require(allowed.count(it.key()),"UNKNOWN_JOINT_NAME");
    if(preview) {
      auto declared=j.at("bounded_missing_joints").get<std::vector<std::string>>();
      require(std::set<std::string>(declared.begin(),declared.end())==missing,"MISSING_JOINT_BOUNDS_CONTRACT");
    }
    w->complete=missing.empty(); w->resolution=scalar(j.at("resolution_m"));
    require(w->resolution>=.01 && w->resolution<=.10,"VOXEL_RESOLUTION");
    if(j.contains("voxel_box_rotation")) {
      require(preview,"VOXEL_BOX_REFINEMENT_SNAPSHOT_ONLY");auto matrix=j.at("voxel_box_rotation");
      require(matrix.is_array()&&matrix.size()==3,"VOXEL_BOX_ROTATION");
      for(int r=0;r<3;++r){require(matrix[r].is_array()&&matrix[r].size()==3,"VOXEL_BOX_ROTATION");for(int c=0;c<3;++c)w->voxel_rotation(r,c)=scalar(matrix[r][c]);}
      require((w->voxel_rotation.transpose()*w->voxel_rotation-Eigen::Matrix3d::Identity()).norm()<1e-6&&std::abs(w->voxel_rotation.determinant()-1.)<1e-6,"VOXEL_BOX_ROTATION");
      w->refine_voxel_boxes=true;
    }
    auto shapes=j.at("shapes"); require(shapes.is_array() && shapes.size()>0 && shapes.size()<=100,"SHAPE_COUNT");
    for(const auto &s:shapes) {
      Shape v;v.link=s.at("link").get<std::string>(); auto m=s.at("matrix");
      require(m.is_array() && m.size()==4,"SHAPE_MATRIX");
      for(int r=0;r<4;++r) {require(m[r].is_array() && m[r].size()==4,"SHAPE_MATRIX");for(int c=0;c<4;++c) scalar(m[r][c]);}
      for(int r=0;r<3;++r) {v.translation[r]=scalar(m[r][3]);for(int c=0;c<3;++c)v.rotation(r,c)=scalar(m[r][c]);}
      require((v.rotation.transpose()*v.rotation-Eigen::Matrix3d::Identity()).norm()<1e-4 && std::abs(v.rotation.determinant()-1.)<1e-4,"SHAPE_ROTATION");
      require(std::abs(scalar(m[3][3])-1.)<1e-8 && std::abs(scalar(m[3][0]))+std::abs(scalar(m[3][1]))+std::abs(scalar(m[3][2]))<1e-8,"SHAPE_HOMOGENEOUS_ROW");
      v.low=vector3(s.at("bounds")[0]);v.high=vector3(s.at("bounds")[1]);
      require((v.high.array()>=v.low.array()).all() && (v.high-v.low).norm()<5. && v.translation.norm()<5.,"SHAPE_BOUNDS");
      w->add_shape(v);
    }
    const auto &points=j.at("occupied_points");require(points.is_array() && points.size()<=50000,"OCCUPIED_POINT_BUDGET");
    for(const auto &p:points) {V3 v=vector3(p); require(v.norm()<1000.,"POINT_RANGE"); w->add_point(v);}
    if(j.contains("live_voxel_refinements")) {
      const auto& rows=j.at("live_voxel_refinements");
      require(preview&&rows.is_array()&&rows.size()<=64,"LIVE_CELL_REFINEMENT_SCOPE");
      require(rows.empty()||w->refine_voxel_boxes,"LIVE_CELL_REQUIRES_GRID_ROTATION");
      for(const auto&r:rows) {
        require(r.at("point_index").is_number_integer(),"LIVE_CELL_INDEX");int index=r.at("point_index").get<int>();
        require(index>=0&&size_t(index)<w->points.size(),"LIVE_CELL_INDEX");
        require(r.at("source")=="THREE_SCAN_RAW_TABLETOP"&&r.at("map_contribution")==false,"LIVE_CELL_SOURCE");
        require(scalar(r.at("supporting_frames"))>=3&&scalar(r.at("raw_points"))>=3,"LIVE_CELL_SUPPORT");
        double uncertainty=scalar(r.at("uncertainty_m"));require(uncertainty>=.02&&uncertainty<=.03,"LIVE_CELL_UNCERTAINTY");
        auto bounds=r.at("bounds");require(bounds.is_array()&&bounds.size()==2,"LIVE_CELL_BOUNDS");
        LiveBounds b{vector3(bounds[0]),vector3(bounds[1])};
        require((b.low.array()>=-w->resolution/2.-1e-9).all()&&(b.high.array()<=w->resolution/2.+1e-9).all()&&(b.high.array()>=b.low.array()).all(),"LIVE_CELL_BOUNDS");
        // A measured point plus its uncertainty occupies at least this span
        // in each axis, including clipping against the original cell face.
        require(((b.high-b.low).array()>=std::min(uncertainty,w->resolution)-1e-9).all(),"LIVE_CELL_UNCERTAINTY_SPAN");
        auto p=w->points[index];require(w->live_bounds.emplace(std::array<double,3>{p.x(),p.y(),p.z()},b).second,"DUPLICATE_LIVE_CELL");
      }
    }
    // An empty occupied list is not a free-space certificate. No free rays are fabricated.
    return w;
  }
};
class WorldStore {
  std::mutex mutex_;
  std::shared_ptr<const World> world_;
public:
  void update(std::shared_ptr<const World> w) {std::lock_guard<std::mutex> lock(mutex_);world_=std::move(w);}
  std::shared_ptr<const World> get() {std::lock_guard<std::mutex> lock(mutex_);return world_;}
};
// Repeated identical publications do not renew observations. Any mutation after
// sealing invalidates this inbox until the node/plugin is restarted.
class SnapshotInbox {
  WorldStore store_;
  std::string sealed_;
  bool violated_=false;
public:
  void receive(const std::string &text,const std::string &model) {
    try {
      require(!violated_,"IMMUTABLE_SNAPSHOT_ALREADY_VIOLATED");
      if(!sealed_.empty()) {
        if(text!=sealed_) {violated_=true;throw std::runtime_error("IMMUTABLE_SNAPSHOT_CHANGED");}
        return;
      }
      auto parsed=World::parse_snapshot_preview(text,model);sealed_=text;store_.update(parsed);
    } catch(...) {store_.update(nullptr);throw;}
  }
  std::shared_ptr<const World> get() {return store_.get();}
};
// Exact pose cache for a sealed snapshot only. Padding is rounded UP by at
// most 10 micrometres; body pose and orientation are never quantized.
class SnapshotQueryCache {
  struct Key {uint64_t x,y,a;int p;bool operator==(const Key&o)const{return x==o.x&&y==o.y&&a==o.a&&p==o.p;}};
  struct Hash {size_t operator()(const Key&k)const{return size_t(k.x^(k.y*0x9e3779b97f4a7c15ULL)^(k.a*0xbf58476d1ce4e5b9ULL)^uint64_t(k.p));}};
  std::shared_ptr<const World> world_;
  std::unordered_map<Key,bool,Hash> cache_;
  struct CoarseKey {int x,y,a,p;bool operator==(const CoarseKey&o)const{return x==o.x&&y==o.y&&a==o.a&&p==o.p;}};
  struct CoarseHash {size_t operator()(const CoarseKey&k)const{return size_t(uint64_t(int64_t(k.x)*73856093)^uint64_t(int64_t(k.y)*19349663)^uint64_t(k.a*83492791)^uint64_t(k.p*2654435761u));}};
  std::unordered_map<CoarseKey,uint8_t,CoarseHash> coarse_;
  static uint64_t bits(double value){if(value==0.)value=0.;uint64_t out;std::memcpy(&out,&value,sizeof(out));return out;}
public:
  size_t calls=0,hits=0;
  void reset(std::shared_ptr<const World>w){if(w!=world_){world_=w;cache_.clear();coarse_.clear();calls=hits=0;}}
  size_t size()const{return cache_.size()+coarse_.size();}
  bool at(const Pose&pose,double interpolation_padding) {
    constexpr double dp=.00001;
    int padding_bin=int(std::ceil(interpolation_padding/dp));
    Key k{bits(pose[0]),bits(pose[1]),bits(pose[2]),padding_bin};
    ++calls;auto found=cache_.find(k);if(found!=cache_.end()){++hits;return found->second;}
    // Coarse cells are acceleration hints only. A Hausdorff motion bound
    // proves clear/blocked over the entire cell; boundary cells fall back to
    // the original exact query with no additional centimetre-scale padding.
    constexpr double xy=.025,da=M_PI/90.,padding_step=.0025;
    CoarseKey ck{int(std::round(pose[0]/xy)),int(std::round(pose[1]/xy)),int(std::round(angle(pose[2])/da)),int(std::floor(interpolation_padding/padding_step))};
    auto ci=coarse_.find(ck);uint8_t classification=2;
    if(ci!=coarse_.end())classification=ci->second;
    else {
      Pose sample={ck.x*xy,ck.y*xy,ck.a*da};
      double error=std::sqrt(2.)*xy/2.+2.*world_->radius*std::sin(da/4.)+1e-7;
      double lower=ck.p*padding_step,upper=(ck.p+1)*padding_step;
      if(!world_->at(sample,upper+error).collision)classification=0;
      else if(!world_->refine_voxel_boxes && world_->margin+std::sqrt(3.)*world_->resolution/2.+lower-error>0. && world_->at(sample,lower-error).collision)classification=1;
      if(coarse_.size()<250000)coarse_.emplace(ck,classification);
    }
    if(classification<2){++hits;return classification==1;}
    bool collision=world_->at(pose,padding_bin*dp).collision;
    if(cache_.size()<1000000)cache_.emplace(k,collision);
    return collision;
  }
  bool segment(const Pose&a,const Pose&b) {
    double dyaw=angle(b[2]-a[2]);double travel=std::hypot(b[0]-a[0],b[1]-a[1])+world_->radius*std::abs(dyaw);
    require(std::isfinite(travel)&&travel<=20.,"SEGMENT_QUERY_BUDGET");
    int n=std::max(1,int(std::ceil(travel/.025)));require(n<=1200,"SEGMENT_SAMPLE_BUDGET");
    for(int i=0;i<=n;++i){double f=double(i)/n;if(at({a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*dyaw},travel/n/2.))return true;}
    return false;
  }
};
inline void isolation_check() {
  auto env=[](const char *key){auto v=std::getenv(key);return std::string(v?v:"");};
  require(env("ROS_DOMAIN_ID")=="80" && env("ROS_LOCALHOST_ONLY")=="1","DOMAIN80_LOCALHOST_REQUIRED");
  for(auto k:{"FASTDDS_DEFAULT_PROFILES_FILE","FASTRTPS_DEFAULT_PROFILES_FILE","CYCLONEDDS_URI","ROS_DISCOVERY_SERVER"}) require(env(k).empty(),"EXTERNAL_DISCOVERY_PROHIBITED");
}
}  // namespace tianji_nav3d
