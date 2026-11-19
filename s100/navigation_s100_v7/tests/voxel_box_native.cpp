#include "tianji_navigation_3d/world.hpp"
#include <random>
using namespace tianji_nav3d;
void check(bool value,const char*why){if(!value)throw std::runtime_error(why);}
World make_world(double x,bool refined=true){World w;w.resolution=.05;w.margin=.05;w.refine_voxel_boxes=refined;Shape s;s.link="test";s.low=V3::Constant(-.05);s.high=V3::Constant(.05);w.add_shape(s);w.add_point(V3(x,0,0));return w;}
int main(){
 check(make_world(.14,false).at({0,0,0}).collision,"LEGACY_BOUND_PRESERVED");
 check(!make_world(.14).at({0,0,0}).collision,"FULL_CELL_CLEAR_6_5CM");
 check(make_world(.12).at({0,0,0}).collision,"FULL_CELL_BLOCK_4_5CM");
 check(make_world(.125).at({0,0,0}).collision,"EXACT_5CM_BOUNDARY_BLOCKED");
 check(make_world(.14).at({0,0,0},.02).collision,"MOTION_PADDING_PRESERVED");
 auto rotated=make_world(.14);rotated.voxel_rotation=Eigen::AngleAxisd(M_PI/4.,V3::UnitZ()).toRotationMatrix();check(!rotated.at({0,0,0}).collision,"ROTATED_CELL_CLEAR");rotated.points.clear();rotated.grid.clear();rotated.add_point(V3(.13,0,0));check(rotated.at({0,0,0}).collision,"ROTATED_CELL_CONTACT");
 auto grid=make_world(.14);for(int i=0;i<130;++i)grid.add_point(V3(20+i*.3,20,20));check(!grid.at({0,0,0}).collision,"GRID_QUERY_REFINEMENT");
 Json q=Json::object();for(auto side:{"L","R"})for(int i=1;i<=7;++i)q["Joint"+std::to_string(i)+"_"+side]=0.;for(auto name:{"head_pitch_joint","left_robotiq_85_left_knuckle_joint","right_robotiq_85_left_knuckle_joint"})q[name]=0.;
 Json packet={{"schema_version",1},{"observation_time",1.},{"pose_time",1.},{"snapshot_id","test"},{"model_sha256",std::string(64,'a')},{"frame_id","nav3d_map"},{"source","recorded"},{"alignment_verified",false},{"planning_mode","immutable_snapshot_replay"},{"execution_permission",false},{"clearance_assumed",true},{"geometry_owner","verified_local_urdf"},{"alignment_assumption","vendor_base_to_model_identity"},{"joint_positions_rad",q},{"bounded_missing_joints",Json::array()},{"resolution_m",.05},{"voxel_box_rotation",{{1.,0.,0.},{0.,1.,0.},{0.,0.,1.}}},{"shapes",Json::array({{{"link","test"},{"matrix",{{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}}},{"bounds",{{-.05,-.05,-.05},{.05,.05,.05}}}}})},{"occupied_points",{{.14,0.,0.}}}};
 auto parsed=World::parse_snapshot_preview(packet.dump(),std::string(64,'a'));check(parsed->refine_voxel_boxes&&!parsed->at({0,0,0}).collision,"PARSED_BOX_FRAME");
 auto bad=packet;bad["voxel_box_rotation"][0][0]=2.;bool rejected=false;try{World::parse_snapshot_preview(bad.dump(),std::string(64,'a'));}catch(const std::exception&){rejected=true;}check(rejected,"INVALID_CELL_ROTATION_REJECTED");
 auto live=packet;live["occupied_points"]={{.12,0.,0.}};
 live["live_voxel_refinements"]=Json::array({{{"point_index",0},{"bounds",{{0.,-.025,-.025},{.025,.025,.025}}},{"uncertainty_m",.02},{"supporting_frames",3},{"raw_points",6},{"map_contribution",false},{"source","THREE_SCAN_RAW_TABLETOP"}}});
 auto fine=World::parse_snapshot_preview(live.dump(),std::string(64,'a'));
 check(!fine->at({0,0,0}).collision,"LIVE_RAW_WITH_2CM_UNCERTAINTY_CLEAR");
 check(fine->at({.03,0,0}).collision,"LIVE_RAW_REAL_CONTACT_BLOCKED");
 check(fine->at({0,0,0},.02).collision,"LIVE_RAW_MOTION_PADDING_PRESERVED");
 live["live_voxel_refinements"][0]["map_contribution"]=true;rejected=false;try{World::parse_snapshot_preview(live.dump(),std::string(64,'a'));}catch(const std::exception&){rejected=true;}check(rejected,"MAP_REFINEMENT_REJECTED");
 live["live_voxel_refinements"][0]["map_contribution"]=false;live["live_voxel_refinements"][0]["uncertainty_m"]=.005;rejected=false;try{World::parse_snapshot_preview(live.dump(),std::string(64,'a'));}catch(const std::exception&){rejected=true;}check(rejected,"UNCERTAINTY_REDUCTION_REJECTED");
 packet.erase("voxel_box_rotation");check(World::parse_snapshot_preview(packet.dump(),std::string(64,'a'))->at({0,0,0}).collision,"LEGACY_PACKET_NOT_REINTERPRETED");
 auto cache_world=std::make_shared<World>(make_world(.14));SnapshotQueryCache cache;cache.reset(cache_world);
 for(int i=-30;i<=30;++i){Pose pose={i*.001,0,0};check(cache.at(pose,.001)==cache_world->at(pose,.001).collision,"CACHE_BOUNDARY_CONSISTENCY");}
 check(!cache_world->segment({0,0,0},{.005,0,0}).collision,"SAFE_SEGMENT");check(cache_world->segment({0,0,0},{.04,0,0}).collision,"CONTACT_ON_SEGMENT");
 std::mt19937 generator(471);std::uniform_real_distribution<double> unit(-1,1);int witnesses=0;
 for(int trial=0;trial<3000;++trial){
  World w;w.resolution=.05;w.margin=.05;w.refine_voxel_boxes=true;Shape shape;shape.link="random";shape.low=V3(-.09,-.05,-.07);shape.high=V3(.09,.05,.07);shape.translation=V3(unit(generator),unit(generator),unit(generator));shape.rotation=(Eigen::AngleAxisd(unit(generator)*3.,V3::UnitZ())*Eigen::AngleAxisd(unit(generator),V3::UnitY())).toRotationMatrix();w.add_shape(shape);w.voxel_rotation=(Eigen::AngleAxisd(unit(generator)*3.,V3::UnitX())*Eigen::AngleAxisd(unit(generator),V3::UnitZ())).toRotationMatrix();
  V3 witness_local(.09*unit(generator),.05*unit(generator),.07*unit(generator));int axis=trial%3;witness_local[axis]=(trial%2?shape.high[axis]:shape.low[axis])+(trial%2?1:-1)*(.049*std::abs(unit(generator)));
  V3 witness=shape.rotation*witness_local+shape.translation;V3 offset(.025*unit(generator),.025*unit(generator),.025*unit(generator));V3 cell=witness-w.voxel_rotation*offset;w.add_point(cell);
  V3 gap=(shape.low-witness_local).cwiseMax(witness_local-shape.high).cwiseMax(0.);check(gap.norm()<.05,"INVALID_TEST_WITNESS");check(w.at({0,0,0}).collision,"REAL_CONTACT_MUST_NEVER_BE_CLEARED");++witnesses;
  World::LiveBounds measured{(offset-V3::Constant(.02)).cwiseMax(-.025),(offset+V3::Constant(.02)).cwiseMin(.025)};
  w.live_bounds.emplace(std::array<double,3>{cell.x(),cell.y(),cell.z()},measured);
  check(w.at({0,0,0}).collision,"MEASURED_CONTACT_MUST_NEVER_BE_CLEARED");
 }
 std::cout<<"VOXEL_BOX_PASS witnesses="<<witnesses<<std::endl;
}
