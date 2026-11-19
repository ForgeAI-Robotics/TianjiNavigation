// Continuous SE(2) Hybrid A*: differential travel, bounded local lateral assist.
// Offline immutable geometry only. No ROS, network, SDK or motion interface.
#include "tianji_navigation_3d/world.hpp"
#include <fstream>
#include <queue>
#include <unordered_map>
#include <algorithm>
using namespace tianji_nav3d;
#ifdef TIANJI_OFFICE_PREVIEW
constexpr double short_sweep_step=.002;
#else
constexpr double short_sweep_step=.018;
#endif
struct Key {int x,y,a,gear,lat,run,mode;bool operator==(const Key&b)const{return x==b.x&&y==b.y&&a==b.a&&gear==b.gear&&lat==b.lat&&run==b.run&&mode==b.mode;}};
struct Hash {size_t operator()(const Key&k)const {size_t h=0;for(int v:{k.x,k.y,k.a,k.gear,k.lat,k.run,k.mode})h=h*1000003+size_t(v+10000);return h;}};
struct Motion {double axial=0,lateral=0,turn=0;int gear=1;bool side=false;};
struct Node {Pose p;int parent;double g;Motion action;int lateral_steps=0,lateral_run=0;double tangent=0;};
struct Entry {double f;int id;bool operator<(const Entry&b)const{return f>b.f;}};
Pose advance(const Pose&p,const Motion&m,double f) {
 double d=m.axial*f,a=m.turn*f;
 if(m.side)return {p[0]-m.lateral*f*std::sin(p[2]),p[1]+m.lateral*f*std::cos(p[2]),p[2]};
 if(std::abs(m.turn)<1e-9)return {p[0]+d*std::cos(p[2]),p[1]+d*std::sin(p[2]),angle(p[2]+a)};
 if(std::abs(m.axial)<1e-9)return {p[0],p[1],angle(p[2]+a)};
 double r=m.axial/m.turn;return {p[0]+r*(std::sin(p[2]+a)-std::sin(p[2])),p[1]+r*(std::cos(p[2])-std::cos(p[2]+a)),angle(p[2]+a)};
}
struct Search {
 std::shared_ptr<World> world;Json in,config;Pose goal;V3 low,high;std::vector<int> grid;int width,height;double resolution;SnapshotQueryCache cache;
 bool known(const Pose&p)const {
  int x=int(std::floor((p[0]-low.x())/resolution)),y=int(std::floor((p[1]-low.y())/resolution));
  return x>=0&&y>=0&&x<width&&y<height&&grid[y*width+x]>=0&&grid[y*width+x]<100;
 }
 Key key(const Node&n,bool assist)const {
  bool fine=config.value("near_start_fine_escape_enabled",false)&&std::hypot(n.p[0],n.p[1])<.45;
  double cell=fine?.025:.075,yaw=fine?2.5:7.5;int bins=fine?144:48;
  return {int(std::round(n.p[0]/cell)),int(std::round(n.p[1]/cell)),(int(std::round(angle(n.p[2])*180/M_PI/yaw))+bins)%bins,n.action.gear,assist?n.lateral_steps:0,assist?n.lateral_run:0,(assist&&n.action.side?1:0)+(fine?2:0)};
 }
 bool swept(const Pose&p,const Motion&m,double step=.035) {
  double span=std::abs(m.axial)+std::abs(m.lateral)+world->radius*std::abs(m.turn);int count=std::max(1,int(std::ceil(span/step)));Pose prev=p;
  for(int i=1;i<=count;i++){auto q=advance(p,m,double(i)/count);if(!known(q)||cache.segment(prev,q))return false;prev=q;}return true;
 }
 double heading_tolerance() const {
  // Share terminal acceptance with the measured-pose follower. Never rewrite
  // the requested goal or the actual body heading for collision checks.
  auto fine=config.value("fine_goal",Json::object());
  double degrees=fine.value("enabled",false)?scalar(fine.at("heading_tolerance_deg")):3.;
  require(degrees>=.3&&degrees<=3.,"INVALID_PLANNER_HEADING_TOLERANCE");
  return degrees*M_PI/180.;
 }
 bool terminal_rotation_required() const {return std::abs(angle(goal[2]))>heading_tolerance();}
 bool short_segment_clear(const Pose& a,const Pose& b) {
  if(!world->segment(a,b,short_sweep_step).collision)return true;
  // Resolve near-threshold interpolation overlap at a finer resolution.
  // World::segment retains the same clearance and certified sample padding.
  return !world->segment(a,b,.00025).collision;
 }
 Json short_rectilinear() {
  double d=std::hypot(goal[0],goal[1]),total=std::abs(goal[0])+std::abs(goal[1]);
  if(!config.value("short_rectilinear_enabled",true)||d>config.value("short_translation_goal_distance_m",1.5)||total>config.value("short_rectilinear_max_length_m",2.)||std::abs(goal[0])<.05||std::abs(goal[1])<.05||std::abs(goal[2])>M_PI/6)return {{"found",false}};
  // Two body-axis translations, with fixed chassis heading, no corner rotation.
  for(bool lateral_first:{true,false}) {
   Pose previous={0,0,0};Json path=Json::array({previous}),motions=Json::array();bool valid=true;
   for(int leg=0;leg<2&&valid;leg++) {
    bool side=(leg==0)==lateral_first;Motion m;if(side){m.side=true;m.lateral=goal[1];}else m.axial=goal[0];
    Pose begin=previous;int n=std::max(1,int(std::ceil((std::abs(m.axial)+std::abs(m.lateral))/short_sweep_step)));
    for(int i=1;i<=n;i++){Pose p=advance(begin,m,double(i)/n);if(!known(p)||!short_segment_clear(previous,p)){valid=false;break;}path.push_back(p);previous=p;}
    motions.push_back({{"axial_m",m.axial},{"lateral_m",m.lateral},{"turn_rad",0.},{"mode",side?"local_lateral":"straight_translation"}});
   }
   if(!valid)continue;
   if(terminal_rotation_required()){Motion m;m.turn=goal[2];Pose begin=previous;int n=std::max(1,int(std::ceil(world->radius*std::abs(m.turn)/short_sweep_step)));for(int i=1;i<=n;i++){Pose p=advance(begin,m,double(i)/n);if(!known(p)||!short_segment_clear(previous,p)){valid=false;break;}path.push_back(p);previous=p;}motions.push_back({{"axial_m",0.},{"lateral_m",0.},{"turn_rad",m.turn},{"mode","in_place_rotation"}});}
   if(valid)return {{"found",true},{"profile","short_rectilinear_translation"},{"reason","SHORT_FIXED_HEADING_ORTHOGONAL_TRANSLATIONS"},{"path",path},{"primitives",motions},{"length_m",total},{"lateral_distance_m",std::abs(goal[1])},{"goal_error_m",0.},{"geometry_recheck_passed",true},{"expanded_nodes",0},{"compute_ms",0.},{"whole_route_holonomic",false}};
  }
  return {{"found",false}};
 }
 Json short_bridge() {
  Pose start={0,0,0};double d=std::hypot(goal[0],goal[1]);
  if(d>config.value("short_translation_goal_distance_m",1.5)||(!config.value("short_diagonal_enabled",false)&&std::abs(goal[0])>config.value("short_translation_max_axial_m",.35))||std::abs(goal[2])>M_PI/6)return {{"found",false}};
  Json path=Json::array();Pose previous=start;int count=std::max(1,int(std::ceil(d/short_sweep_step)));bool valid=true;
  for(int i=0;i<=count;i++){Pose p={goal[0]*i/count,goal[1]*i/count,0.};if(!known(p)||!short_segment_clear(previous,p)){valid=false;break;}path.push_back(p);previous=p;}
  if(valid&&terminal_rotation_required()){
   int n=std::max(1,int(std::ceil(world->radius*std::abs(goal[2])/short_sweep_step)));Pose begin=previous;
   for(int i=1;i<=n;i++){Pose p={goal[0],goal[1],goal[2]*i/n};if(!known(p)||!short_segment_clear(previous,p)){valid=false;break;}path.push_back(p);previous=p;}
  }
  if(!valid)return {{"found",false}};
  return {{"found",true},{"profile",std::abs(goal[1])<.05?"short_axial_translation":std::abs(goal[0])>=.05?"short_diagonal_translation":"short_local_translation"},{"reason","SIMPLE_SHORT_DISTANCE_SWEPT_TRANSLATION"},{"path",path},{"length_m",d},{"lateral_distance_m",std::abs(goal[1])},{"goal_error_m",0.},{"geometry_recheck_passed",true},{"expanded_nodes",0},{"compute_ms",0.},{"whole_route_holonomic",false}};
 }
 Json short_turn_first() {
  double d=std::hypot(goal[0],goal[1]);
  // Near the goal, recover a modest heading error directly rather than
  // forcing the 20cm search lattice to invent a large translating detour.
  double max_turn=M_PI/6;
  if(!terminal_rotation_required()||d>config.value("short_translation_goal_distance_m",1.5)||std::abs(goal[2])>max_turn)return {{"found",false}};
  Pose previous={0,0,0};Json path=Json::array({previous});
  auto append=[&](const Pose&p){if(!known(p)||!short_segment_clear(previous,p))return false;path.push_back(p);previous=p;return true;};
  int turns=std::max(1,int(std::ceil(world->radius*std::abs(goal[2])/short_sweep_step)));
  for(int i=1;i<=turns;i++)if(!append({0,0,goal[2]*i/turns}))return {{"found",false}};
  // These are coordinates in the original snapshot frame, not unrotated
  // body-axis commands. The follower converts with each measured heading.
  int count=std::max(1,int(std::ceil(d/short_sweep_step)));
  for(int i=1;i<=count;i++)if(!append({goal[0]*i/count,goal[1]*i/count,goal[2]}))return {{"found",false}};
  return {{"found",true},{"profile","short_turn_then_translation"},{"reason","SWEPT_START_ROTATION_THEN_TRANSLATION"},{"path",path},{"length_m",d},{"lateral_distance_m",std::abs(-std::sin(goal[2])*goal[0]+std::cos(goal[2])*goal[1])},{"goal_error_m",0.},{"geometry_recheck_passed",true},{"expanded_nodes",0},{"compute_ms",0.},{"whole_route_holonomic",false}};
 }
 Json search(bool assist) {
  cache.reset(world);auto began=std::chrono::steady_clock::now();double direct=std::hypot(goal[0],goal[1]);
  auto h=[&](const Pose&p){return std::hypot(p[0]-goal[0],p[1]-goal[1])+.10*std::abs(angle(p[2]-goal[2]));};
  std::vector<Node> nodes{{{0,0,0},-1,0,Motion{},0,0,0}};std::priority_queue<Entry> queue;queue.push({h(nodes[0].p),0});std::unordered_map<Key,double,Hash> best;best[key(nodes[0],assist)]=0;
  int finish=-1,expanded=0,rejected=0;std::string reason="SEARCH_EXHAUSTED";
  // One counter unit is 2.5cm, so fine and coarse lateral primitives consume
  // the same physical distance budget instead of charging 10cm per action.
  int max_lateral=int(std::floor(std::min(config.value("long_route_lateral_total_max_m",1.5),std::max(.4,direct*.25))/.025+1e-6));
  if(direct<=config.value("short_translation_goal_distance_m",1.5))max_lateral=60;
  int max_run=int(std::floor(config.value("local_lateral_episode_max_m",.4)/.025+1e-6));
  auto initial=world->at({0,0,0});
  if(initial.collision)return {{"found",false},{"reason","START_COLLISION"},{"blocking_link",initial.link}};
  if(!known({0,0,0})||!known(goal))return {{"found",false},{"reason","START_OR_GOAL_OUTSIDE_OBSERVED_CANDIDATE_GRID"}};
  if(world->at(goal).collision)return {{"found",false},{"reason","GOAL_COLLISION"},{"blocking_link",world->at(goal).link}};
  while(!queue.empty()) {
   if(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>config.value("search_budget_s_per_stage",12.)){reason="SEARCH_BUDGET_EXCEEDED";break;}
   if(int(nodes.size())>config.value("maximum_nodes",220000)){reason="NODE_BUDGET_EXCEEDED";break;}
   auto entry=queue.top();queue.pop();Node current=nodes[entry.id];if(current.g>best[key(current,assist)]+1e-8)continue;++expanded;
   if(std::hypot(current.p[0]-goal[0],current.p[1]-goal[1])<=.075&&std::abs(angle(current.p[2]-goal[2]))<=heading_tolerance()){finish=entry.id;reason="SWEPT_GEOMETRY_PATH_FOUND";break;}
   std::vector<Motion> actions;
   for(int gear:{1,-1})for(int t=-2;t<=2;t++)actions.push_back({gear*.20,0,t*M_PI/18,gear,false});
   actions.push_back({0,0,M_PI/12,0,false});actions.push_back({0,0,-M_PI/12,0,false});
   bool fine_start=config.value("near_start_fine_escape_enabled",false)&&std::hypot(current.p[0],current.p[1])<.45;
   if(fine_start){
    for(double step:{.05,.025})for(int gear:{1,-1})for(int t=-1;t<=1;t++)actions.push_back({gear*step,0,t*M_PI/72,gear,false});
    actions.push_back({0,0,M_PI/72,0,false});actions.push_back({0,0,-M_PI/72,0,false});
   }
   bool clutter=assist&&world->at(current.p,config.value("clutter_distance_m",.35)).collision;
   if(assist&&current.lateral_steps<max_lateral&&(clutter||h(current.p)<.75)&&(!current.action.side||current.lateral_run<max_run)) {
    actions.push_back({0,.10,0,0,true});actions.push_back({0,-.10,0,0,true});
    if(fine_start){for(double step:{.05,.025}){actions.push_back({0,step,0,0,true});actions.push_back({0,-step,0,0,true});}}
   }
   for(const auto&m:actions) {
    int lateral_units=m.side?int(std::lround(std::abs(m.lateral)/.025)):0;
    if(m.side&&(current.lateral_steps+lateral_units>max_lateral||current.lateral_run+lateral_units>max_run))continue;
    Node next;next.p=advance(current.p,m,1.);next.parent=entry.id;next.action=m;next.lateral_steps=current.lateral_steps+lateral_units;next.lateral_run=m.side?current.lateral_run+lateral_units:0;
    next.tangent=m.side?current.p[2]+(m.lateral>0?M_PI/2:-M_PI/2):current.p[2]+(m.gear<0?M_PI:0);
    double cost=std::abs(m.axial)*(m.gear<0?1.3:1.)+.08*std::abs(m.turn)+(m.gear==0&&!m.side?.08:0);
    if(m.side)cost+=std::abs(m.lateral)*config.value("lateral_distance_cost_multiplier",3.)+(current.action.side?0:config.value("lateral_activation_cost_m",.30));
    if(m.side!=current.action.side&&current.parent>=0)cost+=.10;
    if(m.gear*current.action.gear<0)cost+=.18;
    if(std::abs(m.axial)+std::abs(m.lateral)>0&&current.parent>=0)cost+=.10*std::max(0.,std::abs(angle(next.tangent-current.tangent))-M_PI/36);
    cost+=.015*std::abs(m.turn-current.action.turn);next.g=current.g+cost;
    auto k=key(next,assist);auto old=best.find(k);if(old!=best.end()&&old->second<=next.g)continue;
    if(!swept(current.p,m)){++rejected;continue;}best[k]=next.g;nodes.push_back(next);queue.push({next.g+1.15*h(next.p),int(nodes.size()-1)});
   }
  }
  Json path=Json::array(),motions=Json::array();double length=0,lateral=0;bool checked=false;double goal_error=-1.;
  if(finish>=0) {
   std::vector<int> ids;for(int i=finish;i>=0;i=nodes[i].parent)ids.push_back(i);std::reverse(ids.begin(),ids.end());path.push_back(nodes[0].p);checked=true;
   for(size_t i=1;i<ids.size();i++) {
    const auto&a=nodes[ids[i-1]],&b=nodes[ids[i]];const auto&m=b.action;length+=std::abs(m.axial)+std::abs(m.lateral);lateral+=std::abs(m.lateral);
    int n=std::max(1,int(std::ceil((std::abs(m.axial)+std::abs(m.lateral)+world->radius*std::abs(m.turn))/.018)));Pose previous=a.p;
    for(int j=1;j<=n;j++){auto p=advance(a.p,m,double(j)/n);if(!known(p)||world->segment(previous,p,.018).collision)checked=false;path.push_back(p);previous=p;}
    motions.push_back({{"axial_m",m.axial},{"lateral_m",m.lateral},{"turn_rad",m.turn},{"mode",m.side?"local_lateral":(m.gear?"differential_curve":"in_place_rotation")}});
   }
   goal_error=std::hypot(nodes[finish].p[0]-goal[0],nodes[finish].p[1]-goal[1]);
  }
  return {{"found",finish>=0&&checked},{"reason",reason},{"profile",assist?"differential_with_bounded_local_lateral":"differential_only"},{"path",path},{"primitives",motions},{"length_m",length},{"lateral_distance_m",lateral},{"lateral_budget_m",max_lateral*.025},{"goal_error_m",goal_error},{"geometry_recheck_passed",checked},{"expanded_nodes",expanded},{"rejected_edges",rejected},{"compute_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()},{"whole_route_holonomic",false}};
 }
};
int main(int argc,char**argv) {
 if(argc!=3)return 2;
 try {
  Json input;std::ifstream(argv[1])>>input;Search s;s.in=input;s.config=input.at("motion_policy");s.world=World::parse_snapshot_preview(input.at("world").dump(),input.at("world").at("model_sha256"));s.cache.reset(s.world);
  s.goal={scalar(input["goal"][0]),scalar(input["goal"][1]),scalar(input["goal"][2])};s.low=V3(scalar(input["grid"]["origin"][0]),scalar(input["grid"]["origin"][1]),0);s.width=input["grid"]["width"];s.height=input["grid"]["height"];s.resolution=input["grid"]["resolution"];s.grid=input["grid"]["data"].get<std::vector<int>>();require(int(s.grid.size())==s.width*s.height,"GRID_SHAPE");
  Json attempts=Json::array(),selected;auto initial=s.world->at({0,0,0});
  if(!initial.collision) {
   // Compare checked short-route candidates; a small preference rewards
   // aligning first, but never overrides collision/unknown-space checks.
   double best_cost=1e100;
   for(auto candidate:{s.short_turn_first(),s.short_bridge(),s.short_rectilinear()}){
    if(!candidate.value("found",false))continue;
    bool first=candidate["profile"]=="short_turn_then_translation";
    double cost=scalar(candidate["length_m"])+.08*std::abs(s.goal[2])+(s.terminal_rotation_required()&&!first?.01:0.);
    candidate["selection_cost_m"]=cost;candidate["heading_tolerance_deg"]=s.heading_tolerance()*180/M_PI;
    candidate["remaining_heading_error_deg"]=std::abs(angle(s.goal[2]-scalar(candidate["path"].back()[2])))*180/M_PI;
    attempts.push_back(candidate);if(cost<best_cost){best_cost=cost;selected=candidate;}
   }
  }
  if(selected.is_null()) {auto first=s.search(false);attempts.push_back(first);if(first.value("found",false))selected=first;
   else if(first.value("reason",std::string())!="START_COLLISION"&&first.value("reason",std::string())!="GOAL_COLLISION"){auto assisted=s.search(true);attempts.push_back(assisted);if(assisted.value("found",false))selected=assisted;}}
  if(!selected.is_null()){
   // Retain the finer witness samples in the exported path. Downstream
   // swept rechecks must see the same certified interpolation intervals.
   Json dense=Json::array();Pose previous;bool first=true;
   for(const auto&row:selected["path"]){Pose p={scalar(row[0]),scalar(row[1]),scalar(row[2])};
    if(first){dense.push_back(p);first=false;previous=p;continue;}
    if(s.world->segment(previous,p,.02).collision){
     double da=angle(p[2]-previous[2]);double span=std::hypot(p[0]-previous[0],p[1]-previous[1])+s.world->radius*std::abs(da);
     int n=std::max(1,int(std::ceil(span/.00025)));Pose begin=previous;
     for(int i=1;i<=n;i++)dense.push_back(Pose{begin[0]+(p[0]-begin[0])*i/n,begin[1]+(p[1]-begin[1])*i/n,begin[2]+da*i/n});
    }else dense.push_back(p);previous=p;
   }
   selected["path"]=dense;
  }
  Json out={{"engine","continuous SE2 Hybrid A*"},{"attempts",attempts},{"selected",selected},{"execution_permission",false},{"motion_commands_sent",0},{"clearance_assumed",true},{"unknown_space_certified_free",false}};
  std::ofstream(argv[2])<<out.dump(2);std::cout<<(selected.is_null()?"NO_PATH":selected["profile"].get<std::string>())<<"\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}return 0;
}
