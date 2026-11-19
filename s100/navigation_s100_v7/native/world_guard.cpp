#include "tianji_navigation_3d/world.hpp"
#include <fstream>
#include <iostream>
using namespace tianji_nav3d;
int main(int argc,char**argv){
 if(argc!=2)return 2;
 try{
  Json input;std::ifstream(argv[1])>>input;auto world=World::parse_snapshot_preview(input.dump(),input.at("model_sha256"));std::string line;
  while(std::getline(std::cin,line)){
   try{Json request=Json::parse(line);auto poses=request.at("poses");bool collision=false;std::string link;Pose prev;bool first=true;
    for(const auto&q:poses){Pose p={scalar(q[0]),scalar(q[1]),scalar(q[2])};Contact c=first?world->at(p):world->segment(prev,p,.02);if(c.collision){collision=true;link=c.link;break;}prev=p;first=false;}
    std::cout<<Json({{"id",request.value("id",0)},{"clear",!collision&&!first},{"link",link}}).dump()<<std::endl;
   }catch(const std::exception&e){std::cout<<Json({{"clear",false},{"error",e.what()}}).dump()<<std::endl;}
  }
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}return 0;
}
