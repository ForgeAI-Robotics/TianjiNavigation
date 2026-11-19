// File-only adapter to the installed PCL GeneralizedIterativeClosestPoint.
#include <pcl/registration/gicp.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
Cloud::Ptr read(const char* name) {
 std::ifstream f(name,std::ios::binary|std::ios::ate);if(!f)throw std::runtime_error("missing cloud");auto bytes=f.tellg();if(bytes%12)throw std::runtime_error("invalid xyz data");f.seekg(0);Cloud::Ptr p(new Cloud);p->resize(bytes/12);
 for(auto& v:*p)f.read(reinterpret_cast<char*>(&v.x),12);if(!f)throw std::runtime_error("truncated cloud");return p;
}
int main(int argc,char** argv) {
 if(argc!=5 && argc!=6) {std::cerr<<"source.xyz target.xyz seeds.txt results.txt [max_distance]\n";return 2;}
 auto source=read(argv[1]);auto target=read(argv[2]);std::ifstream seeds(argv[3]);std::ofstream results(argv[4]);results<<std::setprecision(10);
 pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
 gicp.setInputSource(source);gicp.setInputTarget(target);gicp.setCorrespondenceRandomness(20);gicp.setMaximumIterations(50);gicp.setMaximumOptimizerIterations(25);gicp.setMaxCorrespondenceDistance(argc==6?std::stof(argv[5]):.45);gicp.setTransformationEpsilon(1e-8);gicp.setEuclideanFitnessEpsilon(1e-7);
 float x,y,z,yaw;int i=0;
 while(seeds>>x>>y>>z>>yaw) {
  Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();guess(0,0)=cos(yaw);guess(0,1)=-sin(yaw);guess(1,0)=sin(yaw);guess(1,1)=cos(yaw);guess(0,3)=x;guess(1,3)=y;guess(2,3)=z;
  auto start=std::chrono::steady_clock::now();Cloud aligned;gicp.align(aligned,guess);auto T=gicp.getFinalTransformation();double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  results<<i<<' '<<int(gicp.hasConverged())<<' '<<gicp.getFitnessScore(.09)<<' '<<elapsed;
  for(int r=0;r<4;r++)for(int c=0;c<4;c++)results<<' '<<T(r,c);results<<'\n';results.flush();std::cout<<"seed "<<i++<<" converged "<<gicp.hasConverged()<<" s "<<elapsed<<std::endl;
 }
}
