#include <cuda_runtime.h>
#include <cmath>
#include <cstdio>
struct Context{double*p=nullptr,*s=nullptr,*out=nullptr;int*ids=nullptr;size_t np=0,ns=0,no=0;int n=0,m=0;char error[256]={0};};
static int check(Context*c,cudaError_t e){if(e==cudaSuccess)return 0;snprintf(c->error,sizeof(c->error),"%s",cudaGetErrorString(e));return -1;}
static int reserve(Context*c,double**p,size_t*size,size_t n){if(*size>=n)return 0;cudaFree(*p);*p=nullptr;*size=0;if(check(c,cudaMalloc((void**)p,n*sizeof(double))))return -1;*size=n;return 0;}
__global__ void query(const double*p,const double*s,const int*ids,double*out,int n,int count,double limit){
 int i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=n)return;double best=INFINITY;
 for(int j=0;j<count;j++){
  const double*a=s+26*ids[j];
  if(limit>=0){double dx=p[3*i]-a[22],dy=p[3*i+1]-a[23],dz=p[3*i+2]-a[24],r=a[25]+limit+1e-10;if(dx*dx+dy*dy+dz*dz>r*r)continue;}
  double x=p[3*i]-a[3],y=p[3*i+1]-a[7],z=p[3*i+2]-a[11];
  double q[3]={x*a[0]+y*a[4]+z*a[8],x*a[1]+y*a[5]+z*a[9],x*a[2]+y*a[6]+z*a[10]};
  double d=0.;for(int t=0;t<3;t++){double e=fmax(fmax(a[16+t]-q[t],q[t]-a[19+t]),0.);d+=e*e;}best=fmin(best,d);
 }out[i]=sqrt(best);
}
extern "C" void* obb_create(){return new Context;}
extern "C" const char* obb_error(void*p){return ((Context*)p)->error;}
extern "C" void obb_destroy(void*p){auto*c=(Context*)p;cudaFree(c->p);cudaFree(c->s);cudaFree(c->out);cudaFree(c->ids);delete c;}
extern "C" int obb_bind(void*p,int n,int m,const double*points,const double*shapes){
 auto*c=(Context*)p;if(n<0||m<0||n>100000||m>128)return -2;c->n=n;c->m=m;if(!n||!m)return 0;
 if(reserve(c,&c->p,&c->np,3*n)||reserve(c,&c->s,&c->ns,26*m)||reserve(c,&c->out,&c->no,n))return -1;
 if(!c->ids&&check(c,cudaMalloc((void**)&c->ids,128*sizeof(int))))return -1;
 if(check(c,cudaMemcpy(c->p,points,3*n*sizeof(double),cudaMemcpyHostToDevice)))return -1;
 return check(c,cudaMemcpy(c->s,shapes,26*m*sizeof(double),cudaMemcpyHostToDevice));
}
extern "C" int obb_query(void*p,int count,const int*ids,double limit,double*out){
 auto*c=(Context*)p;if(count<0||count>128)return -2;for(int j=0;j<count;j++)if(ids[j]<0||ids[j]>=c->m)return -2;if(!c->n)return 0;
 if(check(c,cudaMemcpy(c->ids,ids,count*sizeof(int),cudaMemcpyHostToDevice)))return -1;
 query<<<(c->n+127)/128,128>>>(c->p,c->s,c->ids,c->out,c->n,count,limit);if(check(c,cudaGetLastError()))return -1;
 return check(c,cudaMemcpy(out,c->out,c->n*sizeof(double),cudaMemcpyDeviceToHost));
}
