#include <cstdio>
#include <cassert>
#include "../src/shifter_logic.h"
using namespace lsk;
int fails=0;
#define CHECK(c,msg) do{ if(!(c)){printf("FAIL: %s\n",msg); fails++;} }while(0)
uint8_t go(State&s, float th, float ph, Layout L){ return updateH(s,th,ph,L); }
int main(){
  // Standard layout: drive each gear from neutral
  struct {float ph; float th; uint8_t g;} cases[] = {
    {15.26f,14,1},{15.26f,-14,2},{0,14,3},{0,-14,4},{-15.26f,14,5},{-15.26f,-14,6},{-28.61f,14,7},{-28.61f,-14,8}};
  for (auto&c: cases){
    State s; 
    for(float p=0; fabsf(p)<fabsf(c.ph); p+= (c.ph>0?0.5f:-0.5f)) go(s,0,p,LAYOUT_STANDARD);
    go(s,0,c.ph,LAYOUT_STANDARD);
    uint8_t g=0; for(float t=0; fabsf(t)<=14; t+=(c.th>0?0.5f:-0.5f)) g=go(s,t,c.ph,LAYOUT_STANDARD);
    char m[64]; sprintf(m,"std gear expect %d got %d",c.g,g); CHECK(g==c.g,m);
    // back to neutral
    for(float t=c.th; fabsf(t)>0; t-=(c.th>0?0.5f:-0.5f)) g=go(s,t,c.ph,LAYOUT_STANDARD);
    CHECK(go(s,0,c.ph,LAYOUT_STANDARD)==0,"returns to neutral");
  }
  // Dogleg: gate0 front = R, rear = 1
  { State s; go(s,0,15.26f,LAYOUT_DOGLEG); uint8_t g=0; for(float t=0;t>=-14;t-=0.5f) g=go(s,t,15.26f,LAYOUT_DOGLEG); CHECK(g==1,"dogleg 1st"); }
  { State s; go(s,0,-28.61f,LAYOUT_DOGLEG); uint8_t g=0; for(float t=0;t<=14;t+=0.5f) g=go(s,t,-28.61f,LAYOUT_DOGLEG); CHECK(g==6,"dogleg 6th"); }
  // Hysteresis: φ wobbling near the 3-4 / 5-6 boundary (−4.1°) must not flip the gate
  { State s; for(int i=0;i<100;i++){ float p=-7.63f+((i%2)?0.5f:-0.5f); go(s,0,p,LAYOUT_STANDARD);} CHECK(s.gate==1,"hysteresis keeps gate"); }
  // Gate is locked while in gear: once in 3rd, φ noise must not change it
  { State s; go(s,0,0,LAYOUT_STANDARD); for(float t=0;t<=14;t+=0.5f) go(s,t,0,LAYOUT_STANDARD);
    uint8_t g=go(s,14,-12.0f,LAYOUT_STANDARD); CHECK(g==3,"gear held despite phi noise"); }
  // Synchro-hump region (5–7.9°) is not engaged yet
  { State s; uint8_t g=0; for(float t=0;t<=9.4f;t+=0.1f) g=go(s,t,0,LAYOUT_STANDARD); CHECK(g==0,"not engaged before 8.5 deg"); }
  // SEQ: single edge + latch
  { State s; int edges=0; for(float t=0;t<=7.9f;t+=0.1f) edges+= (updateSeq(s,t)!=0);
    for(float t=7.9f;t>=0;t-=0.1f) edges+= (updateSeq(s,t)!=0);
    for(float t=0;t>=-7.9f;t-=0.1f) edges+= (updateSeq(s,t)==-1);
    CHECK(edges==2,"seq edges"); }
  // rawToDeg wrap
  CHECK(fabsf(rawToDeg(10,4090,1)-(16*360.0f/4096))<1e-3,"wrap");
  CHECK(fabsf(rawToDeg(20,16380,1,16384)-(24*360.0f/16384))<1e-4,"wrap14");
  { State s; int e=0; for(float t=0;t<=6.9f;t+=0.1f) e+=(updateSeq(s,t)!=0); CHECK(e==0,"no seq edge before click"); }
  printf(fails? "%d FAILED\n":"ALL PASS\n",fails);
  return fails;
}
