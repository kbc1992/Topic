#ifndef SENSORS_OPTIMAL_COVER_H_INCLUDED
#define SENSORS_OPTIMAL_COVER_H_INCLUDED

#include "enviroment.h"

using namespace std;


float getDataRate(int col_data_point,POS sensor_pos);
void getAveColTime(map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[],map<set<short>,float>& ave_col_times);
void findOptiOverlReg(map<set<short>,vector<int> >& overlapping_regions,map<set<short>,float>& ave_col_times,map<set<short>,vector<int> >& optimal_overlapping_regions);


#endif // SENSORS_OPTIMAL_COVER_H_INCLUDED
