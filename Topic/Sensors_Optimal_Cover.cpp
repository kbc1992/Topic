#include "Sensors_Optimal_Cover.h"
#include <iostream>
#include <algorithm>
#include <iterator>
using namespace std;
float getDataRate(int col_data_point,POS sensor_pos)
{
    POS point;
    int_to_point(col_data_point,point);
    double len = dist(point,sensor_pos);
    if(len<0)
        return 0.0;
    else if(len<20)
        return 250.0;
    else if(len<50)
        return 19.2;
    else if(len<120)
        return 9.6;
    else if(len<200)
        return 4.8;
    else
        return 0.0;
}


void getAveColTime(map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[],map<set<short>,float>& ave_col_times)
{
    map<set<short>,vector<int> >::iterator ite_over_reg = overlapping_regions.begin();
    while(ite_over_reg!=overlapping_regions.end())
    {
        set<short> sensor_nodes = ite_over_reg->first;
        vector<int> points = ite_over_reg->second;
        float sum_col_time = 0.0;
        vector<int>::iterator ite_points = points.begin();
        while(ite_points!=points.end())
        {
            float col_time = 0.0;
            set<short>::iterator ite_sensor_nodes = sensor_nodes.begin();
            while(ite_sensor_nodes!= sensor_nodes.end())
            {
                short i = *ite_sensor_nodes;
                float dateRate = getDataRate(*ite_points,s[i].pos);
                col_time = col_time + sensorData * 8/dateRate;
                ite_sensor_nodes++;
            }
            sum_col_time = sum_col_time + col_time;
            ite_points++;
        }
        float ave_col_time = sum_col_time/(points.size()*1.0);
        ave_col_times.insert(pair<set<short>,float>(sensor_nodes,ave_col_time));

        set<short>::iterator ite_sen_nodes;
        ite_sen_nodes = sensor_nodes.begin();
        while(ite_sen_nodes != sensor_nodes.end())
        {
            cout << *ite_sen_nodes << " " ;
            ite_sen_nodes++;
        }
        cout <<ave_col_time<< " ";
        cout << endl;

        ite_over_reg++;
    }
}

void findOptiOverlReg(map<set<short>,vector<int> >& overlapping_regions,map<set<short>,float>& ave_col_times,map<set<short>,vector<int> >& optimal_overlapping_regions)
{
    set<short> entile_sensor_nodes;
    set<set<short> > temp_overlapping_regions;
    set<set<short> > temp_optimal_overlapping_regions;
    map<set<short>,float>::iterator ite_ave_col_time=ave_col_times.begin();
    set<short> min_cost_reg;
    short min_sensors_to_collect,temp_sensors_to_collect;
    float min_col_time,temp_col_time;
    set<short> min_intersect_sensors;
    set<short> temp_intersect_sensors;
    set<set<short> >::iterator ite_temp_ovel_reg;

    for(short i=0;i<SN_NUM;i++)
    {
        entile_sensor_nodes.insert(i);
    }
    while(ite_ave_col_time!=ave_col_times.end())
    {
        temp_overlapping_regions.insert(ite_ave_col_time->first);
        ite_ave_col_time++;
    }
    while(!entile_sensor_nodes.empty()|| temp_overlapping_regions.empty())
    {
        ite_temp_ovel_reg=temp_overlapping_regions.begin();
        while(ite_temp_ovel_reg!=temp_overlapping_regions.end())
        {
            set_intersection(entile_sensor_nodes.begin(),entile_sensor_nodes.end(),(*ite_temp_ovel_reg).begin(),(*ite_temp_ovel_reg).end(),inserter(temp_intersect_sensors,temp_intersect_sensors.begin()));
            if(temp_intersect_sensors.size()==0)
                ite_temp_ovel_reg++;
            else
                break;
        }
        min_cost_reg = *ite_temp_ovel_reg;
        ite_temp_ovel_reg++;

        while(ite_temp_ovel_reg!=temp_overlapping_regions.end())
        {
            min_intersect_sensors.clear();
            temp_intersect_sensors.clear();
			float min = 0.0, temp = 0.0;
            set_intersection(entile_sensor_nodes.begin(),entile_sensor_nodes.end(),min_cost_reg.begin(),min_cost_reg.end(),inserter(min_intersect_sensors,min_intersect_sensors.begin()));
            set_intersection(entile_sensor_nodes.begin(),entile_sensor_nodes.end(),(*ite_temp_ovel_reg).begin(),(*ite_temp_ovel_reg).end(),inserter(temp_intersect_sensors,temp_intersect_sensors.begin()));
            
			min_sensors_to_collect = min_intersect_sensors.size() ;
			if (min_sensors_to_collect > 1)
			{
				min = min_sensors_to_collect * weight;//* (1 + SN_NUM / sensorData * 0.1);
			}
			else
				min = (float)min_sensors_to_collect;

            temp_sensors_to_collect = temp_intersect_sensors.size();
			if (temp_sensors_to_collect > 1)
			{
				temp = temp_sensors_to_collect * weight; //* (1 + SN_NUM / sensorData * 0.1);
			}
			else
				temp = (float)temp_sensors_to_collect;

            //cout<<min_sensors_to_collect<< " "<<temp_sensors_to_collect<<" "<<endl;
			//cout << min << " " << temp << endl;

            min_col_time = ave_col_times.find(min_cost_reg)->second;
            temp_col_time = ave_col_times.find(*ite_temp_ovel_reg)->second;

            if(temp_sensors_to_collect!=0)
            {

                if((temp_col_time/temp) < (min_col_time/min))
                {
                    min_cost_reg = *ite_temp_ovel_reg;
					//cout << temp_col_time / temp << endl;
                }
            }

            ite_temp_ovel_reg++;
        }
        temp_overlapping_regions.erase(min_cost_reg);
        temp_optimal_overlapping_regions.insert(min_cost_reg);

        set<short> new_entile_sensor_nodes;
        set_difference(entile_sensor_nodes.begin(),entile_sensor_nodes.end(),min_cost_reg.begin(),min_cost_reg.end(),inserter(new_entile_sensor_nodes,new_entile_sensor_nodes.begin()));
        entile_sensor_nodes = new_entile_sensor_nodes;


        cout<<entile_sensor_nodes.size()<<" ";
        
        set<short>::iterator ite_entile_sensor_nodes = entile_sensor_nodes.begin();
        while(ite_entile_sensor_nodes!=entile_sensor_nodes.end())
        {
            cout<<*ite_entile_sensor_nodes<< " ";
            ite_entile_sensor_nodes++;
        }
        cout<<endl;

        set<short>::iterator ite_min_cost_reg = min_cost_reg.begin();
        while(ite_min_cost_reg!=min_cost_reg.end())
        {
            cout<<*ite_min_cost_reg<< " ";
            ite_min_cost_reg++;
        }
        cout<<endl;
        cout<<endl;
        
    }


    set<set<short> >::iterator ite_temp_optimal_overlapping_regions = temp_optimal_overlapping_regions.begin();
    while(ite_temp_optimal_overlapping_regions!=temp_optimal_overlapping_regions.end())
    {
        optimal_overlapping_regions.insert(pair<set<short>,vector<int> >(*ite_temp_optimal_overlapping_regions,overlapping_regions.find(*ite_temp_optimal_overlapping_regions)->second));
        ite_temp_optimal_overlapping_regions++;
    }

}
