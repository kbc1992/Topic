#ifndef SFLA_H_INCLUDED
#define SFLA_H_INCLUDED

#include "enviroment.h"
#include "Sensors_Optimal_Cover.h"
#include <vector>
#include "GA.h"

using namespace std;

#define Gene 8000  /*混合迭代次数*/
#define Popus 100 /*个体总数*/
#define Meme 10 /*子群数*/
#define Indis 10 /*因此，一个子群中的个体数是10*/
#define Num 10 /*子群内更新次数*/
#define GInd 5    /*子群组中的个体数*/
#define E1  461.08   /*无人机飞行能耗 j/s*/
#define E2  352.59  /*无人机悬停能耗 j/s*/
#define Emax 359640.0 /*电池最大能耗*/
const static short x_start = 0;
const static short y_start = 0;
const static short z_start = 0;
const double EPS = 1e-8;
const static double velocity = 10.0; //unit m/s;

static  POS center_all_regions;
static map<set<short>,POS> center_in_each_region;

typedef struct Individual
{
    vector<WAYPOINT> path;
    double fitness;
	double tour_length;
	double collect_time;
    Individual():fitness(0){path.clear();};
    Individual(vector<WAYPOINT> vec,double fit,double tour_length,double collect_time)
    {
        this->path = vec;
        this->fitness = fit;
		this->tour_length = tour_length;
		this->collect_time = collect_time;
    }
    ~Individual(){};
    Individual& operator=(const Individual &indi)
    {
        path = indi.path;
        fitness = indi.fitness;
		tour_length = indi.tour_length;
		collect_time = indi.collect_time;
        return *this;
    }
}Individual;

//获取解的适应度
double getFitness(Individual& solution,SENSOR_NODE s[]);
//随机生成一个可行解
Individual randGeneOneSolut(map<set<short>,vector<int> >& optimal_overlapping_regions, SENSOR_NODE s[],short h);
//按照最优解的遍历顺序生成一个可行解
Individual geneOneSolutByBestIndi(Individual& bestSolution,map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h);
//判断一个解是不是可行解
bool isFeasibleSolution(Individual& solution);
//初始化群落
void init(map<set<short>,vector<int> >& optimal_overlapping_regions,vector<Individual>& population,SENSOR_NODE s[],short h);
//降序排序
bool lessSort(Individual a,Individual b);
//给多个个体解降序排序
void sortIndividuals(vector<Individual>& population);
//将初始群落划分成若干子群落
void seperateIntoSubPopulation(vector<Individual>& population,vector<vector<Individual> >& subPopulations);
//将若干子群落合并为一个完整群落
void mergyIntoPopulation(vector<Individual>& population,vector<vector<Individual> >& subPopulations);
//从子群落中构建一个子群组
void constructGroup(vector<Individual>& subPopulation,vector<Individual>& subGroup);
//将子群组中更新的解转移到原来的子群落中
void reBuildSubPopulation(vector<Individual>& subPopulation,vector<Individual>& subGroup);
//更新每个解对应的采集点的遍历顺序
void traversalOrderUpdate(Individual& bestSolution,Individual& worstSolution,SENSOR_NODE s[]);
//将整形数转换为POS类型
void convertToPos(vector<int>& points,vector<POS>& pos_points);
//从一组最佳的采集点集合中找出一个中心
POS findBestDCP(vector<int>& points);
//找到一组最佳的采集点集合
POS findBestDCPSet(set<short>& overlapping_region,vector<int>& points,SENSOR_NODE s[]);
//找到监测区域中心位置
POS findCenterAllRegion(SENSOR_NODE s[],short h);
//更新性能较差的解所对应的采集点的相应位置
void positionUpdate(Individual& bestSolution,Individual& worstSolution,map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[]);
//随机置换性能较差的解所对应的采集点的相对访问顺序
void relativeOrderUpdate(Individual& worstSolution,SENSOR_NODE s[]);
//改进性能较差的解
Individual updateWosrtSolution(Individual& bestSolution,Individual& worstSolution,map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[]);
//混合蛙跳算法的框架
Individual shuffledFrogLeapingAlgo(map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h);

Individual my_algorithm(SENSOR_NODE s[]);
#endif // SFLA_H_INCLUDED
