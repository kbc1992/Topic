#ifndef SFLA_H_INCLUDED
#define SFLA_H_INCLUDED

#include "enviroment.h"
#include "Sensors_Optimal_Cover.h"
#include <vector>
#include "GA.h"

using namespace std;

#define Gene 8000  /*��ϵ�������*/
#define Popus 100 /*��������*/
#define Meme 10 /*��Ⱥ��*/
#define Indis 10 /*��ˣ�һ����Ⱥ�еĸ�������10*/
#define Num 10 /*��Ⱥ�ڸ��´���*/
#define GInd 5    /*��Ⱥ���еĸ�����*/
#define E1  461.08   /*���˻������ܺ� j/s*/
#define E2  352.59  /*���˻���ͣ�ܺ� j/s*/
#define Emax 359640.0 /*�������ܺ�*/
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

//��ȡ�����Ӧ��
double getFitness(Individual& solution,SENSOR_NODE s[]);
//�������һ�����н�
Individual randGeneOneSolut(map<set<short>,vector<int> >& optimal_overlapping_regions, SENSOR_NODE s[],short h);
//�������Ž�ı���˳������һ�����н�
Individual geneOneSolutByBestIndi(Individual& bestSolution,map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h);
//�ж�һ�����ǲ��ǿ��н�
bool isFeasibleSolution(Individual& solution);
//��ʼ��Ⱥ��
void init(map<set<short>,vector<int> >& optimal_overlapping_regions,vector<Individual>& population,SENSOR_NODE s[],short h);
//��������
bool lessSort(Individual a,Individual b);
//���������⽵������
void sortIndividuals(vector<Individual>& population);
//����ʼȺ�仮�ֳ�������Ⱥ��
void seperateIntoSubPopulation(vector<Individual>& population,vector<vector<Individual> >& subPopulations);
//��������Ⱥ��ϲ�Ϊһ������Ⱥ��
void mergyIntoPopulation(vector<Individual>& population,vector<vector<Individual> >& subPopulations);
//����Ⱥ���й���һ����Ⱥ��
void constructGroup(vector<Individual>& subPopulation,vector<Individual>& subGroup);
//����Ⱥ���и��µĽ�ת�Ƶ�ԭ������Ⱥ����
void reBuildSubPopulation(vector<Individual>& subPopulation,vector<Individual>& subGroup);
//����ÿ�����Ӧ�Ĳɼ���ı���˳��
void traversalOrderUpdate(Individual& bestSolution,Individual& worstSolution,SENSOR_NODE s[]);
//��������ת��ΪPOS����
void convertToPos(vector<int>& points,vector<POS>& pos_points);
//��һ����ѵĲɼ��㼯�����ҳ�һ������
POS findBestDCP(vector<int>& points);
//�ҵ�һ����ѵĲɼ��㼯��
POS findBestDCPSet(set<short>& overlapping_region,vector<int>& points,SENSOR_NODE s[]);
//�ҵ������������λ��
POS findCenterAllRegion(SENSOR_NODE s[],short h);
//�������ܽϲ�Ľ�����Ӧ�Ĳɼ������Ӧλ��
void positionUpdate(Individual& bestSolution,Individual& worstSolution,map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[]);
//����û����ܽϲ�Ľ�����Ӧ�Ĳɼ������Է���˳��
void relativeOrderUpdate(Individual& worstSolution,SENSOR_NODE s[]);
//�Ľ����ܽϲ�Ľ�
Individual updateWosrtSolution(Individual& bestSolution,Individual& worstSolution,map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[]);
//��������㷨�Ŀ��
Individual shuffledFrogLeapingAlgo(map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h);

Individual my_algorithm(SENSOR_NODE s[]);
#endif // SFLA_H_INCLUDED
