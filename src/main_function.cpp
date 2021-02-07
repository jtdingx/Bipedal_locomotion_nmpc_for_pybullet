#include "RTControl/MpcRTControlClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include <fstream>   
#include <string>  
#include <cassert>
#include <vector>


using namespace Eigen;
using namespace std;

#define GET_ARRAY_LEN(array, len) { len = sizeof(array)/sizeof(array[0]);}

// int read_scanf(const string &filename, const int &cols, vector<double *> &_vector)
// {
// 	FILE *fp = fopen(filename.c_str(), "r");
// 	bool flag = true;
// 	int i = 0;
// 	if (!fp) 
// 	{ 
// 		cout << "File open error!\n"; 
// 		return 0; 
// 	}
//  
// 	while (flag)
// 	{
// 		double *rowArray = new double[cols]; //new一个double类型的动态数组
//  
// 		for (i = 0; i < cols; i++) //读取数据，存在_vector[cols]中
// 		{
// 			if (EOF == fscanf(fp,"%lf", &rowArray[i]))
// 			{ 
// 				flag = false; 
// 				break; 
// 			}
// 			//输出rowArray存入的数据
// 			//cout << rowArray[0] << " " << rowArray[1] << " " << rowArray[2] << " " << rowArray[3] << endl;
// 		}
// 		if (cols == i) //将txt文本文件中的一行数据存入rowArray中，并将rowArray存入vector中
// 			_vector.push_back(rowArray);
// 	}
// 	fclose(fp);
// 	return 1;
// }
// 
// 
// MatrixXd rea_save()
// {
// 
//     string file ="/home/jiatao/Dropbox/mpc_code_py_test/python_to_c.txt";
// 	//firstly, read the file in one column: read in arrow
// 	int columns = 1;
// 	vector<double *> output_vector;
//     MatrixXd feedback_data;
// 	if (!read_scanf(file, columns, output_vector))
// 	{
// 		return feedback_data;
// 	}
// 		
// // 	//output_vector output check：
// // 	int rows = output_vector.size();
// // 	for (int i = 0; i < rows; i++)
// // 	{
// // 		for (int j = 0; j < columns; j++) 
// // 		{ 
// // 			cout << output_vector[i][j] << " "; 
// // 		}
// // 		cout << endl;
// // 	}
// 	
// 	///// reset the dimension of the 2D vector
//     int data_num = output_vector.size();
//     
//     int rowNum = 1;
//     
//     int columnNum = round(data_num/rowNum);
// 	
//     feedback_data.setZero(rowNum,columnNum);
//     
// 	for (int i = 0; i < rowNum; i++)
// 	{
// 		for (int j = 0; j < columnNum; j++) 
// 		{   
//             int ind = i*columnNum+j;
//             feedback_data(i,j) = output_vector[ind][0];
// 		}
// 	}    
//     
//     return feedback_data;
//     
// }


void File_wlxx(MatrixXd CoMMM_ZMP_foot)
{
	std::string fileName1 = "NMPC_optimal_trajectory.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<CoMMM_ZMP_foot.rows(); i++){
           for(int j=0; j<CoMMM_ZMP_foot.cols(); j++){
                 outfile1 << (double) CoMMM_ZMP_foot(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();	
	
	
	
}



int main(int argc,char* argv[])
{
    
/*    for(int x = 0;x< argc;x++){
        cout<<argv[x]<<endl;
    } */ 
    
    
    Matrix<double,12,1> mpc_gait_result;
    
    /// for cpp test
/*     MpcRTControlClass mpc_planner;*/    
//     for (int i = 1; i < 4000; i++) {	
//       MpcRTControlClass mpc_planner; 
//       mpc_gait_result = mpc_planner.WalkingReactStepping(i);
//       cout<<mpc_gait_result.transpose();
//     }   



    ////////////////////////////////////////////////////////////////////
    ///=============================================================///
    
    // the first one denote the loop count 
    int n=atoi(argv[1]);
    MpcRTControlClass mpc_planner; 
    mpc_gait_result = mpc_planner.WalkingReactStepping(n);
    cout<<mpc_gait_result.transpose();   

     
    return 0;
}










