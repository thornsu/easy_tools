#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

// vin-mono 输出的轨迹是 timestamps x y z  qx qy qz qw 
// tum建议的轨迹格式是 timestamps x y z qw qx qy qz 
// 使用evo工具分析轨迹的时候要用tum的格式 稍微转化一下就行
int main(int argc, char **argv) {
  
    ifstream infile(argv[1]);
    string data_line;
    
    ofstream outfile(argv[2]);
    
    while(!infile.eof())
    {
      getline(infile,data_line);
      vector<string> data_array;
      stringstream sstr(data_line);
      string tmp;
      while(getline(sstr,tmp,','))
      {
	data_array.push_back(tmp);
	cout<<tmp<<" ";
      }
      
      cout<<endl;
      outfile << data_array[0]<<" "<<data_array[1]<<" "<<data_array[2]<<" "<<data_array[3]<<" "
		  <<data_array[7]<<" "<<data_array[4]<<" "<<data_array[5]<<" "<<data_array[6]<<endl;
      
    }
    
    infile.close();
    outfile.close();
    return 0;
}
