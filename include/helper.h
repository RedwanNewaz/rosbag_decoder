#ifndef HELPER_H
#define HELPER_H
#include <iostream>
#include <string>
#include <vector>
#include <glob.h>

// read all files inside a given folder
inline std::vector<std::string> glob(const std::string& pat){
    using namespace std;
    glob_t glob_result;
    glob(pat.c_str(),GLOB_TILDE,NULL,&glob_result);
    vector<string> ret;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        ret.push_back(string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return ret;
}
template<typename Out>
inline void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}
inline std::string get_basename(std::string filename){
  std::vector<std::string> elems,name_ext;
  split(filename, '/', std::back_inserter(elems));
  std::string fullname=elems[elems.size()-1];
  split(fullname, '.', std::back_inserter(name_ext));
  return name_ext[0];
}

inline void get_new_name(std::string &filename, std::string &nodename){
  std::vector<std::string> elems;
  split(filename, '/', std::back_inserter(elems));
  //get dir by omiting the last element of the elems
  std::string dir="";
  for(auto itr(elems.begin());itr!=elems.end()-1;itr++){
    dir+=*itr+"/";
  }
  //new name
  filename= dir+nodename+"_"+elems[elems.size()-1];
  std::cout<<filename<<std::endl;
}


#endif // HELPER_H
