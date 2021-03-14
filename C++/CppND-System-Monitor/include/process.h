#ifndef PROCESS_H
#define PROCESS_H

#include <string>
#include "linux_parser.h"
/*
Basic class for Process representation
It contains relevant attributes as shown below
*/
class Process {
 public:
  int Pid();                               // : See src/process.cpp
  std::string User();                      // : See src/process.cpp
  std::string Command();                   // : See src/process.cpp
  float CpuUtilization();            // : See src/process.cpp
  std::string Ram();                       // : See src/process.cpp
  long int UpTime();                       // : See src/process.cpp
  bool operator<(Process& a) ;  // : See src/process.cpp
  Process(int pid) : pid(pid) {
    user = LinuxParser::User(pid);
    command = LinuxParser::Command(pid);
    processCpu = LinuxParser::ProcessCpu(pid);
    ram = LinuxParser::Ram(pid);
    uptime = LinuxParser::UpTime(pid);
  }

  // : Declare any necessary private members
 private:
  int pid;
  std::string user;
  std::string command;
  std::vector<std::string> processCpu;
  std::string ram;
  long int uptime;
};

#endif
