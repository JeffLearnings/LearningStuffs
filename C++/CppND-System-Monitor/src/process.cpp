#include <unistd.h>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>
#include<iostream>

#include "process.h"
#include"linux_parser.h"

using std::string;
using std::to_string;
using std::vector;

// : Return this process's ID
int Process::Pid() { return pid; }

// : Return this process's CPU utilization
float Process::CpuUtilization() {
  long cpuUptime{LinuxParser::UpTime()};
  float one = std::stof(this->processCpu[0]);
  float two = std::stof(this->processCpu[1]);
  float three = std::stof(this->processCpu[2]);
  float four = std::stof(this->processCpu[3]);
  float start = std::stof(this->processCpu[4]);
  float sum{0.0f};
  long live{1};

  sum = one + two + three + four;
  sum /= sysconf(_SC_CLK_TCK);
  live = cpuUptime - start/sysconf(_SC_CLK_TCK);
    //std::cout << this->cpuUtilization[4] << ' ';
  if(sum/live > 1) return 0;
  return sum/live;
}

// : Return the command that generated this process
string Process::Command() { 
  return command; 
}

// : Return this process's memory utilization
string Process::Ram() { return ram; }

// : Return the user (name) that generated this process
string Process::User() { 
    user.resize(5);
    return user;
}

// : Return the age of this process (in seconds)
long int Process::UpTime() { return uptime; }

// : Overload the "less than" comparison operator for Process objects
// REMOVE: [[maybe_unused]]
bool Process::operator<(Process& a) {
  return this->CpuUtilization() > a.CpuUtilization();
}
