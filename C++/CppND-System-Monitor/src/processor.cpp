#include <vector>
#include <string>
#include "processor.h"
#include <iostream>
#include "linux_parser.h"
using std::string;
using std::vector;

// : Return the aggregate CPU utilization
float Processor::Utilization() { 
  long totalCPU = LinuxParser::Jiffies();
  long totalIdle = LinuxParser::IdleJiffies();


  long dTotal = totalCPU - this->preTotal;
  long dIdle = totalIdle - this->preIdle;
  this->preTotal = totalCPU;
  this->preIdle = totalIdle;

  return 1.0*(dTotal - dIdle)/dTotal*1.0; 
  //std::cout << "NOTHING";
  //return 1;
}
