#include <dirent.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>

#include "linux_parser.h"

using std::stof;
using std::string;
using std::to_string;
using std::vector;

string LinuxParser::OperatingSystem() {
  string line;
  string key;
  string value;
  std::ifstream filestream(kOSPath);
  if (filestream.is_open()) {
    while (std::getline(filestream, line)) {
      std::replace(line.begin(), line.end(), ' ', '_');
      std::replace(line.begin(), line.end(), '=', ' ');
      std::replace(line.begin(), line.end(), '"', ' ');
      std::istringstream linestream(line);
      while (linestream >> key >> value) {
        if (key == "PRETTY_NAME") {
          std::replace(value.begin(), value.end(), '_', ' ');
          return value;
        }
      }
    }
  }
  return value;
}

string LinuxParser::Kernel() {
  string name, os, kernel;
  string line;
  std::ifstream stream(kProcDirectory + kVersionFilename);
  if (stream.is_open()) {
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> name >> os >> kernel;
  }
  return kernel;
}

// BONUS: Update this to use std::filesystem
vector<int> LinuxParser::Pids() {
  vector<int> pids;
  DIR* directory = opendir(kProcDirectory.c_str());
  struct dirent* file;
  while ((file = readdir(directory)) != nullptr) {
    // Is this a directory?
    if (file->d_type == DT_DIR) {
      // Is every character of the name a digit?
      string filename(file->d_name);
      if (std::all_of(filename.begin(), filename.end(), isdigit)) {
        int pid = stoi(filename);
        pids.push_back(pid);
      }
    }
  }
  closedir(directory);
  return pids;
}

// : Read and return the system memory utilization
float LinuxParser::MemoryUtilization() {
  float mem{0.0f};
  string line{""};
  std::ifstream memFile(kProcDirectory + kMeminfoFilename);

  if (memFile.is_open()) {
    vector<long> memValues(2);
    long mem_;
    int rows{0};

    // row 1 = memTotal, row 2 = memFree
    while (rows < 2) {
      std::string temp{""};
      std::getline(memFile, line);
      std::istringstream sstream(line);

      while (sstream >> temp) {
        if (std::stringstream(temp) >> mem_) {
          memValues[rows] = mem_;
          break;
        }
      }
      ++rows;
    }
    // for(auto it:memValues) std::cout << it << '\n';
    mem = (memValues[0] - memValues[1]) * 1.0 / memValues[0];
  } else {
    std::cout << "Cannot open file\n";
  }
  return mem;
}

// : Read and return the system uptime
long LinuxParser::UpTime() {
  long uptime{0};
  std::ifstream upTimeFile(kProcDirectory + kUptimeFilename);
  if (upTimeFile.is_open()) {
    std::string line{""};
    std::getline(upTimeFile, line);
    std::istringstream getter(line);
    std::stringstream(line) >> uptime;
  }

  return uptime;
}

// : Read and return the number of jiffies for the system
long LinuxParser::Jiffies() { 
  return LinuxParser::ActiveJiffies() + LinuxParser::IdleJiffies();
}

// : Read and return the number of active jiffies for a PID
// REMOVE: [[maybe_unused]] once you define the function
long LinuxParser::ActiveJiffies(int pid) { 
  std::ifstream statFile(kProcDirectory + std::to_string(pid) +
                         LinuxParser::kStatFilename);
  long cpu{0};
  if (statFile.is_open()) {
    string line{""};
    long temp{0};
    std::getline(statFile, line);
    std::istringstream iss(line);
    int count{0};
    //float val{0.0f};
    while (count < 22 && iss >> temp) {
      if ((count >= 13 && count <= 16) || count == 21) {
        // convert to seconds
        //val = std::stof(temp) / sysconf(_SC_CLK_TCK);
        cpu += temp;
      }
      ++count;
    }
    /*while(iss >> temp) {
      cpu.emplace_back(temp);
    }*/
  }
  //std::cout << "done 1";
  // std::cout << cpu.size() << '\n';
  return cpu; 
}

// : Read and return the number of active jiffies for the system
long LinuxParser::ActiveJiffies() { 
  vector<string> jiffiesString{LinuxParser::CpuUtilization()};
  vector<long> jiffiesLong;
  auto converted = std::back_inserter(jiffiesLong);
  for (auto &it : jiffiesString) {
    converted = stol(it);
  }
  long totalActive = jiffiesLong[kUser_] + jiffiesLong[kNice_] + jiffiesLong[kSystem_] + jiffiesLong[kIRQ_] + jiffiesLong[kSoftIRQ_];
  return totalActive;
}

// : Read and return the number of idle jiffies for the system
long LinuxParser::IdleJiffies() { 
  vector<string> jiffiesString{LinuxParser::CpuUtilization()};
  vector<long> jiffiesLong;
  auto converted = std::back_inserter(jiffiesLong);
  for (auto &it : jiffiesString) {
    converted = stol(it);
  }
  long totalIdle = jiffiesLong[LinuxParser::kIdle_] + jiffiesLong[kIOwait_];
  return totalIdle;
}

// : Read and return CPU utilization
vector<string> LinuxParser::CpuUtilization() {
  std::ifstream statFile(kProcDirectory + kStatFilename);
  if (statFile.is_open()) {
    vector<string> timeUnits;
    string temp{""}, user_{""}, nice_{""}, system_{""}, idle_{""}, iowait_{""},
        irq_{""}, softirq_{""}, steal_{""}, guest_{""}, guest_nice_{""};
    string line{""};
    while (std::getline(statFile, line)) {
      std::istringstream iss(line);
      if (iss >> temp && temp == "cpu") {
        iss >> user_ >> nice_ >> system_ >> idle_ >> iowait_ >> irq_ >>
            softirq_ >> steal_ >> guest_ >> guest_nice_;
        timeUnits.emplace_back(user_);
        timeUnits.emplace_back(nice_);
        timeUnits.emplace_back(system_);
        timeUnits.emplace_back(idle_);
        timeUnits.emplace_back(iowait_);
        timeUnits.emplace_back(irq_);
        timeUnits.emplace_back(softirq_);
        timeUnits.emplace_back(steal_);
        timeUnits.emplace_back(guest_);
        timeUnits.emplace_back(guest_nice_);
      } else {
        break;
      }
    }
    return timeUnits;
  }
  return {};
}

// : Read and return the total number of processes
int LinuxParser::TotalProcesses() {
  int total{0};
  std::ifstream statFile(kProcDirectory + kStatFilename);
  if (statFile.is_open()) {
    std::string temp{""};
    std::string line{""};
    while (std::getline(statFile, line)) {
      std::stringstream ss(line);
      if (ss >> temp && temp == "processes") {
        ss >> total;
        return total;
      }
    }
  }
  return 0;
}

// : Read and return the number of running processes
int LinuxParser::RunningProcesses() {
  int running{0};
  std::ifstream statFile(kProcDirectory + kStatFilename);
  if (statFile.is_open()) {
    std::string line{""};
    std::string temp{""};
    while (std::getline(statFile, line)) {
      std::stringstream ss(line);
      if (ss >> temp && temp == "procs_running") {
        ss >> running;
        return running;
        ;
      }
    }
  }
  return 0;
}

//: Read an return the CPU utilization of a process
vector<string> LinuxParser::ProcessCpu(int pid) {
  std::ifstream statFile(kProcDirectory + std::to_string(pid) +
                         LinuxParser::kStatFilename);
  vector<string> cpu;
  if (statFile.is_open()) {
    string line{""};
    string temp{""};
    std::getline(statFile, line);
    std::istringstream iss(line);
    int count{0};
    while (count < 22 && iss >> temp) {
      if ((count >= 13 && count <= 16) || count == 21) {
        cpu.emplace_back(temp);
      }
      ++count;
    }
  }
  //std::cout << "done 1";
  // std::cout << cpu.size() << '\n';
  return cpu; 
}
// : Read and return the command associated with a process
// REMOVE: [[maybe_unused]] once you define the function
string LinuxParser::Command(int pid) {
  string command{""};
  std::ifstream commandFile(kProcDirectory + std::to_string(pid) +
                            kCmdlineFilename);
  if (commandFile.is_open()) {
    std::getline(commandFile, command);
  }
  //std::cout << command;
  return command;
}

// : Read and return the memory used by a process
// REMOVE: [[maybe_unused]] once you define the function
string LinuxParser::Ram(int pid) {
  std::ifstream statusFile(kProcDirectory + std::to_string(pid) +
                           kStatusFilename);
  if (statusFile.is_open()) {
    string line{""}, temp{""};
    long mem{0};
    while (std::getline(statusFile, line)) {
      std::istringstream iss(line);
      if (iss >> temp && temp == "VmSize:") {
        iss >> mem;
        return std::to_string(mem/1000);
      }
    }
  }
  //std::cout << "done 2";
  return "";
}

// : Read and return the user ID associated with a process
// REMOVE: [[maybe_unused]] once you define the function
string LinuxParser::Uid(int pid) {
  std::ifstream statusFile(kProcDirectory + std::to_string(pid) +
                           kStatusFilename);
  if (statusFile.is_open()) {
    string line{""}, temp{""}, uid{""};
    while (std::getline(statusFile, line)) {
      std::replace(line.begin(), line.end(), ':', ' ');
      std::istringstream iss(line);
      if (iss >> temp && temp == "Uid") {
        iss >> uid;
        return uid;
      }
    }
  }
  //std::cout << "done 3";
  return string();
}

// : Read and return the user associated with a process
// REMOVE: [[maybe_unused]] once you define the function
string LinuxParser::User(int pid) {
  std::ifstream passwdFile(kPasswordPath);
  string uid = LinuxParser::Uid(pid);
  if (passwdFile.is_open()) {
    string line{""}, name, x, number;
    while (std::getline(passwdFile, line)) {
      std::replace(line.begin(), line.end(), ':', ' ');
      std::istringstream iss(line);
      while (iss >> name >> x >> number && number == Uid(pid)) {
        return name;
      }
    }
  }
  //std::cout << "done 4";
  return string();
}

// : Read and return the uptime of a process
// REMOVE: [[maybe_unused]] once you define the function
long LinuxParser::UpTime(int pid) {
  std::ifstream statFile(kProcDirectory + std::to_string(pid) + kStatFilename);
  if (statFile.is_open()) {
    string line{""};
    string numbers{""};
    vector<string> tokens;
    std::getline(statFile, line);
    std::istringstream iss(line);
    int count{0};

    while (iss >> numbers && count < 22) {
      tokens.emplace_back(numbers);
      ++count;
    }
  //std::cout << "done 5";
    return UpTime() - (stol(tokens[21]) / sysconf(_SC_CLK_TCK));
  }
  return 0;
}
