#ifndef PROCESSOR_H
#define PROCESSOR_H

class Processor {
 public:
  float Utilization();  // : See src/processor.cpp

  // : Declare any necessary private members
 private:
  int preIdle{0}, preTotal{0};
};

#endif
