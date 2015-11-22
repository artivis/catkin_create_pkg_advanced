#include "example_cpp/example_cpp.h"

#include <iostream>

namespace foo
{

  SomeClass::SomeClass() :
    SomeClassImpl(), // Call base class default constructor
    myDouble_(0.)
  {

  }

  SomeClass::~SomeClass()
  {

  }

  Point2d SomeClass::doSomething(const std::vector<float>& vector_in)
  {
    if (vector_in.size() == 0) return Point2d();

  }

  void SomeClass::doTheMaths()
  {

  }

} //namespace foo
