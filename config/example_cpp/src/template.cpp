#include "package_name/package_name.h"

#include <iostream>

namespace foo
{

  package_name::package_name() :
    package_nameImpl() // Call base class default constructor
  {

  }

  package_name::~package_name()
  {

  }

  bool package_name::doSomething()
  {
    return true;
  }

  void package_name::doTheMaths()
  {

  }

} //namespace foo
