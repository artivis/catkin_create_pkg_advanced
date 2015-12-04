#ifndef EXAMPLE_CPP_H
#define EXAMPLE_CPP_H

#include <vector>

namespace foo
{

  // Class to define the API
  class package_nameImpl
  {
  public:

    package_nameImpl() { }

    virtual ~package_nameImpl() { }

    virtual bool doSomething() = 0;

    virtual void doTheMaths() = 0;

    virtual double getStuff()
    {
      //return stuff_;
    }

    virtual double setStuff(/*type stuff*/)
    {
      //stuff_ = stuff;
    }

  protected:

    //type stuff_;
  };

  // Class to actually do stuff
  class package_name : public package_nameImpl
  {
   public:

    package_name();

    ~package_name();

    virtual bool doSomething();

    virtual void doTheMaths();
  };

} //namespace foo

#endif //EXAMPLE_CPP_H
