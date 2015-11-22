#ifndef EXAMPLE_CPP_H
#define EXAMPLE_CPP_H

#include <vector>

namespace foo
{

  template<typename T>
  struct Holder
  {
    Holder() : x(0), y(0) { }

    T x;
    T y;
  };

  typedef Holder<double> Point2d;

  // Class to define the API
  class SomeClassImpl
  {
  public:

    SomeClassImpl() : offset_(0.) { }

    virtual ~SomeClassImpl() { }

    virtual Point2d doSomething(const std::vector<float>& vector_in) = 0;

    virtual void doTheMaths() = 0;

    virtual double getOffset()
    {
      return offset_;
    }

    virtual double setOffset(double offset)
    {
      offset_ = offset;
    }

  protected:

    double offset_;
  };

  // Class to actually do stuff
  class SomeClass : public SomeClassImpl
  {
   public:

    SomeClass();

    ~SomeClass();

    virtual Point2d doSomething(const std::vector<float>& vector_in);

    virtual void doTheMaths();

  protected:

    double myDouble_;
  };

} //namespace foo

#endif //EXAMPLE_CPP_H
