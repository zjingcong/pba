
%module PbaHou
%{
#include "PbaHou.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;

%include "carrays.i"
%array_class(float, floatArray);
%array_class(double, doubleArray);

%include "PbaHou.h"
