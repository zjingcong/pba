
%module PbaThings
%{
#include "BoidFlocking.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "BoidFlocking.h"
