
%module PbaThings
%{
#include "RotatingCube.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "RotatingCube.h"
