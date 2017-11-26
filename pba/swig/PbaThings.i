
%module PbaThings
%{
#include "SpheresThing.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "SpheresThing.h"
