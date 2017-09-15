
%module PbaThings
%{
#include "TriangleCollision.h"
#include "unitTest.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "TriangleCollision.h"
%include "unitTest.h"
