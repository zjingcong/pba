
%module PbaThings
%{
#include "RotatingCube.h"
#include "TriangleCollision.h"
#include "unitTest.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "RotatingCube.h"
%include "TriangleCollision.h"
%include "unitTest.h"
