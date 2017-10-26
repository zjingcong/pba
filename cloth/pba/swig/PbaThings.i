
%module PbaThings
%{
#include "UnitTest.h"
#include "ClothThing.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "UnitTest.h"
%include "ClothThing.h"
