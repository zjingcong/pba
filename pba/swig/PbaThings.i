
%module PbaThings
%{
#include "TestThing.h"
#include "ParticlesThing.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "TestThing.h"
%include "ParticlesThing.h"
