

OFILES = base/PbaViewer.o \
         base/PbaHou.o \
         base/DynamicalState.o \
         base/PbaThing.o \
         base/Matrix.o \
         base/LinearAlgebra.o \
         base/Force.o \
         base/Solver.o \
         base/Tools.o \
         base/Triangle.o \
         base/Geometry.o \
         base/Collision.o \
         base/BBox.o \
         base/KdTree.o \
         base/Boid.o \
         base/RigidBodyState.o \
         base/RBD.o \
         base/RBDCollision.o \
         base/SoftBodyState.o \
         base/SBD.o \
         base/SphereState.o \
         base/SphereCollision.o \
         base/AdvectByVolume.o


THINGS = things/TestThing.h \
         things/ParticlesThing.h


ROOTDIR = .
LIB = $(ROOTDIR)/lib/libpba.a 


ifeq ("$(shell uname)", "Darwin")
  GLLDFLAGS     = -framework Foundation -framework GLUT -framework OpenGL -lm
else
  ifeq ("$(shell uname)", "Linux")
    GLLDFLAGS     = -lglut -lGL -lm -lGLU
  endif
endif



ifeq ("$(shell uname)", "Darwin")
  #CXX = clang-omp -Wall -g -O2 -fPIC $(DEFINES) -fopenmp -flat_namespace
  CXX = g++ -Wall -g -O2 -fPIC $(DEFINES) -flat_namespace
  OIIOMACH = macosx
  OIIOLIB = -L/usr/local/lib -lOpenImageIO -L/opt/local/lib/libomp -lomp 
  SWIGLIB = -bundle -undefined suppress /usr/lib/libc.dylib
  CHROMELIB =  -ldl 
  PYTHONINCLUDE =  `python2.7-config --cflags --ldflags`
  SWIGLD = $(CXX)
  SWIGEXEC = swig
  INSTALL_TOOL = sudo install_name_tool -change libOpenImageIO.1.5.dylib /usr/local/lib/libOpenImageIO.1.5.dylib swig/_pba.so
else
  ifeq ("$(shell uname)", "Linux")
  CXX = g++ -Wall -g -O2 -fPIC $(DEFINES) -fopenmp -std=c++11
  OIIOMACH = linux64
  OIIOLIB = -L${OIIO_LIB_LOCATION} -lOpenImageIO
  VDBLIB = -L/group/dpa/local/openvdb/lib -lHalf -ltbb -lopenvdb -Wl,--rpath=/group/dpa/local/openvdb/lib
  SWIGLIB = 
  CHROMELIB = -ldl
  PYTHONINCLUDE = -I/usr/include/python2.7 -I/usr/lib/python2.7/config
  SWIGLD = $(CXX) -shared
  SWIGEXEC = swig
  endif
 endif


INCLUDES = -I ./include/ -I ./things -I /usr/local/include/ -I /opt/local/include/ -I/usr/include/ -I/opt/local/include/libomp $(PYTHONINCLUDE) -I /group/dpa/local/openvdb/include




.C.o:
	$(CXX) -c $(INCLUDES) $< -o $@

base: $(OFILES)
	ar rv $(LIB) $?

stuff: $(OFILES) $(THINGS) swig/PbaThings.i
	make base
	make genthings

clean:
	rm -rf *.o base/*.o base/*~ include/*~ python/*~ things/*~ core $(LIB)  *~ *.dSYM swig/*.cxx swig/*~ swig/*.so swig/*.o swig/pba.py* swig/PbaViewer.py swig/PbaThings.py swig/*.pyc ./*.pyc base/*.C.swp

viewer:	swig/PbaViewer.i	$(OFILES)
	$(SWIGEXEC) -c++ -python -shadow -I./include/ swig/PbaViewer.i
	$(CXX) -c swig/PbaViewer_wrap.cxx  $(INCLUDES) $(VDBLIB) -o swig/PbaViewer_wrap.o
	$(SWIGLD) swig/PbaViewer_wrap.o $(SWIGLIB) -ldl -L./lib -lpba $(GLLDFLAGS) $(VDBLIB)  -o swig/_PbaViewer.so


hou:	swig/PbaHou.i	$(OFILES)
	$(SWIGEXEC) -c++ -python -shadow -I./include/ swig/PbaHou.i
	$(CXX) -c swig/PbaHou_wrap.cxx  $(INCLUDES) $(VDBLIB) -o swig/PbaHou_wrap.o
	$(SWIGLD) swig/PbaHou_wrap.o $(SWIGLIB) -ldl -L./lib -lpba $(GLLDFLAGS) $(VDBLIB)  -o swig/_PbaHou.so


genthings:	swig/PbaThings.i $(THINGS)
	$(SWIGEXEC) -c++ -python -shadow -I./include/ -I./things swig/PbaThings.i
	$(CXX) -c swig/PbaThings_wrap.cxx  $(INCLUDES) $(VDBLIB) -o swig/PbaThings_wrap.o
	$(SWIGLD)  -L./lib -lpba swig/PbaThings_wrap.o $(SWIGLIB) -lpba -ldl $(GLLDFLAGS) $(VDBLIB)   -o swig/_PbaThings.so
