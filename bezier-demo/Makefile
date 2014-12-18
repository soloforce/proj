bezier-demo:	main.o bzinterpolating.o bzpoint.o canvas.o
	g++ main.o bzinterpolating.o bzpoint.o canvas.o -o bezier-demo -lopencv_core -lopencv_highgui -lopencv_imgproc
main.o: main.cpp
	g++ -c main.cpp
bzinterpolating.o: bzinterpolating.cpp bzinterpolating.hpp
	g++ -c bzinterpolating.cpp
bzpoint.o: bzpoint.cpp bzpoint.hpp
	g++ -c bzpoint.cpp
canvas.o: canvas.cpp canvas.hpp
	g++ -c canvas.cpp
clean:
	rm bezier-demo *.o

