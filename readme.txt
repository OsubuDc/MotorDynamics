g++ -std=c++17 Motordynamics.cpp -o motor -I /usr/include/eigen3 -lyaml-cpp
-std=c++17 = use c++17 standard
-o motor = output executable
- I = include directory path to eigen3
when you do #include -> it tells compiler to also look in this folder
but that means you also need to declare where that folder is located

Run the program "motor"
./motor
