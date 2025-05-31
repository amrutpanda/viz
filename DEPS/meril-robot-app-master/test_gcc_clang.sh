
red=`tput setaf 1`
reset=`tput sgr0`

rm -rf ./build
echo "${red}Compiling with clang...${reset}"
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++
mkdir ./build
cd ./build
cmake ..
make VERBOSE=1 -j8
cd ..

rm -rf ./build
echo "${red}Compiling with gcc...${reset}"
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
rm -rf ./buid
mkdir ./build
cd ./build
cmake ..
make VEREBOSE=1 -j8
cd ..

