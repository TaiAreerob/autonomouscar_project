#include <iostream>
#include <fstream>

/* Viewer for the pgm made with ros's map_server */
void exit_program(std::string str){
	std::cerr << str << '\n';
	exit(18);
}

int main(int argc, char *argv[]){
	
	if (argc != 2) exit_program("usage : exe <something.pgm>");
	std::ifstream in_pgm(argv[1]);	
	
	std::string line;
	int width, height;
	int max_val;

	std::getline(in_pgm, line); std::cout << line << '\n'; //should be P5
	std::getline(in_pgm, line); std::cout << line << '\n'; //should be comment
	in_pgm >> width >> height; std::cout << width << ' ' << height << '\n'; //should be width, height
	in_pgm >> max_val;  std::cout << max_val << '\n';
	
	unsigned char v;
	in_pgm.read((char *)&v, 1); //newline
	for(int i = 0 ; i < height; ++i){
		for(int j = 0 ; j < width; ++j){
			in_pgm.read((char *)&v, 1); printf("%d ", v);
		}
		std::cout << '\n'; 
		if (i > height/2) std::cin >> v;//for stop;
	}
}