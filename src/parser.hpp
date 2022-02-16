#ifndef PARSER_HPP
#define PARSER_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

void openFile(std::ifstream &file, std::string filePath);
std::vector<std::vector<int> > processFile(std::ifstream &file);
std::vector<std::vector<int> > parse(std::string filePath);

#endif
